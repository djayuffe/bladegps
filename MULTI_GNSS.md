# Multi-GNSS Architecture and Implementation Contract

bladeGPS is being evolved from a GPS-only signal generator into a multi-GNSS simulator. This document separates implemented behavior from registered architecture so that command-line labels, RF output, and documentation cannot overstate capability.

## Current capability

Only `gps-l1ca` has a production waveform backend. The other profiles are intentionally visible through `bladegps -L`, but selecting one exits before downloading ephemeris or opening the bladeRF.

| Layer | GPS L1 C/A | Galileo E1 OS | BeiDou B1I | GLONASS L1OF |
| --- | --- | --- | --- | --- |
| Signal/RF profile | Implemented | Implemented | Implemented | Implemented |
| Passband validation | Implemented | Implemented | Implemented | Profile and per-satellite FDMA carrier checks implemented |
| Broadcast ephemeris parser | RINEX 2 production path; typed RINEX 3/4 LNAV reader added | Typed RINEX 3/4 INAV/FNAV reader added | Typed RINEX 3/4 D1/D2 reader added | Typed RINEX 3/4 FDMA reader added |
| Orbit/clock model | Implemented | Broadcast Kepler/clock/relativity model added; channel integration pending | MEO/IGSO and GEO broadcast models added; channel integration pending | RK4 state-vector/J2/Earth-rotation model added; channel integration pending |
| Ranging-code generator | Implemented | All 50 official E1-B/C primary codes, memory-code decoder, and CBOC primitives implemented | B1I generator implemented for PRN 1-63 | L1OF generator implemented |
| Navigation message | GPS LNAV | I/NAV ephemeris word types 1-4 and service word type 5, RINEX-to-ICD quantization, nominal E1-B vertical-page assembly, CRC-24Q, ICD-oriented convolutional coding, 30x8 interleaving, sync insertion, SSP selection, and nominal word scheduling implemented; almanac/FEC2 words remain | Complete D1 subframes 1-3, D2 GEO basic-navigation pages 1-10, and shared D1/D2 186-bit almanac pages are implemented with schedule validation, RINEX D1/D2 scaling, explicit ionosphere inputs, BCH/interleaving, preamble/FraID/SOW and rollover handling; D2 integrity, ionosphere-grid and time-offset service pages remain | Complete 15-string GNAV frames are assembled from RINEX-derived immediate strings 1-4, system-time string 5 and five almanac pairs with sign-magnitude fields, time mark and Hamming protection |
| Modulation/mixer | Production BPSK(1) | Sample-level E1-B/E1-C CBOC mixer implemented with continuous code/data/pilot-secondary/carrier phase; runtime scheduler pending | Sample-level BPSK mixer and 1 kchip/s D1 NH overlay implemented; D1/D2 runtime scheduler pending | Relative-code/meander/time-mark formatter and per-slot continuous-phase FDMA mixer implemented; runtime scheduler pending |
| Hardware validation | Requires local shielded lab | Not implemented | Not implemented | Not implemented |

## Core separation

The simulator is divided into layers that must remain constellation-aware:

1. `gnss.c` owns immutable signal metadata: constellation, carrier plan, code rate, code length, minimum sample rate, occupied bandwidth, satellite limit, FDMA/CDMA behavior, and implementation status.
2. The CLI selects a profile and obtains safe RF defaults. User overrides are checked to ensure the entire signal fits inside the complex sampled passband.
3. A future navigation layer parses RINEX 3/4 mixed-system records into constellation-specific records. It must preserve each system's time scale rather than treating GST, BDT, GLONASS time, and GPS time as interchangeable.
4. A future propagation layer evaluates the correct Keplerian or state-vector model, clock model, group delay, health, validity interval, and Earth-rotation correction for each system.
5. Constellation navigation backends generate ranging codes and encoded navigation pages/strings. The remaining scheduler work is responsible for choosing the correct page at each constellation epoch.
6. `gnss_rf.c` converts those streams to signal levels and combines only healthy, above-mask channels that fit the configured center frequency/sample rate. It preserves carrier, code, data, and overlay phase across arbitrary producer block boundaries.

## Shared RF channel contract

`gnss_rf_channel_t` uses `+1/-1` chips and symbols in transmission order. The
renderer supports ordinary BPSK channels with an optional independent overlay,
plus the Galileo OS E1 composite defined by OS SIS ICD 2.2 equation 12:
equal-power E1-B data and E1-C pilot components, in-phase/anti-phase CBOC
subcarriers, and the E1-C 25-chip secondary code. BeiDou D1 supplies its 50 bit/s
data as the primary symbol stream and its 1 kchip/s Neumann-Hoffman sequence as
the overlay; D2 omits the NH overlay and uses its 500 bit/s stream directly.
GLONASS GNAV strings are differentially encoded, modulo-2 combined with the
100 Hz auxiliary meander for 1.7 seconds, and followed by the 30-chip time mark.

The allocator is constellation-neutral. It rejects invalid systems, unhealthy
satellites, elevations below the mask, and signals whose occupied bandwidth does
not fit the complex sampled passband. Survivors are selected deterministically in
descending elevation order. For GLONASS, the candidate carrier is the individual
FDMA slot frequency, not the 1602 MHz nominal center.

After selection, `gnss_rf_reconcile()` rebuilds the active channel bank while
preserving carrier, code, data, and overlay phases for unchanged signals. This
keeps mixed-constellation allocation stable when satellites rise, set, become
unhealthy, or cross the configured elevation mask.

## Band planning

GPS L1 C/A and Galileo E1 share the 1575.42 MHz center region and can be combined by a sufficiently sampled L1/E1 mixer. BeiDou B1I is centered at 1561.098 MHz and normally requires a separate RF run or a wider complex sample rate. GLONASS L1OF is FDMA: each space vehicle uses a frequency slot around the 1602 MHz base, so allocation and passband checks must operate on slot frequency rather than one constellation-wide carrier.

The profile defaults are safe starting points, not proof that a particular SDR, antenna path, filter, or receiver can cover a requested combination.

## Required backends

### Galileo E1 Open Service

- Parse Galileo RINEX navigation data and GST timing.
- Implement E1-B data and E1-C pilot primary/secondary codes from the official tables.
- Implement I/NAV page construction, convolutional coding, interleaving, synchronization, and CRC.
- Implement the specified E1 OS modulation, including BOC/CBOC component weighting and subcarrier timing.
- Validate code chips, pages, spectrum, acquisition, pseudorange, Doppler, and data decoding against official vectors and an independent receiver.

### BeiDou B1I

- Parse BeiDou broadcast records and BDT timing without applying GPS assumptions.
- Implement B1I Gold-code assignment and phase selection for the supported PRNs.
- Implement D1/D2 navigation selection, BCH coding, interleaving, frame timing, and satellite-type behavior.
- Apply BeiDou-specific orbit/clock/group-delay fields and health/validity rules.
- Validate chips, navigation words, spectrum, and receiver solution against official vectors.

### GLONASS L1OF

- Parse GLONASS state-vector records and time/frequency-channel fields.
- Propagate broadcast position, velocity, and acceleration using the specified numerical model.
- Generate the shared L1OF ranging code, meander sequence, time mark, and GNAV strings.
- Allocate FDMA frequency slots and maintain independent carrier phase/Doppler per satellite.
- Validate every supported slot frequency, navigation string, spectrum, and receiver solution.

## Ephemeris acquisition

The implemented GPS downloader tries two daily RINEX 2 GPS sources in order:

1. NOAA/NGS CORS: `https://geodesy.noaa.gov/corsdata/rinex/YYYY/DDD/brdcDDD0.YYn.gz`
2. BKG IGS archive: `https://igs.bkg.bund.de/root_ftp/IGS/BRDC/YYYY/DDD/brdcDDD0.YYn.gz`

True multi-GNSS automatic acquisition requires a RINEX 3/4 mixed-navigation parser first. Suitable archive products include BKG/IGS `BRDC` mixed-system files. The downloader must not fetch those files and then pass them to the GPS-only RINEX 2 parser.

## Acceptance gates

A profile may be marked `implemented` only after all of these pass:

- Parser tests using valid, truncated, malformed, unhealthy, expired, and week/day-boundary navigation records.
- Published or independently generated ranging-code and navigation-message vectors.
- Orbit, clock, pseudorange, Doppler, and rollover comparisons against an independent implementation.
- Sample-level tests for code phase, symbol transitions, carrier continuity, amplitude, occupied spectrum, and block-boundary continuity.
- Multi-satellite and multi-constellation allocator tests at passband edges.
- Receiver acquisition, navigation-data decoding, and position/velocity/time solution in a shielded or conducted RF setup.
- Sanitizer, static-analysis, and graceful-shutdown checks for the complete runtime path.

## Normative references

- Galileo Open Service Signal-in-Space Interface Control Document, current in-force edition: <https://www.gsc-europa.eu/electronic-library/programme-reference-documents/galileo-in-force/open-service>
- BeiDou Navigation Satellite System Signal In Space Interface Control Documents: <https://www.csno-tarc.cn/userSupport/document>
- IGS RINEX 4.02 format: <https://files.igs.org/pub/data/format/rinex_4.02.pdf>
- IGS Multi-GNSS Experiment data and products: <https://igs.org/mgex/data-products/>

Implementations must be checked against the applicable official ICD revision at development time; this document is an architecture contract, not a substitute for those specifications.
