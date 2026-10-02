# Multi-GNSS Architecture and Implementation Contract

bladeGPS provides integrated software paths for GPS L1 C/A, Galileo E1 OS,
BeiDou B1I, and GLONASS L1OF. This document records their implementation and
validation boundaries without implying receiver or RF-laboratory certification.
For the exhaustive layer-by-layer status, input compatibility, motion, hardware,
and validation matrices, see [SUPPORT_MATRIX.md](SUPPORT_MATRIX.md).

## Current capability

All four profiles have integrated waveform backends. GPS can auto-download its
legacy daily navigation file; the other profiles auto-download or accept `-e`
with mixed RINEX 3/4.

`-S mixed-open` activates one joint production loop for all four systems. It
uses a 50 Msps stream, 47.1 MHz minimum waveform span, 48 MHz analog filter,
constellation-native transmit time, and a
single elevation-ranked 16-channel bank. Unsupported hardware bandwidth fails
before transmission.

| Layer | GPS L1 C/A | Galileo E1 OS | BeiDou B1I | GLONASS L1OF |
| --- | --- | --- | --- | --- |
| Signal/RF profile | Implemented | Implemented | Implemented | Implemented |
| Passband validation | Implemented | Implemented | Implemented | Profile and per-satellite FDMA carrier checks implemented |
| Broadcast ephemeris parser | Shared typed GPS RINEX 2 and RINEX 3/4 LNAV | Typed RINEX 3/4 INAV/FNAV; E1 production consumes INAV only | Typed RINEX 3/4 D1/D2 | Typed RINEX 3/4 FDMA |
| Orbit/clock model | Implemented | Integrated Kepler/clock/relativity, iterative transmit time, Sagnac, range rate and Doppler | Integrated MEO/IGSO/GEO orbit/clock, iterative transmit time, Sagnac, range rate and Doppler | Integrated RK4 state-vector/J2/Earth-rotation, iterative transmit time, range rate and Doppler |
| Ranging-code generator | Implemented | All 50 official E1-B/C primary codes, memory-code decoder, and CBOC primitives implemented | B1I generator implemented for PRN 1-63 | L1OF generator implemented |
| Navigation message | GPS LNAV | I/NAV ephemeris words 1-4 and word 5 with RINEX health/BGD, correct GST week/TOW, CRC-24Q, convolutional coding, interleaving, sync/SSP, and exact circular E1-B page-part timing; unavailable optional service/almanac/FEC2/ISM slots use the 192-bit vertical ICD dummy page | D1 subframes 1-3, D2 GEO basic pages 1-10, RINEX 3/4 Klobuchar coefficients, BCH/interleaving, headers and rollover; library almanac builders are available, while service subframes not supplied by ephemeris/ION input are emitted as encoded reserved payloads | Immediate strings 1-4 and calendar-derived string 5 with live UTC(SU)+3 frame time, Hamming protection, relative/meander/time-mark symbols; almanac pairs absent from an FDMA ephemeris record are explicitly marked non-operational |
| Modulation/mixer | Production BPSK(1) | Production E1-B/E1-C CBOC with continuous code/data/pilot-secondary/carrier phase | Production BPSK with D1 NH overlay and D2 500 bit/s scheduling | Production relative-code/meander/time-mark formatting and per-slot continuous-phase FDMA |
| Hardware configuration/read-back | Implemented; shielded receiver validation required | Implemented; shielded receiver validation required | Implemented; shielded receiver validation required | Implemented; shielded receiver validation required |

## Core separation

The simulator is divided into layers that must remain constellation-aware:

1. `gnss.c` owns immutable signal metadata: constellation, carrier plan, code
   rate, code length, minimum sample rate, minimum waveform span, recommended
   analog filter, satellite limit, FDMA/CDMA behavior, and implementation status.
2. The CLI selects a profile and obtains safe RF defaults. User overrides are checked to ensure the entire signal fits inside the complex sampled passband.
3. The navigation layer parses GPS RINEX 2 and supported RINEX 3/4 mixed-system
   ephemeris dynamically, skips unrelated record families without
   desynchronizing, and reads BeiDou Klobuchar data from RINEX 3 headers or
   RINEX 4 ION records.
4. The propagation layer evaluates constellation-specific Keplerian, BeiDou GEO, or GLONASS state-vector models plus clocks, iterative transmit time, Earth rotation, range rate and Doppler.
5. Constellation navigation backends generate ranging codes and encoded navigation pages/strings. The runtime rebuilds each 30-second cycle at its constellation epoch and substitutes explicitly unavailable service content safely.
6. `gnss_rf.c` converts those streams to signal levels and combines only healthy, above-mask channels that fit the configured center frequency/sample rate. It preserves carrier, code, data, and overlay phase across arbitrary producer block boundaries.

All received component clocks use the same per-channel Doppler scale. The
carrier offset, primary-code clock, navigation-symbol clock, and secondary/NH
overlay clock therefore remain mutually coherent instead of accumulating
bit/code boundary error during receiver or satellite motion.

`gnss_schedule.c` assembles the transmitter-facing cycles: fifteen two-second
Galileo pages, five BeiDou D1 subframes, ten complete three-second BeiDou D2 GEO
frames, and fifteen GLONASS strings. Reserved service slots are emitted as valid
FEC-protected reserved payloads rather than unencoded zeros.

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
The terminal differential bit is retained per satellite and seeds the next
30-second frame, so relative coding does not restart at scheduler boundaries.

The standalone Galileo E1 profile preserves the ICD 24.552 MHz receiver
reference bandwidth as its minimum waveform span, requests a 28 MHz analog
filter, and uses a 36.828 Msps complex sample rate. This
provides 36 samples per 1.023 Mcps code chip—three samples for each of the
twelve CBOC subchips—while retaining digital and analog passband margin for
Doppler and device filter quantization. It
avoids aliasing the 6.138 MHz BOC(6,1) component into the old 4.092 Msps
approximation.

The allocator is constellation-neutral. It rejects invalid systems, unhealthy
satellites, elevations below the mask, and signals whose occupied bandwidth does
not fit the complex sampled passband. Survivors are selected deterministically in
descending elevation order. For GLONASS, the candidate carrier is the individual
FDMA slot frequency, not the 1602 MHz nominal center.

After selection, `gnss_rf_reconcile()` rebuilds the active channel bank while
preserving carrier, code, data, and overlay phases for unchanged signals. This
keeps mixed-constellation allocation stable when satellites rise, set, become
unhealthy, or cross the configured elevation mask.

The CLI can select either one constellation profile or `mixed-open`. Every
profile, including standalone GPS L1 C/A, uses this same producer, geometry,
SC16 normalization, buffering, and RF channel contract. In mixed mode, typed
GPS LNAV records use the same geometry and RF channel contract as
Galileo, BeiDou, and GLONASS, so elevation pressure, passband rejection, health,
and live phase continuity are resolved in one allocator rather than by merging
independent output files.

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

The GPS downloader tries two daily RINEX 2 GPS sources in order:

1. NOAA/NGS CORS: `https://geodesy.noaa.gov/corsdata/rinex/YYYY/DDD/brdcDDD0.YYn.gz`
2. BKG IGS archive: `https://igs.bkg.bund.de/root_ftp/IGS/BRDC/YYYY/DDD/brdcDDD0.YYn.gz`

For Galileo, BeiDou, and GLONASS, the downloader tries the BKG daily
`BRDC00IGS_R`, `BRDC00WRD_S`, and `BRDM00DLR_S` mixed RINEX products in that
order. The typed RINEX 3/4 parser then selects only records matching the chosen
signal profile.

## Validation gates

The registry's `implemented` flag means that an executable software waveform
path exists and passes the repository's automated checks. A profile must not be
described as receiver-validated or RF-certified until all of these pass:

- Parser tests using valid, truncated, malformed, unhealthy, expired, and week/day-boundary navigation records.
- Published or independently generated ranging-code and navigation-message vectors.
- Orbit, clock, pseudorange, Doppler, and rollover comparisons against an independent implementation.
- Sample-level tests for code phase, symbol transitions, carrier continuity, amplitude, occupied spectrum, and block-boundary continuity.
- Multi-satellite and multi-constellation allocator tests at passband edges.
- Receiver acquisition, navigation-data decoding, and position/velocity/time solution in a shielded or conducted RF setup.
- Sanitizer, static-analysis, and graceful-shutdown checks for the complete runtime path.

## Normative references

- GPS Interface Control Documents, including IS-GPS-200: <https://www.gps.gov/interface-control-documents-icds-interface-specifications-iss>
- Galileo Open Service Signal-in-Space Interface Control Document, current in-force edition: <https://www.gsc-europa.eu/electronic-library/programme-reference-documents/galileo-in-force/open-service>
- BeiDou Navigation Satellite System Signal In Space Interface Control Documents: <https://www.csno-tarc.cn/userSupport/document>
- GLONASS Interface Control Documents and official reference material: <https://glonass-svoevi.ru/documents.php>
- IGS RINEX 4.02 format: <https://files.igs.org/pub/data/format/rinex_4.02.pdf>
- IGS Multi-GNSS Experiment data and products: <https://igs.org/mgex/data-products/>

Implementations must be checked against the applicable official ICD revision at development time; this document is an architecture contract, not a substitute for those specifications.
