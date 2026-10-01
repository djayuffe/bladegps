# Multi-GNSS Architecture and Implementation Contract

bladeGPS is being evolved from a GPS-only signal generator into a multi-GNSS simulator. This document separates implemented behavior from registered architecture so that command-line labels, RF output, and documentation cannot overstate capability.

## Current capability

Only `gps-l1ca` has a production waveform backend. The other profiles are intentionally visible through `bladegps -L`, but selecting one exits before downloading ephemeris or opening the bladeRF.

| Layer | GPS L1 C/A | Galileo E1 OS | BeiDou B1I | GLONASS L1OF |
| --- | --- | --- | --- | --- |
| Signal/RF profile | Implemented | Implemented | Implemented | Implemented |
| Passband validation | Implemented | Implemented | Implemented | Profile check implemented; per-satellite slot mixer pending |
| Broadcast ephemeris parser | RINEX 2 production path; typed RINEX 3/4 LNAV reader added | Typed RINEX 3/4 INAV/FNAV reader added | Typed RINEX 3/4 D1/D2 reader added | Typed RINEX 3/4 FDMA reader added |
| Orbit/clock model | Implemented | Broadcast Kepler/clock/relativity model added; channel integration pending | MEO/IGSO and GEO broadcast models added; channel integration pending | RK4 state-vector/J2/Earth-rotation model added; channel integration pending |
| Ranging-code generator | Implemented | All 50 official E1-B/C primary codes, memory-code decoder, and CBOC primitives implemented | B1I generator implemented for PRN 1-63 | L1OF generator implemented |
| Navigation message | GPS LNAV | I/NAV ephemeris word types 1-4 and service word type 5, RINEX-to-ICD quantization, nominal E1-B vertical-page assembly, CRC-24Q, ICD-oriented convolutional coding, 30x8 interleaving, sync insertion, SSP selection, and nominal word scheduling implemented; almanac/FEC2 words remain | Common D1/D2 300-bit subframe encoder, BCH/interleaving, preamble/FraID/SOW headers, and complete D1 ephemeris subframes 2/3 with RINEX scaling implemented; D1 subframe 1/almanac and D2 page payloads remain | Ranging/time-mark, 85/77 Hamming protection, and exact immediate strings 1-4 with sign-magnitude fields implemented; RINEX conversion, string 5, and almanac strings remain |
| Modulation/mixer | BPSK(1) | CBOC subcarrier primitive implemented; channel mixer pending | BPSK primitive pending channel integration | FDMA carrier mapping implemented; channel mixer pending |
| Hardware validation | Requires local shielded lab | Not implemented | Not implemented | Not implemented |

## Core separation

The simulator is divided into layers that must remain constellation-aware:

1. `gnss.c` owns immutable signal metadata: constellation, carrier plan, code rate, code length, minimum sample rate, occupied bandwidth, satellite limit, FDMA/CDMA behavior, and implementation status.
2. The CLI selects a profile and obtains safe RF defaults. User overrides are checked to ensure the entire signal fits inside the complex sampled passband.
3. A future navigation layer parses RINEX 3/4 mixed-system records into constellation-specific records. It must preserve each system's time scale rather than treating GST, BDT, GLONASS time, and GPS time as interchangeable.
4. A future propagation layer evaluates the correct Keplerian or state-vector model, clock model, group delay, health, validity interval, and Earth-rotation correction for each system.
5. A future signal backend generates the exact ranging code, secondary code, navigation coding, symbol timing, and modulation for its signal.
6. A future mixer combines only signals that fit the configured center frequency/sample rate and keeps carrier phase continuous across 100 ms producer blocks.

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
