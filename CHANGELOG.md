# Changelog

## Unreleased

- Regenerates non-GPS navigation-message cycles at every 30-second boundary,
  preventing stale Galileo TOW and BeiDou SOW fields from repeating indefinitely.
- Selects non-GPS ephemerides by full calendar week and epoch rather than wrapped
  seconds-of-week alone, eliminating same-SOW selection of stale weekly records.
- Replaces the fixed 4096-record mixed-RINEX buffer with validated two-pass dynamic
  allocation and reports precise non-GPS producer initialization/render failures.
- Reads BeiDou Klobuchar coefficients from RINEX 3 `BDSA`/`BDSB` and RINEX 4
  `ION C ... D1D2` records, then validates and encodes the ICD-scaled coefficients.
- Converts continuous RINEX Galileo weeks to the on-air GST week, fills I/NAV word
  5 health/DVS and BGD fields from RINEX, and uses word type 63 dummy content when
  optional service data is unavailable instead of broadcasting false typed zeros.
- Derives GLONASS string-5 day and four-year-cycle fields from the record epoch,
  marks unavailable almanac filler records non-operational, and rejects fractional
  or out-of-range FDMA slots before carrier assignment.
- Adds automatic non-GPS daily mixed-RINEX acquisition from three BKG/IGS
  broadcast products with atomic cache writes and manual-file fallback.
- Infers RINEX 3 navigation families for untagged mixed records from Galileo
  data-source bits, BeiDou GEO PRNs, and GLONASS system identity so downloaded
  RINEX 3 products reach the correct non-GPS backend instead of being rejected
  as generic legacy records.
- Aligns Galileo secondary-code and BeiDou D1 Neumann-Hoffman phases to the
  requested scenario epoch and lowers the worst-case Galileo per-channel level
  so a full 16-channel coherent CBOC sum cannot clip SC16 Q11 output.
- Fails non-GPS initialization with a specific diagnostic when the selected
  scenario epoch has no healthy, in-age record instead of silently streaming an
  all-zero RF scenario forever.

- Added the shared mixed-GNSS RF renderer with sample-continuous carrier, code,
  navigation-symbol and overlay-code phases, SC16 Q11 saturation, passband
  enforcement, and elevation-ranked healthy-satellite allocation.
- Added Galileo E1-B/E1-C equal-power CBOC composition using the ICD-defined
  opposite pilot sign, BeiDou D1 Neumann-Hoffman overlay support, GLONASS
  relative-code/meander/time-mark formatting, and per-channel FDMA offsets.
- Added split-block equivalence, invalid-state, passband, capacity-pressure, and
  constellation-specific RF-path regression tests.
- Added D1 and D2 BeiDou almanac-page assembly with exact 186-bit ICD field
  placement, two's-complement range checks, common BCH/interleaving, and strict
  format-specific subframe/page schedules.
- Added the ten D2 GEO basic-navigation pages with exact cross-page clock and
  ephemeris field splits, reserved-bit handling, three-second page epochs, and
  RINEX D2 conversion through the shared BeiDou quantizers.
- Added active-channel-bank reconciliation that preserves every live signal phase
  through mixed-constellation allocator updates and rejects duplicate identities.
- Added deterministic transmitter symbol-cycle scheduling for Galileo E1 I/NAV,
  BeiDou D1, BeiDou D2 GEO, and GLONASS GNAV, including valid FEC-protected
  reserved service subframes and regression coverage from mixed RINEX records.
- Added shared live-observation geometry with iterative transmit time, Earth-rotation
  correction, satellite clock/range-rate correction, local azimuth/elevation,
  Doppler, and pseudorange-derived code/carrier phase initialization.
- Connected Galileo E1, BeiDou B1I D1/D2, and GLONASS L1OF to the production
  motion/geometry/allocation/render/FIFO thread, including health and ephemeris-age
  filtering, FDMA slot selection, exact sample-rate timing, and error propagation.

- Corrected the Galileo convolutional encoder register orientation against all
  three official OS SIS ICD 2.2 SSP symbol vectors.
- Added nominal E1-B I/NAV vertical-page assembly with exact protected-field
  ordering, CRC-24Q, convolutional encoding, 30x8 interleaving, sync symbols,
  secondary synchronization patterns, and the nominal 30-second word schedule.
- Added range-checked Galileo I/NAV word types 1-5 and exact conversion of
  Galileo RINEX ephemeris/clock values into the ICD-scaled word types 1-4.
- Corrected typed Galileo RINEX parsing so its three-field Orbit-5 and one-field
  Orbit-7 lines no longer shift SISA, health, BGD, and transmission-time fields.
- Added GLONASS GNAV 85/77 Hamming check-bit generation from the ICD 5.1
  verification matrices, including total even parity.
- Added the common BeiDou D1/D2 300-bit subframe encoder, including the special
  first word, dual BCH codewords, bit interleaving, preamble, FraID, and split
  BDT seconds-of-week header.
- Added complete BeiDou D1 ephemeris subframes 2 and 3 plus strict RINEX-to-ICD
  field quantization, split-field placement, and BDT week rollover handling.
- Corrected BeiDou RINEX A23 Orbit-7 parsing to consume its two defined fields
  instead of manufacturing two trailing semantic values.
- Added GLONASS GNAV immediate strings 1-4 with exact ICD bit positions,
  sign-magnitude numeric fields, time/health/control fields, and Hamming-protected
  85-bit output strings.
- Added GLONASS RINEX A15 conversion for state vectors, clock/frequency terms,
  health/status flags, accuracy and age fields, including UTC(SU)+3 message time
  and the GLONASS four-year calendar-day index.
- Added GLONASS GNAV system-time string 5 and generic almanac string pairs
  6/7 through 14/15, covering all frame string layouts with range-checked
  sign-magnitude fields and Hamming protection.
- Added complete BeiDou D1 clock/service subframe 1 and RINEX conversion for
  BDT week/time, clock polynomial, AODC/AODE, health, URAI and both TGDs;
  ionosphere coefficients are explicit because they originate in RINEX headers.
- Added validated assembly of complete 15-string GLONASS GNAV frames from
  immediate, system-time, and five almanac records.

- Adds typed RINEX 3/4 navigation ingestion for GPS LNAV, Galileo INAV/FNAV, BeiDou D1/D2, and GLONASS FDMA records.
- Preserves blank RINEX fields and skips unrelated RINEX 4 STO/ION/EOP and unsupported ephemeris message blocks without losing record synchronization.
- Adds Galileo/BeiDou broadcast-Kepler propagation, including BeiDou GEO's ICD-defined tilted-frame transform and constellation clock/relativity models.
- Adds GLONASS FDMA state-vector propagation with RK4 integration, J2, Earth rotation, and broadcast luni-solar accelerations.
- Adds Galileo CRC-24Q, rate-1/2 constraint-length-7 convolutional coding with the inverted G2 branch, and block interleaving.
- Adds BeiDou BCH(15,11,1) systematic encoding and two-codeword bit interleaving.
- Adds ICD-derived BeiDou B1I ranging-code generation for all 63 published phase assignments.
- Adds the GLONASS L1OF 511-chip ranging code and standard FDMA slot-to-carrier mapping.
- Adds all 50 official Galileo E1-B and all 50 E1-C 4092-chip primary memory codes, hexadecimal decoding, and exact CBOC subcarrier primitives.
- Adds the Galileo E1-C 25-chip secondary code, BeiDou B1I Neumann-Hoffman code, and GLONASS 30-bit time mark.
- Adds fixed-vector checksums, balance checks, FDMA edge tests, and CBOC coefficient tests for the new signal primitives.
- Adds a constellation/signal registry for GPS L1 C/A, Galileo E1, BeiDou B1I, and GLONASS L1OF, with explicit implementation status and passband validation.
- Adds `-S`/`-L` signal-profile controls and runtime bladeRF device, frequency, sample-rate, bandwidth, gain, and elevation-mask options.
- Keeps unsupported constellation profiles fail-closed until their waveform, navigation-message, ephemeris, and validation backends are complete.
- Adds geodetic latitude/longitude/height motion CSV input through `-p`.
- Derives generation-block and FIFO sizes from the selected runtime sample rate.
- Rejects bladeRF sample-rate coercion so hardware and synthesized code/carrier timing cannot diverge.
- Adds the BKG IGS archive as a fallback for automatic legacy GPS broadcast-ephemeris downloads.
- Documents the multi-GNSS architecture, band-planning constraints, backend requirements, and acceptance gates.
- Selects the closest in-fit ephemeris independently for each satellite and refreshes it at navigation-frame boundaries.
- Replaces linear light-time extrapolation with an iterative transmit-time and exact Earth-rotation solution.
- Adds relativistic satellite-clock drift to range-rate modeling.
- Preserves Doppler continuity across IODE/IODC/TOE ephemeris handovers.
- Corrects 30-second navigation-frame alignment and bounds code phase across the full word buffer.
- Emits the complete requested number of 100 ms blocks.
- Transmits the exact final partial buffer instead of discarding the scenario tail.
- Fixes dynamic-motion LLH initialization and refreshes interactive local axes while moving.
- Replaces 864,000 small motion allocations with contiguous storage.
- Validates RINEX 2 headers, orbital parameters, motion numbers, and Gregorian century dates.
- Reads RINEX fit intervals and streams `.gz`/`.Z` navigation files safely on POSIX.
- Propagates generator failures to the process exit code and handles SIGINT/SIGTERM cleanly.
- Adds deterministic core-model regression tests through `make check`.

## v1.0.0

- Added automatic NOAA/NGS GPS broadcast ephemeris download when `-e` is omitted.
- Wired interactive up/down movement keys (`e`/`q`) that were defined but not handled.
- Added satellite health/accuracy parsing, health-aware channel allocation, and URA propagation.
- Normalized GPS receiver time and code-phase timing across week boundaries.
- Made channel allocation respect the elevation-mask argument and use the corrected visibility path.
- Hardened zero-duration validation to avoid initialization hangs.
- Hardened GPS producer shutdown when TX fails while FIFO space is unavailable.
- Added cleanup for early GPS-task exits and standalone 8-bit/1-bit I/Q buffers.
- Reuses cached `brdcDDD0.YYn` files in the working directory.
- Hardens download handling with temporary files, dependency checks, decompression validation, and partial-file cleanup.
- Improves RINEX parser bounds checks and malformed-file handling.
- Improves TX and GPS thread shutdown/error handling.
- Expands README documentation, safety notes, build instructions, and examples.
