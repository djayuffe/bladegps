# Changelog

## Unreleased

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
