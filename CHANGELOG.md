# Changelog

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
