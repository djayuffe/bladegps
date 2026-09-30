# Changelog

## v1.0.0

- Added automatic NOAA/NGS GPS broadcast ephemeris download when `-e` is omitted.
- Reuses cached `brdcDDD0.YYn` files in the working directory.
- Hardens download handling with temporary files, dependency checks, decompression validation, and partial-file cleanup.
- Improves RINEX parser bounds checks and malformed-file handling.
- Improves TX and GPS thread shutdown/error handling.
- Expands README documentation, safety notes, build instructions, and examples.
