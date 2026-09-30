# GPS L1 C/A Coverage

This document tracks bladeGPS coverage against the public GPS L1 C/A signal model described by IS-GPS-200. It is not a certification claim. Full compliance requires bit-level test vectors, RF measurement, receiver interoperability testing, and review against the current official interface specification.

## Implemented

- L1 carrier center frequency at 1575.42 MHz.
- C/A code length of 1023 chips.
- PRN generation for GPS PRN 1-32 using the G1/G2 delay table currently present in `codegen()`.
- 50 bps navigation-data timing through 20 C/A-code epochs per data bit.
- 30-bit navigation words and subframe buffers.
- Broadcast ephemeris parsing from RINEX navigation files.
- RINEX SV health and accuracy parsing.
- Unhealthy-satellite exclusion during visibility/channel allocation.
- URA index propagation into subframe 1 from RINEX SV accuracy.
- Navigation message generation from broadcast ephemeris.
- Satellite ECEF position, velocity, clock correction, range, range-rate, azimuth, and elevation calculations.
- GPS week-normalized receiver time and code-phase timing.
- Receiver motion from static LLH, ECEF CSV, NMEA GGA, or interactive keyboard motion.
- Dynamic channel allocation for healthy visible satellites above the configured elevation mask.
- SC16 I/Q synthesis at 2.6 Msps.
- Real-time streaming through bladeRF synchronous TX.
- Automatic daily GPS broadcast ephemeris download from NOAA/NGS CORS when `-e` is omitted.

## Recently Wired Gaps

- Interactive vertical motion keys are now active:
  - `e` moves up.
  - `q` moves down.
- Zero-duration runs are rejected so the GPS producer cannot fail to signal readiness.
- The GPS producer exits cleanly if TX fails while the FIFO is full.
- GPS-task early-exit cleanup now frees allocated motion and I/Q buffers.
- Standalone file-output cleanup frees the 1-bit/8-bit I/Q buffer.
- Satellite visibility now uses the same range/azimuth/elevation path as pseudorange generation.
- Channel allocation now respects its elevation-mask argument.
- Receiver time and code phase are normalized across GPS week boundaries.

## Known Non-Certified Areas

These areas are not currently claimed as fully certified:

- PRN coverage beyond GPS PRN 1-32.
- GPS modernized civil signals such as L1C, L2C, or L5.
- SBAS or non-GPS GNSS signal generation.
- Formal parity/word-level comparison against official IS-GPS-200 test vectors.
- RF spectral mask, absolute power, group delay, and modulation quality validation on physical hardware.
- Leap-second and UTC/GPS edge-case validation across historical navigation files.
- Formal URA/health bit-level validation against official navigation-message vectors.
- Robust external-process-free ephemeris download on Windows without `curl` and `gzip`.
- Automated receiver-acquisition/regression tests using hardware-in-the-loop.

## Practical Validation Checklist

Before claiming a release as RF-ready:

```sh
cc -O3 -Wall -I/opt/local/include -c bladegps.c -o /tmp/bladegps.o
cc -O3 -Wall -I/opt/local/include -c gpssim.c -o /tmp/gpssim.o
cc -O3 -Wall -I/opt/local/include -c getch.c -o /tmp/getch.o
cc /tmp/bladegps.o /tmp/gpssim.o /tmp/getch.o -lm -lpthread -L/opt/local/lib -lbladeRF -o /tmp/bladegps
clang --analyze -Xanalyzer -analyzer-output=text -I/opt/local/include bladegps.c gpssim.c getch.c
git diff --check
```

Then perform shielded RF validation with a GPS receiver or simulator test set:

- Verify acquisition for static-location scenarios.
- Verify pseudorange continuity during CSV, NMEA, and interactive motion.
- Verify navigation message decode.
- Verify no unintended RF leakage from the test setup.
- Verify TX gain and attenuation are appropriate before enabling any RF output.

## Reference Documents

- IS-GPS-200, current public GPS space segment/user segment interface specification.
- GPS SPS Performance Standard, current public GPS Standard Positioning Service performance document.
