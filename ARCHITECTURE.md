# bladeGPS Architecture

This document describes how bladeGPS is organized, how data moves through the simulator, and where to change specific behavior. It is intended for maintainers who need to audit, extend, port, or debug the project.

bladeGPS is a real-time multi-GNSS simulator framework for bladeRF with integrated software backends for GPS L1 C/A, Galileo E1, BeiDou B1I, and GLONASS L1OF. Hardware receiver certification is outside the automated test boundary.

## Source Layout

| File | Responsibility |
| --- | --- |
| `bladegps.c` | Command-line entry point for real-time bladeRF operation, automatic ephemeris download, FIFO utilities, TX thread, bladeRF setup, cleanup, and process lifecycle. |
| `bladegps.h` | bladeGPS-specific constants, thread/FIFO state, simulator option state, bladeRF TX state, and cross-module declarations. |
| `gnss.c` / `gnss.h` | Constellation and signal registry, RF/code profiles, profile parsing, capability status, and passband validation. |
| `gnss_codes.c` / `gnss_codes.h` | ICD-derived BeiDou B1I and GLONASS L1OF ranging codes, GLONASS FDMA carrier mapping, and Galileo E1 memory-code decoding/CBOC primitives. |
| `galileo_e1_codes.c` | All 50 E1-B and 50 E1-C primary memory codes from the Galileo OS SIS ICD v2.2 electronic Annex C. |
| `gnss_nav.c` / `gnss_nav.h` | Dynamically sized typed mixed RINEX 3/4 navigation records for GPS LNAV, Galileo INAV/FNAV, BeiDou D1/D2, and GLONASS FDMA, plus BeiDou Klobuchar metadata. |
| `gnss_orbit.c` / `gnss_orbit.h` | Constellation-aware Keplerian, BeiDou GEO, clock/relativity, and GLONASS numerical propagation models. |
| `gnss_geometry.c` / `gnss_geometry.h` | Iterative transmit-time observations, Earth-rotation correction, azimuth/elevation, satellite clock correction, range rate, Doppler, and pseudorange-derived initial code/carrier phases. |
| `gnss_fec.c` / `gnss_fec.h` | Galileo CRC-24Q/convolutional/interleaving primitives and BeiDou BCH/interleaving primitives. |
| `gnss_galileo_nav.c` / `gnss_galileo_nav.h` | Galileo nominal E1-B I/NAV vertical-page construction, CRC coverage, FEC/interleaving, sync/SSP insertion, and 30-second word schedule. |
| `gnss_beidou_nav.c` / `gnss_beidou_nav.h` | BeiDou D1/D2 word coding and interleaving, D1 clock/ephemeris subframes 1-3, D2 GEO basic-navigation pages 1-10, shared almanac payloads, format-specific schedules, and RINEX-to-ICD scaling. |
| `gnss_glonass_nav.c` / `gnss_glonass_nav.h` | GLONASS RINEX A15 conversion and GNAV immediate strings 1-4, including sign-magnitude fields, UTC(SU)+3 timing, four-year day index, and Hamming-protected 85-bit strings. |
| `gnss_rf.c` / `gnss_rf.h` | Shared mixed-constellation channel validation, elevation-ranked allocation, continuous carrier/code/data/overlay phase, SC16 Q11 mixing, Galileo E1 CBOC, BPSK overlay modulation, and GLONASS relative/meander symbol formation. |
| `gnss_schedule.c` / `gnss_schedule.h` | Time-ordered Galileo 30-second I/NAV, BeiDou D1 frame, BeiDou D2 ten-frame GEO, and GLONASS 15-string symbol-cycle assembly for the RF renderer. |
| `gnss_task.c` / `gnss_task.h` | Non-GPS production thread: mixed-RINEX selection, motion, health/age filtering, geometry, FDMA assignment, channel reconciliation, I/Q rendering, FIFO flow, and shutdown. |
| `gpssim.c` | GPS signal model: ephemeris parsing, satellite geometry, navigation message generation, channel allocation, motion parsing, I/Q synthesis, and GPS producer thread. |
| `gpssim.h` | GPS constants and data structures: times, ephemeris records, pseudorange records, and channel state. |
| `getch.c` / `getch.h` | POSIX keyboard helpers used by interactive mode. Windows uses `conio.h`. |
| `getopt.c` / `getopt.h` | Windows-compatible `getopt` implementation retained for portability. |
| `Makefile` | Builds `bladegps`, discovering libbladeRF through `pkg-config` when available. |
| `Readme.md` | User-facing install, usage, safety, and examples. |
| `CHANGELOG.md` | Release history. |
| `brdc*.??n`, `*.csv` | Sample ephemeris and motion/input data. |

## Runtime Overview

The real-time executable is built with `BLADE_GPS` enabled through `gpssim.h`. In that mode, `gpssim.c` exposes `gps_task()` instead of the standalone file-output simulator main.

```text
CLI options
   |
   v
bladegps.c main()
   |
   |-- optional ephemeris auto-download
   |-- bladeRF open/configure
   |-- allocate TX buffer and FIFO
   |
   +--> selected producer: gps_task() or gnss_task()
   |       |
   |       |-- read motion and constellation navigation records
   |       |-- compute observations and allocate healthy visible satellites
   |       |-- schedule navigation data and synthesize 0.1 s I/Q blocks
   |       +-- write blocks into FIFO
   |
   +--> TX consumer thread: tx_task()
           |
           |-- read I/Q samples from FIFO
           +-- stream SC16_Q11 buffers through bladerf_sync_tx()
```

The FIFO decouples synthesis from hardware transmission. The producer generates `tx_sample_rate / 10` samples per 100 ms block, while the TX thread consumes `SAMPLES_PER_BUFFER` samples per libbladeRF transfer.

The shared `gnss_rf` layer is independent of the constellation producers.
It can combine enabled channels from different constellations when every occupied
band fits the configured complex passband. Each channel retains carrier, ranging
code, data-symbol, and overlay-code phase between calls, so changing producer
block size does not introduce discontinuities. Its allocator filters unhealthy,
below-mask, malformed, and out-of-band candidates before retaining the highest
elevation signals. `gnss_task()` supplies validated observations, time-ordered
navigation cycles, motion, and FIFO production for the three non-GPS profiles;
`gps_task()` retains the mature GPS L1 C/A path.

Allocator updates use `gnss_rf_reconcile()`. A surviving signal is identified by
constellation, PRN, carrier, modulation, and timing configuration; all four live
phases are copied into the replacement channel bank. New signals use their
caller-supplied initial phases, removed signals are disabled, duplicate identities
are rejected, and reallocating a bank therefore cannot silently restart an
unchanged carrier or spreading code.

## Command-Line Lifecycle

`main()` in `bladegps.c` performs the real-time program lifecycle:

1. Parse CLI options.
2. If `-e` is absent, determine the ephemeris date from `-t` or current UTC date.
3. Download/cache the broadcast ephemeris if needed.
4. Validate required motion/static-location settings.
5. Initialize shared simulator state with `init_sim()`.
6. Allocate the bladeRF transfer buffer and internal FIFO.
7. Open and configure the bladeRF device.
8. Optionally configure XB200.
9. Start the selected GPS or non-GPS producer thread.
10. Wait for producer initialization.
11. Configure and enable the bladeRF synchronous TX interface.
12. Start the TX consumer thread.
13. Join TX, disable TX, join GPS, free resources, and close the device.

The program returns non-zero if setup fails, GPS generation cannot initialize, or bladeRF TX streaming reports an error.

## Ephemeris Download Flow

The downloader is in `download_broadcast_ephemeris()` in `bladegps.c`.

When `-e` is omitted:

1. The date comes from `-t YYYY/MM/DD,hh:mm:ss` if supplied.
2. Otherwise, `utc_today()` uses the current UTC calendar date.
3. `day_of_year()` maps the date to RINEX day-of-year.
4. GPS uses `brdcDDD0.YYn`; non-GPS profiles use the long-name mixed RINEX cache `BRDC00IGS_R_YYYYDDD0000_01D_MN.rnx`.
5. Existing local files are reused.
6. GPS files come from NOAA/NGS CORS with BKG as fallback. Non-GPS files try three daily BKG mixed products:

```text
https://geodesy.noaa.gov/corsdata/rinex/YYYY/DDD/brdcDDD0.YYn.gz
https://igs.bkg.bund.de/root_ftp/IGS/BRDC/YYYY/DDD/brdcDDD0.YYn.gz
https://igs.bkg.bund.de/root_ftp/IGS/BRDC/YYYY/DDD/BRDC00IGS_R_YYYYDDD0000_01D_MN.rnx.gz
https://igs.bkg.bund.de/root_ftp/IGS/BRDC/YYYY/DDD/BRDC00WRD_S_YYYYDDD0000_01D_MN.rnx.gz
https://igs.bkg.bund.de/root_ftp/IGS/BRDC/YYYY/DDD/BRDM00DLR_S_YYYYDDD0000_01D_MN.rnx.gz
```

The downloader requires `curl` and `gzip`. It writes compressed and decompressed data to `.tmp` files first, verifies that those files exist, then renames them into place. On failure, temporary files are removed and the user is told to provide `-e <nav_file>` manually.

Auto-downloaded files are ignored by git through `.gitignore` patterns, while existing tracked sample files remain tracked.

## Shared State

The central runtime state is `sim_t` in `bladegps.h`.

| Field | Purpose |
| --- | --- |
| `opt` | Parsed simulator options: signal profile, bladeRF device/RF settings, elevation mask, nav file, motion file, duration, start time, motion mode, and interactive mode. |
| `tx` | bladeRF TX state: device handle, transfer buffer, TX thread, TX error flag. |
| `gps` | GPS generation thread state and initialization condition variable. |
| `finished` | Shared shutdown signal. |
| `fifo` | Interleaved SC16 I/Q sample FIFO. |
| `head`, `tail` | FIFO write/read sample positions. |
| `fifo_read_ready` | Signals the TX thread that samples are available. |
| `fifo_write_ready` | Signals the GPS thread that FIFO space is available. |

FIFO state is protected with `gps.lock`. The older `tx.lock` remains in the struct but current FIFO coordination uses the GPS mutex consistently.

## Thread Model

### GPS Producer

`gps_task()` in `gpssim.c` owns signal synthesis.

Main responsibilities:

- Parse receiver position input.
- Parse RINEX broadcast ephemeris.
- Select a valid ephemeris set for the scenario start time.
- Allocate satellites above the elevation mask.
- Generate navigation messages.
- Compute pseudorange/rate and code/carrier phase.
- Fill the I/Q buffer for each 0.1 second step.
- Write the generated block into the shared FIFO.

If initialization fails, the thread sets `finished`, marks `gps.ready`, and broadcasts condition variables so `main()` and `tx_task()` do not deadlock.

### TX Consumer

`tx_task()` in `bladegps.c` owns hardware streaming.

Main responsibilities:

- Wait until FIFO data is available or generation is finished.
- Copy enough samples into `tx.buffer`.
- Call `bladerf_sync_tx()` with `SAMPLES_PER_BUFFER`.
- Submit the exact final partial buffer instead of dropping the tail of a scenario.
- On TX error, set `tx.error`, set `finished`, broadcast FIFO condition variables, and exit.

`main()` joins the TX thread first during normal completion, then disables TX and joins the GPS thread.

## FIFO Design

The generation block and FIFO sizes are runtime values derived from the selected sample rate:

| Constant | Value | Meaning |
| --- | --- | --- |
| `DEFAULT_TX_SAMPLERATE` | `2600000` | GPS default of 2.6 Msps. |
| `sim_t.iq_block_samples` | `tx_sample_rate / 10` | Samples per 0.1 second generation block. |
| `sim_t.fifo_length` | `iq_block_samples * 2` | FIFO capacity in complex samples. |
| `SAMPLES_PER_BUFFER` | `32 * 1024` | Samples per bladeRF sync TX transfer. |

Each sample is interleaved I/Q as two `int16_t` values.

`get_sample_length()` computes available complex samples. `fifo_read()` copies from the ring buffer and handles wraparound. The producer writes complete `iq_block_samples` blocks and advances `head`.

## GPS Signal Pipeline

Most signal logic is in `gpssim.c`.

### Ephemeris

`readRinexNavAll()` parses RINEX broadcast navigation records into `ephem_t eph[EPHEM_ARRAY_SIZE][MAX_SAT]`. It:

- Validates and skips the RINEX 2 header.
- Groups ephemerides into time sets.
- Validates line length before fixed-column access.
- Bounds-checks PRN before indexing `eph`.
- Rejects non-finite or physically invalid orbital records.
- Parses SV health and accuracy fields.
- Converts RINEX `D` exponent designators to `E`.
- Streams `.gz` and legacy `.Z` input through a shell-free POSIX `gzip` child process.
- Reads each record's fit interval, defaulting to four hours when omitted.
- Precomputes orbital working values such as semi-major axis and mean motion.

`selectEphemerides()` chooses the nearest in-fit record independently for every
PRN. Selection is refreshed on each 30-second navigation-message boundary.
When IODE, IODC, or TOE changes, channel subframes are rebuilt and the previous
range is reevaluated with the new orbit to prevent a false Doppler step.

### Receiver Motion

Supported modes:

- Static LLH from `-l`.
- CSV ECEF motion from `-u`.
- CSV geodetic motion from `-p`.
- NMEA GGA stream from `-g`.
- Keyboard interactive motion from `-i`: `w/s/a/d` for horizontal movement and `e/q` for up/down.

Static LLH is converted through `llh2xyz()`. NMEA GGA is parsed into LLH and then ECEF. CSV motion expects:

```text
time_seconds,ecef_x_m,ecef_y_m,ecef_z_m
```

The row-pointer table and ECEF samples use one contiguous allocation, avoiding
hundreds of thousands of small allocations in a 24-hour scenario. Dynamic ECEF
input is converted back to LLH for status output and interactive local-axis
motion. The local tangent frame is refreshed while the receiver moves.

### Satellite Geometry

Important functions:

- `satpos()` computes satellite position, velocity, clock bias, relativistic correction, TGD, and clock drift with week-aware time differences.
- `computeRange()` iterates signal transmit time, applies exact Earth-rotation correction, and computes geometric distance, pseudorange, range rate, azimuth, and elevation.
- `checkSatVisibility()` rejects unhealthy satellites and applies the elevation mask using the same Sagnac-aware range path as pseudorange generation.
- `allocateChannel()` assigns healthy visible satellites to simulated channels.

### Navigation Message

Important functions:

- `codegen()` generates the C/A code for PRN 1-32.
- `eph2sbf()` maps ephemeris to GPS subframe words.
- `generateNavMsg()` prepares channel navigation message buffers.

### I/Q Synthesis

For each 0.1 second step:

1. Update active channel range/code/carrier state.
2. Compute path loss and antenna gain.
3. For each sample, accumulate each channel's data bit, C/A code chip, and carrier table values.
4. Store interleaved I/Q samples in `iq_buff`.
5. Write the block into the FIFO for TX.

The loop emits exactly the requested number of 100 ms blocks. Carrier Doppler
is derived from consecutive pseudoranges so receiver and satellite motion are
both represented. Signed I/Q scaling uses symmetric rounding.

Carrier sine/cosine values use 512-entry integer lookup tables.

## bladeRF Configuration

`main()` configures the TX module from the selected signal profile and optional CLI overrides. The GPS defaults are:

| Setting | Value |
| --- | --- |
| Center frequency | `1575420000` Hz |
| Sample rate | `2600000` sps |
| Bandwidth | `2500000` Hz |
| Format | `BLADERF_FORMAT_SC16_Q11` |
| TX VGA1 | `-25` dB |
| TX VGA2 | `0` dB |

XB200 mode (`-x 200`) attaches the expansion board, selects custom TX/RX filter banks, and uses the bypass path.

## Build Architecture

The Makefile builds:

```text
bladegps.o + gpssim.o + gnss.o + getch.o -> bladegps
```

Dependency discovery:

1. Try `pkg-config --cflags --libs libbladeRF`.
2. If unavailable, fall back to a sibling Nuand bladeRF source/build tree:

```text
../bladeRF/host/libraries/libbladeRF/include
../bladeRF/host/build/output
```

The source can also be built with explicit `BLADERF_CFLAGS` and `BLADERF_LIBS`.

## Portability Notes

- The real-time build is centered on POSIX threads and libbladeRF.
- Windows compatibility files are retained (`getopt.c`, `getopt.h`, `conio.h` paths).
- POSIX interactive keyboard mode uses `getch.c`.
- Automatic ephemeris download uses external `curl` and `gzip` commands.
- Compressed RINEX input uses `fork`/`exec` on POSIX; Windows currently requires a decompressed navigation file.
- The command construction assumes generated file names only, not arbitrary user-controlled download paths.

## Error Handling Strategy

The current code favors clear failure over partial or silent operation:

- CLI path strings are bounded to `MAX_CHAR`.
- Static LLH, duration, and XB board values are parsed with validation.
- Missing ephemeris data fails before hardware TX starts.
- Malformed NMEA lines are skipped.
- Malformed RINEX records stop parsing without indexing outside satellite bounds.
- Unhealthy satellites from RINEX navigation records are not allocated to channels.
- GPS receiver time is normalized across week boundaries during long simulations.
- GPS initialization failure wakes waiting threads.
- GPS generation failures propagate through `gps.error` and return non-zero.
- TX stream errors stop generation, wake the GPS producer if it is waiting for FIFO space, and return non-zero.
- `SIGINT` and `SIGTERM` set an async-signal-safe flag; generation stops at a block boundary, buffered TX drains, and the process returns status 130.
- Download failures clean temporary files and suggest manual `-e`.

## Safety Model

bladeGPS can generate signals in a protected satellite navigation band. The software does not enforce RF containment; that is the operator's responsibility. Use a shielded test setup, direct cable injection, suitable attenuation, and local legal authorization.

## Known Limits

- GPS L1 C/A only.
- PRN support is limited to GPS PRN 1-32.
- Galileo E1, BeiDou B1I, and GLONASS L1OF are registered profiles but are deliberately rejected until their complete waveform/navigation/ephemeris backends pass the gates in `MULTI_GNSS.md`.
- No integrity or ionospheric scenario editor.
- No built-in almanac download.
- Auto-download currently uses NOAA/NGS and BKG daily legacy GPS broadcast navigation files.
- Hardware behavior depends on local bladeRF firmware, libbladeRF version, clocking, gain setup, and RF test environment.
- The realtime path is tested by build/static analysis here; full RF validation requires hardware and shielded lab equipment.
- See `GPS_L1_CA_COVERAGE.md` for a more detailed implementation and non-certified-area matrix.

## Extension Points

Good places to extend:

- Add new ephemeris sources in `download_broadcast_ephemeris()`.
- Add CLI options in `main()` and `usage()` in `bladegps.c`.
- Add receiver-motion formats beside `readUserMotion()` and `readNmeaGGA()`.
- Tune TX settings in `bladegps.h`.
- Add new GNSS constellations by extending ephemeris, code generation, channel state, and nav message generation in `gpssim.c`.
- Add release artifacts by extending the GitHub release workflow outside the C code.

## Verification Checklist

Before release:

```sh
make clean all
clang --analyze -I/opt/local/include bladegps.c gpssim.c getch.c
git diff --check
./bladegps
```

For environments where the source tree is read-only or protected, compile objects into a temporary directory:

```sh
cc -O3 -Wall -I/opt/local/include -c bladegps.c -o /tmp/bladegps.o
cc -O3 -Wall -I/opt/local/include -c gpssim.c -o /tmp/gpssim.o
cc -O3 -Wall -I/opt/local/include -c getch.c -o /tmp/getch.o
cc /tmp/bladegps.o /tmp/gpssim.o /tmp/getch.o -lm -lpthread -L/opt/local/lib -lbladeRF -o /tmp/bladegps
```
