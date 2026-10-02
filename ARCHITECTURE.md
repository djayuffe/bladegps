# bladeGPS Architecture

This document describes how bladeGPS is organized, how data moves through the simulator, and where to change specific behavior. It is intended for maintainers who need to audit, extend, port, or debug the project.

bladeGPS is a real-time multi-GNSS simulator framework for bladeRF with integrated software backends for GPS L1 C/A, Galileo E1, BeiDou B1I, and GLONASS L1OF. Hardware receiver certification is outside the automated test boundary.

## Source Layout

| File | Responsibility |
| --- | --- |
| `bladegps.c` | Command-line entry point for real-time bladeRF operation, automatic ephemeris download, FIFO utilities, TX thread, bladeRF setup, cleanup, and process lifecycle. |
| `bladegps.h` | bladeGPS-specific constants, thread/FIFO state, simulator option state, bladeRF TX state, and cross-module declarations. |
| `blade_hw.c` / `blade_hw.h` | Device capability/range queries, bladeRF-model adaptation, RF passband validation, generic/legacy gain configuration, XB200 TX filtering, and SC16 Q11 level telemetry. |
| `gnss.c` / `gnss.h` | Constellation and signal registry, RF/code profiles, profile parsing, capability status, and passband validation. |
| `gnss_time.c` / `gnss_time.h` | Historical leap-second table and lossless GPS/GST/BDT/GLONASS-UTC conversion used by record selection, propagation, and symbol scheduling. |
| `gnss_receiver.c` / `gnss_receiver.h` | Independent coherent BPSK carrier/code search used for generated-I/Q loopback validation. |
| `motion_controller.c` / `motion_controller.h` | Optional SDL2 controller discovery, dead-zone processing, disconnect handling, and north/east/up velocity commands. |
| `gnss_codes.c` / `gnss_codes.h` | ICD-derived BeiDou B1I and GLONASS L1OF ranging codes, GLONASS FDMA carrier mapping, and Galileo E1 memory-code decoding/CBOC primitives. |
| `galileo_e1_codes.c` | All 50 E1-B and 50 E1-C primary memory codes from the Galileo OS SIS ICD v2.2 electronic Annex C. |
| `gnss_nav.c` / `gnss_nav.h` | Dynamically sized typed GPS RINEX 2 and mixed RINEX 3/4 navigation records for GPS LNAV, Galileo INAV/FNAV, BeiDou D1/D2, and GLONASS FDMA, plus BeiDou Klobuchar metadata. |
| `gnss_orbit.c` / `gnss_orbit.h` | Constellation-aware Keplerian, BeiDou GEO, clock/relativity, and GLONASS numerical propagation models. |
| `gnss_geometry.c` / `gnss_geometry.h` | Iterative transmit-time observations, Earth-rotation correction, azimuth/elevation, satellite clock correction, range rate, Doppler, and pseudorange-derived initial code/carrier phases. |
| `gnss_fec.c` / `gnss_fec.h` | Galileo CRC-24Q/convolutional/interleaving primitives and BeiDou BCH/interleaving primitives. |
| `gnss_galileo_nav.c` / `gnss_galileo_nav.h` | Galileo nominal E1-B I/NAV vertical-page construction, CRC coverage, FEC/interleaving, sync/SSP insertion, and 30-second word schedule. |
| `gnss_beidou_nav.c` / `gnss_beidou_nav.h` | BeiDou D1/D2 word coding and interleaving, D1 clock/ephemeris subframes 1-3, D2 GEO basic-navigation pages 1-10, shared almanac payloads, format-specific schedules, and RINEX-to-ICD scaling. |
| `gnss_glonass_nav.c` / `gnss_glonass_nav.h` | GLONASS RINEX A15 conversion and GNAV immediate strings 1-4, including sign-magnitude fields, UTC(SU)+3 timing, four-year day index, and Hamming-protected 85-bit strings. |
| `gnss_rf.c` / `gnss_rf.h` | Shared mixed-constellation channel validation, elevation-ranked allocation, continuous carrier/code/data/overlay phase, SC16 Q11 mixing, Galileo E1 CBOC, BPSK overlay modulation, and GLONASS relative/meander symbol formation. |
| `gnss_schedule.c` / `gnss_schedule.h` | Time-ordered Galileo 30-second I/NAV, BeiDou D1 frame, BeiDou D2 ten-frame GEO, and GLONASS 15-string symbol-cycle assembly for the RF renderer. |
| `gnss_task.c` / `gnss_task.h` | Typed-RINEX and mixed-open production thread: native time conversion, GPS LNAV adaptation, joint selection, motion, health/age filtering, geometry, FDMA assignment, reconciliation, I/Q rendering, FIFO flow, and shutdown. |
| `gpssim.c` | Inherited GPS/time/coordinate/motion primitives, C/A and LNAV generation, plus retained legacy standalone/file-output and GPS producer paths not selected by the production CLI. |
| `gpssim.h` | GPS constants and data structures: times, ephemeris records, pseudorange records, and channel state. |
| `getch.c` / `getch.h` | POSIX keyboard helpers used by interactive mode. Windows uses `conio.h`. |
| `getopt.c` / `getopt.h` | Windows-compatible `getopt` implementation retained for portability. |
| `Makefile` | Builds `bladegps`, discovering libbladeRF through `pkg-config` when available. |
| `Readme.md` | User-facing install, usage, safety, and examples. |
| `CHANGELOG.md` | Release history. |
| `SUPPORT_MATRIX.md` | Exact service, input, navigation-payload, time, motion, hardware, validation, and unsupported-feature boundaries. |
| `CLI_REFERENCE.md` | Exact option syntax, ranges, defaults, interactions, RF-plan equations, and exit behavior. |
| `DATA_FORMATS.md` | Navigation, compression, receiver-motion, NMEA, and SC16 stream schemas. |
| `API_REFERENCE.md` | Exported C types/functions and their parameter, unit, validation, ownership, and return contracts. |
| `brdc*.??n`, `*.csv` | Sample ephemeris and motion/input data. |

## Runtime Overview

The real-time executable is built with `BLADE_GPS` enabled through `gpssim.h`.
The historical `gps_task()` implementation remains available for source
compatibility, while every CLI profile is routed through `gnss_task()`.

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
   +--> shared producer: gnss_task()
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
navigation cycles, motion, and FIFO production for GPS L1 C/A, every individual
non-GPS profile, and the four-constellation `mixed-open` profile. GPS RINEX 2
and typed RINEX 3/4 LNAV records are both adapted to the same navigation record
model before scheduling and synthesis.

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
9. Start the shared GNSS producer thread.
10. Wait for producer initialization.
11. Configure and enable the bladeRF synchronous TX interface.
12. Start the TX consumer thread.
13. Join TX, disable TX, join the GNSS producer, free resources, and close the device.

The program returns non-zero if setup fails, GNSS generation cannot initialize,
or bladeRF TX streaming reports an error.

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
| `gps` | GNSS generation thread state and initialization condition variable; the field name is retained for source continuity. |
| `finished` | Shared shutdown signal. |
| `fifo` | Interleaved SC16 I/Q sample FIFO. |
| `head`, `tail` | FIFO write/read sample positions. |
| `fifo_read_ready` | Signals the TX thread that samples are available. |
| `fifo_write_ready` | Signals the GNSS producer that FIFO space is available. |

FIFO state is protected with `gps.lock`. The older `tx.lock` remains in the struct but current FIFO coordination uses the GPS mutex consistently.

## Thread Model

### GNSS Producer

`gnss_task()` owns production signal synthesis for every advertised profile.

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
- Measure SC16 Q11 peaks and rail contact before each transfer.
- Zero-pad and submit the final partial buffer as a complete synchronous transfer so the stream cannot retain the scenario tail.
- On TX error, set `tx.error`, set `finished`, broadcast FIFO condition variables, and exit.

`main()` joins the TX thread first during normal completion, then disables TX
and joins the GNSS producer thread.

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

## Shared GNSS Signal Pipeline

The CLI production path is coordinated by `gnss_task.c`. It reuses proven GPS
LNAV primitives from `gpssim.c`, while constellation-neutral record selection,
geometry, scheduling, allocation, and RF rendering are implemented in the
`gnss_*` modules.

### Navigation ingestion and selection

`gnss_load_rinex_nav()` performs a validated two-pass load into dynamically
allocated `gnss_nav_record_t` records. It accepts GPS RINEX 2 and supported
RINEX 3/4 GPS LNAV, Galileo INAV/FNAV, BeiDou D1/D2, and GLONASS FDMA records.
Unsupported RINEX 4 record families are skipped at record boundaries. Blank
orbit fields remain explicit `NAN` values rather than becoming meaningful zero.

At every 100 ms generation step, `gnss_task()`:

1. Filters records by the selected service.
2. Rejects unhealthy, malformed, arbitrary-future, and out-of-age records.
3. Selects the newest applicable record independently per system and PRN.
4. Converts the receiver epoch to the record's native system time.
5. Builds one candidate observation per usable satellite.

GPS RINEX records are adapted to the inherited `ephem_t`/LNAV builders only
after typed validation. Galileo E1 consumes INAV records; recognized FNAV data
is not silently substituted into the E1 I/NAV scheduler.

### Receiver Motion

Supported modes:

- Static LLH from `-l`.
- CSV ECEF motion from `-u`.
- CSV geodetic motion from `-p`.
- NMEA GGA stream from `-g`.
- Keyboard interactive motion from `-i`: `w/s/a/d` for horizontal movement and `e/q` for up/down.
- SDL2 USB/Bluetooth controller motion from `-j` when SDL2 is available.

Static LLH is converted through `llh2xyz()`. NMEA GGA is parsed into LLH and then ECEF. CSV motion expects:

```text
time_seconds,ecef_x_m,ecef_y_m,ecef_z_m
```

The row-pointer table and ECEF samples use one contiguous allocation, avoiding
hundreds of thousands of small allocations in a 24-hour scenario. Dynamic ECEF
input is converted back to LLH for status output and interactive local-axis
motion. The local tangent frame is refreshed while the receiver moves.

### Satellite geometry and timing

`gnss_observe()` dispatches to the constellation-specific orbit backend and
iterates satellite transmit time. The observation contains Sagnac-corrected
range, azimuth/elevation, range rate, Doppler, signal group delay, transmit
seconds-of-week, and initial code/carrier phase. GPS and Galileo use Keplerian
models; BeiDou adds its GEO transform; GLONASS uses numerical state-vector
propagation. Clock bias, clock drift, relativity, and the applicable broadcast
group delay are kept in the modeled measurement path.

### Codes and navigation schedules

`build_store()` prepares a complete service cycle for the selected record:

- GPS C/A PRN 1–37 plus a 30-second LNAV cycle built with `eph2sbf()` and
  `generateNavMsg()`.
- Galileo E1-B/E1-C primary codes, E1-C secondary code, and scheduled I/NAV.
- BeiDou B1I code, D1 NH overlay, and the applicable D1 or D2 navigation cycle.
- GLONASS L1OF code and a fifteen-string GNAV cycle containing relative-code,
  meander, and time-mark symbols.

Stores are rebuilt when their broadcast record, system week, or 30-second
schedule epoch changes. Navigation and overlay phase starts at satellite
transmit time, not receiver time.

### Allocation and I/Q synthesis

For each 100 ms block:

1. `gnss_rf_allocate()` filters out-of-band/below-mask candidates and selects
   at most sixteen healthy satellites in descending elevation order.
2. A desired channel bank is created with carrier, code, data, pilot/overlay,
   Doppler, amplitude, and initial phase state.
3. `gnss_rf_reconcile()` preserves every live phase for unchanged channel
   identities while applying their newly observed Doppler and clock rates.
4. `gnss_rf_render()` generates BPSK or Galileo E1 CBOC samples and sums all
   channels into interleaved SC16 Q11.
5. The complete block is written to the FIFO for the TX consumer.

Carrier, code, navigation-data, and secondary/NH clocks are continuous across
blocks. Code/data/overlay clocks use one Doppler scale per satellite. The mixer
computes a deterministic whole-bank peak bound and applies one fixed headroom
scale for the bank, avoiding block-by-block AGC pumping.

### Retained legacy GPS implementation

`gpssim.c` still contains the original standalone GPS producer and file-output
path for source compatibility. The CLI no longer selects that producer. Shared
production nevertheless reuses its GPS C/A, LNAV, time, coordinate, and motion
primitives where those remain authoritative in this codebase.

## bladeRF Configuration

`main()` builds a hardware request from the selected signal profile and optional
CLI overrides. `blade_hw_configure_tx()` applies it in this order:

1. Identify the board and require a configured FPGA.
2. Configure optional XB200 TX-only native L-band bypass and automatic filter selection.
3. Query the live frequency range, tune, and require exact read-back.
4. Query the live sample-rate range, configure it, and reject timing-changing coercion.
5. Query/configure analog bandwidth and validate the realized filter width against `occupied bandwidth + 2 * abs(carrier - center)`.
6. Apply generic overall gain, or explicitly requested bladeRF 1.0 named stages, after tuning because valid gain ranges can depend on frequency.

libbladeRF range structures are interpreted using `value * scale`. Ranges are
queried from the open device rather than cached, allowing one binary to adapt
to bladeRF 1.0 and 2.0 hardware. The analog bandwidth must contain the whole
modulated signal and must not exceed the complex sample rate.

ADC setup is intentionally absent because bladeGPS never enables RX. The TX
equivalent concern is DAC input integrity, handled through SC16 Q11 peak and
rail accounting. Overall TX gain is relative and is not a calibrated output
power measurement.

The GPS defaults are:

| Setting | Value |
| --- | --- |
| Center frequency | `1575420000` Hz |
| Sample rate | `2600000` sps |
| Bandwidth | `2500000` Hz |
| Format | `BLADERF_FORMAT_SC16_Q11` |
| Overall TX gain | `27` dB |

Legacy bladeRF 1.0 stage overrides require both `-a` and `-A`. XB200 mode
(`-x 200`) changes TX only and uses automatic 1 dB filter selection on the
native-frequency bypass path.

## Build Architecture

The Makefile links the CLI/hardware layer, inherited GPS primitives, shared GNSS
runtime, constellation backends, RF renderer, controller adapter, and platform
helpers:

```text
bladegps + blade_hw + gpssim + gnss_task + gnss_{time,nav,orbit,geometry}
         + gnss_{codes,fec,schedule,rf} + constellation navigation backends
         + motion_controller + platform helpers -> bladegps executable
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
- Compressed GPS or mixed RINEX input uses `fork`/`exec` on POSIX; Windows
  currently requires a decompressed navigation file.
- The command construction assumes generated file names only, not arbitrary user-controlled download paths.

## Error Handling Strategy

The current code favors clear failure over partial or silent operation:

- CLI path strings are bounded to `MAX_CHAR`.
- Static LLH, duration, and XB board values are parsed with validation.
- Missing ephemeris data fails before hardware TX starts.
- Malformed NMEA lines are skipped.
- Malformed RINEX records stop parsing without indexing outside satellite bounds.
- Unhealthy satellites from RINEX navigation records are not allocated to channels.
- GNSS receiver time is normalized across week boundaries during long simulations.
- GNSS initialization failure wakes waiting threads.
- GNSS generation failures propagate through `gps.error` and return non-zero;
  the legacy field name is retained in `sim_t`.
- TX stream errors stop generation, wake the GNSS producer if it is waiting for FIFO space, and return non-zero.
- `SIGINT` and `SIGTERM` set an async-signal-safe flag; generation stops at a block boundary, buffered TX drains, and the process returns status 130.
- Download failures clean temporary files and suggest manual `-e`.

## Safety Model

bladeGPS can generate signals in a protected satellite navigation band. The software does not enforce RF containment; that is the operator's responsibility. Use a shielded test setup, direct cable injection, suitable attenuation, and local legal authorization.

## Known Limits

- Advertised waveforms are GPS L1 C/A, Galileo E1 OS, BeiDou B1I, GLONASS
  L1OF, and their `mixed-open` combination. Modernized GPS, SBAS, and other
  signal families listed in `SUPPORT_MATRIX.md` are not selectable.
- GPS C/A assignments stop at PRN 37; SBAS PRN 120–158 are not generated.
- Optional navigation content that is not present in broadcast ephemeris input
  is marked dummy, reserved, or unavailable instead of being fabricated.
- There is no integrity, multipath, spoofing, or ionospheric scenario editor.
- There is no independent built-in almanac downloader.
- Automatic acquisition uses NOAA/NGS and BKG GPS or mixed daily broadcast
  navigation products; it is not a general archival product client.
- Hardware behavior depends on local bladeRF firmware, FPGA, libbladeRF,
  clocking, gain setup, filter response, and RF test environment.
- Software tests do not replace calibrated spectrum, navigation decode, PVT,
  or shielded receiver interoperability validation.

## Extension Points

Good places to extend:

- Add new ephemeris sources in `download_broadcast_ephemeris()`.
- Add CLI options in `main()` and `usage()` in `bladegps.c`.
- Add receiver-motion formats beside `readUserMotion()` and `readNmeaGGA()`.
- Tune TX settings in `bladegps.h`.
- Add a new service by extending `gnss.c` profiles, typed navigation ingestion,
  time/group-delay rules, orbit/clock dispatch, code generation, navigation/FEC
  scheduling, `gnss_rf` modulation, production routing, and independent tests.
- Add release artifacts by extending the GitHub release workflow outside the C code.

## Verification Checklist

Before release:

```sh
make clean
make check
clang --analyze -I/opt/local/include bladegps.c gnss_task.c gnss_nav.c gnss_rf.c
git diff --check
./bladegps -L
```

For environments where the source tree is read-only or protected, compile objects into a temporary directory:

```sh
cc -O3 -Wall -I/opt/local/include -c bladegps.c -o /tmp/bladegps.o
cc -O3 -Wall -I/opt/local/include -c gpssim.c -o /tmp/gpssim.o
cc -O3 -Wall -I/opt/local/include -c getch.c -o /tmp/getch.o
cc /tmp/bladegps.o /tmp/gpssim.o /tmp/getch.o -lm -lpthread -L/opt/local/lib -lbladeRF -o /tmp/bladegps
```
