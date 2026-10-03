# bladeGPS

Real-time, broadcast-ephemeris-driven GNSS signal-simulator framework for
bladeRF, based on the GPS signal model from
[gps-sdr-sim](https://github.com/osqzss/gps-sdr-sim). Production backends turn
receiver position and motion into continuous SC16 I/Q for GPS L1 C/A, Galileo
E1 OS, BeiDou B1I D1/D2, GLONASS L1OF, or a jointly allocated mixture of all
four. The pipeline includes native time conversion, orbit and clock modeling,
iterative transmit time, navigation-message construction, spreading codes,
modulation, Doppler, channel allocation, hardware validation, and real-time TX.

**Repository description:** Real-time multi-GNSS baseband and bladeRF transmitter
for GPS L1 C/A, Galileo E1, BeiDou B1I, and GLONASS L1OF, driven by broadcast
RINEX navigation and static, recorded, keyboard, or controller receiver motion.

**Suggested GitHub topics:** `bladerf`, `gnss`, `gps`, `galileo`, `beidou`,
`glonass`, `sdr`, `signal-simulator`, `rinex`, `baseband`, `c`, `navigation`.

This is research and lab software. Only transmit GPS-like RF signals inside a properly shielded test setup, with appropriate attenuation, and only where you are legally allowed to do so.

## Features

- GPS L1 C/A PRN 1-37 baseband generation with up to 16 simulated channels.
- Galileo E1-B/E1-C CBOC, BeiDou B1I D1/D2 BPSK, and GLONASS L1OF FDMA generation.
- Mixed RINEX 3/4 navigation ingestion with constellation-specific orbit, clock,
  navigation-message, code, modulation, visibility, Doppler, and allocation paths.
- Static receiver mode using latitude, longitude, and height.
- Dynamic receiver mode from CSV ECEF user motion files.
- Dynamic receiver mode from geodetic latitude/longitude/height CSV files.
- Dynamic receiver mode from NMEA GGA streams.
- Optional keyboard-controlled interactive motion mode.
- Optional live SDL2 USB/Bluetooth game-controller receiver motion.
- Timestamp-aware 10 Hz interpolation of ECEF, geodetic, and NMEA motion.
- RINEX broadcast navigation file parsing.
- Direct POSIX streaming of plain, `.gz`, and legacy Unix-compressed `.Z`
  supported RINEX 2, 3, and 4 navigation files.
- Per-satellite ephemeris selection and seamless 30-second ephemeris refresh.
- Iterative signal transit-time and Earth-rotation (Sagnac) correction.
- GPS/GST/BDT/GLONASS-UTC conversion with historical leap-second handling.
- Fraction-preserving GPS-to-UTC conversion, including explicit `23:59:60.x`
  round trips, and integer-derived 10 Hz scenario ticks without cumulative drift.
- Per-satellite transmit-time navigation-symbol/overlay alignment.
- Satellite clock bias, relativistic correction, signal group delay, and clock-drift modeling.
- Automatic live/daily broadcast download through BKG rolling and merged
  products, independent IGN mirrors, and NOAA/NGS/BKG GPS legacy fallbacks,
  with content, signal-family, health, orbit-age, and completeness validation.
- Capability-driven bladeRF 1.0/2.0 adaptation for center frequency, exact sample rate, analog bandwidth, and portable overall TX gain.
- Hardware range checks and configuration read-back before RF transmission.
- Analog-filter validation against the complete occupied signal span, including an offset carrier.
- Deterministic SC16 Q11 headroom normalization, peak/rail telemetry, and
  hardware-timestamped continuous-burst transmission with verified final drain.
- Signal registry and RF validation for GPS L1 C/A, Galileo E1, BeiDou B1I, and GLONASS L1OF.
- Real-time SC16 I/Q streaming to bladeRF.
- Optional XB200 setup for GPS-band transmit filtering/path selection.
- Graceful generator/TX error propagation and `SIGINT`/`SIGTERM` shutdown.
- A true `mixed-open` runtime that jointly allocates and renders GPS L1 C/A,
  Galileo E1, BeiDou B1I, and GLONASS L1OF from one mixed navigation file.
- Independent software BPSK acquisition/correlation loopback validation.
- Deterministic core tests for system time, leap seconds, transmit alignment,
  coordinates, PRN codes, navigation coding, RF continuity, motion, and selection.
- Portable Makefile that uses `pkg-config libbladeRF` when available, with the original adjacent bladeRF source-tree fallback.

## Requirements

- C compiler with C99-compatible libc behavior.
- POSIX threads.
- libbladeRF headers and library.
- A bladeRF device supported by libbladeRF.
- `pkg-config` is recommended so the Makefile can discover libbladeRF automatically.
- `curl` is required for automatic ephemeris download. `gzip` is required for
  downloads and for direct `.gz`/legacy `.Z` navigation input on POSIX.
- SDL2 is optional; when found through `pkg-config`, `-j` live controller input is enabled.

On macOS with MacPorts, for example, the build can use libbladeRF from `/opt/local` through `pkg-config`. On Linux, install libbladeRF development files through your package manager or build them from Nuand's source tree.

## Build

```sh
make
```

Run the non-RF core-model checks with:

```sh
make check
```

If libbladeRF is not discoverable through `pkg-config`, the Makefile falls back to the historical layout:

```text
../bladeRF/host/libraries/libbladeRF/include
../bladeRF/host/build/output
```

You can also pass flags explicitly:

```sh
make BLADERF_CFLAGS="-I/path/to/libbladeRF/include" BLADERF_LIBS="-L/path/to/lib -lbladeRF"
```

Clean build products:

```sh
make clean
```

For a guided first run, hardware/filter explanation, motion examples, and
troubleshooting, read [USER_GUIDE.md](USER_GUIDE.md).

## Usage

List the known signal profiles before configuring a run:

```sh
./bladegps -L
```

The status column is authoritative. Non-GPS profiles use a supplied mixed RINEX 3/4 file or auto-download a daily mixed file when `-e` is omitted. Complete GLONASS GNAV requires populated extended FDMA fields. RINEX 4 supplies them; RINEX 3.05 Orbit-4 is accepted when populated, while daily files containing blank/sentinel fields are rejected rather than fabricated.

```text
Usage: bladegps [options]
Options:
  -e <nav_file>    RINEX navigation file (daily broadcast data auto-downloads if omitted)
  -u <user_motion> User motion file (dynamic mode)
  -p <llh_motion>  Geodetic CSV motion: time,latitude,longitude,height
  -g <nmea_gga>    NMEA GGA stream (dynamic mode)
  -l <location>    Lat,Lon,Hgt (static mode) e.g. 35.274,137.014,100
  -t <date,time>   Scenario start time YYYY/MM/DD,hh:mm:ss
  -R               Use current UTC/GPS time and align sample zero to the wall clock
  -d <duration>    Duration [sec] (max: 86400)
  -x <XB number>   Enable XB board, e.g. '-x 200' for XB200
  -S <signal>      Signal profile (gps-l1ca, galileo-e1, beidou-b1i, glonass-l1of, mixed-open)
  -L               List signal profiles and implementation status
  -D <device>      libbladeRF device identifier
  -f <Hz>          TX center frequency
  -r <samples/s>   TX sample rate (at least 1 MHz and divisible by 10)
  -b <Hz>          TX analog bandwidth
  -G <dB>          Portable overall TX gain (default: 27 dB)
  -a <dB>          Legacy bladeRF 1 TXVGA1 gain (requires -A)
  -A <dB>          Legacy bladeRF 1 TXVGA2 gain (requires -a)
  -M <degrees>     Satellite elevation mask (-90 to 90)
  -i               Interactive mode: North='w', South='s', East='d', West='a', Up='e', Down='q'
  -j <index>       Live SDL USB/Bluetooth controller (left stick NE, right stick vertical)
```

Static location example:

```sh
./bladegps -e brdc2940.18n -l 59.3293,18.0686,30 -d 60
```

Automatic ephemeris download example:

```sh
./bladegps -l 59.3293,18.0686,30 -d 60
```

When `-e` is omitted for a live start, bladeGPS first tries BKG's rolling
24-hour multi-GNSS navigation product, which BKG refreshes every 15 minutes.
Daily fallbacks include BKG's comprehensive DLR RINEX 4 product,
`BRDC00IGS`, `BRDM00DLR`, and the WRD receiver/stream merges, followed by
independent IGN mirrors. GPS additionally falls back to NOAA/NGS and BKG
legacy RINEX 2 files. A cached or downloaded file is reused only after parsing
proves that it contains healthy, in-age records usable by the selected profile.
Without `-t`, automatic download also enables live mode: sample zero represents
current UTC converted to GPS time plus a five-second generation lead and is
scheduled onto the matching future FPGA timestamp. `-R` enables the same
behavior with a supplied `-e` file. The host UTC clock must be synchronized.

The downloader writes to temporary `.tmp` files, decompresses and parses every candidate, checks signal family, health, extended GLONASS usability, and scenario age, then atomically promotes only a usable result. For a live start it can try the preceding UTC day. Failed or stale products are never reported as successful downloads.

Galileo example using a mixed RINEX 3/4 navigation file:

```sh
./bladegps -S galileo-e1 -e BRDC00IGS_R_20262740000_01D_MN.rnx \
  -l 59.3293,18.0686,30 -d 60
```

Use `-S beidou-b1i` or `-S glonass-l1of` for those integrated backends. The
selected center frequency, sample rate, and bandwidth default to the profile's
safe values and can be overridden with `-f`, `-r`, and `-b`.

Simultaneous open-service example (wideband hardware and a mixed RINEX 3/4 file):

```sh
./bladegps -S mixed-open -e BRDC00IGS_R_20262740000_01D_MN.rnx \
  -l 59.3293,18.0686,30 -d 60
```

The default mixed plan is centered at 1582.3925 MHz with 50 Msps and a 48 MHz
analog filter around a 47.1 MHz minimum waveform span. The allocator considers all four constellations together and
keeps the 16 highest-elevation healthy signals that fit the realized device
passband. Device range checks can reject this plan on hardware that cannot
provide the required instantaneous bandwidth.

### Hardware adaptation and RF filtering

bladeGPS queries frequency, sample-rate, analog-bandwidth, and gain ranges from
the opened device instead of assuming bladeRF 1.0 limits. It sets and reads
back every timing-critical value. Exact sample rate and center frequency are
required because silent coercion would change code, symbol, and carrier timing.
Analog bandwidth may be quantized by hardware, but the realized value is
accepted only when read-back agrees with libbladeRF, contains the complete
modulated signal, and is no wider than the complex sample rate. Waveform
occupancy and requested filter bandwidth are separate profile properties, so
guard margin is not incorrectly advertised as emitted spectrum.

| Profile | Minimum waveform span | Default sample rate | Default filter |
| --- | ---: | ---: | ---: |
| `gps-l1ca` | 2.2506 MHz | 2.6 Msps | 2.5 MHz |
| `galileo-e1` | 24.552 MHz | 36.828 Msps | 28 MHz |
| `beidou-b1i` | 4.5012 MHz | 5 Msps | 5 MHz |
| `glonass-l1of` | 8.9992 MHz | 12 Msps | 10 MHz |
| `mixed-open` | 47.1 MHz | 50 Msps | 48 MHz |

The Galileo filter is wider than its reference bandwidth so satellite Doppler
and device filter quantization do not sit on the acceptance boundary. The mixed
sample rate is wider than its analog filter, preserving a digital Nyquist guard
at both edges.

Use `-G` for model-independent overall TX gain. This is a relative gain setting,
not calibrated RF power. `-a` and `-A` remain available together for bladeRF
1.0 systems that explicitly require the legacy `txvga1` and `txvga2` stages;
they are rejected on devices without those named stages.

With `-x 200`, the XB200 uses the native L-band bypass path and automatic
low-loss TX filter selection; attachment, path, and filter selection are read
back and verified. The RX path is not modified. On bladeRF 2.0,
libbladeRF chooses the AD9361 interpolation/FIR mode while configuring sample
rate; forcing another FIR mode here would override that device adaptation.

bladeGPS is transmit-only. No ADC or RX channel participates in generation, so
configuring an ADC would be unnecessary and could disturb a separate receiver.
On TX, every submitted SC16 Q11 block is measured; the final report shows peak
level and rail contact. A final partial scenario block is zero-padded to a full
synchronous transfer so libbladeRF cannot retain and drop the scenario tail.

User motion CSV example:

```sh
./bladegps -e brdc2940.18n -u circle.csv -d 120
```

NMEA GGA example:

```sh
./bladegps -e brdc2940.18n -g track.nmea -d 120
```

Geodetic motion example:

```sh
./bladegps -e brdc2940.18n -p route-llh.csv -d 120
```

XB200 example:

```sh
./bladegps -e brdc2940.18n -l 35.274,137.014,100 -x 200 -d 60
```

Interactive movement example:

```sh
./bladegps -e brdc2940.18n -l 59.3293,18.0686,30 -i -d 120
```

In interactive mode, `w/s/a/d` move north/south/west/east and `e/q` move up/down.

## Input files

- `brdc*.??n`, `brdc*.??n.gz`, and `brdc*.??n.Z` files are GPS RINEX 2
  broadcast navigation files. Mixed RINEX 3/4 navigation may likewise be plain,
  `.gz`, or legacy `.Z` on POSIX. Compressed input is streamed through `gzip`
  without constructing a shell command.
- `circle.csv`, `satellite.csv`, and `ss520-4.csv` are sample motion/position data files.
- `run_bladerfGPS.sh` is a convenience script retained from the original project.

User motion CSV rows use:

```text
time_seconds,ecef_x_m,ecef_y_m,ecef_z_m
```

Records are consumed at 10 Hz. All four CSV fields must be finite numbers;
malformed records are rejected instead of silently shortening the scenario.
NMEA input accepts valid GGA fixes and ignores no-fix records.
Checksummed sentences are verified; timestamps are unwrapped across midnight.
All recorded motion formats are linearly resampled to the simulator's 100 ms
clock, so irregular input spacing no longer changes simulated speed.

Geodetic motion rows use decimal degrees and metres:

```text
time_seconds,latitude_degrees,longitude_degrees,height_metres
```

Latitude is limited to -90..90 degrees and longitude to -180..180 degrees. Records are converted to ECEF before signal generation.

Live controller example:

```sh
./bladegps -S gps-l1ca -l 59.3293,18.0686,30 -j 0 -d 120
```

The left stick commands north/east velocity and the right-stick vertical axis
commands up/down velocity. Dead-zone removal and diagonal normalization are
applied. Disconnecting the controller stops generation with an error rather
than freezing the last velocity.

## Signal support

“Implemented” below means connected to the production CLI and covered by
software tests. It does not mean certified ICD conformance or calibrated RF
interoperability. See [SUPPORT_MATRIX.md](SUPPORT_MATRIX.md) for layer-by-layer
coverage, payload boundaries, input formats, hardware support, and validation.

| Profile | Constellation | Nominal carrier | Status | Notes |
| --- | --- | ---: | --- | --- |
| `gps-l1ca` | GPS | 1575.42 MHz | Implemented | C/A ranging code, LNAV, RINEX 2 GPS navigation, PRN 1-37 |
| `galileo-e1` | Galileo | 1575.42 MHz | Software implemented | E1-B/C codes, CBOC, mixed-RINEX orbit/clock, ICD modulo-30 I/NAV page-part timing, vertical dummy pages for unavailable optional words, geometry, allocation and RF synthesis |
| `beidou-b1i` | BeiDou | 1561.098 MHz | Software implemented | PRN 1–63 codes, D1/D2 selection, ephemeris/clock/ionosphere pages, geometry, NH overlay, allocation and RF synthesis |
| `glonass-l1of` | GLONASS | 1602 MHz base | Software implemented | L1OF code, FDMA slot carriers, state-vector propagation, live UTC(SU)+3 immediate/time strings, safe unavailable-almanac marking, relative/meander modulation and RF synthesis |
| `mixed-open` | GPS + Galileo + BeiDou + GLONASS | 1582.3925 MHz plan center | Software implemented | One mixed RINEX input, constellation-native timing, shared health/elevation allocator, 16-channel continuous wideband mixer |

Unknown or non-implemented profiles fail closed instead of silently producing a
GPS waveform. Galileo, BeiDou, GLONASS, and mixed operation consume either a
supplied or automatically downloaded mixed RINEX 3/4 navigation file. See
[MULTI_GNSS.md](MULTI_GNSS.md) for the payload contract and
[SUPPORT_MATRIX.md](SUPPORT_MATRIX.md) for exact layer-level status.

The source tree contains tested signal primitives for all 63 BeiDou B1I ranging-code assignments, the GLONASS L1OF ranging code and FDMA carrier slots, all 50 official Galileo E1-B and E1-C primary codes, and Galileo CBOC shaping.

The typed RINEX 3/4 loader, GPS LNAV adapter, Galileo/BeiDou Keplerian propagation,
BeiDou GEO transform, GLONASS state-vector propagation, navigation scheduling,
modulation, joint channel allocation, and FIFO producer are connected to
`mixed-open` RF output. Software loopback validates independent BPSK acquisition;
physical hardware and receiver certification remain environment-dependent.

## Implementation notes

- The simulator generates 0.1 second blocks at the selected sample rate for bladeRF SC16 transmission; GPS defaults to 2.6 Msps.
- The requested duration emits the complete number of 100 ms blocks.
- Every block epoch is derived from the immutable start plus an integer 10 Hz
  tick; repeated binary `0.1` addition cannot drift across a symbol boundary.
- The newest healthy in-fit broadcast record already in force is selected independently per constellation/PRN; unhealthy or arbitrary future records cannot mask usable data.
- Keplerian ephemeris age is measured from the broadcast orbit reference time
  (`toe`) in the native GPS/GST/BDT week. GLONASS age is measured from its
  UTC(SU) state-vector epoch. GPS fit intervals are honoured when present.
- Navigation data and overlay phases use iterative per-satellite transmit time rather than receiver time.
- Primary-code, navigation-symbol, and secondary/NH overlay clocks share the
  same per-satellite Doppler scale, preserving component alignment in motion.
- Standalone GPS L1 C/A and `mixed-open` use the same constellation-neutral
  scheduler, allocator, continuous-phase renderer, SC16 normalization, and FIFO.
- Non-GPS 30-second navigation cycles are regenerated at every cycle boundary; Galileo GST TOW and BeiDou BDT SOW therefore advance instead of repeating a cached frame.
- Mixed RINEX files are counted and allocated dynamically; there is no fixed 4096-record truncation ceiling.
- BeiDou Klobuchar coefficients are read from RINEX 3 `BDSA`/`BDSB` headers or RINEX 4 `ION C ... D1D2` records and range-checked at their ICD scales.
- Ephemeris handovers rebuild LNAV data while preserving range-rate continuity.
- FIFO access between the GNSS producer and TX thread is protected with the
  legacy-named `gps.lock` mutex.
- Generation completion wakes both FIFO condition variables so shutdown and initialization failures do not deadlock waiting threads.
- Command-line path arguments are bounded to the internal `MAX_CHAR` buffers.
- Malformed NMEA GGA lines are skipped instead of crashing the parser.
- Automatic acquisition uses the live BKG rolling product, five BKG daily
  merged products, three IGN mirrors, and two additional GPS legacy fallbacks.
- Auto-downloaded ephemeris cache files are ignored by git so local runs do not dirty the repository.
- The full module map, data flow, threading model, FIFO behavior, downloader lifecycle, and extension points are documented in [ARCHITECTURE.md](ARCHITECTURE.md).

## Project documentation

- [ARCHITECTURE.md](ARCHITECTURE.md) - maintainer architecture, runtime flow, modules, and extension points.
- [USER_GUIDE.md](USER_GUIDE.md) - installation, safe first run, RF/filter
  planning, every input mode, hardware behavior, shutdown, troubleshooting,
  and conducted acceptance testing.
- [GPS_L1_CA_COVERAGE.md](GPS_L1_CA_COVERAGE.md) - implemented L1 C/A coverage, wired gaps, and non-certified areas.
- [MULTI_GNSS.md](MULTI_GNSS.md) - constellation architecture, current capability matrix, and acceptance gates.
- [SUPPORT_MATRIX.md](SUPPORT_MATRIX.md) - detailed per-service, input, timing,
  hardware, motion, and validation support matrix.
- [CLI_REFERENCE.md](CLI_REFERENCE.md) - every option, accepted range, default,
  interaction, RF-plan equation, exit behavior, and example.
- [DATA_FORMATS.md](DATA_FORMATS.md) - RINEX, compression, motion, NMEA, typed
  navigation-record, and generated-sample formats.
- [API_REFERENCE.md](API_REFERENCE.md) - exported C types/functions, units,
  parameters, return values, validation, and internal helper map.
- [CHANGELOG.md](CHANGELOG.md) - release history.
- [LICENSE](LICENSE) - MIT license.

## Release history

See [CHANGELOG.md](CHANGELOG.md).

## Safety

This software can generate RF signals in protected satellite navigation bands. Never connect a bladeRF running this program to an antenna in an unshielded environment unless you have explicit legal authority. For receiver testing, use a shielded enclosure, direct cable injection, attenuation, and isolation appropriate for your equipment.

## License

Copyright (c) 2015 Takuji Ebinuma.

Distributed under the MIT License. See [LICENSE](LICENSE).
