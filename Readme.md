# bladeGPS

Real-time GNSS signal-simulator framework for bladeRF, based on the GPS signal model from [gps-sdr-sim](https://github.com/osqzss/gps-sdr-sim). Production backends generate GPS L1 C/A, Galileo E1 OS, BeiDou B1I D1/D2, and GLONASS L1OF from broadcast navigation data.

This is research and lab software. Only transmit GPS-like RF signals inside a properly shielded test setup, with appropriate attenuation, and only where you are legally allowed to do so.

## Features

- GPS L1 C/A baseband generation with up to 16 simulated channels.
- Galileo E1-B/E1-C CBOC, BeiDou B1I D1/D2 BPSK, and GLONASS L1OF FDMA generation.
- Mixed RINEX 3/4 navigation ingestion with constellation-specific orbit, clock,
  navigation-message, code, modulation, visibility, Doppler, and allocation paths.
- Static receiver mode using latitude, longitude, and height.
- Dynamic receiver mode from CSV ECEF user motion files.
- Dynamic receiver mode from geodetic latitude/longitude/height CSV files.
- Dynamic receiver mode from NMEA GGA streams.
- Optional keyboard-controlled interactive motion mode.
- RINEX broadcast navigation file parsing.
- Direct POSIX streaming of plain, `.gz`, and legacy Unix-compressed `.Z` RINEX 2 files.
- Per-satellite ephemeris selection and seamless 30-second ephemeris refresh.
- Iterative signal transit-time and Earth-rotation (Sagnac) correction.
- Satellite clock bias, relativistic correction, TGD, and clock-drift modeling.
- Automatic daily GPS broadcast ephemeris download with NOAA/NGS primary and BKG IGS fallback sources when `-e` is omitted.
- Capability-driven bladeRF 1.0/2.0 adaptation for center frequency, exact sample rate, analog bandwidth, and portable overall TX gain.
- Hardware range checks and configuration read-back before RF transmission.
- Analog-filter validation against the complete occupied signal span, including an offset carrier.
- SC16 Q11 peak/rail telemetry and deterministic zero-padded final-buffer flushing.
- Signal registry and RF validation for GPS L1 C/A, Galileo E1, BeiDou B1I, and GLONASS L1OF.
- Real-time SC16 I/Q streaming to bladeRF.
- Optional XB200 setup for GPS-band transmit filtering/path selection.
- Graceful generator/TX error propagation and `SIGINT`/`SIGTERM` shutdown.
- Deterministic core tests for time, coordinates, C/A code, and ephemeris selection.
- Portable Makefile that uses `pkg-config libbladeRF` when available, with the original adjacent bladeRF source-tree fallback.

## Requirements

- C compiler with C99-compatible libc behavior.
- POSIX threads.
- libbladeRF headers and library.
- A bladeRF device supported by libbladeRF.
- `pkg-config` is recommended so the Makefile can discover libbladeRF automatically.
- `curl` and `gzip` are required only when using automatic ephemeris download.

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

## Usage

List the known signal profiles before configuring a run:

```sh
./bladegps -L
```

The status column is authoritative. Non-GPS profiles use a supplied mixed RINEX 3/4 file or auto-download a daily mixed file when `-e` is omitted.

```text
Usage: bladegps [options]
Options:
  -e <nav_file>    RINEX navigation file (daily broadcast data auto-downloads if omitted)
  -u <user_motion> User motion file (dynamic mode)
  -p <llh_motion>  Geodetic CSV motion: time,latitude,longitude,height
  -g <nmea_gga>    NMEA GGA stream (dynamic mode)
  -l <location>    Lat,Lon,Hgt (static mode) e.g. 35.274,137.014,100
  -t <date,time>   Scenario start time YYYY/MM/DD,hh:mm:ss
  -d <duration>    Duration [sec] (max: 86400)
  -x <XB number>   Enable XB board, e.g. '-x 200' for XB200
  -S <signal>      Signal profile (gps-l1ca, galileo-e1, beidou-b1i, glonass-l1of)
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
```

Static location example:

```sh
./bladegps -e brdc2940.18n -l 59.3293,18.0686,30 -d 60
```

Automatic ephemeris download example:

```sh
./bladegps -l 59.3293,18.0686,30 -d 60
```

When `-e` is omitted, GPS downloads daily RINEX 2 navigation from NOAA/NGS CORS and then BKG. Galileo, BeiDou, and GLONASS try BKG's daily `BRDC00IGS`, `BRDC00WRD`, and `BRDM00DLR` mixed-navigation products in order. The downloader uses the `-t` scenario date if provided, otherwise the current UTC date, saves the decompressed file in the working directory, and reuses it on later runs.

The downloader writes to temporary `.tmp` files first, verifies that both the compressed and decompressed files were created, then renames them into place. Failed downloads or decompression errors clean up partial output and print a manual `-e <nav_file>` fallback hint.

Galileo example using a mixed RINEX 3/4 navigation file:

```sh
./bladegps -S galileo-e1 -e BRDC00IGS_R_20262740000_01D_MN.rnx \
  -l 59.3293,18.0686,30 -d 60
```

Use `-S beidou-b1i` or `-S glonass-l1of` for those integrated backends. The
selected center frequency, sample rate, and bandwidth default to the profile's
safe values and can be overridden with `-f`, `-r`, and `-b`.

### Hardware adaptation and RF filtering

bladeGPS queries frequency, sample-rate, analog-bandwidth, and gain ranges from
the opened device instead of assuming bladeRF 1.0 limits. It sets and reads
back every timing-critical value. Exact sample rate and center frequency are
required because silent coercion would change code, symbol, and carrier timing.
Analog bandwidth may be quantized by hardware, but the realized value is
accepted only when it still contains the complete modulated signal and is no
wider than the complex sample rate.

Use `-G` for model-independent overall TX gain. This is a relative gain setting,
not calibrated RF power. `-a` and `-A` remain available together for bladeRF
1.0 systems that explicitly require the legacy `txvga1` and `txvga2` stages;
they are rejected on devices without those named stages.

With `-x 200`, the XB200 uses the native L-band bypass path and automatic
low-loss TX filter selection. The RX path is not modified. On bladeRF 2.0,
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

- `brdc*.??n`, `brdc*.??n.gz`, and `brdc*.??n.Z` files are RINEX 2 broadcast navigation files. Compressed input is streamed through `gzip` on POSIX without constructing a shell command.
- `circle.csv`, `satellite.csv`, and `ss520-4.csv` are sample motion/position data files.
- `run_bladerfGPS.sh` is a convenience script retained from the original project.

User motion CSV rows use:

```text
time_seconds,ecef_x_m,ecef_y_m,ecef_z_m
```

Records are consumed at 10 Hz. All four CSV fields must be finite numbers;
malformed records are rejected instead of silently shortening the scenario.
NMEA input accepts valid GGA fixes and ignores no-fix records.

Geodetic motion rows use decimal degrees and metres:

```text
time_seconds,latitude_degrees,longitude_degrees,height_metres
```

Latitude is limited to -90..90 degrees and longitude to -180..180 degrees. Records are converted to ECEF before signal generation.

## Signal support

| Profile | Constellation | Nominal carrier | Status | Notes |
| --- | --- | ---: | --- | --- |
| `gps-l1ca` | GPS | 1575.42 MHz | Implemented | C/A ranging code, LNAV, RINEX 2 GPS navigation, PRN 1-32 |
| `galileo-e1` | Galileo | 1575.42 MHz | Software implemented | E1-B/C codes, CBOC, mixed-RINEX orbit/clock, scheduled I/NAV ephemeris/service pages, dummy substitution for unavailable optional service words, geometry, allocation and RF synthesis |
| `beidou-b1i` | BeiDou | 1561.098 MHz | Software implemented | PRN 1–63 codes, D1/D2 selection, ephemeris/clock/ionosphere pages, geometry, NH overlay, allocation and RF synthesis |
| `glonass-l1of` | GLONASS | 1602 MHz base | Software implemented | L1OF code, FDMA slot carriers, state-vector propagation, immediate GNAV/time strings, safe unavailable-almanac marking, relative/meander modulation and RF synthesis |

Unknown or non-implemented profiles fail closed instead of silently producing a GPS waveform. Non-GPS paths consume either a supplied or automatically downloaded mixed RINEX 3/4 navigation file. See [MULTI_GNSS.md](MULTI_GNSS.md) for the exact payload boundaries and validation status.

The source tree contains tested signal primitives for all 63 BeiDou B1I ranging-code assignments, the GLONASS L1OF ranging code and FDMA carrier slots, all 50 official Galileo E1-B and E1-C primary codes, and Galileo CBOC shaping.

The typed RINEX 3/4 loader, Galileo/BeiDou Keplerian propagation, BeiDou GEO transform, GLONASS state-vector propagation, navigation scheduling, modulation, channel allocation and FIFO producer are connected to RF output. The CLI selects one signal profile per run; `gnss_rf` is constellation-neutral but the command-line producer does not yet combine GPS/Galileo/BeiDou/GLONASS profiles in one transmission. Hardware/receiver validation remains environment-dependent and is not claimed by the software tests.

## Implementation notes

- The simulator generates 0.1 second blocks at the selected sample rate for bladeRF SC16 transmission; GPS defaults to 2.6 Msps.
- The requested duration emits the complete number of 100 ms blocks.
- The closest in-fit broadcast record is selected independently per PRN and refreshed at navigation-frame boundaries.
- Non-GPS ephemeris selection compares complete GPS-aligned calendar weeks, so a record from an older week cannot win merely because it has the same seconds-of-week.
- Non-GPS 30-second navigation cycles are regenerated at every cycle boundary; Galileo GST TOW and BeiDou BDT SOW therefore advance instead of repeating a cached frame.
- Mixed RINEX files are counted and allocated dynamically; there is no fixed 4096-record truncation ceiling.
- BeiDou Klobuchar coefficients are read from RINEX 3 `BDSA`/`BDSB` headers or RINEX 4 `ION C ... D1D2` records and range-checked at their ICD scales.
- Ephemeris handovers rebuild LNAV data while preserving range-rate continuity.
- FIFO access between the GPS generation thread and TX thread is protected with the GPS mutex.
- Generation completion wakes both FIFO condition variables so shutdown and initialization failures do not deadlock waiting threads.
- Command-line path arguments are bounded to the internal `MAX_CHAR` buffers.
- Malformed NMEA GGA lines are skipped instead of crashing the parser.
- If `-e` is omitted, GPS tries NOAA/NGS and BKG RINEX 2 data; non-GPS profiles try three BKG mixed-RINEX daily products.
- Auto-downloaded ephemeris cache files are ignored by git so local runs do not dirty the repository.
- The full module map, data flow, threading model, FIFO behavior, downloader lifecycle, and extension points are documented in [ARCHITECTURE.md](ARCHITECTURE.md).

## Project documentation

- [ARCHITECTURE.md](ARCHITECTURE.md) - maintainer architecture, runtime flow, modules, and extension points.
- [GPS_L1_CA_COVERAGE.md](GPS_L1_CA_COVERAGE.md) - implemented L1 C/A coverage, wired gaps, and non-certified areas.
- [MULTI_GNSS.md](MULTI_GNSS.md) - constellation architecture, current capability matrix, and acceptance gates.
- [CHANGELOG.md](CHANGELOG.md) - release history.
- [LICENSE](LICENSE) - MIT license.

## Release history

See [CHANGELOG.md](CHANGELOG.md).

## Safety

This software can generate RF signals in protected satellite navigation bands. Never connect a bladeRF running this program to an antenna in an unshielded environment unless you have explicit legal authority. For receiver testing, use a shielded enclosure, direct cable injection, attenuation, and isolation appropriate for your equipment.

## License

Copyright (c) 2015 Takuji Ebinuma.

Distributed under the MIT License. See [LICENSE](LICENSE).
