# bladeGPS

Real-time GPS L1 C/A signal generation for bladeRF, based on the GPS signal model from [gps-sdr-sim](https://github.com/osqzss/gps-sdr-sim). bladeGPS reads broadcast ephemeris data and a static or dynamic receiver path, generates synthetic GPS baseband I/Q samples, and streams them directly to a Nuand bladeRF transmitter.

This is research and lab software. Only transmit GPS-like RF signals inside a properly shielded test setup, with appropriate attenuation, and only where you are legally allowed to do so.

## Features

- GPS L1 C/A baseband generation with up to 16 simulated channels.
- Static receiver mode using latitude, longitude, and height.
- Dynamic receiver mode from CSV ECEF user motion files.
- Dynamic receiver mode from NMEA GGA streams.
- Optional keyboard-controlled interactive motion mode.
- RINEX broadcast navigation file parsing.
- Automatic daily GPS broadcast ephemeris download when `-e` is omitted.
- Real-time SC16 I/Q streaming to bladeRF.
- Optional XB200 setup for GPS-band transmit filtering/path selection.
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

```text
Usage: bladegps [options]
Options:
  -e <gps_nav>     RINEX navigation file for GPS ephemerides (auto-downloads if omitted)
  -u <user_motion> User motion file (dynamic mode)
  -g <nmea_gga>    NMEA GGA stream (dynamic mode)
  -l <location>    Lat,Lon,Hgt (static mode) e.g. 35.274,137.014,100
  -t <date,time>   Scenario start time YYYY/MM/DD,hh:mm:ss
  -d <duration>    Duration [sec] (max: 86400)
  -x <XB number>   Enable XB board, e.g. '-x 200' for XB200
  -i               Interactive mode: North='w', South='s', East='d', West='a'
```

Static location example:

```sh
./bladegps -e brdc2940.18n -l 59.3293,18.0686,30 -d 60
```

Automatic ephemeris download example:

```sh
./bladegps -l 59.3293,18.0686,30 -d 60
```

When `-e` is omitted, bladeGPS downloads the daily GPS broadcast ephemeris from NOAA/NGS CORS using the `-t` scenario date if provided, otherwise the current UTC date. The downloaded file is saved as `brdcDDD0.YYn` in the working directory and reused on later runs.

The downloader writes to temporary `.tmp` files first, verifies that both the compressed and decompressed files were created, then renames them into place. Failed downloads or decompression errors clean up partial output and print a manual `-e <gps_nav>` fallback hint.

User motion CSV example:

```sh
./bladegps -e brdc2940.18n -u circle.csv -d 120
```

NMEA GGA example:

```sh
./bladegps -e brdc2940.18n -g track.nmea -d 120
```

XB200 example:

```sh
./bladegps -e brdc2940.18n -l 35.274,137.014,100 -x 200 -d 60
```

## Input files

- `brdc*.??n` files are RINEX broadcast navigation files.
- `circle.csv`, `satellite.csv`, and `ss520-4.csv` are sample motion/position data files.
- `run_bladerfGPS.sh` is a convenience script retained from the original project.

User motion CSV rows use:

```text
time_seconds,ecef_x_m,ecef_y_m,ecef_z_m
```

## Implementation notes

- The simulator generates 0.1 second blocks at 2.6 Msps for bladeRF SC16 transmission.
- FIFO access between the GPS generation thread and TX thread is protected with the GPS mutex.
- Generation completion wakes both FIFO condition variables so shutdown and initialization failures do not deadlock waiting threads.
- Command-line path arguments are bounded to the internal `MAX_CHAR` buffers.
- Malformed NMEA GGA lines are skipped instead of crashing the parser.
- If `-e` is omitted, the downloader fetches NOAA/NGS CORS RINEX v2 daily GPS navigation data from `https://geodesy.noaa.gov/corsdata/rinex/YYYY/DDD/brdcDDD0.YYn.gz`.
- Auto-downloaded ephemeris cache files are ignored by git so local runs do not dirty the repository.
- The full module map, data flow, threading model, FIFO behavior, downloader lifecycle, and extension points are documented in [ARCHITECTURE.md](ARCHITECTURE.md).

## Project documentation

- [ARCHITECTURE.md](ARCHITECTURE.md) - maintainer architecture, runtime flow, modules, and extension points.
- [CHANGELOG.md](CHANGELOG.md) - release history.
- [LICENSE](LICENSE) - MIT license.

## Release history

See [CHANGELOG.md](CHANGELOG.md).

## Safety

This software can generate RF signals in protected satellite navigation bands. Never connect a bladeRF running this program to an antenna in an unshielded environment unless you have explicit legal authority. For receiver testing, use a shielded enclosure, direct cable injection, attenuation, and isolation appropriate for your equipment.

## License

Copyright (c) 2015 Takuji Ebinuma.

Distributed under the MIT License. See [LICENSE](LICENSE).
