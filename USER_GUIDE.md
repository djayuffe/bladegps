# bladeGPS User Guide

This guide covers installation, navigation data, receiver scenarios, bladeRF
configuration, profile selection, real-time operation, validation, and common
errors. `CLI_REFERENCE.md` is the exact option contract; `SUPPORT_MATRIX.md`
states implementation boundaries.

## Safety first

GNSS bands are protected. Use bladeGPS only in a shielded enclosure or through
a conducted cable path with suitable attenuation and isolation. Never connect
the transmitter to an antenna unless you have explicit legal authority. Start
with the lowest useful gain and verify the setup with a spectrum analyzer or
power meter before connecting a receiver.

## Installation

Required components:

- a C compiler and POSIX threads;
- libbladeRF headers, library, firmware, and an FPGA image compatible with the
  connected board;
- `pkg-config` (recommended);
- `curl` and `gzip` when navigation data is downloaded automatically;
- SDL2 only when `-j` controller input is required.

Build and test:

```sh
make
make check
```

If libbladeRF is not visible through `pkg-config`, provide its flags directly:

```sh
make BLADERF_CFLAGS="-I/path/to/include" \
  BLADERF_LIBS="-L/path/to/lib -lbladeRF"
```

Confirm that libbladeRF can see the device before transmitting:

```sh
bladeRF-cli -p
./bladegps -L
```

`-L` is software-only. It lists carrier, minimum waveform span, default sample
rate, default analog filter, and implementation status without opening hardware.

## First controlled run

This GPS example downloads the daily navigation file, simulates a stationary
receiver for 30 seconds, and uses the profile-safe RF defaults:

```sh
./bladegps -S gps-l1ca -l 59.3293,18.0686,30 -d 30
```

To select a particular bladeRF:

```sh
./bladegps -D 'serial=YOUR_SERIAL' -S gps-l1ca \
  -l 59.3293,18.0686,30 -d 30
```

At startup bladeGPS reports the board, FPGA state, TX channel count, USB speed,
realized center frequency, sample rate, analog bandwidth, filter margin, and
gain. Transmission starts only after the GNSS producer has generated valid I/Q
and every hardware read-back passes validation.

## Signal profiles and RF defaults

| Profile | Plan center | Minimum waveform span | Default sample rate | Default TX filter | Rationale |
| --- | ---: | ---: | ---: | ---: | --- |
| `gps-l1ca` | 1575.420 MHz | 2.2506 MHz | 2.600 Msps | 2.500 MHz | BPSK(1) main-lobe engineering span plus filter/Doppler margin. |
| `galileo-e1` | 1575.420 MHz | 24.552 MHz | 36.828 Msps | 28.000 MHz | Preserves the Galileo E1 reference bandwidth and CBOC(6,1), with guard margin. |
| `beidou-b1i` | 1561.098 MHz | 4.5012 MHz | 5.000 Msps | 5.000 MHz | BPSK at 2.046 Mcps with symmetric guard margin. |
| `glonass-l1of` | 1602.000 MHz | 8.9992 MHz | 12.000 Msps | 10.000 MHz | Contains slots `k=-7…+6`, the L1OF spectrum, and Doppler margin. |
| `mixed-open` | 1582.3925 MHz | 47.100 MHz | 50.000 Msps | 48.000 MHz | Contains BeiDou through the upper GLONASS slot while retaining digital/filter guard. |

The “minimum waveform span” is used for passband and allocator decisions. The
default TX filter is deliberately a separate, equal-or-wider value. This avoids
using guard bandwidth as if it were emitted waveform occupancy and avoids
placing Galileo’s reference-bandwidth edge directly on the analog filter edge.

For a signal span centered away from the bladeRF tuning center:

```text
minimum_RF_span = waveform_span + 2 * abs(signal_center - TX_center)
```

bladeGPS requires:

```text
minimum_RF_span <= realized_analog_bandwidth <= realized_sample_rate
minimum_RF_span <= realized_sample_rate
```

libbladeRF devices expose discrete or quantized filter settings. bladeGPS
accepts a realized setting different from `-b` only when `bladerf_get_bandwidth`
confirms the value and the complete RF plan still fits. Center frequency and
sample rate must read back exactly because they define the RF placement and all
code/symbol timing.

Do not reduce `-b` below the profile span. Increasing it unnecessarily admits
more images/noise and can violate the sample-rate anti-alias boundary. Use the
defaults unless a measured, device-specific plan requires an override.

## Hardware behavior

bladeGPS configures TX channel 0 only and uses `BLADERF_TX_X1` with SC16 Q11.
It does not configure RX, ADC, bias tee, clock input, or triggers. At startup it:

1. verifies that the FPGA is configured and at least one TX channel exists;
2. reports the USB link speed;
3. disables and verifies loopback so samples reach the RF output;
4. optionally attaches and verifies XB200 bypass/automatic 1 dB filtering;
5. checks frequency, sample-rate, bandwidth, and gain ranges after tuning;
6. sets and reads back the center frequency and sample rate exactly;
7. sets the analog filter, reads it back, and validates the realized passband;
8. sets overall gain or both requested legacy bladeRF 1 TXVGA stages;
9. configures 32 timestamped synchronous buffers of 32768 samples with 16 USB transfers;
10. enables TX only after the synchronous stream is configured;
11. schedules one continuous burst 100 ms ahead normally, or maps the five-second `-R` wall-clock target to the corresponding future FPGA TX timestamp;
12. marks the final buffer as burst end and waits for the FPGA clock to pass
    every submitted sample before disabling TX.

For SC16 Q11, USB 2.0 High-Speed has an absolute nominal payload ceiling of
15 Msps before protocol overhead. Nuand's
[sample-rate troubleshooting guide](https://github.com/Nuand/bladeRF/wiki/Debugging-dropped-samples-and-identifying-achievable-sample-rates)
reports roughly 5–8 Msps in typical USB 2.0 operation. bladeGPS rejects rates
at or above the nominal ceiling and warns above 5 Msps, where sustained
throughput becomes strongly host/controller dependent. Wideband Galileo and
mixed profiles therefore require USB SuperSpeed; a successful RF sample-rate
setting alone is not a transport-throughput guarantee.

The stream submits complete 32768-sample transfers. A final partial scenario
transfer is zero-padded and explicitly marked as the end of the timestamped
burst. Shutdown reports the digital peak, Q11 rail contacts, padding, scheduled
RF duration, host elapsed time, and whether the FPGA timeline fully drained.

### Gain

`-G` uses libbladeRF’s portable overall TX gain. The number is relative and is
not calibrated RF output power. Hardware may realize a stepped value, which is
reported. Start low and increase only with conducted measurements.

`-a` and `-A` select legacy `txvga1`/`txvga2`; both are required and they cannot
be combined with `-G`. These named stages are intended for compatible bladeRF 1
devices and fail on hardware that does not expose them.

### XB200

`-x 200` attaches XB200, selects the native L-band bypass path, and uses its
automatic 1 dB filter selection. Both path and filter are read back. The XB200
mixer path is inappropriate for native GNSS L-band output. Do not specify
`-x 200` when no physically attached XB200 is present.

### Wideband profiles

`mixed-open` requires a realized 48 MHz analog filter and 50 Msps SC16 stream.
This exceeds the normal instantaneous bandwidth of some bladeRF generations or
USB connections. Capability checks intentionally reject such hardware; select
one narrower profile instead. A successful API configuration is not proof of
an underrun-free host: use a SuperSpeed connection, watch libbladeRF output,
and confirm the producer keeps the FIFO supplied. Although the carrier mixer
avoids per-sample trigonometry, 50 Msps multi-channel synthesis can still exceed
the real-time CPU budget of slower hosts.

## Navigation data

Use `-e FILE` for an existing navigation file. Supported production inputs are:

- GPS RINEX 2 broadcast navigation;
- mixed RINEX 3 navigation containing the selected record family; RINEX 3.05
  optional GLONASS Orbit-4 is parsed, but complete GNAV still requires its
  fields to be populated rather than blank/sentinel;
- mixed RINEX 4 `EPH` records for GPS LNAV, Galileo INAV, BeiDou D1/D2, or
  GLONASS FDMA;
- plain files and, on POSIX, directly streamed `.gz` or legacy `.Z` files.

When `-e` is omitted, GPS `-t` is converted to a UTC download date; otherwise
today’s UTC date is used and live mode is enabled. Downloads and caches are
parsed and checked for the selected family, health, age, and complete required
fields before use. `-R` requests the same synchronized live start with `-e`.

Examples:

```sh
./bladegps -S galileo-e1 -e mixed.rnx \
  -l 59.3293,18.0686,30 -M 10 -d 60

./bladegps -S mixed-open -e mixed.rnx \
  -l 59.3293,18.0686,30 -d 60
```

The scenario time must overlap healthy, in-age ephemerides. Supplying a file
from another day without `-t` commonly produces “no healthy in-age record.”

## Receiver position and motion

Exactly one base position source is active; when several are supplied, the last
of `-l`, `-u`, `-p`, or `-g` wins.

Static geodetic position:

```sh
./bladegps -S gps-l1ca -e nav.rnx -l 59.3293,18.0686,30 -d 60
```

ECEF CSV (`time_seconds,x_m,y_m,z_m`):

```sh
./bladegps -S gps-l1ca -e nav.rnx -u route-ecef.csv -d 120
```

Geodetic CSV (`time_seconds,latitude_degrees,longitude_degrees,height_metres`):

```sh
./bladegps -S gps-l1ca -e nav.rnx -p route-llh.csv -d 120
```

NMEA GGA:

```sh
./bladegps -S gps-l1ca -e nav.rnx -g track.nmea -d 120
```

CSV and GGA tracks are validated and resampled to 10 Hz. Timestamps must move
forward. GGA checksum, time, coordinate, hemisphere, unit, and fix fields are
validated; midnight is unwrapped across multiple days.

`-i` adds keyboard velocity to the base route: `w/s` north/south, `a/d`
west/east, and `e/q` up/down. `-j INDEX` adds an SDL controller: left stick is
north/east and the right vertical axis is up/down. Live input is accumulated as
a local-frame offset; it does not erase prerecorded motion.

## Complete option summary

| Option | Purpose |
| --- | --- |
| `-S PROFILE` | Select an advertised signal profile. |
| `-L` | List profiles and exit without opening hardware. |
| `-e FILE` | Use a navigation file instead of automatic download. |
| `-t YYYY/MM/DD,hh:mm:ss` | Set scenario and download date/time. |
| `-R` | Use synchronized host UTC and align RF sample zero to a five-second future wall-clock epoch; cannot be combined with `-t`. |
| `-d SECONDS` | Set duration, rounded to 100 ms; maximum 86400 seconds. |
| `-l LAT,LON,HGT` | Use a static receiver position in degrees/metres. |
| `-u FILE` | Use timed ECEF CSV motion. |
| `-p FILE` | Use timed geodetic CSV motion. |
| `-g FILE` | Use an NMEA GGA track. |
| `-i` | Add keyboard motion. |
| `-j INDEX` | Add SDL controller motion. |
| `-D SELECTOR` | Pass a libbladeRF device selector to `bladerf_open`. |
| `-f HZ` | Override TX center frequency. |
| `-r SPS` | Override sample rate; must be at least 1 Msps and divisible by 10. |
| `-b HZ` | Override requested analog TX bandwidth. |
| `-G DB` | Set portable overall TX gain. |
| `-a DB -A DB` | Set both legacy bladeRF 1 TXVGA stages. |
| `-M DEGREES` | Set elevation mask from -90 through 90 degrees. |
| `-x 0|200` | Disable expansion handling or configure XB200. |

## Shutdown and exit status

`Ctrl+C`, `SIGINT`, and `SIGTERM` request an orderly block-boundary stop. The
producer and transmitter wake each other, buffered samples drain, TX is
disabled, and the device closes. User interruption returns 130. Invalid input,
hardware setup failure, generator failure, or streaming failure returns nonzero.

## Troubleshooting

### No bladeRF device

Run `bladeRF-cli -p`, verify permissions/driver access, and use `-D` when more
than one device is present.

### FPGA is not configured

Install/load the FPGA image recommended for the board and libbladeRF version.
bladeGPS does not silently continue without FPGA support.

### Exact sample rate unavailable

Choose a device-supported rate that is divisible by 10 and still exceeds the
minimum waveform span. Changing sample rate changes every digital clock, so
bladeGPS rejects silent coercion.

### Analog bandwidth does not contain the signal

Use the profile default, raise `-b`, restore the default center, or select a
narrower profile. The reported minimum span and realized filter margin show the
failed geometry. The analog filter must not exceed the sample rate.

### Selected center/sample rate does not contain the signal

An `-f` override moved the waveform too close to a Nyquist edge, or `-r` is too
low. Apply the RF-plan equation above to both edges.

### Mixed mode is rejected

The device, analog filter, or transport cannot provide 50 Msps and 48 MHz. Use
an individual signal profile or suitable wideband hardware.

### No healthy in-age navigation record

Match `-t` to the navigation file date, verify that the file contains the
selected constellation/message type, and check health fields.

### TX timeout or underrun symptoms

Use USB SuperSpeed, avoid hubs, close competing streams, reduce host load, and
select a narrower profile. Timestamped continuous-burst scheduling prevents
host queue completion from being mistaken for RF completion, but the installed
libbladeRF API does not report TX underrun status. Independent receiver or
spectrum validation therefore remains necessary.

### Digital rail warning

The generated channel bank reached SC16 Q11 `-2048..2047`. This is digital
clipping, independent of RF gain. Report it with the scenario and navigation
file; reducing `-G` does not repair already clipped samples.

## Acceptance testing

`make check` validates software logic without radiating RF. Release acceptance
for a specific board should additionally include conducted measurements of:

- realized center, sample rate, occupied spectrum, and image rejection;
- absence of host-stream discontinuities;
- digital and analog clipping;
- receiver acquisition, navigation decoding, and PVT behavior;
- attenuation and leakage from the shielded setup.

The repository does not claim calibrated RF power, regulatory certification,
or independent receiver conformance for every generated service.
