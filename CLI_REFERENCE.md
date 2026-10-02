# Command-Line Reference

This document defines every production `bladegps` option, its accepted values,
default behavior, interactions, and validation. Values shown here follow the
actual parser in `bladegps.c`; connected bladeRF hardware can impose narrower
limits than the parser permits.

## Synopsis

```text
bladegps [options]
```

At least one argument is required. `-L` lists signal profiles and exits without
opening hardware. All normal runs require a usable navigation file, obtained
from `-e` or automatic download.

## Options

| Option | Argument | Accepted values | Default | Effect and interactions |
| --- | --- | --- | --- | --- |
| `-S` | profile | `gps-l1ca`, `galileo-e1`, `beidou-b1i`, `glonass-l1of`, `mixed-open` | `gps-l1ca` | Selects signal metadata, parser routing, navigation backend, modulation, and RF defaults. Unknown/unimplemented profiles fail before hardware opens. |
| `-L` | none | — | off | Prints each profile's description, carrier, minimum waveform span, default sample rate, default analog filter, and implementation state, then exits successfully. Other run options are ignored because no scenario starts. |
| `-e` | path | Existing plain RINEX; POSIX also accepts `.gz`/`.Z` | automatic download | Navigation path, maximum 99 characters. RINEX family must match the chosen service. |
| `-t` | date,time | `YYYY/MM/DD,hh:mm:ss`, year > 1980, valid date, hour 0–23, minute 0–59, second `[0,60)` | first matching ephemeris epoch; current UTC date for download | Sets scenario time and automatic-download date. Fractional seconds are accepted syntactically and floored to an integer second. |
| `-d` | seconds | finite decimal, `0 < d <= 86400` | 86400 seconds | Rounded to the nearest 100 ms step as `floor(d*10 + 0.5)`. |
| `-l` | lat,lon,height | latitude `[-90,90]` degrees, longitude `[-180,180]` degrees, finite numeric height in metres | `35.274016,137.013765,100` | Static receiver location. Selecting it overrides prior file-motion mode flags. |
| `-u` | path | ECEF motion CSV | none | Selects dynamic `time,x,y,z` ECEF motion and disables static/NMEA/geodetic modes. Path limit is 99 characters. |
| `-p` | path | Geodetic motion CSV | none | Selects dynamic `time,latitude,longitude,height` motion and disables static/NMEA/ECEF modes. |
| `-g` | path | NMEA GGA text stream | none | Selects checksummed/timestamped GGA motion and disables static/CSV modes. |
| `-i` | none | — | off | Enables keyboard velocity commands in addition to the current position source. Keys: `w/s` north/south, `a/d` west/east, `e/q` up/down. |
| `-j` | index | integer 0–255 | disabled (`-1`) | Opens an SDL2 game-controller index. Fails if SDL2 support is absent, the index is invalid, or the controller disconnects. |
| `-D` | identifier | libbladeRF device selector string | first matching device | Passed to `bladerf_open`; maximum 99 characters. |
| `-f` | hertz | unsigned integer 1–`UINT_MAX` | profile carrier/plan center | Requested TX complex center frequency. Must contain the complete profile span at the chosen sample rate. Hardware must reproduce it exactly. |
| `-r` | samples/s | integer 1,000,000–100,000,000 and divisible by 10 | profile minimum | Exact TX complex sample rate. Divisibility by 10 guarantees an integral 100 ms producer block. Hardware coercion is rejected. |
| `-b` | hertz | unsigned integer 1–`UINT_MAX` | profile recommendation | Requested analog TX filter bandwidth. It must contain the complete offset waveform span and may not exceed sample rate. Hardware quantization is accepted only if set-result and read-back agree and the realized value remains safe. |
| `-G` | dB | integer -200–200, then constrained by device | 27 dB | Portable overall TX gain. Mutually exclusive with `-a`/`-A`; it is relative gain, not calibrated RF power. |
| `-a` | dB | integer -100–100, then constrained by stage | -25 dB | Legacy bladeRF 1 `txvga1`. Must be paired with `-A`; cannot be combined with `-G`. |
| `-A` | dB | integer -100–100, then constrained by stage | 0 dB | Legacy bladeRF 1 `txvga2`. Must be paired with `-a`; cannot be combined with `-G`. |
| `-M` | degrees | finite decimal `[-90,90]` | 0 degrees | Satellite elevation mask used before channel allocation. Negative masks are useful only for controlled tests. |
| `-x` | board | `0` or `200` | 0 | `200` attaches XB200, keeps the native L-band TX bypass, and selects automatic 1 dB filtering. RX state is untouched. |

When multiple receiver-position selectors appear, normal option order applies:
the last of `-l`, `-u`, `-p`, or `-g` establishes the base mode. `-i` and `-j`
can add live movement to that base position.

## Profile defaults

| Profile | Center frequency | Minimum waveform span | Sample rate | Analog filter | Maximum advertised SV number | FDMA |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| `gps-l1ca` | 1,575,420,000 Hz | 2,250,600 Hz | 2,600,000 sps | 2,500,000 Hz | 37 | No |
| `galileo-e1` | 1,575,420,000 Hz | 24,552,000 Hz | 36,828,000 sps | 28,000,000 Hz | 36 | No |
| `beidou-b1i` | 1,561,098,000 Hz | 4,501,200 Hz | 5,000,000 sps | 5,000,000 Hz | 63 | No |
| `glonass-l1of` | 1,602,000,000 Hz nominal | 8,999,200 Hz | 12,000,000 sps | 10,000,000 Hz | 24 | Slots -7…+6 |
| `mixed-open` | 1,582,392,500 Hz | 47,100,000 Hz | 50,000,000 sps | 48,000,000 Hz | per service | Yes |

CLI overrides do not change the simulated signal's defined carrier or minimum
waveform span. They change the SDR tuning/filter plan and must still contain the
registered signal. The default analog filter is deliberately wider than the
minimum waveform span to allow Doppler, transition-band, and device-quantization
margin without making the allocator treat that guard as emitted occupancy.

## RF-plan equations

For one registered carrier, required complex/analog span is:

```text
required_bandwidth = minimum_waveform_span
                   + 2 * abs(signal_carrier - tx_center)
```

The plan is accepted only when:

```text
required_bandwidth <= sample_rate
required_bandwidth <= analog_bandwidth
analog_bandwidth   <= sample_rate
```

Per-satellite GLONASS FDMA carriers receive an additional allocator passband
check. The hardware layer then repeats validation against realized center,
sample-rate, and filter values. Frequency and sample rate must read back exactly;
filter quantization is accepted only when the returned and queried values agree
and remain large enough.

## Runtime and exit behavior

- Missing or malformed navigation, no healthy in-age satellite, invalid motion,
  device configuration failure, and TX failure produce a non-zero exit.
- `SIGINT`/`SIGTERM` request block-boundary shutdown, drain buffered output, and
  return interrupt status 130.
- Initialization broadcasts both FIFO conditions, including error paths, so the
  main/TX threads cannot wait forever.
- A completed final partial libbladeRF transfer is zero-padded; generated
  scenario samples are not discarded.

## Examples

```sh
# List profiles without touching RF hardware
./bladegps -L

# Static GPS L1 C/A using automatic ephemeris
./bladegps -S gps-l1ca -l 59.3293,18.0686,30 -d 60

# Galileo E1 with explicit mixed RINEX and elevation mask
./bladegps -S galileo-e1 -e BRDC00IGS_R_20262740000_01D_MN.rnx \
  -l 59.3293,18.0686,30 -M 10 -d 60

# Mixed open services; hardware must support the full instantaneous span
./bladegps -S mixed-open -e BRDC00IGS_R_20262740000_01D_MN.rnx \
  -l 59.3293,18.0686,30 -d 60

# Timestamped geodetic motion plus a live controller
./bladegps -S gps-l1ca -p route-llh.csv -j 0 -d 120
```
