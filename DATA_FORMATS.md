# Input and Data-Format Reference

bladeGPS consumes broadcast navigation plus one receiver-position source. This
document describes accepted representations and validation behavior. All text
paths stored in runtime options are limited to 99 characters plus terminator.

## Broadcast navigation

### GPS RINEX 2

The shared loader accepts RINEX 2 GPS navigation records and maps the 28
broadcast-orbit fields into `gnss_nav_record_t.orbit[]` in specification order.
Two-digit years 80–99 map to 1980–1999; 00–79 map to 2000–2079. Records become
typed GPS `LNAV`/Keplerian records.

### RINEX 3

System-prefixed records are supported for:

| Prefix | System | Inferred/accepted message |
| --- | --- | --- |
| `G` | GPS | LNAV |
| `E` | Galileo | INAV or FNAV inferred from data-source bits; E1 production uses INAV |
| `C` | BeiDou | D2 for GEO PRNs 1–5, otherwise D1 |
| `R` | GLONASS | FDMA |

### RINEX 4

Typed `> EPH` records are accepted for `G/LNAV`, `E/INAV`, `E/FNAV`, `C/D1`,
`C/D2`, and `R/FDMA`. Unsupported record types are skipped to the next `>`
boundary. E1 generation consumes INAV, not FNAV.

BeiDou ionosphere coefficients are read from RINEX 3 `BDSA`/`BDSB` header
records or supported RINEX 4 `ION C ...` records. Missing coefficients are
permitted and leave the encoded coefficients at their explicit default; a
malformed matching record fails.

### Compression

On POSIX, plain files and paths ending in `.gz` or `.Z` are accepted by the
typed loader. Compressed streams are produced by `fork`/`exec` of
`gzip -cd -- path`; the file name is passed as an argument, not shell text.
The child exit status is checked. Windows requires decompressed input.

Automatic downloads are fetched to `.tmp`, decompressed, checked, and renamed
atomically into the working-directory cache.

Live orbit selection uses `toe`, not `toc`, for GPS, Galileo, and BeiDou.
The seconds-of-week difference is evaluated in the constellation's native
GPS/GST/BDT scale with week rollover. GLONASS state-vector age is measured from
its UTC(SU) calendar epoch. GPS records use the advertised fit interval when
present; malformed, future-by-more-than-30-seconds, stale, or unhealthy records
are excluded independently for every PRN.

## Typed navigation record

`gnss_nav_record_t` contains:

| Field | Meaning |
| --- | --- |
| `system` | GPS, Galileo, BeiDou, or GLONASS. |
| `prn` | System satellite/slot identifier from the RINEX record. |
| `message[8]` | `LNAV`, `INAV`, `FNAV`, `D1`, `D2`, or `FDMA`. |
| `toc` | Calendar clock epoch expressed in the constellation's RINEX time scale. |
| `model` | Keplerian or GLONASS state-vector propagation. |
| `clock_bias` | Broadcast clock offset coefficient in seconds. |
| `clock_drift` | First clock coefficient in seconds/second. |
| `clock_drift_rate` | Second clock coefficient in seconds/second². |
| `orbit[32]` | Fixed-order RINEX broadcast fields; blank fields are `NAN`. |
| `orbit_count` | Number of semantically present orbit fields. |

The parser rejects invalid calendars, non-finite numeric text, capacity
overflow, truncated continuation blocks, and malformed supported records.

## Static receiver

`-l` uses:

```text
latitude_degrees,longitude_degrees,height_metres
```

Latitude is -90…90 and longitude -180…180. Height must parse as a finite
floating-point number. Coordinates are converted to WGS-84 ECEF internally.

## ECEF motion CSV

`-u` rows use:

```text
time_seconds,ecef_x_metres,ecef_y_metres,ecef_z_metres
```

Time must be finite and strictly increasing. Position fields must be finite.
Every row must contain exactly four comma-separated numeric fields; trailing
data and truncated overlong lines are rejected. The series is linearly
resampled onto the 10 Hz simulation clock. Receiver velocity is the ECEF
difference between consecutive 100 ms output samples.

## Geodetic motion CSV

`-p` rows use:

```text
time_seconds,latitude_degrees,longitude_degrees,height_metres
```

Latitude/longitude ranges match `-l`. Valid rows are converted to ECEF, then
linearly resampled at 10 Hz. The same exact four-field and line-length rules as
ECEF motion apply.

## NMEA GGA

`-g` consumes GGA fixes. Supported behavior:

- Sentences with a checksum must pass it.
- No-fix records are ignored.
- UTC `hhmmss.s` timestamps are validated and cumulatively unwrapped across
  any number of midnight transitions.
- Numeric tokens must parse completely and remain finite; malformed records are
  skipped without shifting later fields.
- Latitude/longitude degree-minute ranges and `N/S/E/W` hemispheres are
  validated, and altitude plus geoid separation must use metre (`M`) units.
- Valid latitude/longitude and ellipsoidal height are converted to ECEF.
- Accepted fixes are linearly resampled at 10 Hz.

A file with no usable positions fails generation rather than falling back to a
different location.

## Live motion

Keyboard and controller inputs are north/east/up velocities in the receiver's
current local tangent frame. The frame is recomputed as position changes. Live
displacement is accumulated as an offset on the selected static, CSV, or NMEA
base trajectory, so enabling a controller does not freeze prerecorded motion.

- Keyboard speed changes by `DEL_VEL = 0.1 m/s` per update and is capped at
  `MAX_VEL = 1.4 m/s`. A new direction begins at one increment on its first
  key event; updates without a new event decelerate toward zero.
- SDL2 axes use a raw dead zone of 4096, remap the remaining range to 0…1, and
  normalize horizontal diagonals so their magnitude never exceeds one.
- Controller horizontal and vertical maxima are both 1.4 m/s in production.

## Generated sample stream

Samples are interleaved signed `int16_t` values:

```text
I0, Q0, I1, Q1, ...
```

The bladeRF format is `BLADERF_FORMAT_SC16_Q11`, nominal component range
-2048…2047. The renderer applies deterministic channel-bank normalization with
headroom before final saturation. One generation block is exactly
`sample_rate / 10` complex samples (100 ms). The FIFO stores two such blocks in
complex-sample units, and the TX thread submits 32768 complex samples per
synchronous transfer.
