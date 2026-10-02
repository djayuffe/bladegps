# bladeGPS Support Matrix

This document is the detailed capability contract for bladeGPS. It separates
software implementation from standards conformance and physical RF validation.
An `implemented` entry means that an executable code path exists and is covered
by repository tests; it does not mean that the signal has been certified by a
GNSS authority or measured with calibrated RF equipment.

## Status vocabulary

| Status | Meaning |
| --- | --- |
| Implemented | Connected end to end in the production CLI and exercised by automated software tests. |
| Partial payload | The RF waveform is generated, but optional navigation data unavailable from the input is encoded as an ICD-defined dummy, reserved, or explicitly unavailable field. |
| Parsed only | Input records are recognized, but they do not drive an advertised RF profile. |
| Software validated | Deterministic unit, integration, sanitizer, or software-loopback checks exist. |
| RF validation required | A shielded or conducted hardware/receiver test is still required. |
| Unsupported | No production waveform is emitted; selection fails closed. |

## Executable signal profiles

| CLI profile | Service | Carrier plan | Primary code | Navigation rate | Waveform span | Default sample/filter | Production status |
| --- | --- | ---: | --- | ---: | ---: | ---: | --- |
| `gps-l1ca` | GPS L1 C/A + LNAV | 1575.42 MHz CDMA | 1023 chips at 1.023 Mcps | 50 bit/s | 2.2506 MHz | 2.6 Msps / 2.5 MHz | Implemented |
| `galileo-e1` | Galileo E1-B data + E1-C pilot, I/NAV | 1575.42 MHz CDMA | 4092 chips at 1.023 Mcps | 250 symbol/s after coding | 24.552 MHz reference | 36.828 Msps / 28 MHz | Implemented; optional I/NAV content is partial |
| `beidou-b1i` | BeiDou B1I D1/D2 | 1561.098 MHz CDMA | 2046 chips at 2.046 Mcps | D1 50 bit/s; D2 500 bit/s | 4.5012 MHz | 5.0 Msps / 5.0 MHz | Implemented; optional service/almanac content is partial |
| `glonass-l1of` | GLONASS L1OF + GNAV | 1602 MHz + `k × 562.5 kHz`, `k=-7…+6` | 511 chips at 0.511 Mcps | 50 bit/s before 100 Hz meander | 8.9992 MHz full plan | 12.0 Msps / 10.0 MHz | Implemented; unavailable almanac is marked non-operational |
| `mixed-open` | All four services above | 1582.3925 MHz plan center | Per-service | Per-service | 47.1 MHz | 50.0 Msps / 48.0 MHz | Implemented when hardware covers the complete span |

The runtime has sixteen RF channel slots shared by all visible satellites. In
`mixed-open`, allocation is global rather than four independent per-system
allocations. Healthy in-band candidates above the elevation mask are ordered by
elevation, and the best sixteen are retained.

## Satellite, code, and channel coverage

| Service | Assignment coverage | Runtime limit | Notes |
| --- | --- | ---: | --- |
| GPS L1 C/A | GPS PRN 1–37 | 37 | G1/G2 delay assignments; SBAS PRN 120–158 are not generated. |
| Galileo E1 | E1-B and E1-C memory codes 1–50 are present | 36 | The production profile limits active Galileo navigation satellites to 36. |
| BeiDou B1I | PRN 1–63 | 63 | D2 is selected for GEO records and D1 for other supported records. |
| GLONASS L1OF | Shared ranging code | 24 SV records | FDMA slot validation accepts channel numbers -7 through +6. |
| Mixed runtime | Union of the preceding sets | 16 active channels | Per-channel carrier, code, data, overlay, and Doppler state remains independent. |

## End-to-end implementation layers

| Layer | GPS L1 C/A | Galileo E1 OS | BeiDou B1I | GLONASS L1OF | Mixed runtime |
| --- | --- | --- | --- | --- | --- |
| CLI profile and safe RF defaults | Implemented | Implemented | Implemented | Implemented | Implemented |
| Broadcast record ingestion | GPS RINEX 2; typed RINEX 3/4 LNAV | Typed RINEX 3/4 INAV; FNAV recognized but not scheduled on E1 | Typed RINEX 3/4 D1/D2 | Typed RINEX 3/4 FDMA; complete GNAV generation requires the four extended RINEX 4 fields | One typed mixed file |
| Health and age filtering | Implemented | E1-specific packed health interpretation | Implemented | Implemented | Per-system, before joint allocation |
| Orbit and satellite clock | Kepler + clock + relativity | Kepler + clock + relativity | MEO/IGSO Kepler and GEO transform | RK4 state-vector propagation with J2/Earth rotation | Native model per record |
| Iterative transmit-time solution | Implemented | Implemented | Implemented | Implemented | Implemented |
| Sagnac/Earth-rotation correction | Implemented | Implemented | Implemented | Implemented | Implemented |
| Group-delay application | GPS TGD | Galileo BGD for E1 | BeiDou TGD | Not applicable in the same broadcast form | Per service |
| Range rate and Doppler | Implemented | Implemented | Implemented | Implemented | Per channel |
| Primary code | C/A | E1-B/E1-C | B1I | L1OF | Per service |
| Navigation payload | LNAV | I/NAV | D1/D2 | GNAV | Per service |
| FEC/parity/interleaving | GPS word parity through the LNAV backend | CRC-24Q, convolutional coding, interleaving, sync/SSP | BCH and interleaving | Hamming protection | Per service |
| Secondary/overlay sequence | None | E1-C 25-chip secondary code | D1 20-chip NH; none for D2 | Relative code + meander + time mark | Per service |
| RF modulation | BPSK(1) | Equal-power E1-B/E1-C CBOC composite | BPSK with D1 overlay or D2 symbols | BPSK on individual FDMA carriers | Continuous complex summation |
| Continuous block state | Carrier/code/data | Carrier/code/data/pilot overlay | Carrier/code/data/NH | Carrier/code/data plus frame-to-frame relative-code state | Reconciled for surviving channels |
| SC16 Q11 normalization | Implemented | Implemented | Implemented | Implemented | Deterministic whole-bank headroom |
| Hardware/receiver certification | Required | Required | Required | Required | Required |

## Navigation-message content

| Service | Generated content | Behavior when source data is absent |
| --- | --- | --- |
| GPS LNAV | Subframes built from broadcast clock, ephemeris, health, URA, TGD, IODE/IODC, week and time fields. | Required ephemeris fields fail validation; no synthetic healthy ephemeris is invented. |
| Galileo I/NAV | Word types 1–4 ephemeris and word type 5 time/health/BGD, scheduled into the ICD modulo-30 E1-B page-part sequence. | Optional service, almanac, FEC2, and ISM content unavailable from EPH input uses the distinct 192-bit vertical dummy-page format rather than fabricated nominal data. |
| BeiDou D1 | Clock/service subframe 1, ephemeris subframes 2–3, ionosphere coefficients when supplied, BCH/interleaving and frame schedule. | Missing optional service or almanac input is encoded as reserved/unavailable content. |
| BeiDou D2 | GEO basic-navigation pages 1–10 with D2 schedule and coding. | Optional data not represented by the broadcast record is reserved rather than guessed. |
| GLONASS GNAV | Immediate strings 1–4, calendar/time string 5, live UTC(SU)+3 `tk`/`NT`/`NA`/`N4`, relative encoding, meander and time mark. | Almanac data absent from the FDMA record is explicitly non-operational. |

These behaviors are deliberate data-integrity boundaries. A syntactically valid
RF frame must not imply that optional real-world service data was available.

## Time-system and signal-clock support

| Capability | Status |
| --- | --- |
| Continuous GPS week/seconds normalization | Implemented |
| GPS/GST relationship | Implemented for supported broadcast scheduling |
| GPS/BDT 14-second epoch offset | Implemented |
| GLONASS UTC(SU)+3 scheduling basis | Implemented |
| Historical GPS-UTC leap transitions and insertion-date validation | Implemented |
| Arbitrary future leap-second prediction from navigation headers | Not implemented |
| Per-satellite iterative transmit time | Implemented |
| Navigation and overlay initial phase from transmit time | Implemented |
| Common Doppler scale for code, data, and secondary/NH clocks | Implemented |
| Carrier phase continuity across 100 ms producer blocks | Implemented |
| Phase continuity for retained channels after allocator updates | Implemented |

## Navigation-file compatibility

| Input | GPS | Galileo | BeiDou | GLONASS | Notes |
| --- | --- | --- | --- | --- | --- |
| RINEX 2 GPS NAV | Yes | No | No | No | Used by the GPS automatic downloader and accepted by the shared typed loader. |
| RINEX 3 mixed NAV | LNAV | INAV used; FNAV parsed only | D1/D2 inferred from record family/PRN | State-vector propagation only | Legacy GLONASS records omit the status, group-delay, accuracy, and extended-health fields required for complete GNAV generation. |
| RINEX 4 mixed NAV | `EPH G.. LNAV` | `EPH E.. INAV`; FNAV parsed only | `EPH C.. D1/D2` | `EPH R.. FDMA` | Unsupported record families are skipped without desynchronizing the stream. |
| Plain file | Yes | Yes | Yes | Yes | Direct file input. |
| `.gz` / legacy `.Z` navigation | Yes on POSIX | Yes on POSIX | Yes on POSIX | Yes on POSIX | Shared typed loader streams through a shell-free `gzip` child; automatic downloads are also cached decompressed. |
| Automatic daily download | NOAA/NGS, then BKG | BKG mixed products | BKG mixed products | BKG mixed products | Requires `curl` and `gzip`; cached in the working directory. |

## Receiver-motion support

| Mode | CLI | Coordinates | Timing behavior | Status |
| --- | --- | --- | --- | --- |
| Static location | `-l lat,lon,height` | Geodetic degrees/metres | Constant position | Implemented |
| ECEF CSV | `-u file` | `time,x,y,z` in seconds/metres | Resampled to 10 Hz | Implemented |
| Geodetic CSV | `-p file` | `time,lat,lon,height` | Validated and resampled to 10 Hz | Implemented |
| NMEA GGA | `-g file` | Latitude/longitude/altitude | Checksum validation, midnight unwrap, 10 Hz resampling | Implemented |
| Keyboard | `-i` | Local north/east/up velocity offset over the selected base route | First-event start, acceleration/deceleration every 100 ms | Implemented |
| SDL2 controller | `-j index` | Left stick north/east, right stick vertical, accumulated over the selected base route | Dead zone and diagonal normalization | Optional at build time |

Receiver velocity is derived from successive ECEF samples and contributes to
range rate and Doppler. A controller disconnect is an error; the simulator does
not silently continue with a frozen motion command.

## bladeRF and sample-path support

| Capability | Status | Boundary |
| --- | --- | --- |
| bladeRF 1 and bladeRF 2 capability queries | Implemented | Depends on installed libbladeRF and connected hardware. |
| TX channel and USB-speed discovery | Implemented | At least one TX channel is required; link speed is reported for throughput diagnosis. |
| Loopback disable/read-back | Implemented | RF output requires `BLADERF_LB_NONE`; configuration fails if it cannot be verified. |
| Exact center-frequency read-back | Required | A coerced value is rejected. |
| Exact sample-rate read-back | Required | A coerced value is rejected because it changes signal timing. |
| Analog-bandwidth set-result/read-back | Validated | Both values must agree; quantization is accepted only if the realized filter contains the full signal span and does not exceed sample rate. |
| Portable overall TX gain | Implemented | Relative setting, not calibrated output power. |
| Legacy TXVGA1/TXVGA2 controls | Implemented for compatible bladeRF 1 hardware | Both must be supplied together. |
| XB200 TX path | Implemented | Attachment, L-band bypass, and automatic 1 dB filter selection are read back; RX is not changed. |
| SC16 Q11 output | Implemented | Interleaved signed 16-bit I/Q using the bladeRF Q11 range. |
| Whole-bank deterministic normalization | Implemented | Keeps a fixed channel-bank scale with headroom; no block AGC pumping. |
| Peak and rail telemetry | Implemented | Reported for submitted TX buffers. |
| Final partial-buffer handling | Implemented | Zero-padded before the last synchronous transfer. |
| RX/ADC processing | Not applicable | bladeGPS is a transmit-only simulator. |

## Automated validation coverage

| Validation area | Coverage |
| --- | --- |
| Build diagnostics | Normal optimized build plus strict warning build (`-Wall -Wextra -Wpedantic -Werror`). |
| Memory/undefined behavior | AddressSanitizer and UndefinedBehaviorSanitizer test run. |
| Static analysis | Clang analyzer on the production integration and parser paths. |
| Time | GPS epoch/week rollover, Gregorian century, historical leap boundary and invalid-insertion rejection, GST/BDT conversion, live GLONASS UTC(SU)+3 day rollover. |
| Codes | GPS PRN 37, Galileo E1-B/E1-C table checks, BeiDou boundary PRNs, GLONASS L1OF/time mark. |
| Navigation coding | Galileo CRC/FEC/interleaving, circular page-part timing and vertical dummy pages; BeiDou BCH/interleaving/D1/D2 pages; GLONASS live-time strings/Hamming/frame. |
| RF rendering | Block-split equivalence, phase preservation, overlay operation, CBOC sample value, whole-bank headroom. |
| Allocation | Health, elevation order, passband rejection, duplicate rejection, phase reconciliation. |
| Input | Typed RINEX 2/3/4, malformed/truncated cases, compressed legacy sample, motion formats. |
| CLI validation | Non-finite location/height/time/duration, sample-rate divisibility, and conflicting gain modes are rejected before hardware access. |
| Runtime integration | Individual non-GPS blocks, standalone GPS shared producer, stale-navigation rejection, mixed GPS backend. |
| Independent receiver check | Coherent BPSK carrier/code acquisition on generated I/Q. |

The automated receiver check does not yet decode every navigation message or
produce an independent position/velocity/time solution. Calibrated spectrum,
modulation-quality, absolute-power, and receiver interoperability tests remain
physical-laboratory acceptance gates.

## Unsupported or not advertised

The following services are not selectable and are not silently approximated:

- GPS L1C, L2C, and L5.
- SBAS L1 and SBAS PRN 120–158.
- Galileo E5/E6.
- BeiDou B1C, B2, and B3 services.
- GLONASS CDMA services.
- QZSS, NavIC/IRNSS, and other regional services.
- Integrity-fault, spoofing, multipath, atmospheric-grid, and ionospheric
  scenario editors.
- Built-in almanac acquisition independent of the supplied navigation data.

Adding a service requires more than a profile entry: it needs authoritative
code assignments, navigation data/FEC/scheduling, modulation, time-system and
group-delay rules, ephemeris selection, production routing, independent vectors,
and receiver validation. Until those layers exist, the profile must remain
unavailable and fail closed.
