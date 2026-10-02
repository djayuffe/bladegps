# C API and Function Reference

This reference covers the exported C interfaces in bladeGPS. Internal `static`
helpers are implementation details and are described by responsibility in
`ARCHITECTURE.md`. Unless a function says otherwise, success is `0`, failure is
`-1`, output pointers must be non-null, and arrays must have the declared size.

## Common representation rules

- Navigation bits are unpacked: one `uint8_t` per bit, values exactly 0 or 1,
  in transmission/MSB order.
- Spreading codes and RF symbols are `int8_t` levels, exactly +1 or -1.
- Angles are radians internally; CLI latitude/longitude/elevation are degrees.
- Positions are ECEF metres and velocities are ECEF metres/second.
- Frequencies/rates are hertz; phases are cycles/chips/symbols as named.
- `gpstime_t.week` is continuous unless a navigation builder explicitly
  truncates it for an on-air field.
- Public builders validate representable ranges and reject overflow instead of
  silently truncating.

## Signal registry — `gnss.h`

### Types

`gnss_system_t` enumerates GPS, Galileo, BeiDou, and GLONASS.
`gnss_signal_t` enumerates four single-service profiles plus `mixed-open`.

`gnss_signal_profile_t` fields:

| Field | Contract |
| --- | --- |
| `id`, `system` | Registry identity and primary constellation. |
| `name`, `description` | Stable CLI name and human-readable description. |
| `carrier_hz` | Nominal carrier or mixed-plan center. |
| `code_rate_hz`, `code_length` | Primary spreading-code clock and period. |
| `minimum_sample_rate_hz` | Default complex sample rate. |
| `recommended_bandwidth_hz` | Default occupied/analog bandwidth plan. |
| `maximum_sv` | Highest runtime satellite identifier for that profile. |
| `fdma` | Non-zero when individual carriers must be considered. |
| `waveform_implemented` | Non-zero only for executable production profiles. |

### Functions

| Function | Parameters and behavior | Return |
| --- | --- | --- |
| `gnss_signal_profile(signal)` | Index lookup; returned storage is immutable/static. | Profile pointer or `NULL` for an invalid enum. |
| `gnss_signal_profile_by_name(name)` | Exact, case-sensitive CLI-name lookup. | Profile pointer or `NULL`. |
| `gnss_system_name(system)` | Maps enum to display text. | Static name; `"Unknown"` for invalid input. |
| `gnss_signal_parse(name, signal)` | Parses an exact profile name into `*signal`. | 0 or -1. |
| `gnss_frequency_fits(center, sample_rate, carrier, occupied_bw)` | Tests `abs(carrier-center)+occupied_bw/2 <= sample_rate/2`; all inputs must be finite/positive as applicable. | Boolean 1/0. |
| `gnss_print_signal_profiles()` | Prints every profile, system, MHz carrier, and implementation state to stdout. | `void`. |

## Time conversion — `gnss_time.h`

| Function | Parameters and behavior | Return |
| --- | --- | --- |
| `gnss_calendar_to_gps(system, calendar, gps)` | Validates a native RINEX calendar epoch and converts it to continuous GPS week/SOW. GPS/GST are aligned, BDT is 14 seconds behind GPS, and GLONASS epochs use the historical GPS-UTC table. | 0/-1. |
| `gnss_gps_to_system_time(system, gps, system_time)` | Converts continuous GPS time to the constellation scheduling scale while retaining a continuous Sunday-based week. | 0/-1. |
| `gnss_gps_utc_offset(utc, offset)` | Returns historical GPS−UTC seconds. An inserted `23:59:60` uses the offset valid during that leap second. | 0/-1. |
| `gnss_time_difference(newer, older)` | Full-week difference `(week delta × 604800) + SOW delta`. | Seconds; `NAN` for null pointers (non-finite members propagate naturally). |

## Typed navigation input — `gnss_nav.h`

`gnss_calendar_time_t` is year/month/day/hour/minute/second.
`gnss_nav_model_t` selects Keplerian or GLONASS state-vector propagation.
`gnss_nav_record_t` is described in `DATA_FORMATS.md`.

| Function | Parameters and behavior | Return |
| --- | --- | --- |
| `gnss_read_rinex_nav(path, records, capacity, count)` | Parses supported RINEX. Use `records=NULL, capacity=0` for counting; otherwise capacity must hold every accepted record. On success `*count` is exact. POSIX accepts `.gz`/`.Z`. | 0 when at least one record was read; -1 otherwise. |
| `gnss_load_rinex_nav(path, records, count)` | Two-pass count/allocation/load. Sets outputs to empty before work. Caller releases `*records` with `free()`. | 0/-1. |
| `gnss_read_beidou_ionosphere(path, model)` | Reads the last complete BDS alpha/beta model from RINEX 3 header or RINEX 4 ION data. | 0 found, 1 absent, -1 malformed/error. |

## Orbit propagation — `gnss_orbit.h`

| Function | Parameters and behavior | Return |
| --- | --- | --- |
| `gnss_propagate_kepler(record, transmit_sow, position, velocity, clock_bias, clock_drift)` | Propagates GPS/Galileo/BeiDou Keplerian broadcast data; applies the constellation constants, harmonic corrections, relativity, clock polynomial, and BeiDou GEO transform when applicable. | 0/-1. |
| `gnss_propagate_glonass(record, delta_seconds, position, velocity, clock_bias, clock_drift)` | RK4 propagation of a GLONASS state vector with Earth rotation/J2 and broadcast acceleration over `delta_seconds`. | 0/-1. |

## Observation geometry — `gnss_geometry.h`

`gnss_observation_t` returns satellite ECEF position/velocity, geometric and
clock-corrected pseudorange, range rate, azimuth/elevation, clock terms,
Doppler, code/carrier phases, transmit SOW, travel time, and group delay.

`gnss_observe(record, receive_sow, receiver_position, receiver_velocity,
carrier_hz, code_rate_hz, code_length, observation)` iterates transmit time,
dispatches orbit propagation, applies Earth rotation, evaluates range/range
rate and signal delay, then derives Doppler and initial phases. Carrier/code
parameters must be positive and finite. Returns 0/-1.

## Ranging codes and subcarriers — `gnss_codes.h`

| Function | Contract |
| --- | --- |
| `gnss_beidou_b1i_code(prn, chips)` | Generates 2046 B1I chips for PRN 1–63. |
| `gnss_glonass_l1of_code(chips)` | Generates the shared 511-chip L1OF code. |
| `gnss_glonass_l1of_carrier_hz(slot, carrier)` | Maps FDMA slot -7…+6 to `1602 MHz + slot×562.5 kHz`. |
| `gnss_decode_hex_code(hex, count, chips)` | Decodes MSB-first hexadecimal memory-code text to ±1 chips; rejects invalid characters/insufficient input. |
| `gnss_galileo_e1_primary_code(prn, component, chips)` | Loads the 4092-chip E1-B or E1-C memory code for PRN 1–50. |
| `gnss_galileo_e1c_secondary_code(chips)` | Produces the 25-chip E1-C secondary sequence. |
| `gnss_beidou_b1i_nh_code(chips)` | Produces the 20-chip D1 Neumann-Hoffman overlay. |
| `gnss_glonass_time_mark(chips)` | Produces the 30-chip GNAV time mark. |
| `gnss_galileo_e1_cboc(data, pilot)` | Produces twelve subchip weights for the E1-B data and opposite-sign E1-C pilot CBOC branches. |

Code generators return 0/-1 except `gnss_galileo_e1_cboc()`, which is `void`.

## Forward-error correction — `gnss_fec.h`

| Function | Contract |
| --- | --- |
| `gnss_crc24q_bits(bits, count)` | Computes the 24-bit CRC-24Q remainder over unpacked bits; `NULL` returns zero. |
| `gnss_galileo_convolutional_encode(input, count, output, capacity)` | Rate-1/2 Galileo encoder using G1=171 octal and inverted G2=133 octal; output needs `2×count`. |
| `gnss_block_interleave(input, columns, rows, output, capacity)` | Writes down columns and reads across rows; capacity needs `columns×rows`; multiplication overflow is rejected. |
| `gnss_beidou_bch15_11(info, codeword)` | Systematic BCH(15,11), polynomial `0x13`. |
| `gnss_beidou_interleave_2x15(first, second, output)` | Alternates two 15-bit codewords into 30 transmitted bits. |
| `gnss_glonass_hamming_85_77(data, string)` | Adds seven Hamming checks plus total even parity in GLONASS transmission order. |

All encoders except CRC return 0/-1 and reject input bytes other than 0/1.

## Galileo I/NAV — `gnss_galileo_nav.h`

The `galileo_inav_word1_t` … `word5_t` structures contain already quantized
ICD integer fields. Builders reject values wider than their bit allocation.

| Function | Contract |
| --- | --- |
| `gnss_galileo_inav_word1` … `word5` | Build one 128-bit nominal word type from its corresponding structure. |
| `gnss_galileo_inav_ephemeris_words(record, words)` | Quantizes a validated Galileo INAV RINEX record into word types 1–4. |
| `gnss_galileo_inav_e1b_page(word, osnma, sar, spare, ssp, even, odd, crc)` | Constructs one vertical E1-B page: headers/protected data, CRC-24Q, tail, convolutional code, 30×8 interleave, sync and SSP. Inputs/outputs are unpacked. |
| `gnss_galileo_inav_e1b_word_type(gst_second_mod_30)` | Returns the nominal scheduled type for integer second 0…29; returns error for out-of-range input. |
| `gnss_galileo_inav_ssp_for_second(second)` | Selects SSP3, SSP1, SSP2 for successive two-second page epochs; every unsigned input is accepted and indexed with `second % 6`. |

## BeiDou D1/D2 — `gnss_beidou_nav.h`

`beidou_d1_ephemeris_t`, `beidou_d1_clock_t`, and `beidou_almanac_t` contain
quantized on-air integers. `beidou_nav_format_t` distinguishes D1 and D2.

| Function | Contract |
| --- | --- |
| `gnss_beidou_nav_encode_subframe(info, subframe)` | Converts 224 information bits into the 300-bit ten-word BCH/interleaved stream. |
| `gnss_beidou_nav_build_subframe(fraid, sow, payload, subframe)` | Adds preamble/header/FraID/BDT SOW to 186 payload bits and encodes. |
| `gnss_beidou_d1_ephemeris_subframes(fields, frame_sow, sf2, sf3)` | Builds D1 subframes 2/3 with header times `frame_sow+6/+12`, modulo one week. |
| `gnss_beidou_d1_ephemeris_from_rinex(record, fields)` | Quantizes supported D1/D2 Keplerian ephemeris fields. |
| `gnss_beidou_d1_clock_subframe(fields, frame_sow, sf1)` | Builds D1 clock/service subframe 1. |
| `gnss_beidou_d1_clock_from_rinex(record, alpha, beta, fields)` | Quantizes clock, health, URAI, week, TGD, and ionosphere data. |
| `gnss_beidou_ionosphere_quantize(model, alpha, beta)` | Converts floating Klobuchar coefficients to the signed 8-bit BDS scales. |
| `gnss_beidou_almanac_subframe(format, fraid, page, sow, fields, subframe)` | Builds only pages legal for the requested D1/D2 schedule and identifier. |
| `gnss_beidou_d2_basic_pages(clock, ephemeris, frame_sow, pages)` | Builds ten successive three-second GEO D2 page-1…10 frames. |

## GLONASS GNAV — `gnss_glonass_nav.h`

GLONASS structures contain quantized sign-magnitude fields plus time, status,
slot, frequency, and almanac metadata.

| Function | Contract |
| --- | --- |
| `gnss_glonass_gnav_immediate_strings(fields, strings)` | Builds protected strings 1–4 from immediate data. |
| `gnss_glonass_gnav_from_rinex(record, fields)` | Converts a typed FDMA state-vector record to immediate GNAV fields. |
| `gnss_glonass_gnav_string5(fields, string)` | Builds UTC/system-time string 5. |
| `gnss_glonass_gnav_almanac_pair(fields, even_no, even, odd)` | Builds one legal even/odd almanac pair. |
| `gnss_glonass_gnav_frame(immediate, time, almanacs, frame)` | Builds all fifteen strings; the five-element almanac array supplies pairs 6–15. |

## Transmitter schedules — `gnss_schedule.h`

| Function | Output and schedule |
| --- | --- |
| `gnss_schedule_galileo_e1(record, week, tow, symbols)` | 7500 NRZ symbols = fifteen two-second pages = 30 seconds at 250 symbols/s. |
| `gnss_schedule_beidou_d1(record, alpha, beta, sow, symbols)` | 1500 symbols = five six-second subframes = 30 seconds at 50 bit/s. |
| `gnss_schedule_beidou_d2(record, alpha, beta, sow, symbols)` | 15000 symbols = ten three-second frames = 30 seconds at 500 bit/s. |
| `gnss_schedule_glonass(record, symbols)` | 3000 symbols = fifteen two-second strings at the post-meander 100 symbol/s rate. |

Each function validates record family/fields and returns 0/-1.

## Shared RF layer — `gnss_rf.h`

### `gnss_rf_channel_t`

| Field group | Meaning |
| --- | --- |
| identity | `enabled`, modulation, system, PRN. |
| carrier | nominal `carrier_hz`, `doppler_hz`, linear sample `amplitude`, phase radians `[0,2π)`. |
| code | data/pilot code pointers, common length, received code rate, phase in chips. |
| data | NRZ symbol pointer/count, received symbol rate, phase in symbols. |
| overlay | optional secondary/NH pointer/count, received rate and phase. |

### Functions

| Function | Contract |
| --- | --- |
| `gnss_rf_bits_to_symbols(bits, count, symbols)` | Converts bit 0→+1 and bit 1→-1. |
| `gnss_glonass_l1of_symbols(string, previous, symbols)` | Differentially encodes one 85-bit string, applies the 100 Hz meander for 170 symbols, appends 30 time-mark chips, and updates cross-string state. |
| `gnss_rf_validate_channel(channel, center, sample_rate)` | Validates identity, finite/ranged phases/rates, ±1 buffers, modulation-specific pilot requirements, and sampled-passband fit. |
| `gnss_rf_render(channels, count, center, sample_rate, iq, samples)` | Validates enabled channels, computes deterministic bank normalization, renders BPSK or E1 CBOC, advances every phase, and writes saturated interleaved SC16 Q11. |
| `gnss_rf_allocate(candidates, count, center, rate, mask, selected, capacity, selected_count)` | Filters unhealthy/below-mask/out-of-band candidates and returns indices in descending elevation, capped by capacity. |
| `gnss_rf_reconcile(active, capacity, desired, count)` | Rebuilds a unique channel bank and preserves carrier/code/data/overlay phases for unchanged identities. |

## Independent receiver — `gnss_receiver.h`

`gnss_rx_acquire_bpsk(iq, samples, sample_rate, code, code_length, code_rate,
min_carrier, max_carrier, carrier_step, result)` exhaustively searches every
integer code phase and inclusive carrier grid. `result` contains best carrier
offset, code phase, and magnitude/total-energy normalized correlation. It is a
test validator, not a real-time navigation receiver. Returns 0/-1.

## bladeRF hardware adapter — `blade_hw.h`

`blade_hw_config_t` is the requested frequency/sample/bandwidth/signal span,
gain mode, and XB board. `blade_hw_result_t` records realized RF settings and
board name. `blade_sample_stats_t` accumulates complex-sample count, component
rail contacts, and peak absolute component.

| Function | Contract |
| --- | --- |
| `blade_hw_range_contains(range, value)` | Applies libbladeRF `value×scale` bounds and finite-value validation. |
| `blade_hw_required_bandwidth(center, carrier, occupied)` | Returns `occupied + 2×abs(carrier-center)` or `NAN`. |
| `blade_hw_validate_rf_plan(center, carrier, occupied, sample_rate, bandwidth)` | Requires finite positive values, required span within both sample/analog bandwidth, and analog bandwidth no wider than sample rate. |
| `blade_hw_validate_stream_geometry(buffers, size, transfers)` | Requires non-zero values, buffer size multiple of 1024, and `buffers > transfers`. |
| `blade_hw_configure_tx(dev, config, result)` | Requires configured FPGA, validates device ranges, exact frequency/rate read-back, safe realized bandwidth, gain, and optional XB200 TX setup. Returns libbladeRF status. |
| `blade_hw_measure_samples(iq, count, stats)` | Accumulates sample count, max absolute component, and values touching/exceeding Q11 rails. |

## Motion controller — `motion_controller.h`

| Function | Contract |
| --- | --- |
| `motion_controller_open(controller, index)` | Opens a zero-based SDL game-controller. Returns 0, -1 for invalid/runtime failure, or -2 when built without SDL2. |
| `motion_controller_poll(controller, horizontal_max, vertical_max, neu)` | Updates events, checks attachment, applies dead zone/normalization, and returns north/east/up m/s. Returns 0, -1, or -2 without SDL2. |
| `motion_controller_close(controller)` | Closes the handle/subsystem if active and clears state; null-safe. |
| `motion_keyboard_update(requested, active, speed, increment, maximum)` | Advances one keyboard-velocity update: a new positive direction starts at one increment, repeated events accelerate to the cap, and direction zero decelerates to rest. Returns 0/-1 and rejects non-finite or inconsistent state. |

## Production threads and FIFO — `bladegps.c`, `gnss_task.h`

### Runtime structures

| Type | Important fields and units |
| --- | --- |
| `option_t` | Input paths, selected signal, center/sample/filter settings in hertz, gains in dB, elevation mask in degrees, receiver mode, duration in 0.1-second blocks, scenario GPS time, and live-input selection. |
| `tx_t` | TX thread, lock, terminal error, bladeRF handle, interleaved SC16 buffer, accumulated sample statistics, and final-padding count. |
| `gps_t` | Producer thread, lock, initialization result, readiness flag, and initialization condition variable. |
| `sim_t` | Complete immutable options plus TX/producer state, FIFO indices and capacities in complex samples, 100 ms block size, conditions, status, completion flag, and elapsed simulation seconds. |

The FIFO stores *complex-sample units*. Each element consumes two `int16_t`
components in memory. `head`, `tail`, `fifo_length`, `sample_length`, and all
FIFO function counts therefore describe I/Q pairs rather than scalar values.

| Function | Contract |
| --- | --- |
| `init_sim(sim)` | Initializes locks/conditions/state and derives a `sample_rate/10` producer block and two-block FIFO. Returns `void`; pthread initialization failures are not surfaced by this legacy initializer. |
| `get_sample_length(sim)` | Returns currently queued complex samples. The function itself does not lock; production callers hold `sim->gps.lock`. |
| `fifo_read(buffer, samples, sim)` | Reads `min(samples, queued)` complex samples with ring wraparound, advances `tail`, and returns the count read. It does not lock; production callers hold `sim->gps.lock`. `buffer` contains `2 × return_value` components. |
| `is_finished_generation(sim)` | Returns the current completion flag without locking; production callers use `sim->gps.lock`. |
| `is_fifo_write_ready(sim)` | Returns true when one complete `iq_block_samples` producer block fits. It does not lock and also refreshes the cached `sample_length`; production callers hold `sim->gps.lock`. |
| `tx_task(argument)` | FIFO consumer and synchronous bladeRF TX thread. Returns `NULL`; terminal failure is stored in `sim->tx.error`. |
| `start_tx_task(sim)` | Creates `tx_task`; returns pthread status. |
| `start_gnss_task(sim)` | Creates `gnss_task`; returns pthread status. |
| `gnss_task(argument)` | Shared producer: load data/motion, select/observe/schedule/allocate/render, write FIFO, and propagate completion/error. Returns `NULL`; readiness/error/completion are published through `sim_t`. |
| `stop_was_requested()` | Returns non-zero after SIGINT/SIGTERM; reads only the process `sig_atomic_t` stop flag and is safe at generation boundaries. |
| `usage()` | Prints the complete option synopsis, ranges, units, and defaults to standard output. |

## Inherited GPS primitives — `gpssim.h`

These remain public because the shared producer uses several directly and the
legacy standalone path remains buildable.

| Function | Purpose |
| --- | --- |
| `date2gps(t, g)` | Converts a validated `datetime_t` calendar value to GPS week/SOW in `*g`. This legacy routine has no status return; new typed ingestion uses `gnss_calendar_to_gps()`. |
| `gps2date(g, t)` | Converts GPS week/SOW to `datetime_t`; output seconds can be fractional. This is the inverse legacy arithmetic path, not a leap-second database API. |
| `llh2xyz(llh, xyz)` | Converts geodetic latitude/longitude in radians and ellipsoidal height in metres to WGS-84 ECEF metres. |
| `xyz2llh(xyz, llh)` | Converts WGS-84 ECEF metres to latitude/longitude radians and ellipsoidal height metres. |
| `ltcmat(llh, matrix)` | Forms the 3×3 ECEF-to-local north/east/up rotation at geodetic `llh`. |
| `ecef2neu(vector, matrix, neu)` | Applies an already formed local matrix to an ECEF vector; all three arrays must be caller-owned. |
| `codegen(ca, prn)` | Generates GPS L1 C/A PRN 1–37 into 1023 caller-supplied `int` chips represented as 0/1. Out-of-range PRNs leave the output untouched and must be rejected by the caller. |
| `computeChecksum(source, nib)` | Returns a 30-bit GPS LNAV word with parity. `source` includes the two previous transmitted parity bits in bits 31/30; non-zero `nib` allows D29/D30 data-bit solving for words 2 and 10. |
| `subGpsTime(g1, g0)` | Returns `g1-g0` in seconds with GPS week rollover accounted for. |
| `normalizeGpsTime(g)` | Mutates week/SOW until `0 <= sec < 604800`, adjusting the continuous week accordingly. |
| `selectEphemerides(selected, source, count, time)` | For each GPS PRN, selects the newest healthy record whose clock epoch and fit interval cover `time`; returns the number of valid selected satellites. |
| `readRinexNavAll(eph, path)` | Loads legacy GPS RINEX 2 data into at most 13 fixed ephemeris sets; returns the number of sets or a negative error. POSIX compressed input is supported. Production uses the typed loader. |
| `readUserMotion(xyz, path)` | Parses finite, strictly increasing `time,x,y,z` ECEF rows and linearly resamples them at 10 Hz. Returns sample count, -1 on open failure, or -2 on malformed/time-order input. |
| `readLlhMotion(xyz, path)` | Parses finite, ranged, strictly increasing `time,lat,lon,height` rows, converts to ECEF, and resamples at 10 Hz. Return convention matches `readUserMotion()`. |
| `readNmeaGGA(xyz, path)` | Accepts checksum-valid, positive-fix GGA records, unwraps UTC midnight, converts mean-sea-level altitude plus geoid separation to ellipsoidal height, and resamples at 10 Hz. Returns accepted output count, -1 on open failure, or -2 on non-increasing accepted fixes. |
| `eph2sbf(eph, sbf)` | Quantizes one validated GPS ephemeris into five 10-word LNAV subframe templates; TOW and parity are added later. |
| `generateNavMsg(g, channel, init)` | Aligns to the containing 30-second frame, inserts HOW TOW counts, computes cross-word parity, and fills the channel's rolling 60-word navigation buffer. Returns 0. |

## Portability helpers — `getch.h`, `getopt.h`

`_kbhit()` returns non-zero when a console byte is available without blocking;
`_getch()` returns one console byte without line buffering. They exist for the
interactive keyboard path. The bundled `getopt` compatibility layer exports
the conventional `getopt()`, `optarg`, `optind`, `opterr`, and `optopt`
contract on platforms that do not provide it; bladeGPS uses short options only.

## Internal helper map

Private functions are intentionally not callable API, but their responsibilities
are important during audits:

| Module | Internal logic |
| --- | --- |
| `bladegps.c` | Strict CLI parsing, download dates/URLs/atomic files, signal handling, command execution, cleanup. |
| `gnss_task.c` | Health interpretation, record/service matching, GPS record adaptation, per-service store construction, shutdown signaling. |
| `gnss_nav.c` | Compressed-stream lifecycle, fixed-field numeric parsing, calendar validation, RINEX 3 inference, RINEX 4 record skipping. |
| `gnss_orbit.c` | Week wrapping, civil-day conversion, constellation constants, Kepler iteration, GLONASS acceleration/RK4. |
| `gnss_geometry.c` | Group-delay selection, Earth-rotation transform, line-of-sight and local-angle calculation. |
| navigation modules | Bit insertion, signed/unsigned width checks, physical-to-LSB quantization, calendar/schedule helpers. |
| `gnss_rf.c` | Symbol validation, modular phase advancement, Q11 saturation, signal identity comparison. |
| `blade_hw.c` | Scaled hardware ranges, API error reporting, XB200 setup, named legacy gain-stage handling. |
