# weedee

Python library for Keya motor controllers via RS232 (`keya_motor.py`).

- Serial settings: 115200 baud, 8-N-1
- Commands are ASCII, terminated with carriage return (`\r`)
- Acknowledgment: `+` for write commands, a response string for queries, `-` on error

## Command reference (TX)

Outgoing commands recognised by `_describe_tx`, grouped as in the source.

### Motor control

| Key      | Description                     |
| -------- | -------------------------------- |
| `!MG`    | Release emergency stop           |
| `!EX`    | Emergency stop                   |
| `!MS`    | Stop channel                     |
| `!M `    | Set motor speed                  |
| `!P `    | Go to absolute position          |

### Configuration

| Key      | Description                              |
| -------- | ----------------------------------------- |
| `^MAC`   | Set acceleration (× 0.1 RPM/s)            |
| `^MDEC`  | Set deceleration (× 0.1 RPM/s)            |
| `^MVEL`  | Set position velocity (RPM)               |
| `%EESAV` | Save configuration to EEPROM              |

### Digital I/O

| Key   | Description                                    |
| ----- | ----------------------------------------------- |
| `!D0` | Turn OFF digital output                         |
| `!D1` | Turn ON digital output                          |
| `!DS` | Set all digital outputs (bitmask)               |

### Queries

| Key    | Description                                       |
| ------ | -------------------------------------------------- |
| `?V`   | Read voltages (internal / battery / 5 V)            |
| `?FF`  | Read fault flags                                    |
| `?FS`  | Read status flags                                   |
| `?LK`  | Read lock status                                    |
| `?E`   | Read closed-loop error                              |
| `?DO`  | Read digital output states                          |
| `?BS`  | Read brushless motor speed (Hall sensors)           |
| `?CIA` | Read internal analog command value                  |
| `?CIP` | Read internal pulse command value                   |
| `?CIS` | Read internal serial command value                  |
| `?A`   | Read motor amps                                     |
| `?AIC` | Read analog input after conversion (±1000 scale)     |
| `?AI`  | Read analog input (raw mV)                          |
| `?BA`  | Read battery amps                                   |
| `?C`   | Read encoder position (counts)                      |
| `?DI`  | Read digital input                                  |
| `?D`   | Read all digital inputs (bitmask)                   |
| `?F`   | Read feedback                                       |
| `?M`   | Read applied motor command (±1000)                   |
| `?P`   | Read applied power level (±1000)                     |
| `?PIC` | Read pulse input after conversion (±1000 scale)      |
| `?PI`  | Read pulse input (µs)                               |
| `?S`   | Read encoder speed (RPM)                             |
| `?T`   | Read temperatures (°C)                               |

## Response reference (RX)

Incoming response keys (the part before `=`) recognised by `_describe_rx`.

| Key         | Description                                        |
| ----------- | --------------------------------------------------- |
| `+`         | OK — acknowledged                                    |
| `-`         | ERROR — command not recognised                       |
| `V`         | Voltages: internal / battery / 5 V out                |
| `FF`        | Fault flags                                          |
| `FS`        | Status flags                                         |
| `A`         | Motor amps (per channel)                              |
| `BA`        | Battery amps (per channel)                            |
| `M`         | Applied motor command (per channel, ±1000)            |
| `P`         | Applied power level (per channel, ±1000)              |
| `S`         | Encoder speed (per channel, RPM)                      |
| `BS`        | Brushless motor speed (Hall sensor, RPM)              |
| `T`         | Temperatures: IC / ch1 heatsink / ch2 heatsink         |
| `C`         | Encoder count                                        |
| `E`         | Closed-loop error                                    |
| `F`         | Feedback value                                       |
| `LK`        | Lock status                                          |
| `AI`        | Analog input, raw (mV)                                |
| `AIC`       | Analog input, converted (±1000 scale)                 |
| `CIA`       | Internal analog command value (±1000)                 |
| `CIP`       | Internal pulse command value (±1000)                  |
| `CIS`       | Internal serial command value (±1000)                 |
| `DI`        | Digital input state (1 = active, 0 = inactive)         |
| `DO`        | Digital output bitmask                                |
| `D`         | All digital inputs bitmask                            |
| `PI`        | Pulse input, raw (µs)                                 |
| `PIC`       | Pulse input, converted (±1000 scale)                   |
