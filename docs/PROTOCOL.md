# FT9366 USB protocol (as used by the driver)

Everything here was learned from observing the reader's USB traffic; the command tables in
`libfprint/drivers/ft9366/ft9366_script.h` replay slices of such a recording (see `tools/gen_script.py`).
Meanings marked *(inferred)* are guesses from behaviour, not documented facts.

## Device
- USB `2808:a658`: a Realtek RTS5811 USB bridge in front of the FocalTech FT9366 sensor.
- Sensor array 64 columns x 80 rows, 16-bit samples.
- Vendor-specific interface 0, bulk OUT `0x01`, bulk IN `0x82`. No vendor control requests are used after
  configuration.

## Bulk commands
All multi-byte fields are big-endian. The host writes a command on OUT, then reads the reply (if any) on IN.

| OUT bytes                     | meaning                                              | reply            |
|-------------------------------|------------------------------------------------------|------------------|
| `4c 5a 01 00` / `4c 5a 00 00` | bridge sync / mode select *(inferred)*               | 4 bytes          |
| `44 80 00 00`                 | sensor reset *(inferred)*                            | none             |
| `09 AA AA VV`                 | write 8-bit bridge register `AAAA`                   | none             |
| `08 AA AA 00 00`              | read 8-bit bridge register `AAAA`                    | 1 byte           |
| `05 fa AA AA 00 NN <NN words>`| write `NN` 16-bit sensor registers from `AAAA`       | none             |
| `04 fb AA AA 00 NN`           | read `NN` 16-bit sensor registers from `AAAA`        | `2*NN` bytes     |
| `06 f9 AA AA LL LL`           | read `LLLL` words from the sensor image port `AAAA`  | `2*LLLL` bytes   |
| `5a a5 00` / `a5 5a 00`       | enter / leave a sensor transaction *(inferred)*      | none             |
| `c0 3f 00`, `c2 3d 00`, `c4 3b 00` | small bridge commands, not understood           | none             |
| `4b ...` (24 bytes)           | seen once at the end of initialisation               | 2 bytes          |

## Registers that matter for the driver
- Sensor `0x9a82`: interrupt flags. `0x0002` = finger down, `0x0004` = finger up. The driver polls this.
- Sensor `0x9a8b`: reads `0x9362` when the chip is awake, 0 while it sleeps *(inferred: chip id)*.
- Sensor `0x9800`: control. Setting bit 0 (`0x4ffe` -> `0x4fff`) starts an image scan.
- Sensor `0x9a84`: interrupt clear/mask.
- Bridge `0xf780`: status; reads `0x54` just before an image can be read.
- Other registers (`0x9801`, `0x9804`..`0x980b`, `0x9881`, `0x80b0`, `0x80b8`, ...) are set to the values seen in
  the recording; their individual meaning is not understood.

## Image read
1. Switch the analog front end to image-scan settings (register block as recorded), clear the interrupt,
   poll bridge `0xf780` until it reads `0x54`.
2. Read `0x9800`, write it back with bit 0 set.
3. Send `06 f9 9a 05 14 00` (0x1400 = 5120 words) and read 10240 bytes.
4. The data is 5120 big-endian 16-bit samples, row-major 64 x 80. Raw values are roughly 0x0000-0x0e00;
   ridges are lower than valleys. An empty sensor has a frame mean of about 2430, a finger roughly 2000-2150.

## Host flow
open: USB reset, claim interface, wait ~500 ms, replay the init script (it includes two empty-sensor scans that
the driver keeps as the reference image) -> poll for finger: replay the idle script and read the flag in its last
reply; on "finger down" replay the scan script several times to collect frames.
