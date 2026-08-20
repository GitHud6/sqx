# ray2-idf

ESP-IDF 5.5.1 learning workspace for migrating the command-parser and motion-planning path from the Arduino baseline.

## Current milestone

The project is an ESP32-S3 UART0 echo baseline. `uart0_rx_task` puts received
data into a queue and `protocol_task` writes the same bytes back to UART0 TX.

## Build

Run from an ESP-IDF 5.5.1 PowerShell environment:

```powershell
idf.py build
```

The target is fixed to ESP32-S3 by `sdkconfig.defaults`.

## Next milestone

Create a persistent `command_parser_task` that consumes a UART0 receive queue.
