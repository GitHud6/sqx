# ray2-idf

ESP-IDF 5.5.1 learning workspace for migrating the command-parser and motion-planning path from the Arduino baseline.

## Current milestone

The project accepts basic G0/G1 command lines from UART0, turns them into
bounded planner blocks, and executes the blocks in a dedicated planner task.
`motion_submit()` returns after a block is accepted into the queue rather than
after physical motion finishes.

## Build

Run from an ESP-IDF 5.5.1 PowerShell environment:

```powershell
idf.py build
```

The target is fixed to ESP32-S3 by `sdkconfig.defaults`.

## Next milestone

Add acceleration planning, realtime command routing, and laser PWM synchronization
to the planner block lifecycle.
