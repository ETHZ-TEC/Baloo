# Link quality estimation test

| **Author** |**Last modified**|
|:---:|:---:|
|Romain Jacob  |03-07-2019|

## SUMMARY
This application runs link quality tests, built on [Baloo](https://github.com/ETHZ-TEC/Baloo/wiki) and the [strobing primitive](https://github.com/ETHZ-TEC/Baloo/wiki/primitive-strobing).

## GENERAL DESCRIPTION

The test consists in one main Baloo round, where each node in the
network is assigned one data slot.
In these slots, nodes send a fixed number of strobes
(`GMW_CONF_TX_CNT_DATA`, set to 100 by default ).
The assignment of nodes to the slots is pseudo-random (controlled
by `randomseed`).

All other nodes listen to the strobes and log the reception data
over serial. The program execution is also monitored using GPIOs.

The TX power is set to 0dBm for all transmissions.

The default frequency channel used is:
- Channel 26 (2.48 GHz) for the sky mote
- Channel  5 (869  MHz) for the DPP-cc430 mote
This can be modified after compilation by binary patching (see below)

## CODE INSTRUMENTATION

The serial log is produced at the end of the test. They are formatted
as follows:
```c
Log:X:Y:Z[0].Z[1]. (...) .Z[n]
```
where  
- `X`    is the node_id of the initiator
- `Y`    is the number of strobes successfully received
- `Z[i]` is the i-th byte of the bit-stream of reception events;  
i.e., `Z[0]` contains the binary reception information of the first 8 strobes.
For example: `Z[0] = 23 = 0b 0001 0111`, means
 + the first 3 strobes were lost
 + the 4-th was received
 + the 5-th was lost
 + the last 3 strobes were received

For the slot where the node is initiator, the log simply says:
```
Log:slot_assignee:Strobing
```

If a data slot is "missed" or "skipped" by Baloo, a notification
is written in the serial log:
```
Missed/Skipped:slot_index:slot_assignee
```

The corresponding line of results is then:
```
Log:slot_assignee:Err!
```

In addition, the code is instrumented with three GPIO pin lines
to track the program execution (PRIM = {GLOSSY, STROBING})
+  `PRIM_START_PIN` track the execution of
the primitive; i.e., roughly from the slot start till slot end
+  `PRIM_TX_PIN` track the TX phases of the primitives
+  `PRIM_RX_PIN` track the RX phases of the primitives
For the strobing primitive, another pin marks every successful
strobe reception, mapped to the line as the `TX_PIN`;
i.e., successful strobe receptions are followed by a raising and
falling edge of the `TX_PIN` line.

## TIME SYNCHRONIZATION

Before the main Baloo round, a few empty rounds are run. These give
time to all nodes to synchronize with the HOST, and to learn their
internal clock drift.
There are `trigger_counter` such empty rounds.
The round period is to 10 seconds to enable a good estimation of
the drift (the longer the period, the better).

## PROGRAM INTERFACE

The code is written to enable binary patching after compilation.
Currently, this is done via the `tos-set-symbol` utility (included in the directory).
The user can patch
- the strobe payload length, by patching `payload_length`
- the random generator seed, by patching `randomseed`
- the HOST\_ID, by patching `host_id`
- the radio frequency channel, by patching `rf_channel`

The patching command is:
```bash
./tos-set-symbols --exe --objcopy msp430-objcopy --objdump msp430-objdump baloo-link-quality-test.TARGET baloo-link-quality-test.dpp-cc430
rf_channel=YOUR_CHANNEL payload_length=YOUR_PAYLOAD randomseed=YOUR_SEED host_id=YOUR_HOST
```

**TODO** modify this with the patching of an entire struct (method from
Markus). This will allow to have everything in one place, and to
extend to patching to the node list.
This would be nice, as the firmware could be then distributed and
used even without requiring anyone to recompile, just patch it to fit
your testbed!

## Compiling the code
|Platform| Compilation command |
|:---|:---|
|TelosB | make TARGET=sky |
|DPP-cc430 | make TARGET=dpp |
