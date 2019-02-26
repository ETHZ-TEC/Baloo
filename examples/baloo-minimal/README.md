|Platform| Compilation command |
|:---|:---|
|TelosB 
  | make TARGET=sky |
|DPP-cc430 
  | make TARGET=dpp |

All nodes have a slot assigned (from the `static_nodes array`), where they each send 2 bytes, a counter value and a magic number. The round period is set to 2s.

All other paramters and settings are left to their default.