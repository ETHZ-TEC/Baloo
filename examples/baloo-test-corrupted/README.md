|Platform| Compilation command |
|:---|:---|
|TelosB   | make TARGET=sky |
|DPP-cc430   | make TARGET=dpp |
 
This protocol illustrates the use of the interference detection feature. It is controlled (in `project-conf.h) by the following defines:

|#define| Rationale |
|:---|:---|
|GMW_CONF_USE_NOISE_DETECTION 
  | (Des)Activate the feature |
|GMW_CONF_HIGH_NOISE_THRESHOLD 
  | The threshold value for considered as *high noise*, in dBm|
|GMW_CONF_HIGH_NOISE_MIN_COUNT 
  | Number of times the threshold must be exceeded to consider there is *high noise* on the wireless channel. |

After each slot, the `post_callback()` functions pass to the NET layer an event (`gmw_pkt_event_t event`) to inform on the past slot execution:
```c
typedef enum {
  GMW_EVT_PKT_OK = 0,          /* Successful Glossy flood (send or receive) */
  GMW_EVT_PKT_CORRUPTED = 1,   /* A Glossy flood detected but corrupted */
  GMW_EVT_PKT_GARBAGE = 2,     /* Something received but not a Glossy flood */
  GMW_EVT_PKT_SILENCE = 3,     /* Nothing received at all */
  GMW_EVT_PKT_MISSED = 4,      /* Packet missed (bad timing) */
  GMW_EVT_PKT_SKIPPED = 5      /* Slot skipped */
} gmw_pkt_event_t;
```

To generate some contention, in this application, all data slots are contention slots. Every node decides with some probability wether it sends its own packet. Nodes that do not send receive a `GMW_EVT_PKT_CORRUPTED` or `GMW_EVT_PKT_CORRUPTED` event if the contention slot fails.
