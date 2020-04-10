|Platform| Compilation command |
|:---|:---|
|TelosB   | make TARGET=sky |
|DPP-cc430   | make TARGET=dpp |

This simple Baloo protocol illustrates the utilization of the Strobing as communication primitive.

### Using the Strobing primitive

To use multiple communication primitives, the `GMW_CONF_USE_MULTI_PRIMITIVES` must be set to 1.

To use Strobing as a primitive, the following defines must be configured
|#define| Rationale |
|:---|:---|
|STROBING_CONF_PAYLOAD_LEN 
  | Payload lenght to be send using the Strobing primitive |
|STROBING_CONF_USE_TIMER_ISR 
  | Set to 0 enable the multiplexing of the timer ISR (`sky` platform only)|
|STROBING_CONF_USE_RF1A_CALLBACKS 
  | Set to 0 enable the multiplexing of the timer ISR (`dpp` platform only)|
|GLOSSY_CONF_USE_TIMER_ISR 
  | Set to 0 enable the multiplexing of the timer ISR (`sky` platform only)|
|GLOSSY_CONF_USE_RF1A_CALLBACKS 
  | Set to 0 enable the multiplexing of the timer ISR (`dpp` platform only)|

### Using per-slot configuration feature

To use per-slot configuration feature, the `GMW_CONF_USE_CONTROL_SLOT_CONFIG` must be set to 1. Setting this define has for effect to include a `gmw_slot_config_t` section in the control structure.
```c
typedef struct __attribute__((packed)) gmw_slot_config {
  uint8_t n_retransmissions : 3;
  uint8_t slot_time_select  : 3; /* ID of desired slot time (see slot_times)*/
  uint8_t primitive         : 2; /* ID of the primitive to use (0 is Glossy) */
} gmw_slot_config_t;

typedef struct __attribute__((packed)) gmw_control {
  /* ... */
#if GMW_CONF_USE_CONTROL_SLOT_CONFIG
  gmw_slot_config_t   slot_config[GMW_CONF_MAX_SLOTS];
  uint8_t             slot_time_list[GMW_CONTROL_SLOT_CONFIG_TIMELIST_SIZE];
#endif /* GMW_CONF_USE_CONTROL_SLOT_CONFIG */
  /* ... */
} gmw_control_t;
```
> **Note** Adding the `gmw_slot_config_t` section to the control structure does not mean that this section will be sent in the control packet by default! To send this section in a control slot, you must use the  `GMW_CONTROL_SET_SLOT_CONFIG(control)` instruction when preparing your control packet.  
This sets a flag that is read by the middleware which then compiles the `gmw_slot_config_t` to the control packet.

The per-slot configuration let the user define, independently for each slot in a round:
- The number of retransmissions executed by the primitive (if relevant) (from 0 to 7). 
- The desired slot time index (from 0 to 7).  
Up to 8 different slot times can be used. 
- The desired primitive index (from 0 to 3).  
Helpers are defined in `gmw-conf-<platform>.h files to select the desired primitive
```
#define GMW_PRIM_DEFAULT           GMW_PRIM_GLOSSY     /* default Glossy */
#define GMW_PRIM_GLOSSY            0
#define GMW_PRIM_CHAOS             1
#define GMW_PRIM_STROBING          2
```

The settings from the `gmw_slot_config_t` section overwrite those from `gmw_config_t` section. This implies that if you use the per-slot configuration feature, you must define the desired settings for each slot in the `gmw_slot_config_t`.
