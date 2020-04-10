|Platform| Compilation command |
|:---|:---|
|TelosB   | make TARGET=sky |
|DPP-cc430   | make TARGET=dpp |

This simple Baloo protocol illustrates the utilization of the static control feature.

If `GMW_CONF_USE_STATIC_SCHED` and/or `GMW_CONF_USE_STATIC_CONFIG` are set in `project-conf.h`, the corresponding bytes of the control packet are _not sent_ by the host, which allows to save time and energy.

All nodes are then responsible to do all required updates of their copy of the control information. This is done via the `app_control_static_update` function, called in the `control_slot_post_callback`. That implies that there is a pre-agreed upon static control policy, henceforth the name of the feature.

In this example, useful information is contained in `user_byte` section of the control packet: it is used by all nodes to know what static update should be performed. Node alternates between different number of retransmissions for the data slots (2 and 4).

If either defines are unset, the middleware adapts: the control updates will be performed by the host as usual, using the `app_control_update` function in the `host_on_round_finished()` callback.

> **Note** When `GMW_CONF_USE_STATIC_CONFIG` is set to 1, neither the `gmw_config_t` nor the `gmw_slot_config_t` are included in the control packet, even if using the `GMW_CONTROL_SET_CONFIG(control)` and `GMW_CONTROL_SET_SLOT_CONFIG(control)`.
