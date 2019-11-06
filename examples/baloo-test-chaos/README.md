|Platform| Compilation command |
|:---|:---|
|TelosB 
  | make TARGET=sky |
|DPP-cc430 
  | _Not available for this platform_ |

This simple Baloo protocol illustrates the utilization of Chaos as communication primitive. 

In each round, nodes pseudo-randomly generate a boolean value,
which is passed as payload to Chaos.
The custom `chaos_set_payload_cb()` use this boolean value to act
on the actual packet payload: if the boolean is true, the bit
corresponding to the node's position in CHAOS_CONF_NODE_ID_MAPPING
is set to 1 (or 0 otherwise).
This mimics a quick aggregation of ACKs from multiple nodes.

To use multiple communication primitives, the `#define GMW_CONF_USE_MULTI_PRIMITIVES` must be set to 1.

To use Choas as a primitive, the following defines must be configured
|#define| Rationale |
|:---|:---|
|CHAOS_CONF_NUM_NODES 
  | Number of nodes that contributes to the Chaos flood|
|CHAOS_CONF_NODE_ID_MAPPING 
  | Ordering of nodes to assign them a bit-mapping |
|CHAOS_CONF_SHARED_PAYLOAD 
  | When set to 1, the payload area of the Chaos
packet is shared among all participating nodes |
|CHAOS_CONF_PAYLOAD_LEN 
  | Size of the payload area of a Chaos packet.
If CHAOS_CONF_SHARED_PAYLOAD is set to 0 the available payload per node is CHAOS_CONF_PAYLOAD_LEN / CHAOS_CONF_NUM_NODES|
| CHAOS_CONF_SET_CUSTOM_PAYLOAD
  | When set to 1, Chaos uses a custom agregation function: `chaos_set_payload_cb()`.
  When set to 0, Chaos simply copies each node's payload in the payload in their respective place (according to CHAOS_CONF_NODE_ID_MAPPING). |
|CHAOS_CONF_USE_TIMER_ISR 
  | Set to 0 enable the multiplexing of the timer ISR (`sky` platform only)|
|GLOSSY_CONF_USE_TIMER_ISR 
  | Set to 0 enable the multiplexing of the timer ISR (`sky` platform only)|
