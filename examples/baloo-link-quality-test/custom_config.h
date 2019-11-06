#ifndef _CUSTOM_CONFIG_H_
#define _CUSTOM_CONFIG_H_

/* The variable in this file are patched after compilation
 * -> They need to be set as global variable to be used in the configuration
 * of Baloo
 * (By default, Baloo uses macros instead of variables)
 * */

extern uint16_t payload_length;
extern uint16_t randomseed;
extern uint16_t host_id;
extern uint8_t  rf_channel;

#endif /* _CUSTOM_CONFIG_H_ */

