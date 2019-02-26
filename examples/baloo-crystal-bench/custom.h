#ifndef TESTBED_CUSTOM_H_
#define TESTBED_CUSTOM_H_

typedef struct 
{
  uint8_t n_tx_max;     /* Number of retransmissions in a Glossy flood */
  uint8_t n_empty_ta;   /* Number of empty TA pairs to trigger the end of
                         * Crystal rounds */
  uint8_t tx_power;     /* Power setting (in level, see
                         * TxPowerSettings.txt) */
} custom_config_t;

extern volatile custom_config_t custom_cfg;

#endif

