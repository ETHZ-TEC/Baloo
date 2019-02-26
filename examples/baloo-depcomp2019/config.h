#ifndef CONFIG_H_
#define CONFIG_H_

#define TB_NUMPATTERN   8
#define TB_NUMNODES     8

typedef struct
{
  uint8_t traffic_pattern;             // 0:unused, 1:p2p, 2:p2mp, 3:mp2p, 4: mp2mp
  uint8_t source_id[TB_NUMNODES];      // Only source_id[0] is used for p2p/p2mp
  uint8_t destination_id[TB_NUMNODES]; // Only destination_id[0] is used for p2p/mp2p
  uint8_t msg_length;                  // Message length in bytes in/to EEPROM
  uint8_t msg_offsetH;                 // Message offset in bytes in EEPROM (high byte)
  uint8_t msg_offsetL;                 // Message offset in bytes in EEPROM (low byte)

  uint32_t periodicity;                // Period in ms (0 indicates aperiodic traffic)
  uint32_t aperiodic_upper_bound;      // Upper bound for aperiodic traffic in ms
  uint32_t aperiodic_lower_bound;      // Lower bound for aperiodic traffic in ms
} pattern_t;

typedef struct
{
  pattern_t p[TB_NUMPATTERN];          // Up to TB_NUMPATTERN parallel configurations
} config_t;

extern volatile config_t cfg;

#endif

