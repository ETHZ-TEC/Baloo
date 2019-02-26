#ifndef TESTBED_H_
#define TESTBED_H_

// GPIO pin used to signal the availability of data in the EEPROM
#define EVENT_PIN 6

#include "config.h"

// Helper functions to print the input parameters injected by the competition's testbed
void
print_testbed_pattern(pattern_t* p)
{
	uint8_t i;
	printf("    Traffic pattern: ");
	switch(p->traffic_pattern)
	{
		case 0: printf("unused\n");
			break;
		case 1: printf("P2P\n");
			break;
		case 2: printf("P2MP\n");
			break;
		case 3: printf("MP2P\n");
			break;
		case 4: printf("MP2MP\n");
			break;
		default: printf("Unknown\n");
	}
	if( (p->traffic_pattern>0) && (p->traffic_pattern <=4))
	{
		printf("    Sources:\n");
		for(i=0;i<TB_NUMNODES;i++)
		{
			if(p->source_id[i]!=0)
			printf("      %d: %d\n",i,p->source_id[i]);
		}
		printf("    Destinations:\n");
		for(i=0;i<TB_NUMNODES;i++)
		{
			if(p->destination_id[i]!=0)
			printf("      %d: %d\n",i,p->destination_id[i]);
		}
		if(p->periodicity==0)
		{
			printf("    Aperiodic Upper: %lu\n",p->aperiodic_upper_bound);
			printf("    Aperiodic Lower: %lu\n",p->aperiodic_lower_bound);
		}
		else
		{
			printf("    Period: %lu\n",p->periodicity);
		}

		printf("    Message Length: %d\n",p->msg_length);
		printf("    Message OffsetH: %d\n",p->msg_offsetH);
		printf("    Message OffsetL: %d\n",p->msg_offsetL);
	}
	printf("\n");

}

void
print_testbed_config(config_t* cfg)
{
	printf("Testbed configuration:\n");
	uint8_t i;
	for(i=0;i<TB_NUMPATTERN;i++)
	{
	        printf("  Pattern %d:\n",i);
		print_testbed_pattern(&(cfg->p[i]));
	}
}

#endif

