# Collection of technical information regarding the competition scenario

* Bootstrapping
 - no packet in the first 60s
 - energy does not count in that phase
 - There can be interference in that phase

* Message interface
 - Message presence signaled on P2.X where X=EVENT_PIN (#define in testbed.h)
 - Both source and dest use the same pin
 - Option: ISR on P2 to trigger the read operation. Same ISR provided in `i2c-test.c`  
If used, call ISR on falling edge (see slide 28).  
**REMIND** By default ISR already defined in `sky/dev/button-sensor.c`. To remove first. 
 - Same for destination node. See example code.
 - Latency measured between falling edges.
 - **Important** Read and write operations take time! **Do not write more than once every 20ms** to give the observer module time to read the content! 
 - Option for optimization: _watch the I2C clock (SCL) for activity to ensure data that has been read_

* expected layouts
 - For both scenario, Layout 1 is supposed to be representative of the final evaluation scenario.

* GPIO pin tracing. To add into our `gpio.h` file if we want to use them.
 - ADC0 		GPIO 17
 - ADC1 		GPIO 4
 - ADC2/GIO1 	GPIO 18
 - ADC3/GIO0 	GPIO 27
 - ADC7/SVSin 	GPIO 22
 - GIO2 		GPIO 23
 - GIO3/SVSout 	GPIO 24
 - DAC0/ADC6 	GPIO 25

On Grafana, the number 'GPIO pins' displays the bitwise OR of the 8 pins (eg set to 255 to show the 8 pins).

* Performance metrics
 - Everything is computed and available directly in the testbed backend!

* Jamming
 -  the jamming pattern is probabilistic in order to avoid engineered solutions

# Notes from initial experiments
- The precision of the GPIO timestamping is 1 ms
