#!/bin/bash

# check the file name
FILE=$1

if [ ! -f "$FILE" ]; then
	echo "file ${FILE} not found!"
	exit
fi

if [[ $FILE == *.hex ]]; then
	HEXFILE=$FILE
else
	# convert to intel hex
	HEXFILE=${FILE}.hex
	objcopy ${FILE} -O ihex ${HEXFILE}
	echo "hex file generated"
fi

# if a 2nd argument is supplied, treat it as the port
if [ "$#" -gt 1 ]; then
	PORT=$2
else
	PORT=$(motelist -c | grep "tmote sky" | awk '{split($0,a,","); print a[2]}')
fi
echo "programming Tmote Sky on port ${PORT}..."

# to create a hex file from an elf (exe) file:
#objcopy ${HEXFILE} -O ihex ${HEXFILE}.hex

./msp430-bsl-linux --telosb -c $PORT -r
./msp430-bsl-linux --telosb -c $PORT -e && sleep 1
./msp430-bsl-linux --telosb -c $PORT -I -p $HEXFILE && sleep 1
./msp430-bsl-linux --telosb -c $PORT -r
