# Revision 1 Notes

## Power Supply

The DCDC magic box is huge and electrically noisy. A lot to be gained from just 
using a larger transformer and a normal DC supply with a rectifier, reservoir 
caps, and a linear regulator.

Add a bleed resistor to the X2 cap. Consider a power entry module with built in 
line filter, or a better line filter.

## Monitoring Transformer

Distortion at low output loads is awful. Probably can be fixed by ensuring a 
suitable transformer with some load on the secondary, but maybe it would be 
nicer to do an isolated ADC and comparator. Timing performance might suffer.

## GPS Clock Recovery

Might be able to do without the CS2100 and just pump the GPS directly into the 
STM32. It doesn't seem all that jittery really and saves a lot of money and 
board space (CS2100 + TCXO + TCXO LDO + TCXO inverter).

## Ethernet

Swap to a magjack. Christ.

## LCD connector

Get the top mounted one next time.

## SMA connector

Get a longer, panel-mount screw one.
