# NRF24L01+ driver
This driver is designed to be used in an environment that provides [MBED](http://mbed.org)-style classes.

## Static mode
It can also be configured in static mode for small devices (AVR).

#Pinout
## Black NRF24L01+ module from Ebay:

```
 GND (marked) | 1 | 2 | VDD
	       CE | 3 | 4 | CSN
	      SCK | 5 | 6 | MOSI
	     MISO | 7 | 8 | IRQ
```


## nRF24L01+ Mini module pinout:
Pin numbering starts at the side closer to the white dot on the PCB
1. +3V3
2. GND
3. CE
4. CSN
5. SCK
6. MOSI
7. MISO
8. IRQ