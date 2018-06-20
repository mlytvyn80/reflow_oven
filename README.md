# Reflow owen

Description

## Getting Started

The firmware can be compiled using make utility. 
```
make all
```

Some more make targets are supported, namely all, clean, coff extcodd program, debug.
Please consult the Makefile for usage details.


### Prerequisites

In order to compile the HEX-file you need to install the AVR toolchain. 

In Ubuntu/Debian Linux this can be done using the following command:
```
sudo apt-get install gcc-avr
```

To be able to flash the firmware you will need a driver AVR MCU programmer (for example [usbasp](https://www.fischl.de/usbasp/) ) and the 
driving program like [avrdude](https://www.nongnu.org/avrdude/).

avrdude is included in the most of Linux distribution. In Ubuntu/Debian you can issue:
```
sudo apt-get install avrdude
```

### Flashing the firmware
Connect the ISP head with your AVR MCU programmer, programmer to PC and flash the compiled HEX-file:
```
avrdude -c usbasp -p m32u4 -U flash:w:ovencon.hex
```

In order to configure MCU to work with external oscillator (in our case 16MHz quartz), it is necessary to
setup fuse bits.

It can be done by command:
```
avrdude -c usbasp -p m32u4 -U hfuse:w:0xdd:m -U lfuse:w:0xde:m -U efuse:w:0xcf:m
```


## Built With

* [GNU AVR toolchain](http://www.dropwizard.io/1.0.2/docs/) - The web framework used
* [avrdude](https://maven.apache.org/) - Dependency Management
* [usbasp](https://www.fischl.de/usbasp/) - AVR MCU programmer

## Authors

* **Billie Thompson** - *Initial work* - [PurpleBooth](https://github.com/PurpleBooth)
* **Oleg Artamonov** - *PCB Design*  - [olegart.ru](http://olegart.ru/wordpress/reflow-soldering/)
* **Mykhailo Lytvyn** - Collect all necessary build-related information together.


## License

The code is distributed "AS IS" under original License by Dan Strother (see header comments in source code).


