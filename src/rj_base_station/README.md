# RJ Base Station

## Description

The RJ Base Station is meant to run on the Raspberry Pi Base station to send
commands to the robots and receive their statuses.

## Installation

On a fresh Raspberry Pi, ensure the SPI peripherals are enabled.  Specifically,
make sure the following lines are present in `/boot/config.txt`:

```
dtoverlay=spi1-3cs
```
```
```


## Pinout

The following pinout is used for the communication between the Raspberry Pi and
the nRF24L01+ transcievers:
