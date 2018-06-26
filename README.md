# lowpower-i2c-sensor
Another ATMega328PB based wireless sensor, designed for low power consumption

Currently testing and optimizing for 1Mhz operation, in order to be in Safe operating voltage without activating BOD (Vcc >= 1.8V). (4Mhz is not an option yet, no external oscillator forseen in design, so possible options are 1 or 8 Mhz)

using TPL5010 for low power consumption during sleep
Delay between wake-ups is governed by R2 & R4. Default { R2 = 29.4KOhm, R4 = not soldered } == 120.5-121 sec. 29.35KOhm is calculated for 2 min per datasheet (check it for possible values and combinations)

STM1061N for low battery voltage detection. 
Currently testing with 1.9v threshold (STM1061N19WX6F)
On the software side detection occurs using a pin CHANGE interrupt, in order to detect low voltage during the whole operation of the sensor (except sleep mode)

RFM69W for wireless communication, 45mA @ +13dBm. 
