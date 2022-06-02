ADX is an Arduino based Digital Modes Transceiver.

- ADX - is abbreviation for Arduino Digital Xceiver.


- ADX is a mono band (actually quad band) digital modes optimized HF transceiver that can cover four pre-programmed bands one band at a time by swapping Band LPF Modules. 
It can work on 80m, 40m, 30m,20m, 17m and 15m bands and can operate on four of the most popular digital modes, FT8, FT4, JS8call and WSPR. 
No 12m and 10m bands as it stretches RX IC CD2003GP to it’s limits as it is designed for AM band reception. Well I can’t complain for a 30 cents receiver IC! 


My goal with this project is to design a simple HF Transceiver optimized for operating on Digital modes:
-	Simple to procure – meaning not effected by chip shortage
-	Simple to build – 2 modules, 2 IC’s and 4 Mosfets!
-	Simple to setup and tune – One simple calibration procedure is all needed.
-	Simple to operate – Plug in ADX MIC to soundcard MIC input and ADX SPK to PC soundcard speaker input and we are good to go with any digital modes Software.
-	Dirt Cheap – Costs less than 25$ to get all parts and PCB if we exclude ridiculous shipping costs!

- For ease of following which parts to solder while building Main board of ADX, Intercative BOM File - ibom_ADX.html file is a great help. Just run that file on Microsoft edge and follow each highlighted component and solder one by one. This interactive BOM was prepared by Gilles DELPECH - F1BFU. Thank you Gilles!

## Mods by LU7DZ (version 1.2t)

-	Code re-organization/Modularization.
-       CAT support (using Arduino Nano USB-Serial driver), emulates a TS440 CAT protocol.
-	Band expansion (still 4 bands supported), but all HF-6m bands can be selected
-	EEPROM write cycle to reduce wear and tear.
-	Watchdog reset
-	CW transceiver
	- Tunning capability (+/- 20 KHz).
	- (Somewhat crude) LED tunning indication.
	- Full break-in support.
	- CW shift (600 Hz).

### Hardware mods

-	Straight keyer connected from SPKR(tip) to TX pushbutton.
-	Cut wire between SPKR(tip) and SPKR(sleeve), audio signal will be taken from sleeve only.
