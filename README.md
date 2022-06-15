# ADX is an Arduino based Digital Modes Transceiver.

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

## Mods by LU7DZ (version 1.2e)

Version ***"e"*** stands for *"experimental"*, ***use it as your own headache risk*** as it is not the firmware supported by WB2CBA.

-	Code re-organization/Modularization.
-	CAT support (using Arduino Nano USB-Serial driver), emulates a TS440 CAT protocol, uses FLRig as a HamLib server for WSJT-X.
-	Band expansion (still 4 bands supported), but all HF-6m bands can be selected
-	EEPROM write cycle to reduce wear and tear.
-	Watchdog reset
-	CW transceiver
	- Tunning capability (+/- 20 KHz).
	- (Somewhat crude) LED tunning indication.
	- Full break-in support.
	- CW shift (600 Hz).
	- CW step +/-500 Hz.


### Hardware mods

-	Straight keyer connected from SPKR(tip) to TX pushbutton.
-	Cut wire between SPKR(tip) and SPKR(sleeve), audio signal will be taken from sleeve only.

### Software mods for CAT support

-	CAT support works Ok with almost any program based on HamLib, in particular with FLRig.
-	WSJT-X(Z) direct CAT command is still on the works, as a workaround it works Ok thru FLRig.
	- Download and install FLRig from the [W1HKJ site] (http://www.w1hkj.com/files/flrig/) according with your Operating System
		- Configuration tested with Win10, Raspbian Debian and MacOs.
	- Configure ADX CAT suppport at *config/setup/transceiver*.
		- *Rig*: TS-480HX.
		- *Update*: (whatever the USB serial port of the Arduino Nano is recognized as).
		- *Baud*: 115200
		- (check) *2-Stop bit*.
		- *Retries*: 2.
		- *Poll intvl*: 465.
		- Leave all other values as the proposed default.
	- Re-launch FlRig connected with the ADX transceiver running ***this*** firmware version (other versions might behave in a different way), check:
		- Frequency on both OFV reflect the one configured with ADX.
		- Mode USB.
		- PTT would put the ADX in transmit when activated (***ensure an antenna or a dummy load is connected not to blow your finals***).
	- Launch WSJT-X, go to configure Radio, select:
		- *Radio*: FLRig - FLRig.
		- *Poll interval*: 5 secs
		- *PTT method*: CAT
		- *Mode*: None or USB
		- *Split operation*: Fake split
	- Configure the remainingn of the configuration values for WSJT-X as per the normal ADX operation without CAT.


```
Operation with any program other than WSJT-X is possible as long as it support CAT commands thru the HamLib library.
In that case configure the transceiver in a similar way than explained above for FLRig
```











