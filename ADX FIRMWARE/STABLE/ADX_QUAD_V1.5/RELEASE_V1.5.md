# ADX is an Arduino based Digital Modes Transceiver authored by Barb (WB2CBA)


## Release V1.5 (Baseline) by Barb (WB2CBA), Dhiru (VU3CER) and Pedro (LU7DZ)

Baseline configuration supports the following sub-systems

- Optimized the band support to include 80,60,40,30,20,17,15,10 mts bands (8 bands) to match the definition of the QUAD filter board.
- ONEBAND (selects only one possible band, optative, activated with **#define ONEBAND 1** directive.
- EEPROM  (store data in EEPROM, enabled, activated with **#define EE 1** directive.
- CAT support (TS480 protocol), enabled, activated with the **#define CAT 1** directive. Serial port is the Arduino board USB serial.
- WDT support (watchdog), enabled, activated with the **#define WDT 1** directive.
- Support for QUAD multiband board, enabled, activated with the **#define QUAD 1** directive.
- ATU reset control, when band is changed a small pulse is generated, optative, activated with the **#define ATUCTL 1** directive.
- Miscelaneous
	- CW mode, when activated the TX mode shift frequency by CW_SHIFT, default QRP frequencies can be changed thru CAT commands.
	- RESET, reset EEPROM when BUILD is changed, optative, activated with the **#define RESET 1** directive.
	- CAL_RESET, reset the calibration when BUILD is changed, optative, activated with the **#define CAL_RESET 1** directive.
	- Minor memory & code optimizations, bug removal.

### Hardware mods

-	Straight keyer connected from SPKR(tip) to TX pushbutton.
-	Cut wire between SPKR(tip) and SPKR(sleeve), audio signal will be taken from sleeve only.

### Software mods for CAT support

-	CAT support works Ok with almost any program based on HamLib, in particular with FLRig.
-	WSJT-X(Z) direct CAT command are mostly working but still in test, if it experiences problems as a workaround that works Ok is to use FLRig (see below).

#### CAT TS-480 emulation configuration
        - Access WSJT-X Settings (File/Settings/Radio).
	- Select "Rig" as "Kenwood TS-480".
	- Select Serial port the one that your computer assigned to the Arduino Nano (typically COMx on Windows and /dev/tty* on Linux)
	- Select 19200 as the speed.
	- Select 8 bits, 2 stop bits, handshake none.
 	- Select PTT method either VOX or CAT.
	- Select Polling time as 6 secs.
	- Press CAT Test, it should go green after a while.

#### CAT FlRig configuration (for TS480)
	- Download and install FLRig from the [W1HKJ site] (http://www.w1hkj.com/files/flrig/) according with your Operating System
		- Configuration tested with Win10, Raspbian Debian and MacOs.
	- Configure ADX CAT suppport at *config/setup/transceiver*.
		- *Rig*: TS-480HX.
		- *Update*: (whatever the USB serial port of the Arduino Nano is recognized as).
		- *Baud*: 19200
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











