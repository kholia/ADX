# ADX is an Arduino based Digital Modes Transceiver authored by Barb (WB2CBA)

RELEASE NOTES Version 1.2e (experimental)

## Mods by LU7DZ (version 1.2e)

Version ***"e"*** stands for *"experimental"*, ***use it as your own headache risk*** as it is not the firmware supported by WB2CBA.

-	Code re-organization/Modularization.
-	CAT support (using Arduino Nano USB-Serial driver), emulates a TS480 CAT protocol, uses FLRig as a HamLib server for WSJT-X (activated with the **#define CAT 1** and **define TS480 1** statements, default: enabled).
-	Band expansion (still 4 bands supported), but all HF-6m bands can be selected
-	EEPROM write cycle to reduce wear and tear.
-	Watchdog reset
-	CW support (activated with the **define CW 1** sentence, default: disabled).
	- On activation tune the QRP calling frequency of the band currently selected.
	- Activation marked by two central LED lighting.
	- Full break-in support.
	- CW shift (600 Hz).
	- Tuning thru CAT.
	- Straight keyer supported.


## Mods by LU7DZ (version 1.3e)

- Support for QUAD (4 bands) switching scheme (activaded with **#define QUAD 1** sentence, default: enabled).
- Support for external ATU initialization/reset, line D5 would briefly (200 mSecs) flash a high on band changes (activated with the **define ATU 1** sentence, default: enabled).
- Improved CAT support for WSJT-X (TS-480 emulation), overall stability improved.
- Minor memory & code optimizations, bug removal.
- 6 meters band removed from slot[][] options as it's unlikely the receiver can be moved that high, 10m works fine though


## Mods by LU7DZ (version 1.4e)

- Support for IC-746 CAT Protocol as suggested by Alan (AG7XS) because of being lighter and faster, TS480 still supported (activated with the **#define IC746 1** statement, default: disabled). 
	- Consistency rules applies, if both TS480 and IC746 are activated then TS480 is disabled.
- Minor memory & code optimizations, bug removal.


## Mods by LU7DZ (version 1.5e)

- Optimized the band support to include 80,60,40,30,20,17,15,10 mts bands (8 bands) to match the definition of the QUAD filter board.
	- QUAD control thru I2C: 80(1),60(2),40(4),30(8),20(16),17(32),15(64) and 10(128).
	- Bug fixes in the QUAD board activation from the test performed.
- An ANTI-VOX feature was added, when the transceiver is set into reception thru a CAT command a forced mute of the VOX mechanism is established in order to prevent that residual noise keep the transceiver in TX mode. (activated thru the **#define ANTIVOX 1** statement, default: enabled).
- Changed the support for frequency and band changes when using CAT (both TS480 and IC746 protocols).
	- When a frequency change is made:
		- A band change is performed first, validation the band is supported is performed.
		- A mode change is performed based on standard frequencies, the mode is automatically switch to FT8,FT4,JS8 and WSPR based on standard frequency for each band).
		- The board LED are changed to reflect mode changes.
		- This has no effect if the current mode is CW.
- Now the last WSJT-X mode prior to the switch into CW is stored and recovered when CW is de-activated.
- Watchdog inhibits TX to be on for more than two consecutive minutes (activated with the **#define WDT 1** statement, default: enabled).
- Support for CAT FT817 (experimental) (activated with the **#define FT817 1** statement, default: disabled). This initial implementation is still incomplete and might
  require FlRig to be used as an intermediate host. Settings:
	COM Port: As assigned.
	BAUD: 19200
	8N1
- Serial (simple) command line configuration facility (activated with the **#define TERMINAL 1** statement, default: disabled).
	- Enter the terminal mode by held the UP key pressed while booting the board.
		- Terminal parameters are 115200 8N2.
	- Commands without arguments display the current value.
	- Commands with one argument set the parameter to that value (no commitment to EEPROM is made).
	- Exit the command terminal with "quit".

```
	ADX Transceiver Firmware V1.5e Build(nnn)
	>
	>

        Available commands are
		bt		debouncing delay in mSecs
		st		time to consider the pulse a long one in mSecs
		vxm		VOX max try (TX detection).
		vxc		VOX count max (TX detection)
		mbl		Blink interval in mSecs		

		save		Save current values into EEPROM
		reset		Reset EEPROM values to default
		quit		End session and reboot
		help		List all available commands
		
		** only if ATUCTL is defined **
		atu		pin for ATU pulse
		atd		duration of ATU reset pulse in mSecs


		** only if ANTIVOX is defined **
		vxt		Anti-VOX delay in mSecs


		** only if EEPROM support is defined **		
		eet		EEPROM save timeout in mSecs
		list		List EEPROM contents



```
- (fix) Calibration, proper handling of the DOWN key at bootup to enter calibration, then CLK2 is the output with 1 MHz
- (fix) General fixes related to the QUAD filter selection and how changes in frequency are shown in the band/mode LED.
- Build number is added to the firmware (#define BUILD xxx), when the build number set differs from the one stored in EEPROM values are reset to default.
- Changes in the EEPROM management, all values supported by the configuration terminal are now stored in EEPROM.
- Changes in the way to enter calibration mode at the start of the firmware to make it more reliable.
- CAT TS480 is set now as Speed: 19200 for commonality and compatibility with existing software. 
- Calibration, if the UP or DOWN buttons are kept pressed the calibration offset is changed continuously few times per second.
- (fix) While in band change mode the TX button is kept blinking to signal that.
- A RESET mode, if in BAND CHANGEN mode the TX button is kept pressed for more than 2 secs the board is reset, EEPROM content is preserved (enabled with #include RESET 1 directive).
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
	- Select 115200 as the speed.
	- Select 8 bits, 2 stop bits, handshake none.
 	- Select PTT method either VOX or CAT.
	- Select Polling time as 6 secs.
	- Press CAT Test, it should go green after a while.
#### CAT ICOM-746 emulation configuration
        - Access WSJT-X Settings (File/Settings/Radio).
	- Select "Rig" as "Icom IC-746".
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











