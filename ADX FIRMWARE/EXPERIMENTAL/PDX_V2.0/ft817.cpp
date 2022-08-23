#include <Arduino.h>
#include "pdx_common.h"

#ifdef FT817
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
//*               DEFINITIONS SPECIFIC TO FT817 CAT PROTOCOL                                    *
//*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*

/**
   The CAT protocol is used by many radios to provide remote control to computers through
   the serial port.

   This is very much a work in progress. Parts of this code have been liberally
   borrowed from other GPL licensed works like hamlib.

   https://github.com/afarhan/ubitxv6/blob/master/ubitx_cat.cpp

   Note: This code was tested with WSJT-X 2.5.4 and Hamlib (rigctl) 4.3.1 in
   July-2022 by Dhiru (VU3CER).

   Reference: http://www.ka7oei.com/ft817_meow.html
*/

unsigned char doingCAT = 0;
bool txCAT             = false;        // turned on if the transmitting due to a CAT command
char inTx              = 0;                // it is set to 1 if in transmit mode (whatever the reason : cw, ptt or cat)
char isUSB             = 0;

static unsigned long rxBufferArriveTime = 0;
static byte rxBufferCheckCount          = 0;
static byte cat[5];
static byte insideCat                   = 0;
unsigned int skipTimeCount              = 0;
extern void startTx();
extern void stopTx();

// for broken protocol
#define CAT_RECEIVE_TIMEOUT 500

#define CAT_MODE_LSB            0x00
#define CAT_MODE_USB            0x01
#define CAT_MODE_CW             0x02
#define CAT_MODE_CWR            0x03
#define CAT_MODE_AM             0x04
#define CAT_MODE_FM             0x08
#define CAT_MODE_DIG            0x0A
#define CAT_MODE_PKT            0x0C
#define CAT_MODE_FMN            0x88
#define ACK                     0x00

byte setHighNibble(byte b, byte v) {
  // Clear the high nibble
  b &= 0x0f;
  // Set the high nibble
  return b | ((v & 0x0f) << 4);
}

byte setLowNibble(byte b, byte v) {
  // Clear the low nibble
  b &= 0xf0;
  // Set the low nibble
  return b | (v & 0x0f);
}

byte getHighNibble(byte b) {
  return (b >> 4) & 0x0f;
}

byte getLowNibble(byte b) {
  return b & 0x0f;
}

// Takes a number and produces the requested number of decimal digits, staring
// from the least significant digit.
void getDecimalDigits(unsigned long number, byte* result, int digits) {
  for (int i = 0; i < digits; i++) {
    // "Mask off" (in a decimal sense) the LSD and return it
    result[i] = number % 10;
    // "Shift right" (in a decimal sense)
    number /= 10;
  }
}

// Takes a frequency and writes it into the CAT command buffer in BCD form.
void writeFreq(unsigned long freq, byte* cmd) {
  // Convert the frequency to a set of decimal digits. We are taking 9 digits
  // so that we can get up to 999 MHz. But the protocol doesn't care about the
  // LSD (1's place), so we ignore that digit.
  byte digits[9];
  getDecimalDigits(freq, digits, 9);
  // Start from the LSB and get each nibble
  cmd[3] = setLowNibble(cmd[3], digits[1]);
  cmd[3] = setHighNibble(cmd[3], digits[2]);
  cmd[2] = setLowNibble(cmd[2], digits[3]);
  cmd[2] = setHighNibble(cmd[2], digits[4]);
  cmd[1] = setLowNibble(cmd[1], digits[5]);
  cmd[1] = setHighNibble(cmd[1], digits[6]);
  cmd[0] = setLowNibble(cmd[0], digits[7]);
  cmd[0] = setHighNibble(cmd[0], digits[8]);
}

// This function takes a frequency that is encoded using 4 bytes of BCD
// representation and turns it into an long measured in Hz.
//
// [12][34][56][78] = 123.45678? Mhz
unsigned long readFreq(byte* cmd) {
  // Pull off each of the digits
  byte d7 = getHighNibble(cmd[0]);
  byte d6 = getLowNibble(cmd[0]);
  byte d5 = getHighNibble(cmd[1]);
  byte d4 = getLowNibble(cmd[1]);
  byte d3 = getHighNibble(cmd[2]);
  byte d2 = getLowNibble(cmd[2]);
  byte d1 = getHighNibble(cmd[3]);
  byte d0 = getLowNibble(cmd[3]);
  return
    (unsigned long)d7 * 100000000L +
    (unsigned long)d6 * 10000000L +
    (unsigned long)d5 * 1000000L +
    (unsigned long)d4 * 100000L +
    (unsigned long)d3 * 10000L +
    (unsigned long)d2 * 1000L +
    (unsigned long)d1 * 100L +
    (unsigned long)d0 * 10L;
}

// This function is to falsify some readings performed
// into the volatile memory of a typical FT817 radio
void catReadEEPRom(void)
{
  byte temp0 = cat[0];
  byte temp1 = cat[1];
  cat[0] = 0;
  cat[1] = 0;

  switch (temp1)
  {
    case 0x45:
      if (temp0 == 0x03) {
        cat[0] = 0x00;
        cat[1] = 0xD0;
      }
      break;
    case 0x47: //
      if (temp0 == 0x03) {
        cat[0] = 0xDC;
        cat[1] = 0xE0;
      }
      break;
    case 0x55:
      // 0: VFO A/B  0 = VFO-A, 1 = VFO-B
      cat[1] = 0x00;
      break;
    case 0x57:
      cat[0] = 0xC0;
      cat[1] = 0x40;
      break;
    case 0x59:
      // http://www.ka7oei.com/ft817_memmap.html
      break;
    case 0x5C: // Beep Volume (0-100) (#13)
      cat[0] = 0xB2;
      cat[1] = 0x42;
      break;
    case 0x5E:
      cat[1] = 0x25;
      break;
    case 0x61: // Sidetone (Volume) (#44)
      cat[1] = 0x08;
      break;
    case 0x5F:
      cat[0] = 0x32;
      cat[1] = 0x08;
      break;
    case 0x60 : // CW Delay (10-2500 ms)
      // cat[0] = cwDelayTime;
      cat[1] = 0x32;
      break;
    case 0x62:
      cat[1] = 0xB2;
      break;
    case 0x63:
      cat[0] = 0xB2;
      cat[1] = 0xA5;
      break;
    case 0x64:
      break;
    case 0x67: // 6-0 SSB Mic (#46) Contains 0-100 (decimal) as displayed
      cat[0] = 0xB2;
      cat[1] = 0xB2;
      break;
    case 0x69: // FM Mic (#29) Contains 0-100 (decimal) as displayed
      break; // XXX
    case 0x78:
      if (isUSB)
        cat[0] = CAT_MODE_USB;
      else
        cat[0] = CAT_MODE_LSB;
      if (cat[0] != 0) cat[0] = 1 << 5;
      break;
    case 0x79:
      cat[0] = 0x00;
      cat[1] = 0x00;
      break;
    case 0x7A: // SPLIT
      break;
    case 0xB3:
      cat[0] = 0x00;
      cat[1] = 0x4D;
      break;

  }

  // send the data
  Serial.write(cat, 2);
  Serial.flush();
}

void setFrequency(unsigned long f);

void processCATCommand2(byte* cmd) {
  byte response[5];
  unsigned long f;

  int b, i, j, k, q, m;

  switch (cmd[4]) {
    case 0x01:
      // set frequency
      f = readFreq(cmd);
      // setFrequency(f);
      freq = f;
      response[0] = 0;
      Serial.write(response, 1);
      Serial.flush();

/*      
      b = setSlot(freq);
      // If a band change is detected switch to the new band
      if (b != Band_slot) { // band change
        Band_slot = b;
      }
      // Properly register the mode if the frequency implies a WSJT mode change
      // (FT8,FT4,JS8,WSPR)
      i = getBand(freq);
      if (i < 0) {
        break;
      }
      j = findSlot(i);
      if (j < 0 || j > 3) {
        break;
      }
      k = Bands[j];
      q = band2Slot(k);
      m = getMode(q, freq);
      break;
#ifdef DEBUG
      _INFOLIST("%s f=%ld band=%d slot=%d Bands=%d b2s=%d m=%d mode=%d\n", __func__, freq, i, j, k, q, m, mode);
#endif //DEBUG
      if (getWord(SSW, CWMODE) == false) {
        if (mode != m) {
          mode = m;
          Mode_assign();
        }
      }
      // If enabled change filter from the LPF filter bank
#ifdef QUAD // Set the PA & LPF filter board settings if defined
      x = band2QUAD(k);
      if (x != -1) {
        setQUAD(x);
      }
#endif //QUAD

*/

      break;

    case 0x02:
      // split on
      break;
    case 0x82:
      // split off
      break;

    case 0x03:
      writeFreq(freq, response); // Put the frequency into the buffer
      if (isUSB)
        response[4] = 0x01; // USB
      else
        response[4] = 0x00; // LSB
      Serial.write(response, 5);
      Serial.flush();
      break;

    case 0x07: // set mode
      if (cmd[0] == 0x00 || cmd[0] == 0x03)
        isUSB = 0;
      else
        isUSB = 1;
      response[0] = 0x00;
      Serial.write(response, 1);
      Serial.flush();
      // setFrequency(frequency);
      freq = f;
      break;

    case 0x08: // PTT On
      if (!inTx) {
        response[0] = 0;
        inTx = 1;
        setWord(&SSW, CATTX, true);
        switch_RXTX(HIGH);
      } else {
        response[0] = 0xf0;
      }
      Serial.write(response, 1);
      Serial.flush();
      Serial1.write("PTT ON!\n");
      Serial1.flush();
      break;

    case 0x88: // PTT OFF
      if (inTx) {
        inTx = 0;
        setWord(&SSW, CATTX, false);
        switch_RXTX(LOW);
      }
      response[0] = 0;
      Serial.write(response, 1);
      Serial.flush();
      Serial1.write("PTT OFF!\n");
      Serial1.flush();
      break;

    case 0x81:
      // toggle the VFOs
      response[0] = 0;
      Serial.write(response, 1);
      Serial.flush();
      break;

    case 0xBB: // Read FT-817 EEPROM Data
      catReadEEPRom();
      break;

    case 0xe7:
      // Get receiver status, we have hardcoded this as
      // as we don't support ctcss, etc.
      response[0] = 0x09;
      Serial.write(response, 1);
      Serial.flush();
      break;

    case 0xf7:
      {
        boolean isHighSWR = false;
        boolean isSplitOn = false;
        response[0] = ((inTx ? 0 : 1) << 7) +
                      ((isHighSWR ? 1 : 0) << 6) + // Hi swr off / on
                      ((isSplitOn ? 1 : 0) << 5) + // Split on / off
                      (0 << 4) + // dummy data
                      0x08; // P0 meter data
        Serial.write(response, 1);
        Serial.flush();
      }
      break;

    default:
      response[0] = 0x00;
      Serial.write(response[0]);
      Serial.flush();
  }

  insideCat = false;
}
#ifdef FT817
void serialEvent() {
  byte i;

  // Check Serial Port Buffer
  if (Serial.available() == 0) {                            // Set Buffer Clear status
    rxBufferCheckCount = 0;
    return;
  }
  else if (Serial.available() < 5) {                        // First Arrived
    if (rxBufferCheckCount == 0) {
      rxBufferCheckCount = Serial.available();
      rxBufferArriveTime = millis() + CAT_RECEIVE_TIMEOUT;  // Set time for timeout
    }
    else if (rxBufferArriveTime < millis()) {               // Clear Buffer
      for (i = 0; i < Serial.available(); i++)
        rxBufferCheckCount = Serial.read();
      rxBufferCheckCount = 0;
    }
    else if (rxBufferCheckCount < Serial.available()) {     // Increase buffer count, slow arrive
      rxBufferCheckCount = Serial.available();
      rxBufferArriveTime = millis() + CAT_RECEIVE_TIMEOUT;  // Set time for timeout
    }
    return;
  }

  // CAT DATA arrived
  for (i = 0; i < 5; i++)
    cat[i] = Serial.read();

  // Note: This code is not re-entrant!
  if (insideCat == 1)
    return;
  insideCat = 1;

  processCATCommand2(cat);
  insideCat = 0;
}
#endif //FT817
#endif //FT817
