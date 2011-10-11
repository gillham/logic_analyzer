/*
 *
 * SUMP Protocol Implementation for Arduino boards.
 *
 * Copyright (c) 2011 Andrew Gillham
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY ANDREW GILLHAM ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANDREW GILLHAM BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *	$Id: logic_analyzer.pde,v 1.17 2011-08-04 02:31:01 gillham Exp $
 *
 */

/*
 * This Arduino sketch implements a SUMP protocol compatible with the standard
 * SUMP client as well as the alternative client from here:
 *	http://www.lxtreme.nl/ols/
 *
 * This SUMP protocol compatible logic analyzer for the Arduino board supports
 * 5 channels consisting of digital pins 8-12, which are the first 5 bits (0-4)
 * of PORTB. Arduino pin 13 / bit 5 is the Arduino LED, bits 6 & 7 are the
 * crystal oscillator pins.
 * Uncomment CHAN5 below if you want to use the LED pin as an input and have
 * 6 channels.
 *
 * On the Arduino Mega board 8 channels are supported and 7k of samples.
 * Pins 22-29 (Port A) are used by default, you can change the 'CHANPIN' below
 * if something else works better for you.
 *
 * NOTE:
 * If you are using the original SUMP client, or using the alternative client
 * without the device profiles, then you will get a "device not found" error.
 * You must DISABLE the Arduino auto reset feature to use this logic analyzer
 * code. There are various methods to do this, some boards have a jumper,
 * others require you to cut a trace.  You may also install a *precisely*
 * 120 Ohm resistor between the reset & 5V piins.  Make sure it is really
 * 120 Ohm or you may damage your board.
 * It is much easier to use the alternative SUMP client from here:
 *      http://www.lxtreme.nl/ols/
 *
 * The device profiles should be included with this code.  Copy them to the
 * 'plugins' directory of the client.  The location varies depending on the
 * platform, but on the mac it is here by default:
 * /Applications/LogicSniffer.app/Contents/Resources/Java/plugins
 *
 * To use this with the original or alternative SUMP clients,
 * use these settings:
 * 
 * Sampling rate: 1MHz (or lower)
 * Channel Groups: 0 (zero) only
 * Recording Size:
 *    ATmega186:  532 (or lower)
 *    ATmega328:  1024 (or lower)
 *    ATmega2560: 7168 (or lower)
 * Noise Filter: doesn't matter
 * RLE: disabled (unchecked)
 *
 * Triggering is still a work in progress, but generally works for samples
 * below 1MHz.  1MHz works for a basic busy wait trigger that doesn't store
 * until after the trigger fires.
 * Please try it out and report back.
 *
 * Release: v0.04 August 3, 2011.
 *
 */

/*
 * Function prototypes so this can compile from the cli.
 * You'll need the 'arduino-core' package and to check the paths in the
 * Makefile.
 */

void triggerMicro(void);
void captureMicro(void);
void captureMilli(void);
void getCmd(void);
void setupDelay(void);
void blinkled(void);
void get_metadata(void);
void debugprint(void);
void debugdump(void);

/*
 * Uncomment CHAN5 to use it as an additional input on a normal Arduino.
 * You'll need to change the number of channels in the device profile as well.
 *
 * Arduino device profile:      ols.profile-agla.cfg
 * Arduino Mega device profile: ols.profile-aglam.cfg
 */
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define CHANPIN PINA
#define CHAN0 22
#define CHAN1 23
#define CHAN2 24
#define CHAN3 25
#define CHAN4 26
#define CHAN5 27
#define CHAN6 28
#define CHAN7 29
#else
#define CHANPIN PINB
#define CHAN0 8
#define CHAN1 9
#define CHAN2 10
#define CHAN3 11
#define CHAN4 12
//#define CHAN5 13
#endif
#define ledPin 13

/* XON/XOFF are not supported. */
#define SUMP_RESET 0x00
#define SUMP_ARM   0x01
#define SUMP_QUERY 0x02
#define SUMP_XON   0x11
#define SUMP_XOFF  0x13

/* mask & values used, config ignored. only stage0 supported */
#define SUMP_TRIGGER_MASK 0xC0
#define SUMP_TRIGGER_VALUES 0xC1
#define SUMP_TRIGGER_CONFIG 0xC2

/* flags are ignored. */
#define SUMP_SET_DIVIDER 0x80
#define SUMP_SET_READ_DELAY_COUNT 0x81
#define SUMP_SET_FLAGS 0x82

/* extended commands -- self-test unsupported, but metadata is returned. */
#define SUMP_SELF_TEST 0x03
#define SUMP_GET_METADATA 0x04

/* ATmega186:  532 (or lower)
 * ATmega328:  1024 (or lower)
 * ATmega2560: 7168 (or lower)
 */
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  #define DEBUG_CAPTURE_SIZE 7168
  #define CAPTURE_SIZE 7168
#elif defined(__AVR_ATmega328__)
  #define DEBUG_CAPTURE_SIZE 1024
  #define CAPTURE_SIZE 1024
#else
  #define DEBUG_CAPTURE_SIZE 532
  #define CAPTURE_SIZE 532
#endif

#define DEBUG
#ifdef DEBUG
#define MAX_CAPTURE_SIZE DEBUG_CAPTURE_SIZE
#else
#define MAX_CAPTURE_SIZE CAPTURE_SIZE
#endif /* DEBUG */

/*
 * SUMP command from host (via serial)
 * SUMP commands are either 1 byte, or for the extended commands, 5 bytes.
 */
int cmdByte = 0;
byte cmdBytes[5];

#ifdef DEBUG
byte savebytes[128];
int savecount = 0;
#endif /* DEBUG */

byte logicdata[MAX_CAPTURE_SIZE];
unsigned int logicIndex = 0;
unsigned int triggerIndex = 0;
unsigned int readCount = MAX_CAPTURE_SIZE;
unsigned int delayCount = 0;
unsigned int trigger = 0;
unsigned int trigger_values = 0;
unsigned int useMicro = 0;
unsigned int delayTime = 0;
unsigned long divider = 0;

void setup()
{
  Serial.begin(115200);

  /*
   * set debug pin to output right away so it settles.
   * this gets toggled during sampling as a way to measure
   * the sample time.  this is used during development to
   * properly pad out the sampling routines.
   */
  DDRD = DDRD | B10000000; /* debug measurement pin */

  pinMode(CHAN0, INPUT);
  pinMode(CHAN1, INPUT);
  pinMode(CHAN2, INPUT);
  pinMode(CHAN3, INPUT);
  pinMode(CHAN4, INPUT);
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  pinMode(CHAN5, INPUT);
  pinMode(CHAN6, INPUT);
  pinMode(CHAN7, INPUT);
  pinMode(ledPin, OUTPUT);
#else
#ifdef CHAN5
  pinMode(CHAN5, INPUT);
#else
  pinMode(ledPin, OUTPUT);
#endif /* CHAN5 */
#endif /* Mega */
}

void loop()
{
  int i;

  if (Serial.available() > 0) {
    cmdByte = Serial.read();
    switch(cmdByte) {
    case SUMP_RESET:
      /*
       * We don't do anything here as some unsupported extended commands have
       * zero bytes and are mistaken as resets.  This can trigger false resets
       * so we don't erase the data or do anything for a reset.
       */
      break;
    case SUMP_QUERY:
      /* return the expected bytes. */
      Serial.print('1', BYTE);
      Serial.print('A', BYTE);
      Serial.print('L', BYTE);
      Serial.print('S', BYTE);
      break;
    case SUMP_ARM:
      /*
       * Zero out any previous samples before arming.
       * Done here instead via reset due to spurious resets.
       */
      for (i = 0 ; i < MAX_CAPTURE_SIZE; i++) {
        logicdata[i] = 0;
      }
      /*
       * depending on the sample rate we need to delay in microseconds
       * or milliseconds.  We can't do the complex trigger at 1MHz
       * so in that case (delayTime == 1 and triggers enabled) use
       * captureMicro() instead of triggerMicro().
       */
      if (useMicro) {
        if (trigger && (delayTime != 1)) {
          triggerMicro();
        } 
        else {
          captureMicro();
        }
      } 
      else {
        captureMilli();
      }
      break;
    case SUMP_TRIGGER_MASK:
      /*
       * the trigger mask byte has a '1' for each enabled trigger so
       * we can just use it directly as our trigger mask.
       */
      getCmd();
      trigger = cmdBytes[0];
      break;
    case SUMP_TRIGGER_VALUES:
      /*
       * trigger_values can be used directly as the value of each bit
       * defines whether we're looking for it to be high or low.
       */
      getCmd();
      trigger_values = cmdBytes[0];
      break;
    case SUMP_TRIGGER_CONFIG:
      /* read the rest of the command bytes, but ignore them. */
      getCmd();
      break;
    case SUMP_SET_DIVIDER:
      /*
       * the shifting needs to be done on the 32bit unsigned long variable
       * so that << 16 doesn't end up as zero.
       */
      getCmd();
      divider = cmdBytes[2];
      divider = divider << 8;
      divider += cmdBytes[1];
      divider = divider << 8;
      divider += cmdBytes[0];
      setupDelay();
      break;
    case SUMP_SET_READ_DELAY_COUNT:
      /*
       * this just sets up how many samples there should be before
       * and after the trigger fires.  The readCount is total samples
       * to return and delayCount number of samples after the trigger.
       * this sets the buffer splits like 0/100, 25/75, 50/50
       * for example if readCount == delayCount then we should
       * return all samples starting from the trigger point.
       * if delayCount < readCount we return (readCount - delayCount) of
       * samples from before the trigger fired.
       */
      getCmd();
      readCount = 4 * (((cmdBytes[1] << 8) | cmdBytes[0]) + 1);
      if (readCount > MAX_CAPTURE_SIZE)
        readCount = MAX_CAPTURE_SIZE;
      delayCount = 4 * (((cmdBytes[3] << 8) | cmdBytes[2]) + 1);
      if (delayCount > MAX_CAPTURE_SIZE)
        delayCount = MAX_CAPTURE_SIZE;
      break;
    case SUMP_SET_FLAGS:
      /* read the rest of the command bytes, but ignore them. */
      getCmd();
      break;
    case SUMP_GET_METADATA:
      /*
       * We return a description of our capabilities.
       * Check the function's comments below.
       */
      get_metadata();
      break;
    case SUMP_SELF_TEST:
      /* ignored. */
      break;
#ifdef DEBUG
      /*
       * a couple of debug commands used during development.
       */
    case '0':
      /*
       * This resets the debug buffer pointer, effectively clearing the
       * previous commands out of the buffer. Clear the sample data as well.
       * Just send a '0' from the Arduino IDE's Serial Monitor.
       */
      savecount=0;
      for (i = 0 ; i < MAX_CAPTURE_SIZE; i++) {
        logicdata[i] = 0;
      }
      break;
    case '1':
      /*
       * This is used to see what commands were sent to the device.
       * you can use the Arduino serial monitor and send a '1' and get
       * a debug printout.  useless except for development.
       */
#ifndef CHAN5
      blinkled();
#endif /* !CHAN5 */
      debugprint();
      break;
    case '2':
      /*
       * This dumps the sample data to the serial port.  Used for debugging.
       */
      debugdump();
      break;
#endif /* DEBUG */
    default:
      /* ignore any unrecognized bytes. */
      break;
    }
  }
}

void blinkled() {
  digitalWrite(ledPin, HIGH);
  delay(200);
  digitalWrite(ledPin, LOW);
  delay(200);
}

/*
 * Extended SUMP commands are 5 bytes.  A command byte followed by 4 bytes
 * of options. We already read the command byte, this gets the remaining
 * 4 bytes of the command.
 * If we're debugging we save the received commands in a debug buffer.
 * We need to make sure we don't overrun the debug buffer.
 */
void getCmd() {
  delay(10);
  cmdBytes[0] = Serial.read();
  cmdBytes[1] = Serial.read();
  cmdBytes[2] = Serial.read();
  cmdBytes[3] = Serial.read();

#ifdef DEBUG
  if (savecount < 120 ) {
    savebytes[savecount++] = ' ';
    savebytes[savecount++] = cmdByte;
    savebytes[savecount++] = cmdBytes[0];
    savebytes[savecount++] = cmdBytes[1];
    savebytes[savecount++] = cmdBytes[2];
    savebytes[savecount++] = cmdBytes[3];
  }
#endif /* DEBUG */
}

/*
 * This function samples data using a microsecond delay function.
 * It also has rudimentary trigger support where it will just sit in
 * a busy loop waiting for the trigger conditions to occur.
 *
 * This loop is not clocked to the sample rate in any way, it just
 * reads the port as fast as possible waiting for a trigger match.
 * Multiple channels can have triggers enabled and can have different
 * trigger values.  All conditions must match to trigger.
 *
 * After the trigger fires (if it is enabled) the pins are sampled
 * at the appropriate rate.
 *
 */

void captureMicro() {
  int i;

  /*
   * basic trigger, wait until all trigger conditions are met on port B.
   * this needs further testing, but basic tests work as expected.
   */
  if (trigger) {
    while ((trigger_values ^ CHANPIN) & trigger);
  }

  /*
   * disable interrupts during capture to maintain precision.
   * we're hand padding loops with NOP instructions so we absolutely
   * cannot have any interrupts firing.
   */
  cli();

  /*
   * toggle pin a few times to activate trigger for debugging.
   * this is used during development to measure the sample intervals.
   * it is best to just leave the toggling in place so we don't alter
   * any timing unexpectedly.
   * Arduino pin 7 is being used here.
   */
  DDRD = DDRD | B10000000;
  PORTD = B10000000;
  delayMicroseconds(20);
  PORTD = B00000000;
  delayMicroseconds(20);
  PORTD = B10000000;
  delayMicroseconds(20);
  PORTD = B00000000;
  delayMicroseconds(20);

  if (delayTime == 1) {
    /*
     * 1MHz sample rate = 1 uS delay so we can't use delayMicroseconds
     * since our loop takes some time.  The delay is padded out by hand.
     */
    PORTD = B10000000; /* debug timing measurement */
    for (i = 0 ; i < readCount; i++) {
      logicdata[i] = CHANPIN;
      __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");
      __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t");
    }
    PORTD = B00000000; /* debug timing measurement */
  } 
  else if (delayTime == 2) {
    /*
     * 500KHz sample rate = 2 uS delay, still pretty fast so we pad this
     * one by hand too.
     */
    PORTD = B10000000; /* debug timing measurement */
    for (i = 0 ; i < readCount; i++) {
      logicdata[i] = CHANPIN;
      __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t");
      __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t");
      __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t");
      __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t");
      __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t");
      __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t");
    }
    PORTD = B00000000; /* debug timing measurement */
  } 
  else {
    /*
     * not 1MHz or 500KHz; delayMicroseconds(delay - 1) works fine here
     * with two NOPs of padding. (based on measured debug pin toggles with
     * a better logic analyzer)
     * start of real measurement
     */
    PORTD = B10000000; /* debug timing measurement */
    for (i = 0 ; i < readCount; i++) {
      logicdata[i] = CHANPIN;
      delayMicroseconds(delayTime - 1);
      __asm__("nop\n\t""nop\n\t");
    }
    PORTD = B00000000; /* debug timing measurement */
  }

  /* re-enable interrupts now that we're done sampling. */
  sei();

  /*
   * dump the samples back to the SUMP client.  nothing special
   * is done for any triggers, this is effectively the 0/100 buffer split.
   */
  for (i = 0 ; i < readCount; i++) {
    Serial.print(logicdata[i], BYTE);
  }
}

/*
 * This function does straight sampling with basic triggering.  It is
 * for those sample rates that can't be done via the 'delayMicrosecond()' call
 * which is limited to 16383 microseconds max delay.  That is about 62Hz max.
 * This is only used for sample rates < 100Hz.
 *
 * The basic triggering in this function will be replaced by a 'triggerMillis'
 * function eventually that uses the circular trigger buffer.
 *
 * Since we're using delay() and 20ms/50ms/100ms sample rates we're not
 * worried that the sample loops take a few microseconds more than we're
 * supposed to.
 * We could measure the sample loop and use delay(delayTime - 1), then
 * delayMicroseconds() and possibly a bit of NOP padding to ensure our
 * samples our a precise multiple of milliseconds, but for now we'll use
 * this basic functionality.
 */
void captureMilli() {
  int i;

  /*
   * very basic trigger, just like in captureMicros() above.
   */
  if (trigger) {
    while ((trigger_values ^ CHANPIN) & trigger);
  }

  for (i = 0 ; i < readCount; i++) {
    logicdata[i] = CHANPIN;
    delay(delayTime);
  }
  for (i = 0 ; i < readCount; i++) {
    Serial.print(logicdata[i], BYTE);
  }
}

/*
 * This function provides sampling with triggering and a circular trigger
 * buffer.
 * This works ok at 500KHz and lower sample rates.  We don't have enough time
 * with a 16MHz clock to sample at 1MHz into the circular buffer.  A 20MHz
 * clock might be ok but all of the timings would have to be redone.
 * 
 */
void triggerMicro() {
  int i = 0;

  logicIndex = 0;
  triggerIndex = 0;

  /*
   * disable interrupts during capture to maintain precision.
   * we're hand padding loops with NOP instructions so we absolutely
   * cannot have any interrupts firing.
   */
  cli();

  /*
   * toggle pin a few times to activate trigger for debugging.
   * this is used during development to measure the sample intervals.
   * it is best to just leave the toggling in place so we don't alter
   * any timing unexpectedly.
   * Arduino pin 7 is being used here.
   */
  DDRD = DDRD | B10000000;
  PORTD = B10000000;
  delayMicroseconds(20);
  PORTD = B00000000;
  delayMicroseconds(20);
  PORTD = B10000000;
  delayMicroseconds(20);
  PORTD = B00000000;
  delayMicroseconds(20);

  if (delayTime == 1) {
    /*
     * 1MHz case.  We can't really do it at the moment.  Timing is too tight.
     * We can fake it, or skip it, or rework it....
     * This should be retested on a 20MHz clocked microcontroller.
     * The data is flat out wrong for the 1MHz case.
     */

    /*
     * return for now, the client will timeout eventually or the user will
     * click stop.
     */
    return;
  } 
  else if (delayTime == 2) {
    /*
     * 500KHz case.  We should be able to manage this in time.
     *
     * busy loop reading CHANPIN until we trigger.
     * we always start capturing at the start of the buffer
     * and use it as a circular buffer
     */
    PORTD = B10000000; /* debug timing measurement */
    while ((trigger_values ^ (logicdata[logicIndex] = CHANPIN)) & trigger) {
      /* PORTD = B00000000; */
      /* increment index. */
      logicIndex++;
      if (logicIndex >= readCount) {
        logicIndex = 0;
      }
      /*
       * pad loop to 2.0 uS (with pin toggles it is 3 x nop)
       * without pin toggles, will try 1 nop.
       * __asm__("nop\n\t""nop\n\t""nop\n\t");
       */
      __asm__("nop\n\t");
      /* PORTD = B10000000; */
    }
    /* this pads the immediate trigger case to 2.0 uS, just as an example. */
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t");
    PORTD = B00000000; /* debug timing measurement */

    /* 
     * One sample size delay. ends up being 2 uS combined with assignment
     * below.  This padding is so we have a consistent timing interval
     * between the trigger point and the subsequent samples.
     */
    delayMicroseconds(1);
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t");
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t");

    /* 'logicIndex' now points to trigger sample, keep track of it */
    triggerIndex = logicIndex;

    /* keep sampling for delayCount after trigger */
    PORTD = B10000000; /* debug timing measurement */
    /*
     * this is currently taking:
     * 1025.5 uS for 512 samples. (512 samples, 0/100 split)
     *  513.5 uS for 256 samples. (512 samples, 50/50 split)
     */
    for (i = 0 ; i < delayCount; i++) {
      if (logicIndex >= readCount) {
        logicIndex = 0;
      }
      logicdata[logicIndex++] = CHANPIN;
      __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");
      __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");
      __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t");
    }
    PORTD = B00000000; /* debug timing measurement */
    delayMicroseconds(100);
  } 
  else {
    /*
     * Less than 500KHz case.  This uses delayMicroseconds() and some padding
     * to get precise timing, at least for the after trigger samples.
     *
     * busy loop reading CHANPIN until we trigger.
     * we always start capturing at the start of the buffer
     * and use it as a circular buffer
     *
     */
    PORTD = B10000000; /* debug timing measurement */
    while ((trigger_values ^ (logicdata[logicIndex] = CHANPIN)) & trigger) {
      /* PORTD = B00000000; */
      /* increment index. */
      logicIndex++;
      if (logicIndex >= readCount) {
        logicIndex = 0;
      }
      /* PORTD = B10000000; */
    }
    PORTD = B00000000; /* debug timing measurement */

    /* 'logicIndex' now points to trigger sample, keep track of it */
    triggerIndex = logicIndex;

    /*
     * This needs adjustment so that we have the right spacing between the
     * before trigger samples and the after trigger samples.
     */
    delayMicroseconds(delayTime);

    /* keep sampling for delayCount after trigger */
    PORTD = B10000000; /* debug timing measurement */
    for (i = 0 ; i < delayCount; i++) {
      if (logicIndex >= readCount) {
        logicIndex = 0;
      }
      logicdata[logicIndex++] = CHANPIN;
      delayMicroseconds(delayTime - 3);
      __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t");
      __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t");
      __asm__("nop\n\t""nop\n\t""nop\n\t");
    }
    PORTD = B00000000; /* debug timing measurement */
    delayMicroseconds(100);
  }

  /* re-enable interrupts */
  sei();

  /*
   * trigger has fired and we have read delayCount of samples after the
   * trigger fired.  triggerIndex now points to the trigger sample
   * logicIndex now points to the last sample taken and logicIndex + 1
   * is where we should start dumping since it is circular.
   *
   * our buffer starts one entry above the last read entry.
   */
  logicIndex++;

  for (i = 0 ; i < readCount; i++) {
    if (logicIndex >= readCount) {
      logicIndex = 0;
    }
    Serial.print(logicdata[logicIndex++], BYTE);
  }
}

/*
 * This function calculates what delay we need for the specific sample rate.
 * The dividers are based on SUMP's 100Mhz clock.
 * For example, a 1MHz sample rate has a divider of 99 (0x63 in the command
 * byte).
 * rate = clock / (divider + 1)
 * rate = 100,000,000 / (99 + 1)
 * result is 1,000,000 saying we want a 1MHz sample rate.
 * We calculate our inter sample delay from the divider and the delay between
 * samples gives us the sample rate per second.
 * So for 1MHz, delay = (99 + 1) / 100 which gives us a 1 microsecond delay.
 * For 500KHz, delay = (199 + 1) / 100 which gives us a 2 microsecond delay.
 *
 */
void setupDelay() {
  if (divider >= 1500000) {
    useMicro = 0;
    delayTime = (divider + 1) / 100000;
  } 
  else {
    useMicro = 1;
    delayTime = (divider + 1) / 100;
  }
}

/*
 * This function returns the metadata about our capabilities.  It is sent in
 * response to the  OpenBench Logic Sniffer extended get metadata command.
 *
 */
void get_metadata() {
  /* device name */
  Serial.print(0x01, BYTE);
  Serial.print('A', BYTE);
  Serial.print('G', BYTE);
  Serial.print('L', BYTE);
  Serial.print('A', BYTE);
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  Serial.print('M', BYTE);
#endif /* Mega */
  Serial.print('v', BYTE);
  Serial.print('0', BYTE);
  Serial.print(0x00, BYTE);

  /* sample memory (1024) */
  Serial.print(0x21, BYTE);
  Serial.print(0x00, BYTE);
  Serial.print(0x00, BYTE);
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  Serial.print(0x1C, BYTE);
#else
  Serial.print(0x04, BYTE);
#endif /* Mega */
  Serial.print(0x00, BYTE);

  /* sample rate (1MHz) */
  Serial.print(0x23, BYTE);
  Serial.print(0x00, BYTE);
  Serial.print(0x0F, BYTE);
  Serial.print(0x42, BYTE);
  Serial.print(0x40, BYTE);

  /* number of probes (5 by default on Arduino, 8 on Mega) */
  Serial.print(0x40, BYTE);
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  Serial.print(0x08, BYTE);
#else
#ifdef CHAN5
  Serial.print(0x06, BYTE);
#else
  Serial.print(0x05, BYTE);
#endif /* CHAN5 */
#endif /* Mega */

  /* protocol version (2) */
  Serial.print(0x41, BYTE);
  Serial.print(0x02, BYTE);

  /* end of data */
  Serial.print(0x00, BYTE);  
}

/*
 * This is used by the '0' debug command to dump the contents of some
 * interesting variables and the debug buffer.
 *
 */
#ifdef DEBUG
void debugprint() {
  int i;

#if 0
  Serial.print("divider = ");
  Serial.println(divider, DEC);
  Serial.print("delayTime = ");
  Serial.println(delayTime, DEC);
  Serial.print("trigger_values = ");
  Serial.println(trigger_values, BIN);
#endif
  Serial.print("readCount = ");
  Serial.println(readCount, DEC);
  Serial.print("delayCount = ");
  Serial.println(delayCount, DEC);
  Serial.print("logicIndex = ");
  Serial.println(logicIndex, DEC);
  Serial.print("triggerIndex = ");
  Serial.println(triggerIndex, DEC);

  Serial.println("Bytes:");

  for (i = 0 ; i < savecount; i++) {
    if (savebytes[i] == 0x20) {
      Serial.println();
    } 
    else {
      Serial.print(savebytes[i], HEX);
      Serial.print(' ', BYTE);
    }
  }
  Serial.println("done...");
}

/*
 * This is used by the '2' debug command to dump the contents
 * of the sample buffer.
 */
void debugdump() {
  int i;
  int j = 1;

  Serial.print("\r\n");

  for (i = 0 ; i < MAX_CAPTURE_SIZE; i++) {
    Serial.print(logicdata[i], HEX);
    Serial.print(" ");
    if (j == 32) {
      Serial.print("\r\n");
      j = 0;
    }
    j++;
  }
}
#endif /* DEBUG */


