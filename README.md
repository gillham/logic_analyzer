SUMP compatible logic analyzer for Arduino
==========================================

This Arduino sketch implements a SUMP protocol compatible logic analyzer.
This implementation is compatible with the standard SUMP client as well as
an alternative OLS client.  More recently support has been added to Sigrok.

This logic analyzer for Arduino supports 6 channels consisting of digital
pins 8-13, which are the first 6 bits (0-5) of PORTB.
Arduino pin 13 / bit 5 is the Arduino LED, bits 6 & 7 are the crystal
oscillator pins. Comment out CHAN5 if you don't want to use the
LED pin for an input.

On the Arduino Mega board 8 channels are supported and 7k of samples.
Pins 22-29 (Port A) are used by default.

Client Software
===============

Sigrok support via the 'ols' device configuration has been added. This
mostly involved returning the capture buffer in the reverse order.

Use the `logic_analyzer_sigrok` sketch.  Since the OLS alternative client
mentioned below has some issues with newer Java versions, Sigrok is currently
the only practical way to use this logic analyzer.  If you use an older machine
with an older operating system and older Java you can probably use the OLS client.

Sigrok support seems to work fairly well so I would currently recommend it for
anyone interested in trying this sketch.

Run PulseView like this on Linux: (I'll add Windows options after more testing)
```
PulseView --driver=ols:conn=/dev/ttyUSB0 --dont-scan
```

It may be necessary to exit and relaunch PulseView to get it to recognize the device.
An easy way to test the device is using the `sigrok-cli` utility. The command below
samples channel 2 at 1MHz.  If you get a device not found error, but /dev/ttyUSB0 exists,
run this command a couple times and usually it will start working. Due to the way opening
the serial port resets the Arduino there are some issues/bugs to work out yet.
```
sigrok-cli --driver=ols:conn=/dev/ttyUSB0 --config samplerate=1Mhz --config pattern=External --samples 1024 --channels 2
```

The OLS alternative client hasn't had an official release recently so you will
need to compile it yourself.

Follow the build instructions here: https://github.com/jawi/ols


Older details on the OLS client is available at the project page:
https://lxtreme.nl/projects/ols/

Direct link to older releases of the OLS alternative client:
http://www.lxtreme.nl/ols/

The alternative client version is highly recommended.  You can tried the older
release ols-0.9.7.2 but most likely need to build it yourself. Use "ols-0.9.7"
or newer for built-in device profiles.

To use this with the original or alternative SUMP clients,
use these settings:
```
Sampling rate: 4MHz (or lower) (no 2MHz on ATmega168)
Channel Groups: 0 (zero) only
Recording Size:
   ATmega168:  532 (or lower)
   ATmega328:  1024 (or lower)
   ATmega2560: 7168 (or lower)
Noise Filter: doesn't matter
RLE: disabled (unchecked)
```

Using the Logic Analyzer
========================
Triggering is still a work in progress, but generally works for samples
below 1MHz.  1MHz works for a basic busy wait trigger that doesn't store
until after the trigger fires.

Please try it out and report back.  Please provide a detailed bug report
if you file an issue.

Debugging
=========

You can uncomment the `#define DEBUG_MENU` line to add some diagnostic menu
options for capturing or dumping the capture buffer.
You can uncomment the `#define DEBUG` and `#define DEBUG_MENU` for a couple
extra menu options and logging of the received commands.  The DEBUG option
is generally only useful for development, while the DEBUG_MENU option is
good for troubleshooting when the logic_analyzer sketch isn't working for you.
Both are disabled by default to conserve RAM for improved stability.

CLI compiling
=============

If you want to use the `arduino-cli` tool to compile this using the Makefile,
you'll need to install the tool first following instructions here:
https://arduino.github.io/arduino-cli/

Once installed you can simple type `make` and you should get some simple help:
```bash
$ make
---> run 'make build' to compile for Arduino Duemilanove
---> run 'make upload' to upload to /dev/ttyUSB*
```


Other Notes
===========================================================================
```
NOTE: Starting with v0.11 you can sample at 4MHz & 2MHz rates in addition to the 
      previous 1MHz and lower rates.  This is done via unrolled loops which
      makes the source code huge and the binary takes much more of the flash.
      v0.11 is just slightly too big for an ATmega168's flash. The code 
      automatically skips the 2MHz code on ATmega168

NOTE: v0.09 switched the channels BACK to pins 8-13 for trigger reliability.
      Please report any issues.  Uncomment USE_PORTD for pins 2-7.

NOTE: If you are using the original SUMP client, then you will get a
      "device not found" error.
      You must DISABLE the Arduino auto reset feature to use this logic analyzer
      code. There are various methods to do this, some boards have a jumper,
      others require you to cut a trace.  You may also install a *precisely*
      120 Ohm resistor between the reset & 5V piins.  Make sure it is really
      120 Ohm or you may damage your board.  It is much easier to use the
      alternative SUMP client referenced above.
      [ This is not needed with ols-0.9.7 or newer. ]
      [ DO NOT use this resistor unless absolutely necessary on old clients. ]
```
