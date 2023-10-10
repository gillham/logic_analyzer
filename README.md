# SUMP compatible logic analyzer for Arduino

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

# Supported Hardware
The AGLA sketch supports many Arduino boards based on the following microcontrollers, generally using an FTDI based USB to serial, or builtin in the case of the ATmega32U4.  Many inexpensive boards use the 'CH340' USB to serial chipset which may or may not work well.  Please test yours and file an issue.

Generally I test the Arduino Duemilanove and Arduino UNO R3 the most, but I do want to make as many boards as possible work.

The microcontrollers are listed below with corresponding Arduino boards.  This is not an exhaustive list and I do not own all of these boards to test them.  Again please test and file any issues.

## ATmega168
- Arduino Diecimila
- Arduino Mini (Original was '168)

## ATmega328P
- Arduino Duemilanove
- Arduino Mini (Original was '168)
- Arduino Nano
- Arduino Pro
- Arduino Pro Mini
- Arduino Uno (Rev2, R3, SMD)
- Arduino Uno Mini

## ATmega1280
- Arduino Mega

## ATmega2560
- Arduino Mega 2560

## ATmega32U4
- Arduino Leonardo
- Arduino Micro

# Installation

## Arduino IDE Library Manager
Starting with v0.17 you can install directly in the Arduino IDE using the Library Manager.
Look under the menu 'Sketch -> Include Library -> Manage Libraries...' to open the Library Manager, then enter 'LogicAnalyzer' in the search field and it should find this project and you can click INSTALL.

## Manual via ZIP file
You can use the GitHub 'Download ZIP' feature to get an installable "library"
for use with the Arduino IDE.  Select 'Sketch -> Include Library -> Add .ZIP Libary'
from the Arduino IDE 2.x and select the zip file you downloaded from GitHub, then select open.

## After Installation
Once installed you can use the 'File -> Examples -> LogicAnalyzer' menu to find
different versions of the sketches.  You might want to start with `logic_analyzer_sigrok`
and use PulseView.

# Client Software

## Sigrok

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

## OLS Client(s)

*NOTE: This section needs work as due to various Java issuses building a working OLS client is somewhat broken right now.*

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

To use this with the original or alternative SUMP clients, use these settings:
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

# Using the Logic Analyzer

Triggering is still a work in progress, but generally works for samples
below 1MHz.  1MHz works for a basic busy wait trigger that doesn't store
until after the trigger fires.

Please try it out and report back.  Please provide a detailed bug report
if you file an issue.

# Debugging

You can uncomment the `#define DEBUG_MENU` line to add some diagnostic menu
options for capturing or dumping the capture buffer.
You can uncomment the `#define DEBUG` and `#define DEBUG_MENU` for a couple
extra menu options and logging of the received commands.  The DEBUG option
is generally only useful for development, while the DEBUG_MENU option is
good for troubleshooting when the logic_analyzer sketch isn't working for you.
Both are disabled by default to conserve RAM for improved stability.

# CLI compiling

If you want to use the `arduino-cli` tool to compile this using the Makefile,
you'll need to install the tool first following instructions here:
https://arduino.github.io/arduino-cli/

If you use Debian or Ubuntu you can install `arduino-cli` and the AVR toolchain like this:
```bash
sudo snap install arduino-cli
arduino-cli core install arduino:avr
```

Once installed you can simply type `make` and you should get some basic help:
```bash
$ make
---> run 'make build' to compile for Arduino Duemilanove
---> run 'make upload' to upload to /dev/ttyUSB*
```

# Other SUMP compatible projects

There are other projects doing some similar that have been created in the last 12 years or so since I started my work.  I'll start the list with one I've read about recently.

This first project runs on a Raspberry Pi Pico and has some amazing logic analyzer specs for a $5 board!

[μLA: Micro Logic Analyzer](https://github.com/dotcypress/ula/)

[An earlier RPi Pico SUMP logic analyzer](https://github.com/perexg/picoprobe-sump)

[ESP32 based Logic Analyzer](https://github.com/EUA/ESP32_LogicAnalyzer)

[Flexible SUMP library](https://github.com/pschatzmann/logic-analyzer)

[STM32 based SUMP logic analyzer](https://github.com/ag88/SumpSTM32F401cc)

[Another STM32 based logic analyzer](https://github.com/jpbarraca/LogicAlNucleo)

[ESP32 SUMP logic analyzer for Sigrok](https://github.com/Ebiroll/esp32_sigrok)

[Another STM32 logic analyzer](https://github.com/ddrown/stm32-sump)

# Older Notes

```text
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
