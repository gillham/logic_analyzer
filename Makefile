#
# Makefile for an Arduino based logic analyzer using the 'arduino-core'
# package and makefiles.
#
#	$Id: Makefile,v 1.2 2011-02-10 23:20:31 gillham Exp $
#
ARDUINO_DIR  = /usr/share/arduino

TARGET       = logic_analyzer
ARDUINO_LIBS = 

MCU          = atmega328p
F_CPU        = 16000000
ARDUINO_PORT = /dev/ttyUSB*
AVRDUDE_ARD_BAUDRATE = 115200
AVRDUDE_ARD_PROGRAMMER = arduino

include /usr/share/arduino/Arduino.mk

