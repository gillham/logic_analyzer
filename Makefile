#
# Makefile for an Arduino based logic analyzer using the 'arduino-core'
# package and makefiles.
#
#	$Id: Makefile,v 1.3 2011-03-07 02:47:26 gillham Exp $
#
ARDUINO_DIR  = /usr/share/arduino

TARGET       = logic_analyzer
ARDUINO_LIBS = 

MCU          = atmega328p
F_CPU        = 16000000
ARDUINO_PORT = /dev/ttyUSB*
AVRDUDE_ARD_BAUDRATE = 57600
AVRDUDE_ARD_PROGRAMMER = arduino

include /usr/share/arduino/Arduino.mk

