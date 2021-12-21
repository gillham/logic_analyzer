#
# Makefile for an Arduino based logic analyzer using the arduino-cli
#

TARGET	= logic_analyzer
FQBN	= arduino:avr:diecimila
SERIAL	= /dev/ttyUSB*


all:
	@echo ""
	@echo "---> run 'make build' to compile for Arduino Duemilanove"
	@echo "---> run 'make upload' to upload to /dev/ttyUSB*"
	@echo ""

build:
	arduino-cli compile --fqbn $(FQBN) $(TARGET)

upload:
	arduino-cli upload --fqbn $(FQBN) --port $(SERIAL) $(TARGET)

#
# end-of-file
#
