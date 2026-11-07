# Copy the entire firmware/buckler directory to ~/buckler/software/apps/eecs149_robot
# Paths deliberately match lab11/buckler/software/apps/blink/Makefile.
PROJECT_NAME = $(shell basename "$(realpath ./)")
NRF_IC = nrf52832
SDK_VERSION = 15
SOFTDEVICE_MODEL = s132
APP_HEADER_PATHS += .
APP_SOURCE_PATHS += .
APP_SOURCES = $(notdir $(wildcard ./*.c))
NRF_BASE_DIR = ../../nrf52x-base/
include ../../boards/buckler_revC/Board.mk
include $(NRF_BASE_DIR)make/AppMakefile.mk
