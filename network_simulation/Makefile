COMPONENT=NodeC


INCLUDE=-IdataStructures
INCLUDE+=-IdataStructures/interfaces/ -IdataStructures/modules
INCLUDE+=-Ilib/interfaces -Ilib/modules
CFLAGS += -DTOSH_DATA_LENGTH=28
CFLAGS+=$(INCLUDE)
<<<<<<< HEAD
TINYOS_ROOT_DIR = /home/jblock/tiny/tinyos-main/
=======
TINYOS_ROOT_DIR = /home/andrewnguyen/Desktop/tiny/tinyos-main/
>>>>>>> 149558a22e91ad78bd71721a7a1e36288db8f100
include $(TINYOS_ROOT_DIR)/Makefile.include

CommandMsg.py: CommandMsg.h
	nescc-mig python -python-classname=CommandMsg CommandMsg.h CommandMsg -o $@

packet.py: packet.h
	nescc-mig python -python-classname=pack packet.h pack -o packet.py
