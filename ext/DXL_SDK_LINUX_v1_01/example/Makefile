TARGET = ReadWrite SyncWrite

DXL_LIB = ../src/libdxl.a

include ../build_rule.mk

CFLAGS = -O2 $(INCLUDES) -I../include
LDLIBS = -lm `/bin/sh ld_wsock.sh` `/bin/sh ld_setupapi.sh`

all : $(TARGET)

clean :
	$(RM) *.o $(TARGET)

$(TARGET) : ReadWrite.o $(DXL_LIB)

$(DXL_LIB) :
	cd $(@D)/ && $(MAKE) $(@F)
