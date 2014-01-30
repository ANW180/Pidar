LIB_DXL = libdxl.a

include ../build_rule.mk

CFLAGS = -g -O2 $(INCLUDES) -I../include -fPIC

all : $(LIB_DXL)

clean :
	$(RM) *.o $(LIB_DXL)

$(LIB_DXL) : \
	$(LIB_DXL)(dxl_hal.o) \
	$(LIB_DXL)(dynamixel.o) \
