PREFIX = /usr/local
#PREFIX = /mingw

include build_rule.mk

CONFIG_FILE = dxl-config
S_PREFIX = $(shell echo "$(PREFIX)" | sed "s/\//\\\\\\\\\//g")
S_LIBS = $(shell if test x$(MSYSTEM) == x"MINGW32"; then echo "-lwsock32 -lsetupapi"; fi)
all : $(CONFIG_FILE)
	cd src/ && $(MAKE)
	cd example/ && $(MAKE)

clean :
	$(RM) $(CONFIG_FILE)
	cd src/ && $(MAKE) clean
	cd example/ && $(MAKE) clean

install : all
	install -d $(PREFIX)/include/dxl/
	install -m 644 include/*.h $(PREFIX)/include/dxl/
	install -d $(PREFIX)/lib/
	install -m 644 src/libdxl.a $(PREFIX)/lib/
	install -d $(PREFIX)/bin/
	install -m 755 $(CONFIG_FILE) $(PREFIX)/bin/

$(CONFIG_FILE) : dxl-config.in Makefile
	cat $(CONFIG_FILE).in | sed "s/PREFIX/$(S_PREFIX)/g" | sed "s/LIBS/$(S_LIBS)/g" > $(CONFIG_FILE)
