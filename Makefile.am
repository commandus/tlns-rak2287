SUBDIRS = .
ACLOCAL_AMFLAGS = -I m4

EXTRA_LIB =
EXTRA_HEADERS =
EXTRA_DEF =

#
# https://ask-dev.ru/info/1247790/understanding-gcc-5s-glibcxxusecxx11abi-or-the-new-abi
#-D_GLIBCXX_USE_CXX11_ABI=0
# -I../../third-party for make distcheck target be happy
#
COMMON_C_FLAGS = -Ithird-party -I../../third-party
AM_CXXFLAGS= -std=c++11 $(COMMON_C_FLAGS)
AM_CPPFLAGS = $(COMMON_C_FLAGS)

nobase_dist_include_HEADERS = \
    config.h gateway-settings.h lorawan-gateway-listener.h \
    schedule-item-concentrator.h gateway-lora.h log-intf.h scheduler.h \
    lorawan/lorawan-conv.h lorawan/lorawan-error.h lorawan/lorawan-msg.h \
    lorawan/lorawan-types.h lorawan/lorawan-string.h lorawan/lorawan-const.h lorawan/lorawan-date.h \
    config/gateway_usb_conf.h \
    third-party/strptime.h \
    third-party/argtable3/argtable3.h \
    third-party/packet-forwarder/libloragw-helper.h \
    third-party/packet-forwarder/loragw_hal.h.patch \
    third-party/packet-forwarder/subst-call-c.h

#
# Binaries
#
bin_PROGRAMS = tlns-rak2287

tlns_rak2287_SOURCES =\
	gw-rak2287.cpp \
    lorawan-gateway-listener.cpp \
	scheduler.cpp schedule-item-concentrator.cpp \
    lorawan/lorawan-error.cpp lorawan/lorawan-msg.cpp lorawan/lorawan-date.cpp lorawan/lorawan-string.cpp \
    lorawan/lorawan-conv.cpp lorawan/lorawan-types.cpp \
	third-party/packet-forwarder/subst-call-c.cpp \
	third-party/packet-forwarder/libloragw-helper.cpp \
	third-party/strptime.cpp \
	third-party/argtable3/argtable3.c
tlns_rak2287_LDADD = -Llib -lloragw
tlns_rak2287_CPPFLAGS = $(EXTRA_DEF)

#
# Configs, readme, CMake etc.
#
configdir = $(datadir)
dist_config_DATA = \
	autogen.sh CMakeLists.txt CODE_OF_CONDUCT.md CONTRIBUTING.md COPYING HISTORY LICENSE README.md TODO \
	third-party/argtable3/README \
	third-party/packet-forwarder/README.packet_forwarder.md third-party/packet-forwarder/LICENSE.SEMTECH.txt
