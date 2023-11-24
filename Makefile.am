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
COMMON_C_FLAGS = -Ithird-party
AM_CXXFLAGS= -std=c++11 $(COMMON_C_FLAGS)
AM_CPPFLAGS = $(COMMON_C_FLAGS)

HDRS = \
    third-party/strptime.h \
	third-party/argtable3/argtable3.h \
	third-party/packet-forwarder/libloragw-helper.h \
	third-party/packet-forwarder/loragw_hal.h.patch \
	third-party/packet-forwarder/substy-call-c.h

#
# Binaries
#
bin_PROGRAMS = tlns-rak2287

tlns_rak2287_SOURCES =\
	gw-rak2287.cpp \
    lorawan-gateway-listener.cpp \
	scheduler.cpp schedule-item-concentrator.cpp \
	lorawan-error.cpp lorawan-msg.cpp \
	lorawan-date.cpp lorawan-string.cpp lorawan-conv.cpp lorawan-types.cpp \
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
	third-party/packet-forwarder/README.packet_forwarder.md third-party/packet-forwarder/LICENSE.TXT
