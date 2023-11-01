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
	third-party/argtable3/argtable3.h

#
# Binaries
#
bin_PROGRAMS = empty-project

empty_project_SOURCES = \
	main.cpp \
	third-party/argtable3/argtable3.c
empty_project_LDADD =
empty_project_CPPFLAGS = $(EXTRA_DEF)

#
# Configs, readme, CMake etc.
#
configdir = $(datadir)
dist_config_DATA = \
	autogen.sh CMakeLists.txt CODE_OF_CONDUCT.md CONTRIBUTING.md COPYING HISTORY LICENSE README.md TODO \
	third-party/argtable3/README
