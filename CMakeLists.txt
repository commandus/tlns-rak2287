cmake_minimum_required(VERSION 3.9)
set(PROJECT_NAME "tlns-rak2287")
project(${PROJECT_NAME} VERSION 1.0)

#
# Fix path
#
set(LIBLORAGW_ROOT ../../libloragw)
file(REAL_PATH ${LIBLORAGW_ROOT} LIBLORAGW_ROOT EXPAND_TILDE)

set(CMAKE_CXX_STANDARD 11)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_library(LIBLORAGW NAMES loragw HINTS
	"." ".." "lib" "../lib" ${LIBLORAGW_ROOT}
	"../../libloragw" "../../../libloragw"
	"../../libloragw/build" "../../../libloragw/build"
	"${LIBLORAGW_ROOT}/build"
	"${LIBLORAGW_ROOT}/build/Release"
)

set(SRC_MAIN
	gw-rak2287.cpp lorawan-gateway-listener.cpp	scheduler.cpp schedule-item-concentrator.cpp
	lorawan/lorawan-error.cpp lorawan/lorawan-msg.cpp lorawan/lorawan-date.cpp lorawan/lorawan-string.cpp
	lorawan/lorawan-mac.cpp lorawan/lorawan-conv.cpp lorawan/lorawan-types.cpp
	third-party/packet-forwarder/subst-call-c.cpp
	third-party/packet-forwarder/libloragw-helper.cpp
	third-party/strptime.cpp
	third-party/argtable3/argtable3.c
)

include_directories(. "third-party")

add_executable(tlns-rak2287 ${SRC_MAIN})
target_link_libraries(tlns-rak2287 PRIVATE
	${EXTRA_LIBS} ${LIBLORAGW}
)

INSTALL(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/tlns-rak2287 DESTINATION cmake)
INSTALL(FILES README.md LICENSE HISTORY COPYING TODO CODE_OF_CONDUCT.md CONTRIBUTING.md DESTINATION doc)

include(InstallRequiredSystemLibraries)
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
set(CPACK_PACKAGE_VERSION_MAJOR "1")
set(CPACK_PACKAGE_VERSION_MINOR "0")
include(CPack)
