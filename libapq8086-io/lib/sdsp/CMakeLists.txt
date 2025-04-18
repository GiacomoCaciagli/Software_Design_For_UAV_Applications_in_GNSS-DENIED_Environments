# CMakeLists for the SDSP (sensors DSP) code for libvoxl_io
cmake_minimum_required(VERSION 3.0)

# Set a local variables
set(LIBNAME io_rpc)
set(IDLNAME io_rpc)
set(QC_SOC_TARGET APQ8096)

# general compile flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0 -g -std=gnu99 -fno-strict-aliasing \
                    -fdata-sections -fno-zero-initialized-in-bss -Wall -Wextra \
                    -Werror -Wno-unused-parameter -Wno-unused-function \
                    -Wno-unused-variable -Wno-gnu-array-member-paren-init \
                    -Wno-cast-align -Wno-missing-braces -Wno-strict-aliasing")

# include the cmake_hexagon macro directory and the qurt macros
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} $ENV{CMAKE_HEXAGON_DIR})
include(qurt_flags)
include(qurt_lib)
add_definitions(-DDSP_TYPE_SLPI)
add_definitions(-D__QURT)

# generate stubs from idl file, apps-proc code has to do the same
FASTRPC_STUB_GEN(../idl/${IDLNAME}.idl)

# add DSPAL include directory for sDSP headers
include_directories(
    $ENV{DSPAL_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/
    /usr/include/c_library_v2/
)

# cmake_hexagon macro to make the SDSP lib
QURT_LIB(
    LIB_NAME ${LIBNAME}
    IDL_NAME ${IDLNAME}
    SOURCES  io_rpc_misc.c io_rpc_spi.c io_rpc_uart.c io_rpc_gpio.c io_rpc_i2c.c io_rpc_mavparser.c
)

install(
    TARGETS io_rpc io_rpc_skel
    LIBRARY DESTINATION /usr/lib/rfsa/adsp
    RUNTIME DESTINATION /usr/lib/rfsa/adsp
)
