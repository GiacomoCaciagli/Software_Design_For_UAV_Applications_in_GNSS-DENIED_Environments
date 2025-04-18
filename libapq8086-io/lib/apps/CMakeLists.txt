# CMakeLists for the apps-proc side code for libvoxl_io
cmake_minimum_required(VERSION 3.0)

# Set a local variables
set(LIBNAME voxl_io)
set(IDLNAME io_rpc)
set(QC_SOC_TARGET APQ8096)

# general compile flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0 -g -std=gnu99 -fno-strict-aliasing \
                    -fdata-sections -fno-zero-initialized-in-bss -Wall -Wextra \
                    -Werror -Wno-unused-parameter -Wno-unused-function \
                    -Wno-unused-variable -Wno-gnu-array-member-paren-init \
                    -Wno-cast-align -Wno-missing-braces -Wno-strict-aliasing \
                    -mfloat-abi=softfp -pthread")

# include the cmake_hexagon macro directory and the linux_app macros
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} $ENV{CMAKE_HEXAGON_DIR})
include(linux_app)

# generate stubs from idl file, sdsp code has to do the same
FASTRPC_STUB_GEN(../idl/${IDLNAME}.idl)

# all source files, library and examples, will need the headers for this library
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/
    /usr/include/c_library_v2/
)

# cmake_hexagon macro to make a linux lib
LINUX_LIB(
    LIB_NAME ${LIBNAME}
    IDL_NAME ${IDLNAME}
    SOURCES  voxl_io_uart.c voxl_io_misc.c voxl_io_spi.c voxl_io_gpio.c voxl_io_i2c.c voxl_io_mavparser.c 
)

# also build the example test program
add_executable (voxl-uart-loopback voxl-uart-loopback.c)
target_link_libraries (voxl-uart-loopback voxl_io)

# also build the time test program
add_executable (voxl-test-time voxl-test-time.c)
target_link_libraries (voxl-test-time voxl_io)

add_executable (voxl-gpio-loopback voxl-gpio-loopback.c)
target_link_libraries (voxl-gpio-loopback voxl_io)

add_executable (voxl-gpio voxl-gpio.c)
target_link_libraries (voxl-gpio voxl_io)

add_executable (voxl-i2c voxl-i2c.c)
target_link_libraries (voxl-i2c voxl_io)

add_executable (voxl-spi voxl-spi.c)
target_link_libraries (voxl-spi voxl_io)

add_executable (voxl-open-uart open-uart.c)
target_link_libraries (voxl-open-uart voxl_io)

add_executable (voxl-close-uart close-uart.c)
target_link_libraries (voxl-close-uart voxl_io)

add_executable (voxl-read-uart read-uart.c)
target_link_libraries (voxl-read-uart voxl_io)

add_executable (voxl-write-uart write-uart.c)
target_link_libraries (voxl-write-uart voxl_io)

add_executable (voxl-parser parser.c)
target_link_libraries (voxl-parser voxl_io)

# add_executable (voxl-i2c-mpu6050-tester voxl-i2c-mpu6050-tester.c)
# target_link_libraries (voxl-i2c-mpu6050-tester voxl_io)

add_executable (voxl-test-mavparser voxl-test-mavparser.c)
target_link_libraries (voxl-test-mavparser voxl_io pthread)

# find all public header files to install in include directory
file(GLOB LIB_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/../include/*.h)
set_target_properties(${LIBNAME} PROPERTIES PUBLIC_HEADER "${LIB_HEADERS}")

install(
    TARGETS ${LIBNAME} voxl-uart-loopback voxl-test-time voxl-gpio-loopback voxl-gpio voxl-i2c voxl-spi voxl-test-mavparser 
    voxl-open-uart voxl-close-uart voxl-read-uart voxl-write-uart voxl-parser
    LIBRARY DESTINATION /usr/lib
    RUNTIME DESTINATION /usr/bin
    PUBLIC_HEADER DESTINATION /usr/include
)
