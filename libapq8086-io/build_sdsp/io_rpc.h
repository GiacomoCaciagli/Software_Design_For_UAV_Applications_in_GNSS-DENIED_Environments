#ifndef _IO_RPC_H
#define _IO_RPC_H
/*******************************************************************************
 * Copyright 2019 ModalAI Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * 4. The Software is used solely in conjunction with devices provided by
 *    ModalAI Inc.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
#include "AEEStdDef.h"
#ifndef __QAIC_HEADER
#define __QAIC_HEADER(ff) ff
#endif //__QAIC_HEADER

#ifndef __QAIC_HEADER_EXPORT
#define __QAIC_HEADER_EXPORT
#endif // __QAIC_HEADER_EXPORT

#ifndef __QAIC_HEADER_ATTRIBUTE
#define __QAIC_HEADER_ATTRIBUTE
#endif // __QAIC_HEADER_ATTRIBUTE

#ifndef __QAIC_IMPL
#define __QAIC_IMPL(ff) ff
#endif //__QAIC_IMPL

#ifndef __QAIC_IMPL_EXPORT
#define __QAIC_IMPL_EXPORT
#endif // __QAIC_IMPL_EXPORT

#ifndef __QAIC_IMPL_ATTRIBUTE
#define __QAIC_IMPL_ATTRIBUTE
#endif // __QAIC_IMPL_ATTRIBUTE
#ifdef __cplusplus
extern "C" {
#endif
__QAIC_HEADER_EXPORT int __QAIC_HEADER(io_rpc_uart_init)(int32 bus, int32 baudrate) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(io_rpc_uart_set_baud_rate)(int32 bus, int32 baudrate) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(io_rpc_uart_close)(int32 bus) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(io_rpc_uart_write)(int32 bus, const unsigned char* data, int dataLen, int32* bytes_written) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(io_rpc_uart_read)(int32 bus, unsigned char* data, int dataLen, int32* bytes_read) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(io_rpc_uart_flush)(int32 bus) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(io_rpc_uart_drain)(int32 bus) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(io_rpc_sdsp_time_monotonic_ns)(int64* t) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(io_rpc_sdsp_time_realtime_ns)(int64* t) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(io_rpc_spi_init)(int32 bus, int32 bus_mode, int32 freq_hz) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(io_rpc_spi_set_freq)(int32 bus, int32 freq_hz) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(io_rpc_spi_skip_dummy_bit)(int32 bus, int32 val) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(io_rpc_spi_close)(int32 bus) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(io_rpc_spi_write)(int32 bus, const unsigned char* data, int dataLen) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(io_rpc_spi_write_cs)(int32 bus, const unsigned char* data, int dataLen, int32 cs_gpio) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(io_rpc_spi_transfer)(int32 bus, const unsigned char* write, int writeLen, unsigned char* read, int readLen) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(io_rpc_spi_transfer_cs)(int32 bus, const unsigned char* write, int writeLen, unsigned char* read, int readLen, int32 cs_gpio) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(io_rpc_spi_write_reg_byte)(int32 bus, uint8 address, uint8 val) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(io_rpc_spi_write_reg_byte_cs)(int32 bus, uint8 address, uint8 val, int32 cs_gpio) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(io_rpc_spi_write_reg_word)(int32 bus, uint8 address, uint16 val) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(io_rpc_spi_write_reg_word_cs)(int32 bus, uint8 address, uint16 val, int32 cs_gpio) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(io_rpc_spi_read_reg)(int32 bus, uint8 address, unsigned char* data, int dataLen) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(io_rpc_spi_read_reg_cs)(int32 bus, uint8 address, unsigned char* data, int dataLen, int32 cs_gpio) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(io_rpc_spi_read_reg_byte)(int32 bus, uint8 address, uint8* data) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(io_rpc_spi_read_reg_byte_cs)(int32 bus, uint8 address, uint8* data, int32 cs_gpio) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(io_rpc_spi_read_reg_word)(int32 bus, uint8 address, uint16* data) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(io_rpc_spi_read_reg_word_cs)(int32 bus, uint8 address, uint16* data, int32 cs_gpio) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(io_rpc_gather_flir_segment)(int32 spi_bus, unsigned char* outdata, int outdataLen) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(io_rpc_spi_read_imu_fifo)(int32 bus, uint8 count_address, uint8 fifo_address, uint8 packet_size, int32 min_packets, int32* packets_read, unsigned char* data, int dataLen, int32 count_speed, int32 data_speed) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(io_rpc_gpio_init_input)(int32 gpio_pin) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(io_rpc_gpio_init_output)(int32 gpio_pin) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(io_rpc_gpio_read)(int32 gpio_pin, int32* gpio_state) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(io_rpc_gpio_write)(int32 gpio_pin, int32 gpio_state) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(io_rpc_gpio_close)(int32 gpio_pin) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(io_rpc_gpio_is_initialized)(int32 gpio_pin) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(io_rpc_i2c_init)(uint32 bus) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(io_rpc_i2c_slave_config)(uint32 bus, uint32 slave_address, uint32 bit_rate, uint32 timeout_us) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(io_rpc_i2c_write)(uint32 bus, uint32 register_address, uint32 register_address_num_bits, const unsigned char* write_buffer, int write_bufferLen) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(io_rpc_i2c_read)(uint32 bus, uint32 register_address, uint32 register_address_num_bits, unsigned char* read_buffer, int read_bufferLen) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(io_rpc_i2c_close)(uint32 bus) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(io_rpc_mavparser_init)(int32 bus, int32 baudrate, int32 buflen) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(io_rpc_mavparser_read)(int32 bus, unsigned char* data, int dataLen, int32* msgs_read, unsigned char* status, int statusLen) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(io_rpc_mavparser_close)(int32 bus) __QAIC_HEADER_ATTRIBUTE;
#ifdef __cplusplus
}
#endif
#endif //_IO_RPC_H
