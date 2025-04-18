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
/**
 * @file voxl_io_spi.c
 *
 * Apps-proc code for SPI functions of libvoxl_io
 */

#include <stdio.h>
#include <string.h>     // for memset
#include <voxl_io.h>    // user-facing function declarations defined in this file
#include <io_rpc.h>     // sdsp rpc calls from idl file

#define MAX_BUS         16
#define VOSPI_SEGMENT   9840


int voxl_spi_init(int bus, int bus_mode, int freq_hz)
{
    // sanity checks
    if(bus<0 || bus>MAX_BUS){
        fprintf(stderr,"ERROR in voxl_spi_init, bus must be between 0 & %d\n", MAX_BUS);
        return -1;
    }
    if(bus_mode<0 || bus_mode>3){
        fprintf(stderr,"ERROR in voxl_spi_init, bus_mode must be between 0 & 3\n");
        return -1;
    }

    int ret = io_rpc_spi_init(bus, bus_mode, freq_hz);
    if(ret) fprintf(stderr, "io_rpc_spi_init() failed\n");
    return ret;
}

int voxl_spi_set_freq(int bus, int freq_hz)
{
    int ret = io_rpc_spi_set_freq(bus, freq_hz);
    if(ret) fprintf(stderr, "io_rpc_spi_set_freq() failed\n");
    return ret;
}

int voxl_spi_skip_dummy_bit(int bus, int val)
{
    int ret = io_rpc_spi_skip_dummy_bit(bus, val);
    if(ret) fprintf(stderr, "toggle_dummy_bit() failed\n");
    return ret;
}

int voxl_spi_close(int bus)
{
    int ret = io_rpc_spi_close(bus);
    if(ret) fprintf(stderr, "io_rpc_spi_close() failed\n");
    return ret;
}


int voxl_spi_write(int bus, uint8_t* data, int bytes)
{
    if(bytes<0 || bytes>DSPAL_SPI_TRANSMIT_BUFFER_LENGTH){
        fprintf(stderr, "ERROR in voxl_spi_write, bytes out of bounds\n");
        return -1;
    }
    int ret = io_rpc_spi_write(bus, data, bytes);
    if(ret) fprintf(stderr, "io_rpc_spi_write() failed\n");
    return ret;
}

int voxl_spi_write_cs(int bus, uint8_t* data, int bytes, int cs_gpio)
{
    if(bytes<0 || bytes>DSPAL_SPI_TRANSMIT_BUFFER_LENGTH){
        fprintf(stderr, "ERROR in voxl_spi_write_cs, bytes out of bounds\n");
        return -1;
    }
    int ret = io_rpc_spi_write_cs(bus, data, bytes, cs_gpio);
    if(ret) fprintf(stderr, "io_rpc_spi_write_cs() failed\n");
    return ret;
}

int voxl_spi_transfer(int bus, uint8_t* write_buf, int write_len, uint8_t* read_buf, int read_len)
{
    if(write_len<0 || write_len>(DSPAL_SPI_RECEIVE_BUFFER_LENGTH-1)){
        fprintf(stderr, "ERROR in %s can only write up to %d bytes\n", __FUNCTION__,\
                                                (DSPAL_SPI_RECEIVE_BUFFER_LENGTH-1));
        return -1;
    }
    if(read_len<0 || read_len>(DSPAL_SPI_RECEIVE_BUFFER_LENGTH-1)){
        fprintf(stderr, "ERROR in %s can only read up to %d bytes\n", __FUNCTION__,\
                                                (DSPAL_SPI_RECEIVE_BUFFER_LENGTH-1));
        return -1;
    }
    int ret = io_rpc_spi_transfer(bus, write_buf, write_len, read_buf, read_len);
    if(ret) fprintf(stderr, "%s failed\n",__FUNCTION__);
    return ret;
}

int voxl_spi_transfer_cs(int bus, uint8_t* write_buf, int write_len, uint8_t* read_buf, int read_len, int cs_gpio)
{
    if(write_len<0 || write_len>(DSPAL_SPI_RECEIVE_BUFFER_LENGTH-1)){
        fprintf(stderr, "ERROR in %s can only write up to %d bytes\n", __FUNCTION__,\
                                                (DSPAL_SPI_RECEIVE_BUFFER_LENGTH-1));
        return -1;
    }
    if(read_len<0 || read_len>(DSPAL_SPI_RECEIVE_BUFFER_LENGTH-1)){
        fprintf(stderr, "ERROR in %s can only read up to %d bytes\n", __FUNCTION__,\
                                                (DSPAL_SPI_RECEIVE_BUFFER_LENGTH-1));
        return -1;
    }
    int ret = io_rpc_spi_transfer_cs(bus, write_buf, write_len, read_buf, read_len,cs_gpio);
    if(ret) fprintf(stderr, "%s failed\n",__FUNCTION__);
    return ret;
}



int voxl_spi_write_reg_byte(int bus, uint8_t address, uint8_t val)
{
    int ret = io_rpc_spi_write_reg_byte(bus, address, val);
    if(ret) fprintf(stderr, "io_rpc_spi_write_reg_byte() failed\n");
    return ret;
}

int voxl_spi_write_reg_byte_cs(int bus, uint8_t address, uint8_t val, int cs_gpio)
{
    int ret = io_rpc_spi_write_reg_byte_cs(bus, address, val, cs_gpio);
    if(ret) fprintf(stderr, "io_rpc_spi_write_reg_byte_cs() failed\n");
    return ret;
}


int voxl_spi_write_reg_word(int bus, uint8_t address, uint16_t val)
{
    int ret = io_rpc_spi_write_reg_word(bus, address, val);
    if(ret) fprintf(stderr, "io_rpc_spi_write_reg_word() failed\n");
    return ret;
}

int voxl_spi_write_reg_word_cs(int bus, uint8_t address, uint16_t val, int cs_gpio)
{
    int ret = io_rpc_spi_write_reg_word_cs(bus, address, val, cs_gpio);
    if(ret) fprintf(stderr, "io_rpc_spi_write_reg_word_cs() failed\n");
    return ret;
}



int voxl_spi_read_reg(int bus, uint8_t address, uint8_t* out_buf, int length)
{

    if(length<0 || length>(DSPAL_SPI_RECEIVE_BUFFER_LENGTH-1)){
        fprintf(stderr, "ERROR in voxl_spi_read_reg, can only read up to %d bytes\n",\
                                                (DSPAL_SPI_RECEIVE_BUFFER_LENGTH-1));
        return -1;
    }
    int ret = io_rpc_spi_read_reg(bus, address, out_buf, length);
    if(ret) fprintf(stderr, "io_rpc_spi_read_reg() failed\n");
    return ret;
}

int voxl_spi_read_reg_cs(int bus, uint8_t address, uint8_t* out_buf, int length, int cs_gpio)
{
    if(length<0 || length>(DSPAL_SPI_RECEIVE_BUFFER_LENGTH-1)){
        fprintf(stderr, "ERROR in voxl_spi_read_reg_cs, can only read up to %d bytes\n",\
                                                (DSPAL_SPI_RECEIVE_BUFFER_LENGTH-1));
        return -1;
    }
    int ret = io_rpc_spi_read_reg_cs(bus, address, out_buf, length, cs_gpio);
    if(ret) fprintf(stderr, "io_rpc_spi_read_reg_cs() failed\n");
    return ret;
}


int voxl_spi_read_reg_byte(int bus, uint8_t address, uint8_t* out)
{
    int ret = io_rpc_spi_read_reg_byte(bus, address, out);
    if(ret) fprintf(stderr, "io_rpc_spi_read_reg_byte() failed\n");
    return ret;
}

int voxl_spi_read_reg_byte_cs(int bus, uint8_t address, uint8_t* out, int cs_gpio)
{
    int ret = io_rpc_spi_read_reg_byte_cs(bus, address, out, cs_gpio);
    if(ret) fprintf(stderr, "io_rpc_spi_read_reg_byte_cs() failed\n");
    return ret;
}

int voxl_spi_read_reg_word(int bus, uint8_t address, uint16_t* out)
{
    int ret = io_rpc_spi_read_reg_word(bus, address, out);
    if(ret) fprintf(stderr, "io_rpc_spi_read_reg_word() failed\n");
    return ret;
}

int voxl_spi_read_reg_word_cs(int bus, uint8_t address, uint16_t* out, int cs_gpio)
{
    int ret = io_rpc_spi_read_reg_word_cs(bus, address, out, cs_gpio);
    if(ret) fprintf(stderr, "io_rpc_spi_read_reg_word_cs() failed\n");
    return ret;
}

int voxl_spi_read_imu_fifo( int bus, uint8_t count_address, uint8_t fifo_address,\
                            uint8_t packet_size, int min_packets, int* packets_read,\
                            uint8_t* data, int dataLen, int count_speed, int data_speed)
{
    int ret = io_rpc_spi_read_imu_fifo(bus, count_address, fifo_address, \
                        packet_size, min_packets, packets_read, data, dataLen,\
                        count_speed, data_speed);
    if(ret) fprintf(stderr, "io_rpc_spi_read_imu_fifo() failed\n");
    return ret;
}

int voxl_spi_gather_flir_segment(int spi_bus, uint8_t* data, uint8_t* segBufin)
{
    int ret = io_rpc_gather_flir_segment(spi_bus, segBufin, VOSPI_SEGMENT);
    if (ret) fprintf(stderr, "io_rpc_gather_flir_segment() failed\n");
    memcpy(&data[0], &segBufin[0], VOSPI_SEGMENT);
    return ret;
}


