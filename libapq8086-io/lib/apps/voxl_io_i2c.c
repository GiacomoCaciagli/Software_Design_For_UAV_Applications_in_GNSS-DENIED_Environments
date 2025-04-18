/*******************************************************************************
 * Copyright 2020 ModalAI Inc.
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
 * @file voxl_io_i2c.c
 *
 * Apps-proc code for I2C functions of libvoxl_io. These are mostly wrappers
 * for the SDSP RPC calls in io_rpc.idl
 */

#include <stdio.h>
#include <string.h>     // for memset
#include <voxl_io.h>    // user-facing function declarations defined in this file
#include <io_rpc.h>     // sdsp rpc calls from idl file

static int check_i2c_bus_num(uint32_t bus, const char * calling_func)
{
    if(bus>=MAX_NUM_I2C_PORTS){
       fprintf(stderr, "ERROR in %s: i2c port must be between 0 & %d (provided %d)\n", calling_func, MAX_NUM_I2C_PORTS-1,bus);
       return -1;
    }
    return 0;
}

int voxl_i2c_init(uint32_t bus)
{
    if (check_i2c_bus_num(bus,__func__))
        return -1;

    if(io_rpc_i2c_init(bus)){
        fprintf(stderr, "io_rpc_i2c_init() failed\n");
        return -1;
    }

    return 0;
}

int voxl_i2c_close(uint32_t bus)
{
    if (check_i2c_bus_num(bus,__func__))
        return -1;

    if(io_rpc_i2c_close(bus)){
        fprintf(stderr, "io_rpc_i2c_close() failed\n");
        return -1;
    }

    return 0;
}

int voxl_i2c_slave_config(uint32_t bus, uint16_t slave_address, uint32_t bit_rate, uint32_t timeout_us)
{
    if (check_i2c_bus_num(bus,__func__))
        return -1;

    if(io_rpc_i2c_slave_config(bus, slave_address, bit_rate, timeout_us)){
        fprintf(stderr, "io_rpc_i2c_slave_config() failed\n");
        return -1;
    }

    return 0;
}


int voxl_i2c_write(uint32_t bus, uint32_t register_address, uint32_t register_address_num_bits, uint8_t * write_buffer, uint32_t write_length)
{
    if (check_i2c_bus_num(bus,__func__))
        return -1;

    if(io_rpc_i2c_write(bus, register_address, register_address_num_bits, write_buffer, write_length)){
        fprintf(stderr, "io_rpc_i2c_write() failed\n");
        return -1;
    }

    return 0;
}


int voxl_i2c_read(uint32_t bus, uint32_t register_address, uint32_t register_address_num_bits, uint8_t * read_buffer, uint32_t read_length)
{
    if (check_i2c_bus_num(bus,__func__))
        return -1;

    if(io_rpc_i2c_read(bus, register_address, register_address_num_bits, read_buffer, read_length)){
        fprintf(stderr, "io_rpc_i2c_read() failed\n");
        return -1;
    }

    return 0;
}
