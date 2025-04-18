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
 * @file voxl_io_uart.c
 *
 * Apps-proc code for uart functions of libvoxl_io. These are mostly wrappers
 * for the SDSP RPC calls in io_rpc.idl
 */

#include <stdio.h>
#include <string.h>     // for memset
#include <voxl_io.h>    // user-facing function declarations defined in this file
#include <io_rpc.h>     // sdsp rpc calls from idl file


#define MAX_BUS         16


int voxl_uart_init(int bus, int baudrate)
{
    int check;
    // sanity checks
    if(bus<0 || bus>MAX_BUS){
        fprintf(stderr,"ERROR in voxl_uart_init, bus must be between 0 & %d\n", MAX_BUS);
        return -1;
    }

    check=io_rpc_uart_init(bus, baudrate);
    if(check){
        fprintf(stderr, "io_rpc_uart_init() failed,check:%d\n",check);
        return -1;
    }

    return 0;
}


int voxl_uart_close(int bus)
{
    int ret = io_rpc_uart_close(bus);
    if(ret) fprintf(stderr, "io_rpc_uart_close() failed\n");
    return ret;
}


int voxl_uart_flush(int bus)
{
    int ret = io_rpc_uart_flush(bus);
    if(ret) fprintf(stderr, "io_rpc_uart_flush() failed\n");
    return ret;
}


int voxl_uart_drain(int bus)
{
    int ret = io_rpc_uart_drain(bus);
    if(ret) fprintf(stderr, "io_rpc_uart_drain() failed\n");
    return ret;
}


int voxl_uart_write(int bus, uint8_t* data, size_t bytes)
{
    int bytes_written;
    int check;
    check=io_rpc_uart_write(bus, data, bytes, &bytes_written);
    if(check){
        fprintf(stderr, "io_rpc_uart_write() failed,check:%d\n",check);
        return -1;
    }
    return bytes_written;
}


int voxl_uart_read(int bus, uint8_t* data, size_t bytes)
{
    int bytes_read;
    int check;
    check=io_rpc_uart_read(bus, data, bytes, &bytes_read);
    if(check){
        fprintf(stderr, "io_rpc_uart_read() failed, check:%d, bytes read:%d\n",check,bytes_read);
        return -1;
    }
    return bytes_read;
}
