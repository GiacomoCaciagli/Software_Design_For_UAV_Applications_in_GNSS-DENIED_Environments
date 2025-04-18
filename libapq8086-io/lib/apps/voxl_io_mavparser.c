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
#include <common/mavlink.h>

#define MAX_BUS     16
static int buflens[MAX_BUS+1];
static int status_buflens[MAX_BUS+1];


int voxl_mavparser_init(int bus, int baudrate, int buflen, __attribute__((unused))int reserved)
{
    // sanity checks
    if(bus<0 || bus>MAX_BUS){
        fprintf(stderr,"ERROR in %s, bus must be between 0 & %d\n", __FUNCTION__, MAX_BUS);
        return -1;
    }

    // pass through arguments to sDSP
    if(io_rpc_mavparser_init(bus, baudrate, buflen)){
        fprintf(stderr, "ERROR in %s\n", __FUNCTION__);
        return -1;
    }
    buflens[bus] = buflen;
    status_buflens[bus] = buflen/sizeof(mavlink_message_t);
    return 0;
}


int voxl_mavparser_read(int bus, char* data, uint8_t* status)
{
    // sanity checks
    if(bus<0 || bus>MAX_BUS){
        fprintf(stderr,"ERROR in %s, bus must be between 0 & %d\n", __FUNCTION__, MAX_BUS);
        return -1;
    }
    if(!buflens[bus]){
        fprintf(stderr,"ERROR in %s, bus %d not initialized yet\n", __FUNCTION__, bus);
        return -1;
    }
    if(data==NULL || status==NULL){
        fprintf(stderr,"ERROR in %s, received NULL pointer\n", __FUNCTION__);
        return -1;
    }

    int msgs_read = 0;
    if(io_rpc_mavparser_read(bus, (unsigned char*)data, buflens[bus], &msgs_read, (unsigned char*)status, status_buflens[bus])){
        fprintf(stderr,"ERROR in %s calling RPC\n", __FUNCTION__);
        return -1;
    }
    return msgs_read;
}



int voxl_mavparser_close(int bus)
{
    // sanity checks
    if(bus<0 || bus>MAX_BUS){
        fprintf(stderr,"ERROR in %s, bus must be between 0 & %d\n", __FUNCTION__, MAX_BUS);
        return -1;
    }
    if(io_rpc_mavparser_close(bus)){
        fprintf(stderr,"ERROR in %s calling RPC\n", __FUNCTION__);
        return -1;
    }
    return 0;
}
