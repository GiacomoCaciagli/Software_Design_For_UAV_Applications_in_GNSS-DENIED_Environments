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
 * @file voxl_io_misc.c
 *
 * Apps-proc code for misc functions of libvoxl_io
 */

#include <stdio.h>
#include <string.h>     // for memset
#include <voxl_io.h>    // user-facing function declarations defined in this file
#include <io_rpc.h>     // sdsp rpc calls from idl file
#include <stdlib.h>     // for strtoull
#include <fcntl.h>      // for open()
#include <unistd.h>     // for read()
#include <limits.h>     // for LONG_MIN
#include <time.h>

#include "rpcmem.h"


// set to 1 on call to rpcmem_init
int rpcmem_initialized = 0;



// TODO, investigate safer handling of alloc, checking for max sizes etc
uint8_t* voxl_rpc_shared_mem_alloc(size_t bytes)
{
    uint8_t* ret;
    if(!rpcmem_initialized){
        rpcmem_init();
        rpcmem_initialized = 1;
    }

    ret = rpcmem_alloc(0, RPCMEM_HEAP_DEFAULT, bytes);
    if(ret == NULL){
        fprintf(stderr,"ERROR in voxl_alloc_rpc_shared_mem, rpcmem_alloc failed\n");
    }
    return ret;
}


void voxl_rpc_shared_mem_free(uint8_t* ptr)
{
    rpcmem_free(ptr);
    return;
}


void voxl_rpc_shared_mem_deinit()
{
    if(rpcmem_initialized) rpcmem_deinit();
    rpcmem_initialized = 0;
    return;
}


int64_t voxl_sdsp_time_monotonic_ns()
{
    int64_t t;
    if(io_rpc_sdsp_time_monotonic_ns(&t)){
        fprintf(stderr,"ERROR calling io_rpc_sdsp_time_monotonic_ns\n");
        return -1;
    }
    return t;
}


int64_t voxl_sdsp_time_realtime_ns()
{
    int64_t t;
    if(io_rpc_sdsp_time_realtime_ns(&t)){
        fprintf(stderr,"ERROR calling io_rpc_sdsp_time_realtime_ns\n");
        return -1;
    }
    return t;
}


int64_t voxl_apps_time_monotonic_ns()
{
    struct timespec ts;
    if(clock_gettime(CLOCK_MONOTONIC, &ts)){
        fprintf(stderr,"ERROR calling clock_gettime\n");
        return -1;
    }
    return (int64_t)ts.tv_sec*1000000000 + (int64_t)ts.tv_nsec;
}


int64_t voxl_apps_time_realtime_ns()
{
    struct timespec ts;
    if(clock_gettime(CLOCK_REALTIME, &ts)){
        fprintf(stderr,"ERROR calling clock_gettime\n");
        return -1;
    }
    return (int64_t)ts.tv_sec*1000000000 + (int64_t)ts.tv_nsec;
}


int64_t voxl_sdsp_time_offset_ns()
{
    static const char qdspTimerTickPath[] = "/sys/kernel/boot_slpi/qdsp_qtimer" ;
    char qdspTicksStr[20] = "";
    int64_t offset_ns = 0;
    int64_t greatest_offet_ns = LONG_MIN; // final value to return

    // do 10 times and picks the smallest difference
    for(int i=0;i<1000;i++){

        int qdspClockfd = open(qdspTimerTickPath, O_RDONLY);
        if(qdspClockfd == -1){
            perror("ERROR in voxl_sdsp_time_offset_ns opening timer path");
            return -1;
        }

        int rCount = read(qdspClockfd, (void*)qdspTicksStr, 16);
        if(rCount <=0){
            perror("ERROR in voxl_sdsp_time_offset_ns reading timer path");
            close(qdspClockfd);
            return -1;
        }
        close(qdspClockfd);

        // hopefully no other lengthy, higher priority thread preempts here
        // any context switch here will increase the m_OffsetInNs
        // that is why we do multiple iterations to pick the smallest diffrence value
        // Ideally we should do context switch lock here

        qdspTicksStr[rCount] = 0; // string terminator
        uint64_t qdspTicks = strtoull( qdspTicksStr, 0, 16 );
        // convert ticks to nanoseconds given a 19.2Mhz clock
        uint64_t megaticks = qdspTicks * 1000000;
        uint64_t sdsp_time_ns = (megaticks)/19200;
        uint64_t apps_time_ns = voxl_apps_time_monotonic_ns();

        // now compute the offset.
        offset_ns = sdsp_time_ns - apps_time_ns;
        if(offset_ns>greatest_offet_ns) greatest_offet_ns = offset_ns;

        // printf("i:%d apps_ns: %12lld - sdsp_ns %12lld = %12lld \n", i, apps_time_ns, sdsp_time_ns, offset_ns);
    }
    return greatest_offet_ns/1000;


    /* less accurate and very slow but more obvious method */
    /*
    int64_t last_apps_ns;
    int64_t sdsp_ns;
    int64_t current_apps_ns;
    int64_t smallest_delta = LONG_MAX;
    int64_t offset_ns;

    last_apps_ns = voxl_apps_time_monotonic_ns();
    for(int i=0;i<1000;i++){

        sdsp_ns         = voxl_sdsp_time_monotonic_ns();
        current_apps_ns = voxl_apps_time_monotonic_ns();
        int64_t delta = current_apps_ns - last_apps_ns;

        // record the time offset of our fastest round-trip ping
        if(delta<smallest_delta){
            smallest_delta = delta;
            offset_ns = sdsp_ns-last_apps_ns-(delta/2);
        }

        last_apps_ns = current_apps_ns;
    }
    return offset_ns/1000;
    */
}

