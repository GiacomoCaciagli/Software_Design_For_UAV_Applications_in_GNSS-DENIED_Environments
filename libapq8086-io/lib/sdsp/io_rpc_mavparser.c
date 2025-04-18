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
 * @file io_rpc_mavparser.c
 *
 * SDSP code implementing the uart functions in io_rpc.idl
 */

#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <HAP_farf.h>
#include <io_rpc.h>

#include <common/mavlink.h>

#define PRINT(...)          FARF(ALWAYS, __VA_ARGS__);
#define MAX_BUS             16
#define UART_READ_BUF_LEN   512
#define MAV_CHAN            0

// local vars
static int buflens[MAX_BUS+1];
static unsigned char* read_bufs[MAX_BUS+1];
static mavlink_message_t* msg_bufs[MAX_BUS+1];
static uint8_t* status_bufs[MAX_BUS+1];
static int max_msg_buf_entries[MAX_BUS+1]; // max number of messages msg buffer can hold
static int msg_buf_entries[MAX_BUS+1]; // number of messages in message buffer
static int running[MAX_BUS+1];
static pthread_t threads[MAX_BUS+1];
static pthread_mutex_t mutex[MAX_BUS+1];


void* thread_func(void* context)
{
    int bus = (int)context;
    int i;
    int32 bytes_read;
    mavlink_message_t msg;
    memset(&msg, 0, sizeof(mavlink_message_t));
    mavlink_status_t status;
    memset(&status, 0, sizeof(mavlink_status_t));
    uint8_t framing_status = 0;

    // keep reading until running flag set to 0 by the close function
    while(running[bus]){

        // read up to the buffer size, normally much less
        io_rpc_uart_read(bus, read_bufs[bus], UART_READ_BUF_LEN, &bytes_read);

        if(bytes_read<1){
            usleep(100);
            continue;
        }

        // do the mavlink byte-by-byte parsing
        for(i=0; i<bytes_read; i++){
            framing_status = mavlink_frame_char(MAV_CHAN, (uint8_t)read_bufs[bus][i], &msg, &status);

            // new message found
            // possibly bad CRC or bad signature, but definitely not incomplete
            if(framing_status!=MAVLINK_FRAMING_INCOMPLETE){
                pthread_mutex_lock(&mutex[bus]);
                if(msg_buf_entries[bus]>=max_msg_buf_entries[bus]){
                    // TODO, handle overflow better. as long as buffer is huge
                    // this hasn't been a problem yet
                    //PRINT("WARNING msg buffer full\n");
                }
                else{ // copy to buffer
                    int index = msg_buf_entries[bus];
                    msg_bufs[bus][index] = msg;
                    status_bufs[bus][index] = framing_status;
                    msg_buf_entries[bus]++;
                }
                pthread_mutex_unlock(&mutex[bus]);
            }
        }
    }

    PRINT("mavparser thread %d exiting cleanly", bus);
    return NULL;
}



int io_rpc_mavparser_init(int32 bus, int32 baudrate, int32 buflen)
{
    // sanity checks
    if(bus<0 || bus>MAX_BUS){
        PRINT("ERROR in %s, uart bus must be between 0 & %d", __FUNCTION__, MAX_BUS);
        return -1;
    }
    if(running[bus]){
        PRINT("ERROR in %s, thread already running", __FUNCTION__);
        PRINT("trying to close it now");
        if(io_rpc_mavparser_close(bus)) return -1;
    }

    msg_buf_entries[bus] = 0;
    max_msg_buf_entries[bus] = buflen/sizeof(mavlink_message_t);
    if(max_msg_buf_entries[bus]<=2){
        PRINT("ERROR in %s, buflen too small", __FUNCTION__);
        return -1;
    }

    // start by opening the uart bus
    if(io_rpc_uart_init(bus, baudrate)) return -1;

    // allocate memory for the message and read buffer
    msg_bufs[bus] = malloc(buflen);
    if(msg_bufs[bus]==NULL){
        PRINT("ERROR in %s, failed to malloc msg buffer", __FUNCTION__);
        return -1;
    }
    read_bufs[bus] = malloc(UART_READ_BUF_LEN);
    if(msg_bufs[bus]==NULL){
        PRINT("ERROR in %s, failed to malloc read buffer", __FUNCTION__);
        return -1;
    }
    status_bufs[bus] = malloc(max_msg_buf_entries[bus]);
    if(status_bufs[bus]==NULL){
        PRINT("ERROR in %s, failed to malloc status buffer", __FUNCTION__);
        return -1;
    }

    // start thread
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    size_t stack_size = 4 * 1024;
    if(pthread_attr_setstacksize(&attr, stack_size) != 0){
        PRINT("pthread_attr_setstacksize returned error");
        return -1;
    }

    //create a new thread
    running[bus] = 1;
    mutex[bus] = PTHREAD_MUTEX_INITIALIZER;
    if(pthread_create(&threads[bus], &attr, thread_func, (void*)bus) != 0){
        PRINT("pthread_create returned error");
        return -1;
    }
    pthread_attr_destroy(&attr);

    return 0;
}



int io_rpc_mavparser_read(int32 bus, unsigned char* data, int dataLen, int32* msg_read, unsigned char* status, int statusLen)
{
    // sanity checks
    if(bus<0 || bus>MAX_BUS){
        PRINT("ERROR in %s, uart bus must be between 0 & %d", __FUNCTION__, MAX_BUS);
        return -1;
    }
    if(!running[bus]){
        PRINT("ERROR in %s, not running yet", __FUNCTION__);
        return -1;
    }
    if((unsigned int)dataLen < max_msg_buf_entries[bus]*sizeof(mavlink_message_t)){
        PRINT("ERROR in %s, user's buffer length too small", __FUNCTION__);
        return -1;
    }

    // record current time for timeout check later
    int64_t t_start, t_current;
    io_rpc_sdsp_time_monotonic_ns(&t_start);

    // make sure results are 0 unless conditions are met later
    *msg_read = 0;

    // keep waiting until either a message is ready, there is a parsing error
    // or we hit the half-secnd timeout
    int ready_to_return = 0;
    while(!ready_to_return && running[bus]){

        // lock mutex so we don't trample the thread
        pthread_mutex_lock(&mutex[bus]);

        // copy data out if we have anything to read
        if(msg_buf_entries[bus]>0){
            ready_to_return = 1;
            memcpy(data, msg_bufs[bus], msg_buf_entries[bus]*sizeof(mavlink_message_t));
            memcpy(status, status_bufs[bus], msg_buf_entries[bus]);
            *msg_read = msg_buf_entries[bus];
            msg_buf_entries[bus] = 0;
        }


        // unlock mutex
        pthread_mutex_unlock(&mutex[bus]);

        // sleep if none of the conditions were met
        if(!ready_to_return){
            io_rpc_sdsp_time_monotonic_ns(&t_current);
            if((t_current-t_start)>500000000) ready_to_return = 1;
            else usleep(200);
        }
    }

    return 0;
}


int io_rpc_mavparser_close(int32 bus)
{
    // sanity checks
    if(bus<0 || bus>MAX_BUS){
        PRINT("ERROR in %s, bus must be between 0 & %d", __FUNCTION__, MAX_BUS);
        return -1;
    }

    running[bus] = 0;

    // wait until thread is finished
    if(pthread_join(threads[bus], NULL) != 0){
        PRINT("pthread_join returned error");
        return -1;
    }

    // cleanup uart bus and free memory
    io_rpc_uart_close(bus);
    free(read_bufs[bus]);
    free(msg_bufs[bus]);
    read_bufs[bus] = NULL;
    msg_bufs[bus] = NULL;
    threads[bus] = 0;
    return 0;
}
