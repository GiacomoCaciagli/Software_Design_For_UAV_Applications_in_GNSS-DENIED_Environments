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

#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <voxl_io.h>

#define DEFAULT_LENGTH 10

int main(int argc, char** argv) {
    int ret;

    const int MAX_BUF_LEN = 2048;
    char test_str[MAX_BUF_LEN];

    int size = 10; //strlen(test_str); // get number of bytes in test string
    int bus = 5;
    int baudrate = 57600;

    char * p = &test_str[0];

    //initialize the string with 0,1,2,3.....X,Y,Z pattern
    while ( p < &test_str[MAX_BUF_LEN] ){
        for (int i=48; i<91; i++)
        {
            if (p >= &test_str[MAX_BUF_LEN])
                break;
            *p++ = i;
        }
    }

    uint8_t* read_data_buffer = voxl_rpc_shared_mem_alloc(size);
    if(read_data_buffer==NULL){
        fprintf(stderr, "failed to allocate shared rpc memory\n");
        return -1;
    }

    int bytes_read = 0;

    // Read data from UART
    printf("Reading back data...\n");
    memset(read_data_buffer,0,size);
    bytes_read = voxl_uart_read(bus, read_data_buffer, size);
    if(bytes_read!=size) {
        ret=1; // FAIL
        fprintf(stderr, "ERROR: voxl_uart_read %d bytes, expected %d\n", bytes_read, size);
    }
    else{
        printf("Received %d bytes:\n", bytes_read);

        ret=0; // PASS

        if (bytes_read != size)
        {
            printf("Sent number of bytes != received: %d != %d\n",size,bytes_read);
            ret = 1;
        }

        for (int i=0; i < bytes_read; i++){
            printf("%c",read_data_buffer[i]);
            if (test_str[i]!=read_data_buffer[i])
            {
                printf("\n...data mismatch at position %d: %c != %c\n",i,test_str[i],read_data_buffer[i]);
                ret = 1;
            }
        }

        printf("\n");
    }

    // cleanup shared memory
    voxl_rpc_shared_mem_free(read_data_buffer);
    voxl_rpc_shared_mem_deinit();
    
    return ret;
}
