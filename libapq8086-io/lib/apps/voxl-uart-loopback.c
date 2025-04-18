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

static void print_usage(){
    printf("Parameters:\n");
    printf("\t-b\tSpecify baudrate, default = 57600\n");
    printf("\t-d\tSpecify sDSP UART bus number X");
    printf("\t-h\tView this help message\n");
    printf("\t-s\tSet test buffer size, default = 10\n");
    printf("\n");
    printf("Available sDSP UART ports and their bus numbers:\n");
    printf("UART_J7     9   // BLSP 9  on physical port J7  pins 2&3\n");
    printf("UART_J10    7   // BLSP 7  on physical port J10 pins 2&3\n");
    printf("UART_J11    12  // BLSP 12 on physical port J11 pins 2&3\n");
    printf("UART_J12    5   // BLSP 5  on physical port J12 pins 2&3\n");
    return;
}

int main(int argc, char** argv) {
    int opt;
    int ret;

    const int MAX_BUF_LEN = 2048;
    char test_str[MAX_BUF_LEN];

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

    int size = 10; //strlen(test_str); // get number of bytes in test string
    int bus = -1;
    int baudrate = 57600;

    while((opt = getopt(argc, argv, ":b:d:hs:")) != -1) {
        switch(opt) {
            case 'b':
                baudrate = atoi(optarg);
                if(baudrate < 0) {
                    printf("Error: negative baudrate provided\n");
                    return -1;
                }
                break;
            case 'd':
                bus = atoi(optarg);
                if(bus < 0) {
                    printf("Error: negative bus index provided\n");
                    return -1;
                }
                break;
            case 'h':
                print_usage();
                return 1;
            case 's':
                size = atoi(optarg);
                if(size < 0) {
                    printf("Error: negative buffer size provided\n");
                    return -1;
                }
                if (size > MAX_BUF_LEN) {
                  printf("Error: size has to be smaller than %d (it can be increased if needed)\n",MAX_BUF_LEN);
                  return -1;
                }
                break;
            case ':':
                printf("option needs a value: %c\n", optopt);
                print_usage();
                break;
            case '?':
                printf("unknown option: %c\n", optopt);
                print_usage();
                break;
        }
    }

    if(bus < 0) {
        printf("Error: Insufficient input parameters\n");
        print_usage();
        return -1;
    }

    uint8_t* read_data_buffer = voxl_rpc_shared_mem_alloc(size);
    if(read_data_buffer==NULL){
        fprintf(stderr, "failed to allocate shared rpc memory\n");
        return -1;
    }

    uint8_t counter = 0;
    uint32_t cnt = 0;
    uint32_t err_msg_count = 0;
    uint32_t err_byte_count = 0;

    int bytes_read = 0;

    printf("INITIALIZING\n");
    ret = voxl_uart_init(bus, baudrate);
    if(ret) {
        fprintf(stderr, "ERROR initializing uart\n");
        return -1;
    }

    // Write buffer to UART
    printf("Sending %d bytes:\n", size);
    for (int i=0; i < size; i++){
        printf("%c",test_str[i]);
    }
    printf("\n");
    ret = voxl_uart_write(bus, (uint8_t*)test_str, size);
    if(ret!=size) {
        fprintf(stderr, "ERROR: voxl_uart_write wrote %d bytes, expected %d\n", ret, size);
        return -1;
    }

    // Wait a bit before reading
    uint32_t expected_trx_time_us = size * 1000000 * 1.0 / (baudrate/10); //effective throughput is baud rate / 10 (1 start bit, 1 stop bit)
    uint32_t uart_overhead_us     = 5000;
    uint32_t sleep_time_us        = expected_trx_time_us + uart_overhead_us;
    printf("Transfer should take ~%dus, sleeping for %dus (+%dus overhead)\n",expected_trx_time_us, sleep_time_us, uart_overhead_us);
    usleep(sleep_time_us);

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

    printf("closing uart bus\n");
    if(voxl_uart_close(bus)){
        fprintf(stderr, "ERROR closing uart\n");
    }

    // cleanup shared memory
    voxl_rpc_shared_mem_free(read_data_buffer);
    voxl_rpc_shared_mem_deinit();
    
    if(ret) printf("FAIL\n");
    else printf("PASS\n");

    return ret;
}
