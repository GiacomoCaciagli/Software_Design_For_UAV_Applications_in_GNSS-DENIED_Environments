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


#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <voxl_io.h>
#include <pthread.h>
#include "../c_library_v2/common/mavlink.h"

// main port connecting VOXL to PX4 Flight Core
#define UART_BUS	UART_J12
#define BAUDRATE	921600
#define BUF_LEN		(32*1024) // big 32k read buffer, each message is 291 bytes
#define BUF_PACKETS	(BUF_LEN/(int)sizeof(mavlink_message_t))

static int running = 0;
static mavlink_message_t* msg_buf;	// big rpc shared memory buffer for messages
static uint8_t* status_buf;			// to hold status of each read message


// called on sigint or sigterm to trigger clean shutdown
static void sigint_cb(int sig){
	fprintf(stderr, "received signal %d\n", sig);
	running = 0;
	return;
}


// note, reading must be done in a pthread, not in main()!!!
// the signal handler will interfere with the RPC read call otherwise
static void* thread_func(void* context)
{
	int msg_read = 0;
	int parse_errors = 0;
	int i;

	// this is a tight loop around the read function. Read will wait to return
	// untill there is either at least one message available or there was an error
	// this no need to sleep
	while(running){
		msg_read = voxl_mavparser_read(UART_BUS, (char*)msg_buf, status_buf);

		// handle unsuccessful returns
		if(msg_read>=BUF_PACKETS) fprintf(stderr, "WARNING, mavlink buffer full!\n");
		if(msg_read<0){
			fprintf(stderr, "ERROR voxl_mavparser_read returned %d, exiting\n", msg_read);
			fprintf(stderr, "perhaps voxl-vision-px4 or something else is using mavparser in the background?\n");
			running = 0;
			return NULL;
		}

		// no messages, the next read will block for another 1/2 second.
		if(msg_read==0){
			printf("no messages available\n");
			continue;
		}

		// print some info on each message received
		printf("read %d messages:\n", msg_read);
		for(i=0; i<msg_read;i++){
			printf("%3d msgid: %3d sysid:%3d compid:%3d status: %d", i, msg_buf[i].msgid, msg_buf[i].sysid, msg_buf[i].compid, status_buf[i]);
			// print status at the end of the line
			if     (status_buf[i] == 2)       printf(" BAD CRC\n");
			else if(status_buf[i] == MAVLINK_FRAMING_BAD_SIGNATURE) printf(" BAD SIGNATURE\n");
			else printf("\n");
		}

		// uncomment sleep to test cpu getting backed up and buffer getting full
		// usleep(350000);
	}

	return NULL;
}



int main()
{
	// set some basic signal handling for safe shutdown
	struct sigaction sigint_action = {.sa_handler=sigint_cb};
	sigaction(SIGINT, &sigint_action, 0);
	sigaction(SIGTERM, &sigint_action, 0);

	// initialize the read buffer and mavparser
	printf("allocating RPC shared memory\n");
	msg_buf = (mavlink_message_t*)voxl_rpc_shared_mem_alloc(BUF_LEN);
	status_buf = voxl_rpc_shared_mem_alloc(BUF_PACKETS);
	if(msg_buf==NULL || status_buf==NULL){
		fprintf(stderr, "failed to allocate shared memory buffers for mavparser\n");
		return -1;
	}
	if(voxl_mavparser_init(UART_BUS, BAUDRATE, BUF_LEN, 0)){
		fprintf(stderr, "failed to initialize mavparser\n");
		return -1;
	}

	// start the read thread
	running = 1;
	pthread_t thread_id;
	pthread_attr_t attr;
	pthread_attr_init(&attr);
	if(pthread_create(&thread_id, &attr, thread_func, NULL) != 0){
		fprintf(stderr, "couldn't start thread\n");
		return -1;
	}

	// wait for signal handler to trigger shutdown
	while(running){
		usleep(500000);
	}

	// make sure we join the thread and aren't calling the read function when
	// we stop the mavparser and free memory!!
	printf("joining read thread\n");
	pthread_join(thread_id, NULL);

	// close and cleanup
	voxl_mavparser_close(UART_BUS);
	voxl_rpc_shared_mem_free((uint8_t*)msg_buf);
	voxl_rpc_shared_mem_free((uint8_t*)status_buf);
	voxl_rpc_shared_mem_deinit();
	printf("exiting cleanly\n");
	return 0;
}
