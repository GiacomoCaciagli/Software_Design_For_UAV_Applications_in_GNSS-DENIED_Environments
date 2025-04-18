
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <voxl_io.h>

#define DEFAULT_LENGTH 10

int main(int argc, char** argv) {

    //int bus = -1;
    int bus = 5;
    int baudrate = 57600;
    int err;

    printf("closing uart bus\n");
    err=voxl_uart_close(bus);
    if(err){
        fprintf(stderr, "ERROR closing uart, err:%d\n",err);
    }

    return 0;
}
