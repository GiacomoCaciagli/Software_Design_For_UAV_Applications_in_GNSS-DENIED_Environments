
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <voxl_io.h>

#define DEFAULT_LENGTH 10

int main(int argc, char** argv) {
    int ret;
    
    //int bus = -1;
    int bus = 5;
    int baudrate = 57600;

    printf("INITIALIZING\n");
    ret = voxl_uart_init(bus, baudrate);
    if(ret) {
        fprintf(stderr, "ERROR initializing uart, err:%d\n",ret);
        return -1;
    }

    sleep(10);

    return ret;
}
