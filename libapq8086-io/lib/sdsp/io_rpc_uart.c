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
 * @file io_rpc_uart.c
 *
 * SDSP code implementing the uart functions in io_rpc.idl
 */

#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <termios.h>
#include <HAP_farf.h>
#include <sys/ioctl.h>
#include <io_rpc.h>

#include <dev_fs_lib_serial.h>

#define PRINT(...)  FARF(ALWAYS, __VA_ARGS__);
#define MAX_BUS     16

// file descriptors for all ports
static int uart_fd[MAX_BUS+1];

int io_rpc_uart_init(int32 bus, int32 baudrate)
{
    int tmpfd;
    char buf[16];
    struct termios config;

    // sanity checks
    if(bus<0 || bus>MAX_BUS){
        PRINT("ERROR in io_rpc_uart_init, bus must be between 0 & %d", MAX_BUS);
        return -1;
    }

    if(uart_fd[bus]){
        PRINT("WARNING in io_rpc_uart_init, trying to initialize bus %d when already initialized", bus);
        return 2;
    }

    // wipe tc_config and start setting flags
    memset(&config,0,sizeof(config));

    /*
     * Right now DSPAL only supports these 3 settings as-is!
     * hard code them for now, make configurable when DSPAL updates
     */
    const int canonical_en = 0;
    const int parity_en = 0;
    const int stop_bits = 1;

    if(canonical_en) config.c_lflag |= ICANON; // enable canonical mode
    else config.c_lflag &= ~ICANON;

    if(parity_en) config.c_cflag |= PARENB; // enable parity
    else config.c_cflag &= ~PARENB;

    if(stop_bits==1) config.c_cflag &= ~CSTOPB; // disable 2 stop bits (use just 1)
    else config.c_cflag |= CSTOPB; // enable 2 stop bits

    config.c_cflag &= ~CSIZE;   // wipe all size masks
    config.c_cflag |= CS8;      // set size to 8 bit characters
    config.c_cflag |= CREAD;    // enable reading
    config.c_cflag |= CLOCAL;   // ignore modem status lines

    //speed_t speed = baudrate;
    speed_t speed = 9600;  //set some default speed, since will use io_rpc_uart_set_baud_rate below

    // set speed in config struct
    if(cfsetispeed(&config, speed)==-1){
        PRINT("ERROR: in voxl_uart_init calling cfsetispeed");
        return -1;
    }
    if(cfsetospeed(&config, speed)==-1){
        PRINT("ERROR: in voxl_uart_init calling cfsetospeed");
        return -1;
    }

    // open file descriptor for blocking reads
    snprintf(buf,sizeof(buf),"/dev/tty-%d", (int)bus);
    tmpfd = open(buf, O_RDWR);
    if(tmpfd==-1){
        PRINT("ERROR in io_rpc_uart_init while opening file descriptor %s", buf);
        PRINT("open() returned %d", tmpfd);
        PRINT("errno=%d",errno);
        return -1;
    }

    // flush and set attributes
    if(tcflush(tmpfd,TCIOFLUSH)==-1){
        PRINT("ERROR in io_rpc_uart_init calling tcflush");
        close(tmpfd);
        return 3;
    }
    if(tcsetattr(tmpfd, TCSANOW, &config) < 0) {
        PRINT("cannot set uart%d attributes", bus);
        close(uart_fd[bus]);
        return -1;
    }
    if(tcflush(tmpfd,TCIOFLUSH)==-1){
        PRINT("ERROR in io_rpc_uart_init calling tcflush");
        close(tmpfd);
        return -1;
    }

    uart_fd[bus]=tmpfd;

    if (io_rpc_uart_set_baud_rate(bus, baudrate)==-1){
        PRINT("ERROR in io_rpc_uart_init calling io_rpc_uart_set_baud_rate");
        close(tmpfd);
        uart_fd[bus]=0;
        return -1;
    }

    /*if(strcmp(buf,"/dev/tty-5")==0){
        return 1;
    }*/

    return 0;
}


int io_rpc_uart_close(int32 bus)
{
    // sanity checks
    if(bus<0 || bus>MAX_BUS){
        PRINT("ERROR in io_rpc_uart_close, bus must be between 0 & %d", MAX_BUS);
        return -1;
    }
    // if not initialized already, return
    if(uart_fd[bus]==0){
        printf("Not Initialized");
        return 1;
    }
    // flush and close
    tcflush(uart_fd[bus],TCIOFLUSH);
    close(uart_fd[bus]);
    uart_fd[bus]=0;
    return 0;
}


int io_rpc_uart_write(int32 bus, const unsigned char* data, int dataLen, int32* bytes_written)
{
    // sanity checks
    PRINT("non funziona dio")
    if(bus<0 || bus>MAX_BUS){
        PRINT("ERROR in io_rpc_uart_write, bus must be between 0 & %d", MAX_BUS);
        return 1;
    }
    if(dataLen<1){
        PRINT("ERROR in io_rpc_uart_write, number of bytes to send must be >1");
        return 2;
    }
    if(uart_fd[bus]==0){
        PRINT("ERROR in io_rpc_uart_write, bus %d must be initialized first", bus);
        return 3;
    }
    int ret = write(uart_fd[bus], data, dataLen);
    if(ret==-1){
        PRINT("ERROR in io_rpc_uart_write, write() returned %d", ret);
        PRINT("errno=%d",errno);
        *bytes_written = ret;
        return 4;
    }
    *bytes_written = ret;
    return 0;
}


int io_rpc_uart_read(int32 bus, unsigned char* data, int dataLen, int32* bytes_read)
{
    // sanity checks
    if(bus<0 || bus>MAX_BUS){
        PRINT("ERROR in io_rpc_uart_read, uart bus must be between 0 & %d", MAX_BUS);
        return 1;
    }
    if(dataLen<1){
        PRINT("ERROR in io_rpc_uart_read, number of bytes to read must be >=1");
        return 2;
    }
    if(uart_fd[bus]==0){
        PRINT("ERROR in io_rpc_uart_read, bus %d must be initialized first", bus);
        return 3;
    }
    int ret = read(uart_fd[bus], data, dataLen);
    if(ret==-1){
        PRINT("ERROR in io_rpc_uart_read, read() returned %d", ret);
        PRINT("errno=%d",errno);
        *bytes_read = ret;
        return 4;
    }
    *bytes_read = ret;
    return 0;
}


int io_rpc_uart_flush(int32 bus)
{
    // sanity checks
    if(bus<0 || bus>MAX_BUS){
        PRINT("ERROR in io_rpc_uart_flush, bus must be between 0 & %d", MAX_BUS);
        return -1;
    }
    if(uart_fd[bus]==0){
        PRINT("ERROR in io_rpc_uart_flush, bus %d must be initialized first", bus);
        return -1;
    }
    if(tcflush(uart_fd[bus],TCIOFLUSH)==-1){
        PRINT("ERROR in io_rpc_uart_flush calling tcflush");
        PRINT("errno=%d",errno);
        return -1;
    }
    return 0;
}


int io_rpc_uart_drain(int32 bus)
{
    // sanity checks
    if(bus<0 || bus>MAX_BUS){
        PRINT("ERROR in io_rpc_uart_drain, bus must be between 0 & %d", MAX_BUS);
        return -1;
    }
    if(uart_fd[bus]==0){
        PRINT("ERROR in io_rpc_uart_drain, bus %d must be initialized first", bus);
        return -1;
    }
    if(tcdrain(uart_fd[bus])==-1){
        PRINT("ERROR in io_rpc_uart_drain calling tcdrain");
        PRINT("errno=%d",errno);
        return -1;
    }
    return 0;
}

int io_rpc_uart_set_baud_rate(int32 bus, int32 baudrate)
{
    // sanity checks
    if(bus<0 || bus>MAX_BUS){
        PRINT("ERROR in io_rpc_uart_set_baud_rate, bus must be between 0 & %d", MAX_BUS);
        return -1;
    }
    if(uart_fd[bus]==0){
        PRINT("ERROR in io_rpc_uart_set_baud_rate, bus %d must be initialized first", bus);
        return -1;
    }

    struct dspal_serial_ioctl_data_rate ioctl_baud_rate;

    switch (baudrate)
    {
        case 4800 :   ioctl_baud_rate.bit_rate = DSPAL_SIO_BITRATE_4800;    break;
        case 9600 :   ioctl_baud_rate.bit_rate = DSPAL_SIO_BITRATE_9600;    break;
        case 14400:   ioctl_baud_rate.bit_rate = DSPAL_SIO_BITRATE_14400;   break;
        case 19200:   ioctl_baud_rate.bit_rate = DSPAL_SIO_BITRATE_19200;   break;
        case 38400:   ioctl_baud_rate.bit_rate = DSPAL_SIO_BITRATE_38400;   break;
        case 57600:   ioctl_baud_rate.bit_rate = DSPAL_SIO_BITRATE_57600;   break;
        case 115200:  ioctl_baud_rate.bit_rate = DSPAL_SIO_BITRATE_115200;  break;
        case 230400:  ioctl_baud_rate.bit_rate = DSPAL_SIO_BITRATE_230400;  break;
        case 250000:  ioctl_baud_rate.bit_rate = DSPAL_SIO_BITRATE_250000;  break;
        case 460800:  ioctl_baud_rate.bit_rate = DSPAL_SIO_BITRATE_460800;  break;
        case 921600:  ioctl_baud_rate.bit_rate = DSPAL_SIO_BITRATE_921600;  break;
        case 2000000: ioctl_baud_rate.bit_rate = DSPAL_SIO_BITRATE_2000000; break;
        case 3000000: ioctl_baud_rate.bit_rate = DSPAL_SIO_BITRATE_3000000; break;
        case 4000000: ioctl_baud_rate.bit_rate = DSPAL_SIO_BITRATE_4000000; break;
        default:
            PRINT("ERROR in io_rpc_uart_set_baud_rate: unsupported baud rate : %d",baudrate);
            return -1;
    }

    int ioctl_ret = ioctl(uart_fd[bus], SERIAL_IOCTL_SET_DATA_RATE, &ioctl_baud_rate);
    if (ioctl_ret != 0)
    {
        PRINT("ERROR in io_rpc_uart_set_baud_rate: unable to set baud rate %d",baudrate);
        return -1;
    }

    return 0;
}
