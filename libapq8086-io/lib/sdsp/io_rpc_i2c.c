/*******************************************************************************
 * Copyright 2020 ModalAI Inc.
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
 * @file io_rpc_i2c.c
 *
 * SDSP code implementing the I2C functions in io_rpc.idl
 */

#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <termios.h>
#include <HAP_farf.h>
#include <sys/ioctl.h>
#include <io_rpc.h>
#include <dspal_time.h>
#include "voxl_io.h"
#include "dev_fs_lib_i2c.h"

#define PRINT(...)  FARF(ALWAYS, __VA_ARGS__);

// file descriptors for all possible i2c ports
static int i2c_fds[MAX_NUM_I2C_PORTS];

//ports that DSPaL does not allow opening: 0 and > 12
//ports that DSPaL allows opening and reading, but should be blocked : 1, 10 (using by on-board SPI IMUs)
//ports that DSPaL allows opening, but causes board crash : 2,4,5,11,12
//ports that work : 3,6,7,8
//ports that DSPaL allows opening and does not crash, need to be actually tested : 9 (marked as GPIO) on J7
static uint32_t i2c_port_whitelist[] = {3,6,7,8,9};

int64_t get_sdsp_time_realtime_ns()
{
    struct timespec ts;
    if(clock_gettime(CLOCK_REALTIME, &ts)) return -1;
    int64_t t = (int64_t)ts.tv_sec*1000000000 + (int64_t)ts.tv_nsec;
    return t;
}

static int check_i2c_bus_num(uint32_t i2c_bus, const char * calling_func)
{
    if(i2c_bus>=MAX_NUM_I2C_PORTS){
       PRINT("ERROR in %s: i2c bus number must be between 0 & %d (provided %d)", calling_func, MAX_NUM_I2C_PORTS-1,i2c_bus);
       return -1;
    }

    int valid_bus_num = 0;  //true / false
    int whitelist_size = sizeof(i2c_port_whitelist) / sizeof(i2c_port_whitelist[0]);
    int ii;
    for (ii=0;ii<whitelist_size;ii++){
        if (i2c_bus == i2c_port_whitelist[ii]){
            valid_bus_num = 1;
            break;
        }
    }

    if (valid_bus_num == 0){
        PRINT("ERROR in %s: i2c bus number %d is invalid", calling_func, i2c_bus);
        return -1;
    }

    return 0;
}

static int check_i2c_bus_init_status(uint32_t i2c_bus, const char * calling_func)
{
    if(i2c_fds[i2c_bus]==0){
       PRINT("ERROR in %s: i2c bus %d is not initialized", calling_func, i2c_bus);
       return -1;
    }
    return 0;
}

int io_rpc_i2c_init(uint32_t i2c_bus)
{
    if (check_i2c_bus_num(i2c_bus,__func__))
        return -1;

    if(i2c_fds[i2c_bus] != 0){
        PRINT("WARNING in io_rpc_i2c_init, trying to initialize i2c bus %d when already initialized", i2c_bus);
        return 0;
    }

    char device_path[16];
    snprintf(device_path,sizeof(device_path),"/dev/iic-%d", (int)i2c_bus);
    int i2c_fd = open(device_path,0);

    if (i2c_fd == -1){
        PRINT("ERROR in io_rpc_i2c_init: could not open device %s",device_path);
        return -1;
    }

    i2c_fds[i2c_bus] = i2c_fd;

    return 0;
}

int io_rpc_i2c_slave_config(uint32_t i2c_bus, uint32_t slave_address, uint32_t bit_rate, uint32_t timeout_us)
{
    if (check_i2c_bus_num(i2c_bus,__func__))
        return -1;

    if(check_i2c_bus_init_status(i2c_bus,__func__))
        return -1;

    struct dspal_i2c_ioctl_slave_config slave_config;
    memset(&slave_config, 0, sizeof(slave_config));

    slave_config.slave_address                 = slave_address & 1023;  //TODO: do we support 10-bit slave addressing??
    slave_config.bus_frequency_in_khz          = bit_rate/1000;
    slave_config.byte_transer_timeout_in_usecs = timeout_us;
    if (ioctl(i2c_fds[i2c_bus], I2C_IOCTL_SLAVE, &slave_config) != 0){
        PRINT("ERROR in io_rpc_i2c_slave_config, i2c bus %d, slave address %d, bit rate %d, timeout %d",i2c_bus, slave_address, bit_rate, timeout_us);
        return -1;
    }

    return 0;
}

int io_rpc_i2c_read(uint32_t i2c_bus, uint32_t register_address, uint32_t register_address_num_bits, uint8_t * read_buffer, int read_length)
{
    //int64_t t_i2c_read_start = get_sdsp_time_realtime_ns();
    if (check_i2c_bus_num(i2c_bus,__func__))
        return -1;

    if(check_i2c_bus_init_status(i2c_bus,__func__))
        return -1;

    int reg_addr_len_bytes = 1;
    switch (register_address_num_bits){
        case 8: case 16: case 24: case 32:
            reg_addr_len_bytes = register_address_num_bits / 8;
            break;
        default:
            PRINT("ERROR in io_rpc_i2c_read: provided invalid number of bits in register address: %d (acceptable values are 8, 16, 24, 32)",register_address_num_bits);
            return -1;
    }

    // we will first write the register address to read from then read the data (in one transaction)
    struct dspal_i2c_ioctl_combined_write_read ioctl_write_read;

    ioctl_write_read.write_buf     = (uint8_t*)(&register_address);
    ioctl_write_read.write_buf_len = reg_addr_len_bytes;
    ioctl_write_read.read_buf      = read_buffer;
    ioctl_write_read.read_buf_len  = read_length;

    int bytes_read = ioctl(i2c_fds[i2c_bus], I2C_IOCTL_RDWR, &ioctl_write_read);
    if (bytes_read != read_length)
    {
        PRINT("ERROR in io_rpc_i2c_read: read %d bytes, but requested %d bytes",bytes_read,read_length);
        return -1;
    }

    //int64_t t_i2c_read_end = get_sdsp_time_realtime_ns();
    //PRINT("I2C read time = %dus",(int32_t)(t_i2c_read_end-t_i2c_read_start)/1000);

    return 0;
}

int io_rpc_i2c_write(uint32_t i2c_bus, uint32_t register_address, uint32_t register_address_num_bits, const uint8_t * write_buffer, int write_length)
{
    //int64_t t_i2c_write_start = get_sdsp_time_realtime_ns();

    if (check_i2c_bus_num(i2c_bus,__func__))
        return -1;

    if(check_i2c_bus_init_status(i2c_bus,__func__))
        return -1;

    int reg_addr_len_bytes = 1;
    switch (register_address_num_bits){
        case 8: case 16: case 24: case 32:
            reg_addr_len_bytes = register_address_num_bits / 8;
            break;
        default:
            PRINT("ERROR in io_rpc_i2c_write: provided invalid number of bits in register address: %d (acceptable values are 8, 16, 24, 32)",register_address_num_bits);
            return -1;
    }

    int write_length_full = write_length + reg_addr_len_bytes;

    if (write_length_full > MAX_I2C_TX_BUFFER_SIZE)
    {
        PRINT("ERROR in io_rpc_i2c_write : Caller's buffer size (%d) exceeds size of local buffer (%d)",write_length_full,MAX_I2C_TX_BUFFER_SIZE);
        return -1;
    }

    uint8_t write_buffer_full[MAX_I2C_TX_BUFFER_SIZE];

    memcpy(write_buffer_full,&register_address,reg_addr_len_bytes);
    memcpy(&write_buffer_full[reg_addr_len_bytes], write_buffer, write_length);
    int bytes_written = write(i2c_fds[i2c_bus], (char *)write_buffer_full, write_length_full);
    if (bytes_written != write_length_full)
    {
        PRINT("ERROR in io_rpc_i2c_write: wrote %d bytes, but requested %d bytes (including target start register)",bytes_written,write_length_full);
        return -1;
    }

    //int64_t t_i2c_write_end = get_sdsp_time_realtime_ns();
    //PRINT("I2C write time = %dus",(int32_t)(t_i2c_write_end-t_i2c_write_start)/1000);

    return 0;
}

int io_rpc_i2c_close(uint32_t i2c_bus)
{
    if (check_i2c_bus_num(i2c_bus,__func__))
        return -1;

    // if not initialized already, return
    if(i2c_fds[i2c_bus]==0) return 0;

    close(i2c_fds[i2c_bus]);
    i2c_fds[i2c_bus]=0;
    return 0;
}
