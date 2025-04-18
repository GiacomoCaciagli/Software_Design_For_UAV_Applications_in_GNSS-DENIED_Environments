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


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <voxl_io.h>
#include <math.h>

#define MPU6050_I2C_SLAVE_ADDRESS 0b1101000

static const uint32_t read_data_buffer_size = 16;
static uint8_t * read_data_buffer = NULL;

static void print_usage(const char * prog_name)
{
    printf("Usage:\n");
    printf("%s <i2c_bus_number>\n",prog_name);
}

int mpu6050_detect(uint32_t i2c_bus)
{
    //read WHOAMI register
    uint8_t test_register_addr = 117;
    if (voxl_i2c_read(i2c_bus, test_register_addr, 8, read_data_buffer, 1)){
        fprintf(stderr, "ERROR reading MPU6050 WHOAMI register\n");
        return -1;
    }

    printf("read WHOAMI register : 0x%02X\n",read_data_buffer[0]);

    if (read_data_buffer[0] == 0x68){
        printf("MPU6050 detected successfully!\n");
        return 0;
    }

    printf("WHOAMI register does not match (expected 0x68)\n");
    return -1;
}

int mpu6050_write_reg(uint32_t i2c_bus, uint8_t register_address, uint8_t value)
{
    return voxl_i2c_write(i2c_bus, register_address, 8, &value, 1);
}

int mpu6050_initialize(uint32_t i2c_bus)
{
    if (mpu6050_write_reg(i2c_bus, 107, 0))         return 107;  //set to run mode
    usleep(150000);

    if (mpu6050_write_reg(i2c_bus, 26, 0b00000000)) return 26;   //set the gyro sampling rate
    if (mpu6050_write_reg(i2c_bus, 27, 0b00011000)) return 27;   //set the gyro sensitivity to 2000
    if (mpu6050_write_reg(i2c_bus, 28 ,0b00011000)) return 28;   //set the accel sensitivity to 16g

    return 0;
}

int mpu6050_read_data(int i2c_bus)
{
    uint8_t data_register_address = 59;
    if (voxl_i2c_read(i2c_bus, data_register_address, 8, read_data_buffer, 14)){
        fprintf(stderr, "ERROR reading mpu6050 data\n");
        return -1;
    }

    int16_t tr,axr,ayr,azr,wxr,wyr,wzr;
    uint8_t * data = read_data_buffer;

    //parse the data..
    //raw values
    axr = data[0];  axr <<= 8;  axr |=data[1];
    ayr = data[2];  ayr <<= 8;  ayr |=data[3];
    azr = data[4];  azr <<= 8;  azr |=data[5];
    tr  = data[6];  tr  <<= 8;  tr  |=data[7];
    wxr = data[8];  wxr <<= 8;  wxr |=data[9];
    wyr = data[10]; wyr <<= 8;  wyr |=data[11];
    wzr = data[12]; wzr <<= 8;  wzr |=data[13];

    //convert to real units
    const float gyro_scale  = (2000.0 / 180 * M_PI) / 32768.0; //2000 degrees max range
    const float accel_scale = 16.0 / 32768.0;                  //16G max range

    float axf = axr * accel_scale;
    float ayf = ayr * accel_scale;
    float azf = azr * accel_scale;
    float wxf = wxr * gyro_scale;
    float wyf = wyr * gyro_scale;
    float wzf = wzr * gyro_scale;
    float tempf = 35.0 + (tr + 521) / 340.0;

    //printf("%+5d  %+5d %+5d %+5d   %+5d %+5d %+5d\n",tr,axr,ayr,azr,wxr,wyr,wzr);
    printf("%+4.2f  %+4.2f %+4.2f %+4.2f   %+4.2f %+4.2f %+4.2f\n",tempf,axf,ayf,azf,wxf,wyf,wzf);

    return 0;
}


int main(int argc, char** argv)
{
    if (argc < 2){
        print_usage(argv[0]);
        return -1;
    }

    int i2c_bus = atoi(argv[1]);

    // check bus number because atoi returns 0 if it can't parse an int
    if ((i2c_bus == 0) && (argv[1][0] != '0')){
        fprintf(stderr, "ERROR parsing i2c bus number : %s\n",argv[1]);
        print_usage(argv[0]);
        return -1;
    }

    if (voxl_i2c_init(i2c_bus)){
        fprintf(stderr, "ERROR initializing i2c bus %d\n",i2c_bus);
        return -1;
    }

    int i2c_bit_rate_hz    = 400000;
    int i2c_timeout_us     = 1000;
    if (voxl_i2c_slave_config(i2c_bus, MPU6050_I2C_SLAVE_ADDRESS, i2c_bit_rate_hz, i2c_timeout_us)){
        fprintf(stderr, "failed to set i2c slave config on bus %d, address %d, bit rate %d, timeout %dus\n",
                i2c_bus, MPU6050_I2C_SLAVE_ADDRESS,i2c_bit_rate_hz,i2c_timeout_us);
        return -1;
    }

    // allocate shared memory buffer for reading data
    read_data_buffer = voxl_rpc_shared_mem_alloc(read_data_buffer_size);
    if(read_data_buffer==NULL){
        fprintf(stderr, "failed to allocate shared rpc memory\n");
        return -1;
    }

    int ret;

    printf("Detecting MPU6050\n");
    if (mpu6050_detect(i2c_bus))
        return -1;

    if ( (ret=mpu6050_initialize(i2c_bus)) ){
        fprintf(stderr, "failed to initialize mpu6050 (setting register %d)\n",ret);
        return -1;
    }

    int ii;
    for (ii=0; ii<10000; ii++){
        mpu6050_read_data(i2c_bus);
        usleep(10000);
    }

    voxl_i2c_close(i2c_bus);

    // cleanup shared memory
    voxl_rpc_shared_mem_free(read_data_buffer);
    voxl_rpc_shared_mem_deinit();

    return 0;
}
