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

int str2int32(char * str, int * result_out);
int str2uint8(char * str, uint8_t * result_out);

static void print_usage(const char * prog_name)
{
    printf("\nDescription:\n");
    printf("\tCommand line tool for using i2c functionality via SDSP\n");
    printf("\nUsage:\n");
    printf("  %s scan <i2c_bus> <bit_rate> [register_address=0]\n",prog_name);
    printf("\tScan all i2c addresses (0-126)\n");
    printf("\tPrint the value of specified register for detected devices\n");
    printf("\n");
    printf("  %s read <i2c_bus> <bit_rate> <i2c_address> <start_register> <read_size>\n",prog_name);
    printf("\tRead data from i2c slave starting from start_register and print their values\n");
    printf("\n");
    printf("  %s write <i2c_bus> <bit_rate> <i2c_address> <start_register> <data>\n",prog_name);
    printf("\tWrite data to i2c slave starting from start_register\n");
    printf("\tData should be provided as space-delimited array of uint8's\n");
    printf("\n");
}

int scan_i2c_bus(uint32_t i2c_bus, uint32_t bit_rate, uint32_t reg_to_read)
{
    printf("Scanning I2C Bus %d, bit rate %d, read address %d (0x%02X)\n",i2c_bus,bit_rate,reg_to_read,reg_to_read);

    if (voxl_i2c_init(i2c_bus)){
        fprintf(stderr, "ERROR initializing i2c bus %d\n",i2c_bus);
        return -1;
    }

    // allocate shared memory buffer for reading data
    uint32_t read_data_buffer_size = 1;
    uint8_t* read_data_buffer = voxl_rpc_shared_mem_alloc(read_data_buffer_size);
    if(read_data_buffer==NULL){
        fprintf(stderr, "failed to allocate shared rpc memory\n");
        return -1;
    }

    uint32_t test_addr;
    uint32_t test_register_addr = reg_to_read;
    uint32_t i2c_bit_rate_hz    = bit_rate;
    uint32_t i2c_timeout_us     = 1000;
    uint32_t found_devices      = 0;

    for (test_addr=1; test_addr<127; test_addr++){
        printf("Testing I2C slave address %d...\n",test_addr);
        if (voxl_i2c_slave_config(i2c_bus, test_addr, i2c_bit_rate_hz, i2c_timeout_us)){
            fprintf(stderr, "failed to set i2c slave config on bus %d, address %d, bit rate %d, timeout %dus\n",i2c_bus, test_addr,i2c_bit_rate_hz,i2c_timeout_us);
            break;
        }

        if (voxl_i2c_read(i2c_bus, test_register_addr, 8, read_data_buffer, 1) == 0){
            printf("Found I2C device with slave address %d (0x%02X). Register %d (0x%02X) is %d (0x%02X)\n",
            test_addr,test_addr,reg_to_read,reg_to_read,read_data_buffer[0],read_data_buffer[0]);
            found_devices++;
        }
    }

    printf("Found %d devices\n",found_devices);

    voxl_i2c_close(i2c_bus);

    // cleanup shared memory
    voxl_rpc_shared_mem_free(read_data_buffer);
    voxl_rpc_shared_mem_deinit();

    return 0;
}

int read_registers(uint32_t i2c_bus, uint32_t i2c_bit_rate, uint32_t i2c_address, uint32_t start_register, uint32_t read_size)
{
    //int64_t t_init_start = voxl_apps_time_realtime_ns();
    if (voxl_i2c_init(i2c_bus)){
        fprintf(stderr, "ERROR initializing i2c bus %d\n",i2c_bus);
        return -1;
    }

    // allocate shared memory buffer for reading data
    //int64_t t_mem_alloc_start = voxl_apps_time_realtime_ns();
    uint32_t read_data_buffer_size = read_size;
    uint8_t* read_data_buffer = voxl_rpc_shared_mem_alloc(read_data_buffer_size);
    if(read_data_buffer==NULL){
        fprintf(stderr, "failed to allocate shared rpc memory\n");
        return -1;
    }

    uint32_t i2c_timeout_us     = 1000;

    //int64_t t_slave_config_start = voxl_apps_time_realtime_ns();
    if (voxl_i2c_slave_config(i2c_bus, i2c_address, i2c_bit_rate, i2c_timeout_us)){
        fprintf(stderr, "failed to set i2c slave config on bus %d, address %d, bit rate %d, timeout %dus\n",i2c_bus, i2c_address,i2c_bit_rate,i2c_timeout_us);
        return -1;
    }

    //int64_t t_read_start = voxl_apps_time_realtime_ns();
    if (voxl_i2c_read(i2c_bus, start_register, 8, read_data_buffer, read_size)){
        fprintf(stderr, "failed to read %d bytes from i2c_address %d @ %d\n",read_size, i2c_address, start_register);
        return -1;
    }
    else {
        uint32_t ii;
        for (ii=0; ii<read_size; ii++){
            //printf("%d(0x%02X) ",read_data_buffer[ii],read_data_buffer[ii]);
            printf("%d ",read_data_buffer[ii]);
        }
        printf("\n");
    }

    //int64_t t_cleanup_start = voxl_apps_time_realtime_ns();
    voxl_i2c_close(i2c_bus);

    // cleanup shared memory
    voxl_rpc_shared_mem_free(read_data_buffer);
    voxl_rpc_shared_mem_deinit();
/*
    int64_t t_cleanup_end = voxl_apps_time_realtime_ns();

    printf("I2C Init time       = %dus\n",(int32_t)(t_mem_alloc_start - t_init_start)/1000);
    printf("I2C mem alloc time  = %dus\n",(int32_t)(t_slave_config_start - t_mem_alloc_start)/1000);
    printf("I2C slave conf time = %dus\n",(int32_t)(t_read_start - t_slave_config_start)/1000);
    printf("I2C read time       = %dus\n",(int32_t)(t_cleanup_start - t_read_start)/1000);
    printf("I2C cleanup time    = %dus\n",(int32_t)(t_cleanup_end - t_cleanup_start)/1000);
*/
    return 0;
}

int write_registers(uint32_t i2c_bus, uint32_t i2c_bit_rate, uint32_t i2c_address, uint32_t start_register, uint8_t * write_data, uint32_t write_size)
{
    if (voxl_i2c_init(i2c_bus)){
        fprintf(stderr, "ERROR initializing i2c bus %d\n",i2c_bus);
        return -1;
    }

    uint32_t i2c_timeout_us     = 1000;

    if (voxl_i2c_slave_config(i2c_bus, i2c_address, i2c_bit_rate, i2c_timeout_us)){
        fprintf(stderr, "failed to set i2c slave config on bus %d, address %d, bit rate %d, timeout %dus\n",i2c_bus, i2c_address,i2c_bit_rate,i2c_timeout_us);
        return -1;
    }

    if (voxl_i2c_write(i2c_bus, start_register, 8, write_data, write_size)){
        fprintf(stderr, "failed to write %d bytes to i2c_address %d @ %d\n",write_size, i2c_address, start_register);
        return -1;
    }

    voxl_i2c_close(i2c_bus);
/*
    printf("wrote data: ");
    uint32_t ii;
    for (ii=0; ii<write_size; ii++)
        printf("%d ",write_data[ii]);
    printf("\n");
*/
    return 0;
}

int main(int argc, char** argv)
{
    if (argc < 4){
        print_usage(argv[0]);
        return -1;
    }

    int32_t i2c_bus      = 0;
    int32_t i2c_bit_rate = 0;

    // parse i2c bus number
    if ( str2int32(argv[2],&i2c_bus) ){
        fprintf(stderr, "ERROR parsing i2c bus number : %s\n",argv[2]);
        print_usage(argv[0]);
        return -1;
    }

    // parse and check bit rate
    if ( (str2int32(argv[3],&i2c_bit_rate)) || ((i2c_bit_rate != 100000) && (i2c_bit_rate != 400000) && (i2c_bit_rate != 1000000))){
        fprintf(stderr, "ERROR parsing i2c bit rate. The supported bit rates are : 100000, 400000, 1000000. Provided : %s\n",argv[3]);
        print_usage(argv[0]);
        return -1;
    }

    if (strcmp(argv[1],"scan")==0)
    {
        int32_t reg_to_read = 0;
        if (argc == 5){
            if ( (str2int32(argv[4],&reg_to_read)) || (reg_to_read < 0) || (reg_to_read>255)){
                fprintf(stderr, "ERROR parsing reg_to_read : %s. Please check format and range\n",argv[4]);
                return -1;
            }
        }

        return scan_i2c_bus(i2c_bus, i2c_bit_rate, reg_to_read);
    }

    if (argc < 6){
        print_usage(argv[0]);
        return -1;
    }

    int32_t i2c_address = 0;
    if ( (str2int32(argv[4],&i2c_address)) || (i2c_address < 1) || (i2c_address>126)){
        fprintf(stderr, "ERROR parsing i2c_address : %s. Please check format and range\n",argv[4]);
        return -1;
    }

    int32_t start_register = 0;
    if ( (str2int32(argv[5],&start_register)) || (start_register<0) || (start_register>255)){
        fprintf(stderr, "ERROR parsing start_register from string: %s. Please check format and range\n",argv[5]);
        return -1;
    }

    else if (strcmp(argv[1],"read")==0)
    {
        if (argc != 7){
            fprintf(stderr, "ERROR. Please check number of input arguments for read cmd\n");
            print_usage(argv[0]);
            return -1;
        }

        int32_t read_size = 0;
        if ( (str2int32(argv[6],&read_size)) || (read_size < 1) || (read_size>127)){
            fprintf(stderr, "ERROR parsing read_size : %s. Please check format and range\n",argv[6]);
            return -1;
        }

        return read_registers(i2c_bus, i2c_bit_rate, i2c_address, start_register, read_size);
    }

    else if (strcmp(argv[1],"write")==0)
    {
        if (argc < 7){
            fprintf(stderr, "ERROR. Please check number of input arguments for write cmd\n");
            print_usage(argv[0]);
            return -1;
        }

        int32_t write_size = argc-6;
        uint8_t data[write_size];

        int ii;
        for (ii=0; ii<write_size; ii++){
            if ( str2uint8(argv[6+ii],&data[ii]) ){
                fprintf(stderr, "ERROR parsing uint8 data for writing : %s. Please check format and range\n",argv[6+ii]);
                return -1;
            }
        }

        return write_registers(i2c_bus, i2c_bit_rate, i2c_address, start_register, data, write_size);
    }

    print_usage(argv[0]);
    return -1;
}


int str2int32(char * str, int * result_out)
{
    char * end_ptr;
    int val = (int)strtol(str, &end_ptr, 0);
    if (end_ptr == str)
        return -1;

    *result_out = val;
    return 0;
}

int str2uint8(char * str, uint8_t * result_out)
{
    int32_t val;
    if ( str2int32(str,&val) )
        return -1;

    if ((val<0) || (val>255))
        return -2;

    *result_out = (uint8_t)val;
    return 0;
}
