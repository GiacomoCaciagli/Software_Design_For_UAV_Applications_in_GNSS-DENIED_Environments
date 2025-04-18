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

#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <HAP_farf.h>
#include <sys/ioctl.h>
#include <io_rpc.h>
#include <dev_fs_lib_spi.h>
#include <sys/ioctl.h>
#include <stdlib.h> // for malloc

#define PRINT(...)  FARF(ALWAYS, __VA_ARGS__);
#define MAX_BUS     16
#define VOSPI_PACKET_SIZE     164

// file descriptors for all ports
static int spi_fd[MAX_BUS+1];

static int spi_skip_bit_fd[MAX_BUS+1] = { 0 };
// read buffer for spi allocated on heap at init
static uint8_t* read_buf[MAX_BUS+1];

int io_rpc_spi_init(int32 bus, int32 bus_mode, int32 freq_hz)
{
    int tmpfd, ret;
    char buf[16];

    // sanity checks
    if(bus<0 || bus>MAX_BUS){
        PRINT("ERROR in io_rpc_spi_init, bus must be between 0 & %d", MAX_BUS);
        return -1;
    }
    if(spi_fd[bus]){
        PRINT("WARNING in io_rpc_spi_init, trying to initialize bus %d when already initialized", bus);
        return 0;
    }

    // open file descriptor
    snprintf(buf,sizeof(buf),"/dev/spi-%d", (int)bus);
    tmpfd = open(buf, O_RDWR);
    if(tmpfd==-1){
        PRINT("ERROR in io_rpc_spi_init while opening file descriptor %s", buf);
        PRINT("open() returned %d", tmpfd);
        PRINT("errno=%d",errno);
        return -1;
    }

    // set speed
    struct dspal_spi_ioctl_set_bus_frequency bus_freq;
    bus_freq.bus_frequency_in_hz = freq_hz;
    ret = ioctl(tmpfd, SPI_IOCTL_SET_BUS_FREQUENCY_IN_HZ, &bus_freq);
    if(ret){
        PRINT("ERROR in io_rpc_spi_init while setting bitrate");
        PRINT("ioctl returned %d", ret);
        PRINT("errno=%d",errno);
        return -1;
    }

    // set mode
    struct dspal_spi_ioctl_set_spi_mode mode;
    switch(bus_mode){
        case 0:
            mode.eClockPolarity = SPI_CLOCK_IDLE_LOW;
            mode.eShiftMode = SPI_INPUT_FIRST;
            break;
        case 1:
            mode.eClockPolarity = SPI_CLOCK_IDLE_LOW;
            mode.eShiftMode = SPI_OUTPUT_FIRST;
            break;
        case 2:
            mode.eClockPolarity = SPI_CLOCK_IDLE_HIGH;
            mode.eShiftMode = SPI_INPUT_FIRST;
            break;
        case 3:
            mode.eClockPolarity = SPI_CLOCK_IDLE_HIGH;
            mode.eShiftMode = SPI_OUTPUT_FIRST;
            break;
        default:
            PRINT("ERROR in io_rpc_spi_init, invalid bus mode");
            return -1;
    }
    ret = ioctl(tmpfd, SPI_IOCTL_SET_SPI_MODE, &mode);
    if(ret){
        PRINT("ERROR in io_rpc_spi_init while setting bus mode");
        PRINT("ioctl returned %d", ret);
        PRINT("errno=%d",errno);
        return -1;
    }

    read_buf[bus] = malloc(DSPAL_SPI_RECEIVE_BUFFER_LENGTH);
    if(read_buf[bus]==NULL){
        PRINT("ERROR in io_rpc_spi_init, failed to malloc read buf");
        return -1;
    }

    spi_fd[bus]=tmpfd;
    return 0;
}

int io_rpc_spi_set_freq(int32 bus, int32 freq_hz)
{
    // sanity checks
    if(bus<0 || bus>MAX_BUS){
        PRINT("ERROR in io_rpc_spi_set_freq, bus must be between 0 & %d", MAX_BUS);
        return -1;
    }
    if(spi_fd[bus]==0){
        PRINT("ERROR in io_rpc_spi_set_freq, bus %d must be initialized first", bus);
        return -1;
    }
    // set speed
    struct dspal_spi_ioctl_set_bus_frequency bus_freq;
    bus_freq.bus_frequency_in_hz = freq_hz;
    int ret = ioctl(spi_fd[bus], SPI_IOCTL_SET_BUS_FREQUENCY_IN_HZ, &bus_freq);
    if(ret){
        PRINT("ERROR in io_rpc_spi_set_freq");
        PRINT("ioctl returned %d", ret);
        PRINT("errno=%d",errno);
        return -1;
    }
    return 0;
}


int io_rpc_spi_skip_dummy_bit(int32 bus, int32 val){

    if(bus<0 || bus>MAX_BUS){
        PRINT("ERROR in skip_dummy_bit, bus must be between 0 & %d", MAX_BUS);
        return -1;
    }
    if (val){
        spi_skip_bit_fd[bus] = 1;
    } else {
        spi_skip_bit_fd[bus] = 0;
    }
    return 0;
}


int io_rpc_spi_close(int32 bus)
{
    // sanity checks
    if(bus<0 || bus>MAX_BUS){
        PRINT("ERROR in io_rpc_spi_close, bus must be between 0 & %d", MAX_BUS);
        return -1;
    }
    // if not initialized already, return
    if(spi_fd[bus]==0) return 0;
    // close
    close(spi_fd[bus]);
    spi_fd[bus]=0;
    // free read buf memory
    if(read_buf[bus]!=NULL) free(read_buf[bus]);
    return 0;
}


int io_rpc_spi_write(int32 bus, const unsigned char* data, int dataLen)
{
    // sanity checks
    if(bus<0 || bus>MAX_BUS){
        PRINT("ERROR in io_rpc_spi_write, bus must be between 0 & %d", MAX_BUS);
        return -1;
    }
    if(dataLen<1){
        PRINT("ERROR in io_rpc_spi_write, number of bytes to send must be >1");
        return -1;
    }
    if(spi_fd[bus]==0){
        PRINT("ERROR in io_rpc_spi_write, bus %d must be initialized first", bus);
        return -1;
    }
    int ret = write(spi_fd[bus], data, dataLen);
    if(ret!=dataLen){
        PRINT("ERROR in io_rpc_spi_write, write() returned %d", ret);
        PRINT("errno=%d",errno);
        return -1;
    }
    return 0;
}

int io_rpc_spi_write_cs(int32 bus, const unsigned char* data, int dataLen, int32 cs_gpio)
{
    int ret;

    if (io_rpc_gpio_is_initialized(cs_gpio) != 1)  //make sure the chip select pin has been initialized to gpio
        return -1;

    io_rpc_gpio_write(cs_gpio,0);  //drive pin low to enable the slave device
    ret = io_rpc_spi_write(bus,data,dataLen);
    io_rpc_gpio_write(cs_gpio,1);  //drive pin high to disable the slave device

    return ret;
}


int io_rpc_spi_transfer(int32 bus, const unsigned char* write_buf, int writeLen, unsigned char* read_buf, int readLen)
{
    // sanity checks
    if(bus<0 || bus>MAX_BUS){
        PRINT("ERROR in %s, bus must be between 0 & %d", __FUNCTION__, MAX_BUS);
        return -1;
    }
    if(writeLen<1 || writeLen>(DSPAL_SPI_TRANSMIT_BUFFER_LENGTH-1)){
        PRINT("ERROR in %s, number of bytes to write must be <=%d", __FUNCTION__, DSPAL_SPI_TRANSMIT_BUFFER_LENGTH-1);
        return -1;
    }
    if(readLen<1 || readLen>(DSPAL_SPI_RECEIVE_BUFFER_LENGTH-1)){
        PRINT("ERROR in %s, number of bytes to read must be <=%d", __FUNCTION__, DSPAL_SPI_RECEIVE_BUFFER_LENGTH-1);
        return -1;
    }
    if(spi_fd[bus]==0){
        PRINT("ERROR in %s, bus %d must be initialized first", __FUNCTION__, bus);
        return -1;
    }

    struct dspal_spi_ioctl_read_write read_write;
    uint8_t rx_buf[readLen+1];

    read_write.read_buffer         = rx_buf;
    read_write.read_buffer_length  = readLen+1; // extra byte for dummy byte
    read_write.write_buffer        = (void*)write_buf;
    read_write.write_buffer_length = writeLen;

    int ret = ioctl(spi_fd[bus], SPI_IOCTL_RDWR, &read_write);
    if(ret < 0){
        PRINT("ERROR in %s, ioctl returned %d", __FUNCTION__, ret);
        return -1;
    }

    // skip the first (dummy) byte and read out the rest
    memcpy(read_buf, &(rx_buf[1]), readLen);

  return 0;
}


int io_rpc_spi_transfer_cs(int32 bus, const unsigned char* write_buf, int writeLen, unsigned char* read_buf, int readLen, int32 cs_gpio)
{
    int ret;

    if (io_rpc_gpio_is_initialized(cs_gpio) != 1)  //make sure the chip select pin has been initialized to gpio
        return -1;

    io_rpc_gpio_write(cs_gpio,0);  //drive pin low to enable the slave device
    ret = io_rpc_spi_transfer(bus,write_buf,writeLen,read_buf,readLen);
    io_rpc_gpio_write(cs_gpio,1);  //drive pin high to disable the slave device

    return ret;
}


int io_rpc_spi_write_reg_byte(int32 bus, uint8 address, uint8 val)
{
  uint8_t buf[2] = {address,val};

  if(io_rpc_spi_write(bus,buf,2)){
    PRINT("ERROR in io_rpc_spi_write_reg_byte");
    return -1;
  }
  return 0;
}

int io_rpc_spi_write_reg_byte_cs(int32 bus, uint8 address, uint8 val, int32 cs_gpio)
{
  uint8_t buf[2] = {address,val};

  if(io_rpc_spi_write_cs(bus,buf,2,cs_gpio)){
    PRINT("ERROR in io_rpc_spi_write_reg_byte_cs");
    return -1;
  }
  return 0;
}


int io_rpc_spi_write_reg_word(int32 bus, uint8 address, uint16 val)
{
  uint8_t buf[3] = {address,((val|0xFF00)>>8), (val|0xFF)};

  if(io_rpc_spi_write(bus,buf,3)){
    PRINT("ERROR in io_rpc_spi_write_reg_word");
    return -1;
  }
  return 0;
}

int io_rpc_spi_write_reg_word_cs(int32 bus, uint8 address, uint16 val, int32 cs_gpio)
{
  uint8_t buf[3] = {address,((val|0xFF00)>>8), (val|0xFF)};

  if(io_rpc_spi_write_cs(bus,buf,3,cs_gpio)){
    PRINT("ERROR in io_rpc_spi_write_reg_word_cs");
    return -1;
  }
  return 0;
}


int io_rpc_spi_read_reg(int32 bus, uint8 address, unsigned char* data, int dataLen)
{
    // sanity checks
    if(bus<0 || bus>MAX_BUS){
        PRINT("ERROR in io_rpc_spi_read_reg, bus must be between 0 & %d", MAX_BUS);
        return -1;
    }
    if(dataLen<1){
        PRINT("ERROR in io_rpc_spi_read_reg, number of bytes to read must be >=1");
        return -1;
    }
    if(dataLen>(DSPAL_SPI_RECEIVE_BUFFER_LENGTH-1)){
        PRINT("ERROR in io_rpc_spi_read_reg, number of bytes to read must be <=%d", DSPAL_SPI_RECEIVE_BUFFER_LENGTH-1);
        return -1;
    }
    if(spi_fd[bus]==0){
        PRINT("ERROR in io_rpc_spi_read_reg, bus %d must be initialized first", bus);
        return -1;
    }

    // Change spi_skip_bit_fd value if reading from source which requires an address specified
    int transfer_bytes;
    if (spi_skip_bit_fd[bus] == 0){
        transfer_bytes = dataLen + 1; // first byte is address
    } else {
        transfer_bytes = dataLen; // first byte is address
    }

    struct dspal_spi_ioctl_read_write read_write;
    uint8_t tx_buf[1]; // one byte for address
    uint8_t rx_buf[transfer_bytes];

    if (spi_skip_bit_fd[bus] == 0){
        tx_buf[0] = address | 0x80; // register high bit=1 for read
    } else {
        tx_buf[0] = 0;
    }

    read_write.read_buffer         = rx_buf;
    read_write.read_buffer_length  = transfer_bytes;
    read_write.write_buffer        = tx_buf;

    if (spi_skip_bit_fd[bus] == 0){
        read_write.write_buffer_length = 1;
    } else {
        read_write.write_buffer_length = 0;
    }

    int ret = ioctl(spi_fd[bus], SPI_IOCTL_RDWR, &read_write);
    if(ret != transfer_bytes){
        PRINT("ERROR in io_rpc_spi_read_reg, ioctl returned %d", ret);
        return -1;
    }

    if (spi_skip_bit_fd[bus] == 0){
        memcpy(data, &(rx_buf[1]), dataLen); // skip the first (dummy) byte and read out the rest
    } else {
        memcpy(data, &(rx_buf[0]), dataLen);
    }

  return 0;
}

int io_rpc_spi_read_reg_cs(int32 bus, uint8 address, unsigned char* data, int dataLen, int32 cs_gpio)
{
    int ret;

    if (io_rpc_gpio_is_initialized(cs_gpio) != 1)  //make sure the chip select pin has been initialized to gpio
        return -1;

    io_rpc_gpio_write(cs_gpio,0);  //drive pin low to enable the slave device
    ret = io_rpc_spi_read_reg(bus, address, data, dataLen);
    io_rpc_gpio_write(cs_gpio,1);  //drive pin high to disable the slave device

    return ret;
}


int io_rpc_spi_read_reg_byte(int32 bus, uint8 address, uint8* data)
{
    // sanity checks
    if(bus<0 || bus>MAX_BUS){
        PRINT("ERROR in io_rpc_spi_read_reg_byte, bus must be between 0 & %d", MAX_BUS);
        return -1;
    }
    if(spi_fd[bus]==0){
        PRINT("ERROR in io_rpc_spi_read_reg_byte, bus %d must be initialized first", bus);
        return -1;
    }

    int transfer_bytes = 2; // first byte is address
    struct dspal_spi_ioctl_read_write read_write;
    uint8_t tx_buf[1]; // one byte for address
    uint8_t rx_buf[transfer_bytes];

    tx_buf[0] = address | 0x80; // register high bit=1 for read

    read_write.read_buffer         = rx_buf;
    read_write.read_buffer_length  = transfer_bytes;
    read_write.write_buffer        = tx_buf;
    read_write.write_buffer_length = 1;

    int ret = ioctl(spi_fd[bus], SPI_IOCTL_RDWR, &read_write);

    if(ret != transfer_bytes){
        PRINT("ERROR in io_rpc_spi_read_reg_byte, ioctl returned %d", ret);
        return -1;
    }

    // skip the first (dummy) byte and read out the rest
    *data = rx_buf[1];
    return 0;
}


int io_rpc_spi_read_reg_byte_cs(int32 bus, uint8 address, uint8* data, int32 cs_gpio)
{
    int ret;

    if (io_rpc_gpio_is_initialized(cs_gpio) != 1)  //make sure the chip select pin has been initialized to gpio
        return -1;

    io_rpc_gpio_write(cs_gpio,0);  //drive pin low to enable the slave device
    ret = io_rpc_spi_read_reg_byte(bus, address, data);
    io_rpc_gpio_write(cs_gpio,1);  //drive pin high to disable the slave device

    return ret;
}


int io_rpc_spi_read_reg_word(int32 bus, uint8 address, uint16* data)
{
    // sanity checks
    if(bus<0 || bus>MAX_BUS){
        PRINT("ERROR in io_rpc_spi_read_reg_word, bus must be between 0 & %d", MAX_BUS);
        return -1;
    }
    if(spi_fd[bus]==0){
        PRINT("ERROR in io_rpc_spi_read_reg_word, bus %d must be initialized first", bus);
        return -1;
    }

    int transfer_bytes = 3; // first byte is address
    struct dspal_spi_ioctl_read_write read_write;
    uint8_t tx_buf[1]; // one byte for address
    uint8_t rx_buf[transfer_bytes];

    tx_buf[0] = address | 0x80; // register high bit=1 for read

    read_write.read_buffer         = rx_buf;
    read_write.read_buffer_length  = transfer_bytes;
    read_write.write_buffer        = tx_buf;
    read_write.write_buffer_length = 1;

    int ret = ioctl(spi_fd[bus], SPI_IOCTL_RDWR, &read_write);

    if(ret != transfer_bytes){
        PRINT("ERROR in io_rpc_spi_read_reg_word, ioctl returned %d", ret);
        return -1;
    }

    // skip the first (dummy) byte and read out the rest
    *data = rx_buf[1]<<8 | rx_buf[2];
    return 0;
}

int io_rpc_spi_read_reg_word_cs(int32 bus, uint8 address, uint16* data, int32 cs_gpio)
{
    int ret;

    if (io_rpc_gpio_is_initialized(cs_gpio) != 1)  //make sure the chip select pin has been initialized to gpio
        return -1;

    io_rpc_gpio_write(cs_gpio,0);  //drive pin low to enable the slave device
    ret = io_rpc_spi_read_reg_word(bus, address, data);
    io_rpc_gpio_write(cs_gpio,1);  //drive pin high to disable the slave device

    return ret;
}

int io_rpc_spi_read_imu_fifo(int32 bus, uint8 count_addr, uint8 fifo_addr,\
                        uint8 packet_size, int32 min_packets, int32* packets_read,\
                        unsigned char* data, int dataLen, int32 count_speed, int32 data_speed)
{
    int ret;
    *packets_read = 0; // zero out packets read until we actually read
    uint16_t fifo_count = 0;
    int32_t p_available = -1;

    // sanity checks
    if(bus<0 || bus>MAX_BUS){
        PRINT("ERROR in io_rpc_spi_read_imu_fifo, bus must be between 0 & %d", MAX_BUS);
        return -1;
    }
    if(spi_fd[bus]==0){
        PRINT("ERROR in io_rpc_spi_read_imu_fifo, bus %d must be initialized first", bus);
        return -1;
    }
    if(read_buf[bus]==NULL){
        PRINT("ERROR in io_rpc_spi_read_imu_fifo, read buffer not initialized yet", bus);
        return -1;
    }

    // record current time for timeout check later
    int64_t t_start, t_current;
    io_rpc_sdsp_time_monotonic_ns(&t_start);

    // set speed for reading count address
    io_rpc_spi_set_freq(bus, count_speed);

    // first read how many bytes are available using the fifo_count register
    // keep looping until we have enough data to read
    while(p_available < min_packets){
        ret = io_rpc_spi_read_reg_word(bus, count_addr, &fifo_count);
        if(ret){
            PRINT("ERROR in io_rpc_spi_read_imu_fifo, failed to read count address %d", count_addr);
            return -1;
        }

        if(fifo_count>dataLen){
            PRINT("WARNING in io_rpc_spi_read_imu_fifo, impossibly large fifo_count:%d", fifo_count);
            PRINT("trying again");
            io_rpc_spi_read_reg_word(bus, count_addr, &fifo_count);
            if(fifo_count>dataLen) return -1;
        }

        // packets available to read
        // this rounds down and warns if a partial packet is in fifo
        p_available = fifo_count/packet_size;

        // if nothing to read, check timeout and wait until there is
        if(p_available < min_packets){
            io_rpc_sdsp_time_monotonic_ns(&t_current);
            if((t_current-t_start)>500000000){
                PRINT("ERROR in io_rpc_spi_read_imu_fifo, timeout waiting for data\n");
                return -1;
            }
            else usleep(200);
        }
    }

    // no need to read the fifo if it is empty so just return now
    if(p_available==0) return 0;

    // warn of partial packets, this is normal for MPU9250
    if(fifo_count%packet_size){
        PRINT("WARNING in io_rpc_spi_read_imu_fifo, fifo_count reported partial packets in fifo, count=%d", fifo_count);
    }

     // set speed for reading fifo data
    if(data_speed!=count_speed) io_rpc_spi_set_freq(bus, data_speed);

    // things for the SPI ioctl transfer
    struct dspal_spi_ioctl_read_write read_write;
    uint8_t tx_buf[1]; // one byte for address
    tx_buf[0] = fifo_addr | 0x80; // register high bit=1 for read
    read_write.read_buffer         = read_buf[bus];
    read_write.write_buffer        = tx_buf;
    read_write.write_buffer_length = 1;

    // keep reading the fifo until it is empty
    // limit maximum packets to read at a time to this
    int max_p_per_read = (DSPAL_SPI_RECEIVE_BUFFER_LENGTH-1)/packet_size;
    int p_read = 0;
    while(p_read < p_available){
        // read up to max_p_per_read
        int p_to_read = p_available-p_read;
        if(p_to_read>max_p_per_read) p_to_read=max_p_per_read;
        int b_to_read = p_to_read * packet_size;

        // set up ioctl transfer struct
        int transfer_bytes = 1 + b_to_read; // first byte is address
        read_write.read_buffer_length  = transfer_bytes;

        // read bus
        int ret = ioctl(spi_fd[bus], SPI_IOCTL_RDWR, &read_write);
        if(ret != transfer_bytes){
            PRINT("ERROR in io_rpc_spi_read_imu_fifo, ioctl returned %d", ret);
            return -1;
        }

        // skip the first (dummy) byte and read out the rest
        memcpy(&data[p_read*packet_size], &(read_buf[bus][1]), b_to_read);

        // keep track of how many we read this loop
        p_read+=p_to_read;
    }

    // let the user know how many packets got read
    *packets_read = p_read;
    return 0;
}


int io_rpc_gather_flir_segment(int32 spi_bus, unsigned char* outdata, int dataLen)
{

    int discard_pile = 0;
    int ret;

    // Sanity checks
    if(spi_bus<0 || spi_bus>MAX_BUS){
        PRINT("ERROR in io_rpc_spi_gather_flir_segment, bus must be between 0 & %d", MAX_BUS);
        return -1;
    }

    if(spi_fd[spi_bus]==0){
        PRINT("ERROR in io_rpc_spi_gather_flir_segment, bus %d must be initialized first", spi_bus);
        return -1;
    }

    while (true){

        // Constantly read packets into first index
        ret = io_rpc_spi_read_reg(spi_bus, 0, &outdata[0], 164);
        if (ret < 0){
            PRINT("ERROR reading segment\n");
            return -1;
        }

        // If we get too many discards, return and resync
        if (discard_pile > 100){
            PRINT("Too many discard packets, resync\n");
            discard_pile = 0;
            return -1;
        }

        // Identify discard packets, add to the pile
        if ((outdata[0] & 0x0f) == 0x0f){
            discard_pile++;
            continue;
        }

        // If we encounter start of segment, break out of loop and begin reading rest
        if (outdata[1] == 0){
            break;
        }

    }

    // Read 2 more packets
    ret = io_rpc_spi_read_reg(spi_bus, 0, &outdata[164], 164 * 2);
    if (ret < 0){
        PRINT("ERROR reading segment\n");
        return -1;
    }

    // Check if second packet is in order
    if (outdata[164 + 1] != 1){
        PRINT("Packet following frame 0 not in order\n");
        return -1;
    }

    // Grab frames 3-59 since we've already grabbed frames 0-2 from initial spi_reads
    for (int i = 1; i < 20; i++){
        // Read in chunks of 3 packets at a time (DSP spi read limit is 512 bytes at a time)
        ret = io_rpc_spi_read_reg(spi_bus, 0, &outdata[(i * 164 * 3)], 164 * 3);
        if(ret < 0){
            PRINT("ERROR reading segment\n");
            return -1;
        }
        // Print every 3rd packet (Reading 3 at a time)
        PRINT("Packet ID: %d\n", outdata[(i * 164 * 3) + 1]);
    }
    return 0;
}

