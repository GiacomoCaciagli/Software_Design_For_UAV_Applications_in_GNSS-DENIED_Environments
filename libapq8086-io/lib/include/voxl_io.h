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

/*
 * <voxl_io.h>
 *
 * @brief      C interface for the VOXL apps proc to interface with IO
 * ports through the sensors DSP (SLPI).
 *
 *
 * @author     James Strawson
 * @date       7/8/2019
 */

#ifndef VOXL_IO_H
#define VOXL_IO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>





/*******************************************************************************
 * RPC Shared Memory
 ******************************************************************************/

/**
 * @brief      allocates memory shared between SDSP and APPS proc
 *
 * @param[in]  bytes  bytes to allocate
 *
 * @return     pointer to memory on success, NULL on failure
 */
uint8_t* voxl_rpc_shared_mem_alloc(size_t bytes);


/**
 * @brief      frees previously allocated RPC shared memory
 *
 * @param      ptr   pointer to memory
 */
void voxl_rpc_shared_mem_free(uint8_t* ptr);


/**
 * @brief      deinitiazes the RPC shared memory system
 */
void voxl_rpc_shared_mem_deinit();


/*******************************************************************************
 * UART
 ******************************************************************************/

#define UART_J7		9	// BLSP 9  on physical port J7  pins 2&3
#define UART_J10	7	// BLSP 7  on physical port J10 pins 2&3
#define UART_J11	12	// BLSP 12 on physical port J11 pins 2&3
#define UART_J12	5	// BLSP 5  on physical port J12 pins 2&3

#define UART_DATA_BUF_LEN	128	// uart read buffer should be this length

/**
 * @brief      Initializes a UART bus /dev/tty-{bus} at specified baudrate
 *
 *             This is a very generalized function that configures the bus for
 *             8-bit characters, ignores the modem status lines, disables
 *             parity, disables canonical mode, and enables 1 stop bit. More
 *             flexibility in configuration could be added in the future if
 *             necessary. For now this covers the majority of sensors.
 *
 * @param[in]  bus       UART bus number (BLSP #)
 * @param[in]  baudrate  must be one of the standard speeds in the UART spec.
 *                       115200 and 57600 are most common.
 *
 * @return     0 on success, -1 on failure
 */
int voxl_uart_init(int bus, int baudrate);


/**
 * @brief      closes a UART bus
 *
 *             Please make an effort to call this before your process exits to
 *             help ensure a safe cleanup.
 *
 * @param[in]  bus   UART bus number (BLSP #)
 *
 * @return     returns 0 on success, -1 on error.
 */
int voxl_uart_close(int bus);


/**
 * @brief      Sends data out the specified uart port through the SDSP
 *
 *             This will return immediately, even if the data hasn't been sent
 *             out yet. Use voxl_uart_drain if you need to ensure the data has
 *             been sent before continuing. Max write length is currently
 *             untested, UART_DATA_BUF_LEN is a safe limit.
 *
 * @param[in]  bus    UART bus number (BLSP #)
 * @param[in]  data   pointer to data to be sent
 * @param[in]  bytes  number of bytes to send
 *
 * @return     returns number of bytes sent or -1 on error
 */
int voxl_uart_write(int bus, uint8_t* data, size_t bytes);


/**
 * @brief      Reads any and all data available from the UART hardware buffer
 *
 *             This is a non-blocking function call! You do not get to specify
 *             the number of bytes it reads, it will always read out everything
 *             in the hardware buffer. The dataLen argument should be the size
 *             of the data buffer in bytes which should generally be set to
 *             UART_DATA_BUF_LEN (128)
 *
 * @param[in]  bus      UART bus number (BLSP #)
 * @param      data     pointer to read buffer
 * @param[in]  dataLen  length of the read buffer (should be UART_DATA_BUF_LEN)
 *
 * @return     Returns number of bytes read or -1 on error.
 */
int voxl_uart_read(int bus, uint8_t* data, size_t dataLen);


/**
 * @brief      flushes (discards) any data received but not read, or data
 *             written but not sent.
 *
 *             uses tcflush(fd, TCIOFLUSH)
 *
 * @param[in]  bus   UART bus number (BLSP #)
 *
 * @return     0 on success or -1 on failure
 */
int voxl_uart_flush(int bus);


/**
 * @brief      waits for data in the write buffer to be sent before returning
 *
 * @param[in]  bus   UART bus number (BLSP #)
 *
 * @return     0 on success or -1 on failure
 */
int voxl_uart_drain(int bus);






/*******************************************************************************
 * Timing
 ******************************************************************************/

/**
 * @brief      get the current monotonic time of the sdsp in nanoseconds
 *
 * @return     current monotonic time of the sdsp or -1 on error
 */
int64_t voxl_sdsp_time_monotonic_ns();


/**
 * @brief      get the current realtime time of the sdsp in nanoseconds
 *
 * @return     current realtime time of the sdsp or -1 on error
 */
int64_t voxl_sdsp_time_realtime_ns();


/**
 * @brief      get the current monotonic time of the apps proc in nanoseconds
 *
 * @return     current monotonic time of the apps proc or -1 on error
 */
int64_t voxl_apps_time_monotonic_ns();


/**
 * @brief      get the current realtime time of the apps proc in nanoseconds
 *
 * @return     current realtime time of the apps proc or -1 on error
 */
int64_t voxl_apps_time_realtime_ns();


/**
 * @brief      get the time difference between apps and dsp clocks
 *
 *             sdsp clock is ahead of the apps proc so this offset should be
 *             positive. Note that the SDSP clock is sourced differently than
 *             the apps-proc clock so you may see 10us drift each second between
 *             the two clocks. Don't assume this offset is constant forever!
 *
 *             Represented as offset_ns = sdsp_monotonic_ns - apps_monotonic_ns
 *
 * @return     current monotonic time of the sdsp or -1 on error
 */
int64_t voxl_sdsp_time_offset_ns();


/*******************************************************************************
 * SPI
 ******************************************************************************/

/**
 * The maximum length of any receive or transmit over SPI bus.
 */
#define DSPAL_SPI_TRANSMIT_BUFFER_LENGTH 512
#define DSPAL_SPI_RECEIVE_BUFFER_LENGTH  512

/**
 * @brief      Initializes a SPI port a specified mode and frequency
 *
 *             For description of the 4 SPI bus modes, see
 *             <https://en.wikipedia.org/wiki/Serial_Peripheral_Interface_Bus#Mode_numbers>
 *
 *             Frequency is provided in hertz. MPU9250 claims to support up to 20mhz but we observed corrupted data. In practice 10mhz is plenty fast.
 *
 * @param[in]  bus       SPI bus number (BLSP #)
 * @param[in]  bus_mode  0,1,2, or 3
 * @param[in]  freq_hz   The frequency in hz
 *
 * @return     0 on success, -1 on failure
 */
int voxl_spi_init(int bus, int bus_mode, int freq_hz);


/**
 * @brief      update the speed of the SPI port
 *
 *             Use this is you wish to run the bus slower for some operations
 *             and faster for others. For example register configuration may
 *             want to run more slowly for robustness but read in data quickly.
 *
 * @param[in]  bus      SPI bus number (BLSP #)
 * @param[in]  freq_hz  The frequency in hz
 *
 * @return     0 on success, -1 on failure
 */
int voxl_spi_set_freq(int bus, int freq_hz);

/**
 * @brief      declare if SPI read function accounts for an initial dummy bit for specific fd
 *
 * @param[in]  bus      SPI bus number (BLSP #)
 * @param[in]  val      0 to keep dummy bit, non-zero to skip dummy bit (0 is default)
 *
 * @return     0 on success, -1 on failure
 */
int voxl_spi_skip_dummy_bit(int bus, int val);

/**
 * @brief      closes a SPI bus
 *
 *             Please make an effort to call this before your process exits to
 *             help ensure a safe cleanup.
 *
 * @param[in]  bus   SPI bus number (BLSP #)
 *
 * @return     returns 0 on success, -1 on error.
 */
int voxl_spi_close(int bus);

/**
 * @brief      writes data out the SPI bus
 *
 * data length can't exceed DSPAL_SPI_TRANSMIT_BUFFER_LENGTH (512)
 *
 * @param[in]  bus    SPI bus number (BLSP #)
 * @param      data   pointer to data to be written
 * @param[in]  bytes  number of bytes to send
 *
 * @return     0 on success -1 on failure
 */
int voxl_spi_write(int bus, uint8_t* data, int bytes);
int voxl_spi_write_cs(int bus, uint8_t* data, int bytes, int cs_gpio);


/**
 * @brief      reads specified number of bytes out of a register
 *
 *             You cannot read/write more than DSPAL_SPI_RECEIVE_BUFFER_LENGTH (512)
 *             bytes at a time.
 *
 * @param[in]  bus        SPI bus number (BLSP #)
 * @param      write_buf  data to send out bus
 * @param[in]  write_len  number of bytes to write
 * @param      read_buf   Buffer to write result into
 * @param[in]  read_len   number of bytes to read
 *
 * @return     0 on success, -1 on failure
 */
int voxl_spi_transfer(int bus, uint8_t* write_buf, int write_len, uint8_t* read_buf, int read_len);
int voxl_spi_transfer_cs(int bus, uint8_t* write_buf, int write_len, uint8_t* read_buf, int read_len, int cs_gpio);

/**
 * @brief      writes a single byte register at specified address
 *
 *             this works by writing the address followed by val which is how we
 *             write registers on the mpu9250 IMU. It may not work on your
 *             sensor.
 *
 * @param[in]  bus      SPI bus number (BLSP #)
 * @param[in]  address  register address
 * @param[in]  val      The value to write
 *
 * @return     0 on success, -1 on failure
 */
int voxl_spi_write_reg_byte(int bus, uint8_t address, uint8_t val);
int voxl_spi_write_reg_byte_cs(int bus, uint8_t address, uint8_t val, int cs_gpio);

/**
 * @brief      writes a single word (16 bit) register at specified address
 *
 *             this works by writing the address followed by val which is how we
 *             write registers on the mpu9250 IMU. It may not work on your
 *             sensor.
 *
 * @param[in]  bus      SPI bus number (BLSP #)
 * @param[in]  address  register address
 * @param[in]  val      The value to write
 *
 * @return     0 on success, -1 on failure
 */
int voxl_spi_write_reg_word(int bus, uint8_t address, uint16_t val);
int voxl_spi_write_reg_word_cs(int bus, uint8_t address, uint16_t val, int cs_gpio);

/**
 * @brief      reads specified number of bytes out of a register
 *
 *             This works by sending the requested address with the MSB set to 1
 *             indicating a read operation on MPU9250 and similar IMUs. It then
 *             reads the specified number of bytes out. This is intended for MPU
 *             series IMUs and may not work with your sensor if it behaves
 *             differently.
 *
 *             If reading chunks of data you should allocate RPC shared memory
 *             with voxl_rpc_shared_mem_alloc(size_t bytes) to use for the
 *             output buffer.
 *
 *             You cannot read more than DSPAL_SPI_RECEIVE_BUFFER_LENGTH (512)
 *             bytes at a time.
 *
 * @param[in]  bus      SPI bus number (BLSP #)
 * @param[in]  address  register address
 * @param      out_buf  pointer to buffer write the result into
 * @param[in]  length   The length of data to read in bytes
 *
 * @return     0 on success, -1 on failure
 */
int voxl_spi_read_reg(int bus, uint8_t address, uint8_t* out_buf, int length);
int voxl_spi_read_reg_cs(int bus, uint8_t address, uint8_t* out_buf, int length, int cs_gpio);

/**
 * @brief      reads a single byte register
 *
 *             This works by sending the requested address with the MSB set to 1
 *             indicating a read operation on MPU9250 and similar IMUs. It then
 *             reads one byte out. This is intended for MPU series IMUs and may
 *             not work with your sensor if it behaves differently.
 *
 * @param[in]  bus      SPI bus number (BLSP #)
 * @param[in]  address  register address
 * @param      out      pointer to where to write the result
 *
 * @return     0 on success, -1 on failure
 */
int voxl_spi_read_reg_byte(int bus, uint8_t address, uint8_t* out);
int voxl_spi_read_reg_byte_cs(int bus, uint8_t address, uint8_t* out, int cs_gpio);

/**
 * @brief      reads a single word (16-bit) register
 *
 *             This works by sending the requested address with the MSB set to 1
 *             indicating a read operation on MPU9250 and similar IMUs. It then
 *             reads two bytes out. This is intended for MPU series IMUs and may
 *             not work with your sensor if it behaves differently.
 *
 * @param[in]  bus      SPI bus number (BLSP #)
 * @param[in]  address  register address
 * @param      out      pointer to where to write the result
 *
 * @return     0 on success, -1 on failure
 */
int voxl_spi_read_reg_word(int bus, uint8_t address, uint16_t* out);
int voxl_spi_read_reg_word_cs(int bus, uint8_t address, uint16_t* out, int cs_gpio);

/**
 * @brief      helper function to read out a regularly formed FIFO buffer
 *
 *             This is intended for the MPU series IMU FIFO buffers. The process
 *             consists of first reading the number of bytes available in the
 *             IMU's FIFO as read from the fifo_count register address. This
 *             count should be a multiple of packet_size.
 *
 *             The data (even multiple of packet_size) is then read out of the
 *             fifo_address register and placed into the out_buf buffer.
 *
 *             The user should allocate RPC shared memory with
 *             voxl_rpc_shared_mem_alloc(size_t bytes) to use for the output
 *             buffer with at least the size of the IMU's FIFO. Also provide the
 *             size of this buffer with the dataLen argument so the functions
 *             knows not to write too far and create an overflow. You should
 *             make sure your RPC shared mem and this length are appropriately
 *             sized for the fifo you are reading.
 *
 *             For example the icm42688 has a 2k buffer but also has additional
 *             read cache so it can technically hold 103 packets (2060) bytes.
 *             We use a 2k + 128 = 2170 bytes buffer in voxl-imu-server to be
 *             safe.
 *
 *             The user can specify an optinal minimum number of packets to
 *             read. If greater than 0, the function will not return until the
 *             fifo has at least min_packets available to read. This can help
 *             reduce CPU usage at high sample rates.
 *
 * @param[in]  bus            SPI bus number (BLSP #)
 * @param[in]  count_address  fifo_count register address
 * @param[in]  fifo_address   fifo_data address
 * @param[in]  packet_size    expected bytes per fifo packet
 * @param[in]  min_packets    The minimum number of packets to read before
 *                            returning
 * @param[out] packets_read   The number of packets read
 * @param[out] data           The output buffer (should be RPC shared memory)
 * @param[in]  dataLen        Size of the output buffer.
 * @param[in]  count_speed    SPI speed in hz to read fifo_count register
 * @param[in]  data_speed     SPI speed in hz to read the fifo data register
 *
 * @return     0 on success, -1 on failure
 */
 
 
int voxl_spi_read_imu_fifo( int bus, uint8_t count_address, uint8_t fifo_address,\
                            uint8_t packet_size, int min_packets, \
                            int* packets_read, uint8_t* data, int dataLen, \
                            int count_speed, int data_speed);
                            
/**
 *  @brief     helper function to read out 60 VOSPI_PACKETS as detailed in FLIR 
 *             lepton engineering datasheet
 *
 *             This works by constantly reading 164 Byte chunks through designated spi_bus
 *             and then doing a quick check to ensure each packet is either the start of
 *             a segment, or a discard packet.
 *
 *             After the start of a segment is identified, the remaining 59 packets are
 *             read and stored into a shared memory buffer.
 *
 * @param[in]  spi_bus        SPI bus number (BLSP #)
 * @param[in]  segBufin       Shared memory buffer
 *
 * @param[out] data           The output data (Will be filled with data from segBufin)
 *
 * @return     0 on success, -1 on failure
 */
int voxl_spi_gather_flir_segment(int spi_bus, uint8_t* data, uint8_t* segBufin);


/*******************************************************************************
 * GPIO
 ******************************************************************************/

//highest number of GPIO pin which can be used (note that actual available pin count may vary)
#define MAX_NUM_GPIO_PINS 256
#define GPIO_DIRECTION_INPUT 0
#define GPIO_DIRECTION_OUTPUT 1

/**
 * @brief      Initializes a GPIO pin with desired direction and extra flags
 *
 * @param[in]  gpio_pin       GPIO pin number (not BLSP #)
 * @param[in]  direction      Direction : GPIO_DIRECTION_INPUT or GPIO_DIRECTION_OUTPUT
 * @param[in]  flags          Reserved for future use
 *
 * @return     0 on success, -1 on failure
 */
int voxl_gpio_init(int gpio_pin, int direction, int flags);

/**
 * @brief      Read the state of a GPIO pin
 *
 *             The pin must be initialized using voxl_gpio_init before reading its state
 *
 * @param[in]  gpio_pin       GPIO pin number
 * @param[in]  gpio_state_ptr Pointer to the integer where pin state will be written
 *
 * @return     0 on success, -1 on failure
 */
int voxl_gpio_read(int gpio_pin, int * gpio_state_ptr);

/**
 * @brief      Set the state of a GPIO pin
 *
 *             The pin must be initialized using voxl_gpio_init in output mode before setting its state
 *
 * @param[in]  gpio_pin       GPIO pin number
 * @param[out] gpio_state     Desired state of the pin (0=LOW or 1=High)
 *
 * @return     0 on success, -1 on failure
 */
int voxl_gpio_write(int gpio_pin, int gpio_state);

/**
 * @brief      Clean up after using GPIO pin
 *
 * @param[in]  gpio_pin       GPIO pin number
 *
 * @return     0 on success, -1 on failure
 */
int voxl_gpio_close(int gpio_pin);



/*******************************************************************************
 * I2C
 ******************************************************************************/

#define MAX_NUM_I2C_PORTS 16 //highest number of i2c port (BLSP #)
#define MAX_I2C_TX_BUFFER_SIZE 128

/**
 * @brief      Initializes an I2C port
 *
 *             Note that I2C bus speed and slave ID are sent using voxl_i2c_slave_config
 *
 * @param[in]  bus       I2C bus number (BLSP #)
 *
 * @return     0 on success, -1 on failure
 */
int voxl_i2c_init(uint32_t bus);

/**
 * @brief      Closes an I2C port
 *
 * @param[in]  bus       I2C bus number (BLSP #)
 *
 * @return     0 on success, -1 on failure
 */
int voxl_i2c_close(uint32_t bus);

/**
 * @brief      Configures I2C port for communication with a certain i2c slave ID
 *
 * @param[in]  bus                  I2C bus number (BLSP #)
 * @param[in]  slave_address_7bit   I2C slave address (7bit format, 7bit only (10 bit slave addresses are not supported))
 * @param[in]  bit_rate             I2C bus bit rate (e.g 100000, 400000)
 * @param[in]  timeout_us           Timeout for transfer (in microseconds)
 *
 * @return     0 on success, -1 on failure
 */
int voxl_i2c_slave_config(uint32_t bus, uint16_t slave_address, uint32_t bit_rate, uint32_t timeout_us);

/**
 * @brief      Write data to I2C slave device starting from the specified 8-bit register address
 *
 * @param[in]  bus                  I2C bus number (BLSP #)
 * @param[in]  register_address     Register address to access on the device (note: byte order is LS Byte first if > 8 bits)
 * @param[in]  address_num_bits     Number of bits to use for sending register address (8, 16, 24, or 32)
 * @param[in]  output_buffer        Buffer with data to send to the device
 * @param[in]  length               Number of bytes to send to device (in addition to the register address)
 *
 * @return     0 on success, -1 on failure
 */
int voxl_i2c_write(uint32_t bus, uint32_t register_address, uint32_t address_num_bits, uint8_t * write_buffer, uint32_t write_length);

/**
 * @brief      Read data from I2C slave device starting from the specified 8-bit register address
 *
 * @param[in]  bus                  I2C bus number (BLSP #)
 * @param[in]  register_address     Register address to access on the device (note: byte order is LS Byte first if > 8 bits)
 * @param[in]  address_num_bits     Number of bits to use for sending register address (8, 16, 24, or 32)
 * @param[in]  input_buffer         Buffer to hold the incoming data
 * @param[in]  length               Number of bytes to read from device
 *
 * @return     0 on success, -1 on failure
 */
int voxl_i2c_read(uint32_t bus, uint32_t register_address, uint32_t address_num_bits, uint8_t * read_buffer, uint32_t read_length);


/*******************************************************************************
 * MAVPARSER
 ******************************************************************************/

/**
 * @brief      starts a UART mavlink parsing thread on the SDSP
 *
 *             buflen specifies how large of a buffer the SDSP should allocate
 *             internally. The user should allocate their own RPC shared memory
 *             buffer matching this length for the data to be copied out into
 *             late.
 *
 *             There is currenly only one valid flag available:
 *
 *             FLAG_MAVPARSER_ALLOW_CRC_FAILURES
 *
 *             The allow CRC failures flag is useful because the CRC on Mavlink
 *             messages is more than just a message integrity check. It has been
 *             overloaded with a type of message version check and this can be
 *             specific to certain implementations. See the description of
 *             CRC_EXTRA at this link:
 *             https://mavlink.io/en/guide/serialization.html#crc_extra
 *
 * @param[in]  bus       The UART bus
 * @param[in]  baudrate  The baudrate
 * @param[in]  buflen    Buffer length for SDSP to use
 * @param[in]  flags     flags to change behavior
 *
 * @return     0 on success, -1 on failure
 */
int voxl_mavparser_init(int bus, int baudrate, int buflen, int flags);


/**
 * @brief      Copies out any complete parsed mavlnk messages and their status
 *             to the RPC data buffer allocated during the init step.
 *
 *             Each message has a corresponding status written to the user's
 *             status array. The status matches mavlink_framing_t:
 *
 *             MAVLINK_FRAMING_OK=1, MAVLINK_FRAMING_BAD_CRC=2,
 *             MAVLINK_FRAMING_BAD_SIGNATURE=3
 *
 *             Messages are returned to the user even if they are parsed with
 *             bad signatures or bad CRCs since these two errors don't necessary
 *             mean there was an integrity problem.
 *
 *             The CRC on Mavlink messages is more than just a message integrity
 *             check. It has been overloaded with a type of message version
 *             check and this can be specific to certain implementations. If the
 *             mavlink packet structures used by the sender differs from the
 *             version we have built into this library, the CRC may fail. But
 *             once the message is forwarded to another receiver, they may
 *             decode the packet correctly with no CRC error. See the
 *             description of CRC_EXTRA at this link:
 *             https://mavlink.io/en/guide/serialization.html#crc_extra
 *
 *             This library is always built with the latest master branch of the
 *             Mavlink project to keep up with new packets.
 *
 *             The status array should be long enough to hold the single-byte
 *             status of the maxiumum number of mavlink_message_t structs that
 *             can fit in the user's data buffer. See voxl-test-mavparser.c for
 *             an example.
 *
 *             This function will block for up to 1/2 second and will return if
 *             at least one message is ready or the 1/2 second timeout elapsed.
 *
 * @param[in]  bus     The UART bus
 * @param[out] data    pointer to a mavlink_msg_t array to write out to
 * @param      status  pointer to user's status array
 *
 * @return     returns number of mavlink messages read or -1 on failure
 */
int voxl_mavparser_read(int bus, char* data, uint8_t* status);


/**
 * @brief      stop a mavparser thread
 *
 * @param[in]  bus   The uart bus
 *
 * @return     0 on success, -1 on failure
 */
int voxl_mavparser_close(int bus);




#ifdef __cplusplus
}
#endif

#endif // VOXL_IO_H
