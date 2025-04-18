// Abilitate the MicroXRCE Client-Agent connection on a ModalAi VOXL board.
// Read the messages sent by the Client on a specific serial port and forward them 
// to the Agent over UDP and viceversa. The messages in both directions are manipulated
// to match the two different communication types.
#include <limits.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h> 
#include <voxl_io.h>
#include <sys/types.h>
#include <sys/socket.h>

#include <arpa/inet.h>
#include <netinet/in.h>
	
#define PORT			8888		// port number for the udp connection
#define MAXLINE		4096		// max number of bytes readable with every read, also max message length
#define MAXQUEUE	20			// max number of uart->udp messages queued
#define BUS 			5				// serial port number to read from
#define BAUDRATE 	921600	// port baudrate

// matrix for the computation of the cyclic redundancy check
static uint16_t crc16_table[256] =
  {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
  };

// specific byte needed by the serial protocol
static uint8_t framing_begin_flag = 0x7E;
static uint8_t framing_esc_flag = 0x7D; // indicates that the next byte needs a specific operation or that the message is end
static int8_t framing_xor_flag = 0x20; // byte needed in the processing of specific bytes in the message

enum InputState{
	UXR_FRAMING_UNINITIALIZED,
	UXR_FRAMING_READING_SRC_ADDR,
	UXR_FRAMING_READING_DST_ADDR,
	UXR_FRAMING_READING_LEN_LSB,
	UXR_FRAMING_READING_LEN_MSB,
	UXR_FRAMING_READING_PAYLOAD,
	UXR_FRAMING_READING_CRC_LSB,
	UXR_FRAMING_READING_CRC_MSB,
};

/*
	Collects the messages in the uart->udp communication
*/
struct Queue {
	uint8_t arr[MAXQUEUE][MAXLINE];
	int lenghts[MAXQUEUE];
	int back;
	int front;
	int is_full;
	int is_empty;
	pthread_mutex_t mutex;
};

struct ServerComm {
	int sockfd;
	struct sockaddr_in servaddr;
	socklen_t len;
};

/*
	Contains the data that needs to be shared between threads
*/
struct GlobalData{
	struct ServerComm server_comm;
	struct Queue queue;
};

/*
	Contains the message for the udp->uart communication	
*/
struct WriteBuffer {
	uint8_t* buffer_; // pointer to the memory buffer
	uint16_t buffer_idx; // actual position
	int num_bytes; // number of bytes effectively in the buffer
};

/*
	Debug structure, not necessary for the code
*/
struct DebugQueue {
	uint8_t arr[MAXQUEUE][MAXLINE];
	int lenghts[MAXQUEUE];
	int idx;
};

/*
	Contains all the raw bytes in the uart->udp communication
*/
struct ReadBuffer {
	uint8_t* buffer_; 
	uint16_t read_idx; 
	uint16_t buffer_len; 
	uint16_t counter; // number of bytes read, handle the case where the lenght of the buffer is equal to the max
	enum InputState state; // indicate the currently processed byte
	uint16_t msg_len; // length of the payload
	uint16_t msg_pos; // actual position in the read operation of the payload
	uint16_t msg_crc; // crc read
	uint16_t cmp_crc; // crc computed
  uint8_t remote_addr_; // flight controller address
};

/**
 * 	@brief Compute the cyclic redundancy check
 */
void update_crc(uint16_t* crc,const uint8_t data)
{
    *crc = (*crc >> 8) ^ crc16_table[(*crc ^ data) & 0xFF];
}

/**
 * @brief Gets the next byte in the buffer and handles the specific bytes needed
 * 				by the serial protocol. Returns 0 (false) if we need to read again from
 * 				the port
 */
int get_next_octet(struct ReadBuffer* read_buffer,uint8_t* octet) 
{
	int rv = 0;
  *octet = 0;

	if (read_buffer->counter != read_buffer->buffer_len)
	{
    if (framing_esc_flag != read_buffer->buffer_[read_buffer->read_idx])
		{
			*octet = read_buffer->buffer_[read_buffer->read_idx];
			read_buffer->read_idx = (uint16_t)((size_t)(read_buffer->read_idx + 1) % MAXLINE);
			read_buffer->counter = read_buffer->counter + 1;

			rv = (framing_begin_flag != *octet);
		}else
		{
			uint16_t temp_tail = (uint16_t)((size_t)(read_buffer->read_idx + 1) % MAXLINE);

			if (temp_tail != read_buffer->buffer_len && temp_tail != 0)
			{
				*octet = read_buffer->buffer_[temp_tail];
				read_buffer->read_idx = (uint16_t)((size_t)(read_buffer->read_idx + 2) % MAXLINE);
				read_buffer->counter = read_buffer->counter + 2;

				if (framing_begin_flag != *octet)
				{
					*octet ^= framing_xor_flag;
					rv = 1;
				}
			}
		}
	}
	return rv;
}

/**
 * @brief Adds a byte to the buffer and handles the specific bytes needed
 * 				by the serial protocol. Returns 0 (false) if the buffer is full 
 */
int add_next_octet(struct WriteBuffer* write_buffer,uint8_t* octet)
{
	int rv = 0;

	if (framing_begin_flag == *octet || framing_esc_flag == *octet)
	{
		if ((write_buffer->buffer_idx + 1) < MAXLINE)
		{
			write_buffer->buffer_[write_buffer->buffer_idx] = framing_esc_flag;
			write_buffer->buffer_[write_buffer->buffer_idx + 1] = *octet ^ framing_xor_flag;
			write_buffer->buffer_idx += 2;
			rv = 1;
		}
	}
	else if (write_buffer->buffer_idx < MAXLINE)
	{
		write_buffer->buffer_[write_buffer->buffer_idx] = *octet;
		++write_buffer->buffer_idx;
		rv = 1;
	}

	return rv;
}

/**
 * @brief Processes one message received from the serial port and extracts 
 * 				the remote address and the payload of the message. Returns 0 (false)
 * 				if the message is incomplete or the crc is unsuccessful, if returns 0,
 * 				the message will be discarded
 */
int read_framed_msg(struct ReadBuffer* read_buffer, uint8_t* remote_addr, size_t len, uint8_t* msg, struct DebugQueue* debug_buffer)
{
	int rv=1;
	uint8_t local_addr=0;
	int bytes=0; // need if we need to read again from the uart
	
	int exit_cond = 0;
	int i,j,ind=0,sleep_us=100;
	while (!exit_cond)
	{
		uint8_t octet = 0;
		switch (read_buffer->state)
		{
			case UXR_FRAMING_UNINITIALIZED: // frame start (1 byte)
			{
				octet = 0;
				debug_buffer->lenghts[debug_buffer->idx] = 0;
				while ((framing_begin_flag != octet))
				{
					octet = read_buffer->buffer_[read_buffer->read_idx];
					read_buffer->read_idx =(uint16_t)((size_t)(read_buffer->read_idx + 1) % MAXLINE);
					read_buffer->counter = read_buffer->counter + 1;

					debug_buffer->arr[debug_buffer->idx][ind] = octet;
					debug_buffer->lenghts[debug_buffer->idx]++;
					ind++;
				}

				if (framing_begin_flag == octet) // confirm the start of the message
				{
					read_buffer->state = UXR_FRAMING_READING_SRC_ADDR;
					if (debug_buffer->arr[debug_buffer->idx][0]!=126)
					{
						debug_buffer->idx++;
						debug_buffer->arr[debug_buffer->idx][0] = octet;
						debug_buffer->lenghts[debug_buffer->idx] = 1;
						ind = 1;
					}
				}
				else
				{
					printf("esco 1\n");
					exit_cond = 1; // 1 is true
				}
				break;
			}
			case UXR_FRAMING_READING_SRC_ADDR: // flight controller address (1 byte)
			{
				if (get_next_octet(read_buffer,&read_buffer->remote_addr_))
				{
					read_buffer->state = UXR_FRAMING_READING_DST_ADDR;
					debug_buffer->arr[debug_buffer->idx][ind] = read_buffer->remote_addr_;
					debug_buffer->lenghts[debug_buffer->idx]++;
					ind++;
				}
				else
				{
					if (framing_begin_flag == octet)
					{
						read_buffer->state = UXR_FRAMING_READING_SRC_ADDR;
						debug_buffer->idx++;
						debug_buffer->arr[debug_buffer->idx][0] = octet;
						debug_buffer->lenghts[debug_buffer->idx] = 1;
						ind = 1;
					}
					else
					{
						do
						{
							usleep(sleep_us);
							bytes = voxl_uart_read(BUS, read_buffer->buffer_, MAXLINE);
							if (bytes!=0)
							{
								read_buffer->buffer_len = bytes;
								read_buffer->read_idx = 0;
								read_buffer->counter = 0;
							}
							sleep_us+=100;
						} while (bytes==0);
						sleep_us = 100;
					}
				}
				break;
			}
			case UXR_FRAMING_READING_DST_ADDR: // companion address (1 byte)
			{
				if (get_next_octet(read_buffer,&octet))
				{
					read_buffer->state = (octet == local_addr) ? UXR_FRAMING_READING_LEN_LSB : UXR_FRAMING_UNINITIALIZED;
					debug_buffer->arr[debug_buffer->idx][ind] = octet;
					debug_buffer->lenghts[debug_buffer->idx]++;
					ind++;
				}
				else 
				{	
					if (framing_begin_flag == octet)
					{
						read_buffer->state = UXR_FRAMING_READING_SRC_ADDR;
						debug_buffer->idx++;
						debug_buffer->arr[debug_buffer->idx][0] = octet;
						debug_buffer->lenghts[debug_buffer->idx] = 1;
						ind = 1;
					}
					else
					{
						do
						{
							usleep(sleep_us);
							bytes = voxl_uart_read(BUS, read_buffer->buffer_, MAXLINE);
							if (bytes!=0)
							{
								read_buffer->buffer_len = bytes;
								read_buffer->read_idx = 0;
								read_buffer->counter = 0;
							}
							sleep_us+=100;
						} while (bytes==0);
						sleep_us = 100;
					}
				}
				break;
			}
			case UXR_FRAMING_READING_LEN_LSB: // least significant bits of the message lenght (1 byte)
			{
				if (get_next_octet(read_buffer,&octet))
				{
					read_buffer->msg_len = octet;
					read_buffer->state = UXR_FRAMING_READING_LEN_MSB;
					debug_buffer->arr[debug_buffer->idx][ind] = octet;
					debug_buffer->lenghts[debug_buffer->idx]++;
					ind++;
				}
				else
				{
					
					if (framing_begin_flag == octet)
					{
						read_buffer->state = UXR_FRAMING_READING_SRC_ADDR;
						debug_buffer->idx++;
						debug_buffer->arr[debug_buffer->idx][0] = octet;
						debug_buffer->lenghts[debug_buffer->idx] = 1;
						ind = 1;
					}
					else
					{
						do
						{
							usleep(sleep_us);
							bytes = voxl_uart_read(BUS, read_buffer->buffer_, MAXLINE);
							if (bytes!=0)
							{
								read_buffer->buffer_len = bytes;
								read_buffer->read_idx = 0;
								read_buffer->counter = 0;
							}
							sleep_us+=100;
						} while (bytes==0);
						sleep_us = 100;
					}
				}
				break;
			}
			case UXR_FRAMING_READING_LEN_MSB: // most significant bits of the message lenght (1 byte)
			{
				if (get_next_octet(read_buffer,&octet))
				{
					read_buffer->msg_len += (octet << 8);
					read_buffer->msg_pos = 0;
					read_buffer->cmp_crc = 0;
					if (len < read_buffer->msg_len)
					{
						read_buffer->state = UXR_FRAMING_UNINITIALIZED;
						exit_cond = 1;
						printf("esco 5\n");
					}
					else
					{
						read_buffer->state = UXR_FRAMING_READING_PAYLOAD;
						debug_buffer->arr[debug_buffer->idx][ind] = octet;
						debug_buffer->lenghts[debug_buffer->idx]++;
						ind++;
					}
				}
				else 
				{
					if (framing_begin_flag == octet)
					{
						read_buffer->state = UXR_FRAMING_READING_SRC_ADDR;
						debug_buffer->idx++;
						debug_buffer->arr[debug_buffer->idx][0] = octet;
						debug_buffer->lenghts[debug_buffer->idx] = 1;
						ind = 1;
					}
					else
					{
						printf("Need to read again (LEN MSB)\n");
						do
						{
							usleep(sleep_us);
							bytes = voxl_uart_read(BUS, read_buffer->buffer_, MAXLINE);
							if (bytes!=0)
							{
								read_buffer->buffer_len = bytes;
								read_buffer->read_idx = 0;
								read_buffer->counter = 0;
							}
							sleep_us+=100;
						} while (bytes==0);
						sleep_us = 100;
					}
				}
				break;
			}
			case UXR_FRAMING_READING_PAYLOAD: // payload, the message to send
			{
				while ((read_buffer->msg_pos < read_buffer->msg_len) && get_next_octet(read_buffer,&octet))
				{
					msg[(size_t)(read_buffer->msg_pos)] = octet;
					read_buffer->msg_pos++;
					update_crc(&read_buffer->cmp_crc, octet);
					debug_buffer->arr[debug_buffer->idx][ind] = octet;
					debug_buffer->lenghts[debug_buffer->idx]++;
					ind++;
				}

				if (read_buffer->msg_pos == read_buffer->msg_len)
				{
					read_buffer->state = UXR_FRAMING_READING_CRC_LSB;
				}
				else
				{
					if (framing_begin_flag == octet)
					{
						read_buffer->state = UXR_FRAMING_READING_SRC_ADDR;
						debug_buffer->idx++;
						debug_buffer->arr[debug_buffer->idx][0] = octet;
						debug_buffer->lenghts[debug_buffer->idx] = 1;
						ind = 1;
					}
					else
					{
						do
						{
							usleep(sleep_us);
							bytes = voxl_uart_read(BUS, read_buffer->buffer_, MAXLINE);
							if (bytes!=0)
							{
								read_buffer->buffer_len = bytes;
								read_buffer->read_idx = 0;
								read_buffer->counter = 0;
							}
							sleep_us+=100;
						} while (bytes==0);
						sleep_us = 100;
					}
				}
				break;
			}
			case UXR_FRAMING_READING_CRC_LSB: // least significant bits of the cyclic redundancy check (1 byte)
			{
				if (get_next_octet(read_buffer,&octet))
				{
					read_buffer->msg_crc = octet;
					read_buffer->state = UXR_FRAMING_READING_CRC_MSB;
					debug_buffer->arr[debug_buffer->idx][ind] = octet;
					debug_buffer->lenghts[debug_buffer->idx]++;
					ind++;
				}
				else 
				{
					if (framing_begin_flag == octet)
					{
						read_buffer->state = UXR_FRAMING_READING_SRC_ADDR;
						debug_buffer->idx++;
						debug_buffer->arr[debug_buffer->idx][0] = octet;
						debug_buffer->lenghts[debug_buffer->idx] = 1;
						ind = 1;
					}
					else
					{
						do
						{
							usleep(sleep_us);
							bytes = voxl_uart_read(BUS, read_buffer->buffer_, MAXLINE);
							if (bytes!=0)
							{
								read_buffer->buffer_len = bytes;
								read_buffer->read_idx = 0;
								read_buffer->counter = 0;
							}
							sleep_us+=100;
						} while (bytes==0);
						sleep_us = 100;
					}
				}
				break;
			}
			case UXR_FRAMING_READING_CRC_MSB: // most significant bits of the cyclic redundancy check (1 byte)
			{
				if (get_next_octet(read_buffer,&octet))
				{
					read_buffer->msg_crc += (octet << 8);
					read_buffer->state = UXR_FRAMING_UNINITIALIZED;
					debug_buffer->arr[debug_buffer->idx][ind] = octet;
					debug_buffer->lenghts[debug_buffer->idx]++;
					ind++;
					if (read_buffer->cmp_crc == read_buffer->msg_crc)
					{
						*remote_addr = read_buffer->remote_addr_;
						rv=0;
						debug_buffer->idx--;
					}
					exit_cond = 1;
				}
				else
				{
					if (framing_begin_flag == octet)
					{
						read_buffer->state = UXR_FRAMING_READING_SRC_ADDR;
						debug_buffer->idx++;
						debug_buffer->arr[debug_buffer->idx][0] = octet;
						debug_buffer->lenghts[debug_buffer->idx] = 1;
						ind = 1;
					}
					else
					{
						do
						{
							usleep(sleep_us);
							bytes = voxl_uart_read(BUS, read_buffer->buffer_, MAXLINE);
							if (bytes!=0)
							{
								read_buffer->buffer_len = bytes;
								read_buffer->read_idx = 0;
								read_buffer->counter = 0;
							}
							sleep_us+=100;
						} while (bytes==0);
						sleep_us = 100;
					}
				}
				break;
			}
			default:
			{
				printf("ERROR: impossible state reached");
				exit_cond = 1;
				break;
			}
		}
	}
    
	return rv;
}

/**
 * @brief Builds the message to be sent to the Client over the serial connection
 */
size_t write_framed_msg(struct WriteBuffer* write_buffer, size_t len, uint8_t remote_addr, uint8_t* msg)
{
	int i;
  uint8_t octet = 0;

  /* Buffer being flag. */
  write_buffer->buffer_[0] = framing_begin_flag;
  write_buffer->buffer_idx = 1;

  /* Buffer header. */
	octet = 0;
	add_next_octet(write_buffer,&octet);
	octet = remote_addr;
	add_next_octet(write_buffer,&octet);
	octet = (uint8_t)(len & 0xFF);
	add_next_octet(write_buffer,&octet);
	octet = (uint8_t)(len >> 8);
	add_next_octet(write_buffer,&octet);

  /* Write payload. */
  uint16_t written_len = 0;
  uint16_t crc = 0;
	int cond = 1;
  while (written_len < len && cond)
  {
    octet = (uint8_t)(*(msg + written_len));
		if (add_next_octet(write_buffer,&octet))
		{
			update_crc(&crc, octet);
			written_len++;
		}else
		{
			cond = voxl_uart_write(BUS, (uint8_t*)write_buffer->buffer_, write_buffer->buffer_idx);
		}
		
  }

  /* Write CRC. */
  uint8_t tmp_crc[2];
  tmp_crc[0] = (uint8_t)(crc & 0xFF);
  tmp_crc[1] = (uint8_t)(crc >> 8);
  written_len = 0;
  while (written_len < sizeof(tmp_crc) && cond)
  {
    octet = *(tmp_crc + written_len);
		if (add_next_octet(write_buffer,&octet))
		{
			update_crc(&crc, octet);
			written_len++;
		}else
		{
			cond = voxl_uart_write(BUS, (uint8_t*)write_buffer->buffer_, write_buffer->buffer_idx);
		}
  }

	cond = voxl_uart_write(BUS, (uint8_t*)write_buffer->buffer_, write_buffer->buffer_idx);

  return cond==write_buffer->buffer_idx ? len : 0;
}

/**
 * @brief Thread which reads and processes the messages incoming from the serial connection,
 * 				after the processing, the messages are stored in a queue waiting to be send to the 
 * 				agent
 */
void *uart_receiver_thread(void *data)
{
	int bytes,i,j,app,tot, tot_125, tot_msgs_sent;
	uint16_t len;
	int i_len;
	struct Queue *scheduler = (struct Queue *)data;
	struct ReadBuffer read_buffer={.buffer_=voxl_rpc_shared_mem_alloc(MAXLINE), .read_idx=0, .buffer_len=0 , .state=UXR_FRAMING_UNINITIALIZED, .counter = 0,
	.msg_len=0, .msg_pos=0, .msg_crc=0, .cmp_crc=0};
	struct DebugQueue debug_buffer = {.idx=0};
  uint8_t* msg = voxl_rpc_shared_mem_alloc(MAXLINE); // contains the message of the packets recevide
	uint8_t remote_addr;

	int err;

	memset(msg,0,MAXLINE);
	memset(read_buffer.buffer_, 0, MAXLINE);

  printf("Uart receiver thread started\n");

	while (1)
	{
		bytes=voxl_uart_read(BUS, read_buffer.buffer_, MAXLINE);

		if (bytes != 0)
		{
			if (bytes == -1)
			{
				printf("ERRORE\n");
				break;
			}else
			{
				tot = 0;
				tot_msgs_sent = 0;
				i_len = 0;
				debug_buffer.idx = 0;

				read_buffer.read_idx = 0;
				read_buffer.counter = 0;
				read_buffer.buffer_len = bytes;
	
				do
				{
					err=read_framed_msg(&read_buffer,&remote_addr,MAXLINE,msg,&debug_buffer);
					debug_buffer.idx++;

					if (err)
					{
						printf("There is a problem in the message \n");
						printf("crc read:%d, crc computed:%d\n",read_buffer.msg_crc,read_buffer.cmp_crc);
						printf("The message will be discarded");

					}else
					{
						app = scheduler->back;

						do
						{
							pthread_mutex_lock(&scheduler->mutex);

							if (!scheduler->is_full) 
							{
								scheduler->lenghts[scheduler->back] = read_buffer.msg_len;
								i_len++;
								tot_msgs_sent++;
								tot = tot + read_buffer.msg_len + 7;

								for(i=0;i<read_buffer.msg_len;i++)
								{
									scheduler->arr[scheduler->back][i] = msg[i];
								}

								scheduler->back = (scheduler->back + 1) % MAXQUEUE;
								
								if (scheduler->back == scheduler->front )
									scheduler->is_full = 1;

								scheduler->is_empty = 0;
							}

							pthread_mutex_unlock(&scheduler->mutex);

						} while (app == scheduler->back);
					}
				} while (read_buffer.counter < read_buffer.buffer_len);
				
				if (debug_buffer.idx>0)
				{
					printf("messages lost:%d\n",debug_buffer.idx);

					for (i=0;i<debug_buffer.idx;i++)
					{
						printf("msg %d:\n",i+1);

						for(j=0;j<debug_buffer.lenghts[i];j++)
						{
							printf("%d:%d ",j,debug_buffer.arr[i][j]);
						}

						printf("\n");
					}

					printf("\n\n");
				}		
			}
		}
	}

	voxl_rpc_shared_mem_free(read_buffer.buffer_);
	voxl_rpc_shared_mem_free(msg);

	return 0;	
}

/**
 * @brief Thread which sends the messages stored
 */
void *udp_sender_thread(void *data)
{
	struct GlobalData *full_struct = (struct GlobalData *)data;
	struct Queue *scheduler = &full_struct->queue;
	struct ServerComm *serv_comm = &full_struct->server_comm;
	int err,i;

  printf("Udp sender thread started\n");

	while (1)
	{
		pthread_mutex_lock(&scheduler->mutex);
		if (!scheduler->is_empty)
		{
			err=sendto(serv_comm->sockfd, (const uint8_t *)scheduler->arr[scheduler->front], scheduler->lenghts[scheduler->front],MSG_CONFIRM, (const struct sockaddr *) &serv_comm->servaddr,serv_comm->len);
			scheduler->front = (scheduler->front + 1) % MAXQUEUE;
			if (err == -1)
			{
				printf("ERRORE send udp\n");
			}

			if (scheduler->front == scheduler->back)
				scheduler->is_empty = 1;
			
			scheduler->is_full = 0;
		}
		pthread_mutex_unlock(&scheduler->mutex);
	}
	return 0;	
}

/**
 * @brief Thread that handles the udp->uart communication
 */
void *uart_sender_thread(void *data)
{
	struct ServerComm *serv_comm = (struct ServerComm *)data;
	struct WriteBuffer write_buffer={.buffer_=voxl_rpc_shared_mem_alloc(MAXLINE), .buffer_idx=0, .num_bytes=0};
	uint8_t* msg = voxl_rpc_shared_mem_alloc(MAXLINE);
	int bytes,res;
	uint8_t remote_addr=1;

  printf("Uart sender thread started\n");

	while(1)
	{
		bytes = recvfrom(serv_comm->sockfd, (uint8_t *)msg, MAXLINE,MSG_WAITALL, (struct sockaddr *) &serv_comm->servaddr,&serv_comm->len);

		if (bytes!=0)
		{
			if (bytes==-1)
			{
				printf("ERRORE:%s\n",strerror(errno));
			}else
			{
				write_buffer.num_bytes=bytes+7; // frame start, src addr, dst addr, lsb msg len, msb msg len, lsb crc, msb crc
				res=write_framed_msg(&write_buffer,bytes,remote_addr,msg);
				write_buffer.buffer_idx = 0;

				if (res!=bytes)
					printf("writing error");
			}
		}
	}

	voxl_rpc_shared_mem_free(write_buffer.buffer_);
	voxl_rpc_shared_mem_free(msg);
	return 0;	
}

int main() {
	
	pthread_t uart_receiver_t;
  pthread_t sender_t;
	pthread_t uart_sender_t;
	struct GlobalData data = {.queue = {.arr = {-1},.lenghts={-1}, .back = 0, .front = 0, .is_full=0, .is_empty=1}, .server_comm = {.len=sizeof(data.server_comm.servaddr)}};
	pthread_mutex_init(&data.queue.mutex, NULL);
	int err;
	
	if ( (data.server_comm.sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
		perror("socket creation failed");
		exit(EXIT_FAILURE);
	}

	if (setsockopt(data.server_comm.sockfd, SOL_SOCKET, SO_REUSEADDR, &(int){1}, sizeof(int)) < 0)
        perror("setsockopt(SO_REUSEADDR) failed");
	
	memset(&data.server_comm.servaddr, 0, sizeof(data.server_comm.servaddr));
		
	// Filling server information
	data.server_comm.servaddr.sin_family = AF_INET;
	data.server_comm.servaddr.sin_port = htons(PORT);
	data.server_comm.servaddr.sin_addr.s_addr = INADDR_ANY;

	// initializing uart port
  printf("INITIALIZING\n");
  err = voxl_uart_init(BUS, BAUDRATE);
  if(err) {
    fprintf(stderr, "ERROR initializing uart, err:%d\n",err);
    return -1;
  }

	pthread_create(&uart_receiver_t, NULL, uart_receiver_thread, &data.queue);
  pthread_create(&sender_t, NULL, udp_sender_thread, &data);
	pthread_create(&uart_sender_t, NULL, uart_sender_thread, &data.server_comm);

	pthread_join(uart_receiver_t, NULL);
	pthread_join(sender_t, NULL);
	pthread_join(uart_sender_t, NULL);

	voxl_rpc_shared_mem_deinit();
	printf("closing uart bus\n");
	err=voxl_uart_close(BUS);
	
	if(err){
			fprintf(stderr, "ERROR closing uart, err:%d\n",err);
	}
	close(data.server_comm.sockfd);	

	return 0;
}
