# *******************************************************************************
# * Copyright 2020 ModalAI Inc.
# *
# * Redistribution and use in source and binary forms, with or without
# * modification, are permitted provided that the following conditions are met:
# *
# * 1. Redistributions of source code must retain the above copyright notice,
# *    this list of conditions and the following disclaimer.
# *
# * 2. Redistributions in binary form must reproduce the above copyright notice,
# *    this list of conditions and the following disclaimer in the documentation
# *    and/or other materials provided with the distribution.
# *
# * 3. Neither the name of the copyright holder nor the names of its contributors
# *    may be used to endorse or promote products derived from this software
# *    without specific prior written permission.
# *
# * 4. The Software is used solely in conjunction with devices provided by
# *    ModalAI Inc.
# *
# * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# * POSSIBILITY OF SUCH DAMAGE.
# ******************************************************************************

import ctypes as ct
import numpy as np
import time

libvoxl_io = ct.CDLL("libvoxl_io.so")

#int voxl_i2c_init(uint32_t bus);
voxl_i2c_init          = libvoxl_io.voxl_i2c_init
voxl_i2c_init.argtypes = [ct.c_uint] # bus number
voxl_i2c_init.restype  = ct.c_int

#int voxl_i2c_close(uint32_t bus);
voxl_i2c_close          = libvoxl_io.voxl_i2c_close
voxl_i2c_close.argtypes = [ct.c_uint] # bus number
voxl_i2c_close.restype  = ct.c_int

#int voxl_i2c_slave_config(uint32_t bus, uint16_t slave_address, uint32_t bit_rate, uint32_t timeout_us);
voxl_i2c_slave_config          = libvoxl_io.voxl_i2c_slave_config
voxl_i2c_slave_config.argtypes = [ct.c_uint,ct.c_int16,ct.c_uint,ct.c_uint]
voxl_i2c_slave_config.restype  = ct.c_int

#int voxl_i2c_write(uint32_t bus, uint32_t register_address, uint32_t address_num_bits, uint8_t * write_buffer, uint32_t write_length);
voxl_i2c_write          = libvoxl_io.voxl_i2c_write
voxl_i2c_write.argtypes = [ct.c_uint, ct.c_uint, ct.c_uint, ct.POINTER(ct.c_uint8), ct.c_uint]
voxl_i2c_write.restype  = ct.c_int

#int voxl_i2c_read(uint32_t bus, uint32_t register_address, uint32_t address_num_bits, uint8_t * read_buffer, uint32_t read_length);
voxl_i2c_read          = libvoxl_io.voxl_i2c_read
voxl_i2c_read.argtypes = [ct.c_uint, ct.c_uint, ct.c_uint, ct.POINTER(ct.c_uint8), ct.c_uint]
voxl_i2c_read.restype  = ct.c_int

# allocate RPC buffer for reading data from DSP
voxl_rpc_shared_mem_alloc          = libvoxl_io.voxl_rpc_shared_mem_alloc
voxl_rpc_shared_mem_alloc.argtypes = [ct.c_size_t]
voxl_rpc_shared_mem_alloc.restype  = ct.POINTER(ct.c_uint8)

# free shared RPC buffer
voxl_rpc_shared_mem_free          = libvoxl_io.voxl_rpc_shared_mem_free
voxl_rpc_shared_mem_free.argtypes = [ct.POINTER(ct.c_uint8)]
voxl_rpc_shared_mem_free.restype  = None

# de-init RPC shared memory structures
voxl_rpc_shared_mem_deinit          = libvoxl_io.voxl_rpc_shared_mem_deinit
voxl_rpc_shared_mem_deinit.argyptes = []
voxl_rpc_shared_mem_deinit.restype  = None

class VoxlI2CPort():
    initialized   = False
    bit_rate      = 0
    port          = ''
    port_num      = -1
    timeout_us    = 1000
    read_buf_ptr  = None
    read_buf_size = 256

    def __init__(self):
        self.initialized = False

    def __del__(self):
        self.close()

    def open(self):
        if self.initialized:
            raise Exception('already initialized')
        self.port_num = int(self.port.split('-')[-1])
        self.bit_rate = int(self.bit_rate)
        if (voxl_i2c_init(self.port_num) != 0):
            raise Exception('could not initialize i2c port')
        self.read_buf_ptr = voxl_rpc_shared_mem_alloc(self.read_buf_size)
        self.initialized  = True

    def close(self):
        if not self.initialized:
            return

        voxl_i2c_close(self.port_num)

        if self.read_buf_ptr is not None:
            voxl_rpc_shared_mem_free(self.read_buf_ptr)
            self.read_buf_ptr = None

        voxl_rpc_shared_mem_deinit()
        self.initialized = False

    def slave_config(self, slave_address):
        if not self.initialized:
            raise Exception('port is not initialized')
        voxl_i2c_slave_config(self.port_num, slave_address, self.bit_rate, self.timeout_us)

    def read(self, register_address, address_num_bits, num_bytes_to_read):
        if not self.initialized:
            raise Exception('port is not initialized')

        if num_bytes_to_read > self.read_buf_size:
            raise Exception('internal read buffer is too small')

        if (voxl_i2c_read(self.port_num, register_address, address_num_bits, self.read_buf_ptr, num_bytes_to_read) != 0):
            raise Exception('could not read data')

        data_array = ''.join([chr(i) for i in self.read_buf_ptr[0:num_bytes_to_read]]) #there should be a faster way to convert to string

        return data_array

    def write(self, register_address, address_num_bits, write_buffer):
        if not self.initialized:
            raise Exception('port is not initialized')

        data = np.array(write_buffer)
        data_ptr = data.ctypes.data_as(ct.POINTER(ct.c_uint8))
        if (voxl_i2c_write(self.port_num, register_address, address_num_bits, data_ptr, data.nbytes) != 0):
            raise Exception('could not write data')
