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

from voxl_i2c import VoxlI2CPort
import time
import numpy as np

p          = VoxlI2CPort()
p.port     = '/dev/i2c-sdsp-8'
p.bit_rate = 400000
p.open()

#APM V2 has two INA231 devices .. ID 68 (battery monitoring) and 69 (5V regulator monitoring)
ID0 = 68
ID1 = 69

#values of configuration registers 0x00 and 0x01
#64x average, 140us sample time for voltage and current, continuous samplings. MSB first
CONFIG_REGS = np.uint8([int('0b01000110',2),int('0b00000111',2)])

#addresses of registers for reading data
REG_VRAW = np.uint8(2) #0x02
REG_IRAW = np.uint8(1) #0x01

#write configuration to both ina231 devices
p.slave_config(ID0)
p.write(0,8,CONFIG_REGS)

p.slave_config(ID1)
p.write(0,8,CONFIG_REGS)

while(1):
    p.slave_config(ID0)
    vraw_batt = p.read(REG_VRAW,8,2)
    iraw_batt = p.read(REG_IRAW,8,2)

    p.slave_config(ID1)
    vraw_reg = p.read(REG_VRAW,8,2)
    iraw_reg = p.read(REG_IRAW,8,2)

    voltage_raw_batt = np.uint16(ord(vraw_batt[0]) << 8) + (ord(vraw_batt[1]))   #16-bit data is MSB first
    current_raw_batt = np.int16(ord(iraw_batt[0]) << 8) + (ord(iraw_batt[1]))
    voltage_batt     = float(voltage_raw_batt) * 0.00125                #1.25mV resolution
    current_batt     = float(current_raw_batt) * 0.0000025 * 2000.0     #2.5uV per LSB, 0.0005 ohm current sensing resistor.. so x 2000.0

    voltage_raw_reg  = np.uint16(ord(vraw_reg[0]) << 8) + (ord(vraw_reg[1]))     #16-bit data is MSB first
    current_raw_reg  = np.int16(ord(iraw_reg[0]) << 8) + (ord(iraw_reg[1]))
    voltage_reg      = float(voltage_raw_reg) * 0.00125                 #1.25mV resolution
    current_reg      = float(current_raw_reg) * 0.0000025 * 200.0       #2.5uV per LSB, 0.005 ohm current sensing resistor.. so x 200.0

    #note that current measurement offset and scale may be off, needs calibration
    print('BATT| Voltage %2.2fV, Current %2.3fA.. REG| Voltage %2.2fV, Current %2.3fA..' % (voltage_batt, current_batt, voltage_reg, current_reg))

    time.sleep(0.05)

p.close()
