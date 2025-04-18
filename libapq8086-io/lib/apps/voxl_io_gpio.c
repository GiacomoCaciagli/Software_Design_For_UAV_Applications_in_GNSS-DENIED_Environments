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
 * @file voxl_io_gpio.c
 *
 * Apps-proc code for GPIO functions of libvoxl_io. These are mostly wrappers
 * for the SDSP RPC calls in io_rpc.idl
 */

#include <stdio.h>
#include <string.h>     // for memset
#include <voxl_io.h>    // user-facing function declarations defined in this file
#include <io_rpc.h>     // sdsp rpc calls from idl file

static int check_gpio_pin_num(int32_t gpio_pin, const char * calling_func)
{
    if(gpio_pin<0 || gpio_pin>=MAX_NUM_GPIO_PINS){
       fprintf(stderr, "ERROR in %s: gpio pin must be between 0 & %d\n", calling_func, MAX_NUM_GPIO_PINS-1);
       return -1;
    }
    return 0;
}

int voxl_gpio_init(int gpio_pin, int direction, int flags)
{
    //flags input is reserved for future use

    // sanity checks
    if (check_gpio_pin_num(gpio_pin,__func__))
        return -1;

    if (direction == GPIO_DIRECTION_INPUT){
        if(io_rpc_gpio_init_input(gpio_pin)){
            fprintf(stderr, "io_rpc_gpio_init_input() failed\n");
            return -1;
        }
    }
    else if (direction == GPIO_DIRECTION_OUTPUT){
        if(io_rpc_gpio_init_output(gpio_pin)){
            fprintf(stderr, "io_rpc_gpio_init_output() failed\n");
            return -1;
        }
    }
    else{
        fprintf(stderr,"ERROR in voxl_gpio_init, unknown pin direction : %d\n", direction);
        return -1;
    }

    return 0;
}

int voxl_gpio_read(int gpio_pin, int * gpio_state_ptr)
{
    // sanity checks
    if (check_gpio_pin_num(gpio_pin,__func__))
        return -1;

    if(io_rpc_gpio_read(gpio_pin, gpio_state_ptr)){
        fprintf(stderr, "io_rpc_gpio_read() failed\n");
        return -1;
    }

    return 0;
}
int voxl_gpio_write(int gpio_pin, int gpio_state)
{
    // sanity checks
    if (check_gpio_pin_num(gpio_pin,__func__))
        return -1;

    if ((gpio_state != 0) && (gpio_state != 1)){
        fprintf(stderr, "ERROR in voxl_gpio_write: invalid pin value : %d\n",gpio_state);
        return -1;
    }

    if(io_rpc_gpio_write(gpio_pin, gpio_state)){
        fprintf(stderr, "io_rpc_gpio_write() failed\n");
        return -1;
    }

    return 0;
}

int voxl_gpio_close(int gpio_pin)
{
    // sanity checks
    if (check_gpio_pin_num(gpio_pin,__func__))
        return -1;

    if(io_rpc_gpio_close(gpio_pin)){
        fprintf(stderr, "io_rpc_gpio_close() failed\n");
        return -1;
    }

    return 0;
}
