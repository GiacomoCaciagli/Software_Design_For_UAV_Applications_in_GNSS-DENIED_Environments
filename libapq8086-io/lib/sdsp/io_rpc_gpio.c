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
 * @file io_rpc_gpio.c
 *
 * SDSP code implementing the GPIO functions in io_rpc.idl
 */

#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <termios.h>
#include <HAP_farf.h>
#include <sys/ioctl.h>
#include <io_rpc.h>
#include "voxl_io.h"

#include "dev_fs_lib_gpio.h"

#define PRINT(...)  FARF(ALWAYS, __VA_ARGS__);

// file descriptors for all possible pins
// WARNING: this can take up some RAM (1KB for 256 pins?). Maybe do something more efficient..
static int gpio_fds[MAX_NUM_GPIO_PINS];

static int check_gpio_pin_num(int32_t gpio_pin, const char * calling_func)
{
    if(gpio_pin<0 || gpio_pin>=MAX_NUM_GPIO_PINS){
       PRINT("ERROR in %s: gpio pin must be between 0 & %d", calling_func, MAX_NUM_GPIO_PINS-1);
       return -1;
    }
    return 0;
}

int io_rpc_gpio_is_initialized(int32_t gpio_pin)
{
  if (check_gpio_pin_num(gpio_pin,__func__))
      return -1;

  if(gpio_fds[gpio_pin])
      return 1;
  else
      return 0;
}

int io_rpc_gpio_init_input(int32_t gpio_pin)
{
    if (check_gpio_pin_num(gpio_pin,__func__))
        return -1;

    if(gpio_fds[gpio_pin]){
        PRINT("WARNING in io_rpc_gpio_init_input, trying to initialize pin %d when already initialized", gpio_pin);
        return 0;
    }

    struct dspal_gpio_ioctl_config_io config = {
        .direction = DSPAL_GPIO_DIRECTION_INPUT,
        .pull  = DSPAL_GPIO_NO_PULL,   //TODO: need to double check whether pull-downs and pull-ups are working
        .drive = DSPAL_GPIO_2MA,       //does not matter for input
    };

    char device_path[16];
    snprintf(device_path,sizeof(device_path),"/dev/gpio-%d", (int)gpio_pin);
    int gpio_fd = open(device_path,0);

    if (gpio_fd == -1){
        PRINT("ERROR in io_rpc_gpio_init_input: could not open device %s",device_path);
        return -1;
    }

    if (ioctl(gpio_fd, DSPAL_GPIO_IOCTL_CONFIG_IO, (void*) &config) != 0) {
        PRINT("ERROR in io_rpc_gpio_init_input: gpio device ioctl failed");
        close(gpio_fd);
        return -1;
    }

    gpio_fds[gpio_pin] = gpio_fd;

    return 0;
}

int io_rpc_gpio_init_output(int32_t gpio_pin)
{
    if (check_gpio_pin_num(gpio_pin,__func__))
        return -1;

    if(gpio_fds[gpio_pin]){
        PRINT("WARNING in io_rpc_gpio_init_output, trying to initialize pin %d when already initialized", gpio_pin);
        return 0;
    }

    struct dspal_gpio_ioctl_config_io config = {
        .direction = DSPAL_GPIO_DIRECTION_OUTPUT,
        .pull  = DSPAL_GPIO_NO_PULL,
        .drive = DSPAL_GPIO_2MA,
    };

    char device_path[16];
    snprintf(device_path,sizeof(device_path),"/dev/gpio-%d", (int)gpio_pin);
    int gpio_fd = open(device_path,0);

    if (gpio_fd == -1){
        PRINT("ERROR in io_rpc_gpio_init_output: could not open device %s",device_path);
        return -1;
    }

    if (ioctl(gpio_fd, DSPAL_GPIO_IOCTL_CONFIG_IO, (void*) &config) != 0) {
        PRINT("ERROR in io_rpc_gpio_init_output: gpio device ioctl failed");
        close(gpio_fd);
        return -1;
    }

    gpio_fds[gpio_pin] = gpio_fd;

    return 0;
}

int io_rpc_gpio_read(int32_t gpio_pin, int32_t * gpio_state_ptr)
{
    if (check_gpio_pin_num(gpio_pin,__func__))
        return -1;

    if(gpio_fds[gpio_pin]==0){
        PRINT("ERROR in io_rpc_gpio_write, gpio pin %d must be initialized first", gpio_pin);
        return -1;
    }

    enum DSPAL_GPIO_VALUE_TYPE value_read;
    int ret = read(gpio_fds[gpio_pin], &value_read, 1);
    if (ret != 1) {
      PRINT("ERROR in io_rpc_gpio_write: read returned %d",ret);
      return -1;
    }

    if (value_read == DSPAL_GPIO_HIGH_VALUE)
        *gpio_state_ptr = 1;
    else
        *gpio_state_ptr = 0;

    return 0;
}

int io_rpc_gpio_write(int32_t gpio_pin, int32_t gpio_state)
{
    if (check_gpio_pin_num(gpio_pin,__func__))
        return -1;

    if(gpio_fds[gpio_pin]==0){
        PRINT("ERROR in io_rpc_gpio_write, gpio pin %d must be initialized first", gpio_pin);
        return -1;
    }

    enum DSPAL_GPIO_VALUE_TYPE value_write = DSPAL_GPIO_LOW_VALUE;
    if (gpio_state !=0)
        value_write = DSPAL_GPIO_HIGH_VALUE;

    int ret = write(gpio_fds[gpio_pin], &value_write, 1);

    if(ret != 1){
        PRINT("ERROR in io_rpc_gpio_write, write() returned %d", ret);
        return -1;
    }

    return 0;
}

int io_rpc_gpio_close(int32_t gpio_pin)
{
    if (check_gpio_pin_num(gpio_pin,__func__))
        return -1;

    // if not initialized already, return
    if(gpio_fds[gpio_pin]==0) return 0;

    close(gpio_fds[gpio_pin]);
    gpio_fds[gpio_pin]=0;
    return 0;
}
