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

#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <voxl_io.h>

static void print_usage(const char * prog_name)
{
    printf("Usage:\n");
    printf("%s read <pin_number>\n",prog_name);
    printf("%s write <pin_number> <0,1>\n",prog_name);
}

int read_pin(int gpio_pin)
{
    int ret;

    //initialize the pin to input
    if ( (ret = voxl_gpio_init(gpio_pin,GPIO_DIRECTION_INPUT,0)) )
    {
        fprintf(stderr, "ERROR initializing input pin : %d\n",gpio_pin);
        return -1;
    }

    int read_value =0;
    if (voxl_gpio_read(gpio_pin,&read_value)){
        fprintf(stderr, "ERROR reading from pin %d\n",gpio_pin);
        return -1;
    }

    voxl_gpio_close(gpio_pin);

    printf("%d\n",read_value);

    return 0;
}

int write_pin(int gpio_pin, int gpio_val)
{
    int ret;

    //initialize the pin to output
    if ( (ret = voxl_gpio_init(gpio_pin,GPIO_DIRECTION_OUTPUT,0)) ){
        fprintf(stderr, "ERROR initializing output pin : %d\n",gpio_pin);
        return -1;
    }

    if (voxl_gpio_write(gpio_pin,gpio_val)){
        fprintf(stderr, "ERROR writing to pin\n");
        return -1;
    }

    voxl_gpio_close(gpio_pin);

    return 0;
}

int main(int argc, char** argv)
{
    if (argc < 3){
        print_usage(argv[0]);
        return -1;
    }

    if (strcmp(argv[1],"read")==0)
    {
        int gpio_pin = atoi(argv[2]);

        //check pin number because atoi returns 0 if it can't parse an int
        if ((gpio_pin == 0) && (argv[2][0] != '0')){
            fprintf(stderr, "ERROR parsing pin number : %s\n",argv[2]);
            print_usage(argv[0]);
            return -1;
        }

        return read_pin(gpio_pin);
    }

    else if (strcmp(argv[1],"write")==0)
    {
        if (argc < 4){
            print_usage(argv[0]);
            return -1;
        }
        int gpio_pin = atoi(argv[2]);
        int gpio_val = atoi(argv[3]);

        //check pin number and value because atoi returns 0 if it can't parse an int
        if ((gpio_pin == 0) && (argv[2][0] != '0')){
            fprintf(stderr, "ERROR parsing pin number : %s\n",argv[2]);
            print_usage(argv[0]);
            return -1;
        }

        if ((gpio_val == 0) && (argv[3][0] != '0')){
            fprintf(stderr, "ERROR parsing pin value : %s\n",argv[3]);
            print_usage(argv[0]);
            return -1;
        }

        return write_pin(gpio_pin,gpio_val);
    }

    else{
        print_usage(argv[0]);
        return -1;
    }

    return 0;
}
