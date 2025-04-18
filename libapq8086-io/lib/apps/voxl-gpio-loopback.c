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

void print_usage(const char * prog_name)
{
    printf("GPIO Loopback Test\n");
    printf("Usage:\n");
    printf("%s <output_pin> <input_pin>\n",prog_name);
    printf("  Output pin will be toggled high / low while reading input pin state.\n");
    printf("  If the two pins are externally connected, their state should match.\n");
}

int main(int argc, char** argv)
{
    if (argc != 3){
        print_usage(argv[0]);
        return -1;
    }

    int output_pin = atoi(argv[1]);
    int input_pin  = atoi(argv[2]);

    //check pin numbers because atoi returns 0 if it can't parse an int
    if ((output_pin == 0) && (argv[1][0] != '0')){
        fprintf(stderr, "ERROR parsing output pin number : %s\n",argv[1]);
        print_usage(argv[0]);
        return -1;
    }

    if ((input_pin == 0) && (argv[2][0] != '0')){
        fprintf(stderr, "ERROR parsing output pin number : %s\n",argv[2]);
        print_usage(argv[0]);
        return -1;
    }

    printf("Using output pin %d, input pin %d\n",output_pin,input_pin);

    int ret;

    if ( (ret = voxl_gpio_init(input_pin,GPIO_DIRECTION_INPUT,0)) )
    {
        fprintf(stderr, "ERROR initializing input pin\n");
        return -1;
    }

    if ( (ret = voxl_gpio_init(output_pin,GPIO_DIRECTION_OUTPUT,0)) )
    {
        fprintf(stderr, "ERROR initializing output pin\n");
        return -1;
    }

    int read_value =0;
    int write_value=0;

    //toggle the output pin between high and low and read the state of output in and input pin
    for (int ii=0; ii<10; ii++)
    {
        write_value ^= 1;
        if (voxl_gpio_write(output_pin,write_value)) fprintf(stderr, "ERROR writing to pin\n");
        else                                         printf("Pin %d set to   %d\n",output_pin,write_value);

        //reading state of output pin is allowed
        if (voxl_gpio_read(output_pin,&read_value))  fprintf(stderr, "ERROR reading from pin %d\n",output_pin);
        else                                         printf("Pin %d state is %d\n",output_pin, read_value);

        if (voxl_gpio_read(input_pin,&read_value))   fprintf(stderr, "ERROR reading from pin %d\n",input_pin);
        else                                         printf("Pin %d state is %d\n",input_pin, read_value);

        printf("\n");
        usleep(1000000);
    }

    voxl_gpio_close(input_pin);
    voxl_gpio_close(output_pin);

    return 0;
}
