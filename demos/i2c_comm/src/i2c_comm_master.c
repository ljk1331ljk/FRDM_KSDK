/*
 * Copyright (c) 2013 - 2014, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

///////////////////////////////////////////////////////////////////////////////
//  Includes
///////////////////////////////////////////////////////////////////////////////

// Standard C Included Files
#include <string.h>
#include <math.h>
#include <stdio.h>

// SDK Included Files
#include "board.h"
#include "fsl_i2c_master_driver.h"

///////////////////////////////////////////////////////////////////////////////
// Definitions
///////////////////////////////////////////////////////////////////////////////

enum _subaddress_index_e
{
    Subaddress_Index_0 = 0x00,
    Subaddress_Index_1 = 0x01,
    Subaddress_Index_2 = 0x02,
    Subaddress_Index_3 = 0x03,
    Subaddress_Index_4 = 0x04,
    Subaddress_Index_5 = 0x05,
    Subaddress_Index_6 = 0x06,
    Subaddress_Index_7 = 0x07,
    Invalid_Subaddress_Index = 50, // set to 50==0x32
    Max_Subaddress_Index
};

#define CURRENT_SENSOR_ADD 0x40
i2c_master_state_t master;
i2c_status_t returnValue;

i2c_device_t slave =
{
//        .address = 0x1D,
    .address = CURRENT_SENSOR_ADD,	
    .baudRate_kbps = 100
};

///////////////////////////////////////////////////////////////////////////////
// Code
///////////////////////////////////////////////////////////////////////////////

static void LED_toggle_master(int color)
{
    switch(color){
	case 2: 
    	   GPIO_DRV_TogglePinOutput(kGpioLED2);
	   break;
	case 3: 
    	   GPIO_DRV_TogglePinOutput(kGpioLED3);
	   break;
	default:
	   GPIO_DRV_TogglePinOutput(kGpioLED1);
    }
}

/*!
 * @brief main function
 */

static void ina219_get_current(){

    uint8_t sendBuff[1] = {0};
    uint8_t receiveBuff[2]= {0};

    sendBuff[0] = 0x04;
    I2C_DRV_MasterReceiveDataBlocking(
	BOARD_I2C_COMM_INSTANCE,&slave,NULL,0,sendBuff,
	sizeof(sendBuff),500);

    OSA_TimeDelay(1);

    returnValue = I2C_DRV_MasterReceiveDataBlocking(
	BOARD_I2C_COMM_INSTANCE,&slave,NULL,0,receiveBuff,
	sizeof(receiveBuff),500);

    printf("\r\nCurrent reading: ");
    if (returnValue == kStatus_I2C_Success){
	OSA_TimeDelay(500);
	LED_toggle_master(2);
    	OSA_TimeDelay(500);
    	LED_toggle_master(2);//blink green
	printf("%d A",((uint16_t) receiveBuff[0]<<8)+receiveBuff[1]);
	    
    }else{
	OSA_TimeDelay(500);
	LED_toggle_master(1);
    	OSA_TimeDelay(500);
    	LED_toggle_master(1); //blink red
        printf("I2C communication failed, error code: %d", returnValue);
    }
}
static void ina219_get_power(){

    uint8_t sendBuff[1] = {0};
    uint8_t receiveBuff[2]= {0};

    sendBuff[0] = 0x03;
    I2C_DRV_MasterReceiveDataBlocking(
	BOARD_I2C_COMM_INSTANCE,&slave,NULL,0,sendBuff,
	sizeof(sendBuff),500);

    OSA_TimeDelay(1);

    returnValue = I2C_DRV_MasterReceiveDataBlocking(
	BOARD_I2C_COMM_INSTANCE,&slave,NULL,0,receiveBuff,
	sizeof(receiveBuff),500);

    printf("\r\nPower reading: ");
    if (returnValue == kStatus_I2C_Success){
	OSA_TimeDelay(500);
	LED_toggle_master(2);
    	OSA_TimeDelay(500);
    	LED_toggle_master(2);//blink green
	printf("%d A",((uint16_t) receiveBuff[0]<<8)+receiveBuff[1]);
	    
    }else{
	OSA_TimeDelay(500);
	LED_toggle_master(1);
    	OSA_TimeDelay(500);
    	LED_toggle_master(1); //blink red
        printf("I2C communication failed, error code: %d", returnValue);
    }
}
int main(void)
{
/*    
    uint8_t i;
    uint8_t index, indexChar, value;
    uint8_t cmdBuff[1] = {0xFF};
    uint8_t sendBuff[1] = {0xFF};       // save data sent to i2c slave
    uint8_t receiveBuff[1] = {0xFF};    // save data received from i2c slave

    i2c_master_state_t master;
    i2c_status_t returnValue;

    i2c_device_t slave =
    {
//        .address = 0x1D,
	.address = CURRENT_SENSOR_ADD,	
        .baudRate_kbps = 100
    };*/

    hardware_init();

    dbg_uart_init();

    // Configure I2C pins
    configure_i2c_pins(BOARD_I2C_COMM_INSTANCE);

    OSA_Init();

    GPIO_DRV_Init(0, ledPins);

    // Init I2C module
    I2C_DRV_MasterInit(BOARD_I2C_COMM_INSTANCE, &master);

    printf("\r\n====== I2C Master ======\r\n\r\n");

    OSA_TimeDelay(500);
    LED_toggle_master(1);
    OSA_TimeDelay(500);
    LED_toggle_master(1);
    OSA_TimeDelay(500);
    LED_toggle_master(2);
    OSA_TimeDelay(500);
    LED_toggle_master(2);
    OSA_TimeDelay(500);
    LED_toggle_master(3);
    OSA_TimeDelay(500);
    LED_toggle_master(3);

    while(1){
	/*
	printf("\r\nInput Register Address: ");
        indexChar = getchar();
        putchar(indexChar);
	sendBuff[0] = indexChar;
*/
	printf("\r\n\nReading Sensor");
	ina219_get_current();
	ina219_get_power();

	OSA_TimeDelay(1000);
    }
/*
    while (1)
    {
        printf("\r\nI2C Master reads values from I2C Slave sub address:\r\n");
        printf("\r\n------------------------------------");
        printf("\r\nSlave Sub Address   |    Character         ");
        printf("\r\n------------------------------------");
        for (i=Subaddress_Index_0; i<Invalid_Subaddress_Index; i++)
        {
            cmdBuff[0] = i;
            returnValue = I2C_DRV_MasterReceiveDataBlocking(
                                                       BOARD_I2C_COMM_INSTANCE,
                                                       &slave,
                                                       cmdBuff,
                                                       1,
                                                       receiveBuff,
                                                       sizeof(receiveBuff),
                                                       500);
            if (returnValue == kStatus_I2C_Success)
            {
                printf("\r\n[%d]                      %c", i, receiveBuff[0]);
            }
            else
            {
                printf("\r\nI2C communication failed, error code: %d", returnValue);
            }

        }
        printf("\r\n------------------------------------");
        printf("\r\n");

        printf("\r\nPlease input Slave sub address and the new character.");

        do
        {
            printf("\r\nSlave Sub Address: ");
            indexChar = getchar();
            putchar(indexChar);

            printf("\r\nInput New Character: ");
            value = getchar();
            putchar(value);

            printf("\n");

            index = (uint8_t)(indexChar - '0');

            if (index >= Invalid_Subaddress_Index)
            {
                printf("\r\nInvalid Sub Address.");
            }
        } while (index >= Invalid_Subaddress_Index);

        cmdBuff[0]  = index;
        sendBuff[0] = value;

        returnValue = I2C_DRV_MasterSendDataBlocking(
                                                    BOARD_I2C_COMM_INSTANCE,
                                                    &slave,
                                                    cmdBuff,
                                                    1,
                                                    sendBuff,
                                                    sizeof(sendBuff),
                                                    500);
        if (returnValue != kStatus_I2C_Success)
        {
            printf("\r\nI2C communication failed, error code: %d", returnValue);
        }
    }
*/
}
