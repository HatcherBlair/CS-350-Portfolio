/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>

/* Driver configuration */
#include "ti_drivers_config.h"

/* I2C Driver */
#include <ti/drivers/I2C.h>

/* UART2 Driver */
#include <ti/drivers/UART2.h>

size_t bytesWritten = 0;
#define DISPLAY(x) UART2_write(uart, &output, x, &bytesWritten);

/* Timer Driver */
#include <ti/drivers/Timer.h>


// I2C Global Variables
static const struct {
    uint8_t address;
    uint8_t resultReg;
    char *id;
} sensors[3] = {
                { 0x48, 0x0000, "11X" },
                { 0x49, 0x0000, "116" },
                 {0x41, 0x0001, "006" }
};

uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;

// Driver Handles - Global variables
I2C_Handle i2c;

// UART Global Variables
char output[64];
int bytesToSend;

// Temperature control Global variables
int16_t buttonValue = 21; // Temperature is initialized to 21 degrees c

// Elapsed time counter
unsigned long elapsedTime = 100000;
const unsigned long timerPeriod = 100000;

// variable for current temperature and set temperature
int16_t setTemp = 21; // Default setTemperature is 21 degrees
int16_t currentTemp = 0;

// Driver Handles - Global variables
UART2_Handle uart;

void initUART(void) {
    UART2_Params uartParams;

    // Configure the driver
    UART2_Params_init(&uartParams);
    uartParams.baudRate = 115200;

    uart = UART2_open(CONFIG_UART2_0, &uartParams);

    if (uart == NULL) {
        while (1) {}
    }
}

// Timer Driver Handles - Global variables
Timer_Handle timer0;

volatile unsigned char TimerFlag = 0;
void timerCallback (Timer_Handle myHandle, int_fast16_t status) {
    TimerFlag = 1;
}

void initTimer(void) {
    Timer_Params params;

    // Init the driver
    Timer_init();

    // Configure the driver
    Timer_Params_init(&params);
    params.period = 100000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    // Open the driver
    timer0 = Timer_open(CONFIG_TIMER_0, &params);

    if (timer0 == NULL) {
        while (1) {}
    }

    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        while(1) {}
    }
}

// initUART() needs to be called before this
void initI2C (void) {
    int8_t i, found;
    I2C_Params i2cParams;

    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "))

    // Init the driver
    I2C_init();

    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL) {
        DISPLAY(snprintf(output, 64, "Failed\n\r"))
        while (1);
    }

    DISPLAY(snprintf(output, 64, "Passed\n\r"))

    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;
    found = false;

    for (i=0; i <3; ++i) {
        i2cTransaction.targetAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;

        DISPLAY(snprintf(output, 64, "Is this %s?", sensors[i].id))

        if (I2C_transfer(i2c, &i2cTransaction)) {
            DISPLAY(snprintf(output, 64, "Found\n\r"))
            found = true;
            break;
        }
        DISPLAY(snprintf(output, 64, "No\n\r"))
    }

    if (found) {
        DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address: %x\n\r", sensors[i].id, i2cTransaction.targetAddress))
    } else {
        DISPLAY(snprintf(output, 64, "Temperature sensor not found, contact professor\n\r"))
    }
}

int16_t readTemp(void) {
    int j;
    int16_t temperature = 0;

    i2cTransaction.readCount = 2;
    if (I2C_transfer(i2c, &i2cTransaction)) {
        // Extract degrees C from the received data
        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
        temperature *= 0.0078125;

        // If the MSB is set 1 then we have a 2's complement negative value
        if (rxBuffer[0] & 0x80) {
            temperature |= 0xF000;
        }
    }  else {

        DISPLAY(snprintf(output, 64, "Error reading temperature sensor (%d)\n\r", i2cTransaction.status))
        DISPLAY(snprintf(output, 64, "Please power cycle the board\n\r"))
    }

    return temperature;
}

/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index)
{
    // Decrease the set temperature by 1 degree c
    buttonValue--;
}

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
 *  This may not be used for all boards.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn1(uint_least8_t index)
{
    // Increase the set temperature by 1 degree c
    buttonValue++;
}

//  Returns the buttonTemperature to update the set temperature and checks to make sure buttonTemp
//  is in range (called every 0.2s)
int updateSetTemperature() {
    if (buttonValue <= 10) {
        // Minimum temperature is 10 degrees
        buttonValue = 10;
        return buttonValue;
    } else if (buttonValue >= 30) {
        // Maximum temperature is 30 degrees
        buttonValue = 30;
        return buttonValue;
    } else {
        return buttonValue;
    }
}

// Updates the status of the furnace and sends message
// If the currentTemp is less than the setTemp, it will turn the LED on
// otherwise the LED is off
// the message is sent via UART and is in the format <currentTemp, setTemp, Heating(bool), Uptime>
void updateFurnace() {
    bool heatingFlag = false;
    if (currentTemp < setTemp) {
        // Furnace is heating
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
        heatingFlag = true;
    } else {
        // Furnace is off
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
    }

    DISPLAY(snprintf(output, 64, "<%02d,%02d,%d,%04lu>\n\r", currentTemp, setTemp, heatingFlag, elapsedTime / 1000000))
}


/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);


    /* Install Button callbacks */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);
    GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);
    GPIO_enableInt(CONFIG_GPIO_BUTTON_1);

    // Init UART, must be done before I2C init
    initUART();
    initI2C(); // Init I2C
    initTimer(); // Init Timer

    // Time counter for different tasks
    unsigned long buttonCycleTime = 100000;
    unsigned long tempCycleTime = 100000;
    unsigned long furnaceCycleTime = 100000;

    while (1) {

        // Wait for timer flag
        while (!TimerFlag) {}
        TimerFlag = 0;

        // Check for button inputs
        if (buttonCycleTime >= 200000) {
            // Every 200ms this runs
            buttonCycleTime = 0;
            setTemp = updateSetTemperature();
        }

        // Check for temperature update
        if (tempCycleTime >= 500000) {
            // Every 500ms this runs
            tempCycleTime = 0;
            currentTemp = readTemp();
        }

        // check for furnace update
        if (furnaceCycleTime >= 1000000) {
            // Every 1000ms this runs
            furnaceCycleTime = 0;
            updateFurnace();
        }

        elapsedTime += timerPeriod;
        buttonCycleTime += timerPeriod;
        tempCycleTime += timerPeriod;
        furnaceCycleTime += timerPeriod;

    }

    return (NULL);
}
