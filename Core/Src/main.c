/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
    STATUS_NORMAL,
    STATUS_WARNING,
    STATUS_ERROR
} SystemStatus;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint32_t adcValue = 0;
float voltage = 0.0f;
char txBuffer[160];
int16_t encoderCount = 0;
float currentZeroVoltage = 0.0f;   // Voltage when the motor is not using current
float motorCurrent = 0.0f;         // Motor current in amps
float currentAverageADC = 0.0f;    // Average ADC reading while measuring current
float currentZeroADC = 0.0f;       // Average ADC reading when the current is 0 A
float motorTemperature = 0.0f;
uint8_t adxl345DeviceID = 0;
uint8_t adxl345PowerControl = 0x08;
int16_t adxlX = 0;
int16_t adxlY = 0;
int16_t adxlZ = 0;
uint8_t adxlData[6];
float accelX_g = 0.0f;
float accelY_g = 0.0f;
float accelZ_g = 0.0f;
float totalAccel_g = 0.0f;
float vibration_g = 0.0f;
float vibrationRMS_g = 0.0f;
uint8_t adxlDataFormat = 0;
uint8_t adxlBWRate = 0;
uint8_t adxlIntSource = 0;
SystemStatus systemStatus = STATUS_NORMAL;
// Actuator position data
int16_t commandedPosition = 0;
int16_t actualPosition = 0;
int16_t positionError = 0;
uint32_t movementTime_ms = 0;
char rxBuffer[50];
char commandBuffer[50];
uint8_t rxByte;
uint8_t rxIndex = 0;
volatile uint8_t commandReady = 0;
volatile uint8_t stopRequested = 0;
uint16_t modbusRegister = 0;
uint8_t lastDS18B20Raw[9] = {0};
uint16_t heartbeatCounter = 0;
uint32_t lastHeartbeatTick = 0;
static uint16_t targetPosition = 0;
static uint16_t commandCode = 0;
static uint16_t commandSequence = 0;
static uint16_t acknowledgedSequence = 0;
static uint16_t motionStatus = 0; // 0 = idle; dry-run only
static uint8_t motionActive = 0;
static int16_t motionTarget = 0;
static uint32_t motionStartTick = 0;
static int16_t motionLastPosition = 0;
static uint32_t motionLastProgressTick = 0;
// Keep physical motor output disabled during software integration
static const uint8_t motionOutputEnabled = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM5_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void I2C1_BusRecovery(void);
void I2C1_TestSDA(void);
void PB9_OutputTest(void);
void PrintSystemData(void);
void SendSystemData(void);
void ProcessCommand(char *command);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Reset the W5500 Ethernet module
void W5500_Reset(void)
{
    // Pull RSTn low
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);

    HAL_Delay(10);

    // Release RSTn
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);

    // Wait for W5500 to restart
    HAL_Delay(100);
}


// Test SPI communication with W5500
void W5500_Test(void)
{
    uint8_t txData[4] = {0x00, 0x39, 0x00, 0xFF};
    uint8_t rxData[4] = {0};

    HAL_StatusTypeDef spiStatus;

    // Select W5500 (CS = PB0)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

    // Read W5500 version register
    spiStatus = HAL_SPI_TransmitReceive(&hspi1,
                                       txData,
                                       rxData,
                                       4,
                                       100);

    // Release W5500
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

    // Print SPI result
    snprintf(txBuffer, sizeof(txBuffer),
             "W5500 SPI Status:%d Version:0x%02X\r\n",
             spiStatus, rxData[3]);

    HAL_UART_Transmit(&huart2,
                      (uint8_t *)txBuffer,
                      strlen(txBuffer),
                      100);

    // Check W5500 version
    if (spiStatus == HAL_OK && rxData[3] == 0x04)
    {
        snprintf(txBuffer, sizeof(txBuffer),
                 "W5500 SPI TEST PASSED\r\n");
    }
    else
    {
        snprintf(txBuffer, sizeof(txBuffer),
                 "W5500 SPI TEST FAILED\r\n");
    }

    HAL_UART_Transmit(&huart2,
                      (uint8_t *)txBuffer,
                      strlen(txBuffer),
                      100);
}


void W5500_WriteRegister(uint16_t address, uint8_t data)
{
    uint8_t txData[4];

    txData[0] = (uint8_t)(address >> 8);
    txData[1] = (uint8_t)(address & 0xFF);
    txData[2] = 0x04;
    txData[3] = data;

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

    HAL_StatusTypeDef status =
        HAL_SPI_Transmit(&hspi1, txData, 4, 100);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

    // Print errors only
    if (status != HAL_OK)
    {
        snprintf(txBuffer, sizeof(txBuffer),
                 "SPI WRITE ERROR | Address:0x%04X Status:%d\r\n",
                 address, status);

        HAL_UART_Transmit(&huart2,
                          (uint8_t *)txBuffer,
                          strlen(txBuffer),
                          100);
    }
}


// Read one byte from a W5500 common register
uint8_t W5500_ReadRegister(uint16_t address)
{
    uint8_t txData[4];
    uint8_t rxData[4] = {0};

    txData[0] = (uint8_t)(address >> 8);
    txData[1] = (uint8_t)(address & 0xFF);
    txData[2] = 0x00;  // Common register, read, VDM
    txData[3] = 0x00;

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

    HAL_SPI_TransmitReceive(&hspi1, txData, rxData, 4, 100);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

    return rxData[3];
}


void W5500_RegisterTest(void)
{
    uint8_t before;
    uint8_t after;
    uint8_t versionBefore;
    uint8_t versionAfter;

    // Read W5500 version before the test
    versionBefore = W5500_ReadRegister(0x0039);

    // Read current IP last byte
    before = W5500_ReadRegister(0x000F + 3);

    // Write IP last byte
    W5500_WriteRegister(0x000F + 3, 50);

    HAL_Delay(10);

    // Read back the values
    after = W5500_ReadRegister(0x000F + 3);
    versionAfter = W5500_ReadRegister(0x0039);

    snprintf(txBuffer, sizeof(txBuffer),
             "SPI TEST | Version:%02X->%02X IP:%u->%u\r\n",
             versionBefore,
             versionAfter,
             before,
             after);

    HAL_UART_Transmit(&huart2,
                      (uint8_t *)txBuffer,
                      strlen(txBuffer),
                      100);
}


void W5500_NetworkInit(void)
{
    // MAC address: 02:00:00:00:00:01
    uint8_t mac[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};

    // IP address: 192.168.1.50
    uint8_t ip[4] = {192, 168, 1, 50};

    // Subnet mask: 255.255.255.0
    uint8_t subnet[4] = {255, 255, 255, 0};

    // Gateway: 192.168.1.1
    uint8_t gateway[4] = {192, 168, 1, 1};

    // Set gateway
    for (int i = 0; i < 4; i++)
    {
        W5500_WriteRegister(0x0001 + i, gateway[i]);
    }

    // Set subnet mask
    for (int i = 0; i < 4; i++)
    {
        W5500_WriteRegister(0x0005 + i, subnet[i]);
    }

    // Set MAC address
    for (int i = 0; i < 6; i++)
    {
        W5500_WriteRegister(0x0009 + i, mac[i]);
    }

    // Set IP address
    for (int i = 0; i < 4; i++)
    {
        W5500_WriteRegister(0x000F + i, ip[i]);
        HAL_Delay(1);
    }

    // Check the last IP byte
    uint8_t lastIP = W5500_ReadRegister(0x0012);

    snprintf(txBuffer, sizeof(txBuffer),
             "IP Last Byte Before Fix: %u\r\n", lastIP);

    HAL_UART_Transmit(&huart2,
                      (uint8_t *)txBuffer,
                      strlen(txBuffer),
                      100);

    // Try writing the last IP byte again
    if (lastIP != 50)
    {
        W5500_WriteRegister(0x0012, 50);
        HAL_Delay(10);
    }

    // Read it again to check the result
    lastIP = W5500_ReadRegister(0x0012);

    snprintf(txBuffer, sizeof(txBuffer),
             "IP Last Byte After Fix: %u\r\n", lastIP);

    HAL_UART_Transmit(&huart2,
                      (uint8_t *)txBuffer,
                      strlen(txBuffer),
                      100);

    // Read back the IP address
    uint8_t readIP[4];

    for (int i = 0; i < 4; i++)
    {
        readIP[i] = W5500_ReadRegister(0x000F + i);
    }

    // Print the IP address
    snprintf(txBuffer, sizeof(txBuffer),
             "W5500 IP: %u.%u.%u.%u\r\n",
             readIP[0], readIP[1], readIP[2], readIP[3]);

    HAL_UART_Transmit(&huart2,
                      (uint8_t *)txBuffer,
                      strlen(txBuffer),
                      100);

    // Check if the IP address is correct
    if (readIP[0] == 192 &&
        readIP[1] == 168 &&
        readIP[2] == 1 &&
        readIP[3] == 50)
    {
        snprintf(txBuffer, sizeof(txBuffer),
                 "W5500 Network Init PASSED\r\n");
    }
    else
    {
        snprintf(txBuffer, sizeof(txBuffer),
                 "W5500 Network Init FAILED\r\n");
    }

    HAL_UART_Transmit(&huart2,
                      (uint8_t *)txBuffer,
                      strlen(txBuffer),
                      100);
}


void W5500_CheckLink(void)
{
    uint8_t phyStatus = W5500_ReadRegister(0x002E);

    // Check Ethernet connection
    if (phyStatus & 0x01)
    {
        // Ethernet connected
    }
    else
    {
        // Ethernet disconnected
    }
}


// Read one byte from a W5500 socket register
uint8_t W5500_ReadSocketRegister(uint16_t address)
{
    uint8_t txData[4];
    uint8_t rxData[4] = {0};

    txData[0] = (uint8_t)(address >> 8);
    txData[1] = (uint8_t)(address & 0xFF);
    txData[2] = 0x08;  // Socket 0 register, read
    txData[3] = 0x00;

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

    HAL_SPI_TransmitReceive(&hspi1, txData, rxData, 4, 100);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

    return rxData[3];
}


// Read data from W5500 Socket 0 RX buffer
void W5500_ReadRXBuffer(uint16_t address, uint8_t *buffer, uint16_t length)
{
    uint8_t header[3];

    header[0] = (uint8_t)(address >> 8);
    header[1] = (uint8_t)(address & 0xFF);
    header[2] = 0x18;  // Socket 0 RX buffer, read

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

    HAL_SPI_Transmit(&hspi1, header, 3, 100);
    HAL_SPI_Receive(&hspi1, buffer, length, 100);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
}


// Write data to W5500 Socket 0 TX buffer
void W5500_WriteTXBuffer(uint16_t address, uint8_t *buffer, uint16_t length)
{
    uint8_t header[3];

    header[0] = (uint8_t)(address >> 8);
    header[1] = (uint8_t)(address & 0xFF);
    header[2] = 0x14;  // Socket 0 TX buffer, write

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

    HAL_SPI_Transmit(&hspi1, header, 3, 100);
    HAL_SPI_Transmit(&hspi1, buffer, length, 100);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
}


void W5500_ReadTXBuffer(uint16_t address, uint8_t *buffer, uint16_t length)
{
    uint8_t header[3];

    header[0] = (uint8_t)(address >> 8);
    header[1] = (uint8_t)(address & 0xFF);
    header[2] = 0x10;  // Socket 0 TX buffer, read

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

    HAL_SPI_Transmit(&hspi1, header, 3, 100);
    HAL_SPI_Receive(&hspi1, buffer, length, 100);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
}


// Write one byte to a W5500 socket register
void W5500_WriteSocketRegister(uint16_t address, uint8_t data)
{
    uint8_t txData[4];

    txData[0] = (uint8_t)(address >> 8);
    txData[1] = (uint8_t)(address & 0xFF);
    txData[2] = 0x0C;  // Socket 0 register, write
    txData[3] = data;

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

    HAL_SPI_Transmit(&hspi1, txData, 4, 100);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
}

// Send a command to Socket 0
uint8_t W5500_SocketCommand(uint8_t command)
{
    W5500_WriteSocketRegister(0x0001, command);

    // Wait until W5500 finishes the command
    for (int i = 0; i < 100; i++)
    {
        if (W5500_ReadSocketRegister(0x0001) == 0)
        {
            return 1;
        }

        HAL_Delay(1);
    }

    return 0;
}


// Configure Socket 0 as a TCP server on port 502
void W5500_TCPServerInit(void)
{
    uint8_t socketStatus;

    // Close Socket 0 before configuring it
    W5500_SocketCommand(0x10);

    // TCP mode
    W5500_WriteSocketRegister(0x0000, 0x01);

    // Source port = 502 = 0x01F6
    W5500_WriteSocketRegister(0x0004, 0x01);
    W5500_WriteSocketRegister(0x0005, 0xF6);

    // Open the TCP socket
    if (!W5500_SocketCommand(0x01))
    {
        snprintf(txBuffer, sizeof(txBuffer),
                 "TCP OPEN command timeout\r\n");

        HAL_UART_Transmit(&huart2, (uint8_t *)txBuffer,
                          strlen(txBuffer), 100);
        return;
    }

    socketStatus = W5500_ReadSocketRegister(0x0003);

    if (socketStatus != 0x13)
    {
        snprintf(txBuffer, sizeof(txBuffer),
                 "TCP OPEN FAILED | Status:0x%02X\r\n",
                 socketStatus);

        HAL_UART_Transmit(&huart2, (uint8_t *)txBuffer,
                          strlen(txBuffer), 100);
        return;
    }

    // Start listening for incoming TCP connections
    if (!W5500_SocketCommand(0x02))
    {
        snprintf(txBuffer, sizeof(txBuffer),
                 "TCP LISTEN command timeout\r\n");

        HAL_UART_Transmit(&huart2, (uint8_t *)txBuffer,
                          strlen(txBuffer), 100);
        return;
    }

    socketStatus = W5500_ReadSocketRegister(0x0003);

    snprintf(txBuffer, sizeof(txBuffer),
             "TCP Server Port:502 | Status:0x%02X\r\n",
             socketStatus);

    HAL_UART_Transmit(&huart2, (uint8_t *)txBuffer,
                      strlen(txBuffer), 100);
}


/* PLC FC16: 400011..400013 -> command, target, sequence.
 * PLC FC03: 400014..400015 -> ack sequence, motion status.
 * No motor GPIO/PWM output is changed by this code.
 */
void W5500_TCPServerTask(void)
{
    uint8_t socketStatus = W5500_ReadSocketRegister(0x0003);

    if (socketStatus == 0x17)
    {
        /* Process up to four complete requests per call. TCP can split or
           combine Modbus frames, so never assume one recv == one request. */
        for (uint8_t frame = 0; frame < 4; frame++)
        {
        	uint16_t rxSize =
                ((uint16_t)W5500_ReadSocketRegister(0x0026) << 8) |
                W5500_ReadSocketRegister(0x0027);
            if (rxSize < 7) break;

        	if (rxSize > 0)
        	{
        	    printf("TCP RX Available: %u\r\n", rxSize);
        	}

            uint16_t rxPointer =
                ((uint16_t)W5500_ReadSocketRegister(0x0028) << 8) |
                W5500_ReadSocketRegister(0x0029);
            uint8_t header[7];
            W5500_ReadRXBuffer(rxPointer, header, 7);
            uint16_t mbapLength = ((uint16_t)header[4] << 8) | header[5];
            uint32_t requestLength = (uint32_t)mbapLength + 6U;

            /* The only supported requests are 12-byte FC03/FC06 and
               19-byte FC16 (three registers). Reject malformed framing
               by closing the socket; reconnect is handled below. */
            if (header[2] != 0 || header[3] != 0 ||
                mbapLength < 2 || requestLength > 260)
            {
                W5500_SocketCommand(0x10); /* CLOSE */
                break;
            }
            if (rxSize < requestLength) break; /* Wait for rest of TCP frame */

            uint8_t rxBuffer[260] = {0};
            W5500_ReadRXBuffer(rxPointer, rxBuffer, (uint16_t)requestLength);
            rxPointer = (uint16_t)(rxPointer + requestLength);
            W5500_WriteSocketRegister(0x0028, (uint8_t)(rxPointer >> 8));
            W5500_WriteSocketRegister(0x0029, (uint8_t)rxPointer);
            W5500_SocketCommand(0x40); /* RECV */

            if (rxBuffer[6] != 1) continue; /* Unit ID 1 */

            uint8_t response[260] = {0};
            uint16_t responseLength = 0;
            uint8_t function = rxBuffer[7];

            char debugMsg[50];

            snprintf(debugMsg, sizeof(debugMsg),
                     "Modbus Function: 0x%02X\r\n", function);

            HAL_UART_Transmit(&huart2,
                              (uint8_t *)debugMsg,
                              strlen(debugMsg),
                              100);
            uint16_t address = 0;
            uint16_t quantity = 0;
            if (requestLength >= 12)
            {
                address = ((uint16_t)rxBuffer[8] << 8) | rxBuffer[9];
                quantity = ((uint16_t)rxBuffer[10] << 8) | rxBuffer[11];
            }

            /* Preserve transaction ID, protocol ID and Unit ID. */
            response[0] = rxBuffer[0];
            response[1] = rxBuffer[1];
            response[6] = rxBuffer[6];

            if (function == 0x06 && requestLength == 12 && address == 0)
            {
                modbusRegister = quantity; /* FC06 value */
                memcpy(response, rxBuffer, 12);
                responseLength = 12;
            }
            else if (function == 0x03 && requestLength == 12 &&
                     quantity >= 1 && quantity <= 4 &&
                     ((address < 4 && (uint32_t)address + quantity <= 4) ||
                      (address >= 13 && (uint32_t)address + quantity <= 15)))
            {
                responseLength = (uint16_t)(9 + quantity * 2);
                uint16_t lengthField = (uint16_t)(3 + quantity * 2);
                response[4] = (uint8_t)(lengthField >> 8);
                response[5] = (uint8_t)lengthField;
                response[7] = 0x03;
                response[8] = (uint8_t)(quantity * 2);
                for (uint16_t i = 0; i < quantity; i++)
                {
                    uint16_t value = 0;
                    switch (address + i)
                    {
                        case 0: value = modbusRegister; break;
                        case 1: value = (uint16_t)(motorTemperature * 100.0f); break;
                        case 2: value = (uint16_t)(vibrationRMS_g * 1000.0f); break;
                        case 3: value = heartbeatCounter; break;
                        case 13: value = acknowledgedSequence; break;
                        case 14: value = motionStatus; break;
                        default: break;
                    }
                    response[9 + i * 2] = (uint8_t)(value >> 8);
                    response[10 + i * 2] = (uint8_t)value;
                }
            }
            else if (function == 0x10 && requestLength == 19 &&
            		 address == 10 && quantity == 3 && rxBuffer[12] == 6)
            {
                uint16_t newCommand = ((uint16_t)rxBuffer[13] << 8) | rxBuffer[14];
                uint16_t newTarget = ((uint16_t)rxBuffer[15] << 8) | rxBuffer[16];
                uint16_t newSequence = ((uint16_t)rxBuffer[17] << 8) | rxBuffer[18];

                if (newTarget <= 1000 && newCommand <= 2)
                {
                    targetPosition = newTarget;
                    commandCode = newCommand;
                    commandSequence = newSequence;
                    /* Communication-only ACK: not proof of motor movement. */
                    acknowledgedSequence = newSequence;
                    motionStatus = 0;
                    response[4] = 0;
                    response[5] = 6;
                    response[7] = 0x10;
                    response[8] = rxBuffer[8];
                    response[9] = rxBuffer[9];
                    response[10] = rxBuffer[10];
                    response[11] = rxBuffer[11];
                    responseLength = 12;
                }
                else
                {
                    response[4] = 0;
                    response[5] = 3;
                    response[7] = 0x90;
                    response[8] = 0x03; /* Illegal data value */
                    responseLength = 9;
                }
            }
            else
            {
                response[4] = 0;
                response[5] = 3;
                response[7] = (uint8_t)(function | 0x80);
                response[8] = 0x02; /* Illegal data address */
                responseLength = 9;
            }

            uint16_t txFree =
                ((uint16_t)W5500_ReadSocketRegister(0x0020) << 8) |
                W5500_ReadSocketRegister(0x0021);
            if (txFree < responseLength) break;
            uint16_t txPointer =
                ((uint16_t)W5500_ReadSocketRegister(0x0024) << 8) |
                W5500_ReadSocketRegister(0x0025);
            W5500_WriteTXBuffer(txPointer, response, responseLength);
            txPointer = (uint16_t)(txPointer + responseLength);
            W5500_WriteSocketRegister(0x0024, (uint8_t)(txPointer >> 8));
            W5500_WriteSocketRegister(0x0025, (uint8_t)txPointer);
            W5500_SocketCommand(0x20); /* SEND */
        }
    }
    else if (socketStatus == 0x00 || socketStatus == 0x1C)
    {
        W5500_TCPServerInit();
    }
}


// Test if PB9 can be released high
void PB9_OutputTest(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    // Stop I2C from controlling PB9
    HAL_I2C_DeInit(&hi2c1);
    // Use PB9 as an open-drain output
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    // Release PB9
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    HAL_Delay(10);
    snprintf(txBuffer, sizeof(txBuffer),
             "PB9 Released: %d\r\n",
             HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9));
    HAL_UART_Transmit(&huart2,
                      (uint8_t *)txBuffer,
                      strlen(txBuffer),
                      100);
}


// Wait for a given number of microseconds
void delay_us(uint32_t us)
{
    // Start counting from 0
    __HAL_TIM_SET_COUNTER(&htim5, 0);
    // Keep waiting until the requested time has passed
    while (__HAL_TIM_GET_COUNTER(&htim5) < us)
    {
    }
}


// Reset the DS18B20 and check if it responds
uint8_t DS18B20_Reset(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    uint8_t presence = 1;
    // Use open-drain output
    GPIO_InitStruct.Pin = DS18B20_DATA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DS18B20_DATA_GPIO_Port, &GPIO_InitStruct);
    // Pull DATA low
    HAL_GPIO_WritePin(DS18B20_DATA_GPIO_Port,
                      DS18B20_DATA_Pin,
                      GPIO_PIN_RESET);
    delay_us(600);
    // Release DATA
    HAL_GPIO_WritePin(DS18B20_DATA_GPIO_Port,
                      DS18B20_DATA_Pin,
                      GPIO_PIN_SET);
    // Listen for the sensor
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DS18B20_DATA_GPIO_Port, &GPIO_InitStruct);
    // Check if the sensor pulls DATA low
    for (uint32_t i = 0; i < 200; i++)
    {
        if (HAL_GPIO_ReadPin(DS18B20_DATA_GPIO_Port,
                            DS18B20_DATA_Pin) == GPIO_PIN_RESET)
        {
            presence = 0;
            break;
        }
        delay_us(1);
    }
    // Wait for the sensor to finish
    delay_us(300);
    // Change DATA back to open-drain output
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DS18B20_DATA_GPIO_Port, &GPIO_InitStruct);
    // Release DATA
    HAL_GPIO_WritePin(DS18B20_DATA_GPIO_Port,
                      DS18B20_DATA_Pin,
                      GPIO_PIN_SET);
    return presence;
}


// Send one bit to the DS18B20
// The duration of LOW varies. Shorter → 1 | Longer → 0
void DS18B20_WriteBit(uint8_t bit)
{
    // Pull DATA low to start one write time slot
    HAL_GPIO_WritePin(DS18B20_DATA_GPIO_Port,
                      DS18B20_DATA_Pin,
                      GPIO_PIN_RESET);
    if (bit == 1)
    {
        // A short LOW means bit 1
        delay_us(5);
        // Release DATA
        HAL_GPIO_WritePin(DS18B20_DATA_GPIO_Port,
                          DS18B20_DATA_Pin,
                          GPIO_PIN_SET);
        // Wait for the rest of the time slot
        delay_us(60);
    }
    else
    {
        // A long LOW means bit 0
        delay_us(60);
        // Release DATA
        HAL_GPIO_WritePin(DS18B20_DATA_GPIO_Port,
                          DS18B20_DATA_Pin,
                          GPIO_PIN_SET);
        // Give the bus a short recovery time
        delay_us(10);
    }
}


// Send one byte to the DS18B20
void DS18B20_WriteByte(uint8_t data)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        // Send the lowest bit first
        DS18B20_WriteBit(data & 0x01);
        // Move the next bit to the lowest position
        data >>= 1;
    }
}


// Read one bit from the DS18B20
uint8_t DS18B20_ReadBit(void)
{
    uint8_t bit;
    // Pull DATA low to start one read time slot
    HAL_GPIO_WritePin(DS18B20_DATA_GPIO_Port,
                      DS18B20_DATA_Pin,
                      GPIO_PIN_RESET);
    delay_us(2);
    // Release DATA so the DS18B20 can control the line
    HAL_GPIO_WritePin(DS18B20_DATA_GPIO_Port,
                      DS18B20_DATA_Pin,
                      GPIO_PIN_SET);
    // Change PC0 to input so STM32 can listen
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DS18B20_DATA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DS18B20_DATA_GPIO_Port,
                  &GPIO_InitStruct);
    // Wait before reading the bit
    delay_us(10);
    // Read the bit sent by the DS18B20
    bit = HAL_GPIO_ReadPin(DS18B20_DATA_GPIO_Port,
                           DS18B20_DATA_Pin);
    // Wait for this read time slot to finish
    delay_us(50);
    // Change PC0 back to open-drain output
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DS18B20_DATA_GPIO_Port,
                  &GPIO_InitStruct);
    // Release DATA
    HAL_GPIO_WritePin(DS18B20_DATA_GPIO_Port,
                      DS18B20_DATA_Pin,
                      GPIO_PIN_SET);
    return bit;
}


// Read one byte from the DS18B20
uint8_t DS18B20_ReadByte(void)
{
    uint8_t data = 0;
    // Read 8 bits to make one byte
    for (uint8_t i = 0; i < 8; i++)
    {
        // Read one bit from the DS18B20
        uint8_t bit = DS18B20_ReadBit();
        // Move this bit to the correct position
        bit = bit << i;
        // Add this bit to the byte
        data = data | bit;
    }
    return data;
}


// Check if the DS18B20 data is correct
uint8_t DS18B20_CalculateCRC(uint8_t *data, uint8_t length)
{
    uint8_t crc = 0;
    // Check each byte
    for (uint8_t i = 0; i < length; i++)
    {
        uint8_t currentByte = data[i];
        // Check each bit in this byte
        for (uint8_t bit = 0; bit < 8; bit++)
        {
            // Compare the lowest bits
            uint8_t mix = (crc ^ currentByte) & 0x01;
            // Move to the next bit
            crc >>= 1;
            // Update the CRC if the bits are different
            if (mix != 0)
            {
                crc ^= 0x8C;
            }
            // Move to the next data bit
            currentByte >>= 1;
        }
    }
    // Return the calculated CRC
    return crc;
}


// Read the temperature from the DS18B20
float DS18B20_ReadTemperature(void)
{
    uint8_t temperatureLowByte;
    uint8_t temperatureHighByte;
    int16_t rawTemperature;
    float temperatureC;
    uint8_t scratchpad[9];
    if (DS18B20_Reset() != 0)
    {
        // Read the DATA line after the failed reset
        GPIO_PinState dataState =
            HAL_GPIO_ReadPin(DS18B20_DATA_GPIO_Port,
                             DS18B20_DATA_Pin);
        // Show the error and DATA state
        snprintf(txBuffer, sizeof(txBuffer),
                 "DS18B20 ERROR: First reset failed | DATA:%d\r\n",
                 dataState);
        HAL_UART_Transmit(&huart2,
                          (uint8_t *)txBuffer,
                          strlen(txBuffer),
                          100);
        return -1000.0f;
    }
    // Use the only DS18B20 on the bus
    DS18B20_WriteByte(0xCC);
    // Start temperature conversion
    DS18B20_WriteByte(0x44);
    // Wait for the measurement to finish
    HAL_Delay(800);
    // Check if the sensor responds again
    if (DS18B20_Reset() != 0)
    {
        // Show the error
        snprintf(txBuffer, sizeof(txBuffer),
                 "DS18B20 ERROR: Second reset failed\r\n");
        HAL_UART_Transmit(&huart2,
                          (uint8_t *)txBuffer,
                          strlen(txBuffer),
                          100);
        return -1000.0f;
    }
    // Use the only DS18B20 on the bus
    DS18B20_WriteByte(0xCC);
    // Ask for the stored temperature data
    DS18B20_WriteByte(0xBE);
    // Read all 9 bytes from the scratchpad
    for (uint8_t i = 0; i < 9; i++)
    {
        scratchpad[i] = DS18B20_ReadByte();
    }
    memcpy(lastDS18B20Raw, scratchpad, 9);

    // Get the temperature bytes
    temperatureLowByte = scratchpad[0];
    temperatureHighByte = scratchpad[1];
    // Combine the two bytes
    rawTemperature = temperatureHighByte;
    rawTemperature = rawTemperature << 8;
    rawTemperature = rawTemperature | temperatureLowByte;
    // Convert the raw value to degrees Celsius
    temperatureC = rawTemperature / 16.0f;
    // Check if the data is correct
    if (DS18B20_CalculateCRC(scratchpad, 8) != scratchpad[8])
    {
        // Show the error
        snprintf(txBuffer, sizeof(txBuffer),
                 "DS18B20 ERROR: CRC failed\r\n");
        HAL_UART_Transmit(&huart2,
                          (uint8_t *)txBuffer,
                          strlen(txBuffer),
                          100);
        return -1000.0f;
    }
    // Reject the invalid 85 C reading
    if (temperatureC == 85.0f)
    {
        snprintf(txBuffer, sizeof(txBuffer),
                 "DS18B20 ERROR: Invalid 85 C reading\r\n");
        HAL_UART_Transmit(&huart2,
                          (uint8_t *)txBuffer,
                          strlen(txBuffer),
                          100);
        return -1000.0f;
    }
    return temperatureC;
}


void Motor_Stop(void)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
}


// Set the motor direction to the negative encoder direction
void Motor_Forward(void)
{
    HAL_GPIO_WritePin(MOTOR_A1_GPIO_Port, MOTOR_A1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_B1_GPIO_Port, MOTOR_B1_Pin, GPIO_PIN_RESET);
}


// Set the motor direction to the positive encoder direction
void Motor_Backward(void)
{
    HAL_GPIO_WritePin(MOTOR_A1_GPIO_Port, MOTOR_A1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_B1_GPIO_Port, MOTOR_B1_Pin, GPIO_PIN_SET);
}


// Measure the voltage using the ACS724 100 times when the motor is not running,
// and take the average value. Use this average voltage as the "0 A reference"
// Find the ACS724 zero-current ADC value and show the raw readings
void ACS724_CalibrateZero(void)
{
    uint32_t sum = 0;
    uint32_t firstHalfSum = 0;
    uint32_t secondHalfSum = 0;
    // Read the sensor 20 times while the motor is stopped
    for (int i = 0; i < 20; i++)
    {
        HAL_ADC_Start(&hadc1);
        // Wait until the ADC finishes one reading
        HAL_ADC_PollForConversion(&hadc1, 100);
        // Save this ADC reading
        uint32_t rawADC = HAL_ADC_GetValue(&hadc1);
        // Add it to the total for the final average
        sum += rawADC;
        // Save the first 10 and last 10 samples separately
        if (i < 10)
        {
            firstHalfSum += rawADC;
        }
        else
        {
            secondHalfSum += rawADC;
        }
        // Print the raw ADC value so we can see if it is stable
        snprintf(txBuffer, sizeof(txBuffer),
                 "Zero Sample:%d ADC:%lu\r\n",
                 i,
                 (unsigned long)rawADC);
        HAL_UART_Transmit(&huart2,
                          (uint8_t *)txBuffer,
                          strlen(txBuffer),
                          100);
        // Wait before taking the next reading
        HAL_Delay(5);
    }
    // Calculate the average ADC reading at 0 A
    currentZeroADC = (float)sum / 20.0f;
    float firstHalfAverage = (float)firstHalfSum / 10.0f;
    float secondHalfAverage = (float)secondHalfSum / 10.0f;
    // Convert the zero-current ADC reading into voltage
    currentZeroVoltage = (currentZeroADC / 4095.0f) * 3.3f;
    snprintf(txBuffer, sizeof(txBuffer),
             "First 10 Avg: %.0f  Last 10 Avg: %.0f\r\n",
             firstHalfAverage,
             secondHalfAverage);
    HAL_UART_Transmit(&huart2,
                      (uint8_t *)txBuffer,
                      strlen(txBuffer),
                      100);
    // Print the final zero-current average
    snprintf(txBuffer, sizeof(txBuffer),
             "ZERO AVERAGE: %.0f\r\n",
             currentZeroADC);
    HAL_UART_Transmit(&huart2,
                      (uint8_t *)txBuffer,
                      strlen(txBuffer),
                      100);
}


// Read the average motor current from the ACS724
float ACS724_ReadCurrent(void)
{
	uint32_t sum = 0;
    // Take 20 ADC readings over a short period of time
    for (int i = 0; i < 20; i++)
    {
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 100);
        // Add this ADC reading to the total
        sum += HAL_ADC_GetValue(&hadc1);
        // Wait before taking the next sample
        HAL_Delay(1);
    }
    // Save the average ADC reading while the motor is running
    currentAverageADC = (float)sum / 20.0f;
    // Change the average ADC number into voltage
    float adcVoltage = (currentAverageADC / 4095.0f) * 3.3f;
    // Calculate the motor current in amps
    float current = (adcVoltage - currentZeroVoltage) / 0.1333f;
    return current;
}


// Try to release a stuck I2C bus
void I2C1_BusRecovery(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    HAL_I2C_DeInit(&hi2c1);
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    for (int i = 0; i < 9; i++)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
        HAL_Delay(1);
    }
    HAL_I2C_Init(&hi2c1);
}


// Check the SDA line without I2C control
void I2C1_TestSDA(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    HAL_I2C_DeInit(&hi2c1);
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    snprintf(txBuffer, sizeof(txBuffer),
             "SDA GPIO Test: %d\r\n",
             HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9));
    HAL_UART_Transmit(&huart2,
                      (uint8_t *)txBuffer,
                      strlen(txBuffer),
                      100);
}


// Check the actuator health and return its operating status
SystemStatus UpdateSystemStatus(float current,
                                float temperature,
                                float vibration)
{
    // ERROR conditions
    if (current >= 0.90f ||
        temperature >= 80.0f ||
        vibration >= 1.00f)
    {
        return STATUS_ERROR;
    }
    // WARNING conditions
    if (current >= 0.70f ||
        temperature >= 60.0f ||
        vibration >= 0.50f)
    {
        return STATUS_WARNING;
    }
    // Everything is within the normal range
    return STATUS_NORMAL;
}


// Convert the system status to text
const char *GetSystemStatusText(SystemStatus status)
{
    switch (status)
    {
        case STATUS_NORMAL:
            return "NORMAL";
        case STATUS_WARNING:
            return "WARNING";
        case STATUS_ERROR:
            return "ERROR";
        default:
            return "UNKNOWN";
    }
}


// Apply safety action based on the actuator status
void ApplySafetyAction(SystemStatus status)
{
    switch (status)
    {
        case STATUS_NORMAL:
            // Normal operation
            break;
        case STATUS_WARNING:
            // Keep running but report the warning
            break;
        case STATUS_ERROR:
            // Stop the motor immediately
            Motor_Stop();
            break;
        default:
            // Unknown state is treated as unsafe
            Motor_Stop();
            break;
    }
}


// Read the ADXL345 and calculate vibration RMS
float ADXL345_ReadVibrationRMS(void)
{
    float vibrationSum = 0.0f;
    int validSamples = 0;
    uint32_t startTime = HAL_GetTick();
    HAL_StatusTypeDef adxlStatus;
    // Collect 50 valid acceleration samples
    while (validSamples < 50)
    {
        // Prevent the function from getting stuck forever
        if ((HAL_GetTick() - startTime) >= 1000)
        {
            return -1.0f;
        }
        // Check if new acceleration data is ready
        adxlStatus = HAL_I2C_Mem_Read(&hi2c1,
                                      0x53 << 1,
                                      0x30,
                                      I2C_MEMADD_SIZE_8BIT,
                                      &adxlIntSource,
                                      1,
                                      100);
        if (adxlStatus != HAL_OK)
        {
            return -1.0f;
        }
        // Bit 7 means new data is ready
        if ((adxlIntSource & 0x80) != 0)
        {
            // Read X, Y and Z acceleration
            adxlStatus = HAL_I2C_Mem_Read(&hi2c1,
                                          0x53 << 1,
                                          0x32,
                                          I2C_MEMADD_SIZE_8BIT,
                                          adxlData,
                                          6,
                                          100);
            if (adxlStatus != HAL_OK)
            {
                return -1.0f;
            }
            // Convert the 6 bytes into X, Y and Z values
            adxlX = (int16_t)((adxlData[1] << 8) | adxlData[0]);
            adxlY = (int16_t)((adxlData[3] << 8) | adxlData[2]);
            adxlZ = (int16_t)((adxlData[5] << 8) | adxlData[4]);
            // Convert raw values to g
            accelX_g = adxlX * 0.0039f;
            accelY_g = adxlY * 0.0039f;
            accelZ_g = adxlZ * 0.0039f;
            // Calculate total acceleration
            totalAccel_g = sqrtf(accelX_g * accelX_g +
                                 accelY_g * accelY_g +
                                 accelZ_g * accelZ_g);
            // Remove approximately 1 g of gravity
            vibration_g = fabsf(totalAccel_g - 1.0f);
            // Add the squared vibration value
            vibrationSum += vibration_g * vibration_g;
            validSamples++;
        }
        HAL_Delay(1);
    }
    // Calculate RMS vibration
    return sqrtf(vibrationSum / validSamples);
}


// Move the actuator to the target position
void Actuator_MoveTo(int16_t targetPosition)
{
    float Kp = 0.01f;
    int16_t deadband = 20;
    uint32_t timeout = 5000;
    uint32_t startTime = HAL_GetTick();
    int16_t lastPrintedPosition = 0;
    // Save the commanded position
    commandedPosition = targetPosition;
    // Reset movement data
    movementTime_ms = 0;
    stopRequested = 0;
    while (1)
    {
        // Stop the actuator if requested
        if (stopRequested)
        {
            Motor_Stop();
            stopRequested = 0;

            actualPosition =
                (int16_t)__HAL_TIM_GET_COUNTER(&htim4);

            positionError =
                commandedPosition - actualPosition;

            movementTime_ms =
                HAL_GetTick() - startTime;

            snprintf(txBuffer, sizeof(txBuffer),
                     "MOVE STOPPED | Target:%d Position:%d Error:%d Time:%lu ms\r\n",
                     commandedPosition,
                     actualPosition,
                     positionError,
                     (unsigned long)movementTime_ms);

            HAL_UART_Transmit(&huart2,
                              (uint8_t *)txBuffer,
                              strlen(txBuffer),
                              100);
            return;
        }

        // Stop the motor if the system reports an error
        if (systemStatus == STATUS_ERROR)
        {
            Motor_Stop();

            const char msg[] = "MOVE ABORTED | SYSTEM ERROR\r\n";

            HAL_UART_Transmit(&huart2,
                              (uint8_t *)msg,
                              sizeof(msg) - 1,
                              100);
            return;
        }

        // Read the real encoder position
        actualPosition =
            (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
        // Calculate position error
        positionError =
            commandedPosition - actualPosition;
        // Update movement time
        movementTime_ms =
            HAL_GetTick() - startTime;
        // Check if the target is reached
        if (abs(positionError) <= deadband)
        {
            Motor_Stop();
            snprintf(txBuffer, sizeof(txBuffer),
                     "TARGET REACHED | Target:%d Position:%d Error:%d Time:%lu ms\r\n",
                     commandedPosition,
                     actualPosition,
                     positionError,
                     (unsigned long)movementTime_ms);
            HAL_UART_Transmit(&huart2,
                              (uint8_t *)txBuffer,
                              strlen(txBuffer),
                              100);
            return;
        }
        // Check movement timeout
        if (movementTime_ms >= timeout)
        {
            Motor_Stop();
            systemStatus = STATUS_ERROR;
            snprintf(txBuffer, sizeof(txBuffer),
                     "POSITION ERROR | Target:%d Position:%d Error:%d Time:%lu ms | Status:ERROR\r\n",
                     commandedPosition,
                     actualPosition,
                     positionError,
                     (unsigned long)movementTime_ms);
            HAL_UART_Transmit(&huart2,
                              (uint8_t *)txBuffer,
                              strlen(txBuffer),
                              100);
            return;
        }
        // Calculate PWM from position error
        int32_t absoluteError = abs(positionError);
        uint32_t pwm =
            (uint32_t)(Kp * absoluteError);
        // Limit maximum PWM
        if (pwm > 12)
        {
            pwm = 12;
        }
        // Keep enough PWM to move the motor
        if (pwm < 10)
        {
            pwm = 10;
        }
        // Choose motor direction
        if (positionError < 0)
        {
            Motor_Forward();
        }
        else
        {
            Motor_Backward();
        }
        // Apply PWM
        __HAL_TIM_SET_COMPARE(&htim3,
                              TIM_CHANNEL_1,
                              pwm);
        // Show movement progress
        if (abs(actualPosition - lastPrintedPosition) >= 500)
        {
            snprintf(txBuffer, sizeof(txBuffer),
                     "Target:%d Position:%d Error:%d PWM:%lu Time:%lu ms\r\n",
                     commandedPosition,
                     actualPosition,
                     positionError,
                     (unsigned long)pwm,
                     (unsigned long)movementTime_ms);
            HAL_UART_Transmit(&huart2,
                              (uint8_t *)txBuffer,
                              strlen(txBuffer),
                              100);
            lastPrintedPosition = actualPosition;
        }
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  MX_TIM5_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  // Reset W5500
  W5500_Reset();

  // Test SPI communication
  W5500_Test();
  // Start UART command reception

  W5500_RegisterTest();
  W5500_NetworkInit();
  W5500_CheckLink();
  W5500_TCPServerInit();

  HAL_UART_Receive_IT(&huart2, &rxByte, 1);
//  PB9_OutputTest();
//  while (1)
//  {
//	    snprintf(txBuffer, sizeof(txBuffer),
//	             "PB9 SDA: %d\r\n",
//	             HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9));
//
//	    HAL_UART_Transmit(&huart2,
//	                      (uint8_t *)txBuffer,
//	                      strlen(txBuffer),
//	                      100);
//
//	    HAL_Delay(1000);
//  }
//  // I2C1_BusRecovery();
//  I2C1_TestSDA();
//
//  snprintf(txBuffer, sizeof(txBuffer),
//           "PB9 Test: MODER=0x%08lX PUPDR=0x%08lX IDR=0x%08lX\r\n",
//           GPIOB->MODER,
//           GPIOB->PUPDR,
//           GPIOB->IDR);
//
//  HAL_UART_Transmit(&huart2,
//                    (uint8_t *)txBuffer,
//                    strlen(txBuffer),
//                    100);
  // Recover the I2C bus
//  I2C1_BusRecovery();
  // Read the ADXL345 device ID
  HAL_StatusTypeDef adxlStatus;
  adxlStatus = HAL_I2C_Mem_Read(&hi2c1,
                                0x53 << 1,
                                0x00,
                                I2C_MEMADD_SIZE_8BIT,
                                &adxl345DeviceID,
                                1,
                                100);
  snprintf(txBuffer, sizeof(txBuffer),
           "ADXL345 Status: %d | Device ID: 0x%02X\r\n",
           adxlStatus,
           adxl345DeviceID);
  HAL_UART_Transmit(&huart2,
                    (uint8_t *)txBuffer,
                    strlen(txBuffer),
                    100);
  // Start ADXL345 measurement
  if ((adxlStatus == HAL_OK) && (adxl345DeviceID == 0xE5))
  {
      adxlStatus = HAL_I2C_Mem_Write(&hi2c1,
                                     0x53 << 1,
                                     0x2D,
                                     I2C_MEMADD_SIZE_8BIT,
                                     &adxl345PowerControl,
                                     1,
                                     100);
      snprintf(txBuffer, sizeof(txBuffer),
               "ADXL345 Measurement Start Status: %d\r\n",
               adxlStatus);
      HAL_UART_Transmit(&huart2,
                        (uint8_t *)txBuffer,
                        strlen(txBuffer),
                        100);
  }
  // Show the I2C bus state
  snprintf(txBuffer, sizeof(txBuffer),
           "I2C Bus: SCL=%d SDA=%d\r\n",
           HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8),
           HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9));
  HAL_UART_Transmit(&huart2,
                    (uint8_t *)txBuffer,
                    strlen(txBuffer),
                    100);
  // Show the I2C status registers
  snprintf(txBuffer, sizeof(txBuffer),
           "I2C Status: SR1=0x%04lX SR2=0x%04lX\r\n",
           hi2c1.Instance->SR1,
           hi2c1.Instance->SR2);
  HAL_UART_Transmit(&huart2,
                    (uint8_t *)txBuffer,
                    strlen(txBuffer),
                    100);
//  // Read the ADXL345 device ID
//  HAL_StatusTypeDef adxlStatus;
//
//  adxlStatus = HAL_I2C_Mem_Read(&hi2c1,
//                                0x53 << 1,
//                                0x00,
//                                I2C_MEMADD_SIZE_8BIT,
//                                &adxl345DeviceID,
//                                1,
//                                100);
//
//  snprintf(txBuffer, sizeof(txBuffer),
//           "ADXL345 Status: %d | Device ID: 0x%02X\r\n",
//           adxlStatus,
//           adxl345DeviceID);
//
//  HAL_UART_Transmit(&huart2,
//                    (uint8_t *)txBuffer,
//                    strlen(txBuffer),
//                    100);
//
//  // Start ADXL345 measurement
//  if ((adxlStatus == HAL_OK) && (adxl345DeviceID == 0xE5))
//  {
//      adxlStatus = HAL_I2C_Mem_Write(&hi2c1,
//                                     0x53 << 1,
//                                     0x2D,
//                                     I2C_MEMADD_SIZE_8BIT,
//                                     &adxl345PowerControl,
//                                     1,
//                                     100);
//
//      snprintf(txBuffer, sizeof(txBuffer),
//               "ADXL345 Measurement Start Status: %d\r\n",
//               adxlStatus);
//
//      HAL_UART_Transmit(&huart2,
//                        (uint8_t *)txBuffer,
//                        strlen(txBuffer),
//                        100);
//  }
  // Start TIM5 for microsecond timing
  HAL_TIM_Base_Start(&htim5);
  // Release the DS18B20 data line
  HAL_GPIO_WritePin(DS18B20_DATA_GPIO_Port,
                    DS18B20_DATA_Pin,
                    GPIO_PIN_SET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(MOTOR_A1_GPIO_Port, MOTOR_A1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MOTOR_B1_GPIO_Port, MOTOR_B1_Pin, GPIO_PIN_RESET);
  // Start the encoder so STM32 can track the motor position
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  // Treat the current motor position as position 0
  __HAL_TIM_SET_COUNTER(&htim4, 0);
  // Start PWM for motor control
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  // Keep the motor stopped
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);

  // One-time motor control test
  Motor_Backward();

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 15);

  HAL_Delay(500);

  Motor_Stop();

  const char msg[] = "MOTOR TEST COMPLETED\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t *)msg, sizeof(msg) - 1, 100);
  // Find the ACS724 voltage at 0 A
  // ACS724_CalibrateZero();

  motorTemperature = DS18B20_ReadTemperature();

  snprintf(txBuffer, sizeof(txBuffer),
           "Temperature Test: %.2f C\r\n",
           motorTemperature);

  HAL_UART_Transmit(&huart2,
                    (uint8_t *)txBuffer,
                    strlen(txBuffer),
                    100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Update heartbeat every second
	  if (HAL_GetTick() - lastHeartbeatTick >= 1000)
	  {
		  lastHeartbeatTick = HAL_GetTick();
		  heartbeatCounter++;
	  }

      // Check Ethernet link every 2 seconds
      static uint32_t lastLinkCheck = 0;

      if (HAL_GetTick() - lastLinkCheck >= 2000)
      {
          lastLinkCheck = HAL_GetTick();
          W5500_CheckLink();
      }

      // Check TCP connection
      W5500_TCPServerTask();

      static uint8_t debugPrinted = 0;

      if (debugPrinted == 0)
      {
          const char msg[] = "NEW FIRMWARE RUNNING\r\n";

          HAL_UART_Transmit(&huart2,
                            (uint8_t *)msg,
                            sizeof(msg) - 1,
                            100);

          debugPrinted = 1;
      }

      // Handle STOP when the actuator is idle
      if (stopRequested)
      {
          Motor_Stop();
          stopRequested = 0;

          const char msg[] = "STOP CONFIRMED\r\n";

          HAL_UART_Transmit(&huart2,
                            (uint8_t *)msg,
                            sizeof(msg) - 1,
                            100);
      }

      // Process a received command
      if (commandReady)
      {
          ProcessCommand(commandBuffer);
          commandReady = 0;
      }

      // Read temperature every 2 seconds
      static uint32_t lastTemperatureRead = 0;

      if (HAL_GetTick() - lastTemperatureRead >= 2000)
      {
          lastTemperatureRead = HAL_GetTick();
          motorTemperature = DS18B20_ReadTemperature();
      }

      // Read vibration every 500 ms
      static uint32_t lastVibrationRead = 0;

      if (HAL_GetTick() - lastVibrationRead >= 500)
      {
          lastVibrationRead = HAL_GetTick();
          vibrationRMS_g = ADXL345_ReadVibrationRMS();
      }

      // ACS724 wiring test: read the ADC on A0 once per second.
      // Keep the 12V motor supply disconnected during this test.
      // Do not use this uncalibrated value for current protection yet.
      static uint32_t lastCurrentTestTick = 0;
      if (HAL_GetTick() - lastCurrentTestTick >= 1000)
      {
          lastCurrentTestTick = HAL_GetTick();
          if (HAL_ADC_Start(&hadc1) == HAL_OK)
          {
              if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
              {
                  uint32_t adcRaw = HAL_ADC_GetValue(&hadc1);
                  float adcVoltage = (float)adcRaw * 3.3f / 4095.0f;
                  snprintf(txBuffer, sizeof(txBuffer),
                           "ACS724 A0 ADC:%lu Voltage:%.3f V\r\n",
                           (unsigned long)adcRaw, adcVoltage);
                  HAL_UART_Transmit(&huart2, (uint8_t *)txBuffer,
                                    strlen(txBuffer), 100);
              }
              else
              {
                  const char msg[] = "ACS724 ADC timeout\r\n";
                  HAL_UART_Transmit(&huart2, (uint8_t *)msg,
                                    sizeof(msg) - 1, 100);
              }
              HAL_ADC_Stop(&hadc1);
          }
          else
          {
              const char msg[] = "ACS724 ADC start failed\r\n";
              HAL_UART_Transmit(&huart2, (uint8_t *)msg,
                                sizeof(msg) - 1, 100);
          }
      }
      motorCurrent = 0.0f; // Placeholder until calibration and safe motor test.


      actualPosition =
          (int16_t)__HAL_TIM_GET_COUNTER(&htim4);

      // Encoder hand-turn test: print the raw TIM4 counter once per second.
      // Motor 12V supply must remain disconnected.
      static uint32_t lastEncoderTestTick = 0;
      if (HAL_GetTick() - lastEncoderTestTick >= 1000)
      {
          lastEncoderTestTick = HAL_GetTick();
          uint16_t encoderRaw = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
          snprintf(txBuffer, sizeof(txBuffer),
                   "ENCODER TIM4:%u Position:%d\r\n",
                   (unsigned int)encoderRaw, (int)(int16_t)encoderRaw);
          HAL_UART_Transmit(&huart2, (uint8_t *)txBuffer,
                            strlen(txBuffer), 100);
      }
      // Read the actuator temperature
  //    motorTemperature = DS18B20_ReadTemperature();
      // Show the temperature
  //    snprintf(txBuffer, sizeof(txBuffer),
  //             "Temperature: %.2f C\r\n",
  //             motorTemperature);
  //
  //    HAL_UART_Transmit(&huart2,
  //                      (uint8_t *)txBuffer,
  //                      strlen(txBuffer),
  //                      100);
//      // Test ACS724 raw ADC value
//      HAL_ADC_Start(&hadc1);
//
//      if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
//      {
//          uint32_t adcRaw = HAL_ADC_GetValue(&hadc1);
//
//          snprintf(txBuffer, sizeof(txBuffer),
//                   "PA0 ADC Raw: %lu\r\n",
//                   adcRaw);
//
//          HAL_UART_Transmit(&huart2,
//                            (uint8_t *)txBuffer,
//                            strlen(txBuffer),
//                            100);
//      }
//      else
//      {
//          snprintf(txBuffer, sizeof(txBuffer),
//                   "ADC Conversion Error\r\n");
//
//          HAL_UART_Transmit(&huart2,
//                            (uint8_t *)txBuffer,
//                            strlen(txBuffer),
//                            100);
//      }
//
//      HAL_Delay(500);
//
//
//  #if 0
//
//      // ADXL345 vibration test
//      float vibrationSum = 0.0f;
//      int validSamples = 0;
//
//      // Take 50 new acceleration samples
//      while (validSamples < 50)
//      {
//          // Check if new ADXL345 data is ready
//          adxlStatus = HAL_I2C_Mem_Read(&hi2c1,
//                                        0x53 << 1,
//                                        0x30,
//                                        I2C_MEMADD_SIZE_8BIT,
//                                        &adxlIntSource,
//                                        1,
//                                        100);
//
//          if (adxlStatus != HAL_OK)
//          {
//              // Show I2C error information
//              snprintf(txBuffer, sizeof(txBuffer),
//                       "I2C Error: Status=%d SCL=%d SDA=%d SR1=0x%04lX SR2=0x%04lX\r\n",
//                       adxlStatus,
//                       HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8),
//                       HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9),
//                       hi2c1.Instance->SR1,
//                       hi2c1.Instance->SR2);
//
//              HAL_UART_Transmit(&huart2,
//                                (uint8_t *)txBuffer,
//                                strlen(txBuffer),
//                                100);
//
//              break;
//          }
//
//          // Bit 7 = DATA_READY
//          if ((adxlIntSource & 0x80) != 0)
//          {
//              // Read ADXL345 X, Y and Z data
//              adxlStatus = HAL_I2C_Mem_Read(&hi2c1,
//                                            0x53 << 1,
//                                            0x32,
//                                            I2C_MEMADD_SIZE_8BIT,
//                                            adxlData,
//                                            6,
//                                            100);
//
//              if (adxlStatus == HAL_OK)
//              {
//                  // Convert raw bytes to signed values
//                  adxlX = (int16_t)((adxlData[1] << 8) | adxlData[0]);
//                  adxlY = (int16_t)((adxlData[3] << 8) | adxlData[2]);
//                  adxlZ = (int16_t)((adxlData[5] << 8) | adxlData[4]);
//
//                  // Convert raw values to g
//                  accelX_g = adxlX * 0.0039f;
//                  accelY_g = adxlY * 0.0039f;
//                  accelZ_g = adxlZ * 0.0039f;
//
//                  // Calculate total acceleration
//                  totalAccel_g = sqrtf(accelX_g * accelX_g +
//                                       accelY_g * accelY_g +
//                                       accelZ_g * accelZ_g);
//
//                  // Calculate instantaneous vibration
//                  vibration_g = fabsf(totalAccel_g - 1.0f);
//
//                  // Add squared vibration for RMS calculation
//                  vibrationSum += vibration_g * vibration_g;
//
//                  validSamples++;
//              }
//              else
//              {
//                  // Show I2C error information
//                  snprintf(txBuffer, sizeof(txBuffer),
//                           "I2C Error: Status=%d SCL=%d SDA=%d SR1=0x%04lX SR2=0x%04lX\r\n",
//                           adxlStatus,
//                           HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8),
//                           HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9),
//                           hi2c1.Instance->SR1,
//                           hi2c1.Instance->SR2);
//
//                  HAL_UART_Transmit(&huart2,
//                                    (uint8_t *)txBuffer,
//                                    strlen(txBuffer),
//                                    100);
//
//                  break;
//              }
//          }
//
//          HAL_Delay(1);
//      }
//
//      // Calculate RMS using successful samples
//      if (validSamples > 0)
//      {
//          vibrationRMS_g = sqrtf(vibrationSum / validSamples);
//      }
//      else
//      {
//          vibrationRMS_g = 0.0f;
//      }
//
//      // Show the latest acceleration and vibration RMS
//      snprintf(txBuffer, sizeof(txBuffer),
//               "Accel: X=%.3f Y=%.3f Z=%.3f Total=%.3f g | Vibration RMS=%.3f g | Samples=%d\r\n",
//               accelX_g,
//               accelY_g,
//               accelZ_g,
//               totalAccel_g,
//               vibrationRMS_g,
//               validSamples);
//
//      HAL_UART_Transmit(&huart2,
//                        (uint8_t *)txBuffer,
//                        strlen(txBuffer),
//                        100);
//
//      HAL_Delay(500);
//
//  #endif
//	    // Run the motor forward
//	    Motor_Backward();
//
//	    // Set PWM to 15
//	    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 15);
//
//	    // Run for 500 ms
//	    HAL_Delay(500);
//
//	    // Stop the motor
//	    Motor_Stop();
//
//	    // Wait 3 seconds before the next test
//	    HAL_Delay(3000);
//	    // Test values
//	    // Status: NORMAL
//	    motorCurrent = 0.50f;
//	    motorTemperature = 40.0f;
//	    vibrationRMS_g = 0.20f;
//
//	    // Status: WARNING
//	    motorCurrent = 0.75f;
//	    motorTemperature = 40.0f;
//	    vibrationRMS_g = 0.20f;
//	    // Status: WARNING
//	    motorCurrent = 0.50f;
//	    motorTemperature = 85.0f;
//	    vibrationRMS_g = 0.20f;
//	    // Update actuator health
//	    systemStatus = UpdateSystemStatus(motorCurrent,
//	                                      motorTemperature,
//	                                      vibrationRMS_g);
//
//	    // Apply the safety action
//	    ApplySafetyAction(systemStatus);
//
//	    // Show actuator health
//	    snprintf(txBuffer, sizeof(txBuffer),
//	             "Current: %.2f A | Temperature: %.1f C | Vibration: %.2f g | Status: %s\r\n",
//	             motorCurrent,
//	             motorTemperature,
//	             vibrationRMS_g,
//	             GetSystemStatusText(systemStatus));
//
//	    HAL_UART_Transmit(&huart2,
//	                      (uint8_t *)txBuffer,
//	                      strlen(txBuffer),
//	                      100);
//
//	    HAL_Delay(1000);
//	    Actuator_MoveTo(3000);
//
//	    while (1)
//	    {
//	        Motor_Stop();
//	        HAL_Delay(100);
//	    }
	    // Check the temperature sensor
	    if (motorTemperature <= -999.0f)
	    {
	        systemStatus = STATUS_ERROR;
	        // Stop the motor for safety
	        ApplySafetyAction(systemStatus);
	        // Show the temperature sensor error
	        snprintf(txBuffer, sizeof(txBuffer),
	                 "Temperature Sensor Error | Status: ERROR\r\n");
	        HAL_UART_Transmit(&huart2,
	                          (uint8_t *)txBuffer,
	                          strlen(txBuffer),
	                          100);
	    }
	    // Check the vibration sensor
	    else if (vibrationRMS_g < 0.0f)
	    {
	        systemStatus = STATUS_ERROR;
	        // Stop the motor for safety
	        ApplySafetyAction(systemStatus);
	        // Show the vibration sensor error
	        snprintf(txBuffer, sizeof(txBuffer),
	                 "Vibration Sensor Error | Status: ERROR\r\n");
	        HAL_UART_Transmit(&huart2,
	                          (uint8_t *)txBuffer,
	                          strlen(txBuffer),
	                          100);
	    }
	    // All active sensors are working
	    else
	    {
	        // Update actuator health
	        systemStatus = UpdateSystemStatus(motorCurrent,
	                                          motorTemperature,
	                                          vibrationRMS_g);
	        // Apply the safety action
	        ApplySafetyAction(systemStatus);
	        // Show all actuator test bench data
	        static uint32_t lastPrint = 0;

	        if (HAL_GetTick() - lastPrint >= 2000)
	        {
	            lastPrint = HAL_GetTick();

	            // W5500 Ethernet link
	            uint8_t phyStatus = W5500_ReadRegister(0x002E);

	            snprintf(txBuffer, sizeof(txBuffer),
	                     "W5500 PHY:0x%02X Link:%s\r\n",
	                     phyStatus,
	                     (phyStatus & 0x01) ? "CONNECTED" : "DISCONNECTED");

	            HAL_UART_Transmit(&huart2, (uint8_t *)txBuffer,
	                              strlen(txBuffer), 100);

	            // TCP socket status
	            uint8_t socketStatus = W5500_ReadSocketRegister(0x0003);

	            snprintf(txBuffer, sizeof(txBuffer),
	                     "TCP Socket Status:0x%02X\r\n",
	                     socketStatus);

	            HAL_UART_Transmit(&huart2, (uint8_t *)txBuffer,
	                              strlen(txBuffer), 100);

	            // DS18B20 raw data
	            snprintf(txBuffer, sizeof(txBuffer),
	                     "DS18B20 RAW: %02X %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
	                     lastDS18B20Raw[0], lastDS18B20Raw[1],
	                     lastDS18B20Raw[2], lastDS18B20Raw[3],
	                     lastDS18B20Raw[4], lastDS18B20Raw[5],
	                     lastDS18B20Raw[6], lastDS18B20Raw[7],
	                     lastDS18B20Raw[8]);

	            HAL_UART_Transmit(&huart2, (uint8_t *)txBuffer,
	                              strlen(txBuffer), 100);

	            // Actuator system data
	            PrintSystemData();

	            HAL_UART_Transmit(&huart2,
	                              (uint8_t *)"----------------------------------------\r\n",
	                              strlen("----------------------------------------\r\n"),
	                              100);
	        }
	    }
	    // Wait before the next health check
	    HAL_Delay(10);
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */
  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */
  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */
  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */
  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */
  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */
  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */
  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */
  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 49;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */
  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */
  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */
  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */
  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 83;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */
  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */
  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DS18B20_DATA_GPIO_Port, DS18B20_DATA_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_10, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MOTOR_A1_Pin|MOTOR_B1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : DS18B20_DATA_Pin */
  GPIO_InitStruct.Pin = DS18B20_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DS18B20_DATA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR_A1_Pin MOTOR_B1_Pin */
  GPIO_InitStruct.Pin = MOTOR_A1_Pin|MOTOR_B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Receive commands from the computer
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        // End of command
        if (rxByte == '\n' || rxByte == '\r')
        {
            if (rxIndex > 0)
            {
                rxBuffer[rxIndex] = '\0';
                // STOP must work while the actuator is moving
                if (strcmp(rxBuffer, "STOP") == 0)
                {
                    stopRequested = 1;
                }
                else if (!commandReady)
                {
                    // Save the complete command
                    strcpy(commandBuffer, rxBuffer);
                    // Tell the main program a command is ready
                    commandReady = 1;
                }
                // Start receiving a new command
                rxIndex = 0;
            }
        }
        else
        {
        	// Accept only valid command characters
        	if ((rxByte >= 'A' && rxByte <= 'Z') ||
        	    (rxByte >= '0' && rxByte <= '9') ||
        	    rxByte == ',' ||
        	    rxByte == '-')
        	{
        	    if (rxIndex < sizeof(rxBuffer) - 1)
        	    {
        	        rxBuffer[rxIndex] = rxByte;
        	        rxIndex++;
        	    }
        	}
            else
            {
                // Reset if the command is too long
                rxIndex = 0;
            }
        }
        // Receive the next character
        HAL_UART_Receive_IT(&huart2, &rxByte, 1);
    }
}


// Process commands from the computer
void ProcessCommand(char *command)
{
	// Move command - TEST MODE (motor disabled)
	if (strncmp(command, "MOVE,", 5) == 0)
	{
	    int32_t target = atoi(&command[5]);

	    if (target >= -30000 && target <= 30000)
	    {
	        // Keep the motor stopped during testing
	        Motor_Stop();

	        snprintf(txBuffer, sizeof(txBuffer),
	                 "MOVE ACCEPTED | TEST MODE | Target:%ld\r\n",
	                 (long)target);

	        HAL_UART_Transmit(&huart2,
	                          (uint8_t *)txBuffer,
	                          strlen(txBuffer),
	                          100);
	    }
	    else
	    {
	        snprintf(txBuffer, sizeof(txBuffer),
	                 "COMMAND ERROR | Invalid position\r\n");

	        HAL_UART_Transmit(&huart2,
	                          (uint8_t *)txBuffer,
	                          strlen(txBuffer),
	                          100);
	    }
	}

    // Stop the motor
    else if (strcmp(command, "STOP") == 0)
    {
        Motor_Stop();
        snprintf(txBuffer, sizeof(txBuffer),
                 "COMMAND RECEIVED | STOP\r\n");
        HAL_UART_Transmit(&huart2,
                          (uint8_t *)txBuffer,
                          strlen(txBuffer),
                          100);
    }
    // Send the current system data
    else if (strcmp(command, "STATUS") == 0)
    {
        SendSystemData();
    }
    // Unknown command
    else
    {
        snprintf(txBuffer, sizeof(txBuffer),
                 "COMMAND ERROR | Unknown command: %s\r\n",
                 command);
        HAL_UART_Transmit(&huart2,
                          (uint8_t *)txBuffer,
                          strlen(txBuffer),
                          100);
    }
}


// Send system data to the computer
void SendSystemData(void)
{
    snprintf(txBuffer, sizeof(txBuffer),
             "DATA,%d,%d,%d,%lu,%.2f,%.2f,%.3f,%d\r\n",
             commandedPosition,
             actualPosition,
             positionError,
             (unsigned long)movementTime_ms,
             motorCurrent,
             motorTemperature,
             vibrationRMS_g,
             systemStatus);
    HAL_UART_Transmit(&huart2,
                      (uint8_t *)txBuffer,
                      strlen(txBuffer),
                      100);
}
// Show all actuator test bench data
void PrintSystemData(void)
{
    snprintf(txBuffer, sizeof(txBuffer),
             "Command:%d | Position:%d | Error:%d | Time:%lu ms | Current:%.2f A | Temperature:%.2f C | Vibration:%.3f g | Status:%s\r\n",
             commandedPosition,
             actualPosition,
             positionError,
             (unsigned long)movementTime_ms,
             motorCurrent,
             motorTemperature,
             vibrationRMS_g,
             GetSystemStatusText(systemStatus));
    HAL_UART_Transmit(&huart2,
                      (uint8_t *)txBuffer,
                      strlen(txBuffer),
                      100);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
