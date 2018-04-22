/**
  ******************************************************************************
  * @file    sensors_data.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    04-September-2017
  * @brief   Manage sensors of STM32L475 IoT board.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "sensors_data.h"


#include "stm32l4xx_hal.h"
#include "stm32l475e_iot01.h"
#include "stm32l475e_iot01_tsensor.h"
#include "stm32l475e_iot01_hsensor.h"
#include "stm32l475e_iot01_psensor.h"
#include "stm32l475e_iot01_magneto.h"
#include "stm32l475e_iot01_gyro.h"
#include "stm32l475e_iot01_accelero.h"
#include "vl53l0x_proximity.h"
#include "msg.h"

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/

static float    TEMPERATURE_Value;
static float    HUMIDITY_Value;
static float    PRESSURE_Value;
static int16_t  ACC_Value[3];
static float    GYR_Value[3];
static int16_t  MAG_Value[3];
static uint16_t PROXIMITY_Value;
static char outputString1[24];
static char outputString2[24];
static char outputString3[24];
static char outputString4[24];
static int sum1 = 0;
static int sum2 = 0;
static int sum3 = 0;
static int sum4 = 0;

/* Private function prototypes -----------------------------------------------*/
int getInputStringSensor(char* inputString, int commandType);
//void clearRx();
/* Functions Definition ------------------------------------------------------*/

/**
  * @brief  init_sensors
  * @param  none
  * @retval 0 in case of success
  *         -1 in case of failure
  */
int init_sensors(void)
{
  int ret = 0;
  
  if (HSENSOR_OK != BSP_HSENSOR_Init())
  {
    msg_error("BSP_HSENSOR_Init() returns %d\n", ret);
    ret = -1;
  }
  
  if (TSENSOR_OK != BSP_TSENSOR_Init())
  {
    msg_error("BSP_TSENSOR_Init() returns %d\n", ret);
    ret = -1;
  }
  
  if (PSENSOR_OK != BSP_PSENSOR_Init())
  {
    msg_error("BSP_PSENSOR_Init() returns %d\n", ret);
    ret = -1;
  }
  
  if (MAGNETO_OK != BSP_MAGNETO_Init())
  {
    msg_error("BSP_MAGNETO_Init() returns %d\n", ret);
    ret = -1;
  }

  if (GYRO_OK != BSP_GYRO_Init())
  {
    msg_error("BSP_GYRO_Init() returns %d\n", ret);
    ret = -1;
  }
  
  if (ACCELERO_OK != BSP_ACCELERO_Init())
  {
    msg_error("BSP_ACCELERO_Init() returns %d\n", ret);
    ret = -1;
  }
  
  VL53L0X_PROXIMITY_Init();
  
  return ret;
}

/**
  * @brief  fill the payload with the sensor values
  * @param  none
  * @param PayloadBuffer is the char pointer for the Payload buffer to be filled
  * @param PayloadSize size of the above buffer
  * @retval 0 in case of success
  *         -1 in case of failure
  */
int PrepareMqttPayload(char * PayloadBuffer, int PayloadSize, char * deviceID)
{
  char * Buff = PayloadBuffer;
  int BuffSize = PayloadSize;
  int snprintfreturn = 0;
  static int counter = 0;

  TEMPERATURE_Value = BSP_TSENSOR_ReadTemp();
  HUMIDITY_Value = BSP_HSENSOR_ReadHumidity();
  PRESSURE_Value = BSP_PSENSOR_ReadPressure();
  PROXIMITY_Value = VL53L0X_PROXIMITY_GetDistance();
  BSP_ACCELERO_AccGetXYZ(ACC_Value);
  BSP_GYRO_GetXYZ(GYR_Value);
  BSP_MAGNETO_GetXYZ(MAG_Value);
  if (counter == 0) {
	  counter++;

  }
  else {
	//HAL_Delay(500);

    sum1 = getInputStringSensor(outputString1, 1);
   // HAL_Delay(500);
    sum2 = getInputStringSensor(outputString2, 2);
   // HAL_Delay(500);
    sum3 = getInputStringSensor(outputString3, 3);
   // HAL_Delay(500);
    sum4 = getInputStringSensor(outputString4, 4);
//    if (counter == 1){
//    	HAL_Delay(5000);
//    	counter++;
//    }
  }


  // Insert ODB commands and calculations

 #ifdef BLUEMIX
    snprintfreturn = snprintf( Buff, BuffSize, "{\"d\":{"
             "\"temperature\": %.2f, \"humidity\": %.2f, \"pressure\": %.2f, \"proximity\": %d, "
             "\"acc_x\": %d, \"acc_y\": %d, \"acc_z\": %d, "
             "\"gyr_x\": %.0f, \"gyr_y\": %.0f, \"gyr_z\": %.0f, "
             "\"mag_x\": %d, \"mag_y\": %d, \"mag_z\": %d"
               "}}",
             TEMPERATURE_Value, HUMIDITY_Value, PRESSURE_Value, PROXIMITY_Value,
             ACC_Value[0], ACC_Value[1], ACC_Value[2],
             GYR_Value[0], GYR_Value[1], GYR_Value[2],
             MAG_Value[0], MAG_Value[1], MAG_Value[2] );
  
 #else
  if (deviceID != NULL)
  {

    snprintfreturn = snprintf( Buff, BuffSize, "{\"deviceId\":\"%s\","
             "\"temperature\": %.2f, \"humidity\": %.2f, \"pressure\": %.2f, \"proximity\": %d, "
             "\"acc_x\": %d, \"acc_y\": %d, \"acc_z\": %d, "
             "\"gyr_x\": %.0f, \"gyr_y\": %.0f, \"gyr_z\": %.0f, "
             "\"mag_x\": %d, \"mag_y\": %d, \"mag_z\": %d"
             "}",
             deviceID,
             TEMPERATURE_Value, HUMIDITY_Value, PRESSURE_Value, PROXIMITY_Value,
             ACC_Value[0], ACC_Value[1], ACC_Value[2],
             GYR_Value[0], GYR_Value[1], GYR_Value[2],
             MAG_Value[0], MAG_Value[1], MAG_Value[2]);
  }
  else
  {
	  // Replace ODB_data with appropriate data titles
  snprintfreturn = snprintf( Buff, BuffSize, "{\n \"state\": {\n  \"reported\": {\n"
           "   \"temperature\": %.2f,\n   \"humidity\": %.2f,\n   \"pressure\": %.2f,\n   \"proximity\": %d,\n"
           "   \"acc_x\": %d, \"acc_y\": %d, \"acc_z\": %d,\n"
           "   \"gyr_x\": %.0f, \"gyr_y\": %.0f, \"gyr_z\": %.0f,\n"
           "   \"mag_x\": %d, \"mag_y\": %d, \"mag_z\": %d,\n"
    	   "   \"ODB_data1\": %d, \"ODB_data2\": %d, \"ODB_data3\": %d"
           "  }\n }\n}",
           TEMPERATURE_Value, HUMIDITY_Value, PRESSURE_Value, PROXIMITY_Value,
           ACC_Value[0], ACC_Value[1], ACC_Value[2],
           GYR_Value[0], GYR_Value[1], GYR_Value[2],
           MAG_Value[0], MAG_Value[1], MAG_Value[2],
		   sum1, sum2, sum3);	// Values to be replaced with ODB data/calculations
  }
 #endif
  /* Check total size to be less than buffer size
            * if the return is >=0 and <n, then
            * the entire string was successfully formatted; if the return is
            * >=n, the string was truncated (but there is still a null char
            * at the end of what was written); if the return is <0, there was
            * an error.
            */
  if (snprintfreturn >= 0 && snprintfreturn < PayloadSize)
  {
      return 0;
  }
  else if(snprintfreturn >= PayloadSize)
  {
      msg_error("Data Pack truncated\n");
      return 0;
  }
  else
  {
      msg_error("Data Pack Error\n");
      return -1;
  }
}


//void clearRx()
//{
//  int i = 0;
//  int c;
//  while (i < 25)
//  {
//    c = getchar();
//    i++;
//  }
//}

// Added functionality for getting return information from ODB reader
int getInputStringSensor(char* inputString, int commandType)
{
  //static int counter = 0;
  int len = 24;	// change to 8?
  size_t currLen = 0;
  int c = 0;

  switch(commandType)
  {
    case 1:
      printf("010D\r");
	  break;
    case 2:
      printf("010B\r");
	  break;
    case 3:
      printf("010C\r");
	  break;
    case 4:
      printf("0144\r");
      break;
  }

  c = getchar();
//  if (((c == '\r') || (c == 0x0D)) && (counter == 0)){
//	  c = getchar();
//  }
  //counter++;
// && (c != '\n') && (c != 0x0D) && (c != 0x0A)
  while (c != '\r')
  {
	  c = getchar();
  }

  c = getchar();

  while ((c != EOF) && ((currLen + 1) < len) && (c != '\r'))
  {

    if (currLen < (len-1))
    {
      inputString[currLen] = c;
    }

    ++currLen;

    c = getchar();

    //printf("Char: %c\n",c);
  }
  if (currLen != 0)
  { /* Close the string in the input buffer... only if a string was written to it. */
    inputString[currLen] = '\0';
  }
  if (c == '\r')
  {
    c = getchar(); /* assume there is '\n' after '\r'. Just discard it. */
  }

  //printf("%s", inputString);
  //counter = 0;
  return currLen;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
