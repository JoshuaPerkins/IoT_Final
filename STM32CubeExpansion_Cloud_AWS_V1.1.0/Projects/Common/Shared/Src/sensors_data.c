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
static char outputString5[24];
static float vehicleSpeed = 0.0;
static float intakePressure = 0.0;
static float engineRPM = 0.0;
static float fuelAirRatio = 0.0;
static float intakeTemp = 0.0;
static float mpg = 0.0;

/* Private function prototypes -----------------------------------------------*/
float getInputStringSensor(char* inputString, int commandType);
float calculateMPG();

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
    vehicleSpeed = getInputStringSensor(outputString1, 1);

    intakePressure = getInputStringSensor(outputString2, 2);

    engineRPM = getInputStringSensor(outputString3, 3);

    fuelAirRatio = getInputStringSensor(outputString4, 4);

    intakeTemp = getInputStringSensor(outputString5, 5);

    mpg = calculateMPG();
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
    		 "\"mag_x\": %d, \"mag_y\": %d, \"mag_z\": %d,\n"
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
//		   "   \"Vehicle_Speed\": %.2f, \"Intake_Pressure\": %.2f, \"Engine_RPM\": %.2f, \"FA_Ratio\": %.2f"
		   "   \"Vehicle_Speed\": %.2f, \"Engine_RPM\": %.2f, \"MPG\": %.2f"
           "  }\n }\n}",
           TEMPERATURE_Value, HUMIDITY_Value, PRESSURE_Value, PROXIMITY_Value,
           ACC_Value[0], ACC_Value[1], ACC_Value[2],
           GYR_Value[0], GYR_Value[1], GYR_Value[2],
           MAG_Value[0], MAG_Value[1], MAG_Value[2],
		   vehicleSpeed, engineRPM, mpg);	// Values to be replaced with ODB data/calculations
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
float getInputStringSensor(char* inputString, int commandType)
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
    case 5:
      printf("010F\r");
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

  //char *ptr;
 	    char data1[3];
 	    data1[0] = ' ';
 	    data1[1] = ' ';
 	    data1[2] = '\0';
 	    char data2[3];
 	    data2[0] = ' ';
 	    data2[1] = ' ';
 	    data2[2] = '\0';
 	    float f1 = 0.0;
 	    float f2 = 0.0;
 	    float final = 0.0;

 	    //char *inputCopy;

 	    //strcpy(inputCopy, input);

 	    switch(commandType)
 	    {
 	      // 010D: Vehicle Speed
 		  case 1:
			  strncpy(data1, inputString+6, 2);
			  f1 = (float)strtol(data1, NULL, 16);
			  final = f1 * 0.6214;
			  break;
 		  // 010B: Intake manifold absolute pressure
 		  case 2:
			  strncpy(data1, inputString+6, 2);
			  f1 = (float)strtol(data1, NULL, 16);
			  final = f1;
			  break;
 		  // 010C: Engine RPM
 		  case 3:
 			  strncpy(data1, inputString+6, 2);
			  f1 = (float)strtol(data1, NULL, 16);
			  strncpy(data2, inputString+9, 2);
			  f2 = (float)strtol(data2, NULL, 16);
			  final = (256.0*f1 + f2)/4.0;
			  break;
		  // 0144: Fuel-Air commanded equivalence ratio
 		  case 4:
			  strncpy(data1, inputString+6, 2);
			  f1 = (float)strtol(data1, NULL, 16);
			  strncpy(data2, inputString+9, 2);
			  f2 = (float)strtol(data2, NULL, 16);
			  final = (2.0/65536.0)*(256.0*f1+f2);
			  break;
		  // 010F: Intake temperature
 		  case 5:
			  strncpy(data1, inputString+6, 2);
			  f1 = (float)strtol(data1, NULL, 16);
			  final = f1 - 40;
			  break;
 	    }

    return final;
}

float calculateMPG() {

	float calculation = 0.0;
	// volumetric efficiency estimate
	float VE = 72.0;
	// engine displacement
	float ED = 2.0;
	// molecular mass of air constant
	float MM = 28.97;
	// mole constant
	float R = 8.314;

	float IMAP = engineRPM * intakePressure / (intakeTemp+273.15);

	float MAF = (IMAP/120)*(VE/100)*ED*(MM/R);

	calculation = (14.7*6.17*4.54 * vehicleSpeed) / (3600 * MAF/100);

	return calculation;
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
