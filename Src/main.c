/**
 ******************************************************************************
 * @file    main.c
 * @author  MEMS Software Solutions Team
 * @brief   Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <math.h>   /* trunc, pow */
#include <stdio.h>  /* snprintf */
#include "main.h"
#include "com.h"
#include "DemoSerial.h"

/** @addtogroup X_NUCLEO_IKS01A3_Examples X_NUCLEO_IKS01A3 Examples
 * @{
 */

/** @addtogroup DATALOG_EXTENDED DATALOG EXTENDED
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
typedef struct displayFloatToInt_s
{
  int8_t sign; /* 0 means positive, 1 means negative*/
  uint32_t out_int;
  uint32_t out_dec;
} displayFloatToInt_t;

/* Private define ------------------------------------------------------------*/
#define MAX_BUF_SIZE 256

/* Extern variables ----------------------------------------------------------*/
extern volatile uint8_t DataLoggerActive; /*!< DataLogger Flag */
extern UART_HandleTypeDef UartHandle;     /*!< UART HANDLE */
extern int UseLSI;

extern uint32_t AccInstance;
extern uint32_t GyrInstance;
extern uint32_t MagInstance;
extern uint32_t HumInstance;
extern uint32_t TmpInstance;
extern uint32_t PrsInstance;

extern volatile uint32_t SensorsEnabled; /* This "redundant" line is here to fulfil MISRA C-2012 rule 8.4 */
volatile uint32_t SensorsEnabled = 1;    /*!< Enable Sensor Flag */
volatile uint8_t IntStatus = 0;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static int RtcSynchPrediv;
static char DataOut[MAX_BUF_SIZE];            /*!< DataOut Frame */
static RTC_HandleTypeDef RtcHandle;           /*!< RTC HANDLE */
static uint32_t PreviousSensorsEnabled = 0;   /*!< Previously Stored Enable Sensor Flag */
static volatile uint8_t AutoInit = 0;         /*!< Auto Init */
static volatile uint32_t IntCurrentTime1 = 0; /*!< IntCurrentTime1 Value */
static volatile uint32_t IntCurrentTime2 = 0; /*!< IntCurrentTime2 Value */
static uint8_t NewData = 0;
static uint8_t NewDataFlags = 0;

uint8_t yMode, yModeMem = 1;

/* Private function prototypes -----------------------------------------------*/
static void GPIO_Init(void);

static void RTC_Config(void);
static void RTC_TimeStampConfig(void);

static void Initialize_All_Sensors(void);
static void Enable_Disable_Sensors(void);
static void Float_To_Int(float In, displayFloatToInt_t *OutValue, int32_t DecPrec);

static void RTC_Handler(TMsg *Msg);
static void Accelero_Sensor_Handler(TMsg *Msg, uint32_t Instance);
static void Gyro_Sensor_Handler(TMsg *Msg, uint32_t Instance);
static void Magneto_Sensor_Handler(TMsg *Msg, uint32_t Instance);
static void Press_Sensor_Handler(TMsg *Msg, uint32_t Instance);
static void Hum_Sensor_Handler(TMsg *Msg, uint32_t Instance);
static void Temp_Sensor_Handler(TMsg *Msg, uint32_t Instance);
static void Sensors_Interrupt_Handler(TMsg *Msg);
static void MLC_Handler(TMsg *Msg);

/* Private functions ---------------------------------------------------------*/
/**
 * @brief  Main function is to show how to use sensor expansion board to send data from a Nucleo board
 *         using UART to a connected PC or Desktop and display it on generic applications like
 *         TeraTerm and specific application Unicleo-GUI, which is developed by STMicroelectronics
 *         and provided with a separated package.
 *
 *         After connection has been established:
 *         - the user can view the data from various on-board environment sensors like Temperature, Humidity and Pressure
 *         - the user can also view data from various on-board MEMS sensors as well like Accelerometer, Gyroscope and Magnetometer
 *         - the user can also visualize this data as graphs using Unicleo-GUI application
 * @param  None
 * @retval Integer
 */
int main(void); /* This "redundant" line is here to fulfil MISRA C-2012 rule 8.4 */
int main(void)
{
  TMsg msg_dat;
  TMsg msg_cmd;

  /* STM32F4xx HAL library initialization:
  - Configure the Flash prefetch, instruction and Data caches
  - Configure the Systick to generate an interrupt each 1 msec
  - Set NVIC Group Priority to 4
  - Global MSP (MCU Support Package) initialization
  */
  (void)HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize LED */
  BSP_LED_Init(LED2);

  /* Initialize button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);

  /* Initialize GPIO */
  GPIO_Init();

  /* Initialize UART */
  USARTConfig();

  /* Initialize RTC */
  RTC_Config();
  RTC_TimeStampConfig();

  /* Initialize all sensors */
  Initialize_All_Sensors();

  /*   */
//  (void)snprintf(DataOut, MAX_BUF_SIZE, "\n JPO: F401RE_IKS01_DataLog vt/gui " __TIME__ "\n");
//  (void)HAL_UART_Transmit(&UartHandle, (uint8_t *)DataOut, (uint16_t)strlen(DataOut), 5000);

  for (;;)
  {
	/*afficher le mode actif */
	yMode = AutoInit | DataLoggerActive;
	if (yMode != yModeMem)
	{
	  (void)snprintf(DataOut, MAX_BUF_SIZE, "\n JPO fw vt/gui : " yPROG " (" yVER ") "__TIME__ "\n"
											"\tdebug : AutoInit %d / DataLoggerActive %d\r\n", AutoInit, DataLoggerActive);
	  (void)HAL_UART_Transmit(&UartHandle, (uint8_t *)DataOut, (uint16_t)strlen(DataOut), 5000);
	  yModeMem = yMode;
	}

    if (UART_ReceivedMSG((TMsg *)&msg_cmd) != 1)
    {
	 printf("\n---- msg_cmd? %u %u %u %u ---", msg_cmd.Data[0],msg_cmd.Data[1],msg_cmd.Data[2],msg_cmd.Data[3]);
     if (msg_cmd.Data[0] == DEV_ADDR)
      {
        (void)HandleMSG((TMsg *)&msg_cmd);
         if (DataLoggerActive != 0U)
        {
          AutoInit = 0;
        }
      }
    }

    if (AutoInit == 1U)		//yFlag AutoInit, VT mode
    {
      SensorsEnabled = 0xFFFFFFFF;		//select/deselect all sensors
    }

    if (PreviousSensorsEnabled != SensorsEnabled)
    {
      PreviousSensorsEnabled = SensorsEnabled;
      Enable_Disable_Sensors();
    }

    //yFlag Tout le temps (en mode VT & GUI)

    RTC_Handler(&msg_dat);		//yFlag envoyer l'heure VT ou preparer pour GUI

    if ((SensorsEnabled & ACCELEROMETER_SENSOR) == ACCELEROMETER_SENSOR)
    {
      Accelero_Sensor_Handler(&msg_dat, AccInstance);
    }
    if ((SensorsEnabled & GYROSCOPE_SENSOR) == GYROSCOPE_SENSOR)
    {
      Gyro_Sensor_Handler(&msg_dat, GyrInstance);
    }
    if ((SensorsEnabled & MAGNETIC_SENSOR) == MAGNETIC_SENSOR)
    {
      Magneto_Sensor_Handler(&msg_dat, MagInstance);
    }
    if ((SensorsEnabled & HUMIDITY_SENSOR) == HUMIDITY_SENSOR)
    {
      Hum_Sensor_Handler(&msg_dat, HumInstance);
    }
    if ((SensorsEnabled & TEMPERATURE_SENSOR) == TEMPERATURE_SENSOR)
    {
      Temp_Sensor_Handler(&msg_dat, TmpInstance);
    }
    if ((SensorsEnabled & PRESSURE_SENSOR) == PRESSURE_SENSOR)
    {
      Press_Sensor_Handler(&msg_dat, PrsInstance);
    }

    Sensors_Interrupt_Handler(&msg_dat);
    
    if (DataLoggerActive != 0U)
    {
      MLC_Handler(&msg_dat);	//sensor lsm6dsox non présent (reserve). je laisse la ligne mais tout le code en comment!!
    }

    /* To fulfil MISRA C-2012 rule 13.5 following split "redundant" conditions cannot be put together into one using
       || operator */
    if (DataLoggerActive != 0U)
    {
      BSP_LED_Toggle(LED2);
    }
    else if (AutoInit != 0U)	//mode VT
    {
      BSP_LED_Toggle(LED2);
    }
    else
    {
      BSP_LED_Off(LED2);
    }

    if (DataLoggerActive != 0U)	//yFlag DataLoggerActive, Unicleo mode
    {
      if (NewData != 0U)
      {
        INIT_STREAMING_HEADER(&msg_dat);	//ds DemoSerial.c
        msg_dat.Data[55] = NewDataFlags;
        msg_dat.Len = STREAMING_MSG_LENGTH;
        UART_SendMsg(&msg_dat);
        NewData = 0;
        NewDataFlags = 0;
      }
    }

    if (AutoInit != 0U)		//mode VT
    {
      HAL_Delay(3000);
    }
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Interrupt pins X-NUCLEO-IKS01A3 */
  // LSM6DSO INT1                        PB5
  // LSM6DSO INT2                        PB4
  // LPS22HH INT1                        PB10
  // STTS751 INT                         PC1
  // M INT1                              PC0
  // LIS2MDL DRDY/LIS2DW12 INT           PA4
  // LIS2DW12 INT/LIS2MDL DRDY           PB0
  // USER INT                            PA10

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* Configure GPIO pins : PA4 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Configure GPIO pins : PB0 PB4 PB5 PB10*/
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Configure GPIO pins : PC0 PC1*/
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}


/**
 * @brief  Initialize all sensors
 * @param  None
 * @retval None
 */
static void Initialize_All_Sensors(void)
{
  (void)IKS01A3_MOTION_SENSOR_Init(AccInstance, MOTION_ACCELERO);
  (void)IKS01A3_MOTION_SENSOR_Init(GyrInstance, MOTION_GYRO);
  (void)IKS01A3_MOTION_SENSOR_Init(MagInstance, MOTION_MAGNETO);
  (void)IKS01A3_ENV_SENSOR_Init(HumInstance, ENV_HUMIDITY);
  (void)IKS01A3_ENV_SENSOR_Init(TmpInstance, ENV_TEMPERATURE);
  (void)IKS01A3_ENV_SENSOR_Init(PrsInstance, ENV_PRESSURE);
}

/**
 * @brief  Enable/disable desired sensors
 * @param  None
 * @retval None
 */
static void Enable_Disable_Sensors(void)
{
  if ((SensorsEnabled & ACCELEROMETER_SENSOR) == ACCELEROMETER_SENSOR)
  {
    (void)IKS01A3_MOTION_SENSOR_Enable(AccInstance, MOTION_ACCELERO);
  }
  else
  {
    (void)IKS01A3_MOTION_SENSOR_Disable(AccInstance, MOTION_ACCELERO);
  }
  if ((SensorsEnabled & GYROSCOPE_SENSOR) == GYROSCOPE_SENSOR)
  {
    (void)IKS01A3_MOTION_SENSOR_Enable(GyrInstance, MOTION_GYRO);
  }
  else
  {
    (void)IKS01A3_MOTION_SENSOR_Disable(GyrInstance, MOTION_GYRO);
  }
  if ((SensorsEnabled & MAGNETIC_SENSOR) == MAGNETIC_SENSOR)
  {
    (void)IKS01A3_MOTION_SENSOR_Enable(MagInstance, MOTION_MAGNETO);
  }
  else
  {
    (void)IKS01A3_MOTION_SENSOR_Disable(MagInstance, MOTION_MAGNETO);
  }
  if ((SensorsEnabled & HUMIDITY_SENSOR) == HUMIDITY_SENSOR)
  {
    (void)IKS01A3_ENV_SENSOR_Enable(HumInstance, ENV_HUMIDITY);
  }
  else
  {
    (void)IKS01A3_ENV_SENSOR_Disable(HumInstance, ENV_HUMIDITY);
  }
  if ((SensorsEnabled & TEMPERATURE_SENSOR) == TEMPERATURE_SENSOR)
  {
    (void)IKS01A3_ENV_SENSOR_Enable(TmpInstance, ENV_TEMPERATURE);
  }
  else
  {
    (void)IKS01A3_ENV_SENSOR_Disable(TmpInstance, ENV_TEMPERATURE);
  }
  if ((SensorsEnabled & PRESSURE_SENSOR) == PRESSURE_SENSOR)
  {
    (void)IKS01A3_ENV_SENSOR_Enable(PrsInstance, ENV_PRESSURE);
  }
  else
  {
    (void)IKS01A3_ENV_SENSOR_Disable(PrsInstance, ENV_PRESSURE);
  }
}

/**
 * @brief  Splits a float into two integer values
 * @param  In the float value as input
 * @param  OutValue the pointer to the output integer structure
 * @param  DecPrec the decimal precision to be used
 * @retval None
 */
static void Float_To_Int(float In, displayFloatToInt_t *OutValue, int32_t DecPrec)
{
  if (In >= 0.0f)
  {
    OutValue->sign = 0;
  }
  else
  {
    OutValue->sign = 1;
    In = -In;
  }

  OutValue->out_int = (uint32_t)In;
  In = In - (float)(OutValue->out_int);
  OutValue->out_dec = (uint32_t)trunc(In * pow(10.0f, (float)DecPrec));
}

/**
 * @brief  Handles the time+date getting/sending
 * @param  Msg the time+date part of the stream
 * @retval None
 */
static void RTC_Handler(TMsg *Msg)
{
  uint8_t sub_sec;
  uint32_t ans_uint32;
  int32_t ans_int32;
  RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructure;

  if (DataLoggerActive != 0U)		//mode GUI
  {
	/* envoyer data a Unicleo */
    (void)HAL_RTC_GetTime(&RtcHandle, &stimestructure, FORMAT_BIN);
    (void)HAL_RTC_GetDate(&RtcHandle, &sdatestructureget, FORMAT_BIN);

    /* To be MISRA C-2012 compliant the original calculation:
       sub_sec = ((((((int)RtcSynchPrediv) - ((int)stimestructure.SubSeconds)) * 100) / (RtcSynchPrediv + 1)) & 0xFF);
       has been split to separate expressions */
    ans_int32 = (RtcSynchPrediv - (int32_t)stimestructure.SubSeconds) * 100;
    ans_int32 /= RtcSynchPrediv + 1;
    ans_uint32 = (uint32_t)ans_int32 & 0xFFU;
    sub_sec = (uint8_t)ans_uint32;

    Msg->Data[3] = (uint8_t)stimestructure.Hours;
    Msg->Data[4] = (uint8_t)stimestructure.Minutes;
    Msg->Data[5] = (uint8_t)stimestructure.Seconds;
    Msg->Data[6] = sub_sec;
  }
  else if (AutoInit != 0U)		//mode VT
  {
    /* envoyer temps et valeurs a la VT */
	(void)HAL_RTC_GetTime(&RtcHandle, &stimestructure, FORMAT_BIN);
    (void)HAL_RTC_GetDate(&RtcHandle, &sdatestructureget, FORMAT_BIN);

    /* To be MISRA C-2012 compliant the original calculation:
       sub_sec = ((((((int)RtcSynchPrediv) - ((int)stimestructure.SubSeconds)) * 100) / (RtcSynchPrediv + 1)) & 0xFF);
       has been split to separate expressions */
    ans_int32 = (RtcSynchPrediv - (int32_t)stimestructure.SubSeconds) * 100;
    ans_int32 /= RtcSynchPrediv + 1;
    ans_uint32 = (uint32_t)ans_int32 & 0xFFU;
    sub_sec = (uint8_t)ans_uint32;

    (void)snprintf(DataOut, MAX_BUF_SIZE, "\nTimeStamp: %d:%d:%d.%d\r\n",
    						stimestructure.Hours, stimestructure.Minutes, stimestructure.Seconds, sub_sec);
    (void)HAL_UART_Transmit(&UartHandle, (uint8_t *)DataOut, (uint16_t)strlen(DataOut), 5000);
  }
  else
  {
    /* Nothing to do */
  }
}

/**
 * @brief  Handles the ACCELERO axes data getting/sending
 * @param  Msg the ACCELERO part of the stream
 * @param  Instance the device instance
 * @retval None
 */
static void Accelero_Sensor_Handler(TMsg *Msg, uint32_t Instance)
{
  int32_t data[6];
  IKS01A3_MOTION_SENSOR_Axes_t acceleration;
  uint8_t status = 0;

  if (IKS01A3_MOTION_SENSOR_Get_DRDY_Status(Instance, MOTION_ACCELERO, &status) == BSP_ERROR_NONE && status == 1U)
  {
    NewData++;
    NewDataFlags |= 1U;

    (void)IKS01A3_MOTION_SENSOR_GetAxes(Instance, MOTION_ACCELERO, &acceleration);

    if (DataLoggerActive != 0U)
    {
      Serialize_s32(&Msg->Data[19], acceleration.x, 4);
      Serialize_s32(&Msg->Data[23], acceleration.y, 4);
      Serialize_s32(&Msg->Data[27], acceleration.z, 4);
    }
    else if (AutoInit != 0U)
    {
      data[0] = acceleration.x;
      data[1] = acceleration.y;
      data[2] = acceleration.z;

      (void)snprintf(DataOut, MAX_BUF_SIZE, "ACC_X: %d, ACC_Y: %d, ACC_Z: %d\r\n", (int)data[0], (int)data[1], (int)data[2]);
      (void)HAL_UART_Transmit(&UartHandle, (uint8_t *)DataOut, (uint16_t)strlen(DataOut), 5000);
    }
    else
    {
      /* Nothing to do */
    }
  }
}

/**
 * @brief  Handles the GYRO axes data getting/sending
 * @param  Msg the GYRO part of the stream
 * @param  Instance the device instance
 * @retval None
 */
static void Gyro_Sensor_Handler(TMsg *Msg, uint32_t Instance)
{
  int32_t data[6];
  IKS01A3_MOTION_SENSOR_Axes_t angular_velocity;
  uint8_t status = 0;

  if (IKS01A3_MOTION_SENSOR_Get_DRDY_Status(Instance, MOTION_GYRO, &status) == BSP_ERROR_NONE && status == 1U)
  {
    NewData++;
    NewDataFlags |= 2U;

    (void)IKS01A3_MOTION_SENSOR_GetAxes(Instance, MOTION_GYRO, &angular_velocity);

    if (DataLoggerActive != 0U)
    {
      Serialize_s32(&Msg->Data[31], angular_velocity.x, 4);
      Serialize_s32(&Msg->Data[35], angular_velocity.y, 4);
      Serialize_s32(&Msg->Data[39], angular_velocity.z, 4);
    }
    else if (AutoInit != 0U)
    {
      data[0] = angular_velocity.x;
      data[1] = angular_velocity.y;
      data[2] = angular_velocity.z;

      (void)snprintf(DataOut, MAX_BUF_SIZE, "GYR_X: %d, GYR_Y: %d, GYR_Z: %d\r\n", (int)data[0], (int)data[1], (int)data[2]);
      (void)HAL_UART_Transmit(&UartHandle, (uint8_t *)DataOut, (uint16_t)strlen(DataOut), 5000);
    }
    else
    {
      /* Nothing to do */
    }
  }
}

/**
 * @brief  Handles the MAGNETO axes data getting/sending
 * @param  Msg the MAGNETO part of the stream
 * @param  Instance the device instance
 * @retval None
 */
static void Magneto_Sensor_Handler(TMsg *Msg, uint32_t Instance)
{
  int32_t data[3];
  IKS01A3_MOTION_SENSOR_Axes_t magnetic_field;
  uint8_t status = 0;

  if (IKS01A3_MOTION_SENSOR_Get_DRDY_Status(Instance, MOTION_MAGNETO, &status) == BSP_ERROR_NONE && status == 1U)
  {
    NewData++;
    NewDataFlags |= 4U;

    (void)IKS01A3_MOTION_SENSOR_GetAxes(Instance, MOTION_MAGNETO, &magnetic_field);

    if (DataLoggerActive != 0U)
    {
      Serialize_s32(&Msg->Data[43], (int32_t)magnetic_field.x, 4);
      Serialize_s32(&Msg->Data[47], (int32_t)magnetic_field.y, 4);
      Serialize_s32(&Msg->Data[51], (int32_t)magnetic_field.z, 4);
    }
    else if (AutoInit != 0U)
    {
      data[0] = magnetic_field.x;
      data[1] = magnetic_field.y;
      data[2] = magnetic_field.z;

      (void)snprintf(DataOut, MAX_BUF_SIZE, "MAG_X: %d, MAG_Y: %d, MAG_Z: %d\r\n", (int)data[0], (int)data[1], (int)data[2]);
      (void)HAL_UART_Transmit(&UartHandle, (uint8_t *)DataOut, (uint16_t)strlen(DataOut), 5000);
    }
    else
    {
      /* Nothing to do */
    }
  }
}

/**
 * @brief  Handles the PRESSURE sensor data getting/sending
 * @param  Msg the PRESSURE part of the stream
 * @param  Instance the device instance
 * @retval None
 */
static void Press_Sensor_Handler(TMsg *Msg, uint32_t Instance)
{
  float pressure;
  uint8_t status = 0;

  if (IKS01A3_ENV_SENSOR_Get_DRDY_Status(Instance, ENV_PRESSURE, &status) == BSP_ERROR_NONE && status == 1U)
  {
    NewData++;
    NewDataFlags |= 8U;

    (void)IKS01A3_ENV_SENSOR_GetValue(Instance, ENV_PRESSURE, &pressure);

    if (DataLoggerActive != 0U)
    {
      (void)memcpy(&Msg->Data[7], (void *)&pressure, sizeof(float));
    }
    else if (AutoInit != 0U)
    {
      displayFloatToInt_t out_value;
      Float_To_Int(pressure, &out_value, 2);
      (void)snprintf(DataOut, MAX_BUF_SIZE, "PRESS: %d.%02d\r\n", (int)out_value.out_int, (int)out_value.out_dec);
      (void)HAL_UART_Transmit(&UartHandle, (uint8_t *)DataOut, (uint16_t)strlen(DataOut), 5000);
    }
    else
    {
      /* Nothing to do */
    }
  }
}

/**
 * @brief  Handles the HUMIDITY sensor data getting/sending
 * @param  Msg the HUMIDITY part of the stream
 * @param  Instance the device instance
 * @retval None
 */
static void Hum_Sensor_Handler(TMsg *Msg, uint32_t Instance)
{
  float humidity;
  uint8_t status = 0;

  if (IKS01A3_ENV_SENSOR_Get_DRDY_Status(Instance, ENV_HUMIDITY, &status) == BSP_ERROR_NONE && status == 1U)
  {
    NewData++;
    NewDataFlags |= 16U;

    (void)IKS01A3_ENV_SENSOR_GetValue(Instance, ENV_HUMIDITY, &humidity);

    if (DataLoggerActive != 0U)
    {
      (void)memcpy(&Msg->Data[15], (void *)&humidity, sizeof(float));
    }
    else if (AutoInit != 0U)
    {
      displayFloatToInt_t out_value;
      Float_To_Int(humidity, &out_value, 2);
      (void)snprintf(DataOut, MAX_BUF_SIZE, "HUM: %d.%02d\r\n", (int)out_value.out_int, (int)out_value.out_dec);
      (void)HAL_UART_Transmit(&UartHandle, (uint8_t *)DataOut, (uint16_t)strlen(DataOut), 5000);
    }
    else
    {
      /* Nothing to do */
    }
  }
}

/**
 * @brief  Handles the TEMPERATURE sensor data getting/sending
 * @param  Msg the TEMPERATURE part of the stream
 * @param  Instance the device instance
 * @retval None
 */
static void Temp_Sensor_Handler(TMsg *Msg, uint32_t Instance)
{
  float temperature;
  uint8_t status = 0;
  uint8_t drdy = 0;
  static uint8_t stts751_is_busy = 0;

  if (Instance == IKS01A3_STTS751_0)
  {
    if (IKS01A3_ENV_SENSOR_Get_DRDY_Status(Instance, ENV_TEMPERATURE, &status) == BSP_ERROR_NONE)
    {
      if (status == 0)
      {
        stts751_is_busy = 1;
        drdy = 0;
      }
      else
      {
        if (stts751_is_busy == 1)
        {
          stts751_is_busy = 0;
          drdy = 1;
        }
      }
    }
  }
  else
  {
    if (IKS01A3_ENV_SENSOR_Get_DRDY_Status(Instance, ENV_TEMPERATURE, &status) == BSP_ERROR_NONE && status == 1U)
    {
      drdy = 1;
    }
    else
    {
      drdy = 0;
    }
  }

  if (drdy == 1)
  {
    NewData++;
    NewDataFlags |= 32U;

    (void)IKS01A3_ENV_SENSOR_GetValue(Instance, ENV_TEMPERATURE, &temperature);

    if (DataLoggerActive != 0U)		//mode GUI
    {
      (void)memcpy(&Msg->Data[11], (void *)&temperature, sizeof(float));
    }
    else if (AutoInit != 0U)		//mode VT
    {
      displayFloatToInt_t out_value;
      Float_To_Int(temperature, &out_value, 2);
      (void)snprintf(DataOut, MAX_BUF_SIZE, "TEMP: %c%d.%02d\r\n", ((out_value.sign != 0) ? '-' : '+'),
                     (int)out_value.out_int, (int)out_value.out_dec);
      (void)HAL_UART_Transmit(&UartHandle, (uint8_t *)DataOut, (uint16_t)strlen(DataOut), 5000);
    }
    else
    {
      /* Nothing to do */
    }
  }
}

/**
 * @brief  Handles the sensors interrupts
 * @param  Msg the INTERRUPT part of the stream
 * @retval None
 */
static void Sensors_Interrupt_Handler(TMsg *Msg)
{
  static uint8_t mem_int_status = 0;

  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_SET) IntStatus |= (1 << 0); else IntStatus &= ~(1 << 0);
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_SET) IntStatus |= (1 << 1); else IntStatus &= ~(1 << 1);
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET) IntStatus |= (1 << 2); else IntStatus &= ~(1 << 2);
  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == GPIO_PIN_SET) IntStatus |= (1 << 3); else IntStatus &= ~(1 << 3);
  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_SET) IntStatus |= (1 << 4); else IntStatus &= ~(1 << 4);
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET) IntStatus |= (1 << 5); else IntStatus &= ~(1 << 5);
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_SET) IntStatus |= (1 << 6); else IntStatus &= ~(1 << 6);
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_SET) IntStatus |= (1 << 7); else IntStatus &= ~(1 << 7);

  if (mem_int_status != IntStatus)
  {
    NewData++;
    NewDataFlags |= 64U;
    Msg->Data[56] = IntStatus;
    mem_int_status = IntStatus;
  }
}

/*
 * @brief  Handles the MLC status data
 * @param  Msg the MLC part of the stream
 * @retval None
*/
static void MLC_Handler(TMsg *Msg)
{
  /*if ((AccInstance == IKS01A3_LSM6DSOX_0) && (GyrInstance == IKS01A3_LSM6DSOX_0))
  {
    (void)IKS01A3_MOTION_SENSOR_Write_Register(IKS01A3_LSM6DSOX_0, LSM6DSOX_FUNC_CFG_ACCESS, LSM6DSOX_EMBEDDED_FUNC_BANK << 6);

    (void)IKS01A3_MOTION_SENSOR_Read_Register(IKS01A3_LSM6DSOX_0, LSM6DSOX_MLC0_SRC, &Msg->Data[57]);
    (void)IKS01A3_MOTION_SENSOR_Read_Register(IKS01A3_LSM6DSOX_0, LSM6DSOX_MLC1_SRC, &Msg->Data[58]);
    (void)IKS01A3_MOTION_SENSOR_Read_Register(IKS01A3_LSM6DSOX_0, LSM6DSOX_MLC2_SRC, &Msg->Data[59]);
    (void)IKS01A3_MOTION_SENSOR_Read_Register(IKS01A3_LSM6DSOX_0, LSM6DSOX_MLC3_SRC, &Msg->Data[60]);
    (void)IKS01A3_MOTION_SENSOR_Read_Register(IKS01A3_LSM6DSOX_0, LSM6DSOX_MLC4_SRC, &Msg->Data[61]);
    (void)IKS01A3_MOTION_SENSOR_Read_Register(IKS01A3_LSM6DSOX_0, LSM6DSOX_MLC5_SRC, &Msg->Data[62]);
    (void)IKS01A3_MOTION_SENSOR_Read_Register(IKS01A3_LSM6DSOX_0, LSM6DSOX_MLC6_SRC, &Msg->Data[63]);
    (void)IKS01A3_MOTION_SENSOR_Read_Register(IKS01A3_LSM6DSOX_0, LSM6DSOX_MLC7_SRC, &Msg->Data[64]);

    (void)IKS01A3_MOTION_SENSOR_Write_Register(IKS01A3_LSM6DSOX_0, LSM6DSOX_FUNC_CFG_ACCESS, LSM6DSOX_USER_BANK << 6);
  }*/
	__NOP();
}

/**
 * @brief  Configures the RTC
 * @param  None
 * @retval None
 */
static void RTC_Config(void)
{
  /*##-1- Configure the RTC peripheral #######################################*/
  /* Check if LSE can be used */
  RCC_OscInitTypeDef rcc_osc_init_struct;

  /*##-2- Configure LSE as RTC clock soucre ###################################*/
  rcc_osc_init_struct.OscillatorType =  RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
  rcc_osc_init_struct.PLL.PLLState = RCC_PLL_NONE;
  rcc_osc_init_struct.LSEState = RCC_LSE_ON;
  rcc_osc_init_struct.LSIState = RCC_LSI_OFF;
  if (HAL_RCC_OscConfig(&rcc_osc_init_struct) != HAL_OK)
  {
    /* LSE not available, we use LSI */
    UseLSI = 1;
    RtcHandle.Init.AsynchPrediv = RTC_ASYNCH_PREDIV_LSI;
    RtcHandle.Init.SynchPrediv = RTC_SYNCH_PREDIV_LSI;
    RtcSynchPrediv = RTC_SYNCH_PREDIV_LSI;
  }
  else
  {
    /* We use LSE */
    UseLSI = 0;
    RtcHandle.Init.AsynchPrediv = RTC_ASYNCH_PREDIV_LSE;
    RtcHandle.Init.SynchPrediv = RTC_SYNCH_PREDIV_LSE;
    RtcSynchPrediv = RTC_SYNCH_PREDIV_LSE;
  }
  RtcHandle.Instance = RTC;

  /* Configure RTC prescaler and RTC data registers */
  /* RTC configured as follow:
  - Hour Format    = Format 24
  - Asynch Prediv  = Value according to source clock
  - Synch Prediv   = Value according to source clock
  - OutPut         = Output Disable
  - OutPutPolarity = High Polarity
  - OutPutType     = Open Drain */
  RtcHandle.Init.HourFormat = RTC_HOURFORMAT_24;
  RtcHandle.Init.OutPut = RTC_OUTPUT_DISABLE;
  RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  RtcHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;

  if (HAL_RTC_Init(&RtcHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

/**
 * @brief  Configures the current time and date
 * @param  None
 * @retval None
 */
static void RTC_TimeStampConfig(void)
{
  RTC_DateTypeDef sdatestructure;
  RTC_TimeTypeDef stimestructure;

  /*##-3- Configure the Date #################################################*/
  /* Set Date: Monday January 1st 2000 */
  sdatestructure.Year = 0x00;
  sdatestructure.Month = RTC_MONTH_JANUARY;
  sdatestructure.Date = 0x01;
  sdatestructure.WeekDay = RTC_WEEKDAY_MONDAY;

  if (HAL_RTC_SetDate(&RtcHandle, &sdatestructure, FORMAT_BCD) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-4- Configure the Time #################################################*/
  /* Set Time: 00:00:00 */
  stimestructure.Hours = 0x00;
  stimestructure.Minutes = 0x00;
  stimestructure.Seconds = 0x00;
  stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

  if (HAL_RTC_SetTime(&RtcHandle, &stimestructure, FORMAT_BCD) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

/**
 * @brief  EXTI line detection callbacks
 * @param  GPIOPin the pin connected to EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIOPin)
{
  int do_operation = 0;
  uint32_t int_current_time2_local;

  if (GPIOPin == KEY_BUTTON_PIN)		//button B1 blue
  {
    int_current_time2_local = IntCurrentTime2;

    /* Manage software debouncing*/
    if (IntCurrentTime1 == 0U && int_current_time2_local == 0U)
    {
      IntCurrentTime1 = user_currentTimeGetTick();
      do_operation = 1;
    }
    else
    {
      IntCurrentTime2 = user_currentTimeGetTick();
      int_current_time2_local = IntCurrentTime2;

      /* If I receive a button interrupt after more than 300 ms from the first one I get it, otherwise I discard it */
      if ((int_current_time2_local - IntCurrentTime1) > 300U)
      {
        IntCurrentTime1 = IntCurrentTime2;
        do_operation = 1;
      }
    }

    if (do_operation != 0)
    {
      if (DataLoggerActive != 0U)	/* mode GUI actif */
      {
        AutoInit = 0; /* always off, oublier le BP1! */
      }
      else
      {
        AutoInit = (AutoInit != 0U) ? 0U : 1U; /* toggle on each button pressed */
      }
    }
  }
}

/**
 * @brief  Configures the current time and date
 * @param  hh the hour value to be set
 * @param  mm the minute value to be set
 * @param  ss the second value to be set
 * @retval None
 */
void RTC_TimeRegulate(uint8_t hh, uint8_t mm, uint8_t ss)
{
  RTC_TimeTypeDef stimestructure;

  stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
  stimestructure.Hours = hh;
  stimestructure.Minutes = mm;
  stimestructure.Seconds = ss;
  stimestructure.SubSeconds = 0;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

  if (HAL_RTC_SetTime(&RtcHandle, &stimestructure, FORMAT_BIN) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

/**
 * @brief  Get the current tick value in millisecond
 * @param  None
 * @retval The tick value
 */
uint32_t user_currentTimeGetTick(void)
{
  return HAL_GetTick();
}

/**
 * @brief  Get the delta tick value in millisecond from Tick1 to the current tick
 * @param  Tick1 the reference tick used to compute the delta
 * @retval The delta tick value
 */
uint32_t user_currentTimeGetElapsedMS(uint32_t Tick1)
{
  volatile uint32_t delta;
  volatile uint32_t tick2;

  tick2 = HAL_GetTick();

  /* Capture computation */
  delta = tick2 - Tick1;
  return delta;
}

/**
 * @brief  This function is executed in case of error occurrence
 * @param  None
 * @retval None
 */
void Error_Handler(void)
{
  if (AutoInit != 0U)
  {
    (void)snprintf(DataOut, MAX_BUF_SIZE, "Error");
    (void)HAL_UART_Transmit(&UartHandle, (uint8_t *)DataOut, (uint16_t)strlen(DataOut), 5000);
  }

  for (;;)
  {
    BSP_LED_On(LED2);
    HAL_Delay(100);
    BSP_LED_Off(LED2);
    HAL_Delay(100);
  }
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number where the assert_param error has occurred
 * @param  file pointer to the source file name
 * @param  line assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  for (;;)
  {}
}
#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
