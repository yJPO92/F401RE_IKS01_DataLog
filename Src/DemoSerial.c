/**
 ******************************************************************************
 * @file    DemoSerial.c
 * @author  MEMS Software Solutions Team
 * @brief   Handle the Serial Protocol
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
#include "DemoSerial.h"
#include "main.h"
#include "com.h"
#include "sensor_commands.h"
#include <stdio.h>

/** @addtogroup X_NUCLEO_IKS01A3_Examples X_NUCLEO_IKS01A3 Examples
 * @{
 */

/** @addtogroup DATALOG_EXTENDED DATALOG EXTENDED
 * @{
 */

/* Extern variables ----------------------------------------------------------*/
extern volatile uint32_t SensorsEnabled;

/* Exported variables ---------------------------------------------------------*/
extern volatile uint8_t DataLoggerActive; /* This "redundant" line is here to fulfil MISRA C-2012 rule 8.4 */
volatile uint8_t DataLoggerActive;

/* Private variables ---------------------------------------------------------*/
#define FW_ID "101"					//a priori pour dire data logger
#define FW_VERSION "6.2.0"			//version des exemlpes MEMS
#define LIB_VERSION yVER			//"1.1.4" ou plus voir main.h
#define EXPANSION_BOARD "IKS01A3"	//la carte!

static uint8_t PresentationString[] = {"MEMS shield demo,"FW_ID","FW_VERSION","LIB_VERSION","EXPANSION_BOARD};
//static uint8_t PresentationString[] = {"MEMS shield demo,101,6.2.0,1.0.0,IKS01A3"};
//static volatile uint8_t DataStreamingDest = 1;
static volatile uint8_t DataStreamingDest = 2;

/**
 * @brief  Build the reply header
 * @param  Msg the pointer to the message to be built
 * @retval None
 */
void BUILD_REPLY_HEADER(TMsg *Msg)
{
  Msg->Data[0] = Msg->Data[1];
  Msg->Data[1] = DEV_ADDR;
  Msg->Data[2] += CMD_Reply_Add;
}

/**
 * @brief  Build the nack header
 * @param  Msg the pointer to the message to be built
 * @retval None
 */
void BUILD_NACK_HEADER(TMsg *Msg)
{
  Msg->Data[0] = Msg->Data[1];
  Msg->Data[1] = DEV_ADDR;
  Msg->Data[2] = CMD_NACK;
}

/**
 * @brief  Initialize the streaming header
 * @param  Msg the pointer to the header to be initialized
 * @retval None
 */
void INIT_STREAMING_HEADER(TMsg *Msg)
{
  Msg->Data[0] = DataStreamingDest;
  Msg->Data[1] = DEV_ADDR;
  Msg->Data[2] = CMD_Start_Data_Streaming;
  Msg->Len = 3;
}

/**
 * @brief  Initialize the streaming message
 * @param  Msg the pointer to the message to be initialized
 * @retval None
 */
void INIT_STREAMING_MSG(TMsg *Msg)
{
  uint32_t i;

  Msg->Data[0] = DataStreamingDest;
  Msg->Data[1] = DEV_ADDR;
  Msg->Data[2] = CMD_Start_Data_Streaming;
  for (i = 3; i < STREAMING_MSG_LENGTH + 3; i++)
  {
    Msg->Data[i] = 0;
  }
  Msg->Len = 3;
}

/**
 * @brief  Handle a message
 * @param  Msg the pointer to the message to be handled
 * @retval 1 if the message is correctly handled, 0 otherwise
 */
int HandleMSG(TMsg *Msg)
/*  DestAddr | SouceAddr | CMD | SUBCMD | PAYLOAD
        1          1        1       1        N    */
{
  uint32_t i;
  int ret = 1;

  if (Msg->Len < 2U)
  {
    return 0;
  }
  if (Msg->Data[0] != DEV_ADDR)
  {
    return 0;
  }
  switch (Msg->Data[2])   /* CMD */
  {
    case CMD_Ping:
      printf("\n\t--Handle: ping");
      if (Msg->Len != 3U)
      {
        return 0;
      }
      BUILD_REPLY_HEADER(Msg);
      Msg->Len = 3;
      UART_SendMsg(Msg);
      break;

    case CMD_Enter_DFU_Mode:
      if (Msg->Len != 3U)
      {
        return 0;
      }
      BUILD_REPLY_HEADER(Msg);
      Msg->Len = 3;
      break;

    case CMD_Read_PresString:
      //printf("\n\t--Handle: presString");
      //printf("\n---msg data: %u %u %u", Msg->Data[0],Msg->Data[1],Msg->Data[2]);
      if (Msg->Len != 3U)
      {
        return 0;
      }
      BUILD_REPLY_HEADER(Msg);
      i = 0;
      while (i < (sizeof(PresentationString) - 1U))
      {
        Msg->Data[3U + i] = PresentationString[i];
        i++;
      }
      Msg->Len = 3U + i;
      UART_SendMsg(Msg);
      break;

    case CMD_CheckModeSupport:
      if (Msg->Len < 3U)
      {
        return 0;
      }
      BUILD_REPLY_HEADER(Msg);
      Serialize_s32(&Msg->Data[3], DATALOG_EXT_MODE, 4);
      Msg->Len = 3 + 4;
      UART_SendMsg(Msg);
      break;

    case CMD_Sensor:
      /* Check if the command lenght is at least 5 bytes */
      printf("\n---cmdSensor: %x %x %x %x %x", Msg->Data[0],Msg->Data[1],Msg->Data[2],Msg->Data[3],Msg->Data[4]);
      if (Msg->Len < 5U)
      {
        return 0;
      }
      (void)Handle_Sensor_command(Msg);
      break;

    case CMD_Start_Data_Streaming:
      if (Msg->Len < 3U)
      {
        return 0;
      }
      SensorsEnabled = Deserialize(&Msg->Data[3], 4);
      DataLoggerActive = 1;
      DataStreamingDest = Msg->Data[1];
      BUILD_REPLY_HEADER(Msg);
      Msg->Len = 3;
      UART_SendMsg(Msg);
      break;

    case CMD_Stop_Data_Streaming:
      if (Msg->Len < 3U)
      {
        return 0;
      }
      SensorsEnabled = 0;
      DataLoggerActive = 0;
      BUILD_REPLY_HEADER(Msg);
      UART_SendMsg(Msg);
      break;

    case CMD_Set_DateTime:
      if (Msg->Len < 3U)
      {
        return 0;
      }
      BUILD_REPLY_HEADER(Msg);
      Msg->Len = 3;
      RTC_TimeRegulate(Msg->Data[3], Msg->Data[4], Msg->Data[5]);
      UART_SendMsg(Msg);
      break;

    default:
      ret = 0;
      break;
  }

  return ret;
}

/**
 * @}
 */

/**
 * @}
 */
