/**
  ******************************************************************************
  * @file    EEPROM_Emulation/inc/eeprom.h 
  * @author  MCD Application Team & Goran Jurkovic	
  * @version V3.1.1
  * @date    07/18/2017
  * @brief   This file contains all the functions prototypes for the EEPROM 
  *          emulation firmware library.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  * <h2><centar>Modified by goran.jurkovic.zg@gmail.com</center></h2>
  *
  * No virtual addresses bullshit, just address from 1 to number of variables.
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EEPROM_H
#define __EEPROM_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx.h"
#include "stm32f1xx_hal_conf.h"
#include "stm32f1xx_hal_flash.h"

#define FLASH_Status				HAL_StatusTypeDef
#define FLASH_COMPLETE				HAL_OK

/* Exported constants --------------------------------------------------------*/
/* Define the STM32F10Xxx Flash page size depending on the used STM32 device */
#if defined (STM32F10X_LD) || defined (STM32F10X_MD)
  #define PAGE_SIZE  (uint16_t)0x400  /* Page size = 1KByte */
#elif defined (STM32F10X_HD) || defined (STM32F10X_CL)
  #define PAGE_SIZE  (uint16_t)0x800  /* Page size = 2KByte */
#endif

/* EEPROM start address in Flash */
#define EEPROM_START_ADDRESS    (0x08010000) /* EEPROM emulation start address:
                                                  after 64KByte of used Flash memory */

/* Pages 0 and 1 base and end addresses */
#define PAGE0_BASE_ADDRESS      (EEPROM_START_ADDRESS)
#define PAGE0_END_ADDRESS       ((uint32_t)(EEPROM_START_ADDRESS + (PAGE_SIZE - 1)))

#define PAGE1_BASE_ADDRESS      ((uint32_t)(EEPROM_START_ADDRESS + PAGE_SIZE))
#define PAGE1_END_ADDRESS       ((uint32_t)(EEPROM_START_ADDRESS + (2 * PAGE_SIZE - 1)))

/* Used Flash pages for EEPROM emulation */
#define PAGE0                   (0x0000)
#define PAGE1                   (0x0001)

/* No valid page define */
#define NO_VALID_PAGE           ((uint16_t)0x00AB)

/* Page status definitions */
#define ERASED_PAGE				(0xFFFF)     /* PAGE is empty */
#define RECEIVE_DATA_PAGE		(0xEEEE)     /* PAGE is marked to receive data */
#define VALID_PAGE				(0x0000)     /* PAGE containing valid data */

/* Valid pages in read and write defines */
#define READ_FROM_VALID_PAGE    ((uint8_t)0x00)
#define WRITE_IN_VALID_PAGE     ((uint8_t)0x01)

/* Page full define */
#define PAGE_FULL               ((uint8_t)0x80)

/* Variables' number */
#define NumbOfVar               ((uint8_t)22)

/* Exported types ------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/* Address goes from 1 to NumbOfVar */
uint16_t EE_Init(void);
uint16_t EE_ReadVariable(uint16_t Address, uint16_t* Data);
uint16_t EE_WriteVariable(uint16_t Address, uint16_t Data);

#endif /* __EEPROM_H */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
