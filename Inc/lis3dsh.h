#ifndef LIS3DSH_H_

#define LIS3DSH_H_

 

#include "stm32f4xx_hal.h"

#include <string.h>

//————————————————

#define ABS(x)         (x < 0) ? (-x) : x

//————————————————

#define LD_PORT GPIOD

#define LD3 GPIO_PIN_13 //ORANGE

#define LD4 GPIO_PIN_12 //GREEN

#define LD5 GPIO_PIN_14 //RED

#define LD6 GPIO_PIN_15 //BLUE

#define LD3_ON HAL_GPIO_WritePin(LD_PORT, LD3, GPIO_PIN_SET) //ORANGE

#define LD4_ON HAL_GPIO_WritePin(LD_PORT, LD4, GPIO_PIN_SET) //GREEN

#define LD5_ON HAL_GPIO_WritePin(LD_PORT, LD5, GPIO_PIN_SET) //RED

#define LD6_ON HAL_GPIO_WritePin(LD_PORT, LD6, GPIO_PIN_SET) //BLUE

#define LD3_OFF HAL_GPIO_WritePin(LD_PORT, LD3, GPIO_PIN_RESET) //ORANGE

#define LD4_OFF HAL_GPIO_WritePin(LD_PORT, LD4, GPIO_PIN_RESET) //GREEN

#define LD5_OFF HAL_GPIO_WritePin(LD_PORT, LD5, GPIO_PIN_RESET) //RED

#define LD6_OFF HAL_GPIO_WritePin(LD_PORT, LD6, GPIO_PIN_RESET) //BLUE

//————————————————
#define CS_GPIO_PORT GPIOE

#define CS_PIN GPIO_PIN_3

#define CS_ON HAL_GPIO_WritePin(CS_GPIO_PORT, CS_PIN, GPIO_PIN_RESET)

#define CS_OFF HAL_GPIO_WritePin(CS_GPIO_PORT, CS_PIN, GPIO_PIN_SET)

#define DUMMY_BYTE                        ((uint8_t)0x00)

//————————————————

#define LIS3DSH_WHO_AM_I_ADDR                0x0F

//————————————————

#define READWRITE_CMD                     ((uint8_t)0x80)

#define MULTIPLEBYTE_CMD                  ((uint8_t)0x40)

//————————————————
#define LIS3DSH_CTRL_REG4_ADDR               0x20
#define LIS3DSH_CTRL_REG5_ADDR               0x24
//------------------------------------------------
#define LIS3DSH_DATARATE_POWERDOWN           ((uint8_t)0x00)  /* Power Down Mode*/
#define LIS3DSH_DATARATE_3_125               ((uint8_t)0x10)  /* 3.125 Hz Normal Mode */
#define LIS3DSH_DATARATE_6_25                ((uint8_t)0x20)  /* 6.25  Hz Normal Mode */
#define LIS3DSH_DATARATE_12_5                ((uint8_t)0x30)  /* 12.5  Hz Normal Mode */
#define LIS3DSH_DATARATE_25                  ((uint8_t)0x40)  /* 25    Hz Normal Mode */
#define LIS3DSH_DATARATE_50                  ((uint8_t)0x50)  /* 50    Hz Normal Mode */
#define LIS3DSH_DATARATE_100                 ((uint8_t)0x60)  /* 100   Hz Normal Mode */
#define LIS3DSH_DATARATE_400                 ((uint8_t)0x70)  /* 400   Hz Normal Mode */
#define LIS3DSH_DATARATE_800                 ((uint8_t)0x80)  /* 800   Hz Normal Mode */
#define LIS3DSH_DATARATE_1600                ((uint8_t)0x90)  /* 1600  Hz Normal Mode */
//------------------------------------------------
#define LIS3DSH_X_ENABLE                     ((uint8_t)0x01)
#define LIS3DSH_Y_ENABLE                     ((uint8_t)0x02)
#define LIS3DSH_Z_ENABLE                     ((uint8_t)0x04)
#define LIS3DSH_XYZ_ENABLE                   ((uint8_t)0x07)
//------------------------------------------------
#define LIS3DSH_SERIALINTERFACE_4WIRE        ((uint8_t)0x00)
#define LIS3DSH_SERIALINTERFACE_3WIRE        ((uint8_t)0x01)
//------------------------------------------------
#define LIS3DSH_SELFTEST_NORMAL              ((uint8_t)0x00)
#define LIS3DSH_SELFTEST_P                   ((uint8_t)0x02)
#define LIS3DSH_SELFTEST_M                   ((uint8_t)0x04)
//------------------------------------------------
#define LIS3DSH_FULLSCALE_2                  ((uint8_t)0x00)  /* 2 g  */
#define LIS3DSH_FULLSCALE_4                  ((uint8_t)0x08)  /* 4 g  */
#define LIS3DSH_FULLSCALE_6                  ((uint8_t)0x10)  /* 6 g  */
#define LIS3DSH_FULLSCALE_8                  ((uint8_t)0x18)  /* 8 g  */
#define LIS3DSH_FULLSCALE_16                 ((uint8_t)0x20)  /* 16 g */
#define LIS3DSH__FULLSCALE_SELECTION         ((uint8_t)0x38)
//------------------------------------------------
#define LIS3DSH_FILTER_BW_800                ((uint8_t)0x00)  /* 800 Hz */
#define LIS3DSH_FILTER_BW_40                 ((uint8_t)0x08)  /* 40 Hz  */
#define LIS3DSH_FILTER_BW_200                ((uint8_t)0x10)  /* 200 Hz */
#define LIS3DSH_FILTER_BW_50                 ((uint8_t)0x18)  /* 50 Hz  */
//------------------------------------------------
#define LIS3DSH_SENSITIVITY_0_06G            0.06  /* 0.06 mg/digit*/
#define LIS3DSH_SENSITIVITY_0_12G            0.12  /* 0.12 mg/digit*/
#define LIS3DSH_SENSITIVITY_0_18G            0.18  /* 0.18 mg/digit*/
#define LIS3DSH_SENSITIVITY_0_24G            0.24  /* 0.24 mg/digit*/
#define LIS3DSH_SENSITIVITY_0_73G            0.73  /* 0.73 mg/digit*/
//------------------------------------------------
#define LIS3DSH_OUT_X_L_ADDR                 0x28
#define LIS3DSH_OUT_X_H_ADDR                 0x29
#define LIS3DSH_OUT_Y_L_ADDR                 0x2A
#define LIS3DSH_OUT_Y_H_ADDR                 0x2B
#define LIS3DSH_OUT_Z_L_ADDR                 0x2C
#define LIS3DSH_OUT_Z_H_ADDR                 0x2D
//------------------------------------------------
 

void Accel_Ini(void);
void Accel_IO_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);
void Accel_ReadAcc(void);
static uint8_t SPIx_WriteRead(uint8_t Byte);
void Accel_IO_Write(uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite);
//————————————————

#endif /* LIS3DSH_H_ */

 



 

