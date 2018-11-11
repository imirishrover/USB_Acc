#include "lis3dsh.h"
#include "usbd_cdc_if.h"

//————————————
extern SPI_HandleTypeDef hspi1;
uint8_t buf2[8]={0};

char str1[30]={0};
//————————————

static void Error (void)

{
 LD5_ON;
}

//—————————————

void Accel_IO_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)

{
  if(NumByteToRead > 0x01)

  {
    ReadAddr |= (uint8_t)(READWRITE_CMD | MULTIPLEBYTE_CMD);
  }

  else
  {
    ReadAddr |= (uint8_t)READWRITE_CMD;
  }
  CS_ON;

  SPIx_WriteRead(ReadAddr);

  while(NumByteToRead > 0x00)
  {
    /* Send dummy byte (0x00) to generate the SPI clock to ACCELEROMETER (Slave device) */

    *pBuffer = SPIx_WriteRead(DUMMY_BYTE);

    NumByteToRead--;

    pBuffer++;
  }
  CS_OFF;

}

//—————————————


uint8_t Accel_ReadID(void)

{  

  uint8_t ctrl = 0x00;
	Accel_IO_Read(&ctrl, LIS3DSH_WHO_AM_I_ADDR, 1);
  return ctrl;

}

//—————————————

void Accel_AccFilterConfig(uint8_t FilterStruct)

{

 

}

//—————————————

void AccInit(uint16_t InitStruct)

{
  uint8_t ctrl = 0x00;

  ctrl = (uint8_t) (InitStruct);

        Accel_IO_Write(&ctrl, LIS3DSH_CTRL_REG4_ADDR, 1);

  ctrl = (uint8_t) (InitStruct >> 8);

        Accel_IO_Write(&ctrl, LIS3DSH_CTRL_REG5_ADDR, 1);

}

//—————————————

void Accel_Ini(void)

{
uint16_t ctrl = 0x0000;

HAL_Delay(1000);

if(Accel_ReadID() == 0x3F) LD4_ON;

else Error();
	/* Configure MEMS: power mode(ODR) and axes enable */

ctrl = (uint16_t) (LIS3DSH_DATARATE_100 | LIS3DSH_XYZ_ENABLE);

/* Configure MEMS: full scale and self test */

ctrl |= (uint16_t) ((LIS3DSH_SERIALINTERFACE_4WIRE |

                         LIS3DSH_SELFTEST_NORMAL   |

                         LIS3DSH_FULLSCALE_2  |

                         LIS3DSH_FILTER_BW_50) << 8);

AccInit(ctrl);

        LD6_ON;

}
 



//—————————————
void Accel_GetXYZ(int16_t* pData)

{

 int8_t buffer[6];

  uint8_t crtl, i = 0x00;

  float sensitivity = LIS3DSH_SENSITIVITY_0_06G;

  float valueinfloat = 0;
	Accel_IO_Read(&crtl, LIS3DSH_CTRL_REG5_ADDR, 1);

  Accel_IO_Read((uint8_t*)&buffer[0], LIS3DSH_OUT_X_L_ADDR, 1);

  Accel_IO_Read((uint8_t*)&buffer[1], LIS3DSH_OUT_X_H_ADDR, 1);

  Accel_IO_Read((uint8_t*)&buffer[2], LIS3DSH_OUT_Y_L_ADDR, 1);

  Accel_IO_Read((uint8_t*)&buffer[3], LIS3DSH_OUT_Y_H_ADDR, 1);

  Accel_IO_Read((uint8_t*)&buffer[4], LIS3DSH_OUT_Z_L_ADDR, 1);

  Accel_IO_Read((uint8_t*)&buffer[5], LIS3DSH_OUT_Z_H_ADDR, 1);
	
	 switch(crtl & LIS3DSH__FULLSCALE_SELECTION)

  {

  case LIS3DSH_FULLSCALE_2:

    sensitivity = LIS3DSH_SENSITIVITY_0_06G;

    break;

  case LIS3DSH_FULLSCALE_4:

    sensitivity = LIS3DSH_SENSITIVITY_0_12G;

    break;

  case LIS3DSH_FULLSCALE_6:

    sensitivity = LIS3DSH_SENSITIVITY_0_18G;

    break;

  case LIS3DSH_FULLSCALE_8:

    sensitivity = LIS3DSH_SENSITIVITY_0_24G;

    break;

  case LIS3DSH_FULLSCALE_16:

    sensitivity = LIS3DSH_SENSITIVITY_0_73G;

    break;

  default:

    break;                

}
	for(i=0; i<3; i++)

  {

    valueinfloat = ((buffer[2*i+1] << 8) + buffer[2*i]);
		//* sensitivity;

    pData[i] = (int16_t)valueinfloat;

  }


}

//—————————————

void Accel_ReadAcc(void)

{
int16_t buffer[3] = {0};

  int16_t xval, yval , zval = 0x00;

  Accel_GetXYZ(buffer);
xval = buffer[0];

  yval = buffer[1];

  zval = buffer[2];

//sprintf(str1,"X:%06d Y:%06d Z::%06drn", xval, yval, zval);
//CDC_Transmit_FS((uint8_t*)str1, strlen(str1));

				buf2[0]=0x11;
        buf2[1]=0x55;
        buf2[2] = (uint8_t) (xval >> 8);
        buf2[3] = (uint8_t) xval;
        buf2[4] = (uint8_t) (yval >> 8);
        buf2[5] = (uint8_t) yval;
        buf2[6] = (uint8_t) (zval >> 8);
        buf2[7] = (uint8_t) zval;

        CDC_Transmit_FS(buf2, 8);
				
if((ABS(xval))>(ABS(yval)))

  {

                if(xval > 8000)

                {

                        LD5_ON;

                }

                else if(xval < -8000)

                {

                        LD4_ON;

                }

  }

  else

  {

                if(yval > 8000)

                {

                        LD3_ON;

                }

                else if(yval < -8000)

                {

                        LD6_ON;

                }

  }
	
  HAL_Delay(20);
		LD3_OFF;

        LD4_OFF;

        LD5_OFF;

        LD6_OFF;
 

}

//—————————————


static uint8_t SPIx_WriteRead(uint8_t Byte)

{

  uint8_t receivedbyte = 0;

  if(HAL_SPI_TransmitReceive(&hspi1, (uint8_t*) &Byte, (uint8_t*) &receivedbyte, 1, 0x1000) != HAL_OK)

  {

    Error();

  }

  return receivedbyte;

}
//—————————————
void Accel_IO_Write(uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)

{

        CS_OFF;

  if(NumByteToWrite > 0x01)

  {

    WriteAddr |= (uint8_t)MULTIPLEBYTE_CMD;

  }

        CS_ON;

        SPIx_WriteRead(WriteAddr);

        while(NumByteToWrite >= 0x01)

  {

    SPIx_WriteRead(*pBuffer);

    NumByteToWrite--;

    pBuffer++;

  }

        CS_OFF;

}

//—————————————
