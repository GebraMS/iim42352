/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2020 GebraBit Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively �Software�) is subject
 * to GebraBit and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws. 
 *
 * GebraBit and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from GebraBit is strictly prohibited.
 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT 
 * NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT IN  
 * NO EVENT SHALL GebraBit BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, 
 * OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * @Author       	: Mehrdad Zeinali
 * ________________________________________________________________________________________________________
 */
 
#include	"GebraBit_IIM42352.h"

extern SPI_HandleTypeDef hspi1;
	
/*=========================================================================================================================================
 * @brief     Read data from spacial register.
 * @param     regAddr Register Address of IIM42352
 * @param     regBank Register Bank number .
 * @param     data    Pointer to Variable that register value is saved .
 * @return    stat    Return status
 ========================================================================================================================================*/
uint8_t	GB_IIM42352_Read_Reg_Data ( uint8_t regAddr, IIM42352_Bank_Sel regBank, uint8_t* data)
{	
	uint8_t txBuf[2] = {regAddr|0x80 , 0x00}; //Read operation: set the 8th-bit to 1.
	uint8_t rxBuf[2];
	HAL_StatusTypeDef stat = HAL_ERROR ;
	GB_IIM42352_Bank_Selection(regBank);
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	stat = (HAL_SPI_TransmitReceive(&hspi1, txBuf, rxBuf, 2, HAL_MAX_DELAY));
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	if (stat == HAL_OK)
	{
		*data = rxBuf[1];
	}
	return stat;
}
/*========================================================================================================================================= 
 * @brief     Read data from spacial bits of a register.
 * @param     regAddr     Register Address of IIM42352 .
 * @param     regBank     Register Bank number .
 * @param     start_bit   Start Bit location .(0 to 7)
 * @param     len         Quantity of Bits want to read(1 to 8) 
 * @param     data        Pointer to Variable that register Bits value is saved .
 * @return    status      Return status
 ========================================================================================================================================*/
uint8_t GB_IIM42352_Read_Reg_Bits (uint8_t regAddr, IIM42352_Bank_Sel regBank, uint8_t start_bit, uint8_t len, uint8_t* data)
{
	uint8_t status = HAL_ERROR;
	uint8_t tempData = 0;

	if (len>8 || start_bit>7)
	{
		return HAL_ERROR;
	}

	if (GB_IIM42352_Read_Reg_Data( regAddr, regBank, &tempData) == HAL_OK)
	{
		uint8_t mask = ((1 << len) - 1) << (start_bit - len + 1); //formula for making a broom of 1&0 for gathering desired bits
		tempData &= mask; // zero all non-important bits in data
		tempData >>= (start_bit - len + 1); //shift data to zero position
		*data = tempData;
		status = HAL_OK;
	}
	else
	{
		status = HAL_ERROR;
		*data = 0;
	}
	return status;
}
/*========================================================================================================================================= 
 * @brief     Read multiple data from first spacial register address.
 * @param     regAddr First Register Address of IIM42352 that reading multiple data start from this address
 * @param     regBank Register Bank number .
 * @param     data    Pointer to Variable that multiple data is saved .
 * @param     bytepcs Quantity of data that we want to read .
 * @return    stat    Return status
 ========================================================================================================================================*/
uint8_t GB_IIM42352_Burst_Read(uint8_t regAddr, IIM42352_Bank_Sel regBank, uint8_t *data, uint16_t byteQuantity)
{
	uint8_t *pTxBuf;
	uint8_t *pRxBuf;
	uint8_t status = HAL_ERROR;
	GB_IIM42352_Bank_Selection(regBank);
	pTxBuf = ( uint8_t * )malloc(sizeof(uint8_t) * (byteQuantity + 1)); // reason of "+1" is for register address that comes in first byte
	pRxBuf = ( uint8_t * )malloc(sizeof(uint8_t) * (byteQuantity + 1));
	memset(pTxBuf, 0, (byteQuantity + 1)*sizeof(uint8_t));

	pTxBuf[0] = regAddr | 0x80; //Read operation: set the 8th-bit to 1.

	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	status = (HAL_SPI_TransmitReceive(&hspi1, pTxBuf, pRxBuf, byteQuantity+1, HAL_MAX_DELAY));
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	
	if (status == HAL_OK)
	{
		memcpy(data, &pRxBuf[1], byteQuantity*sizeof(uint8_t)); //here we dont have "+1" beacause we don't need first byte that was register data , we just need DATA itself
	}
	free(pTxBuf);
	free(pRxBuf);
	return status;
}
/*=========================================================================================================================================
 * @brief     Write data to spacial register.
 * @param     regAddr Register Address of IIM42352
 * @param     regBank Register Bank number .
 * @param     data    Value that will be writen to register .
 * @return    stat    Return status
 ========================================================================================================================================*/
uint8_t GB_IIM42352_Write_Reg_Data(uint8_t regAddr, IIM42352_Bank_Sel regBank, uint8_t data)
{
	uint8_t txBuf[2] = {regAddr|0x00 , data}; //Write operation: set the 8th-bit to 0.
	uint8_t rxBuf[2];
	uint8_t status = HAL_ERROR;
	GB_IIM42352_Bank_Selection(regBank);
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	status = (HAL_SPI_TransmitReceive(&hspi1, txBuf, rxBuf, 2, HAL_MAX_DELAY));
	while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	
	return status;	
}

/*=========================================================================================================================================
 * @brief     Write data to spacial bits of a register.
 * @param     regAddr     Register Address of IIM42352 .
 * @param     regBank     Register Bank number .
 * @param     start_bit   Start Bit location .(0 to 7)
 * @param     len         Quantity of Bits want to write(1 to 8) 
 * @param     data        Value that will be writen to register bits .
 * @return    status      Return status
 ========================================================================================================================================*/
uint8_t GB_IIM42352_Write_Reg_Bits(uint8_t regAddr, IIM42352_Bank_Sel regBank, uint8_t start_bit, uint8_t len, uint8_t data)
{
	uint8_t txBuf[2];
	uint8_t rxBuf[2];
	uint8_t status = HAL_ERROR;
	uint8_t tempData = 0;
	if (len>8 || start_bit>7)
	{
		return HAL_ERROR;
	}
	if (GB_IIM42352_Read_Reg_Data( regAddr, regBank, &tempData) == HAL_OK)	
	{
		uint8_t mask = ((1 << len) - 1) << (start_bit - len + 1);
		data <<= (start_bit - len + 1); // shift data into correct position
		data &= mask; // zero all non-important bits in data
		tempData &= ~(mask); // zero all important bits in existing byte
		tempData |= data; // combine data with existing byte

		txBuf[0] = regAddr;
		txBuf[1] = tempData;
	
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
		status = (HAL_SPI_TransmitReceive(&hspi1, txBuf, rxBuf, 2, HAL_MAX_DELAY));
		while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	}
	return status;
}
/*========================================================================================================================================= 
 * @brief     Write value to Multiple register address.
 * @param     regAddr First Register Address of IIM42352 that writing multiple data start from this address
 * @param     regBank Register Bank number .
 * @param     data    Pointer to Variable that multiple data are writen from .
 * @param     bytepcs Quantity of data that we want to write .
 * @return    stat    Return status
 ========================================================================================================================================*/
uint8_t GB_IIM42352_Burst_Write		( uint8_t regAddr, IIM42352_Bank_Sel regBank, uint8_t *data, 	uint16_t byteQuantity)
{
	uint8_t txBuf[byteQuantity + 1]; // +1 is for register address that is 1 byte
	uint8_t rxBuf[byteQuantity + 1];
	uint8_t status = HAL_ERROR;
	GB_IIM42352_Bank_Selection(regBank);
	txBuf[0] = regAddr | 0x00; //Write operation: set the 8th-bit to 0.
	memcpy(txBuf+1, data, byteQuantity); // +1 is for set the address of data from [1]th position of array

	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	status = (HAL_SPI_TransmitReceive(&hspi1, txBuf, rxBuf, byteQuantity+1, HAL_MAX_DELAY));
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

	return status;
}
/*=========================================================================================================================================
 * @brief     Select Register Bank.
 * @param     bsel   Bank number
 * @return    Nothing
 ========================================================================================================================================*/
void GB_IIM42352_Bank_Selection( IIM42352_Bank_Sel bsel)
{
  uint8_t rtxBuf[2] = {IIM42352_REG_BANK_SEL|0x80 , 0x00}; //Read operation: set the 8th-bit to 1.
	uint8_t rrxBuf[2];
	uint8_t wtxBuf[2];
	uint8_t wrxBuf[2];
	HAL_StatusTypeDef stat = HAL_ERROR ;
	uint8_t tempData = 0;
	uint8_t start_bit = START_MSB_BIT_AT_2 ;
	uint8_t len = BIT_LENGTH_3 ;
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	stat = (HAL_SPI_TransmitReceive(&hspi1, rtxBuf, rrxBuf, 2, HAL_MAX_DELAY));
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	if (stat == HAL_OK)
	{
		tempData = rrxBuf[1];
	}
		uint8_t mask = ((1 << len) - 1) << (start_bit - len + 1);
		bsel <<= (start_bit - len + 1); // shift data into correct position
		bsel &= mask; // zero all non-important bits in data
		tempData &= ~(mask); // zero all important bits in existing byte
		tempData |= bsel; // combine data with existing byte

		wtxBuf[0] = IIM42352_REG_BANK_SEL;
		wtxBuf[1] = tempData;
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
		stat = (HAL_SPI_TransmitReceive(&hspi1, wtxBuf, wrxBuf, 2, HAL_MAX_DELAY));
		while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
}
/*=========================================================================================================================================
 * @brief     Select SPI 4 Wire as interface
 * @param     spisel Determines SPI 4 Wire as interface or not 	(NOT_SPI OR 	IS_SPI )                     			 
 * @return    Nothing
 ========================================================================================================================================*/
void GB_IIM42352_Select_SPI4_Interface(GebraBit_IIM42352 * IIM42352 , IIM42352_Interface spisel)
{
 GB_IIM42352_Write_Reg_Bits( IIM42352_INTF_CONFIG4,BANK_1, START_MSB_BIT_AT_1, BIT_LENGTH_1 , spisel);
 GB_IIM42352_Write_Reg_Bits( IIM42352_INTF_CONFIG6,BANK_1, START_MSB_BIT_AT_4, BIT_LENGTH_5 , I3C_DISABLE);
 IIM42352->INTERFACE = spisel ; 
}
/*=========================================================================================================================================
 * @brief     Select Pin 9 Function
 * @param     pin9f Determines Pin 9 Function INT2 , FSYNC , CLKIN
 * @return    Nothing
 ========================================================================================================================================*/
void GB_IIM42352_Select_PIN9_Function(GebraBit_IIM42352 * IIM42352 ,  IIM42352_PIN9_FUNCTION pin9f)
{
 GB_IIM42352_Write_Reg_Bits( IIM42352_INTF_CONFIG5,BANK_1, START_MSB_BIT_AT_2, BIT_LENGTH_2 , pin9f);
 IIM42352->PIN9_FUNCTION = pin9f ; 
}
/*=========================================================================================================================================
 * @brief     Read sensor Data endianess
 * @param     data_end    Store Data endianess BIG or LITTLE
 * @return    Nothing
 ========================================================================================================================================*/
void GB_IIM42352_Get_Sensor_Data_Endian ( IIM42352_Data_Endian * data_end  ) 
{
	GB_IIM42352_Read_Reg_Bits (IIM42352_INTF_CONFIG0, BANK_0 , START_MSB_BIT_AT_4, BIT_LENGTH_1, data_end);
}
/*=========================================================================================================================================
 * @brief     Set sensor Data endianess
 * @param     data_end Determines Data endianess BIG or LITTLE
 * @return    Nothing
 ========================================================================================================================================*/
void GB_IIM42352_Set_Sensor_Data_Endian ( GebraBit_IIM42352 * IIM42352 , IIM42352_Data_Endian  data_end  ) 
{
	GB_IIM42352_Write_Reg_Bits (IIM42352_INTF_CONFIG0, BANK_0 , START_MSB_BIT_AT_4, BIT_LENGTH_1, data_end);
	IIM42352->SENSOR_DATA_ENDIAN = data_end ;
}
/*=========================================================================================================================================
 * @brief     Reset IIM42352
 * @param     IIM42352   GebraBit_IIM42352 Struct
 * @return    Nothing
 ========================================================================================================================================*/
void GB_IIM42352_Soft_Reset ( GebraBit_IIM42352 * IIM42352 )
{
	uint8_t rest_done=0;
	do 
	 {
		GB_IIM42352_Write_Reg_Data( IIM42352_DEVICE_CONFIG ,  BANK_0, IIM42352_RESET); 
		HAL_Delay(1);
		GB_IIM42352_Read_Reg_Bits (IIM42352_INT_STATUS, BANK_0 , START_MSB_BIT_AT_4, BIT_LENGTH_1, &IIM42352->RESET);
		if ( IIM42352->RESET == DONE )
			break;
	 }while(1);
	//GB_IIM42352_Select_SPI4_Interface( IIM42352->Interface);

}
/*=========================================================================================================================================
 * @brief     Set ACCEL Full Scale Range and select sensor SCALE FACTOR
 * @param     IIM42352   GebraBit_IIM42352 Struct
 * @param     fs         Determines Full Scale Range among 2g , 4g , 8g , 16g
 * @return    Nothing
 ========================================================================================================================================*/
void GB_IIM42352_Set_ACCEL_FS ( GebraBit_IIM42352 * IIM42352 , IIM42352_Accel_Fs_Sel fs ) 
{
  GB_IIM42352_Write_Reg_Bits( IIM42352_ACCEL_CONFIG0,BANK_0, START_MSB_BIT_AT_7, BIT_LENGTH_3 , fs);
	IIM42352->ACCEL_FS_SEL =  fs ;
	switch(fs)
	 {
	  case FS_16g:
		IIM42352->SCALE_FACTOR = SCALE_FACTOR_2048_LSB_g ;
    break;
		case FS_8g:
		IIM42352->SCALE_FACTOR = SCALE_FACTOR_4096_LSB_g ;
    break;	
		case FS_4g:
		IIM42352->SCALE_FACTOR = SCALE_FACTOR_8192_LSB_g ;
    break;	
		case FS_2g:
		IIM42352->SCALE_FACTOR = SCALE_FACTOR_16384_LSB_g ;
    break;			
		default:
		IIM42352->SCALE_FACTOR = SCALE_FACTOR_2048_LSB_g ;		
	 }
}
/*=========================================================================================================================================
 * @brief     Set ACCEL Output Data Rate
 * @param     odr       Determines Output Data Rate from 1.565 Hz to 8KHz 
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42352_Set_ACCEL_ODR (GebraBit_IIM42352 * IIM42352 ,  IIM42352_Accel_ODR odr ) 
{
  GB_IIM42352_Write_Reg_Bits(IIM42352_ACCEL_CONFIG0,BANK_0, START_MSB_BIT_AT_3, BIT_LENGTH_4 , odr);
	IIM42352->ACCEL_ODR = odr  ;
}
/*=========================================================================================================================================
 * @brief     Set FIFO MODE
 * @param     mode     Determines FIFO MODE BYPASS ,  STREAM_TO_FIFO , STOP_ON_FULL
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42352_Set_FIFO_MODE (GebraBit_IIM42352 * IIM42352 , IIM42352_FIFO_MODE mode ) 
{
  GB_IIM42352_Write_Reg_Bits( IIM42352_FIFO_CONFIG,BANK_0, START_MSB_BIT_AT_7, BIT_LENGTH_2 , mode);
	IIM42352->FIFO_MODE = mode ;
}
/*=========================================================================================================================================
 * @brief     DISABLE FSYNC Function
 * @param     able     Determines FSYNC Function Disable or not
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42352_DISABLE_FSYNC ( GebraBit_IIM42352 * IIM42352 , IIM42352_Ability able) 
{
  GB_IIM42352_Write_Reg_Bits(IIM42352_FSYNC_CONFIG,BANK_0, START_MSB_BIT_AT_6, BIT_LENGTH_3 ,DO_NOT_TAG_FSYNC_FLAG);//DO_NOT_TAG_FSYNC_FLAG
	GB_IIM42352_Write_Reg_Bits(IIM42352_TMST_CONFIG,BANK_0, START_MSB_BIT_AT_1, BIT_LENGTH_1 , FSYNC_TIME_STAMP_DISABLE);//////////////////////////
	IIM42352->FSYNC = Disable ;
}
/*=========================================================================================================================================
 * @brief     Set Timestamp Resolution
 * @param     res           Determines Timestamp Resolution _16_uS ,  _1_uS
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42352_Set_Timestamp_Resolution ( GebraBit_IIM42352 * IIM42352 , IIM42352_Timestamp_Resolution res) 
{
  GB_IIM42352_Write_Reg_Bits(IIM42352_TMST_CONFIG,BANK_0, START_MSB_BIT_AT_3, BIT_LENGTH_1 , res);
  IIM42352->TMST_RESOLUTION = res  ; 
}
/*=========================================================================================================================================
 * @brief     Set FIFO Count Setting 
 * @param     counting           Determines FIFO count is reported in bytes or record
 * @param     endian             Determines FIFO count is reported in Little Endian format or Big
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42352_SET_FIFO_Count (GebraBit_IIM42352 * IIM42352 , IIM42352_FIFO_Counting counting , IIM42352_Data_Endian endian ) 
{
  GB_IIM42352_Write_Reg_Bits(IIM42352_INTF_CONFIG0,BANK_0, START_MSB_BIT_AT_6, BIT_LENGTH_1 , counting);
	GB_IIM42352_Write_Reg_Bits(IIM42352_INTF_CONFIG0,BANK_0, START_MSB_BIT_AT_5, BIT_LENGTH_1 , endian);
	IIM42352->FIFO_COUNTING = counting ;
	IIM42352->FIFO_COUNT_ENDIAN = endian ;
}
/*
M403Z 
*/
/*=========================================================================================================================================
 * @brief     Get FIFO Count  
 * @param     IIM42352     IIM42352 Struct FIFO_COUNT variable          
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42352_GET_FIFO_Count (GebraBit_IIM42352 * IIM42352 ) 
{
	uint8_t count_h , count_l;
  GB_IIM42352_Read_Reg_Data( IIM42352_FIFO_COUNTH, BANK_0, &count_h);
	GB_IIM42352_Read_Reg_Data( IIM42352_FIFO_COUNTL, BANK_0, &count_l );
	IIM42352->FIFO_COUNT = (uint16_t)((count_h << 8) | count_l);
}

/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE Time Stamp Register
 * @param     ability     Determines Time Stamp Register  Disable or not
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42352_SET_Time_Stamp_Register(GebraBit_IIM42352 * IIM42352 ,IIM42352_Ability ability)
{
  GB_IIM42352_Write_Reg_Bits(IIM42352_TMST_CONFIG,BANK_0, START_MSB_BIT_AT_0, BIT_LENGTH_1 , ability);
	IIM42352->TMST_REGISTER = ability  ;
}
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE TEMP and ACCEL Packets To STORE in FIFO
 * @param     allpack     Can choose between NOTHING_To_FIFO and ACCEL_TEMP_To_FIFO
 * @return    Nothing
 ========================================================================================================================================*/ 	
void GB_IIM42352_Write_ACCEL_TEMP_To_FIFO(GebraBit_IIM42352 * IIM42352 , IIM42352_Packet_To_FIFO allpack) 
{
	if(allpack == NOTHING_To_FIFO)
	{
	  GB_IIM42352_Write_Reg_Bits( IIM42352_FIFO_CONFIG1,BANK_0, START_MSB_BIT_AT_2, BIT_LENGTH_3 , allpack);
	}
	else
	{
  	GB_IIM42352_Write_Reg_Bits( IIM42352_FIFO_CONFIG1,BANK_0, START_MSB_BIT_AT_2, BIT_LENGTH_3 , allpack);
	}
	  IIM42352->PACKET_To_FIFO =  allpack ;  
}
/*=========================================================================================================================================
 * @brief     SET FIFO WATERMARK 
 * @param     watermark     Determines FIFO WATERMARK Enable or not
 * @param     wm            Determines FIFO WATERMARK Value
 * @return    Nothing
 ========================================================================================================================================*/ 	
void GB_IIM42352_SET_FIFO_WATERMARK (GebraBit_IIM42352 * IIM42352 ,IIM42352_Ability watermark , uint16_t wm)
{
    GB_IIM42352_Write_Reg_Bits (IIM42352_FIFO_CONFIG1,BANK_0, START_MSB_BIT_AT_5, BIT_LENGTH_1 , watermark);
		GB_IIM42352_Write_Reg_Data (IIM42352_FIFO_CONFIG2,BANK_0,(uint8_t) (wm & 0xff));
	  GB_IIM42352_Write_Reg_Bits (IIM42352_FIFO_CONFIG3,BANK_0,START_MSB_BIT_AT_3, BIT_LENGTH_4 ,(uint8_t) (wm>> 8));	
		IIM42352->FIFO_WATERMARK  = watermark ;
		IIM42352->WATERMARK_VALUE = wm ;
}
/*=========================================================================================================================================
 * @brief     SET FIFO Decimation Factor
 * @param     factor   FIFO Decimation Factor Value
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42352_SET_FIFO_Decimation_Factor (GebraBit_IIM42352 * IIM42352 ,uint8_t factor )
{
    GB_IIM42352_Write_Reg_Bits (IIM42352_FDR_CONFIG ,BANK_4, START_MSB_BIT_AT_6, BIT_LENGTH_7 , factor);
	IIM42352->FIFO_DECIMATION_FACTOR = factor ;
    
}
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE Data Ready Interrupt
 * @param     ability    Determines Data Ready Interrupt Disable or not
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42352_SET_Data_Ready_Interrupt(GebraBit_IIM42352 * IIM42352 , IIM42352_Ability ability)
{
	GB_IIM42352_Write_Reg_Bits(IIM42352_INT_SOURCE0,BANK_0, START_MSB_BIT_AT_3, BIT_LENGTH_1 , ability);
	IIM42352->DATA_READY_INT = ability  ;
}
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE FIFO Full Interrupt
 * @param     ability    Determines FIFO Full Interrupt Disable or Enable
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42352_SET_FIFO_Full_Interrupt(GebraBit_IIM42352 * IIM42352 , IIM42352_Ability ability)
{
	GB_IIM42352_Write_Reg_Bits(IIM42352_INT_SOURCE0,BANK_0, START_MSB_BIT_AT_1, BIT_LENGTH_1 , ability);
	IIM42352->FIFO_FULL_INT = ability  ;
}

/*=========================================================================================================================================
 * @brief     For register INT_CONFIG1 (bank 0 register 0x64) bit 4 INT_ASYNC_RESET, user should change setting to 0 from default setting 
              of 1, for proper INT1 and INT2 pin operation
 * @param     Nothing
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42352_SET_INT_ASYNC_RESET_ZERO(void )
{
	GB_IIM42352_Write_Reg_Bits( IIM42352_INT_CONFIG1,BANK_0, START_MSB_BIT_AT_4, BIT_LENGTH_1 , 0);
}
/*=========================================================================================================================================
 * @brief     Flush and Empty FIFO
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42352_FIFO_FLUSH(void )
{
	GB_IIM42352_Write_Reg_Bits( IIM42352_SIGNAL_PATH_RESET,BANK_0, START_MSB_BIT_AT_1, BIT_LENGTH_1 , 1);
}

/*=========================================================================================================================================
 * @brief     Configure FIFO
 * @param     IIM42352       IIM42352 Struct
 * @param     fifo           Configure IIM42352 FIFO according it is Disable or Enable
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42352_FIFO_Configuration ( GebraBit_IIM42352 * IIM42352 , IIM42352_FIFO_Ability fifo  )
{
	IIM42352->FIFO_PACKET_QTY = FIFO_DATA_BUFFER_SIZE / BYTE_QTY_IN_ONE_PACKET ;  
	if( fifo==Enable )  
	{
		IIM42352->FIFO = FIFO_ENABLE  ;
		GB_IIM42352_FIFO_FLUSH();
		GB_IIM42352_SET_FIFO_High_Resolution( IIM42352 , Disable);
    GB_IIM42352_SET_FIFO_Count(IIM42352,IN_RECORDS , BIG );//LITTLE
	  GB_IIM42352_SET_FIFO_WATERMARK(IIM42352 , Disable, 0);//Enable , 1 
	  GB_IIM42352_SET_FIFO_Decimation_Factor(IIM42352 ,0);//FIFO_DECIMATION_FACTOR=0
		GB_IIM42352_SET_FIFO_Full_Interrupt( IIM42352 ,Enable ); 
		GB_IIM42352_Set_FIFO_MODE ( IIM42352 , STOP_ON_FULL );//STOP_ON_FULL
	  GB_IIM42352_Write_ACCEL_TEMP_To_FIFO(IIM42352 , ACCEL_TEMP_To_FIFO);//Enable
	}
	else if ( fifo == Disable )
	{
		IIM42352->FIFO = FIFO_DISABLE  ;
		GB_IIM42352_Set_FIFO_MODE  (IIM42352 , BYPASS );//BYPASS
		GB_IIM42352_Write_ACCEL_TEMP_To_FIFO(IIM42352 , NOTHING_To_FIFO);//--****************************---Disable
	}
}
/*=========================================================================================================================================
 * @brief     Set UI Filter Order
 * @param     order  Determines UI Filter Order _1_ORDER ,  _2_ORDER or _3_ORDER
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42352_UI_Filter_Order ( GebraBit_IIM42352 * IIM42352 , IIM42352_UI_Filter_Order order ) 
{
  GB_IIM42352_Write_Reg_Bits(IIM42352_ACCEL_CONFIG1,BANK_0, START_MSB_BIT_AT_4, BIT_LENGTH_2 , order);
	IIM42352->UI_FILTER_ORDER = order ;
}
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE FIFO High Resolution
 * @param     highres    Determines FIFO High Resolution Disable or not
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42352_SET_FIFO_High_Resolution(GebraBit_IIM42352 * IIM42352 , IIM42352_Ability highres)
{

	GB_IIM42352_Write_Reg_Bits( IIM42352_FIFO_CONFIG1,BANK_0, START_MSB_BIT_AT_4, BIT_LENGTH_1 , highres);
  IIM42352->FIFO_HIGH_RESOLUTION = highres ; 
}
/*=========================================================================================================================================
 * @brief     Get Who am I Register Value From Sensor
 * @param     IIM42352     IIM42352 Struct WHO_AM_I variable
 * @return    Nothing
 ========================================================================================================================================*/ 
void	GB_IIM42352_Who_am_I(GebraBit_IIM42352 * IIM42352)
{
	GB_IIM42352_Read_Reg_Data( IIM42352_WHO_AM_I, BANK_0,&IIM42352->WHO_AM_I);
}	
/*=========================================================================================================================================
 * @brief     DISABLE or ENABLE RTC Mode
 * @param     Nothing
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42352_DISABLE_RTC_Mode ( void ) 
{
  GB_IIM42352_Write_Reg_Bits(IIM42352_INTF_CONFIG1,BANK_0, START_MSB_BIT_AT_2, BIT_LENGTH_1 , Disable);
}
/*=========================================================================================================================================
 * @brief     Set ACCEL LN Filter
 * @param     filter       Determines LN FILTER BW from 2 to 40 
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42352_ACCEL_LN_Filter_Configuration( GebraBit_IIM42352 * IIM42352 ,IIM42352_Low_Noise_Filter_BW filter)
{
   GB_IIM42352_Write_Reg_Bits( IIM42352_ACCEL_FILT_CONFIG,BANK_0, START_MSB_BIT_AT_7, BIT_LENGTH_4 , filter);
	 IIM42352->LN_Filter_BW = filter  ;
}
/*=========================================================================================================================================
 * @brief     Set ACCEL LP Filter
 * @param     filter       Determines LP FILTER BW between LP_1x_AVG_FILTER or LP_16x_AVG_FILTER
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42352_ACCEL_LP_Filter_Configuration(GebraBit_IIM42352 * IIM42352 , IIM42352_Low_Power_Filter_AVG filter)
{
   GB_IIM42352_Write_Reg_Bits( IIM42352_ACCEL_FILT_CONFIG,BANK_0, START_MSB_BIT_AT_7, BIT_LENGTH_4 , filter);
	IIM42352->LP_Filter_AVG = filter ;
}
/*=========================================================================================================================================
 * @brief     Enable or Disable Accelerometer in X , Y , Z Axis
 * @param     x_axis    Disable or Enable Accelerometer in X Axis
 * @param     y_axis    Disable or Enable Accelerometer in Y Axis
 * @param     z_axis    Disable or Enable Accelerometer in Z Axis
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42352_Enable_Disable_XYZ_ACCEL(GebraBit_IIM42352 * IIM42352 ,IIM42352_Ability x_axis,IIM42352_Ability y_axis,IIM42352_Ability z_axis )
{
  GB_IIM42352_Write_Reg_Bits (IIM42352_SENSOR_CONFIG0 ,BANK_1, START_MSB_BIT_AT_0, BIT_LENGTH_1,  !x_axis);
	GB_IIM42352_Write_Reg_Bits (IIM42352_SENSOR_CONFIG0 ,BANK_1, START_MSB_BIT_AT_1, BIT_LENGTH_1,  !y_axis);
	GB_IIM42352_Write_Reg_Bits (IIM42352_SENSOR_CONFIG0 ,BANK_1, START_MSB_BIT_AT_2, BIT_LENGTH_1,  !z_axis);
	IIM42352->X_AXIS = x_axis ;
	IIM42352->Y_AXIS = y_axis ;
	IIM42352->Z_AXIS = z_axis ; 
  HAL_Delay(20);
}
/*=========================================================================================================================================
 * @brief     Enable or Disable Temperature 
 * @param     temp      Disable or Enable Temperature
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42352_Enable_Disable_Temperature(GebraBit_IIM42352 * IIM42352 ,IIM42352_Ability temp )
{
	GB_IIM42352_Write_Reg_Bits (IIM42352_PWR_MGMT0 ,BANK_0, START_MSB_BIT_AT_5, BIT_LENGTH_1,  !temp);
	IIM42352->TEMPERATURE = temp ; 
	HAL_Delay(20);
}

/*=========================================================================================================================================
 * @brief     Set Power Mode
 * @param     IIM42352       Configure IIM42352 Power Mode according to IIM42352 Struct Power_Mode variable
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42352_Set_Power_Management(GebraBit_IIM42352 * IIM42352 , IIM42352_Power_Mode pmode) 
{	
	GB_IIM42352_Enable_Disable_XYZ_ACCEL( IIM42352 ,Enable ,Enable ,Enable  );
	GB_IIM42352_Enable_Disable_Temperature(IIM42352 , Enable );

 if(pmode==IIM42352_LOW_POWER)
 {
	GB_IIM42352_ACCEL_LP_Filter_Configuration(IIM42352 , LP_16x_AVG_FILTER);
  GB_IIM42352_Write_Reg_Bits( IIM42352_PWR_MGMT0,BANK_0, START_MSB_BIT_AT_1, BIT_LENGTH_2 ,pmode); 
 }
  else if(pmode==IIM42352_LOW_NOISE)
 {
	GB_IIM42352_ACCEL_LN_Filter_Configuration( IIM42352 , LN_FILTER_BW_5);
  GB_IIM42352_Write_Reg_Bits(IIM42352_PWR_MGMT0,BANK_0, START_MSB_BIT_AT_1, BIT_LENGTH_2 , pmode);  
 }
 else
 {
	GB_IIM42352_Write_Reg_Bits( IIM42352_PWR_MGMT0,BANK_0, START_MSB_BIT_AT_1, BIT_LENGTH_2 , pmode); 
	GB_IIM42352_Write_Reg_Bits( IIM42352_PWR_MGMT0,BANK_0, START_MSB_BIT_AT_4, BIT_LENGTH_1 , 0);
 }
 IIM42352->POWER_MODE = pmode ;
 HAL_Delay(1);
}
/*=========================================================================================================================================
 * @brief     Check if Data is ready
 * @param     IIM42352    Store data ready status on IIM42352 Struct DATA_STATUS variable
 * @return    IS_Ready or IS_NOT_Ready
 ========================================================================================================================================*/ 
IIM42352_Preparation GB_IIM42352_Check_Data_Preparation(GebraBit_IIM42352 * IIM42352)
{
  GB_IIM42352_Read_Reg_Bits (IIM42352_INT_STATUS, BANK_0 , START_MSB_BIT_AT_3, BIT_LENGTH_1, &IIM42352->DATA_STATUS); 
	return IIM42352->DATA_STATUS;
}
/*=========================================================================================================================================
 * @brief     Check if FIFO IS FULL
 * @param     IIM42352    Store FIFO IS FULL status on IIM42352 Struct FIFO_FULL variable
 * @return    IS_Ready or IS_NOT_Ready
 ========================================================================================================================================*/ 
IIM42352_Preparation GB_IIM42352_Check_FIFO_FULL(GebraBit_IIM42352 * IIM42352)
{
  GB_IIM42352_Read_Reg_Bits (IIM42352_INT_STATUS, BANK_0 , START_MSB_BIT_AT_1, BIT_LENGTH_1, &IIM42352->FIFO_FULL); 
	return IIM42352->FIFO_FULL;
}
/*=========================================================================================================================================
 * @brief     Format Data
 * @param     IIM42352       Format Data Base On  IIM42352 Struct SENSOR_DATA_ENDIAN variable
 * @param     datain         raw input Data
 * @param     dataout        Formated output Data 
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42352_Format_Data_Base_On_Endian(GebraBit_IIM42352 * IIM42352, const uint8_t *datain, uint16_t *dataout)
{
	if(IIM42352->SENSOR_DATA_ENDIAN == BIG)
		*dataout = (datain[0] << 8) | datain[1];
	if(IIM42352->SENSOR_DATA_ENDIAN == LITTLE)
		*dataout = (datain[1] << 8) | datain[0];
}
/*=========================================================================================================================================
 * @brief     initialize IIM42352
 * @param     IIM42352     initialize IIM42352 according  GebraBit_IIM42352 Staruct values
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42352_initialize( GebraBit_IIM42352 * IIM42352 )
{
  HAL_Delay(3);
	GB_IIM42352_Select_SPI4_Interface(IIM42352 , IS_SPI);
	GB_IIM42352_Soft_Reset(IIM42352);
	GB_IIM42352_Set_Power_Management( IIM42352 , IIM42352_LOW_NOISE );
	GB_IIM42352_FIFO_Configuration ( IIM42352 ,FIFO_DISABLE ) ;
	GB_IIM42352_Set_Sensor_Data_Endian( IIM42352 ,BIG);
	GB_IIM42352_SET_INT_ASYNC_RESET_ZERO(  );
	GB_IIM42352_SET_Data_Ready_Interrupt( IIM42352 ,Enable );
	GB_IIM42352_Select_PIN9_Function ( IIM42352 ,FSYNC);
	GB_IIM42352_DISABLE_FSYNC ( IIM42352 , Disable) ;//Disable
	//GB_IIM42352_Set_Timestamp_Resolution ( IIM42352 ,_16_uS  ) ;
	GB_IIM42352_UI_Filter_Order ( IIM42352 , _2_ORDER ) ;
	GB_IIM42352_Who_am_I(IIM42352);
//	GB_IIM42352_ACCEL_LN_Filter_Configuration(IIM42352 , LN_FILTER_BW_4);
//	GB_IIM42352_ACCEL_LP_Filter_Configuration(IIM42352 , LP_16x_AVG_FILTER);
}
/*=========================================================================================================================================
 * @brief     Configure IIM42352
 * @param     IIM42352  Configure IIM42352 according  GebraBit_IIM42352 Staruct values
 * @param     fifo      Determines 	FIFO_DISABLE or FIFO_ENABLE 
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42352_Configuration(GebraBit_IIM42352 * IIM42352, IIM42352_FIFO_Ability fifo)
{
  GB_IIM42352_DISABLE_RTC_Mode (  ) ;
	GB_IIM42352_Set_ACCEL_FS( IIM42352 , FS_4g );
	GB_IIM42352_Set_ACCEL_ODR (IIM42352 ,ODR_1KHz); 
	GB_IIM42352_FIFO_Configuration ( IIM42352 ,fifo ) ;
	HAL_Delay(20);	
}

/*=========================================================================================================================================
 * @brief     Read Data Directly from FIFO
 * @param     IIM42352  IIM42352 struct 
 * @param     qty    Determine hoe many Data Byte to read
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42352_Read_FIFO(GebraBit_IIM42352 * IIM42352 , uint16_t qty)  
{
  GB_IIM42352_Burst_Read( IIM42352_FIFO_DATA,BANK_0,IIM42352->FIFO_DATA, qty);
}

/*=========================================================================================================================================
 * @brief     Get Raw Data Of Temprature from Register 
 * @param     IIM42352  store Raw Data Of Temprature in GebraBit_IIM42352 Staruct REGISTER_RAW_TEMP
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42352_Get_Temp_Register_Raw_Data(GebraBit_IIM42352 * IIM42352)
{
	uint8_t temp_msb , temp_lsb;
    GB_IIM42352_Read_Reg_Data( IIM42352_TEMP_DATA1, BANK_0, &temp_msb);
	GB_IIM42352_Read_Reg_Data(IIM42352_TEMP_DATA0, BANK_0, &temp_lsb);
	IIM42352->REGISTER_RAW_TEMP = (int16_t)((temp_msb << 8) | temp_lsb);
}

/*=========================================================================================================================================
 * @brief     Get Valid Data Of Temprature Base on Datasheet Formula 
 * @param     IIM42352  store Valid Data Of Temprature in IIM42352 Staruct VALID_TEMP_DATA
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42352_Get_Temp_Register_Valid_Data(GebraBit_IIM42352 * IIM42352)
{ 
  IIM42352->VALID_TEMP_DATA =(IIM42352->REGISTER_RAW_TEMP / 132.48 ) + 25-ROOM_TEMPERATURE_OFFSET;/// OFFSET!!!
}
/*=========================================================================================================================================
 * @brief     Get Raw Data Of X Axis ACCEL from Register 
 * @param     IIM42352  store Raw Data Of X Axis ACCEL DATA in IIM42352 Staruct REGISTER_RAW_ACCEL_X
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42352_Get_ACCEL_DATA_X_Register_Raw(GebraBit_IIM42352 * IIM42352)
{
	uint8_t accelx_msb , acclx_lsb;
  GB_IIM42352_Read_Reg_Data( IIM42352_ACCEL_DATA_X1, BANK_0, &accelx_msb);
	GB_IIM42352_Read_Reg_Data( IIM42352_ACCEL_DATA_X0, BANK_0, &acclx_lsb );
	IIM42352->REGISTER_RAW_ACCEL_X = (int16_t)((accelx_msb << 8) | acclx_lsb);
}
/*=========================================================================================================================================
 * @brief     Get Raw Data Of Y Axis ACCEL from Register 
 * @param     IIM42352  store Raw Data Of Y Axis ACCEL DATA in IIM42352 Staruct REGISTER_RAW_ACCEL_Y
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42352_Get_ACCEL_DATA_Y_Register_Raw(GebraBit_IIM42352 * IIM42352)
{
	uint8_t accely_msb , accly_lsb;
  GB_IIM42352_Read_Reg_Data( IIM42352_ACCEL_DATA_Y1, BANK_0, &accely_msb);
	GB_IIM42352_Read_Reg_Data( IIM42352_ACCEL_DATA_Y0, BANK_0, &accly_lsb );
	IIM42352->REGISTER_RAW_ACCEL_Y = (int16_t)((accely_msb << 8) | accly_lsb);
}
/*=========================================================================================================================================
 * @brief     Get Raw Data Of Z Axis ACCEL from Register 
 * @param     IIM42352  store Raw Data Of Z Axis ACCEL DATA in IIM42352 Staruct REGISTER_RAW_ACCEL_Z
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42352_Get_ACCEL_DATA_Z_Register_Raw(GebraBit_IIM42352 * IIM42352)
{
	uint8_t accelz_msb , acclz_lsb;
  GB_IIM42352_Read_Reg_Data( IIM42352_ACCEL_DATA_Z1, BANK_0, &accelz_msb);
	GB_IIM42352_Read_Reg_Data( IIM42352_ACCEL_DATA_Z0, BANK_0, &acclz_lsb );
	IIM42352->REGISTER_RAW_ACCEL_Z = (int16_t)((accelz_msb << 8) | acclz_lsb);
}
/*=========================================================================================================================================
 * @brief     Get Valid Data Of X Axis ACCEL Base on IIM42352 Staruct SCALE_FACTOR 
 * @param     IIM42352  store Valid Data Of X Axis ACCEL in IIM42352 Staruct VALID_ACCEL_DATA_X
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42352_Get_ACCEL_DATA_X_Register_Valid_Data(GebraBit_IIM42352 * IIM42352)
{
	float scale_factor = IIM42352->SCALE_FACTOR;
  IIM42352->VALID_ACCEL_DATA_X =(IIM42352->REGISTER_RAW_ACCEL_X /scale_factor);
}
/*=========================================================================================================================================
 * @brief     Get Valid Data Of Y Axis ACCEL Base on IIM42352 Staruct SCALE_FACTOR 
 * @param     IIM42352  store Valid Data Of Y Axis ACCEL in IIM42352 Staruct VALID_ACCEL_DATA_Y
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42352_Get_ACCEL_DATA_Y_Register_Valid_Data(GebraBit_IIM42352 * IIM42352)
{
	float scale_factor = IIM42352->SCALE_FACTOR;
  IIM42352->VALID_ACCEL_DATA_Y =(IIM42352->REGISTER_RAW_ACCEL_Y /scale_factor);
}
/*=========================================================================================================================================
 * @brief     Get Valid Data Of Z Axis ACCEL Base on IIM42352 Staruct SCALE_FACTOR 
 * @param     IIM42352  store Valid Data Of Z Axis ACCEL in IIM42352 Staruct VALID_ACCEL_DATA_Z
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42352_Get_ACCEL_DATA_Z_Register_Valid_Data(GebraBit_IIM42352 * IIM42352)
{
	float scale_factor = IIM42352->SCALE_FACTOR;
  IIM42352->VALID_ACCEL_DATA_Z =(IIM42352->REGISTER_RAW_ACCEL_Z /scale_factor);
}
/*=========================================================================================================================================
 * @brief     Get Temprature Directly 
 * @param     IIM42352       IIM42352 Staruct
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42352_Get_Temperature(GebraBit_IIM42352 * IIM42352)
{
  GB_IIM42352_Get_Temp_Register_Raw_Data  (IIM42352);
	GB_IIM42352_Get_Temp_Register_Valid_Data(IIM42352);
}
/*=========================================================================================================================================
 * @brief     Get XYZ ACCELERATION Directly 
 * @param     IIM42352       GebraBit_IIM42352 Staruct
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42352_Get_XYZ_ACCELERATION(GebraBit_IIM42352 * IIM42352)
{
	GB_IIM42352_Get_ACCEL_DATA_X_Register_Raw(IIM42352);
	GB_IIM42352_Get_ACCEL_DATA_X_Register_Valid_Data(IIM42352);
	GB_IIM42352_Get_ACCEL_DATA_Y_Register_Raw(IIM42352);
	GB_IIM42352_Get_ACCEL_DATA_Y_Register_Valid_Data(IIM42352);
	GB_IIM42352_Get_ACCEL_DATA_Z_Register_Raw(IIM42352);
	GB_IIM42352_Get_ACCEL_DATA_Z_Register_Valid_Data(IIM42352);
}
/*=========================================================================================================================================
 * @brief     Get XYZ ACCELERATION and Temprature Directly From Registers
 * @param     IIM42352       IIM42352 Staruct
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42352_Get_ACCEL_XYZ_TEMP_From_Registers(GebraBit_IIM42352 * IIM42352)
{
  if (IS_Ready==GB_IIM42352_Check_Data_Preparation(IIM42352))
	 {
		 IIM42352->GET_DATA =  FROM_REGISTER ; 
	   GB_IIM42352_Get_Temperature( IIM42352 );
	   GB_IIM42352_Get_XYZ_ACCELERATION( IIM42352);
	 }
}

/*=========================================================================================================================================
 * @brief     Separate XYZ ACCELERATION and Temprature Data From FIFO and caculate Valid data
 * @param     IIM42352  store Valid Data Of XYZ ACCEL Axis and temp from FIFO TO IIM42352 Staruct VALID_FIFO_DATA_X , VALID_FIFO_DATA_Y ,VALID_FIFO_DATA_Z
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42352_FIFO_Data_Partition_ACCEL_XYZ_TEMP(GebraBit_IIM42352 * IIM42352)
{
	uint16_t i,offset=0;
  float scale_factor = IIM42352->SCALE_FACTOR; 
	for ( i = 0 ; i < IIM42352->FIFO_PACKET_QTY ; i++ )
	{
		if ( IIM42352->PACKET_To_FIFO == ACCEL_TEMP_To_FIFO  )
		{
			IIM42352->FIFO_HEADER[i] = IIM42352->FIFO_DATA[offset];
			offset += 1;
			IIM42352->VALID_FIFO_DATA_X[i] = ((int16_t)( (IIM42352->FIFO_DATA[offset] << 8) | IIM42352->FIFO_DATA[offset+1]))/scale_factor ;
			offset += 2; 
			IIM42352->VALID_FIFO_DATA_Y[i] = ((int16_t)( (IIM42352->FIFO_DATA[offset] << 8) | IIM42352->FIFO_DATA[offset+1]))/scale_factor ;
			offset += 2;
			IIM42352->VALID_FIFO_DATA_Z[i] = ((int16_t)( (IIM42352->FIFO_DATA[offset] << 8) | IIM42352->FIFO_DATA[offset+1]))/scale_factor ;
			offset += 2;
			IIM42352->VALID_FIFO_TEMP[i]   = (float)((((int8_t)(IIM42352->FIFO_DATA[offset]))/ 2.07) + 25-ROOM_TEMPERATURE_OFFSET) ;
			offset += 1;
		}
	}
}

/*=========================================================================================================================================
 * @brief     Get XYZ ACCELERATION and Temprature From FIFO
 * @param     IIM42352       IIM42352 Staruct
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42352_Get_ACCEL_XYZ_TEMP_From_FIFO(GebraBit_IIM42352 * IIM42352)
{
		if (IS_Ready==GB_IIM42352_Check_Data_Preparation(IIM42352))
		{
		  if (IS_Ready==GB_IIM42352_Check_FIFO_FULL(IIM42352))
		  {
				IIM42352->GET_DATA =  FROM_FIFO ;
				GB_IIM42352_Read_FIFO(IIM42352,FIFO_DATA_BUFFER_SIZE);
				GB_IIM42352_GET_FIFO_Count(IIM42352);
				GB_IIM42352_FIFO_Data_Partition_ACCEL_XYZ_TEMP(IIM42352);  
				GB_IIM42352_FIFO_FLUSH();
		  } 
		}	
}

/*=========================================================================================================================================
 * @brief     Get Data From IIM42352
 * @param     IIM42352       IIM42352 Staruct
 * @param     get_data       Determine Method of reading data from sensoe : FROM_REGISTER or FROM_FIFO
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_IIM42352_Get_Data(GebraBit_IIM42352 * IIM42352 , IIM42352_Get_DATA get_data)
{
 if( (get_data == FROM_REGISTER)&&(IIM42352->FIFO == Disable) )
	 GB_IIM42352_Get_ACCEL_XYZ_TEMP_From_Registers(IIM42352);
 else if ((get_data == FROM_FIFO)&&(IIM42352->FIFO == Enable)) 
	GB_IIM42352_Get_ACCEL_XYZ_TEMP_From_FIFO(IIM42352); 
}
	

/*----------------------------------------------------------------------------------------------------------------------------------------*
 *                                                                      End                                                               *
 *----------------------------------------------------------------------------------------------------------------------------------------*/

