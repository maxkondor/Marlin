/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "../../platforms.h"



#include "../../../inc/MarlinConfig.h"

#if HAS_SPI_TFT

#include "tft_spi.h"
#include "hc32f460_spi.h"

uint8_t TFT_SPI::data_type;

#define tft_RS_PORT                       (PortB)
#define tft_RS_PIN                        (Pin00)

#define tft_CS_PORT                     (PortB)
#define tft_CS_PIN                      (Pin01)

#define  BL_port                   PortC
#define  BL_pin                    Pin00

#define SPI_UNIT                        (M4_SPI2)
#define SPI_UNIT_CLOCK                  (PWC_FCG1_PERIPH_SPI2)

#define SPI_SCK_PORT                    (PortC)
#define SPI_SCK_PIN                     (Pin05)
#define SPI_SCK_FUNC                    (Func_Spi2_Sck)

#define SPI_MOSI_PORT                   (PortC)
#define SPI_MOSI_PIN                    (Pin04)
#define SPI_MOSI_FUNC                   (Func_Spi2_Mosi)

#define tft_MOSI_H()                      (PORT_SetBits(SPI_MOSI_PORT, SPI_MOSI_PIN))
#define tft_MOSI_L()                      (PORT_ResetBits(SPI_MOSI_PORT, SPI_MOSI_PIN))

#define tft_CS_H()                      (PORT_SetBits(tft_CS_PORT, tft_CS_PIN))
#define tft_CS_L()                      (PORT_ResetBits(tft_CS_PORT, tft_CS_PIN))

#define tft_RS_H()                      (PORT_SetBits(tft_RS_PORT, tft_RS_PIN))
#define tft_RS_L()                      (PORT_ResetBits(tft_RS_PORT, tft_RS_PIN))


void LCD_GPIO_Init(void)
{
   uint8_t index;
  stc_spi_init_t stcSpiInit;
  stc_port_init_t stcPortInit;
    MEM_ZERO_STRUCT(stcPortInit);
  
  #ifndef moni_spi
     tft_RS_L(); 
     PORT_SetFunc(tft_RS_PORT, tft_RS_PIN, Func_Gpio, Disable);
    stcPortInit.enPinMode = Pin_Mode_Out;
    PORT_Init(tft_RS_PORT, tft_RS_PIN, &stcPortInit);
    tft_RS_H();
  
//      stcPortInit.enPinMode = Pin_Mode_In;
//    PORT_Init(tft_RES_PORT, tft_RES_PIN, &stcPortInit);
//    tft_RES_L() ;
//    for(index=0;index<50;index++)
//              {
//               delay_ms(50000);
//              }
//     tft_RES_H();
//    for(index=0;index<200;index++)
//              {
//               delay_ms(50000);
//              }
//    tft_RES_L() ;
//    for(index=0;index<50;index++)
//              {
//               delay_ms(50000);
//              }
//     tft_RES_H();
//    for(index=0;index<200;index++)
//              {
//               delay_ms(50000);
//              }   

    PORT_Init(BL_port, BL_pin, &stcPortInit);
    //BL_ON;
    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcSpiInit);

    /* Configuration peripheral clock */
    PWC_Fcg1PeriphClockCmd(SPI_UNIT_CLOCK, Enable);

    /* Configuration SPI pin */
    PORT_SetFunc(SPI_SCK_PORT, SPI_SCK_PIN, SPI_SCK_FUNC, Disable);
    PORT_SetFunc(tft_CS_PORT, tft_CS_PIN, Func_Spi2_Nss0, Disable);
    PORT_SetFunc(SPI_MOSI_PORT, SPI_MOSI_PIN, SPI_MOSI_FUNC, Disable);
 //   PORT_SetFunc(SPI_MISO_PORT, SPI_MISO_PIN, SPI_MISO_FUNC, Disable);

    /* Configuration SPI structure */
    stcSpiInit.enClkDiv = SpiClkDiv2;
    stcSpiInit.enFrameNumber = SpiFrameNumber1;
    stcSpiInit.enDataLength = SpiDataLengthBit8;
    stcSpiInit.enFirstBitPosition = SpiFirstBitPositionMSB;
    stcSpiInit.enSckPolarity = SpiSckIdleLevelLow;
    stcSpiInit.enSckPhase = SpiSckOddSampleEvenChange;
    stcSpiInit.enReadBufferObject = SpiReadReceiverBuffer;
    stcSpiInit.enWorkMode = SpiWorkMode4Line;
    stcSpiInit.enTransMode = SpiTransFullDuplex;
    stcSpiInit.enCommAutoSuspendEn = Disable;
    stcSpiInit.enModeFaultErrorDetectEn = Disable;
    stcSpiInit.enParitySelfDetectEn = Disable;
    stcSpiInit.enParityEn = Disable;
    stcSpiInit.enParity = SpiParityEven;


    stcSpiInit.enMasterSlaveMode = SpiModeMaster;
    stcSpiInit.stcDelayConfig.enSsSetupDelayOption = SpiSsSetupDelayCustomValue;
    stcSpiInit.stcDelayConfig.enSsSetupDelayTime = SpiSsSetupDelaySck1;
    stcSpiInit.stcDelayConfig.enSsHoldDelayOption = SpiSsHoldDelayCustomValue;
    stcSpiInit.stcDelayConfig.enSsHoldDelayTime = SpiSsHoldDelaySck1;
    stcSpiInit.stcDelayConfig.enSsIntervalTimeOption = SpiSsIntervalCustomValue;
    stcSpiInit.stcDelayConfig.enSsIntervalTime = SpiSsIntervalSck6PlusPck2;
    stcSpiInit.stcSsConfig.enSsValidBit = SpiSsValidChannel0;
    stcSpiInit.stcSsConfig.enSs0Polarity = SpiSsLowValid;


#ifdef SPI_SLAVE_MODE
    stcSpiInit.enMasterSlaveMode = SpiModeSlave;
    stcSpiInit.stcSsConfig.enSsValidBit = SpiSsValidChannel0;
    stcSpiInit.stcSsConfig.enSs0Polarity = SpiSsLowValid;
#endif

    SPI_Init(SPI_UNIT, &stcSpiInit);
    SPI_Cmd(SPI_UNIT, Enable);
    
   #else
    PORT_SetFunc(tft_CS_PORT, tft_CS_PIN, Func_Gpio, Disable);
  
    stcPortInit.enPinMode = Pin_Mode_Out;
    PORT_Init(tft_CS_PORT, tft_CS_PIN, &stcPortInit);
    tft_CS_H();
       PORT_Init(BL_port, BL_pin, &stcPortInit);
    //BL_ON;
    PORT_SetFunc(tft_RS_PORT, tft_RS_PIN, Func_Gpio, Disable);
    stcPortInit.enPinMode = Pin_Mode_Out;
    PORT_Init(tft_RS_PORT, tft_RS_PIN, &stcPortInit);
    
    
     PORT_Init(SPI_SCK_PORT, SPI_SCK_PIN, &stcPortInit);
     tft_SCK_H();
     PORT_Init(SPI_MOSI_PORT, SPI_MOSI_PIN, &stcPortInit);
    #endif
}


static void LCD_Writ_Bus(uint8_t dat) 
{	
  #ifndef moni_spi
      //SPI_ReceiveData8(SPI_UNIT);
      //SPI_UNIT->DR;
	  //SPI_UNIT->DR = dat;
      SPI_SendData8(SPI_UNIT, dat);
            /* Wait rx buffer full */
      while (Reset == SPI_GetFlag(SPI_UNIT, SpiFlagSendBufferEmpty));
            
            
      
  #else
  char i;
 	tft_CS_L();
	//LCD_GPIO_PORRC |= SCLK;
  	for(i=0;i<8;i++)
	{		
     delay_ms(1);	  
	 tft_SCK_L();
	 //LCD_GPIO_PORRC |= SCLK;
		if(dat&0x80)
		{
		   tft_MOSI_H();
		   //LCD_GPIO_POSRC |= MOSI;
		}
		else
		{
		   tft_MOSI_L();
		   //LCD_GPIO_PORRC |= MOSI
		}
		tft_SCK_H();
		//LCD_GPIO_POSRC |= SCLK;
		dat<<=1;
	}	
  tft_CS_H();
	//LCD_GPIO_POSRC |= SCLK;
  #endif

}

static void LCD_WR_DATA8(uint8_t dat)
{

	LCD_Writ_Bus(dat);
}

static void LCD_WR_DATA(uint16_t dat)
{
	LCD_Writ_Bus(dat>>8);
	LCD_Writ_Bus(dat);
}

static void LCD_WR_REG(uint8_t dat)
{
	tft_RS_L();//д����

	LCD_Writ_Bus(dat);
  	delay(1);
	tft_RS_H();//д����

}


void TFT_SPI::init() {
    LCD_GPIO_Init();
      data_type=DATASIZE_16BIT;
}

void TFT_SPI::dataTransferBegin(uint16_t DataSize) {
     data_type=DataSize&0xff;
}

void TFT_SPI::writeData(uint16_t Data)
{
  if(data_type==DATASIZE_16BIT)
    LCD_WR_DATA(Data);
  else
    LCD_WR_DATA8(Data&0xff);
    
}

void TFT_SPI::writeSequence(uint16_t *Data, uint16_t Count)
{
  uint16_t i;
    for(i=0;i<Count;i++)
    {
      LCD_WR_DATA(Data[i]);
    }
}

void TFT_SPI::writeMultiple(uint16_t Color, uint32_t Count) 
{
        uint16_t i;
    for(i=0;i<Count;i++)
    {
      LCD_WR_DATA(Color);
    }
}

void TFT_SPI::writeReg(uint16_t Reg)   
{ 
  LCD_WR_REG(Reg&0xff);
}

#ifdef TFT_DEFAULT_DRIVER
  #include "../../../lcd/tft_io/tft_ids.h"
#endif

uint32_t TFT_SPI::getID() {
//  uint32_t id;
//  id = ReadID(LCD_READ_ID);
//  if ((id & 0xFFFF) == 0 || (id & 0xFFFF) == 0xFFFF) {
//    id = ReadID(LCD_READ_ID4);
//    #ifdef TFT_DEFAULT_DRIVER
//      if ((id & 0xFFFF) == 0 || (id & 0xFFFF) == 0xFFFF)
//        id = TFT_DEFAULT_DRIVER;
//    #endif
//   }
//  return id;
  return 0;
}

uint32_t TFT_SPI::ReadID(uint16_t Reg) {
//  uint32_t Data = 0;
//  #if PIN_EXISTS(TFT_MISO)
//    uint32_t BaudRatePrescaler = SPIx.Init.BaudRatePrescaler;
//    uint32_t i;

//    SPIx.Init.BaudRatePrescaler = SPIx.Instance == SPI1 ? SPI_BAUDRATEPRESCALER_8 : SPI_BAUDRATEPRESCALER_4;
//    DataTransferBegin(DATASIZE_8BIT);
//    WriteReg(Reg);

//    if (SPIx.Init.Direction == SPI_DIRECTION_1LINE) SPI_1LINE_RX(&SPIx);
//    __HAL_SPI_ENABLE(&SPIx);

//    for (i = 0; i < 4; i++) {
//      #if TFT_MISO_PIN != TFT_MOSI_PIN
//        //if (hspi->Init.Direction == SPI_DIRECTION_2LINES) {
//          while ((SPIx.Instance->SR & SPI_FLAG_TXE) != SPI_FLAG_TXE) {}
//          SPIx.Instance->DR = 0;
//        //}
//      #endif
//      while ((SPIx.Instance->SR & SPI_FLAG_RXNE) != SPI_FLAG_RXNE) {}
//      Data = (Data << 8) | SPIx.Instance->DR;
//    }

//    __HAL_SPI_DISABLE(&SPIx);
//    DataTransferEnd();

//    SPIx.Init.BaudRatePrescaler   = BaudRatePrescaler;
//  #endif

//  return Data >> 7;
    return 0;
}

bool TFT_SPI::isBusy() {


//    volatile bool dmaEnabled = (DMAtx.Instance->CCR & DMA_CCR_EN) != RESET;

//  if (dmaEnabled) {
//    if (__HAL_DMA_GET_FLAG(&DMAtx, __HAL_DMA_GET_TC_FLAG_INDEX(&DMAtx)) != 0 || __HAL_DMA_GET_FLAG(&DMAtx, __HAL_DMA_GET_TE_FLAG_INDEX(&DMAtx)) != 0)
//      Abort();
//  }
//  else
//    Abort();
//  return dmaEnabled;
  return 0;
}

void TFT_SPI::abort() {
//  // Wait for any running spi
//  while ((SPIx.Instance->SR & SPI_FLAG_TXE) != SPI_FLAG_TXE) {}
//  while ((SPIx.Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY) {}
//  // First, abort any running dma
//  HAL_DMA_Abort(&DMAtx);
//  // DeInit objects
//  HAL_DMA_DeInit(&DMAtx);
//  HAL_SPI_DeInit(&SPIx);
//  // Deselect CS
//  DataTransferEnd();
}

void TFT_SPI::Transmit(uint16_t Data) {
     if( data_type==DATASIZE_16BIT)
     {
       LCD_WR_DATA(Data); 
     }
     else
     {
       LCD_WR_DATA8(Data&0xff);
     }
//  if (TFT_MISO_PIN == TFT_MOSI_PIN)
//    SPI_1LINE_TX(&SPIx);

//  __HAL_SPI_ENABLE(&SPIx);

//  SPIx.Instance->DR = Data;

//  while ((SPIx.Instance->SR & SPI_FLAG_TXE) != SPI_FLAG_TXE) {}
//  while ((SPIx.Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY) {}

//  if (TFT_MISO_PIN != TFT_MOSI_PIN)
//    __HAL_SPI_CLEAR_OVRFLAG(&SPIx);   // Clear overrun flag in 2 Lines communication mode because received is not read
}

void TFT_SPI::TransmitDMA(uint32_t MemoryIncrease, uint16_t *Data, uint16_t Count) {
  // Wait last dma finish, to start another
//  while (isBusy()) { /* nada */ }

//  DMAtx.Init.MemInc = MemoryIncrease;
//  HAL_DMA_Init(&DMAtx);

//  if (TFT_MISO_PIN == TFT_MOSI_PIN)
//    SPI_1LINE_TX(&SPIx);

//  DataTransferBegin();

//  HAL_DMA_Start(&DMAtx, (uint32_t)Data, (uint32_t)&(SPIx.Instance->DR), Count);
//  __HAL_SPI_ENABLE(&SPIx);

//  SET_BIT(SPIx.Instance->CR2, SPI_CR2_TXDMAEN);   // Enable Tx DMA Request

//  HAL_DMA_PollForTransfer(&DMAtx, HAL_DMA_FULL_TRANSFER, HAL_MAX_DELAY);
//  Abort();
}

#endif // HAS_SPI_TFT

