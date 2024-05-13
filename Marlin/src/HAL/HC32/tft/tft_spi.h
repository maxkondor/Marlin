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
#pragma once
//#include "bsp_spi_tft.h"
#include "stdint.h"
#ifndef LCD_READ_ID
  #define LCD_READ_ID 0x04   // Read display identification information (0xD3 on ILI9341)
#endif
#ifndef LCD_READ_ID4
  #define LCD_READ_ID4 0xD3   // Read display identification information (0xD3 on ILI9341)
#endif

#define DATASIZE_8BIT    0//SPI_DATASIZE_8BIT
#define DATASIZE_16BIT   1//SPI_DATASIZE_16BIT
#define DMA_MAX_WORDS  0xFFFF
#define TFT_IO_DRIVER TFT_SPI

class TFT_SPI {
private:
//  static SPI_HandleTypeDef SPIx;
//  static DMA_HandleTypeDef DMAtx;
  static uint8_t data_type;
  static uint32_t ReadID(uint16_t Reg);
  static void Transmit(uint16_t Data);
  static void TransmitDMA(uint32_t MemoryIncrease, uint16_t *Data, uint16_t Count);

public:
  static void init();
  static uint32_t getID();
  static bool isBusy();
  static void abort();

  static void dataTransferBegin(uint16_t DataWidth);
  static void dataTransferEnd() { }//WRITE(TFT_CS_PIN, HIGH); };
  static void dataTransferAbort();

  static void writeData(uint16_t Data);
  static void writeReg(uint16_t Reg);//WRITE(TFT_A0_PIN, LOW); Transmit(Reg); WRITE(TFT_A0_PIN, HIGH); }

  static void writeSequence(uint16_t *Data, uint16_t Count);

  static void writeMultiple(uint16_t Color, uint32_t Count);
  static void writeSequence_DMA(uint16_t *data, uint16_t count);
  static void writeMultiple_DMA(uint16_t color, uint16_t count);
    /*
    static uint16_t Data; Data = Color;
    while (Count > 0) {
      TransmitDMA(DMA_MINC_DISABLE, &Data, Count > 0xFFFF ? 0xFFFF : Count);
      Count = Count > 0xFFFF ? Count - 0xFFFF : 0;
    }
  }
    */
};
