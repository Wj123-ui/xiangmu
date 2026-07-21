#include "bsp_spi.h"

uint8_t spi_read_write_byte(uint8_t dat)
{
        uint8_t data = 0;
        
        //��������
        DL_SPI_transmitDataBlocking8(SPI_ICM45686_INST, dat);
        //�ȴ�SPI���߿���
        while(DL_SPI_isBusy(SPI_ICM45686_INST));
        //��������
        data = DL_SPI_receiveDataBlocking8(SPI_ICM45686_INST);
        //�ȴ�SPI���߿���
        
        return data;
}

































