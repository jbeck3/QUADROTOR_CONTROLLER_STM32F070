/* NRF24L01+ Utilities and various functions. Please refer to the datasheet of 
the device before using this file. Also be sure to check the SPI peripheral Section*/

#ifndef __NRF_Utilities_H
#define __NRF_Utilities_H

  #ifdef __cplusplus
   extern "C" {
  #endif
     
    #include "stm32f0xx_hal.h"
    #include "spi.h"
    #include "gpio.h"

    /*----------------------NRF Commands--------------------------------------*/
    uint8_t R_REGISTER          =0x00,
            W_REGISTER          =0x20,
            R_RX_PAYLOAD        =0x61,
            W_TX_PAYLOAD        =0xA0,
            FLUSH_TX            =0xE1,
            FLUSH_RX            =0xE2,
            REUSE_TX_PL         =0xE3,
            R_RX_PL_WID         =0x60,
            W_ACK_PAYLOAD       =0xA8,
            W_TX_PAYLOAD_NOACK  =0xB0,
            NOP                 =0xFF;
    /*------------------------------------------------------------------------*/

    /*----------------------NRF Registers-------------------------------------*/
    uint8_t CONFIG              =0x00,
            EN_AA               =0x01,
            EN_RXADDR           =0x02,
            SETUP_AW            =0x03,
            SETUP_RETR          =0x04,
            RF_CH               =0x05,
            RF_SETUP            =0x06,
            STATUS              =0x07,
                RX_DR           =0x40,
                TX_DS           =0x20,
                MAX_RT          =0x10,
                TX_FULL         =0x01,
            OBSERVE_TX          =0x08,
            RPD                 =0x09,
            RX_ADDR_P0          =0x0A,
            RX_ADDR_P1          =0x0B,
            RX_ADDR_P2          =0x0C,
            RX_ADDR_P3          =0x0D,
            RX_ADDR_P4          =0x0E,
            RX_ADDR_P5          =0x0F,
            TX_ADDR             =0x10,
            RX_PW_P0            =0x11,
            RX_PW_P1            =0x12,
            RX_PW_P2            =0x13,
            RX_PW_P3            =0x14,
            RX_PW_P4            =0x15,
            RX_PW_P5            =0x16,
            FIFO_STATUS         =0x17,
            DYNPD               =0x1C,
            FEATURE             =0x1D;
    /*------------------------------------------------------------------------*/
    
    /*----------------------NRF SPI Peripheral--------------------------------*/
    SPI_HandleTypeDef *NRFSPIHandle = &hspi2;
    GPIO_TypeDef *NRFCSPort = CS_NRF_GPIO_Port;
    uint16_t NRFCSPin = CS_NRF_Pin;
    GPIO_TypeDef *NRFCEPort = CE_NRF_GPIO_Port;
    uint16_t NRFCEPin = CE_NRF_Pin;
    /*------------------------------------------------------------------------*/  

    /*----------------------NRF Buffers---------------------------------------*/
    uint8_t rData[33];
    uint8_t ConfigBuffer[33] = NULL;
    uint8_t ReceivedPayload[32] = NULL;
    uint8_t *ReceivedPayloadP;
    /*------------------------------------------------------------------------*/

    /*----------------------Function Prototypes-------------------------------*/
    uint8_t *NRFSPITransmit(uint8_t *pData, uint16_t size);

    void SetupReceiver();

    void SetupTransmitter();

    void TransmitPayload(uint8_t *payload, uint16_t size);

    uint8_t *ReceivePayload();
    
    void SetAckPayload(uint8_t *Payload);
    
    uint8_t *GetStatus();
    
    void ClearRX_DR();
    
    uint8_t *IRQHandler();
    
    uint8_t *GetPayload();
    /*------------------------------------------------------------------------*/

  #ifdef __cplusplus
  }
  #endif
#endif