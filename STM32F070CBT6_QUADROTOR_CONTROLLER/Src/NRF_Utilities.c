/* NRF24L01+ Utilities and various functions. Please refer to the datasheet of 
the device before using this file. */

#include "stm32f0xx_hal.h"
#include "NRF_Utilities.h"
#include "spi.h"
#include "gpio.h"


uint8_t *NRFSPITransmit(uint8_t *pData, uint16_t size){
  
  HAL_GPIO_WritePin(NRFCSPort, NRFCSPin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(NRFSPIHandle, pData, &rData[0], size, 10);
  while(!__HAL_SPI_GET_FLAG(NRFSPIHandle,SPI_FLAG_TXE));
  HAL_GPIO_WritePin(NRFCSPort, NRFCSPin, GPIO_PIN_SET);
  
  return &rData[0];
}

void SetupReceiver(){
  
  ConfigBuffer[0] = W_REGISTER | CONFIG;//Set device to RX and shutdown
  ConfigBuffer[1] = 0x09;
  NRFSPITransmit(&ConfigBuffer[0], 2);
  
  HAL_GPIO_WritePin(NRFCEPort, NRFCEPin, GPIO_PIN_SET);// Pull CE High for RX
  
  ConfigBuffer[0] = W_REGISTER | EN_AA;//Enable autoack only on pipe 0
  ConfigBuffer[1] = 0x01;
  NRFSPITransmit(&ConfigBuffer[0], 2);
  
  ConfigBuffer[0] = W_REGISTER | EN_RXADDR;//Enable RX pipe 0
  ConfigBuffer[1] = 0x01;
  NRFSPITransmit(&ConfigBuffer[0], 2);
  
  ConfigBuffer[0] = W_REGISTER | SETUP_AW;//Setup address width 3 bytes
  ConfigBuffer[1] = 0x01;
  NRFSPITransmit(&ConfigBuffer[0], 2);
  
  ConfigBuffer[0] = W_REGISTER | SETUP_RETR;//Disable Retransmission
  ConfigBuffer[1] = 0xF0;
  NRFSPITransmit(&ConfigBuffer[0], 2);
  
  ConfigBuffer[0] = W_REGISTER | RF_CH;//Setup RF Channel/Same as RX
  ConfigBuffer[1] = 0x02;
  NRFSPITransmit(&ConfigBuffer[0], 2);
  
  ConfigBuffer[0] = W_REGISTER | RF_SETUP;//Setup RF(250kbps, 0dbm)
  ConfigBuffer[1] = 0x26;
  NRFSPITransmit(&ConfigBuffer[0], 2);
  
  ConfigBuffer[0] = W_REGISTER | STATUS;//Clear Interrupts
  ConfigBuffer[1] = 0xF0;
  NRFSPITransmit(&ConfigBuffer[0], 2);
  
  ConfigBuffer[0] = W_REGISTER | RX_ADDR_P0;//Set RX Address
  ConfigBuffer[1] = 0xE7;
  ConfigBuffer[2] = 0xE7;
  ConfigBuffer[3] = 0xE7;
  NRFSPITransmit(&ConfigBuffer[0], 4);
  
  ConfigBuffer[0] = W_REGISTER | TX_ADDR;//Set TX Address
  ConfigBuffer[1] = 0xE7;
  ConfigBuffer[2] = 0xE7;
  ConfigBuffer[3] = 0xE7;
  NRFSPITransmit(&ConfigBuffer[0], 4);
  
  ConfigBuffer[0] = W_REGISTER | RX_PW_P0;//Set Payload length 32 Bytes
  ConfigBuffer[1] = 0x20;
  NRFSPITransmit(&ConfigBuffer[0], 2);
  
  ConfigBuffer[0] = W_REGISTER | DYNPD;//Disable dynamic payload
  ConfigBuffer[1] = 0x00;
  NRFSPITransmit(&ConfigBuffer[0], 2);
  
  ConfigBuffer[0] = W_REGISTER | FEATURE;//Disable features
  ConfigBuffer[1] = 0x00;
  NRFSPITransmit(&ConfigBuffer[0], 2);
  
  ConfigBuffer[0] = W_REGISTER | CONFIG;//Power Up device
  ConfigBuffer[1] = 0x0B;
  NRFSPITransmit(&ConfigBuffer[0], 2);
  
  ConfigBuffer[0] = FLUSH_TX;//Flush TX Buffer
  NRFSPITransmit(&ConfigBuffer[0], 1);
  
  ConfigBuffer[0] = FLUSH_RX;//Flush RX Buffer
  NRFSPITransmit(&ConfigBuffer[0], 1);
  
  return;
}

void SetupTransmitter(){
  
  ConfigBuffer[0] = W_REGISTER | CONFIG;//Set device to TX
  ConfigBuffer[1] = 0x08;
  NRFSPITransmit(&ConfigBuffer[0], 2);
  
  ConfigBuffer[0] = W_REGISTER | EN_AA;//Enable autoack only on pipe 0
  ConfigBuffer[1] = 0x01;
  NRFSPITransmit(&ConfigBuffer[0], 2);
  
  ConfigBuffer[0] = W_REGISTER | EN_RXADDR;//Enable RX pipe 0
  ConfigBuffer[1] = 0x01;
  NRFSPITransmit(&ConfigBuffer[0], 2);
  
  ConfigBuffer[0] = W_REGISTER | SETUP_AW;//Setup address width 3 bytes
  ConfigBuffer[1] = 0x01;
  NRFSPITransmit(&ConfigBuffer[0], 2);
  
  ConfigBuffer[0] = W_REGISTER | SETUP_RETR;//Disable Retransmission
  ConfigBuffer[1] = 0xF0;
  NRFSPITransmit(&ConfigBuffer[0], 2);
  
  ConfigBuffer[0] = W_REGISTER | RF_CH;//Setup RF Channel/Same as RX
  ConfigBuffer[1] = 0x02;
  NRFSPITransmit(&ConfigBuffer[0], 2);
  
  ConfigBuffer[0] = W_REGISTER | RF_SETUP;//Setup RF(250kbps, 0dbm)
  ConfigBuffer[1] = 0x26;
  NRFSPITransmit(&ConfigBuffer[0], 2);
  
  ConfigBuffer[0] = W_REGISTER | STATUS;//Clear Interrupts
  ConfigBuffer[1] = 0xF0;
  NRFSPITransmit(&ConfigBuffer[0], 2);
  
  ConfigBuffer[0] = W_REGISTER | RX_ADDR_P0;//Set RX Address
  ConfigBuffer[1] = 0xE7;
  ConfigBuffer[2] = 0xE7;
  ConfigBuffer[3] = 0xE7;
  NRFSPITransmit(&ConfigBuffer[0], 4);
  
  ConfigBuffer[0] = W_REGISTER | TX_ADDR;//Set TX Address
  ConfigBuffer[1] = 0xE7;
  ConfigBuffer[2] = 0xE7;
  ConfigBuffer[3] = 0xE7;
  NRFSPITransmit(&ConfigBuffer[0], 4);
  
  ConfigBuffer[0] = W_REGISTER | RX_PW_P0;//Set Payload length 32 Bytes
  ConfigBuffer[1] = 0x20;
  NRFSPITransmit(&ConfigBuffer[0], 2);
  
  ConfigBuffer[0] = W_REGISTER | DYNPD;//Disable dynamic payload
  ConfigBuffer[1] = 0x00;
  NRFSPITransmit(&ConfigBuffer[0], 2);
  
  ConfigBuffer[0] = W_REGISTER | FEATURE;//Disable features
  ConfigBuffer[1] = 0x00;
  NRFSPITransmit(&ConfigBuffer[0], 2);
  
  ConfigBuffer[0] = W_REGISTER | CONFIG;//Power Up device
  ConfigBuffer[1] = 0x0A;
  NRFSPITransmit(&ConfigBuffer[0], 2);
  
  ConfigBuffer[0] = FLUSH_TX;//Flush TX Buffer
  NRFSPITransmit(&ConfigBuffer[0], 1);
  
  ConfigBuffer[0] = FLUSH_RX;//Flush RX Buffer
  NRFSPITransmit(&ConfigBuffer[0], 1);
  
  return;
}

void TransmitPayload(uint8_t *payload, uint16_t size){
  
  ConfigBuffer[0] = FLUSH_TX;//Flush TX Buffer
  NRFSPITransmit(&ConfigBuffer[0], 1);
  
  ConfigBuffer[0] = W_REGISTER | STATUS;//Clear All Interrupts
  ConfigBuffer[1] = 0x70;
  NRFSPITransmit(&ConfigBuffer[0], 2);
  
  ConfigBuffer[0] = W_TX_PAYLOAD;//Fill TX payload
  for(uint8_t i = 1; i < size + 1; i++){//Load the payload
    ConfigBuffer[i] = payload[i-1];
  }
  
  NRFSPITransmit(&ConfigBuffer[0], 33);
  
  HAL_GPIO_WritePin(NRFCEPort, NRFCEPin, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(NRFCEPort, NRFCEPin, GPIO_PIN_RESET);
  
  return;
}

uint8_t *ReceivePayload(){
  
  ConfigBuffer[0] = R_RX_PAYLOAD;
  uint8_t *Payload = NRFSPITransmit(&ConfigBuffer[0], 33);
  
  return &Payload[1];//Payload[0] is status byte. Payload[1] begins the actual payload.
}

void SetAckPayload(uint8_t *Payload){
  ConfigBuffer[0] = W_ACK_PAYLOAD;
  for(uint8_t i = 1; i < 33; i++){
    ConfigBuffer[i] = Payload[i-1];
  }
  NRFSPITransmit(&ConfigBuffer[0], 33);
  
  return;
}

uint8_t *GetStatus(){
  ConfigBuffer[0] = NOP;
  return NRFSPITransmit(&ConfigBuffer[0], 1);
}

void ClearRXFlag(){
  ConfigBuffer[0] = W_REGISTER | STATUS;
  ConfigBuffer[1] = 0x40;
  NRFSPITransmit(&ConfigBuffer[0], 2);
}