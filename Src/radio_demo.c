#include <memory.h>
#include "support.h"
#include "nrf24.h"
//
// Created by ilia.motornyi on 13-Dec-18.
//
// Buffer to store a payload of maximum width


#define HEX_CHARS      "0123456789ABCDEF"


extern UART_HandleTypeDef huart2;

void UART_SendChar(char b) {
    HAL_UART_Transmit(&huart2, (uint8_t *) &b, 1, 200);
}

void UART_SendStr(char *string) {
    HAL_UART_Transmit(&huart2, (uint8_t *) string, (uint16_t) strlen(string), 200);
}

void Toggle_LED() {
    HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
}




void UART_SendBufHex(char *buf, uint16_t bufsize) {
    uint16_t i;
    char ch;
    for (i = 0; i < bufsize; i++) {
        ch = *buf++;
        UART_SendChar(HEX_CHARS[(ch >> 4) % 0x10]);
        UART_SendChar(HEX_CHARS[(ch & 0x0f) % 0x10]);
    }
}

void UART_SendHex8(uint16_t num) {
    UART_SendChar(HEX_CHARS[(num >> 4) % 0x10]);
    UART_SendChar(HEX_CHARS[(num & 0x0f) % 0x10]);
}

void UART_SendInt(int32_t num) {
    char str[10]; // 10 chars max for INT32_MAX
    int i = 0;
    if (num < 0) {
        UART_SendChar('-');
        num *= -1;
    }
    do str[i++] = (char) (num % 10 + '0'); while ((num /= 10) > 0);
    for (i--; i >= 0; i--) UART_SendChar(str[i]);
}



#define RX_SINGLE      		0
#define TX_SINGLE      		1
#define DEMO_TX_SINGLE_ESB 	0



// Kinda foolproof :)
#if ((RX_SINGLE + TX_SINGLE + DEMO_TX_SINGLE_ESB) != 1)
#error "Define only one DEMO_xx, use the '1' value"
#endif


#if ((TX_SINGLE + DEMO_TX_SINGLE_ESB))

// Helpers for transmit mode demo

// Timeout counter (depends on the CPU speed)
// Used for not stuck waiting for IRQ
#define nRF24_WAIT_TIMEOUT         (uint32_t)0x000FFFFF

// Result of packet transmission
typedef enum {
    nRF24_TX_ERROR  = (uint8_t)0x00, // Unknown error
    nRF24_TX_SUCCESS,                // Packet has been transmitted successfully
    nRF24_TX_TIMEOUT,                // It was timeout during packet transmit
    nRF24_TX_MAXRT                   // Transmit failed with maximum auto retransmit count
} nRF24_TXResult;
typedef struct
{
	uint8_t ID;
	uint8_t Data[10];
}NRF_Packet;

NRF_Packet payload_packet;

// Length of received payload
uint8_t payload_length = 10;
nRF24_TXResult nRF24_TransmitPacket(uint8_t *pBuf, uint8_t length) {
    volatile uint32_t wait = nRF24_WAIT_TIMEOUT;
    uint8_t status;
    nRF24_CE_L();
    nRF24_WritePayload(pBuf, length);
    nRF24_CE_H();

    do {
        status = nRF24_GetStatus();
        if (status & (nRF24_FLAG_TX_DS | nRF24_FLAG_MAX_RT)) {
            break;
        }
    } while (wait--);
    nRF24_CE_L();

    if (!wait)
        return nRF24_TX_TIMEOUT;

    nRF24_ClearIRQFlags();

    if (status & nRF24_FLAG_MAX_RT)
        return nRF24_TX_MAXRT;


    if (status & nRF24_FLAG_TX_DS)
        return nRF24_TX_SUCCESS;

    nRF24_FlushTX();

    return nRF24_TX_ERROR;
}

#endif // DEMO_TX_


int runRadio(void) {
    nRF24_CE_L();
    if (!nRF24_Check())
    {
        while (1)
        {
            Toggle_LED();
            Delay_ms(50);
        }
    }
    nRF24_Init();


/***************************************************************************/
#if (RX_SINGLE)

    nRF24_SetRFChannel(40);
    nRF24_SetDataRate(nRF24_DR_2Mbps);
    nRF24_SetCRCScheme(nRF24_CRC_2byte);
    nRF24_SetAddrWidth(3);
    static const uint8_t nRF24_ADDR[] = {'E', 'S', 'B'};
    nRF24_SetAddr(nRF24_PIPE1, nRF24_ADDR);
    nRF24_SetRXPipe(nRF24_PIPE1, nRF24_AA_ON, 10);
    nRF24_SetTXPower(nRF24_TXPWR_0dBm);
    nRF24_SetOperationalMode(nRF24_MODE_RX);
    nRF24_ClearIRQFlags();
    nRF24_SetPowerMode(nRF24_PWR_UP);
    nRF24_CE_H();


    // The main loop
    while (1) {
        if (nRF24_GetStatus_RXFIFO() != nRF24_STATUS_RXFIFO_EMPTY) {
        	nRF24_RXResult pipe = nRF24_ReadPayload((uint8_t*)&payload_packet, &payload_length);

            // Clear all pending IRQ flags
            nRF24_ClearIRQFlags();
            UART_SendStr("RCV PIPE#");
            UART_SendInt(pipe);
            UART_SendStr(" PAYLOAD:>");
            Toggle_LED();
            UART_SendBufHex((char *)&payload_packet, payload_length);
            UART_SendStr("<\r\n");
        }
    }

#endif //RX_SINGLE

/***************************************************************************/


#if (TX_SINGLE)

    nRF24_SetRFChannel(40);
    nRF24_SetDataRate(nRF24_DR_2Mbps);
    nRF24_SetCRCScheme(nRF24_CRC_2byte);
    nRF24_SetAddrWidth(3);
    static const uint8_t nRF24_ADDR[] = { 'E', 'S', 'B' };
    nRF24_SetAddr(nRF24_PIPETX, nRF24_ADDR);
    nRF24_SetAddr(nRF24_PIPE0, nRF24_ADDR);
    nRF24_SetTXPower(nRF24_TXPWR_0dBm);
    nRF24_SetAutoRetr(nRF24_ARD_2500us, 10);
    nRF24_EnableAA(nRF24_PIPE0);
    nRF24_SetOperationalMode(nRF24_MODE_TX);
    nRF24_ClearIRQFlags();
    nRF24_SetPowerMode(nRF24_PWR_UP);
    uint8_t i = 0;
    while (1)
    {

    		payload_packet.ID = i;
    		if(i > 10) i = 0;
        nRF24_TXResult result = nRF24_TransmitPacket((uint8_t*)&payload_packet, payload_length);
     	switch (result)
     	{
 			case nRF24_TX_SUCCESS:
 				//todo: đã truyền thành công
 				break;
 			case nRF24_TX_MAXRT:
 				nRF24_ResetPLOS();
 			case nRF24_TX_TIMEOUT:
 			default:
 				//todo: Bị lỗi khi truyền đi
 				break;
 		}
        i++;
        Toggle_LED();
        Delay_ms(500);
    }


#endif // DEMO_TX_SINGLE_ESB

}
