#include "support.h"
#include "nrf24.h"
#include "app_uart.h"
//
// Created by ilia.motornyi on 13-Dec-18.
//
// Buffer to store a payload of maximum width

extern UART_HandleTypeDef huart2;

uint8_t nRF24_payload[6];

void transmitRF(void);

// Pipe number
nRF24_RXResult pipe;

uint32_t i, j, k;

// Some variables
uint32_t packets_lost = 0; // global counter of lost packets
uint8_t otx;
uint8_t otx_plos_cnt; // lost packet count
uint8_t otx_arc_cnt;  // retransmit count

// Length of received payload
uint8_t payload_length;

#define DEMO_RX_SINGLE 0	 // Single address receiver (1 pipe)
#define DEMO_RX_MULTI 0		 // Multiple address receiver (3 pipes)
#define DEMO_RX_SOLAR 0		 // Solar temperature sensor receiver
#define DEMO_TX_SINGLE 0	 // Single address transmitter (1 pipe)
#define DEMO_TX_MULTI 0		 // Multiple address transmitter (3 pipes)
#define DEMO_RX_SINGLE_ESB 0 // Single address receiver with Enhanced ShockBurst (1 pipe)
#define DEMO_TX_SINGLE_ESB 0 // Single address transmitter with Enhanced ShockBurst (1 pipe)
#define DEMO_RX_ESB_ACK_PL 0 // Single address receiver with Enhanced ShockBurst (1 pipe) + payload sent back
#define DEMO_TX_ESB_ACK_PL 1 // Single address transmitter with Enhanced ShockBurst (1 pipe) + payload received in ACK

// Kinda foolproof :)
#if ((DEMO_RX_SINGLE + DEMO_RX_MULTI + DEMO_RX_SOLAR + DEMO_TX_SINGLE + DEMO_TX_MULTI + DEMO_RX_SINGLE_ESB + DEMO_TX_SINGLE_ESB + DEMO_RX_ESB_ACK_PL + DEMO_TX_ESB_ACK_PL) != 1)
#error "Define only one DEMO_xx, use the '1' value"
#endif

#if ((DEMO_TX_SINGLE) || (DEMO_TX_MULTI) || (DEMO_TX_SINGLE_ESB) || (DEMO_TX_ESB_ACK_PL))

// Helpers for transmit mode demo

// Timeout counter (depends on the CPU speed)
// Used for not stuck waiting for IRQ
#define nRF24_WAIT_TIMEOUT (uint32_t)0x000FFFFF

// Result of packet transmission
typedef enum
{
	nRF24_TX_ERROR = (uint8_t)0x00, // Unknown error
	nRF24_TX_SUCCESS,				// Packet has been transmitted successfully
	nRF24_TX_TIMEOUT,				// It was timeout during packet transmit
	nRF24_TX_MAXRT					// Transmit failed with maximum auto retransmit count
} nRF24_TXResult;

nRF24_TXResult tx_res;

// Function to transmit data packet
// input:
//   pBuf - pointer to the buffer with data to transmit
//   length - length of the data buffer in bytes
// return: one of nRF24_TX_xx values
nRF24_TXResult nRF24_TransmitPacket(uint8_t *pBuf, uint8_t length)
{
	volatile uint32_t wait = nRF24_WAIT_TIMEOUT;
	uint8_t status;

	// Deassert the CE pin (in case if it still high)
	nRF24_CE_L();

	// Transfer a data from the specified buffer to the TX FIFO
	nRF24_WritePayload(pBuf, length);

	// Start a transmission by asserting CE pin (must be held at least 10us)
	nRF24_CE_H();

	// Poll the transceiver status register until one of the following flags will be set:
	//   TX_DS  - means the packet has been transmitted
	//   MAX_RT - means the maximum number of TX retransmits happened
	// note: this solution is far from perfect, better to use IRQ instead of polling the status
	do
	{
		status = nRF24_GetStatus();
		if (status & (nRF24_FLAG_TX_DS | nRF24_FLAG_MAX_RT))
		{
			break;
		}
	} while (wait--);

	// Deassert the CE pin (Standby-II --> Standby-I)
	nRF24_CE_L();

	if (!wait)
	{
		// Timeout
		return nRF24_TX_TIMEOUT;
	}

	// Check the flags in STATUS register
	UART_SendStr("[");
	UART_SendHex8(status);
	UART_SendStr("] ");

	// Clear pending IRQ flags
	nRF24_ClearIRQFlags();

	if (status & nRF24_FLAG_MAX_RT)
	{
		// Auto retransmit counter exceeds the programmed maximum limit (FIFO is not removed)
		return nRF24_TX_MAXRT;
	}

	if (status & nRF24_FLAG_TX_DS)
	{
		// Successful transmission
		return nRF24_TX_SUCCESS;
	}

	// Some banana happens, a payload remains in the TX FIFO, flush it
	nRF24_FlushTX();

	return nRF24_TX_ERROR;
}

#endif // DEMO_TX_

void transmitRF(void)
{
	// payload_length = (uint8_t)(2 + (j + j /10)% 7);

	// // Prepare data packet
	// for (i = 0; i < payload_length; i++) {
	// 	nRF24_payload[i] = (uint8_t) j++;
	// 	if (j > 0x000000FF) j = 0;
	// }
	payload_length = sizeof(nRF24_payload);
	// Print a payload
	if ((nRF24_payload[0] == 0xbd) && (nRF24_payload[5] == 0xed))
	{
		UART_SendStr("Sent: ");
		UART_SendBufHex((char *)nRF24_payload, payload_length);
		// UART_SendStr("< ... TX: ");

		// Transmit a packet
		tx_res = nRF24_TransmitPacket(nRF24_payload, payload_length);
		otx = nRF24_GetRetransmitCounters();
		nRF24_ReadPayloadDpl(nRF24_payload, &payload_length);
		otx_plos_cnt = (otx & nRF24_MASK_PLOS_CNT) >> 4; // packets lost counter
		otx_arc_cnt = (otx & nRF24_MASK_ARC_CNT);		 // auto retransmissions counter
		switch (tx_res)
		{
		case nRF24_TX_SUCCESS:
			UART_SendStr("OK");
			break;
		case nRF24_TX_TIMEOUT:
			UART_SendStr("TIMEOUT");
			break;
		case nRF24_TX_MAXRT:
			UART_SendStr("MAX RETRANSMIT");
			packets_lost += otx_plos_cnt;
			nRF24_ResetPLOS();
			break;
		default:
			UART_SendStr("ERROR");
			break;
		}
	}
	// UART_SendStr("   ACK_PAYLOAD=>");
	// UART_SendBufHex((char *) nRF24_payload, payload_length);
	// UART_SendStr("<   ARC=");
	// UART_SendInt(otx_arc_cnt);
	// UART_SendStr(" LOST=");
	// UART_SendInt(packets_lost);
	UART_SendStr("\r\n");
}
int runRadio(void)
{
	UART_SendStr("\r\nSTM32L432KC is online.\r\n");

	// RX/TX disabled
	nRF24_CE_L();

	// Configure the nRF24L01+
	UART_SendStr("nRF24L01+ check: ");
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
	while (!nRF24_Check())
	{
		UART_SendStr("FAIL\r\n");

		Delay_ms(50);
	}
#pragma clang diagnostic pop
	UART_SendStr("OK\r\n");

	// Initialize the nRF24L01 to its default state
	nRF24_Init();

#if (DEMO_TX_ESB_ACK_PL)

	// This is simple transmitter with Enhanced ShockBurst (to one logic address):
	//   - TX address: 'ESB'
	//   - payload: 10 bytes
	//   - RF channel: 40 (2440MHz)
	//   - data rate: 2Mbps
	//   - CRC scheme: 2 byte

	// The transmitter sends a 10-byte packets to the address 'ESB' with Auto-ACK (ShockBurst enabled)

	// Set RF channel
	nRF24_SetRFChannel(10);

	// Set data rate
	nRF24_SetDataRate(nRF24_DR_250kbps);

	// Set CRC scheme
	nRF24_SetCRCScheme(nRF24_CRC_2byte);

	// Set address width, its common for all pipes (RX and TX)
	nRF24_SetAddrWidth(3);

	// Configure TX PIPE
	static const uint8_t nRF24_ADDR[] = {'L', 'I', 'S', 'Z', 'T'};
	nRF24_SetAddr(nRF24_PIPETX, nRF24_ADDR); // program TX address
	nRF24_SetAddr(nRF24_PIPE0, nRF24_ADDR);	 // program address for pipe#0, must be same as TX (for Auto-ACK)

	// Set TX power (maximum)
	nRF24_SetTXPower(nRF24_TXPWR_0dBm);

	// Configure auto retransmit: 10 retransmissions with pause of 2500s in between
	nRF24_SetAutoRetr(nRF24_ARD_2500us, 1);

	// Enable Auto-ACK for pipe#0 (for ACK packets)
	nRF24_EnableAA(nRF24_PIPE0);

	// Set operational mode (PTX == transmitter)
	nRF24_SetOperationalMode(nRF24_MODE_TX);

	// Clear any pending IRQ flags
	nRF24_ClearIRQFlags();

	// Enable DPL
	nRF24_SetDynamicPayloadLength(nRF24_DPL_ON);
	nRF24_SetPayloadWithAck(1);

	// Wake the transceiver
	nRF24_SetPowerMode(nRF24_PWR_UP);

	// The main loop
	j = 0;
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"

	// 	while (1)
	// 	{
	// #pragma clang diagnostic pop

	// 		// payload_length = (uint8_t)(2 + (j + j /10)% 7);

	// 		// // Prepare data packet
	// 		// for (i = 0; i < payload_length; i++) {
	// 		// 	nRF24_payload[i] = (uint8_t) j++;
	// 		// 	if (j > 0x000000FF) j = 0;
	// 		// }
	// 		for (i = 0; i < 4; i++)
	// 		{
	// 			nRF24_payload[i] = receive_buffer[i];
	// 		}
	// 		payload_length = sizeof(nRF24_payload);
	// 		// Print a payload
	// 		UART_SendStr("Sent: ");
	// 		UART_SendBufHex((char *)nRF24_payload, payload_length);
	// 		// UART_SendStr("< ... TX: ");

	// 		// Transmit a packet
	// 		tx_res = nRF24_TransmitPacket(nRF24_payload, payload_length);
	// 		otx = nRF24_GetRetransmitCounters();
	// 		nRF24_ReadPayloadDpl(nRF24_payload, &payload_length);
	// 		otx_plos_cnt = (otx & nRF24_MASK_PLOS_CNT) >> 4; // packets lost counter
	// 		otx_arc_cnt = (otx & nRF24_MASK_ARC_CNT);		 // auto retransmissions counter
	// 		switch (tx_res)
	// 		{
	// 		case nRF24_TX_SUCCESS:
	// 			UART_SendStr("OK");
	// 			break;
	// 		case nRF24_TX_TIMEOUT:
	// 			UART_SendStr("TIMEOUT");
	// 			break;
	// 		case nRF24_TX_MAXRT:
	// 			UART_SendStr("MAX RETRANSMIT");
	// 			packets_lost += otx_plos_cnt;
	// 			nRF24_ResetPLOS();
	// 			break;
	// 		default:
	// 			UART_SendStr("ERROR");
	// 			break;
	// 		}
	// 		// UART_SendStr("   ACK_PAYLOAD=>");
	// 		// UART_SendBufHex((char *) nRF24_payload, payload_length);
	// 		// UART_SendStr("<   ARC=");
	// 		// UART_SendInt(otx_arc_cnt);
	// 		// UART_SendStr(" LOST=");
	// 		// UART_SendInt(packets_lost);
	// 		UART_SendStr("\r\n");

	// 		// Wait ~0.5s
	// 		// Delay_ms(100);
	// 	}

#endif // DEMO_RX_ESB_ACK_PL
}