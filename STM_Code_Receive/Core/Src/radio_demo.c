
#include "support.h"
#include "nrf24.h"
#include <string.h>
#include "app_uart.h"

//
// Created by ilia.motornyi on 13-Dec-18.
//
// Buffer to store a payload of maximum width

uint8_t nRF24_payload[6];

// Pipe number
nRF24_RXResult pipe;

uint32_t i, j, k;

// Length of received payload
uint8_t payload_length;



void radio_receive(void)
{
    //
    // Constantly poll the status of the RX FIFO and get a payload if FIFO is not empty
    //
    // This is far from best solution, but it's ok for testing purposes
    // More smart way is to use the IRQ pin :)
    //
    //UART_SendStr("in radio: ");
    if (nRF24_GetStatus_RXFIFO() != nRF24_STATUS_RXFIFO_EMPTY)
    {
        // Get a payload from the transceiver
        pipe = nRF24_ReadPayload(nRF24_payload, &payload_length);

        // Clear all pending IRQ flags
        nRF24_ClearIRQFlags();
        //UART_SendBufHex((char *)nRF24_payload, sizeof(nRF24_payload));
        //  3            xx                                                                                         +++                                                                                                                                             UART_SendStr("\r\n");
        if ((nRF24_payload[0] == 0xbd) && (nRF24_payload[5] == 0xed))
        {
            // Print a payload contents to UART
            UART_SendStr("Received: ");
            // UART_SendInt(pipe);
            // UART_SendStr(" PAYLOAD:>");
            UART_SendBufHex((char *)nRF24_payload, sizeof(nRF24_payload));
            UART_SendStr("\r\n");
        }
    }
}

#define DEMO_RX_SINGLE 0     // Single address receiver (1 pipe)
#define DEMO_RX_MULTI 0      // Multiple address receiver (3 pipes)
#define DEMO_RX_SOLAR 0      // Solar temperature sensor receiver
#define DEMO_TX_SINGLE 0     // Single address transmitter (1 pipe)
#define DEMO_TX_MULTI 0      // Multiple address transmitter (3 pipes)
#define DEMO_RX_SINGLE_ESB 0 // Single address receiver with Enhanced ShockBurst (1 pipe)
#define DEMO_TX_SINGLE_ESB 0 // Single address transmitter with Enhanced ShockBurst (1 pipe)
#define DEMO_RX_ESB_ACK_PL 1 // Single address receiver with Enhanced ShockBurst (1 pipe) + payload sent back
#define DEMO_TX_ESB_ACK_PL 0 // Single address transmitter with Enhanced ShockBurst (1 pipe) + payload received in ACK

// Kinda foolproof :)
#if ((DEMO_RX_SINGLE + DEMO_RX_MULTI + DEMO_RX_SOLAR + DEMO_TX_SINGLE + DEMO_TX_MULTI + DEMO_RX_SINGLE_ESB + DEMO_TX_SINGLE_ESB + DEMO_RX_ESB_ACK_PL + DEMO_TX_ESB_ACK_PL) != 1)
#error "Define only one DEMO_xx, use the '1' value"
#endif

void runRadio(void)
{
    UART_SendStr("\r\nSTM32F303RE is online.\r\n");

    // RX/TX disabled
    nRF24_CE_L();

    // Configure the nRF24L01+
    UART_SendStr("nRF24L01+ check: ");

    while (!nRF24_Check())
    {
        UART_SendStr("FAIL\r\n");

        Delay_ms(50);
    }

    UART_SendStr("OK\r\n");

    // Initialize the nRF24L01 to its default state
    nRF24_Init();

#if (DEMO_RX_ESB_ACK_PL)

    // This is simple receiver with Enhanced ShockBurst:
    //   - RX address: 'ESB'
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

    // Configure RX PIPE
    static const uint8_t nRF24_ADDR[] = {'L', 'I', 'S', 'Z', 'T'};
    nRF24_SetAddr(nRF24_PIPE1, nRF24_ADDR);       // program address for pipe
    nRF24_SetRXPipe(nRF24_PIPE1, nRF24_AA_ON, 6); // Auto-ACK: enabled, payload length: 10 bytes

    // Set TX power for Auto-ACK (maximum, to ensure that transmitter will hear ACK reply)
    nRF24_SetTXPower(nRF24_TXPWR_0dBm);

    // Set operational mode (PRX == receiver)
    nRF24_SetOperationalMode(nRF24_MODE_RX);

    // Clear any pending IRQ flags
    nRF24_ClearIRQFlags();

    // Wake the transceiver
    nRF24_SetPowerMode(nRF24_PWR_UP);

    // Enable DPL
    nRF24_SetDynamicPayloadLength(nRF24_DPL_ON);

    nRF24_SetPayloadWithAck(1);

    // Put the transceiver to the RX mode
    nRF24_CE_H();

    // The main loop
    //#pragma clang diagnostic push
    //#pragma clang diagnostic ignored "-Wmissing-noreturn"

    //#pragma clang diagnostic pop

#endif // DEMO_RX_SINGLE_ESB
}
