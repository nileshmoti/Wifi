/* Includes */
#include <stdlib.h>
#include "stdio.h"
#include "wiced.h"
#include "Nilesh_Wifi_Project.h"

/* Defines and Macros */
#define UDP_MAX_DATA_LENGTH         30
#define UDP_RX_TIMEOUT              1
#define UDP_TX_INTERVAL             1
#define UDP_RX_INTERVAL             1
#define UDP_TARGET_PORT             6789
#define UDP_TARGET_IS_BROADCAST
#define GET_UDP_RESPONSE

#ifdef UDP_TARGET_IS_BROADCAST
#define UDP_TARGET_IP MAKE_IPV4_ADDRESS(192,168,43,255)
#else
#define UDP_TARGET_IP MAKE_IPV4_ADDRESS(192,168,43,255)
#endif

// Ring buffer size
#define UART_RX_BUFFER_SIZE    64

// UART thread paramters
#define UART_THREAD_PRIORITY     (10)
#define UART_THREAD_STACK_SIZE   (1024)

//static wiced_result_t tx_udp_packet();
//static wiced_result_t rx_udp_packet();

// Thread handle
static wiced_thread_t uartThreadHandle;
static wiced_thread_t udpRxThreadHandle;

/* AP List
 *  AP1 = lab1
 *  AP2 = Huawei Router
 *
 * */

/* UART variables */
char c;
uint32_t expected_data_size = 1;

/* UART Config */
wiced_uart_config_t uart_config = { .baud_rate = 115200, .data_width = DATA_WIDTH_8BIT, .parity = NO_PARITY, .stop_bits = STOP_BITS_1, .flow_control = FLOW_CONTROL_DISABLED, };

/* UDP Variables */
static wiced_udp_socket_t udp_socket;
static uint32_t txUdpCount = 0;   //not needed
// For UDP TX
wiced_packet_t* packet;
char* udp_data;
uint16_t available_data_length;
const wiced_ip_address_t INITIALISER_IPV4_ADDRESS( target_ip_addr, UDP_TARGET_IP );
// For UDP RX
wiced_packet_t* packetUdpRx;
char* udp_dataUdpRx;
uint16_t data_lengthUdpRx;
uint16_t available_data_lengthUdpRx;

/* Ring buffer */
wiced_ring_buffer_t rxUartBuffer;
DEFINE_RING_BUFFER_DATA(uint8_t, rx_data, UART_RX_BUFFER_SIZE)

/****************************************
 *              Threads
 ****************************************/

/* UART Thread
 *
 * This thread receives UART messages and transmits the message via UDP
 *
 * */
void uartThread(wiced_thread_arg_t arg) {

    // Checks for receiving characters on WICED header J6.18(RX) and J6.20(TX)
    while (wiced_uart_receive_bytes(WICED_UART_2, &c, &expected_data_size, WICED_NEVER_TIMEOUT) == WICED_SUCCESS) {

        // Just transmit everything that is received back
        wiced_uart_transmit_bytes(WICED_UART_2, &c, 1);
        expected_data_size = 1;

        // UDP TX
        udp_data = &c;
        // Create a UDP packet
        wiced_packet_create_udp(&udp_socket, UDP_MAX_DATA_LENGTH, &packet, (uint8_t**) &udp_data, &available_data_length);


        /* Set the end of the data portion */
        //wiced_packet_set_data_end( packet, (uint8_t*) udp_data + UDP_MAX_DATA_LENGTH );
        wiced_packet_set_data_end( packet, (uint8_t*) udp_data);
        //wiced_packet_set_data_end( packet, (uint8_t*) &udp_data);
        // Send packet
        wiced_udp_send(&udp_socket, &target_ip_addr, UDP_TARGET_PORT, packet);
    }

}

/* UDP Thread
 *
 * This thread receives UDP messages and changes AP to a different AP
 * */
void udpRxThread(wiced_thread_arg_t arg) {
    while (1) {
        /* Wait for UDP packet */
        wiced_result_t result = wiced_udp_receive(&udp_socket, &packetUdpRx, 1);

        if ((result == WICED_ERROR) || (result == WICED_TIMEOUT)) {

        } else {

            wiced_packet_get_data(packetUdpRx, 0, (uint8_t**) &udp_dataUdpRx, &data_lengthUdpRx, &available_data_lengthUdpRx);

            /* Null terminate the received string */
            udp_dataUdpRx[data_lengthUdpRx] = '\x0';

            /* Print data out */
            WPRINT_APP_INFO(("%s\n\n", udp_dataUdpRx));


            /* From the received UDP message we can now switch to a different AP */
            if (udp_dataUdpRx[0] == '1'){
                WPRINT_APP_INFO(("AAAA"));
                ap1Connect();
            } else if (udp_dataUdpRx[0] == '2'){
                ap2Connect();
            }





            /* Delete packet as it is no longer needed */
            wiced_packet_delete(packetUdpRx);


        }
    }
}

void application_start() {

    /* Initializes the WICED system */
    wiced_init();

    /* Connect to Wifi */
    ap1Connect();

    /* Initializes UDP */
    udpInit();

    /* Initialise ring buffer for UART */
    ring_buffer_init(&rxUartBuffer, rx_data, UART_RX_BUFFER_SIZE);

    /* Initialise UART. A ring buffer is used to hold received characters
     * UART 2 Pinout: WICED header J6.18(RX) and J6.20(TX)               */
    wiced_uart_init(WICED_UART_2, &uart_config, &rxUartBuffer);
    /* Create a UART thread */
    wiced_rtos_create_thread(&uartThreadHandle, UART_THREAD_PRIORITY, "uartThread", uartThread, UART_THREAD_STACK_SIZE, NULL);

    /* Create a UDP thread, this thread will check for incoming UDP messages and connect to the respective AP */
    wiced_rtos_create_thread(&udpRxThreadHandle, UART_THREAD_PRIORITY, "udpRxThread", udpRxThread, UART_THREAD_STACK_SIZE, NULL);

}

/*
 * Sends a UDP packet
 */
wiced_result_t tx_udp_packet() {
    wiced_packet_t* packet;
    char* udp_data;
    uint16_t available_data_length;
    const wiced_ip_address_t INITIALISER_IPV4_ADDRESS( target_ip_addr, UDP_TARGET_IP );

    /* Create the UDP packet */
    if (wiced_packet_create_udp(&udp_socket, UDP_MAX_DATA_LENGTH, &packet, (uint8_t**) &udp_data, &available_data_length) != WICED_SUCCESS) {
        WPRINT_APP_INFO(("UDP tx packet creation failed\n"));
        return WICED_ERROR;
    }

    /* Write packet number into the UDP packet data */
    sprintf(udp_data, "%d", (int) txUdpCount++);

    /* Set the end of the data portion */
    wiced_packet_set_data_end(packet, (uint8_t*) udp_data + UDP_MAX_DATA_LENGTH);

    /* Send the UDP packet */
    if (wiced_udp_send(&udp_socket, &target_ip_addr, UDP_TARGET_PORT, packet) != WICED_SUCCESS) {
        WPRINT_APP_INFO(("UDP packet send failed\n"));
        wiced_packet_delete(packet); /* Delete packet, since the send failed */
        return WICED_ERROR;
    }

    /*
     * NOTE : It is not necessary to delete the packet created above, the packet
     *        will be automatically deleted *AFTER* it has been successfully sent
     */

    WPRINT_APP_INFO(("sent: %d\n", (int)txUdpCount));

    return WICED_SUCCESS;
}

/*
 * Attempts to receive a UDP packet
 */
wiced_result_t rx_udp_packet(uint32_t timeout) {
    wiced_packet_t* packetUdpRx;
    char* udp_dataUdpRx;
    uint16_t data_lengthUdpRx;
    uint16_t available_data_lengthUdpRx;

    /* Wait for UDP packet */
    wiced_result_t result = wiced_udp_receive(&udp_socket, &packetUdpRx, timeout);

    if ((result == WICED_ERROR) || (result == WICED_TIMEOUT)) {
        return result;
    }

    wiced_packet_get_data(packetUdpRx, 0, (uint8_t**) &udp_dataUdpRx, &data_lengthUdpRx, &available_data_lengthUdpRx);

    if (data_lengthUdpRx != available_data_lengthUdpRx) {
        WPRINT_APP_INFO(("Fragmented packets not supported\n"));
        return WICED_ERROR;
    }

    /* Null terminate the received string */
    udp_dataUdpRx[data_lengthUdpRx] = '\x0';

    WPRINT_APP_INFO(("%s\n\n", udp_dataUdpRx));

    /* Delete packet as it is no longer needed */
    wiced_packet_delete(packetUdpRx);

    return WICED_SUCCESS;
}

void udpInit() {
    wiced_udp_create_socket(&udp_socket, UDP_TARGET_PORT, WICED_STA_INTERFACE);
}

void ap1Connect() {
    // Connect to Wifi
    wiced_wifi_down();
    wiced_wifi_up();
    wiced_wifi_join_specific("lab1", 4, WICED_SECURITY_OPEN, NULL, 0, "00:11:12:33:44:55", "1", "192.168.43.30", "255.255.255.0", "192.168.43.1");
}

void ap2Connect() {
    wiced_wifi_down();
    wiced_wifi_up();
    wiced_wifi_join_specific("123", 10, WICED_SECURITY_OPEN, NULL, 0, "22:33:74:49:88:11", "4", "192.168.1.90", "255.255.255.0", "192.168.1.1");
}
