/*
 * Copyright (C) 2014 BlueKitchen GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 4. Any redistribution, use, or modification is done solely for
 *    personal benefit and not for any commercial purpose or for
 *    monetary gain.
 *
 * THIS SOFTWARE IS PROVIDED BY BLUEKITCHEN GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL MATTHIAS
 * RINGWALD OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Please inquire about commercial licensing options at
 * contact@bluekitchen-gmbh.com
 *
 */

#define __BTSTACK_FILE__ "tws_m2.c"

/*
 * tws_m2.c
 */

// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include <mqueue.h>
#include <message.h>

#include "bt_manager.h"
#include "btstack.h"
#include "hci.h"

#define RFCOMM_SERVER_CHANNEL 1

#define TEST_COD 0x5a020c 
//#define TEST_COD 0x1234

static btstack_packet_callback_registration_t hci_event_callback_registration;

// SPP
static uint8_t   tws_m_service_buffer[150];

static uint16_t  rfcomm_mtu;
static uint16_t  rfcomm_cid = 0;

static uint32_t  start_send_time = 0xffffffff;

static uint32_t  test_count = 800;
static uint8_t   test_data[1021];

static bd_addr_t tws_e1_addr = {0x08, 0xb7, 0xe1, 0xde, 0x03, 0x9a};
static bd_addr_t tws_e2_addr = {0x08, 0xb7, 0xe2, 0xde, 0xc5, 0x02};
static int g_s_cnt = 0;
static void tws_m_send_packet(void){
    uint32_t now[8];
//    now[0] = btstack_run_loop_get_time_ms();

//    printf("tws_m_send_packet %ld, %ld\n", now[0], start_send_time);

//    if (now[0] > start_send_time)
    {
if(g_s_cnt >= 10) {
 g_s_cnt = 0;
printf("\n");
}
g_s_cnt ++;
        printf("SEND %u, ", test_count);
//        rfcomm_send(rfcomm_cid, (uint8_t*)&now, 32);//sizeof(uint32_t));
        test_data[0] = test_count;
        test_data[1] = test_count >> 8;

        rfcomm_send(rfcomm_cid, test_data, test_count);
//        test_count++;
//        if (test_count > 50)
//            test_count = 1;
    }
    uint32_t cnt = 0xffff;
//    while (cnt--);
//    cnt = 0xffffff;
//    while (cnt--);
//    printf("call rfcomm_request_can_send_now_event %d\n", rfcomm_cid);
    rfcomm_request_can_send_now_event(rfcomm_cid);
//    printf("call return\n");
}

/*
 * @section Packet Handler
 *
 * @text The packet handler of the combined example is just the combination of the individual packet handlers.
 */
int disconn_flag = 0;
static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel);
    UNUSED(size);

    bd_addr_t event_addr;
    uint8_t   rfcomm_channel_nr;

//    printf("packet_handler %d\n", packet_type);
    switch (packet_type) {
        case HCI_EVENT_PACKET:
            switch (hci_event_packet_get_type(packet)) {

                case HCI_EVENT_COMMAND_COMPLETE:
                   break;

                case HCI_EVENT_PIN_CODE_REQUEST:
                    // inform about pin code request
                    printf("Pin code request - using '0000'\n");
                    hci_event_pin_code_request_get_bd_addr(packet, event_addr);
                    gap_pin_code_response(event_addr, "0000");
                    break;

                case HCI_EVENT_USER_CONFIRMATION_REQUEST:
                    // inform about user confirmation request
                    printf("SSP User Confirmation Request with numeric value '%06"PRIu32"'\n", little_endian_read_32(packet, 8));
                    printf("SSP User Confirmation Auto accept\n");
                    break;

                case RFCOMM_EVENT_INCOMING_CONNECTION:
					// data: event (8), len(8), address(48), channel (8), rfcomm_cid (16)
                    rfcomm_event_incoming_connection_get_bd_addr(packet, event_addr);
                    rfcomm_channel_nr = rfcomm_event_incoming_connection_get_server_channel(packet);
                    rfcomm_cid = rfcomm_event_incoming_connection_get_rfcomm_cid(packet);
                    printf("RFCOMM channel %u requested for %s\n", rfcomm_channel_nr, bd_addr_to_str(event_addr));
                    rfcomm_accept_connection(rfcomm_cid);
					break;

		case RFCOMM_EVENT_CHANNEL_OPENED:
                   // data: event(8), len(8), status (8), address (48), server channel(8), rfcomm_cid(16), max frame size(16)
                    if (rfcomm_event_channel_opened_get_status(packet)) {
                        printf("RFCOMM channel open failed, status %u\n", rfcomm_event_channel_opened_get_status(packet));
disconn_flag = 0;
                        rfcomm_create_channel(packet_handler, tws_e2_addr, RFCOMM_SERVER_CHANNEL, NULL);
                    } else {
                        rfcomm_cid = rfcomm_event_channel_opened_get_rfcomm_cid(packet);
                        rfcomm_mtu = rfcomm_event_channel_opened_get_max_frame_size(packet);
                        printf("RFCOMM channel open succeeded. New RFCOMM Channel ID %u, max frame size %u\n", rfcomm_cid, rfcomm_mtu);

                        // disable page/inquiry scan to get max performance
                        gap_discoverable_control(0);
                        gap_connectable_control(0);

                        //start_send_time = btstack_run_loop_get_time_ms() + 5;
                        rfcomm_request_can_send_now_event(rfcomm_cid);
                    }

                    break;

                case RFCOMM_EVENT_CAN_SEND_NOW:
		    tws_m_send_packet();
                    break;

                case RFCOMM_EVENT_CHANNEL_CLOSED:
                    printf("RFCOMM channel closed, reconnect ...\n");
                    rfcomm_cid = 0;
disconn_flag = 1;
 
rfcomm_create_channel(packet_handler, tws_e2_addr, RFCOMM_SERVER_CHANNEL, NULL);

                    // re-enable page/inquiry scan again
                    gap_discoverable_control(1);
                    gap_connectable_control(1);

                    break;

                case BTSTACK_EVENT_NR_CONNECTIONS_CHANGED:
//                    printf("reconnect ... rfcomm_cid=%d\n", rfcomm_cid);
                    if (disconn_flag == 1)
                    {
disconn_flag = 0;
//                         rfcomm_create_channel(packet_handler, tws_e2_addr, RFCOMM_SERVER_CHANNEL, NULL);
                    }
                    break;
                case BTSTACK_EVENT_STATE:
                    printf("BTSTACK_EVENT_STATE\n");
                    if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) return;
                    printf("Connecting to E2: %s\n", bd_addr_to_str(tws_e2_addr));
disconn_flag = 0;
                    rfcomm_create_channel(packet_handler, tws_e2_addr, RFCOMM_SERVER_CHANNEL, NULL);
                    break;

                default:
                    break;
            }
            break;

        case RFCOMM_DATA_PACKET:
            {
                    //uint32_t tmp = packet[3] << 24 | packet[2] << 16 | packet[1] << 8 | packet[0];
                    //printf("RCV: %u\n", tmp);
                    printf("RCV: %u\n", size);
            }
            break;

        default:
            break;
	}
}

/*
 * @section Main Application Setup
 *
 * @text As with the packet and the heartbeat handlers, the combined app setup contains the code from the individual example setups.
 */
static void* g_rcvr = NULL;
static bt_handle_t    bt_handle;

static int message_handler(message_t *msg, uint16_t len)
{
    uint16_t id = msg->id;

    if (id == MSG_EXIT_LOOP)
        return MESSAGE_EXIT_LOOP;

    return MESSAGE_HANDLED;
}

/* LISTING_START(MainConfiguration): Init L2CAP RFCOMM SDO SM ATT Server and start heartbeat timer */
#include <nuttx/clk/clk.h>
#include <nuttx/clk/clk-provider.h>
volatile uint32_t g_test_val;
int tws_m2_main(int argc, const char * argv[]);
int tws_m2_main(int argc, const char * argv[])
{
    UNUSED(argc);
    (void)argv;
/*
uint32_t i;

struct clk *cpu_clk;

cpu_clk = clk_get("clk_cpu_sys");

if (!cpu_clk)
{
    clk_set_rate(cpu_clk, 102400000);
}
uint32_t temp1, temp2;
temp1 = *((volatile unsigned long *)0xa00e0294);
temp2 = *((volatile unsigned long *)0xa00e0298);

for (i = 0; i < 100000; i ++)
{
g_test_val ++;
}

printf("loop done: %ld %ld %ld %ld %ld\n", temp1, temp2, *((volatile unsigned long *)0xa00e0294), *((volatile unsigned long *)0xa00e0298), g_test_val );
*/
int i;
for (i = 0; i < 800; i ++)
   test_data[i] = (i & 0xFF);

    g_rcvr = message_init(&message_handler, 0, 0);
    if(g_rcvr == NULL)
    {
       return -1;
    }

    bt_handle = bt_manager_register(g_rcvr);
    bt_manager_open();

    // register for HCI events
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    hci_set_bd_addr(tws_e1_addr);

    rfcomm_set_required_security_level(LEVEL_0);
    printf("rfcomm register service\n");
    rfcomm_register_service(packet_handler, RFCOMM_SERVER_CHANNEL, 0xffff);
/*
    printf("spp_create_sdp_record\n");
    memset(tws_m_service_buffer, 0, sizeof(tws_m_service_buffer));
    spp_create_sdp_record(tws_m_service_buffer, 0x10001, RFCOMM_SERVER_CHANNEL, "SPP Streamer");
    sdp_register_service(tws_m_service_buffer);
*/
    gap_set_class_of_device(TEST_COD);
/*
    gap_ssp_set_io_capability(IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
    gap_ssp_set_auto_accept(0);
    gap_set_bondable_mode(0);
    gap_ssp_set_authentication_requirement(0);
*/
    gap_set_local_name("TWS M 00:00:00:00:00:00");

    gap_discoverable_control(1);
    gap_connectable_control(1);

    bt_manager_enable();

    message_loop(g_rcvr);

    return 0;
}
/* LISTING_END */
/* EXAMPLE_END */
