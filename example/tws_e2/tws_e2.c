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

#define __BTSTACK_FILE__ "tws_e2.c"

/*
 * tws_e2.c
 */

// *****************************************************************************
/* EXAMPLE_START(tws_e2): Snoop E2 Example
 */
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
//#include "snoop.h"

#define RFCOMM_SERVER_CHANNEL 1

//#define TEST_COD 0x1234
#define TEST_COD 0x5a020c

static btstack_packet_callback_registration_t hci_event_callback_registration;

// SPP
static uint8_t   e2_service_buffer[150];

static uint16_t  rfcomm_mtu;
static uint16_t  rfcomm_cid = 0;

static bd_addr_t e1_addr = {0x08, 0xb7, 0xe1, 0xde, 0x03, 0x9a};
static bd_addr_t e2_addr = {0x08, 0xb7, 0xe2, 0xde, 0xc5, 0x02};

uint32_t g_prev_timer_tc1, g_timer_tc1;
uint32_t g_prev_timer_tc2, g_timer_tc2;
/*
 * @section Packet Handler
 *
 * @text The packet handler of the combined example is just the combination of the individual packet handlers.
 */
static int g_cnt = 0;
uint32_t g_recv_cnt = 0;

static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel);
    UNUSED(size);

    bd_addr_t event_addr;
    uint8_t   rfcomm_channel_nr;

//    printf("packet_handler %d %d\n", packet_type, hci_event_packet_get_type(packet));

    switch (packet_type) {
        case HCI_EVENT_PACKET:
            switch (hci_event_packet_get_type(packet)) {

                case HCI_EVENT_COMMAND_COMPLETE:
                    //printf("HCI_EVENT_COMMAND_COMPLETE\n");
                    break;

                case HCI_EVENT_PIN_CODE_REQUEST:
                    // inform about pin code request
                    //printf("Pin code request - using '0000'\n");
                    hci_event_pin_code_request_get_bd_addr(packet, event_addr);
                    gap_pin_code_response(event_addr, "0000");
                    break;

                case HCI_EVENT_USER_CONFIRMATION_REQUEST:
                    // inform about user confirmation request
                    printf("SSP User Confirmation Request with numeric value '%06"PRIu32"'\n", little_endian_read_32(packet, 8));
                    printf("SSP User Confirmation Auto accept\n");
                    break;

                case HCI_EVENT_DISCONNECTION_COMPLETE:
                    printf("HCI_EVENT_DISCONNECTION_COMPLETE\n");
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
                    } else {
                        bd_addr_t rfcomm_bdaddr;
                        rfcomm_event_channel_opened_get_bd_addr(packet, rfcomm_bdaddr);
                        rfcomm_cid = rfcomm_event_channel_opened_get_rfcomm_cid(packet);
                        rfcomm_mtu = rfcomm_event_channel_opened_get_max_frame_size(packet);
                        printf("RFCOMM channel open succeeded. New RFCOMM Channel ID %u, max frame size %u\n", rfcomm_cid, rfcomm_mtu);

   g_prev_timer_tc1=*((volatile uint32_t *)0xa00e0294);
   g_prev_timer_tc2=*((volatile uint32_t *)0xa00e0298);

                        //gap_discoverable_control(0);
                        //gap_connectable_control(0);
//rfcomm_request_can_send_now_event(rfcomm_cid);
                    }
                    break;

                case RFCOMM_EVENT_CAN_SEND_NOW:
                    printf("RFCOMM_EVENT_CAN_SEND_NOW\n");
unsigned char test_data[100];
//rfcomm_send(rfcomm_cid, test_data, 100);
//rfcomm_request_can_send_now_event(rfcomm_cid);
                    break;

                case RFCOMM_EVENT_CHANNEL_CLOSED:
                    printf("RFCOMM channel closed\n");

                    // re-enable page/inquiry scan again
                    gap_discoverable_control(1);
                    gap_connectable_control(1);
                    break;

                case BTSTACK_EVENT_STATE:
                    printf("BTSTACK_EVENT_STATE: %d\n", btstack_event_state_get_state(packet));
                    if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) return;
                    printf("Waiting for connection from M2\n");//, bd_addr_to_str(e1_addr));
                    break;

                default:
                    break;
            }
            break;

        case RFCOMM_DATA_PACKET:
            {
//                uint32_t tmp = packet[3] << 24 | packet[2] << 16 | packet[1]<< 8 | packet[0];
                //printf("RCV: %u\n", tmp);
g_recv_cnt += size;
if (g_recv_cnt >= 80000)
{
   g_timer_tc1=*((volatile uint32_t *)0xa00e0294);
   g_timer_tc2=*((volatile uint32_t *)0xa00e0298);

   uint32_t t = g_timer_tc2 - g_prev_timer_tc2;
   printf("RCV: %u %u-100us %u\n", g_recv_cnt, t, (uint32_t)(g_recv_cnt * 10000 / t));
   g_prev_timer_tc1=g_timer_tc1;
   g_prev_timer_tc2=g_timer_tc2;
   g_recv_cnt = 0;  
}
            }
            break;
/*
        case SNOOP_EVENT_PACKET:
            {
                printf("SNOOP_EVENT_PACKET: %dn", packet[0]);
                switch (packet[0]) {
                    case SNOOP_EVENT_PCONN_OPEN:
                        {
                            if (packet[1] == 0) {
                                printf("SNOOP open succeeded.\n");
                            }
                        }
                        break;
                    default:
                        break;
                }
            }
            break;
*/
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

    printf("message_handler %d\n", id);

    if (id == MSG_EXIT_LOOP)
        return MESSAGE_EXIT_LOOP;

    if (id == MSG_BT_MANAGER_STATE_ON) {

//    snoop_init(HCI_SNOOP_ROLE_E2, e1_addr, e2_addr);
//    snoop_register(packet_handler);

    }
    return MESSAGE_HANDLED;
}
#if 0
void test_pin_cfg(uint32_t addr)
{
   uint32_t val;

   val = *((volatile uint32_t*)addr);
   val &= 0xFFFFFFF0;  val |= 3;
   *((volatile uint32_t*)addr) = val;
}

/* LISTING_START(MainConfiguration): Init L2CAP RFCOMM SDO SM ATT Server and start heartbeat timer */

void rf_debug_enable(void)
{
/* enable RF debug pin*/
uint32_t reg_val;

printf("RF Debug Pin Enable\n");

// release RFIP reset
reg_val = *((uint32_t *)0xa00e0180);
*((uint32_t *)0xa00e0180) = reg_val & (~(1<<23));

usleep(10000);
/* supply rfip vdda and vddpa */
// ... ...
*((uint8_t *)0xa0300000 + (0x10a9 << 2)) = 0xF8;


// debug pin
*((uint32_t *)0xa00d00d4) = 0x13;
*((uint32_t *)0xa00d00d8) = 0x13;
*((uint32_t *)0xa00d00dc) = 0x13;
*((uint32_t *)0xa00d00e0) = 0x13;
*((uint32_t *)0xa00d00e4) = 0x13;
*((uint32_t *)0xa00d00e8) = 0x13;
*((uint32_t *)0xa00d0024) = 0x13;
*((uint32_t *)0xa00d0028) = 0x13;
*((uint32_t *)0xa00d0070) = 0x13;
*((uint32_t *)0xa00d0074) = 0x13;
*((uint32_t *)0xa00d0078) = 0x13;

*((uint32_t *)0xa00d0194) = 2;

reg_val = *((uint32_t *)0xa00e0500);
reg_val &= (~(1<<20));
reg_val |= (1<<21);
reg_val |= (1<<23);
*((uint32_t *)0xa00e0500) = reg_val;
}
#endif

static uint8_t   e2_service_buffer[150];

int tws_e2_main(int argc, const char * argv[]);
int tws_e2_main(int argc, const char * argv[])
{
    UNUSED(argc);
    (void)argv;

#if 0
/* Enable CEVA DIAG pins */
uint32_t val;
val = *((volatile uint32_t*)0xa00d0194);
val &= 0xFFFFFFF0;  val |= 1;
*((volatile uint32_t*)0xa00d0194) = val;

test_pin_cfg(0xa00d00D4);
test_pin_cfg(0xa00d00D8);
test_pin_cfg(0xa00d00DC);
test_pin_cfg(0xa00d00E0);
test_pin_cfg(0xa00d00E4);
test_pin_cfg(0xa00d00E8);
test_pin_cfg(0xa00d0024);
test_pin_cfg(0xa00d0028);
test_pin_cfg(0xa00d0070);
test_pin_cfg(0xa00d0074);
test_pin_cfg(0xa00d0078);
test_pin_cfg(0xa00d00B0);
test_pin_cfg(0xa00d00B4);
test_pin_cfg(0xa00d00B8);
test_pin_cfg(0xa00d00BC);
test_pin_cfg(0xa00d00C0);

val = *((volatile uint32_t*)0xa00e0500);
val &= 0xFFFFFFF0;  val |= 7;
*((volatile uint32_t*)0xa00e0500) = val;

#endif

#if 1
    g_rcvr = message_init(message_handler, 0, 0);
    if (g_rcvr == NULL)
    {
      printf("--- message_init error, exit\n");
      return -1;
    }

    bt_handle = bt_manager_register(g_rcvr);
    bt_manager_open();

    // register for HCI events
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    hci_set_bd_addr(e2_addr);

    rfcomm_set_required_security_level(LEVEL_0);

    printf("rfcomm register service\n");
    rfcomm_register_service(packet_handler, RFCOMM_SERVER_CHANNEL, 0xffff);

/*
    printf("spp_create_sdp_record\n");
    memset(e2_service_buffer, 0, sizeof(e2_service_buffer));
    spp_create_sdp_record(e2_service_buffer, 0x10001, RFCOMM_SERVER_CHANNEL, "SPP Streamer");
    sdp_register_service(e2_service_buffer);
*/
    gap_set_class_of_device(TEST_COD);

    // init SDP
/*
    gap_ssp_set_io_capability(IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
    gap_ssp_set_auto_accept(0);
    gap_set_bondable_mode(0);
    gap_ssp_set_authentication_requirement(0);
*/
    gap_set_local_name("TWS X E2 00:00:00:00:00:00");

    gap_discoverable_control(1);
    gap_connectable_control(1);

    bt_manager_enable();

/* CEVA DBG PIN*/
    message_loop(g_rcvr);
#endif

//rf_debug_enable();

    return 0;
}
/* LISTING_END */
/* EXAMPLE_END */
