/*
 * Copyright (C) 2015 SDN-WISE
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
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * \file
 *         SDN-WISE Packet Handler.
 * \author
 *         Sebastiano Milardo <s.milardo@hotmail.it>
 */

/**
 * \addtogroup sdn-wise
 * @{
 */

#include <string.h>
#include <stdio.h>
#include "contiki.h"
#include "dev/watchdog.h"
#include "packet-handler.h"
#include "address.h"
#include "packet-buffer.h"
#include "packet-creator.h"
#include "neighbor-table.h"
#include "flowtable.h"
#include "node-conf.h"
#include "sdn-wise.h"

#include "node-id.h"

#include "project-conf.h"

#include "statistics.h"

typedef enum conf_id{
  RESET,
  MY_NET,
  MY_ADDRESS,
  PACKET_TTL,
  RSSI_MIN,
  BEACON_PERIOD,
  REPORT_PERIOD,
  RESET_PERIOD,
  RULE_TTL,
  ADD_ALIAS,
  REM_ALIAS,
  GET_ALIAS,
  ADD_RULE,
  REM_RULE,
  GET_RULE,
  ADD_FUNCTION,
  REM_FUNCTION,
  GET_FUNCTION
} conf_id_t;

const uint8_t conf_size[RULE_TTL+1] =
{
  0,
  sizeof(conf.my_net),
  sizeof(conf.my_address),
  sizeof(conf.packet_ttl),
  sizeof(conf.rssi_min),
  sizeof(conf.beacon_period),
  sizeof(conf.report_period),
  sizeof(conf.reset_period),
  sizeof(conf.rule_ttl)
};

const void* conf_ptr[RULE_TTL+1] =
{
  NULL,
  &conf.my_net,
  &conf.my_address,
  &conf.packet_ttl,
  &conf.rssi_min,
  &conf.beacon_period,
  &conf.report_period,
  &conf.reset_period,
  &conf.rule_ttl,
};

#define CNF_READ 0
#define CNF_WRITE 1

#ifndef SDN_WISE_DEBUG
#define SDN_WISE_DEBUG 1
#endif
#if SDN_WISE_DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...) 
#endif
/*----------------------------------------------------------------------------*/
  static void handle_beacon(packet_t*);
  static void handle_data(packet_t*);
  static void handle_report(packet_t*);
  static void handle_response(packet_t*);
  static void handle_open_path(packet_t*);
  static void handle_config(packet_t*);
  static void handle_web_req(packet_t*);
/*----------------------------------------------------------------------------*/
  void
  handle_packet(packet_t* p)
  {
    if (p->info.rssi >= conf.rssi_min && p->header.net == conf.my_net){
      if (p->header.typ == BEACON){
        PRINTF("[PHD]: Beacon\n");
        handle_beacon(p);
      } else {
        if (is_my_address(&(p->header.nxh))){
          rx_count_inc(&(p->info.sender));
          
          switch (p->header.typ){
            case DATA:
            PRINTF("[PHD]: Data\n");
            handle_data(p);
            break;

            case RESPONSE:
            PRINTF("[PHD]: Response\n");
            handle_response(p);
            break;

            case OPEN_PATH:
            PRINTF("[PHD]: Open Path\n");
            handle_open_path(p);
            break;

            case CONFIG:
            PRINTF("[PHD]: Config\n");
            handle_config(p);
            break;

            case WEB_REQ:
            PRINTF("[PHD]: Web req\n");
            handle_web_req(p);
            break;

            default:
            PRINTF("[PHD]: Request/Report\n");
            handle_report(p);
            break;
          }
        } else {
          printf("dropped packet\n");
        }
      }
    } else {
      packet_deallocate(p);
    }
  }
/*----------------------------------------------------------------------------*/
  void
  handle_beacon(packet_t* p)
  {
    add_neighbor(&(p->header.src),p->info.rssi);

    if(get_payload_at(p, BEACON_TYPE_INDEX) != BEACON_T_TREE) {
      packet_deallocate(p);
      return;
    }

#if !SINK
    uint8_t tree_version = get_payload_at(p, BEACON_TREE_VERSION_INDEX);
    uint8_t hops_from_sink = get_payload_at(p, BEACON_DEPTH_INDEX);
    if(conf.tree_version > tree_version + 2) {
      conf.tree_version = tree_version;
      packet_deallocate(p);
      return;
    }
    if(tree_version > conf.tree_version) {
      // is new tree version
      conf.tree_version = tree_version;
      conf.hops_from_sink = hops_from_sink + 1;
      conf.nxh_vs_sink = p->header.src;
      conf.distance_from_sink = p->info.rssi;
      conf.sink_address = p->header.nxh;
      send_updated_tree_message(); // rebroadcast
    } else if(tree_version == conf.tree_version) {
      // is current tree
      if(hops_from_sink + 1 < conf.hops_from_sink) {
        // is path better parent than current
        conf.hops_from_sink = hops_from_sink + 1;
        conf.nxh_vs_sink = p->header.src;
        conf.distance_from_sink = p->info.rssi;
        conf.sink_address = p->header.nxh;
        send_updated_tree_message();
      }
    }
#endif
    packet_deallocate(p);
  }
/*----------------------------------------------------------------------------*/
void send_updated_tree_message() {
  packet_t* rebroadcast = create_packet_empty();
  if (rebroadcast != NULL){
    rebroadcast->header.net = conf.my_net;
    set_broadcast_address(&(rebroadcast->header.dst));
    rebroadcast->header.src = conf.my_address;
    rebroadcast->header.typ = BEACON;
    rebroadcast->header.nxh = conf.sink_address;
    set_payload_at(rebroadcast, BEACON_HOPS_INDEX, conf.hops_from_sink);
    set_payload_at(rebroadcast, BEACON_BATT_INDEX, 0);
    set_payload_at(rebroadcast, BEACON_TREE_VERSION_INDEX, conf.tree_version);
    set_payload_at(rebroadcast, BEACON_DEPTH_INDEX, conf.hops_from_sink);
    set_payload_at(rebroadcast, BEACON_TYPE_INDEX, BEACON_T_TREE);
    printf("TREE: [id: %u, depth: %u, next_hop: %u.%u]\n", conf.tree_version, conf.hops_from_sink, conf.nxh_vs_sink.u8[0], conf.nxh_vs_sink.u8[1]);
    rf_broadcast_send(rebroadcast);
  }
}
/*----------------------------------------------------------------------------*/
  void
  handle_data(packet_t* p)
  {
    static uint8_t hops;
    static uint8_t message_id;
    hops = get_payload_at(p,0);
    
    message_id = get_payload_at(p,1);

    //set_payload_at(p,0,hops + 1);
    stat.packets_uc_received_total++;
    printf("DATA: [node: %u, message_id: %u.%u, src: %u, dst: %u, ttl: %u]\n", 
    node_id, p->header.src.u8[1], message_id, p->header.src.u8[1], p->header.dst.u8[1], S_TTL- get_payload_at(p,0));
    if (is_my_address(&(p->header.dst)))
    {
      PRINTF("[PHD]: Consuming Packet\n");
      printf("RXU: [node: %u, message_id: %u.%u, src: %u, dst: %u, ttl: %u]\n", node_id, p->header.src.u8[1], message_id, p->header.src.u8[1], p->header.dst.u8[1], S_TTL- get_payload_at(p,0));
      stat.packets_uc_received_as_dst++;
      stat.hop_sum = stat.hop_sum + get_payload_at(p,0);
      stat.avg_hop_count = stat.hop_sum/stat.packets_uc_received_as_dst;
      packet_deallocate(p);
    } else {
      stat.packets_uc_sent_total++;
      printf("RXU: [node: %u, message_id: %u.%u, src: %u, dst: %u, ttl: %u]\n", node_id, p->header.src.u8[1], message_id, p->header.src.u8[1], p->header.dst.u8[1], S_TTL- get_payload_at(p,0));
      match_packet(p);
    }
  }
/*----------------------------------------------------------------------------*/
  void 
  handle_web_req(packet_t *p)
  {
    static uint8_t message_id;
    message_id = get_payload_at(p, 0);
    printf("WEB:[");
    uint16_t i=0;
    for (i=0; i < (p->header.len - PLD_INDEX); ++i){
      printf("%d ",get_payload_at(p,i));
    }
    printf("]\n");

    if (is_my_address(&(p->header.dst)))
    {
      PRINTF("[PHD]: Consuming WEB Packet\n");    
      printf("WEB: [node: %u, message_id: %u.%u, src: %u, dst: %u, ttl: %u]\n",
            node_id, p->header.src.u8[1], message_id,
            p->header.src.u8[1], p->header.dst.u8[1],
            S_TTL - get_payload_at(p, 0));
      
      p->header.dst = p->header.src;
      p->header.src = conf.my_address;
      p->header.nxh = conf.nxh_vs_sink;
      PRINTF("p-len:%u\n", p->header.len);
      set_payload_at(p, 1, 5 );
      set_payload_at(p, 2, 5 );
      
      match_packet(p);

      //packet_deallocate(p);

    }
    else
    {
      printf("FWD: [node: %u, message_id: %u.%u, src: %u, dst: %u, ttl: %u]\n", node_id, p->header.src.u8[1], message_id, p->header.src.u8[1], p->header.dst.u8[1], S_TTL - get_payload_at(p, 0));
      match_packet(p);
    }
}
/*----------------------------------------------------------------------------*/
  void
  handle_report(packet_t* p)
  {
#if SINK
    print_packet_uart(p);
#else

    p->header.nxh = conf.nxh_vs_sink;
    rf_unicast_send(p);
#endif
  }
/*----------------------------------------------------------------------------*/
  void
  handle_response(packet_t* p)
  {
    if (is_my_address(&(p->header.dst)))
    {
      entry_t* e = get_entry_from_array(p->payload, p->header.len - PLD_INDEX);
      if (e != NULL)
      {
        add_entry(e);
      }
      packet_deallocate(p);
    } else {
      match_packet(p);
    }
  }
/*----------------------------------------------------------------------------*/
  void
  handle_open_path(packet_t* p)
  {
    int i;
    uint8_t n_windows = get_payload_at(p,OPEN_PATH_WINDOWS_INDEX); //2
    uint8_t start = n_windows*WINDOW_SIZE + 1; //2*5+1 = 11
    uint8_t path_len = (p->header.len - (start + PLD_INDEX))/ADDRESS_LENGTH; // (33 - (11 + (8+2)))/2 = 6
    int my_index = -1;
    uint8_t my_position = 0;
    uint8_t end = p->header.len - PLD_INDEX; // 33 - 10 = 23

    for (i = start; i < end; i += ADDRESS_LENGTH)
    {
      address_t tmp = get_address_from_array(&(p->payload[i]));
      if (is_my_address(&tmp))
      {
        my_index = i;
        break;
      }
      my_position++;
    }

    if (my_index == -1){
    	printf("[PHD]: Nothing to learn, matching...\n");
    	match_packet(p);
    } else {
      if (my_position > 0)
      {
        uint8_t prev = my_index - ADDRESS_LENGTH;
        uint8_t first = start;
        entry_t* e = create_entry();

        window_t* w = create_window();
        w->operation = EQUAL;
        w->size = SIZE_2;
        w->lhs = DST_INDEX;
        w->lhs_location = PACKET;
        w->rhs = MERGE_BYTES(p->payload[first], p->payload[first+1]);
        w->rhs_location = CONST;

        add_window(e,w);

        for (i = 0; i<n_windows; ++i)
        {
          add_window(e, get_window_from_array(&(p->payload[i*WINDOW_SIZE + 1])));
        }

        action_t* a = create_action(FORWARD_U, &(p->payload[prev]), ADDRESS_LENGTH);
        add_action(e,a);

        PRINTF("[PHD]: ");
        print_entry(e);
        PRINTF("\n");

        add_entry(e);
      }

      if (my_position < path_len-1)
      {
        uint8_t next = my_index + ADDRESS_LENGTH;
        uint8_t last = end - ADDRESS_LENGTH;
        entry_t* e = create_entry();

        window_t* w = create_window();
        w->operation = EQUAL;
        w->size = SIZE_2;
        w->lhs = DST_INDEX;
        w->lhs_location = PACKET;
        w->rhs = MERGE_BYTES(p->payload[last], p->payload[last+1]);
        w->rhs_location = CONST;

        add_window(e,w);

        for (i = 0; i<n_windows; ++i)
        {
          add_window(e, get_window_from_array(&(p->payload[i*WINDOW_SIZE + 1])));
        }

        action_t* a = create_action(FORWARD_U, &(p->payload[next]), ADDRESS_LENGTH);
        add_action(e,a);
        add_entry(e);

        address_t next_address = get_address_from_array(&(p->payload[next]));
        p->header.nxh = next_address;
        p->header.dst = next_address;
        rf_unicast_send(p);
      }

      if (my_position == path_len-1){
        packet_deallocate(p);
      }
  	}
  }
/*----------------------------------------------------------------------------*/
  void
  handle_config(packet_t* p)
  {
    if (is_my_address(&(p->header.dst)))
    {
#if SINK
      if (!is_my_address(&(p->header.src))){
        print_packet_uart(p);
      } else {
#endif
      uint8_t i = 0;
      uint8_t id = p->payload[i] & 127;
      if ((p->payload[i] & 128) == CNF_READ)
      {
        //READ
        switch (id)
        {
          // TODO
          case RESET:
          case GET_ALIAS:
          case GET_FUNCTION:
          break;

      	  case GET_RULE:
      	    p->header.len += get_array_from_entry_id(&p->payload[i+2],p->payload[i+1]);
            break;

          case MY_NET:
          case MY_ADDRESS:
          case PACKET_TTL:
          case RSSI_MIN:
          case BEACON_PERIOD:
          case REPORT_PERIOD:
          case RULE_TTL:
            // TODO check payload size
            if (conf_size[id] == 1){
              memcpy(&(p->payload[i+1]), conf_ptr[id], conf_size[id]);
            } else if (conf_size[id] == 2) {
              uint16_t value = *((uint16_t*)conf_ptr[id]);
              p->payload[i+1] = value >> 8;
              p->payload[i+2] = value & 0xFF;
            }
  	        p->header.len += conf_size[id];
            break;


          default:
          break;
        }
        swap_addresses(&(p->header.src),&(p->header.dst));
#if !SINK
        match_packet(p);
#else
	print_packet_uart(p);
#endif
      } else {
        //WRITE
        switch (id)
        {
          // TODO
          case ADD_ALIAS:
          case REM_ALIAS:
          case ADD_RULE:
          case REM_RULE:
          case ADD_FUNCTION:
          case REM_FUNCTION:
          break;

          case RESET:
          watchdog_reboot();
          break;

          case MY_NET:
          case MY_ADDRESS:
          case PACKET_TTL:
          case RSSI_MIN:
          case BEACON_PERIOD:
          case REPORT_PERIOD:
          case RESET_PERIOD:
          case RULE_TTL:
          if (conf_size[id] == 1){
            memcpy((uint8_t*)conf_ptr[id], &(p->payload[i+1]), conf_size[id]);
          } else if (conf_size[id] == 2) {
            uint16_t h = p->payload[i+1] << 8;
            uint16_t l = p->payload[i+2];
            *((uint16_t*)conf_ptr[id]) = h + l;
          }
          break;

          default:
          break;
        }
        packet_deallocate(p);
      }
#if SINK
    }
#endif
    }
    else {
      match_packet(p);
    }
  }
/*----------------------------------------------------------------------------*/
  void
  test_handle_open_path(void)
  {
    uint8_t array[19] = {
      1, 19, 0, 1, 0, 2, 5, 100, 0, 1, 0, 0, 1, 0, 2, 0, 3, 0, 4,
    };

    packet_t* p = get_packet_from_array(array);
    handle_open_path(p);
  }
/*----------------------------------------------------------------------------*/
/** @} */
