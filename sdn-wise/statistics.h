#ifndef STATISTICS_H_
#define STATISTICS_H_

typedef struct statistics {
	uint16_t packets_uc_received_total;
 	uint16_t packets_uc_received_as_dst;
 	uint16_t packets_bc_received;
	uint16_t packets_uc_sent_total;
	uint16_t packets_uc_sent_as_src;
	uint16_t packets_bc_sent;
	uint16_t packets_uc_retransmit;
	uint16_t hop_sum;
	float avg_hop_count;
} statistics_t;

statistics_t stat;
#endif
