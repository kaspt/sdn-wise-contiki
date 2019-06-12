/*
 *  project-conf.h
 *
 *  Created on: 27 jan 2016
 *      Author: Sebastiano Milardo
 */

#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

/*************************************************************************/
#define CFS_ENABLED   0
#define ELF_ENABLED   0
//#define SINK 0
#define SDN_WISE_DEBUG 0

#define MULTI 0

#define SRC 3
#define DST 4

#define S_BEACON_PERIOD 5
#define S_REPORT_PERIOD 10
#define S_RESET_PERIOD  900
#define S_TTL           100

#define NETWORK_SIZE 40

#define MESSAGE_INTERVAL 10
#define INITIAL_MESSAGE_DEALAY 20 // wait 5 minutes before sending the first message
#define STATISTICS_PRINT_INTERVALL 10
#define PURGE_FLOWTABLE_INTERVAL 60

/*************************************************************************/
#endif // PROJECT_CONF_H_
