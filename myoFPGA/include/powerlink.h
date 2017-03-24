#pragma once

#ifdef __cplusplus
    extern "C"
    {
#endif

#include <limits.h>
#include <oplk/oplk.h>
#include <oplk/debugstr.h>
#include <system/system.h>
#include <getopt/getopt.h>
#include <console/console.h>
#include <oplk/errordefs.h>

#if defined(CONFIG_USE_PCAP)

#include <pcap/pcap-console.h>

#endif

#include "app.h"
#include "event.h"


#define CYCLE_LEN         50000
#define NODEID            1                   // could be changed by command param
#define IP_ADDR           0xc0a86401          // 192.168.100.1
#define DEFAULT_GATEWAY   0xC0A864FE          // 192.168.100.C_ADR_RT1_DEF_NODE_ID
#define SUBNET_MASK       0xFFFFFF00          // 255.255.255.0

typedef struct {
    UINT32 nodeId;
    char *pLogFile;
} tOptions;

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------
const BYTE aMacAddr_l[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static BOOL fGsOff_l;

tOplkError initPowerlink(UINT32 cycleLen_p, const BYTE* macAddr_p, UINT32 nodeId_p);
void loopMain(void);
void shutdownPowerlink(void);

#ifdef __cplusplus
}
#endif