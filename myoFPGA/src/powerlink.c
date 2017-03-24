#include "powerlink.h"

#ifdef __cplusplus
    extern "C"
    {
#endif

//------------------------------------------------------------------------------
/**
\brief  Initialize the openPOWERLINK stack

The function initializes the openPOWERLINK stack.

\param  cycleLen_p              Length of POWERLINK cycle.
\param  macAddr_p               MAC address to use for POWERLINK interface.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError initPowerlink(UINT32 cycleLen_p, const BYTE *macAddr_p, UINT32 nodeId_p) {
        tOplkError ret = kErrorOk;
        static tOplkApiInitParam initParam;
        static char devName[128];

        printf("Initializing openPOWERLINK stack...\n");

#if defined(CONFIG_USE_PCAP)
        selectPcapDevice(devName);
#endif

        memset(&initParam, 0, sizeof(initParam));
        initParam.sizeOfInitParam = sizeof(initParam);

        // pass selected device name to Edrv
        initParam.hwParam.pDevName = devName;
        initParam.nodeId = nodeId_p;
        initParam.ipAddress = (0xFFFFFF00 & IP_ADDR) | initParam.nodeId;

        /* write 00:00:00:00:00:00 to MAC address, so that the driver uses the real hardware address */
        memcpy(initParam.aMacAddress, macAddr_p, sizeof(initParam.aMacAddress));

        initParam.fAsyncOnly = FALSE;
        initParam.featureFlags = UINT_MAX;
        initParam.cycleLen = cycleLen_p;       // required for error detection
        initParam.isochrTxMaxPayload = 36;               // const
        initParam.isochrRxMaxPayload = 36;               // const
        initParam.presMaxLatency = 50000;            // const; only required for IdentRes
        initParam.preqActPayloadLimit = 36;               // required for initialisation (+28 bytes)
        initParam.presActPayloadLimit = 36;               // required for initialisation of Pres frame (+28 bytes)
        initParam.asndMaxLatency = 150000;           // const; only required for IdentRes
        initParam.multiplCylceCnt = 0;                // required for error detection
        initParam.asyncMtu = 1500;             // required to set up max frame size
        initParam.prescaler = 2;                // required for sync
        initParam.lossOfFrameTolerance = 500000;
        initParam.asyncSlotTimeout = 3000000;
        initParam.waitSocPreq = 1000;
        initParam.deviceType = UINT_MAX;               // NMT_DeviceType_U32
        initParam.vendorId = UINT_MAX;               // NMT_IdentityObject_REC.VendorId_U32
        initParam.productCode = UINT_MAX;               // NMT_IdentityObject_REC.ProductCode_U32
        initParam.revisionNumber = UINT_MAX;               // NMT_IdentityObject_REC.RevisionNo_U32
        initParam.serialNumber = UINT_MAX;               // NMT_IdentityObject_REC.SerialNo_U32
        initParam.applicationSwDate = 0;
        initParam.applicationSwTime = 0;
        initParam.subnetMask = SUBNET_MASK;
        initParam.defaultGateway = DEFAULT_GATEWAY;
        sprintf((char *) initParam.sHostname, "%02x-%08x", initParam.nodeId, initParam.vendorId);
        initParam.syncNodeId = C_ADR_SYNC_ON_SOA;
        initParam.fSyncOnPrcNode = FALSE;

        // set callback functions
        initParam.pfnCbEvent = processEvents;

#if defined(CONFIG_KERNELSTACK_DIRECTLINK)
        initParam.pfnCbSync = processSync;
#else
        initParam.pfnCbSync = NULL;
#endif

        // initialize POWERLINK stack
        ret = oplk_init(&initParam);
        if (ret != kErrorOk) {
            fprintf(stderr, "oplk_init() failed with \"%s\" (0x%04x)\n", debugstr_getRetValStr(ret), ret);
            return ret;
        }

        return kErrorOk;
    }

//------------------------------------------------------------------------------
/**
\brief  Main loop of demo application

This function implements the main loop of the demo application.
- It creates the sync thread which is responsible for the synchronous data
  application.
- It sends a NMT command to start the stack
- It loops and reacts on commands from the command line.
*/
//------------------------------------------------------------------------------
void loopMain(void) {
        tOplkError ret = kErrorOk;
        char cKey = 0;
        BOOL fExit = FALSE;

#if !defined(CONFIG_KERNELSTACK_DIRECTLINK)

#if defined(CONFIG_USE_SYNCTHREAD)
        system_startSyncThread(processSync);
#endif

#endif

        // start processing
        ret = oplk_execNmtCommand(kNmtEventSwReset);
        if (ret != kErrorOk) {
            return;
        }

        printf("start POWERLINK Stack... ok\n");
        printf("Digital I/O interface with openPOWERLINK is ready!\n");
        printf("\n-------------------------------\n");
        printf("Press Esc to leave the programm\n");
        printf("Press r to reset the node\n");
        printf("Press i to increase digital input\n");
        printf("Press d to decrease digital input\n");
        printf("Press p to print digital outputs\n");
        printf("-------------------------------\n\n");

        setupInputs();

        // wait for key hit
        while (!fExit) {
            if (console_kbhit()) {
                cKey = (BYTE) console_getch();

                switch (cKey) {
                    case 'r':
                        ret = oplk_execNmtCommand(kNmtEventSwReset);
                        if (ret != kErrorOk) {
                            fExit = TRUE;
                        }
                        break;

                    case 'i':
                        increaseInputs();
                        break;

                    case 'd':
                        decreaseInputs();
                        break;

                    case 'p':
                        printOutputs();
                        break;

                    case 0x1B:
                        fExit = TRUE;
                        break;

                    default:
                        break;
                }
            }

            if (system_getTermSignalState() == TRUE) {
                fExit = TRUE;
                printf("Received termination signal, exiting...\n");
            }

            if (oplk_checkKernelStack() == FALSE) {
                fExit = TRUE;
                fprintf(stderr, "Kernel stack has gone! Exiting...\n");
            }

#if defined(CONFIG_USE_SYNCTHREAD) || defined(CONFIG_KERNELSTACK_DIRECTLINK)
            system_msleep(100);
#else
            processSync();
#endif
        }

#if (TARGET_SYSTEM == _WIN32_)
        printf("Press Enter to quit!\n");
        console_getch();
#endif

        return;
    }

//------------------------------------------------------------------------------
/**
\brief  Shutdown the demo application

The function shuts down the demo application.
*/
//------------------------------------------------------------------------------
void shutdownPowerlink(void) {
        UINT i;

        fGsOff_l = FALSE;

#if !defined(CONFIG_KERNELSTACK_DIRECTLINK) && defined(CONFIG_USE_SYNCTHREAD)
        system_stopSyncThread();
        system_msleep(100);
#endif

        // halt the NMT state machine so the processing of POWERLINK frames stops
        oplk_execNmtCommand(kNmtEventSwitchOff);

        // small loop to implement timeout waiting for thread to terminate
        for (i = 0; i < 1000; i++) {
            if (fGsOff_l)
                break;
        }

        printf("Stack is in state off ... Shutdown\n");
        oplk_shutdown();
    }

#ifdef __cplusplus
}
#endif