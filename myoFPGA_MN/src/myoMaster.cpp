#include "myoMaster.hpp"

static BOOL* pfGsOff_l;

PI_IN* MyoMaster::pProcessImageIn_l;
const PI_OUT* MyoMaster::pProcessImageOut_l;
UDPSocket *MyoMaster::motorConfigSocket;
UDPSocket *MyoMaster::motorStatusSocket;
bool MyoMaster::updateControllerConfig = false;
int32_t MyoMaster::setPoints[16];
control_Parameters_t MyoMaster::MotorConfig;
bool MyoMaster::fExit = false;

MyoMaster::MyoMaster(int argc, char *argv[]) {
    tOplkError ret = kErrorOk;
    tOptions opts;

    bool powerlink_initialized = true;

    if (getOptions(argc, argv, &opts) < 0)
        powerlink_initialized = false;

    if (system_init() != 0) {
        fprintf(stderr, "Error initializing system!");
        powerlink_initialized = false;
    }

    eventlog_init(opts.logFormat,
                  opts.logLevel,
                  opts.logCategory,
                  (tEventlogOutputCb) console_printlogadd);

    pfGsOff_l = new unsigned char;
    *pfGsOff_l = true;

    printf("----------------------------------------------------\n");
    printf("myoFPGA console MN \n");
    printf("Using openPOWERLINK stack: %s\n", oplk_getVersionString());
    printf("----------------------------------------------------\n");

    eventlog_printMessage(kEventlogLevelInfo,
                          kEventlogCategoryGeneric,
                          "myoMaster: Stack version:%s Stack configuration:0x%08X",
                          oplk_getVersionString(),
                          oplk_getStackConfiguration());

    eventlog_printMessage(kEventlogLevelInfo,
                          kEventlogCategoryGeneric,
                          "Using CDC file: %s",
                          opts.cdcFile);

    const BYTE aMacAddr_l[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    ret = initPowerlink(CYCLE_LEN, opts.cdcFile, opts.devName, aMacAddr_l);

    if (ret != kErrorOk)
        powerlink_initialized = false;

    ret = initProcessImage();
    if (ret != kErrorOk)
        powerlink_initialized = false;

    motorConfigSocket = new UDPSocket("192.168.0.104", 8000);
    motorStatusSocket = new UDPSocket("192.168.0.104", 8001);

    if(powerlink_initialized) {
#ifdef RUN_IN_THREAD
        powerLinkThread = new std::thread(&MyoMaster::mainLoop, this);
        powerLinkThread->detach();
#else
    mainLoop();
#endif
    }
}

MyoMaster::~MyoMaster() {
    fExit = true;
#ifdef RUN_IN_THREAD
    if(powerLinkThread->joinable())
        powerLinkThread->join();
#endif
    shutdownPowerlink();
    system_exit();
}

void MyoMaster::mainLoop() {
    tOplkError ret = kErrorOk;
    char cKey = 0;

#if !defined(CONFIG_KERNELSTACK_DIRECTLINK)

#if defined(CONFIG_USE_SYNCTHREAD)
    system_startSyncThread(processSync);
#endif

#endif

    // start stack processing by sending a NMT reset command
    ret = oplk_execNmtCommand(kNmtEventSwReset);
    if (ret != kErrorOk) {
        fprintf(stderr,
                "oplk_execNmtCommand() failed with \"%s\" (0x%04x)\n",
                debugstr_getRetValStr(ret),
                ret);
        return;
    }

    printf("\n-------------------------------\n");
    printf("Press Esc to leave the program\n");
    printf("Press r to reset the node\n");
    printf("-------------------------------\n\n");

    while (!fExit) {
        if (console_kbhit()) {
            cKey = (char) console_getch();
            switch (cKey) {
                case 'r':
                    ret = oplk_execNmtCommand(kNmtEventSwReset);
                    if (ret != kErrorOk) {
                        fprintf(stderr,
                                "oplk_execNmtCommand() failed with \"%s\" (0x%04x)\n",
                                debugstr_getRetValStr(ret),
                                ret);
                        fExit = true;
                    }
                    break;

                case 'c':
                    ret = oplk_execNmtCommand(kNmtEventNmtCycleError);
                    if (ret != kErrorOk) {
                        fprintf(stderr,
                                "oplk_execNmtCommand() failed with \"%s\" (0x%04x)\n",
                                debugstr_getRetValStr(ret),
                                ret);
                        fExit = true;
                    }
                    break;

                case 0x1B:
                    fExit = true;
                    break;

                default:
                    break;
            }
        }

        if (system_getTermSignalState() == TRUE) {
            fExit = true;
            printf("Received termination signal, exiting...\n");
            eventlog_printMessage(kEventlogLevelInfo,
                                  kEventlogCategoryControl,
                                  "Received termination signal, exiting...");
        }

        if (oplk_checkKernelStack() == FALSE) {
            fExit = true;
            fprintf(stderr, "Kernel stack has gone! Exiting...\n");
            eventlog_printMessage(kEventlogLevelFatal,
                                  kEventlogCategoryControl,
                                  "Kernel stack has gone! Exiting...");
        }

#if (defined(CONFIG_USE_SYNCTHREAD) || \
     defined(CONFIG_KERNELSTACK_DIRECTLINK))
        system_msleep(100);
#else
        processSync();
#endif
    }
}

void MyoMaster::changeControl(int motor, int mode){
    MotorConfig.control_mode[motor] = mode;
}

void MyoMaster::changeControl(int mode){
    fill_n(MotorConfig.control_mode,16,mode);
}

void MyoMaster::changeSetPoint(int motor, int setPoint){
    setPoints[motor] = setPoint;
}

void MyoMaster::changeSetPoint(int setPoint){
    fill_n(setPoints,16,setPoint);
}

void MyoMaster::sendControllerConfig(){
    updateControllerConfig = true;
}

tOplkError MyoMaster::initProcessImage() {
    tOplkError ret = kErrorOk;
    UINT errorIndex = 0;

    printf("Initializing process image...\n");
    printf("Size of process image: Input = %lu Output = %lu\n",
           (ULONG) sizeof(PI_IN),
           (ULONG) sizeof(PI_OUT));
    eventlog_printMessage(kEventlogLevelInfo,
                          kEventlogCategoryGeneric,
                          "Allocating process image: Input:%lu Output:%lu",
                          (ULONG) sizeof(PI_IN),
                          (ULONG) sizeof(PI_OUT));

    ret = oplk_allocProcessImage(sizeof(PI_IN), sizeof(PI_OUT));
    if (ret != kErrorOk)
        return ret;

    pProcessImageIn_l = (PI_IN *) oplk_getProcessImageIn();
    pProcessImageOut_l = (const PI_OUT *) oplk_getProcessImageOut();

    errorIndex = obdpi_setupProcessImage();
    if (errorIndex != 0) {
        eventlog_printMessage(kEventlogLevelFatal,
                              kEventlogCategoryControl,
                              "Setup process image failed at index 0x%04x\n",
                              errorIndex);
        ret = kErrorApiPINotAllocated;
    }

    return ret;
}

int MyoMaster::getOptions(int argc_p, char *const argv_p[], tOptions *pOpts_p) {
    int opt;

    /* setup default parameters */
    strncpy(pOpts_p->cdcFile, "mnobd.cdc", 256);
    strncpy(pOpts_p->devName, "\0", 128);
    pOpts_p->pLogFile = NULL;
    pOpts_p->logFormat = kEventlogFormatReadable;
    pOpts_p->logCategory = 0xffffffff;
    pOpts_p->logLevel = 0xffffffff;

    /* get command line parameters */
    while ((opt = getopt(argc_p, argv_p, "c:l:pv:t:d:")) != -1) {
        switch (opt) {
            case 'c':
                strncpy(pOpts_p->cdcFile, optarg, 256);
                break;

            case 'd':
                strncpy(pOpts_p->devName, optarg, 128);
                break;

            case 'p':
                pOpts_p->logFormat = kEventlogFormatParsable;
                break;

            case 'v':
                pOpts_p->logLevel = strtoul(optarg, NULL, 16);
                break;

            case 't':
                pOpts_p->logCategory = strtoul(optarg, NULL, 16);
                break;

            default: /* '?' */
#if defined(CONFIG_USE_PCAP)
                printf("Usage: %s [-c CDC-FILE] [-d DEV_NAME] [-v LOGLEVEL] [-t LOGCATEGORY] [-p]\n", argv_p[0]);
                printf(" -d DEV_NAME: Ethernet device name to use e.g. eth1\n");
#else
                printf("Usage: %s [-c CDC-FILE] [-v LOGLEVEL] [-t LOGCATEGORY] [-p]\n", argv_p[0]);
#endif
                printf(" -p: Use parsable log format\n");
                printf("              If option is skipped the program prompts for the interface.\n");
                printf(" -v LOGLEVEL: A bit mask with log levels to be printed in the event logger\n");
                printf(" -t LOGCATEGORY: A bit mask with log categories to be printed in the event logger\n");
                return -1;
        }
    }

    return 0;
}

tOplkError MyoMaster::initPowerlink(UINT32 cycleLen_p,
                                    const char* cdcFileName_p,
                                    const char* devName_p,
                                    const UINT8* macAddr_p){
    tOplkError          ret = kErrorOk;
    tOplkApiInitParam   initParam;
    static char         devName[128];

    printf("Initializing openPOWERLINK stack...\n");
    eventlog_printMessage(kEventlogLevelInfo,
                          kEventlogCategoryControl,
                          "Initializing openPOWERLINK stack");

#if defined(CONFIG_USE_PCAP)
    eventlog_printMessage(kEventlogLevelInfo,
                          kEventlogCategoryGeneric,
                          "Using libpcap for network access");
    if (devName_p[0] == '\0')
    {
        if (selectPcapDevice(devName) < 0)
            return kErrorIllegalInstance;
    }
    else
        strncpy(devName, devName_p, 128);
#else
    UNUSED_PARAMETER(devName_p);
#endif

    memset(&initParam, 0, sizeof(initParam));
    initParam.sizeOfInitParam = sizeof(initParam);

    // pass selected device name to Edrv
    initParam.hwParam.pDevName = devName;
    initParam.nodeId = NODEID;
    initParam.ipAddress = (0xFFFFFF00 & IP_ADDR) | initParam.nodeId;

    /* write 00:00:00:00:00:00 to MAC address, so that the driver uses the real hardware address */
    memcpy(initParam.aMacAddress, macAddr_p, sizeof(initParam.aMacAddress));

    initParam.fAsyncOnly              = FALSE;
    initParam.featureFlags            = UINT_MAX;
    initParam.cycleLen                = cycleLen_p;       // required for error detection
    initParam.isochrTxMaxPayload      = 1490;              // const
    initParam.isochrRxMaxPayload      = 1490;             // const
    initParam.presMaxLatency          = 50000;            // const; only required for IdentRes
    initParam.preqActPayloadLimit     = 1400;               // required for initialisation (+28 bytes)
    initParam.presActPayloadLimit     = 1400;               // required for initialisation of Pres frame (+28 bytes)
    initParam.asndMaxLatency          = 200000;           // const; only required for IdentRes
    initParam.multiplCylceCnt         = 0;                // required for error detection
    initParam.asyncMtu                = 300;             // required to set up max frame size
    initParam.prescaler               = 2;                // required for sync
    initParam.lossOfFrameTolerance    = 500000;
    initParam.asyncSlotTimeout        = 3000000;
    initParam.waitSocPreq             = 1000;
    initParam.deviceType              = UINT_MAX;         // NMT_DeviceType_U32
    initParam.vendorId                = UINT_MAX;         // NMT_IdentityObject_REC.VendorId_U32
    initParam.productCode             = UINT_MAX;         // NMT_IdentityObject_REC.ProductCode_U32
    initParam.revisionNumber          = UINT_MAX;         // NMT_IdentityObject_REC.RevisionNo_U32
    initParam.serialNumber            = UINT_MAX;         // NMT_IdentityObject_REC.SerialNo_U32

    initParam.subnetMask              = SUBNET_MASK;
    initParam.defaultGateway          = DEFAULT_GATEWAY;
    sprintf((char*)initParam.sHostname, "%02x-%08x", initParam.nodeId, initParam.vendorId);
    initParam.syncNodeId              = C_ADR_SYNC_ON_SOA;
    initParam.fSyncOnPrcNode          = FALSE;

    // set callback functions
    initParam.pfnCbEvent = &MyoMaster::processEvents;

#if defined(CONFIG_KERNELSTACK_DIRECTLINK)
    initParam.pfnCbSync  = (tSyncCb)&MyoMaster::processSync;
#else
    initParam.pfnCbSync  = NULL;
#endif

    // Initialize object dictionary
    ret = obdcreate_initObd(&initParam.obdInitParam);
    if (ret != kErrorOk)
    {
        eventlog_printMessage(kEventlogLevelFatal,
                              kEventlogCategoryControl,
                              "obdcreate_initObd() failed with \"%s\" (0x%04x)\n",
                              debugstr_getRetValStr(ret),
                              ret);
        return ret;
    }

    // initialize POWERLINK stack
    ret = oplk_initialize();
    if (ret != kErrorOk)
    {
        fprintf(stderr,
                "oplk_initialize() failed with \"%s\" (0x%04x)\n",
                debugstr_getRetValStr(ret),
                ret);
        eventlog_printMessage(kEventlogLevelFatal,
                              kEventlogCategoryControl,
                              "oplk_init() failed with \"%s\" (0x%04x)\n",
                              debugstr_getRetValStr(ret),
                              ret);
        return ret;
    }

    ret = oplk_create(&initParam);
    if (ret != kErrorOk)
    {
        fprintf(stderr,
                "oplk_create() failed with \"%s\" (0x%04x)\n",
                debugstr_getRetValStr(ret),
                ret);
        return ret;
    }

    ret = oplk_setCdcFilename(cdcFileName_p);
    if (ret != kErrorOk)
    {
        fprintf(stderr,
                "oplk_setCdcFilename() failed with \"%s\" (0x%04x)\n",
                debugstr_getRetValStr(ret),
                ret);
        eventlog_printMessage(kEventlogLevelFatal,
                              kEventlogCategoryControl,
                              "oplk_setCdcFilename() failed with \"%s\" (0x%04x)\n",
                              debugstr_getRetValStr(ret),
                              ret);
        return ret;
    }

    return kErrorOk;
}

tOplkError MyoMaster::processSync() {
    tOplkError ret;

    ret = oplk_waitSyncEvent(100000);
    if (ret != kErrorOk)
        return ret;

    ret = oplk_exchangeProcessImageOut();
    if (ret != kErrorOk)
        return ret;

    static int iter = 1;
    if ((iter++) % 50 == 0) {
        printf("\n############## myoFPGA ###################\n");
        printf("springDisplacement: %d\n", pProcessImageOut_l->CN1_MotorStatus_springDisplacement_I16_1);
//        printf("springDisplacement: %d\n", pProcessImageOut_l->CN1_MotorStatus_springDisplacement_I16_2);
//        printf("springDisplacement: %d\n", pProcessImageOut_l->CN1_MotorStatus_springDisplacement_I16_3);
//        printf("springDisplacement: %d\n", pProcessImageOut_l->CN1_MotorStatus_springDisplacement_I16_4);
//        printf("springDisplacement: %d\n", pProcessImageOut_l->CN1_MotorStatus_springDisplacement_I16_5);
//        printf("springDisplacement: %d\n", pProcessImageOut_l->CN1_MotorStatus_springDisplacement_I16_6);
//        printf("springDisplacement: %d\n", pProcessImageOut_l->CN1_MotorStatus_springDisplacement_I16_7);
//        printf("springDisplacement: %d\n", pProcessImageOut_l->CN1_MotorStatus_springDisplacement_I16_8);
//        printf("springDisplacement: %d\n", pProcessImageOut_l->CN1_MotorStatus_springDisplacement_I16_9);
//        printf("springDisplacement: %d\n", pProcessImageOut_l->CN1_MotorStatus_springDisplacement_I16_10);
//        printf("springDisplacement: %d\n", pProcessImageOut_l->CN1_MotorStatus_springDisplacement_I16_11);
//        printf("springDisplacement: %d\n", pProcessImageOut_l->CN1_MotorStatus_springDisplacement_I16_12);
//        printf("springDisplacement: %d\n", pProcessImageOut_l->CN1_MotorStatus_springDisplacement_I16_13);
//        printf("springDisplacement: %d\n", pProcessImageOut_l->CN1_MotorStatus_springDisplacement_I16_14);
//        printf("springDisplacement: %d\n", pProcessImageOut_l->CN1_MotorStatus_springDisplacement_I16_15);
//        printf("springDisplacement: %d\n", pProcessImageOut_l->CN1_MotorStatus_springDisplacement_I16_16);
        MyoFPGAProtobuf::motorConfig msg;
        motorConfigSocket->sendMessage<MyoFPGAProtobuf::motorConfig>(msg);
    }
    // setpoints for 16 motors
    pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_1 = setPoints[0];
    pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_2 = setPoints[1];
    pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_3 = setPoints[2];
    pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_4 = setPoints[3];
    pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_5 = setPoints[4];
    pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_6 = setPoints[5];
    pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_7 = setPoints[6];
    pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_8 = setPoints[7];
    pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_9 = setPoints[8];
    pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_10 = setPoints[9];
    pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_11 = setPoints[10];
    pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_12 = setPoints[11];
    pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_13 = setPoints[12];
    pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_14 = setPoints[13];
    pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_15 = setPoints[14];
    pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_16 = setPoints[15];

    ret = oplk_exchangeProcessImageIn();

    MyoFPGAProtobuf::motorStatus msg;
    if(motorStatusSocket->receiveMessage<MyoFPGAProtobuf::motorStatus>(msg)){
        for(int motor=0; motor<msg.current_size(); motor++){
            printf("-----------------------------------------------\n");
            printf("pwmRef:       %d\n", msg.pwmref(motor));
            printf("position:     %d\n", msg.position(motor));
            printf("velocity:     %d\n", msg.velocity(motor));
            printf("displacement: %d\n", msg.displacement(motor));
            printf("current:      %d\n", msg.current(motor));
        }
    }

//    if(updateControllerConfig){
//        socket->sendMotorConfig(&MotorConfig);
//        updateControllerConfig = false;
//    }

    return ret;
}

void MyoMaster::shutdownPowerlink() {
    UINT i;
    tOplkError ret = kErrorOk;

    // NMT_GS_OFF state has not yet been reached
    fGsOff_l = FALSE;

#if (!defined(CONFIG_KERNELSTACK_DIRECTLINK) && \
     defined(CONFIG_USE_SYNCTHREAD))
    system_stopSyncThread();
    system_msleep(100);
#endif

    // halt the NMT state machine so the processing of POWERLINK frames stops
    ret = oplk_execNmtCommand(kNmtEventSwitchOff);
    if (ret != kErrorOk) {
        fprintf(stderr,
                "oplk_execNmtCommand() failed with \"%s\" (0x%04x)\n",
                debugstr_getRetValStr(ret),
                ret);
    }

    // small loop to implement timeout waiting for thread to terminate
    for (i = 0; i < 1000; i++) {
        if (fGsOff_l)
            break;
    }

    printf("Stack is in state off ... Shutdown\n");
    eventlog_printMessage(kEventlogLevelInfo,
                          kEventlogCategoryControl,
                          "Stack is in state off ... Shutdown openPOWERLINK");

    oplk_destroy();
    oplk_exit();
}

tOplkError MyoMaster::processEvents(tOplkApiEventType EventType_p,
                                    const tOplkApiEventArg *pEventArg_p,
                                    void *pUserArg_p){
    tOplkError  ret = kErrorOk;

    // check if NMT_GS_OFF is reached
    switch (EventType_p)
    {
        case kOplkApiEventNmtStateChange:
            ret = processStateChangeEvent(&pEventArg_p->nmtStateChange, pUserArg_p);
            break;

        case kOplkApiEventCriticalError:
        case kOplkApiEventWarning:
            ret = processErrorWarningEvent(&pEventArg_p->internalError, pUserArg_p);
            break;

        case kOplkApiEventHistoryEntry:
            ret = processHistoryEvent(&pEventArg_p->errorHistoryEntry, pUserArg_p);
            break;

        case kOplkApiEventNode:
            ret = processNodeEvent(&pEventArg_p->nodeEvent, pUserArg_p);
            break;

        case kOplkApiEventPdoChange:
            ret = processPdoChangeEvent(&pEventArg_p->pdoChange, pUserArg_p);
            break;

        case kOplkApiEventCfmProgress:
            ret = processCfmProgressEvent(&pEventArg_p->cfmProgress, pUserArg_p);
            break;

        case kOplkApiEventCfmResult:
            ret = processCfmResultEvent(&pEventArg_p->cfmResult, pUserArg_p);
            break;

        default:
            break;
    }

    return ret;
}

tOplkError MyoMaster::processStateChangeEvent(const tEventNmtStateChange* pNmtStateChange_p,
                                              void* pUserArg_p){
    tOplkError  ret = kErrorOk;

    UNUSED_PARAMETER(pUserArg_p);

    if (pfGsOff_l == NULL)
    {
        console_printlog("Application event module is not initialized!\n");
        return kErrorGeneralError;
    }

    eventlog_printStateEvent(pNmtStateChange_p);

    switch (pNmtStateChange_p->newNmtState)
    {
        case kNmtGsOff:
            // NMT state machine was shut down,
            // because of user signal (CTRL-C) or critical POWERLINK stack error
            // -> also shut down oplk_process() and main()
            ret = kErrorShutdown;

            printf("Stack received kNmtGsOff!\n");

            // signal that stack is off
            *pfGsOff_l = TRUE;
            break;

        case kNmtGsResetCommunication:
            break;

        case kNmtGsResetConfiguration:
            break;

        case kNmtGsInitialising:
        case kNmtGsResetApplication:        // Implement
        case kNmtMsNotActive:               // handling of
        case kNmtMsPreOperational1:         // different
        case kNmtMsPreOperational2:         // states here
        case kNmtMsReadyToOperate:
        case kNmtMsOperational:
        case kNmtMsBasicEthernet:           // no break

        default:
            printf("Stack entered state: %s\n",
                   debugstr_getNmtStateStr(pNmtStateChange_p->newNmtState));
            break;
    }

    return ret;
}

tOplkError MyoMaster::processErrorWarningEvent(const tEventError* pInternalError_p, void* pUserArg_p){
    // error or warning occurred within the stack or the application
    // on error the API layer stops the NMT state machine

    UNUSED_PARAMETER(pUserArg_p);

    eventlog_printErrorEvent(pInternalError_p);

    return kErrorOk;
}

tOplkError MyoMaster::processHistoryEvent(const tErrHistoryEntry* pHistoryEntry_p,
                                      void* pUserArg_p) {
    UNUSED_PARAMETER(pUserArg_p);

    eventlog_printHistoryEvent(pHistoryEntry_p);

    return kErrorOk;
}

tOplkError MyoMaster::processNodeEvent(const tOplkApiEventNode* pNode_p,
                                       void* pUserArg_p){
    UNUSED_PARAMETER(pUserArg_p);

    eventlog_printNodeEvent(pNode_p);

    // check additional argument
    switch (pNode_p->nodeEvent)
    {
        case kNmtNodeEventCheckConf:
            break;

        case kNmtNodeEventUpdateConf:
            break;

        case kNmtNodeEventNmtState:
            printf("Node %d entered state %s\n",
                   pNode_p->nodeId,
                   debugstr_getNmtStateStr(pNode_p->nmtState));
            break;

        case kNmtNodeEventError:
            break;

        case kNmtNodeEventFound:
            printf("Stack found node %d\n",
                   pNode_p->nodeId);
            break;

        case kNmtNodeEventAmniReceived:
            break;

        default:
            break;
    }

    return kErrorOk;
}

tOplkError MyoMaster::processCfmProgressEvent(const tCfmEventCnProgress* pCfmProgress_p,
                                   void* pUserArg_p){
    UNUSED_PARAMETER(pUserArg_p);

    eventlog_printCfmProgressEvent(pCfmProgress_p);

    return kErrorOk;
}

tOplkError MyoMaster::processCfmResultEvent(const tOplkApiEventCfmResult* pCfmResult_p,
                                 void* pUserArg_p){
    UNUSED_PARAMETER(pUserArg_p);

    eventlog_printCfmResultEvent(pCfmResult_p->nodeId, pCfmResult_p->nodeCommand);

    switch (pCfmResult_p->nodeCommand)
    {
        case kNmtNodeCommandConfOk:
            break;

        case kNmtNodeCommandConfErr:
            break;

        case kNmtNodeCommandConfReset:
            break;

        case kNmtNodeCommandConfRestored:
            break;

        default:
            break;
    }

    return kErrorOk;
}

tOplkError MyoMaster::processPdoChangeEvent(const tOplkApiEventPdoChange* pPdoChange_p,
                                            void* pUserArg_p){
    UINT        subIndex;
    UINT64      mappObject;
    tOplkError  ret;
    UINT        varLen;

    UNUSED_PARAMETER(pUserArg_p);

    eventlog_printPdoEvent(pPdoChange_p);

    for (subIndex = 1; subIndex <= pPdoChange_p->mappObjectCount; subIndex++)
    {
        varLen = sizeof(mappObject);
        ret = oplk_readLocalObject(pPdoChange_p->mappParamIndex,
                                   subIndex,
                                   &mappObject,
                                   &varLen);
        if (ret != kErrorOk)
        {
            eventlog_printMessage(kEventlogLevelError,
                                  kEventlogCategoryObjectDictionary,
                                  "Reading 0x%X/%d failed with %s(0x%X)",
                                  pPdoChange_p->mappParamIndex,
                                  subIndex,
                                  debugstr_getRetValStr(ret),
                                  ret);
            continue;
        }

        eventlog_printPdoMap(pPdoChange_p->mappParamIndex,
                             subIndex,
                             mappObject);
    }

    return kErrorOk;
}

tOplkError MyoMaster::processSDO(tSdoConHdl conHdl_p,
                                 const tAsySdoSeq *pSdoSeqData_p,
                                 UINT dataSize_p){
    printf("received UDP\n");
}