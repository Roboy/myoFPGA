/**
********************************************************************************
\file   app.c
\brief  Demo CN application which implements a digital input/output node
This file contains a demo application for digital input/output data.
\ingroup module_demo_cn_console
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2013, SYSTEC electronic GmbH
Copyright (c) 2013, Kalycito Infotech Private Ltd.
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holders nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
------------------------------------------------------------------------------*/

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <stddef.h>

#include <oplk/oplk.h>
#include <oplk/debugstr.h>

#include "app.h"

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------


//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
#include "xap.h"

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
/* process image */
static PI_IN*   pProcessImageIn_l;
static PI_OUT*  pProcessImageOut_l;

typedef struct
{
    // managing node output/ controlled node input
    signed CN1_MotorCommand_setPoint_I32_1:32;
    signed CN1_MotorCommand_setPoint_I32_2:32;
    signed CN1_MotorCommand_setPoint_I32_3:32;
    signed CN1_MotorCommand_setPoint_I32_4:32;
    signed CN1_MotorCommand_setPoint_I32_5:32;
    signed CN1_MotorCommand_setPoint_I32_6:32;
    signed CN1_MotorCommand_setPoint_I32_7:32;
    signed CN1_MotorCommand_setPoint_I32_8:32;
    // controlled node output/ managing node input
    unsigned pwmRef_I16:16;
    signed actualPosition_I32:32;
    signed actualVelocity_I16:16;
    signed actualCurrent_I16:16;
    signed springDisplacement_I16:16;
    signed sensor1_I16:16;
    signed sensor2_I16:16;
} APP_NODE_VAR_T;

static APP_NODE_VAR_T aNodeVar_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError initProcessImage(void);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize the synchronous data application
The function initializes the synchronous data application
\return The function returns a tOplkError error code.
\ingroup module_demo_cn_console
*/
//------------------------------------------------------------------------------
tOplkError initApp(void)
{
    tOplkError ret = kErrorOk;

    ret = initProcessImage();

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Shutdown the synchronous data application
The function shuts down the synchronous data application
\return The function returns a tOplkError error code.
\ingroup module_demo_cn_console
*/
//------------------------------------------------------------------------------
void shutdownApp(void)
{
    oplk_freeProcessImage();
}

//------------------------------------------------------------------------------
/**
\brief  Synchronous data handler
The function implements the synchronous data handler.
\return The function returns a tOplkError error code.
\ingroup module_demo_cn_console
*/
//------------------------------------------------------------------------------
tOplkError processSync(void)
{
    tOplkError      ret = kErrorOk;

    if (oplk_waitSyncEvent(100000) != kErrorOk)
        return ret;

    ret = oplk_exchangeProcessImageOut();
    if (ret != kErrorOk)
        return ret;

    /* read input image - digital outputs */
    aNodeVar_l.CN1_MotorCommand_setPoint_I32_1 = pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_1;
    aNodeVar_l.CN1_MotorCommand_setPoint_I32_2 = pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_2;
    aNodeVar_l.CN1_MotorCommand_setPoint_I32_3 = pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_3;
    aNodeVar_l.CN1_MotorCommand_setPoint_I32_4 = pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_4;
    aNodeVar_l.CN1_MotorCommand_setPoint_I32_5 = pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_5;
    aNodeVar_l.CN1_MotorCommand_setPoint_I32_6 = pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_6;
    aNodeVar_l.CN1_MotorCommand_setPoint_I32_7 = pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_7;
    aNodeVar_l.CN1_MotorCommand_setPoint_I32_8 = pProcessImageIn_l->CN1_MotorCommand_setPoint_I32_8;
////
////    /* setup output image - digital inputs */
    pProcessImageOut_l->CN1_MotorStatus_pwmRef_I16 = aNodeVar_l.pwmRef_I16;
    pProcessImageOut_l->CN1_MotorStatus_actualPosition_I32 = aNodeVar_l.actualPosition_I32;
    pProcessImageOut_l->CN1_MotorStatus_actualVelocity_I16 = aNodeVar_l.actualVelocity_I16;
    pProcessImageOut_l->CN1_MotorStatus_actualCurrent_I16 = aNodeVar_l.actualCurrent_I16;
    pProcessImageOut_l->CN1_MotorStatus_springDisplacement_I16 = aNodeVar_l.springDisplacement_I16;
    pProcessImageOut_l->CN1_MotorStatus_sensor1_I16 = aNodeVar_l.sensor1_I16;
    pProcessImageOut_l->PADDING_VAR_2 = aNodeVar_l.sensor2_I16;

    ret = oplk_exchangeProcessImageIn();

    return ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Initialize process image
The function initializes the process image of the application.
\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError initProcessImage(void)
{
    tOplkError      ret = kErrorOk;
    UINT            varEntries;
    tObdSize        obdSize;

    /* Allocate process image */
    printf("Initializing process image...\n");
    printf("Size of input process image: %ld\n", sizeof(PI_IN));
    printf("Size of output process image: %ld\n", sizeof(PI_OUT));
    ret = oplk_allocProcessImage(sizeof(PI_IN), sizeof(PI_OUT));
    if (ret != kErrorOk)
    {
        return ret;
    }

    pProcessImageIn_l = (PI_IN*)oplk_getProcessImageIn();
    pProcessImageOut_l = (PI_OUT*)oplk_getProcessImageOut();

    /* link process variables used by CN to object dictionary */
    fprintf(stderr, "Linking process image vars:\n");

    varEntries = 1;
    ret &= oplk_linkProcessImageObject(0x6000, 0x01, 0, FALSE, 4, &varEntries);
    ret &= oplk_linkProcessImageObject(0x6000, 0x02, 4, FALSE, 4, &varEntries);
    ret &= oplk_linkProcessImageObject(0x6000, 0x03, 8, FALSE, 4, &varEntries);
    ret &= oplk_linkProcessImageObject(0x6000, 0x04, 12, FALSE, 4, &varEntries);
    ret &= oplk_linkProcessImageObject(0x6000, 0x05, 16, FALSE, 4, &varEntries);
    ret &= oplk_linkProcessImageObject(0x6000, 0x06, 18, FALSE, 4, &varEntries);
    ret &= oplk_linkProcessImageObject(0x6000, 0x07, 20, FALSE, 4, &varEntries);
    ret &= oplk_linkProcessImageObject(0x6000, 0x08, 22, FALSE, 4, &varEntries);
    varEntries = 1;
    ret &= oplk_linkProcessImageObject(0x6001, 0x01, 0, TRUE, 2, &varEntries);
    ret &= oplk_linkProcessImageObject(0x6001, 0x02, 1, TRUE, 2, &varEntries);
    ret &= oplk_linkProcessImageObject(0x6001, 0x03, 2, TRUE, 4, &varEntries);
    ret &= oplk_linkProcessImageObject(0x6001, 0x04, 4, TRUE, 2, &varEntries);
    ret &= oplk_linkProcessImageObject(0x6001, 0x05, 8, TRUE, 2, &varEntries);
    ret &= oplk_linkProcessImageObject(0x6001, 0x06, 10, TRUE, 2, &varEntries);
    ret &= oplk_linkProcessImageObject(0x6001, 0x07, 12, TRUE, 2, &varEntries);
    ret &= oplk_linkProcessImageObject(0x6001, 0x08, 14, TRUE, 2, &varEntries);
    if (ret != kErrorOk)
    {
        fprintf(stderr, "linking process vars failed with \"%s\" (0x%04x)\n", debugstr_getRetValStr(ret), ret);
        return ret;
    }
    fprintf(stderr, "Linking process vars... ok\n\n");

    return kErrorOk;
}

/// \}