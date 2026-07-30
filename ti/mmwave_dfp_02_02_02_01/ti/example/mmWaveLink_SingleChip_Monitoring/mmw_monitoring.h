/****************************************************************************************
* FileName     : mmw_monitoringle.h
*
* Description  : This file implements mmwavelink monitoring application.
*
****************************************************************************************
* (C) Copyright 2014, Texas Instruments Incorporated. - TI web address www.ti.com
*---------------------------------------------------------------------------------------
*
*  Redistribution and use in source and binary forms, with or without modification,
*  are permitted provided that the following conditions are met:
*
*    Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of its
*    contributors may be used to endorse or promote products derived from this
*    software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
*  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
*  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  OWNER OR CONTRIBUTORS
*  BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
*  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
*  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
*  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
*  CONTRACT,  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
*  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
*/

/******************************************************************************
* INCLUDE FILES
*******************************************************************************
*/
#include <ti/control/mmwavelink/mmwavelink.h>
#include "mmwl_port_ftdi.h"
#include "rls_osi.h"

/******************************************************************************
* MACROS
*******************************************************************************
*/
/*Maximum number of devices connected*/
#define NUM_CONNECTED_DEVICES_MAX             (4U)

/* Trasnport Types */
#define RL_IMPL_TRANSPORT_IF_SPI              (0U)
#define RL_IMPL_TRANSPORT_IF_UART             (1U)

/*Default master device*/
#define DEFAULT_MASTER_DEVICE                 (0U)

/* Firmware File Type */
#define MMWL_FILETYPE_META_IMG                (0U)

/******************************************************************************
* GLOBAL VARIABLES
*******************************************************************************
*/
typedef void* DevHandle_t;

/******************************************************************************
* STRUCTURE DEFINATION
******************************************************************************
*/

/* Comments/Errors Id/Strings*/
typedef struct {
    int id;
    const char* idMsg;
} idMsg_t;


/******************************************************************************
* FUNCTION DECLARATION
*******************************************************************************
*/

/*Device poweroff*/
int MMWL_powerOff(unsigned char deviceMap);

/*sensor stop*/
int MMWL_sensorStop(unsigned char deviceMap);

/*Sensor start*/
int MMWL_sensorStart(unsigned char deviceMap);

/* Monitoring Type Trigger configuration*/
int MMWL_MonTypeTrigConf();

/*Frame configuration*/
int MMWL_frameConfig(unsigned char deviceMap);

/*Advance Frame configuration*/
int MMWL_advFrameConfig(unsigned char deviceMap);

int MMWL_setAsyncEventDir(unsigned char  deviceMap);

/*consolidated configuration*/
int MMWL_rlRfAnaMonConfig(unsigned char deviceMap);

int MMWL_setRfTempMonConfig(unsigned char deviceMap);

int MMWL_setRfRxGainPhaMonConfig(unsigned char deviceMap);

int32_t MMWL_setRfRxNoiseMonConfig(unsigned char deviceMap);

int32_t MMWL_setRfRxIfStageMonConfig(unsigned char deviceMap);

int32_t MMWL_setRfTxPowMonConfig(unsigned char deviceMap);

int32_t MMWL_setRfTxBallbreakMonConfig(unsigned char deviceMap);

int32_t MMWL_setRfTxGainPhaMonConfig(unsigned char deviceMap);

int32_t MMWL_setRfTxPhaseShifterMonConfig(unsigned char deviceMap);

int32_t MMWL_setRfExtAnaSignalsMonConfig(unsigned char deviceMap);

int32_t MMWL_setRfTxIntAnaSignalMonConfig(unsigned char deviceMap);

int32_t MMWL_setRfRxIntAnaSignalMonConfig(unsigned char deviceMap);

int32_t MMWL_setRfPmClkLoIntAnaSignalsMonConfig(unsigned char deviceMap);

int32_t MMWL_setRfGpadcIntAnaSignalsMonConfig(unsigned char deviceMap);

int32_t MMWL_setRfPllContrlVoltMonConfig(unsigned char deviceMap);

int32_t MMWL_setRfSynthFreqMonConfig(unsigned char deviceMap);

int32_t MMWL_setRfExtAnaSignalsMonConfig(unsigned char deviceMap);

int32_t MMWL_setRfDualClkCompMonConfig(unsigned char deviceMap);

int32_t MMWL_setRfRxIfSatMonConfig(unsigned char deviceMap);

int32_t MMWL_setRfRxSigImgMonConfig(unsigned char deviceMap);

int32_t MMWL_setRfRxMixerInPwrConfig(unsigned char deviceMap);

/*Chirp configuration*/
int MMWL_chirpConfig(unsigned char deviceMap);

/*Profile configuration*/
int MMWL_profileConfig(unsigned char deviceMap);

int32_t MMMWL_setCalMonConfig(unsigned char deviceMap);
/*HSI lane configuration*/
int MMWL_hsiLaneConfig(unsigned char deviceMap);
int MMWL_laneConfig(unsigned char deviceMap);
int MMWL_lvdsLaneConfig(unsigned char deviceMap);

/*HSI Clock configuration*/
int MMWL_hsiClockConfig(unsigned char deviceMap);
int MMWL_hsiDataRateConfig(unsigned char deviceMap);
int MMWL_setHsiClock(unsigned char deviceMap);

/* Data path(High SPeed Interface: CSI2/LVDS) configuration*/
int MMWL_dataPathConfig(unsigned char deviceMap);

/*RFinit*/
int MMWL_rfInit(unsigned char deviceMap);

/*Lowpower configuration*/
int MMWL_lowPowerConfig(unsigned char deviceMap);

/*Channle, ADC and Dataformat configuration API's*/
int MMWL_basicConfiguration(unsigned char deviceMap, unsigned int cascade);
int MMWL_channelConfig(unsigned char deviceMap, unsigned short cascade);
int MMWL_ldoBypassConfig(unsigned char deviceMap);
int MMWL_adcOutConfig(unsigned char deviceMap);
int MMWL_dataFmtConfig(unsigned char deviceMap);
int MMWL_setMiscConfig(unsigned char deviceMap);

/*RFenable API*/
int MMWL_rfEnable(unsigned char deviceMap);

/*Download firmware API*/
int MMWL_firmwareDownload(unsigned char deviceMap);
int MMWL_fileDownload(unsigned char deviceMap, unsigned int fileLen);
int MMWL_fileWrite(unsigned char deviceMap, unsigned short remChunks,
                unsigned short chunkLen,
                unsigned char *chunk);

/*continuous mode config API*/
int MMWL_setContMode(unsigned char deviceMap);

/*enable dynamic chirp feature*/
int MMWL_dynChirpEnable(unsigned char deviceMap);
/*set chirp dynamically*/
int MMWL_setDynChirpConfig(unsigned char deviceMap);
/*get chirp dynamically*/
int MMWL_getDynChirpConfig(unsigned char deviceMap);

/*Programmable filter configuration*/
int MMWL_progFiltConfig(unsigned char deviceMap);
/*Programmable filter coefficient RAM configuration*/
int MMWL_progFiltCoeffRam(unsigned char deviceMap);

/* Save Calibration Data to a file */
int MMWL_saveCalibDataToFile(unsigned char deviceMap);
int MMWL_savePhShiftCalibDataToFile(unsigned char deviceMap);
/* Load Calibration Data to a file */
int MMWL_LoadCalibDataFromFile(unsigned char deviceMap);
int MMWL_LoadPhShiftCalibDataFromFile(unsigned char deviceMap);

/*Poweron Master*/
int MMWL_powerOnMaster(unsigned char deviceMap);

