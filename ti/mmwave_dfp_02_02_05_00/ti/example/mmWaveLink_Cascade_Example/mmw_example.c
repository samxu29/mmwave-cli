/****************************************************************************************
* FileName     : mmw_example.c
*
* Description  : This file implements mmwave link example - configuration and capture
*				 for cascade system of mmwave sensors.
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
******************************************************************************
*/
#include "mmwl_port_ethernet.h"
#include <stdio.h>
#include <share.h>
#include <string.h>
#include <stdlib.h>
#include "mmw_example.h"
#include "mmw_config.h"
#include <ti/control/mmwavelink/mmwavelink.h>
#include <math.h>
#include "rls_osi.h"
#include <windows.h>

/* AWR2243 meta image file */
#include "firmware/xwr22xx_metaImage.h"

/****************************************************************************************
* USER CONFIGURABLE DEFINITIONS
****************************************************************************************
*/

/* TDA2xx IP Address */
static char mmwl_TDA_IPAddress[] =							"192.168.33.180";

/* TDA2 Configuration Port */
static unsigned short mmwl_TDA_ConfigPort =					5001U;

/* Capture Directory */
static char mmwl_TDA_CaptureDirectory[] =					"/mnt/ssd/MMWL_Capture";

/****************************************************************************************
* MACRO DEFINITIONS
****************************************************************************************
*/
#define MMWL_FW_FIRST_CHUNK_SIZE (224U)
#define MMWL_FW_CHUNK_SIZE (232U)
#define MMWL_META_IMG_FILE_SIZE (sizeof(metaImage))

#define GET_BIT_VALUE(data, noOfBits, location)    ((((rlUInt32_t)(data)) >> (location)) &\
                                               (((rlUInt32_t)((rlUInt32_t)1U << (noOfBits))) - (rlUInt32_t)1U))
/* Async Event Timeouts */
#define MMWL_API_TDA_TIMEOUT								(3000) /* 3 Sec */
#define MMWL_API_INIT_TIMEOUT                (2000) /* 2 Sec*/
#define MMWL_API_START_TIMEOUT               (1000) /* 1 Sec*/
#define MMWL_API_RF_INIT_TIMEOUT             (1000) /* 1 Sec*/

/* MAX unique chirp AWR2243 supports */
#define MAX_UNIQUE_CHIRP_INDEX                (512 -1)

/* MAX index to read back chirp config  */
#define MAX_GET_CHIRP_CONFIG_IDX              14

/* To enable TX2 */
#define ENABLE_TX2                             1

/* LUT Buffer size for Advanced chirp 
   Max size = 12KB (12*1024) */
#define LUT_ADVCHIRP_TABLE_SIZE                5*1024

/******************************************************************************
* GLOBAL VARIABLES/DATA-TYPES DEFINITIONS
******************************************************************************
*/
typedef int (*RL_P_OS_SPAWN_FUNC_PTR)(RL_P_OSI_SPAWN_ENTRY pEntry, const void* pValue, unsigned int flags);
typedef int (*RL_P_OS_DELAY_FUNC_PTR)(unsigned int delay);

/* Global Variable for Device Status */
static CRITICAL_SECTION rlAsyncEvent;
static unsigned char mmwl_bInitComp = 0U;
static unsigned char mmwl_bMssBootErrStatus = 0U;
static unsigned char mmwl_bStartComp = 0U;
static unsigned char mmwl_bRfInitComp = 0U;
static unsigned char mmwl_bSensorStarted = 0U;
static unsigned char mmwl_bGpadcDataRcv = 0U;
static unsigned char mmwl_bMssCpuFault = 0U;
static unsigned char mmwl_bMssEsmFault = 0U;

static unsigned char mmwl_TDA_DeviceMapCascadedMaster = 0U;
static unsigned char mmwl_TDA_DeviceMapCascadedSlaves = 0U;
static unsigned char mmwl_TDA_DeviceMapCascadedAll = 0U;
static unsigned char mmwl_TDA_SlavesEnabled[3] = { 0 };
static unsigned int mmwl_TDA_width[4] = { 0 };
static unsigned int mmwl_TDA_height[4] = { 0 };
static unsigned int mmwl_TDA_framePeriodicity = 0;
static unsigned int mmwl_TDA_numAllocatedFiles = 0;
static unsigned int mmwl_TDA_enableDataPacking = 0;
static unsigned int mmwl_TDA_numFramesToCapture = 0;

static unsigned char mmwl_bTDA_CaptureCardConnect = 0U;
static unsigned char mmwl_bTDA_CreateAppACK = 0U;
static unsigned char mmwl_bTDA_StartRecordACK = 0U;
static unsigned char mmwl_bTDA_StopRecordACK = 0U;
static unsigned char mmwl_bTDA_FramePeriodicityACK = 0U;
static unsigned char mmwl_bTDA_FileAllocationACK = 0U;
static unsigned char mmwl_bTDA_DataPackagingACK = 0U;
static unsigned char mmwl_bTDA_CaptureDirectoryACK = 0U;
static unsigned char mmwl_bTDA_NumFramesToCaptureACK = 0U;
static unsigned char mmwl_bTDA_ARMDone = 0U;

unsigned char gAwr2243CrcType = RL_CRC_TYPE_32BIT;

rlUInt16_t lutOffsetInNBytes = 0;

/* Global variable configurations from config file */
rlDevGlobalCfg_t rlDevGlobalCfgArgs = { 0 };

/* store frame periodicity */
unsigned int framePeriodicity = 0;
/* store frame count */
unsigned int frameCount = 0;
/* store continous streaming time for TDA to capture the data */
unsigned int gContStreamTime = 0;
/* store continous streaming sampling rate */
unsigned int gContStreamSampleRate = 0;

/* SPI Communication handle to AWR2243 device*/
rlComIfHdl_t mmwl_devHdl = NULL;

/* structure parameters of two profile confing and cont mode config are same */
rlProfileCfg_t profileCfgArgs[2] = { 0 };

/* structure parameters of four profile confing of Adv chirp */
rlProfileCfg_t ProfileCfgArgs_AdvChirp[4] = { 0 };

/* strcture to store dynamic chirp configuration */
rlDynChirpCfg_t dynChirpCfgArgs[3] = { 0 };

/* Strcture to store async event config */
rlRfDevCfg_t rfDevCfg = { 0x0 };

/* Structure to store GPADC measurement data sent by device */
rlRecvdGpAdcData_t rcvGpAdcData = {0};

/* calibData is the calibration data sent by the device which needs to store to
   sFlash and will be used for factory calibration or embedded in the application itself */
rlCalibrationData_t calibData = { 0 };
rlPhShiftCalibrationData_t phShiftCalibData = { 0 };

/* File Handle for Calibration Data */
FILE *CalibrationDataPtr = NULL;
FILE *PhShiftCalibrationDataPtr = NULL;

/* Advanced Chirp LUT data */
/* Max size of the LUT is 12KB.
   Maximum of 212 bytes per chunk can be present per SPI message. */
/* This array is created to store the LUT RAM values from the user programmed parameters or config file.
   The populated array is sent over SPI to populate the RadarSS LUT RAM.
   This array is also saved into a file "AdvChirpLUTData.txt" for debug purposes */
/* The chirp paramters start address offset should be 4 byte aligned */
rlInt8_t AdvChirpLUTData[LUT_ADVCHIRP_TABLE_SIZE] = { 0 };

/* File Handle for Advanced Chirp LUT Data */
FILE *AdvChirpLUTDataPtr = NULL;

/* This will be used to alternatively switch the Adv chirp LUT offset 
   between the PING and the PONG buffer */
unsigned char gDynAdvChirpLUTBufferDir = 0;

uint64_t computeCRC(uint8_t *p, uint32_t len, uint8_t width);

#define USE_SYSTEM_TIME
static void rlsGetTimeStamp(char *Tsbuffer)
{
#ifdef USE_SYSTEM_TIME
	SYSTEMTIME SystemTime;
	GetLocalTime(&SystemTime);
	sprintf(Tsbuffer, "[%02d:%02d:%02d:%03d]: ", SystemTime.wHour, SystemTime.wMinute, SystemTime.wSecond, SystemTime.wMilliseconds);
#else
	__int64 tickPerSecond;
	__int64 tick;
	__int64 sec;
	__int64 usec;

	/* Get accuracy */
	QueryPerformanceFrequency((LARGE_INTEGER*)&tickPerSecond);

	/* Get tick */
	QueryPerformanceCounter((LARGE_INTEGER*)&tick);
	sec = (__int64)(tick / tickPerSecond);
	usec = (__int64)((tick - (sec * tickPerSecond)) * 1000000.0 / tickPerSecond);
	sprintf(Tsbuffer, "%07lld.%06lld: ", sec, usec);
#endif
}
#define _CAPTURE_TO_FILE_
#define DEBUG_EN
FILE* rls_traceFp = NULL;
FILE* rls_traceMmwlFp = NULL;
#ifdef DEBUG_EN
void DEBUG_PRINT(char *fmt, ...)
{
	char cBuffer[1000];
	if (TRUE)
	{
		va_list ap;
		va_start(ap, fmt);
		vsnprintf(&cBuffer[0], sizeof(cBuffer), fmt, ap);
#ifdef _CAPTURE_TO_FILE_
		if (rls_traceFp != NULL)
		{
			char tsTime[30] = { 0 };
			rlsGetTimeStamp(tsTime);
			fwrite(tsTime, sizeof(char), strlen(tsTime), rls_traceFp);
			fwrite(cBuffer, sizeof(char), strlen(cBuffer), rls_traceFp);
			fflush(rls_traceFp);
		}
		else
		{
			char tsTime[30] = { 0 };
			rls_traceFp = _fsopen("trace.txt", "wt", _SH_DENYWR);
			rlsGetTimeStamp(tsTime);
			fwrite(tsTime, sizeof(char), strlen(tsTime), rls_traceFp);
			fwrite(cBuffer, sizeof(char), strlen(cBuffer), rls_traceFp);
			fflush(rls_traceFp);
		}
#endif
		va_end(ap);
	}
}
rlInt32_t MMWAVELINK_LOGGING(const rlInt8_t *fmt, ...)
{
	char cBuffer[1000];
	if (TRUE)
	{
		va_list ap;
		va_start(ap, fmt);
		vsnprintf(&cBuffer[0], sizeof(cBuffer), fmt, ap);
#ifdef _CAPTURE_TO_FILE_
		if (rls_traceMmwlFp != NULL)
		{
			char tsTime[30] = { 0 };
			rlsGetTimeStamp(tsTime);
			fwrite(tsTime, sizeof(char), strlen(tsTime), rls_traceMmwlFp);
			fwrite(cBuffer, sizeof(char), strlen(cBuffer), rls_traceMmwlFp);
			fflush(rls_traceMmwlFp);
		}
		else
		{
			char tsTime[30] = { 0 };
			rls_traceMmwlFp = _fsopen("mmwavelink_log.txt", "wt", _SH_DENYWR);
			rlsGetTimeStamp(tsTime);
			fwrite(tsTime, sizeof(char), strlen(tsTime), rls_traceMmwlFp);
			fwrite(cBuffer, sizeof(char), strlen(cBuffer), rls_traceMmwlFp);
			fflush(rls_traceMmwlFp);
		}
#endif
		va_end(ap);
	}
	return 0;
}
#else
#define DEBUG_PRINT
#endif


FILE* rls_traceF = NULL;
void CloseTraceFile()
{
	if (rls_traceF != NULL)
	{
		fclose(rls_traceF);
		rls_traceF = NULL;
	}
}

rlReturnVal_t rlDeviceFileDownloadWrap(rlUInt8_t deviceMap, rlUInt16_t remChunks, rlFileData_t* data)
{
	return(rlDeviceFileDownload(deviceMap, data, remChunks));
}

#define API_TYPE_A		0x00000000
#define API_TYPE_B		0x10000000
#define API_TYPE_C		0x20000000

typedef struct
{
	unsigned int deviceIndex;
	unsigned int apiInfo;
	void *payLoad;
	unsigned int flag;
}taskData;

rlReturnVal_t (*funcTableTypeA[])(unsigned char, void *) =
{
	rlSetAdcOutConfig,
	rlSetLowPowerModeConfig,
	rlSetChannelConfig,
	rlSetBpmChirpConfig,
	rlRfCalibDataRestore,
	rlSetFrameConfig,
	rlSetAdvChirpConfig,
	rlSetAdvFrameConfig,
	rlRfDynamicPowerSave,
	rlRfDfeRxStatisticsReport,
	rlSetContModeConfig,
	rlEnableContMode,
	rlDeviceSetDataFmtConfig,
	rlDeviceSetDataPathConfig,
	rlDeviceSetMiscConfig,
	rlDeviceSetLaneConfig,
	rlDeviceSetDataPathClkConfig,
	rlDeviceSetLvdsLaneConfig,
	rlDeviceSetContStreamingModeConfig,
	rlDeviceSetCsi2Config,
	rlDeviceSetHsiClk,
	rlRfSetLdoBypassConfig,
	rlSetGpAdcConfig,
	rlRfSetDeviceCfg,
	rlRfSetPALoopbackConfig,
	rlRfSetPSLoopbackConfig,
	rlRfSetIFLoopbackConfig,
	rlRfSetProgFiltCoeffRam,
	rlRfSetProgFiltConfig,
	rlRfSetMiscConfig,
	rlRfSetCalMonTimeUnitConfig,
	rlRfSetCalMonFreqLimitConfig,
	rlRfInitCalibConfig,
	rlRfRunTimeCalibConfig,
	rlRfDigMonEnableConfig,
	rlRfCalibDataStore,
	rlDeviceSetTestPatternConfig,
	rlRfTxFreqPwrLimitConfig,
	rlRfRxGainPhMonConfig,
	rlRfInterRxGainPhaseConfig,
	rlRfTxPhShiftMonConfig,
	rlRfAnaFaultInjConfig,
	rlRfRxIfSatMonConfig,
	rlRfRxSigImgMonConfig,
	rlTxGainTempLutSet,
	rlRfAnaMonConfig,
	rlDeviceLatentFaultTests,
	rlDeviceEnablePeriodicTests,
	rlSetLoopBckBurstCfg,
	rlSetSubFrameStart,
	rlDeviceMcuClkConfig,
	rlRfPhShiftCalibDataRestore,
	rlSetInterChirpBlkCtrl,
	rlDevicePmicClkConfig,
	rlSetDynChirpEn,
	rlRfTxGainPhaseMismatchMonConfig,
	rlRfTempMonConfig,
	rlRfExtAnaSignalsMonConfig,
	rlRfGpadcIntAnaSignalsMonConfig,
	rlRfPmClkLoIntAnaSignalsMonConfig,
	rlRfRxIntAnaSignalsMonConfig,
	rlRfTxIntAnaSignalsMonConfig,
	rlRfDualClkCompMonConfig,
	rlRfPllContrlVoltMonConfig,
	rlRfSynthFreqMonConfig,
	rlRfTxPowrMonConfig,
	rlRfRxNoiseMonConfig,
	rlRfRxMixerInPwrConfig,
	rlRfRxIfStageMonConfig,
	rlRfDigMonPeriodicConfig,
	rlRfTxBallbreakMonConfig,
	rlRfPhShiftCalibDataStore,
	rlDeviceGetRfVersion,
	rlDeviceGetMssVersion,
	rlGetAdvFrameConfig,
	rlSetTestSourceConfig,
	rlTestSourceEnable,
	rlDeviceGetVersion,
	rlGetRfDieId,
	rlMonTypeTrigConfig,
	rlRfApllSynthBwCtlConfig,
	rlDeviceSetDebugSigEnableConfig,
	rlDeviceSetHsiDelayDummyConfig,
	rlSetAdvChirpLUTConfig,
	rlFrameStartStop,
	rlGetRfBootupStatus,
    rlSetAdvChirpDynLUTAddrOffConfig,
	rlRfGetTemperatureReport

#define SET_ADC_OUT_IND								0
#define SET_LOW_POWER_MODE_IND						1
#define SET_CHANNEL_CONFIG_IND						2
#define SET_BPM_CHIRP_CONFIG_IND					3
#define RF_CALIB_DATA_RESTORE_IND					4
#define SET_FRAME_CONFIG_IND						5
#define SET_ADV_CHIRP_CONFIG_IND					6
#define SET_ADV_FRAME_CONFIG_IND					7
#define RF_DYNAMIC_POWER_SAVE_IND					8
#define RF_DFE_RX_STATS_REPORT_IND					9
#define SET_CONT_MODE_CONFIG_IND					10
#define ENABLE_CONT_MODE_IND						11
#define SET_DATA_FORMAT_CONFIG_IND					12
#define SET_DATA_PATH_CONFIG_IND					13
#define SET_MISC_CONFIG_IND							14
#define SET_LANE_CONFIG_IND							15
#define SET_DATA_PATH_CLK_CONFIG_IND				16
#define SET_LVDS_LANE_CONFIG_IND					17
#define SET_CONT_STREAM_MODE_CONFIG_IND				18
#define SET_CSI2_CONFIG_IND							19
#define SET_HSI_CLK_IND								20
#define SET_LDO_BYPASS_CONFIG_IND					21
#define SET_GPADC_CONFIG							22
#define RF_SET_DEVICE_CONFIG_IND					23
#define RF_SET_PA_LPBK_CONFIG_IND					24
#define RF_SET_PS_LPBK_CONFIG_IND					25
#define RF_SET_IF_LPBK_CONFIG_IND					26
#define RF_SET_PROG_FILT_COEFF_RAM_IND				27
#define RF_SET_PROG_FILT_CONFIG_IND					28
#define RF_SET_MISC_CONFIG_IND						29
#define RF_SET_CAL_MON_TIME_CONFIG_IND				30
#define RF_SET_CAL_MON_FREQ_LIM_IND					31
#define RF_INIT_CALIB_CONFIG_IND					32
#define RF_RUN_TIME_CALIB_CONFIG_IND				33
#define RF_DIG_MON_ENABLE_CONFIG					34
#define RF_CALIB_DATA_STORE_IND						35
#define SET_TEST_PATTERN_CONFIG_IND					36
#define RF_TX_FREQ_PWR_LIMIT_CONFIG_IND				37
#define RF_RX_GAIN_PH_MON_CONFIG_IND				38
#define RF_INTER_RX_GAIN_PHASE_CONFIG_IND			39
#define RF_TX_PH_SHIFT_MON_CONFIG_IND			    40
#define RF_ANA_FAULT_INJ_CONFIG_IND					41
#define RF_RX_IF_SAT_MON_CONFIG_IND					42
#define RF_RX_SIG_IMG_MON_CONFIG_IND				43
#define TX_GAIN_TEMP_LUT_SET_IND					44
#define RF_ANA_MON_CONFIG_IND						45
#define LATENT_FAULT_TESTS_IND						46
#define ENABLE_PERIODIC_TESTS_IND					47
#define SET_LOOPBAK_BURST_CFG_IND					48
#define SET_SUBFRAME_START_IND						49
#define MCU_CLK_CONFIG_IND							50
#define RF_PH_SHIFT_CALIB_DATA_RESTORE_IND			51
#define SET_INTER_CHIRP_BLK_CTRL_IND				52
#define PMIC_CLK_CONFIG_IND							53
#define SET_DYN_CHIRP_EN_IND						54
#define RF_TX_GAIN_PHASE_MISMATCH_CONFIG_IND		55
#define RF_TEMP_MON_CONFIG_IND						56
#define RF_EXT_ANA_SIGNALS_MON_CONFIG_IND			57
#define RF_GPADC_INT_ANA_SIGNALS_MON_CONFIG_IND		58
#define RF_PMCLK_LO_INT_ANA_SIGNALS_MON_CONFIG_IND	59
#define RF_RX_INT_ANA_SIGNALS_MON_CONFIG_IND		60
#define RF_TX_INT_ANA_SIGNALS_MON_CONFIG_IND		61
#define RF_DUAL_CLK_COMP_MON_CONFIG_IND				62
#define RF_PLL_CONTRL_VOLT_MON_CONFIG_IND			63
#define RF_SYNTH_FREQ_MON_CONFIG_IND				64
#define RF_TX_POWR_MON_CONFIG_IND					65
#define RF_RX_NOISE_MON_CONFIG_IND					66
#define RF_RX_MIXER_IN_PWR_CONFIG_IND				67
#define RF_RX_IF_STAGE_MON_CONFIG_IND				68
#define RF_DIG_MON_PERIODIC_CONFIG_IND				69
#define RF_TX_BALL_BREAK_MON_CONFIG_IND				70
#define RF_PH_SHIFT_CALIB_DATA_STORE_IND			71
#define GET_RF_VERSION_IND							72
#define GET_MSS_VERSION_IND							73
#define GET_ADV_FRAME_CONFIG_IND					74
#define SET_TEST_SOURCE_CONFIG_IND					75
#define RF_TEST_SOURCE_ENABLE						76
#define RF_GET_VERSION_IND							77
#define RF_GET_DIE_ID_IND                           78
#define RF_SET_MON_TYPE_TRIGGER_CONFIG_IND          79
#define RF_SET_APLL_SYNTH_BW_CTL_CONFIG_IND         80
#define RF_SET_DEBUG_SIGNALS_CONFIG_IND             81
#define RF_SET_CSI2_DELAY_DUMMY_CONFIG_IND          82
#define SET_ADV_CHIRP_LUT_CONFIG_IND			    83
#define SENSOR_START_STOP_IND			            84
#define GET_RF_BOOTUP_STATUS_IND			        85
#define SET_ADV_CHIRP_DYN_LUT_CONFIG_IND			86
#define GET_TEMP_DEVICE_IND							87
};								

rlReturnVal_t(*funcTableTypeB[])(unsigned char) =
{
	rlDeviceAddDevices,
	rlDeviceRemoveDevices,
	rlDeviceRfStart,
	rlRfInit,
	rlSensorStart,
	rlSensorStop

#define ADD_DEVICE_IND					0
#define REMOVE_DEVICE_IND				1
#define RF_START_IND					2
#define RF_INIT_IND						3
#define SENSOR_START_IND				4
#define SENSOR_STOP_IND					5
};

rlReturnVal_t(*funcTableTypeC[])(unsigned char, unsigned short, void *) =
{
	rlSetProfileConfig,
	rlSetChirpConfig,
	rlRfSetPhaseShiftConfig,
	rlDeviceFileDownloadWrap,
	rlSetDynChirpCfg,
	rlSetDynPerChirpPhShifterCfg,
	rlGetProfileConfig

#define SET_PROFILE_CONFIG_IND						0
#define SET_CHIRP_CONFIG_IND						1
#define RF_SET_PHASE_SHIFT_CONFIG_IND				2
#define FILE_DOWNLOAD_IND							3
#define SET_DYN_CHIRP_CFG_IND						4
#define SET_DYN_PER_PERCHIRP_PH_SHIFTER_CFG_IND		5
#define GET_PROFILE_CONFIG_IND						6
};

rlReturnVal_t threadRetVal[TDA_NUM_CONNECTED_DEVICES_MAX];
DWORD WINAPI MyThreadFunction(LPVOID lpParam)
{
	taskData *myData = (taskData*)lpParam;
	unsigned int apiId = myData->apiInfo & 0xFFFF;
	unsigned int apiType = myData->apiInfo & 0xF0000000;

	switch (apiType)
	{
	case API_TYPE_A:
		threadRetVal[myData->deviceIndex] = funcTableTypeA[apiId]((1 << myData->deviceIndex), myData->payLoad);
		break;
	case API_TYPE_B:
		threadRetVal[myData->deviceIndex] = funcTableTypeB[apiId](1 << myData->deviceIndex);
		break;
	case API_TYPE_C:
		threadRetVal[myData->deviceIndex] = funcTableTypeC[apiId]((1 << myData->deviceIndex), myData->flag, myData->payLoad);
		break;
	default:
		threadRetVal[myData->deviceIndex] = -1;
	}

	return threadRetVal[myData->deviceIndex];
}

int callThreadApi(unsigned int apiInfo, unsigned int deviceMap, void *apiParams, unsigned int flags)
{
	int 	retVal = RL_RET_CODE_OK;
	DWORD   dwThreadIdArray[TDA_NUM_CONNECTED_DEVICES_MAX] = { 0 };
	HANDLE  hThreadArray[TDA_NUM_CONNECTED_DEVICES_MAX] = { 0 };
	taskData myTaskData[TDA_NUM_CONNECTED_DEVICES_MAX];
	volatile int devIndex = 0;

	while (deviceMap != 0U)
	{
		if ((deviceMap & (1U << devIndex)) != 0U)
		{
			myTaskData[devIndex].deviceIndex = devIndex;
			myTaskData[devIndex].payLoad = apiParams;
			myTaskData[devIndex].apiInfo = apiInfo;
			myTaskData[devIndex].flag = flags;
			threadRetVal[devIndex] = -1;
			/* create a thread */
			hThreadArray[devIndex] = CreateThread(NULL, 0, MyThreadFunction, &myTaskData[devIndex], 0, &dwThreadIdArray[devIndex]);
		}
		deviceMap &= ~(1U << devIndex);
		devIndex++;
	}

	for (devIndex = 0; devIndex < 4; devIndex++)
	{
		if (hThreadArray[devIndex] != 0)
		{
			WaitForSingleObject(hThreadArray[devIndex], INFINITE);
			retVal |= threadRetVal[devIndex];
		}
	}
	return retVal;
}

#define CALL_API(m,n,o,p)		callThreadApi(m, n, o, p)
/******************************************************************************
* all function definations starts here
*******************************************************************************
*/

void TDA_asyncEventHandler(rlUInt16_t deviceMap, rlUInt16_t cmdCode, rlUInt16_t ackCode,
	rlInt32_t status, rlUInt8_t *data)
{
	switch (cmdCode)
	{
	case CAPTURE_RESPONSE_ACK:
	{
		printf("Device map %u : CAPTURE_RESPONSE_ACK Async event recieved with status %d \n\n", deviceMap, status);
		if (ackCode == CAPTURE_CONFIG_CONNECT)
		{
			mmwl_bTDA_CaptureCardConnect = 1U;
		}
		if (deviceMap == 32)
		{
			if (ackCode == CAPTURE_CONFIG_CREATE_APPLICATION)
			{
				mmwl_bTDA_CreateAppACK = 1;
			}
			else if (ackCode == CAPTURE_DATA_START_RECORD)
			{
				mmwl_bTDA_StartRecordACK = 1;
			}
			else if (ackCode == CAPTURE_DATA_STOP_RECORD)
			{
				mmwl_bTDA_StopRecordACK = 1;
			}
			else if (ackCode == CAPTURE_DATA_FRAME_PERIODICITY)
			{
				mmwl_bTDA_FramePeriodicityACK = 1;
			}
			else if (ackCode == CAPTURE_DATA_NUM_ALLOCATED_FILES)
			{
				mmwl_bTDA_FileAllocationACK = 1;
			}
			else if (ackCode == CAPTURE_DATA_ENABLE_DATA_PACKAGING)
			{
				mmwl_bTDA_DataPackagingACK = 1;
			}
			else if (ackCode == CAPTURE_DATA_SESSION_DIRECTORY)
			{
				mmwl_bTDA_CaptureDirectoryACK = 1;
			}
			else if (ackCode == CAPTURE_DATA_NUM_FRAMES)
			{
				mmwl_bTDA_NumFramesToCaptureACK = 1;
			}
		}
	}
	break;
	case CAPTURE_RESPONSE_NACK:
	{
		printf("Device map %u : CAPTURE_RESPONSE_NACK Async event recieved with status %d \n\n", deviceMap, status);
	}
	break;
	case CAPTURE_RESPONSE_VERSION_INFO:
	{
		unsigned char rcvData[100] = { 0 };
		if (data != NULL)
			memcpy(&rcvData, data, sizeof(rcvData));
		printf("Device map %u : CAPTURE_RESPONSE_VERSION_INFO Async event recieved with status %d. TDA Version : %s \n\n", deviceMap, status, \
			rcvData);
	}
	break;
	case CAPTURE_RESPONSE_CONFIG_INFO:
	{
		unsigned char rcvData_1[8] = { 0 };
		if (data != NULL)
			memcpy(&rcvData_1, data, sizeof(rcvData_1));
		unsigned int width = rcvData_1[0] | (rcvData_1[1] << 1) | (rcvData_1[2] << 2) | (rcvData_1[3] << 3);
		unsigned int height = rcvData_1[4] | (rcvData_1[5] << 1) | (rcvData_1[6] << 2) | (rcvData_1[7] << 3);
		printf("Device map %u : CAPTURE_RESPONSE_CONFIG_INFO Async event recieved with status %d. Width : %d and Height : %d \n\n", deviceMap, status, width, height);
	}
	break;
	case CAPTURE_RESPONSE_TRACE_DATA:
	{
		printf("Device map %u : CAPTURE_RESPONSE_TRACE_DATA Async event recieved with status %d \n\n", deviceMap, status);
		break;
	}
	case CAPTURE_RESPONSE_GPIO_DATA:
	{
		unsigned char rcvData_2[12] = { 0 };
		if (data != NULL)
			memcpy(&rcvData_2, data, sizeof(rcvData_2));
		unsigned int gpioVal = rcvData_2[8] | (rcvData_2[9] << 1) | (rcvData_2[10] << 2) | (rcvData_2[11] << 3);
		printf("Device map %u : CAPTURE_RESPONSE_GPIO_DATA Async event recieved with status %d. GPIO Value : %d \n\n", deviceMap, status, gpioVal);
		break;
	}
	case SENSOR_RESPONSE_SOP_INFO:
	{
		unsigned char rcvData_3[4] = { 0 };
		if (data != NULL)
			memcpy(&rcvData_3, data, sizeof(rcvData_3));
		unsigned int sopMode = rcvData_3[0] | (rcvData_3[1] << 1) | (rcvData_3[2] << 2) | (rcvData_3[3] << 3);
		printf("Device map %u : SENSOR_RESPONSE_SOP_INFO Async event recieved with status %d. SOP Mode : %d \n\n", deviceMap, status, sopMode);
		break;
	}
	case CAPTURE_RESPONSE_NETWORK_ERROR:
	{
		printf("CAPTURE_RESPONSE_NETWORK_ERROR Async event recieved! Connection error! Please reboot the TDA board\n\n");
		break;
	}
	default:
	{
		printf("Device map %u : Unhandled Async Event with cmdCode = 0x%x and status = %d  \n\n", deviceMap, cmdCode, status);
		break;
	}
	}
}

/** @fn void MMWL_asyncEventHandler(rlUInt8_t deviceIndex, rlUInt16_t sbId,
*    rlUInt16_t sbLen, rlUInt8_t *payload)
*
*   @brief Radar Async Event Handler callback
*   @param[in] msgId - Message Id
*   @param[in] sbId - SubBlock Id
*   @param[in] sbLen - SubBlock Length
*   @param[in] payload - Sub Block Payload
*
*   @return None
*
*   Radar Async Event Handler callback
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
void MMWL_asyncEventHandler(rlUInt8_t deviceIndex, rlUInt16_t sbId,
    rlUInt16_t sbLen, rlUInt8_t *payload)
{
    rlUInt16_t msgId = sbId / RL_MAX_SB_IN_MSG;
    rlUInt16_t asyncSB = RL_GET_SBID_FROM_MSG(sbId, msgId);

    /* Host can receive Async Event from RADARSS/MSS */
    switch (msgId)
    {
        /* Async Event from RADARSS */
        case RL_RF_ASYNC_EVENT_MSG:
        {
            switch (asyncSB)
            {
            case RL_RF_AE_FRAME_TRIGGER_RDY_SB:
            {
				EnterCriticalSection(&rlAsyncEvent);
				unsigned int deviceMap = createDevMapFromDevId(deviceIndex);
                mmwl_bSensorStarted |= (1 << deviceIndex);
				printf("Device map %u : Frame Start Async event\n\n", deviceMap);
				LeaveCriticalSection(&rlAsyncEvent);
            }
            break;
            case RL_RF_AE_FRAME_END_SB:
            {
				EnterCriticalSection(&rlAsyncEvent);
				unsigned int deviceMap = createDevMapFromDevId(deviceIndex);
				mmwl_bSensorStarted &= ~(1 << deviceIndex);
				printf("Device map %u : Frame End Async event\n\n", deviceMap);
				LeaveCriticalSection(&rlAsyncEvent);
            }
            break;
            case RL_RF_AE_CPUFAULT_SB:
            {
				unsigned int deviceMap = createDevMapFromDevId(deviceIndex);
				printf("Device map %u : BSS CPU Fault Async event\n\n", deviceMap);
                while(1);
            }
            case RL_RF_AE_ESMFAULT_SB:
            {
				unsigned int deviceMap = createDevMapFromDevId(deviceIndex);
				printf("Device map %u : BSS ESM Fault Async event\n\n", deviceMap);
            }
			break;
            case RL_RF_AE_INITCALIBSTATUS_SB:
            {
				EnterCriticalSection(&rlAsyncEvent);
				unsigned int deviceMap = createDevMapFromDevId(deviceIndex);
				mmwl_bRfInitComp |= (1 << deviceIndex);
				printf("Device map %u : RF-Init Async event\n\n", deviceMap);
				LeaveCriticalSection(&rlAsyncEvent);
            }
            break;
            case RL_RF_AE_MON_TIMING_FAIL_REPORT_SB:
            {
                EnterCriticalSection(&rlAsyncEvent);
                unsigned int deviceMap = createDevMapFromDevId(deviceIndex);
				rlCalMonTimingErrorReportData_t *data = (rlCalMonTimingErrorReportData_t*)payload;
                printf("Device map %u : Cal Mon Time Unit Fail [0x%x] Async event\n\n", deviceMap, data->timingFailCode);
                LeaveCriticalSection(&rlAsyncEvent);
            }
			break;
            case RL_RF_AE_RUN_TIME_CALIB_REPORT_SB:
            {
				unsigned int deviceMap = createDevMapFromDevId(deviceIndex);
				printf("Device map %u : Run time Calibration Report [0x%x] Async event\n\n", deviceMap, ((rlRfRunTimeCalibReport_t*)payload)->calibErrorFlag);
            }
            break;
			case RL_RF_AE_DIG_LATENTFAULT_REPORT_SB:
			{
                unsigned int deviceMap = createDevMapFromDevId(deviceIndex);
				rlDigLatentFaultReportData_t *data = (rlDigLatentFaultReportData_t*)payload;
				printf("Device map %u : Dig Latent Fault report [0x%x] Async event\n\n", deviceMap, data->digMonLatentFault);
			}
			break;
			case RL_RF_AE_MON_DIG_PERIODIC_REPORT_SB:
			{
                unsigned int deviceMap = createDevMapFromDevId(deviceIndex);
				rlDigPeriodicReportData_t *data = (rlDigPeriodicReportData_t*)payload;
				printf("Device map %u : Dig periodic report [0x%x] Async event\n\n", deviceMap, data->digMonPeriodicStatus);
			}
			break;
            case RL_RF_AE_GPADC_MEAS_DATA_SB:
            {
				EnterCriticalSection(&rlAsyncEvent);
                mmwl_bGpadcDataRcv |= (1 << deviceIndex);
                /* store the GPAdc Measurement data which AWR2243 will read from the analog test pins
                    where user has fed the input signal */
                memcpy(&rcvGpAdcData, payload, sizeof(rlRecvdGpAdcData_t));
				LeaveCriticalSection(&rlAsyncEvent);
            }
			break;
            default:
            {
                printf("Unhandled Async Event msgId: 0x%x, asyncSB:0x%x  \n\n", msgId, asyncSB);
            }
            break;
            }

        }
        break;

        /* Async Event from MSS */
        case RL_DEV_ASYNC_EVENT_MSG:
        {
            switch (asyncSB)
            {
                case RL_DEV_AE_MSSPOWERUPDONE_SB:
                {
					EnterCriticalSection(&rlAsyncEvent);
					unsigned int deviceMap = createDevMapFromDevId(deviceIndex);
					mmwl_bInitComp |= (1 << deviceIndex);
					printf("Device map %u : MSS Power Up Async event\n\n", deviceMap);
					rlInitComplete_t *data = (rlInitComplete_t*)payload;
					printf("PowerUp Time = %d, PowerUp Status 1 = 0x%x, PowerUp Status 2 = 0x%x, BootTestStatus 1 = 0x%x, BootTestStatus 2 = 0x%x\n\n", data->powerUpTime, data->powerUpStatus1, data->powerUpStatus2, data->bootTestStatus1, data->bootTestStatus2);
					LeaveCriticalSection(&rlAsyncEvent);
                }
                break;
                case RL_DEV_AE_RFPOWERUPDONE_SB:
                {
					EnterCriticalSection(&rlAsyncEvent);
					unsigned int deviceMap = createDevMapFromDevId(deviceIndex);
					mmwl_bStartComp |= (1 << deviceIndex);
					printf("Device map %u : BSS Power Up Async event\n\n", deviceMap);
					LeaveCriticalSection(&rlAsyncEvent);
                }
                break;
                case RL_DEV_AE_MSS_CPUFAULT_SB:
                {
					EnterCriticalSection(&rlAsyncEvent);
					unsigned int deviceMap = createDevMapFromDevId(deviceIndex);
					mmwl_bMssCpuFault |= (1 << deviceIndex);
					printf("Device map %u : MSS CPU Fault Async event\n\n", deviceMap);
					LeaveCriticalSection(&rlAsyncEvent);
                }
                break;
                case RL_DEV_AE_MSS_ESMFAULT_SB:
                {
					EnterCriticalSection(&rlAsyncEvent);
					unsigned int deviceMap = createDevMapFromDevId(deviceIndex);
					mmwl_bMssEsmFault |= (1 << deviceIndex);
					printf("Device map %u : MSS ESM Fault Async event\n\n", deviceMap);
					rlMssEsmFault_t *data = (rlMssEsmFault_t*)payload;
					printf("ESM Grp1 Error = 0x%x, ESM Grp2 Error = 0x%x\n\n", data->esmGrp1Err, data->esmGrp2Err);
					LeaveCriticalSection(&rlAsyncEvent);
                }
                break;
                case RL_DEV_AE_MSS_BOOTERRSTATUS_SB:
                {
					EnterCriticalSection(&rlAsyncEvent);
					unsigned int deviceMap = createDevMapFromDevId(deviceIndex);
					mmwl_bMssBootErrStatus |= (1 << deviceIndex);
					printf("Device map %u : MSS Boot Error Status Async event\n\n", deviceMap);
					LeaveCriticalSection(&rlAsyncEvent);
                }
                break;
				case RL_DEV_AE_MSS_LATENTFLT_TEST_REPORT_SB:
				{
					unsigned int deviceMap = createDevMapFromDevId(deviceIndex);
					rlMssLatentFaultReport_t *data = (rlMssLatentFaultReport_t*)payload;
					printf("Device map %u : MSS Latent fault [0x%x] [0x%x] Async event\n\n", deviceMap, data->testStatusFlg1, data->testStatusFlg2);
				}
				break;
				case RL_DEV_AE_MSS_PERIODIC_TEST_STATUS_SB:
				{
					unsigned int deviceMap = createDevMapFromDevId(deviceIndex);
					rlMssPeriodicTestStatus_t *data = (rlMssPeriodicTestStatus_t*)payload;
					printf("Device map %u : MSS periodic test [0x%x] Async event\n\n", deviceMap, data->testStatusFlg);
				}
				break;
				case RL_DEV_AE_MSS_RF_ERROR_STATUS_SB:
				{
                    unsigned int deviceMap = createDevMapFromDevId(deviceIndex);
					rlMssRfErrStatus_t *data = (rlMssRfErrStatus_t*)payload;
                    printf("Device map %u : MSS RF Error [0x%x] Status Async event\n\n", deviceMap, data->errStatusFlg);
				}
				break;
                default:
                {
                    printf("Unhandled Async Event msgId: 0x%x, asyncSB:0x%x  \n\n", msgId, asyncSB);
                    break;
                }
            }
        }
        break;

        /* Async Event from MMWL */
        case RL_MMWL_ASYNC_EVENT_MSG:
        {
            switch (asyncSB)
            {
                case RL_MMWL_AE_MISMATCH_REPORT:
                {
                    int errTemp = *(int32_t*)payload;
                    /* CRC mismatched in the received Async-Event msg */
                    if (errTemp == RL_RET_CODE_CRC_FAILED)
                    {
                        unsigned int deviceMap = createDevMapFromDevId(deviceIndex);
                        printf("Device map %u : CRC mismatched in the received Async-Event msg\n\n", deviceMap);
                    }
                    /* Checksum mismatched in the received msg */
                    else if (errTemp == RL_RET_CODE_CHKSUM_FAILED)
                    {
                        unsigned int deviceMap = createDevMapFromDevId(deviceIndex);
                        printf("Device map %u : Checksum mismatched in the received msg\n\n", deviceMap);
                    }
                    /* Polling to HostIRQ is timed out,
                    i.e. Device didn't respond to CNYS from the Host */
                    else if (errTemp == RL_RET_CODE_HOSTIRQ_TIMEOUT)
                    {
                        unsigned int deviceMap = createDevMapFromDevId(deviceIndex);
                        printf("Device map %u : HostIRQ polling timed out\n\n", deviceMap);
                    }
                    /* If any of OSI call-back function returns non-zero value */
                    else if (errTemp == RL_RET_CODE_RADAR_OSIF_ERROR)
                    {
                        unsigned int deviceMap = createDevMapFromDevId(deviceIndex);
                        printf("Device map %u : mmWaveLink OS_IF error \n\n", deviceMap);
                    }
                    break;
                }
            }
            break;
        }
        default:
        {
			unsigned int deviceMap = createDevMapFromDevId(deviceIndex);
            printf("Device map %u : Unhandled Async Event msgId: 0x%x, asyncSB:0x%x  \n\n", deviceMap, msgId, asyncSB);
            break;
        }
    }
}

/** @fn int MMWL_computeCRC(unsigned char* data, unsigned int dataLen, unsigned char crcLen,
                        unsigned char* outCrc)
*
*   @brief Compute the CRC of given data
*
*   @param[in] data - message data buffer pointer
*    @param[in] dataLen - length of data buffer
*    @param[in] crcLen - length of crc 2/4/8 bytes
*    @param[out] outCrc - computed CRC data
*
*   @return int Success - 0, Failure - Error Code
*
*   Compute the CRC of given data
*/
int MMWL_computeCRC(unsigned char* data, unsigned int dataLen, unsigned char crcLen,
                        unsigned char* outCrc)
{
    uint64_t crcResult = computeCRC(data, dataLen, (16 << crcLen));
    memcpy(outCrc, &crcResult, (2 << crcLen));
    return 0;
}

/** @fn int MMWL_powerOnMaster(deviceMap)
*
*   @brief Power on Master API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Power on Master API.
*/
int MMWL_powerOnMaster(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK, timeOutCnt = 0;
    /*
     \subsection     porting_step1   Step 1 - Define mmWaveLink client callback structure
    The mmWaveLink framework is ported to different platforms using mmWaveLink client callbacks. These
    callbacks are grouped as different structures such as OS callbacks, Communication Interface
    callbacks and others. Application needs to define these callbacks and initialize the mmWaveLink
    framework with the structure.

     Refer to \ref rlClientCbs_t for more details
     */
    rlClientCbs_t clientCtx = { 0 };

    /*Read all the parameters from config file*/
    MMWL_readPowerOnMaster(&clientCtx);

    /* store CRC Type which has been read from mmwaveconfig.txt file */
    gAwr2243CrcType = clientCtx.crcType;

    /*
    \subsection     porting_step2   Step 2 - Implement Communication Interface Callbacks
    The mmWaveLink device support several standard communication protocol among SPI and MailBox
    Depending on device variant, one need to choose the communication channel. For e.g
    xWR1443/xWR1642 requires Mailbox interface and AWR2243 supports SPI interface.
    The interface for this communication channel should include 4 simple access functions:
    -# rlComIfOpen
    -# rlComIfClose
    -# rlComIfRead
    -# rlComIfWrite

    Refer to \ref rlComIfCbs_t for interface details
    */
    clientCtx.comIfCb.rlComIfOpen = TDACommOpen;
    clientCtx.comIfCb.rlComIfClose = TDACommClose;
    clientCtx.comIfCb.rlComIfRead = spiReadFromDevice;
    clientCtx.comIfCb.rlComIfWrite = spiWriteToDevice;

    /*   \subsection     porting_step3   Step 3 - Implement Device Control Interface
    The mmWaveLink driver internally powers on/off the mmWave device. The exact implementation of
    these interface is platform dependent, hence you need to implement below functions:
    -# rlDeviceEnable
    -# rlDeviceDisable
    -# rlRegisterInterruptHandler

    Refer to \ref rlDeviceCtrlCbs_t for interface details
    */
    clientCtx.devCtrlCb.rlDeviceDisable = TDADisableDevice;
    clientCtx.devCtrlCb.rlDeviceEnable = TDAEnableDevice;
    clientCtx.devCtrlCb.rlDeviceMaskHostIrq = TDACommIRQMask;
    clientCtx.devCtrlCb.rlDeviceUnMaskHostIrq = TDACommIRQUnMask;
    clientCtx.devCtrlCb.rlRegisterInterruptHandler = TDAregisterCallback;
    clientCtx.devCtrlCb.rlDeviceWaitIrqStatus = TDADeviceWaitIrqStatus;

    /*  \subsection     porting_step4     Step 4 - Implement Event Handlers
    The mmWaveLink driver reports asynchronous event indicating mmWave device status, exceptions
    etc. Application can register this callback to receive these notification and take appropriate
    actions

    Refer to \ref rlEventCbs_t for interface details*/
    clientCtx.eventCb.rlAsyncEvent = MMWL_asyncEventHandler;

    /*  \subsection     porting_step5     Step 5 - Implement OS Interface
    The mmWaveLink driver can work in both OS and NonOS environment. If Application prefers to use
    operating system, it needs to implement basic OS routines such as tasks, mutex and Semaphore


    Refer to \ref rlOsiCbs_t for interface details
    */
    /* Mutex */
    clientCtx.osiCb.mutex.rlOsiMutexCreate = osiLockObjCreate;
    clientCtx.osiCb.mutex.rlOsiMutexLock = osiLockObjLock;
    clientCtx.osiCb.mutex.rlOsiMutexUnLock = osiLockObjUnlock;
    clientCtx.osiCb.mutex.rlOsiMutexDelete = osiLockObjDelete;

    /* Semaphore */
    clientCtx.osiCb.sem.rlOsiSemCreate = osiSyncObjCreate;
    clientCtx.osiCb.sem.rlOsiSemWait = osiSyncObjWait;
    clientCtx.osiCb.sem.rlOsiSemSignal = osiSyncObjSignal;
    clientCtx.osiCb.sem.rlOsiSemDelete = osiSyncObjDelete;

    /* Spawn Task */
    clientCtx.osiCb.queue.rlOsiSpawn = (RL_P_OS_SPAWN_FUNC_PTR)osiExecute;

    /* Sleep/Delay Callback*/
    clientCtx.timerCb.rlDelay = (RL_P_OS_DELAY_FUNC_PTR)osiSleep;
#if 0
	/* Logging in mmWavelink*/
	if (rlDevGlobalCfgArgs.EnableMmwlLogging == 1)
	{	
		clientCtx.dbgCb.dbgLevel = RL_DBG_LEVEL_DATABYTE;
		clientCtx.dbgCb.rlPrint = MMWAVELINK_LOGGING;
	}
#endif
    /*  \subsection     porting_step6     Step 6 - Implement CRC Interface
    The mmWaveLink driver uses CRC for message integrity. If Application prefers to use
    CRC, it needs to implement CRC routine.

    Refer to \ref rlCrcCbs_t for interface details
    */
    clientCtx.crcCb.rlComputeCRC = MMWL_computeCRC;

    /*  \subsection     porting_step7     Step 7 - Define Platform
    The mmWaveLink driver can be configured to run on different platform by
    passing appropriate platform and device type
    */
    clientCtx.platform = RL_PLATFORM_HOST;
    clientCtx.arDevType = RL_AR_DEVICETYPE_22XX;

    /*clear all the interupts flag*/
    mmwl_bInitComp = 0;
    mmwl_bStartComp = 0U;
    mmwl_bRfInitComp = 0U;
	InitializeCriticalSection(&rlAsyncEvent);
    /*  \subsection     porting_step8     step 8 - Call Power ON API and pass client context
    The mmWaveLink driver initializes the internal components, creates Mutex/Semaphore,
    initializes buffers, register interrupts, bring mmWave front end out of reset.
    */
    retVal = rlDevicePowerOn(deviceMap, clientCtx);

    /*  \subsection     porting_step9     step 9 - Test if porting is successful
    Once configuration is complete and mmWave device is powered On, mmWaveLink driver receives
    asynchronous event from mmWave device and notifies application using
    asynchronous event callback.
    Refer to \ref MMWL_asyncEventHandler for event details
	@Note: In case of ES1.0 sample application needs to wait for MSS CPU fault as well with some timeout.
    */
    while ((mmwl_bInitComp & deviceMap) != deviceMap)
    {
        osiSleep(1); /*Sleep 1 msec*/
        timeOutCnt++;
        if (timeOutCnt > MMWL_API_INIT_TIMEOUT)
        {
            retVal = RL_RET_CODE_RESP_TIMEOUT;
            break;
        }
    }
    mmwl_bInitComp = 0U;
    return retVal;
}

int MMWL_fileWrite(unsigned char deviceMap,
                unsigned short remChunks,
                unsigned short chunkLen,
                unsigned char *chunk)
{
    int ret_val = -1;

    rlFileData_t fileChunk = { 0 };
    fileChunk.chunkLen = chunkLen;
    memcpy(fileChunk.fData, chunk, chunkLen);

	ret_val = CALL_API(API_TYPE_C | FILE_DOWNLOAD_IND, deviceMap, &fileChunk, remChunks);
    return ret_val;
}

/** @fn int MMWL_fileDownload((unsigned char deviceMap,
                  mmwlFileType_t fileType,
                  unsigned int fileLen)
*
*   @brief Firmware Download API.
*
*   @param[in] deviceMap - Devic Index
*    @param[in] fileType - firmware/file type
*    @param[in] fileLen - firmware/file length
*
*   @return int Success - 0, Failure - Error Code
*
*   Firmware Download API.
*/
int MMWL_fileDownload(unsigned char deviceMap,
                  unsigned int fileLen)
{
    unsigned int imgLen = fileLen;
    int ret_val = -1;
    int mmwl_iRemChunks = 0;
    unsigned short usChunkLen = 0U;
    unsigned int iNumChunks = 0U;
    unsigned short usLastChunkLen = 0;
    unsigned short usFirstChunkLen = 0;
    unsigned short usProgress = 0;

    /*First Chunk*/
    unsigned char firstChunk[MMWL_FW_CHUNK_SIZE];
    unsigned char* pmmwl_imgBuffer = NULL;

    pmmwl_imgBuffer = (unsigned char*)&metaImage[0];

    if(pmmwl_imgBuffer == NULL)
    {
        printf("Device map %u : MMWL_fileDwld Fail. File Buffer is NULL \n\n\r", deviceMap);
        return -1;
    }

    /*Download to Device*/
    usChunkLen = MMWL_FW_CHUNK_SIZE;
    iNumChunks = (imgLen + 8) / usChunkLen;
    mmwl_iRemChunks = iNumChunks;

    if (mmwl_iRemChunks > 0)
    {
        usLastChunkLen = (imgLen + 8) % usChunkLen;
        usFirstChunkLen = MMWL_FW_CHUNK_SIZE;
		mmwl_iRemChunks += 1;
    }
    else
    {
        usFirstChunkLen = imgLen + 8;
    }

    *((unsigned int*)&firstChunk[0]) = (unsigned int)MMWL_FILETYPE_META_IMG;
    *((unsigned int*)&firstChunk[4]) = (unsigned int)imgLen;
    memcpy((char*)&firstChunk[8], (char*)pmmwl_imgBuffer,
                usFirstChunkLen - 8);

    ret_val = MMWL_fileWrite(deviceMap, (mmwl_iRemChunks-1), usFirstChunkLen,
                              firstChunk);
    if (ret_val < 0)
    {
        printf("Device map %u : MMWL_fileDwld Fail. Ftype: %d\n\n\r", deviceMap, MMWL_FILETYPE_META_IMG);
        return ret_val;
    }
    pmmwl_imgBuffer += MMWL_FW_FIRST_CHUNK_SIZE;
    mmwl_iRemChunks--;

    if(mmwl_iRemChunks > 0)
    {
        printf("Device map %u : Download in Progress: ", deviceMap);
    }
    /*Remaining Chunk*/
    while (mmwl_iRemChunks > 0)
    {
        usProgress = (((iNumChunks - mmwl_iRemChunks) * 100) / iNumChunks);
        printf("%d%%..", usProgress);

		/* Last chunk */
		if ((mmwl_iRemChunks == 1) && (usLastChunkLen > 0))
		{
			ret_val = MMWL_fileWrite(deviceMap, 0, usLastChunkLen,
				pmmwl_imgBuffer);
			if (ret_val < 0)
			{
				printf("Device map %u : MMWL_fileDwld last chunk Fail : Ftype: %d\n\n\r", deviceMap,
					MMWL_FILETYPE_META_IMG);
				return ret_val;
			}
		}
		else
		{
			ret_val = MMWL_fileWrite(deviceMap, (mmwl_iRemChunks - 1),
				MMWL_FW_CHUNK_SIZE, pmmwl_imgBuffer);

			if (ret_val < 0)
			{
				printf("\n\n\r Device map %u : MMWL_fileDwld rem chunk Fail : Ftype: %d\n\n\r", deviceMap,
					MMWL_FILETYPE_META_IMG);
				return ret_val;
			}
			pmmwl_imgBuffer += MMWL_FW_CHUNK_SIZE;
		}

        mmwl_iRemChunks--;
    }
     printf("Done!\n\n");
    return ret_val;
}

/** @fn int MMWL_firmwareDownload(deviceMap)
*
*   @brief Firmware Download API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Firmware Download API.
*/
int MMWL_firmwareDownload(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK, timeOutCnt = 0;

    /* Meta Image download */
    printf("Device map %u : Meta Image download started\n\n",
        deviceMap);
    retVal = MMWL_fileDownload(deviceMap, MMWL_META_IMG_FILE_SIZE);
    printf("Device map %u : Meta Image download complete ret = %d\n\n", deviceMap, retVal);

    return retVal;
}

/** @fn int MMWL_rfEnable(deviceMap)
*
*   @brief RFenable API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   RFenable API.
*/
int MMWL_rfEnable(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK, timeOutCnt = 0;
	retVal = CALL_API(API_TYPE_B | RF_START_IND, deviceMap, NULL, 0);
    while ((mmwl_bStartComp & deviceMap) != deviceMap)
    {
        osiSleep(1); /*Sleep 1 msec*/
        timeOutCnt++;
        if (timeOutCnt > MMWL_API_START_TIMEOUT)
        {
			printf("Device map %u : Timeout! RF Enable Status = %u\n\n", (unsigned int)deviceMap, mmwl_bStartComp);
            retVal = RL_RET_CODE_RESP_TIMEOUT;
            break;
        }
    }
	
	mmwl_bStartComp = mmwl_bStartComp & (~deviceMap);

    if(retVal == RL_RET_CODE_OK)
    {
		for (int devId = 0; devId < 4; devId++)
		{
			if ((deviceMap & (1 << devId)) != 0)
			{
				unsigned char devMap = createDevMapFromDevId(devId);
				rlVersion_t verArgs = { 0 };
				rlRfDieIdCfg_t dieId = { 0 };
				retVal = CALL_API(RF_GET_VERSION_IND, devMap, &verArgs, 0);

				printf("Device map %u : RF Version [%2d.%2d.%2d.%2d] \nDevice map %u : MSS version [%2d.%3d.%2d.%3d] \nDevice map %u : mmWaveLink version [%2d.%2d.%2d.%2d]\n\n",
					devMap, verArgs.rf.fwMajor, verArgs.rf.fwMinor, verArgs.rf.fwBuild, verArgs.rf.fwDebug,
					devMap, verArgs.master.fwMajor, verArgs.master.fwMinor, verArgs.master.fwBuild, verArgs.master.fwDebug,
					devMap, verArgs.mmWaveLink.major, verArgs.mmWaveLink.minor, verArgs.mmWaveLink.build, verArgs.mmWaveLink.debug);
				printf("Device map %u : RF Patch Version [%2d.%2d.%2d.%2d] \nDevice map %u : MSS Patch version [%2d.%2d.%2d.%2d]\n\n",
					devMap, verArgs.rf.patchMajor, verArgs.rf.patchMinor, ((verArgs.rf.patchBuildDebug & 0xF0) >> 4), (verArgs.rf.patchBuildDebug & 0x0F),
					devMap, verArgs.master.patchMajor, verArgs.master.patchMinor, ((verArgs.master.patchBuildDebug & 0xF0) >> 4), (verArgs.master.patchBuildDebug & 0x0F));

				retVal = CALL_API(RF_GET_DIE_ID_IND, devMap, &dieId, 0);
			}
		}
    }
    return retVal;
}

/** @fn int MMWL_dataFmtConfig(unsigned char deviceMap)
*
*   @brief Data Format Config API
*
*   @return Success - 0, Failure - Error Code
*
*   Data Format Config API
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
int MMWL_dataFmtConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    rlDevDataFmtCfg_t dataFmtCfgArgs = { 0 };

    /*dataFmtCfgArgs from config file*/
    MMWL_readDataFmtConfig(&dataFmtCfgArgs);

	retVal = CALL_API(SET_DATA_FORMAT_CONFIG_IND, deviceMap, &dataFmtCfgArgs, 0);
    return retVal;
}

/** @fn int MMWL_ldoBypassConfig(unsigned char deviceMap)
*
*   @brief LDO Bypass Config API
*
*   @return Success - 0, Failure - Error Code
*
*   LDO Bypass Config API
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
int MMWL_ldoBypassConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    rlRfLdoBypassCfg_t rfLdoBypassCfgArgs = { 0 };

    printf("Device map %u : Calling rlRfSetLdoBypassConfig With Bypass [%d] \n\n",
        deviceMap, rfLdoBypassCfgArgs.ldoBypassEnable);

	retVal = CALL_API(SET_LDO_BYPASS_CONFIG_IND, deviceMap, &rfLdoBypassCfgArgs, 0);
    return retVal;
}

/** @fn int MMWL_adcOutConfig(unsigned char deviceMap)
*
*   @brief ADC Configuration API
*
*   @return Success - 0, Failure - Error Code
*
*   ADC Configuration API
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
int MMWL_adcOutConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;

    rlAdcOutCfg_t adcOutCfgArgs = { 0 };

    /*read adcOutCfgArgs from config file*/
    MMWL_readAdcOutConfig(&adcOutCfgArgs);


    printf("Device map %u : Calling rlSetAdcOutConfig With [%d]ADC Bits and [%d]ADC Format \n\n",
        deviceMap, adcOutCfgArgs.fmt.b2AdcBits, adcOutCfgArgs.fmt.b2AdcOutFmt);

	retVal = CALL_API(SET_ADC_OUT_IND, deviceMap, &adcOutCfgArgs, 0);
    return retVal;
}

/** @fn int MMWL_RFDeviceConfig(unsigned char deviceMap)
*
*   @brief RF Device Configuration API
*
*   @return Success - 0, Failure - Error Code
*
*   RF Device Configuration API
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
int MMWL_RFDeviceConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;

    rlRfDevCfg_t rfDevCfgArgs = { 0 };
    rfDevCfgArgs.aeDirection       = 0x5;
    rfDevCfgArgs.aeControl	       = 0x0;
	rfDevCfgArgs.bssAnaControl     = 0x0; /* Clear Inter burst power save */
	rfDevCfgArgs.reserved1         = 0x0;
	rfDevCfgArgs.bssDigCtrl        = 0x0; /* Disable BSS WDT */
	rfDevCfgArgs.aeCrcConfig       = gAwr2243CrcType;
	rfDevCfgArgs.reserved2         = 0x0;
	rfDevCfgArgs.reserved3         = 0x0;

    printf("Device map %u : Calling rlRfSetDeviceCfg With bssAnaControl = [%d] and bssDigCtrl = [%d]\n\n",
        deviceMap, rfDevCfgArgs.bssAnaControl, rfDevCfgArgs.bssDigCtrl);

	retVal = CALL_API(RF_SET_DEVICE_CONFIG_IND, deviceMap, &rfDevCfgArgs, 0);
    return retVal;
}

/** @fn int MMWL_channelConfig(unsigned char deviceMap,
                               unsigned short cascading)
*
*   @brief Channel Config API
*
*   @return Success - 0, Failure - Error Code
*
*   Channel Config API
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
int MMWL_channelConfig(unsigned char deviceMap,
                       unsigned short cascade)
{
    int retVal = RL_RET_CODE_OK;
    /* TBD - Read GUI Values */
    rlChanCfg_t rfChanCfgArgs = { 0 };

    /*read arguments from config file*/
    MMWL_readChannelConfig(&rfChanCfgArgs, cascade);

#if (ENABLE_TX2)
    rfChanCfgArgs.txChannelEn |= (1 << 2); // Enable TX2
#endif

    if(cascade == 2)
    {
        rfChanCfgArgs.cascadingPinoutCfg &= ~(1U << 5U); /* Disable OSC CLK OUT for slaves */	
    }
    printf("Device map %u : Calling rlSetChannelConfig With [%d]Rx and [%d]Tx Channel Enabled \n\n",
           deviceMap, rfChanCfgArgs.rxChannelEn, rfChanCfgArgs.txChannelEn);

	retVal = CALL_API(SET_CHANNEL_CONFIG_IND, deviceMap, &rfChanCfgArgs, 0);
    return retVal;
}

/** @fn int MMWL_setMiscConfig(unsigned char deviceMap)
*
*   @brief Sets misc feature such as per chirp phase shifter and Advance chirp
*
*   @param[in] deviceMap - Device Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Sets misc feature such as per chirp phase shifter and Advance chirp
*/
int MMWL_setMiscConfig(unsigned char deviceMap)
{
	int32_t         retVal;
	rlRfMiscConf_t MiscCfg = { 0 };
	/* Enable Adv chirp feature 
		b0 PERCHIRP_PHASESHIFTER_EN
		b1 ADVANCE_CHIRP_CONFIG_EN  */
	MiscCfg.miscCtl = 0x3;
	retVal = CALL_API(RF_SET_MISC_CONFIG_IND, deviceMap, &MiscCfg, 0);
	return retVal;
}

/** @fn int MMWL_setDeviceCrcType(unsigned char deviceMap)
*
*   @brief Set CRC type of async event from AWR2243 MasterSS
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Set CRC type of async event from AWR2243 MasterSS
*/
int MMWL_setDeviceCrcType(unsigned char deviceMap)
{
    int32_t         retVal;
    rlDevMiscCfg_t devMiscCfg = {0};
    /* Set the CRC Type for Async Event from MSS */
    devMiscCfg.aeCrcConfig = gAwr2243CrcType;
	retVal = CALL_API(SET_MISC_CONFIG_IND, deviceMap, &devMiscCfg, 0);
    return retVal;
}

/** @fn int MMWL_basicConfiguration(unsigned char deviceMap)
*
*   @brief Channel, ADC,Data format configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Channel, ADC,Data format configuration API.
*/
int MMWL_basicConfiguration(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;

    /* ADC out data format configuration */
    retVal = MMWL_adcOutConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Device map %u : AdcOut Config failed with error code %d\n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("Device map %u : AdcOut Configuration success\n\n", deviceMap);
    }

    /* RF device configuration */
    retVal = MMWL_RFDeviceConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Device map %u : RF Device Config failed with error code %d\n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("Device map %u : RF Device Configuration success\n\n", deviceMap);
    }

    /* LDO bypass configuration */
    retVal = MMWL_ldoBypassConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Device map %u : LDO Bypass Config failed with error code %d\n\n",
            deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("Device map %u : LDO Bypass Configuration success\n\n", deviceMap);
    }

    /* Data format configuration */
    retVal = MMWL_dataFmtConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Device map %u : Data format Configuration failed with error code %d\n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("Device map %u : Data format Configuration success\n\n", deviceMap);
    }

    /* low power configuration */
    retVal = MMWL_lowPowerConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Device map %u : Low Power Configuration failed with error %d \n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("Device map %u : Low Power Configuration success\n\n", deviceMap);
    }
    
    /* APLL Synth BW configuration */
    retVal = MMWL_ApllSynthBwConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Device map %u : APLL Synth BW Configuration failed with error %d \n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("Device map %u : APLL Synth BW Configuration success\n\n", deviceMap);
    }


	if (rlDevGlobalCfgArgs.LinkAdvChirpTest == TRUE)
	{
		/* Misc control configuration for RadarSS */
		/* This API enables the Advanced chirp and per chirp phase shifter features */
		retVal = MMWL_setMiscConfig(deviceMap);
		if (retVal != RL_RET_CODE_OK)
		{
			printf("Device map %u : Misc control configuration failed with error code %d \n\n",
				deviceMap, retVal);
			return -1;
		}
		else
		{
			printf("Device map %u : Misc control configuration success\n\n", deviceMap);
		}
	}
	
    return retVal;
}

/** @fn int MMWL_rfInit(unsigned char deviceMap)
*
*   @brief RFinit API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   RFinit API.
*/
int MMWL_rfInit(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK, timeOutCnt = 0;
	mmwl_bRfInitComp = mmwl_bRfInitComp & (~deviceMap);

	if (rlDevGlobalCfgArgs.CalibEnable == TRUE)
	{
		rlRfInitCalConf_t rfCalibCfgArgs = { 0 };

		/* Calibration store */
		if (rlDevGlobalCfgArgs.CalibStoreRestore == 1)
		{
			/* Enable only required boot-time calibrations, by default all are enabled in the device */
			rfCalibCfgArgs.calibEnMask = 0x1FF0;
		}
		/* Calibration restore */
		else
		{
			/* Disable all the boot-time calibrations, by default all are enabled in the device */
			rfCalibCfgArgs.calibEnMask = 0x0;
		}
		/* RF Init Calibration Configuration */
		retVal = CALL_API(RF_INIT_CALIB_CONFIG_IND, deviceMap, &rfCalibCfgArgs, 0);		
		if (retVal != RL_RET_CODE_OK)
		{
			printf("Device map %u : RF Init Calibration Configuration failed with error %d \n\n",
				deviceMap, retVal);
			return -1;
		}
		else
		{
			printf("Device map %u : RF Init Calibration Configuration success \n\n", deviceMap);
		}

		/* Calibration restore */
		if (rlDevGlobalCfgArgs.CalibStoreRestore == 0)
		{
			for (int devId = 0; devId < 4; devId++)
			{
				if ((deviceMap & (1 << devId)) != 0)
				{
					unsigned char devMap = createDevMapFromDevId(devId);
                    
                    /* Load Phase shifter Calibration Data from a file */
                    retVal = MMWL_LoadPhShiftCalibDataFromFile(devMap);
                    if (retVal != RL_RET_CODE_OK)
                    {
                        printf("Device map %u : Load Phase shifter Calibration Data from a file failed with error %d \n\n",
                            devMap, retVal);
                        return -1;
                    }
                    else
                    {
                        printf("Device map %u : Load Phase shifter Calibration Data from a file success \n\n", devMap);
                    }
        
                    /* Phase shifter Calibration Data Restore Configuration */
					retVal = CALL_API(RF_PH_SHIFT_CALIB_DATA_RESTORE_IND, devMap, &phShiftCalibData, 0);
                    if (retVal != RL_RET_CODE_OK)
                    {
                        printf("Device map %u : Phase shifter Calibration Data Restore Configuration failed with error %d \n\n",
                            devMap, retVal);
                        return -1;
                    }
                    else
                    {
                        printf("Device map %u : Phase shifter Calibration Data Restore Configuration success \n\n", devMap);
                    }

					/* Load Calibration Data from a file */
					retVal = MMWL_LoadCalibDataFromFile(devMap);
					if (retVal != RL_RET_CODE_OK)
					{
						printf("Device map %u : Load Calibration Data from a file failed with error %d \n\n",
							devMap, retVal);
						return -1;
					}
					else
					{
						printf("Device map %u : Load Calibration Data from a file success \n\n", devMap);
					}

					/* Calibration Data Restore Configuration */
					retVal = CALL_API(RF_CALIB_DATA_RESTORE_IND, devMap, &calibData, 0);
					if (retVal != RL_RET_CODE_OK)
					{
						printf("Device map %u : Calibration Data Restore Configuration failed with error %d \n\n",
							devMap, retVal);
						return -1;
					}
					else
					{
						printf("Device map %u : Calibration Data Restore Configuration success \n\n", devMap);
						while ((mmwl_bRfInitComp & devMap) != devMap)
						{
							osiSleep(1); /*Sleep 1 msec*/
							timeOutCnt++;
							if (timeOutCnt > MMWL_API_RF_INIT_TIMEOUT)
							{
								retVal = RL_RET_CODE_RESP_TIMEOUT;
								break;
							}
						}
						mmwl_bRfInitComp = mmwl_bRfInitComp & (~devMap);
					}
				}
			}
		}
	}
    /* Run boot time calibrations */
    retVal = CALL_API(API_TYPE_B | RF_INIT_IND, deviceMap, NULL, 0);
    while ((mmwl_bRfInitComp & deviceMap) != deviceMap)
    {
        osiSleep(1); /*Sleep 1 msec*/
        timeOutCnt++;
        if (timeOutCnt > MMWL_API_RF_INIT_TIMEOUT)
        {
            retVal = RL_RET_CODE_RESP_TIMEOUT;
            break;
        }
    }
    mmwl_bRfInitComp = mmwl_bRfInitComp & (~deviceMap);
	if (rlDevGlobalCfgArgs.CalibEnable == TRUE)
	{
		/* Calibration Store */
		if (rlDevGlobalCfgArgs.CalibStoreRestore == 1)
		{
			for (int devId = 0; devId < 4; devId++)
			{
				if ((deviceMap & (1 << devId)) != 0)
				{
					unsigned char devMap = createDevMapFromDevId(devId);
					/* If all the calibration is done successfully as per above Async-event status,
					   now get the calibration data from the device */
					   /* Calibration Data Store Configuration */
					retVal = CALL_API(RF_CALIB_DATA_STORE_IND, devMap, &calibData, 0);
					if (retVal != RL_RET_CODE_OK)
					{
						printf("Device map %u : Calibration Data Store Configuration failed with error %d \n\n",
							devMap, retVal);
						return -1;
					}
					else
					{
						printf("Device map %u : Calibration Data Store Configuration success \n\n", devMap);
					}
                    
                    /* Phase shifter Calibration Data Store Configuration */
                    retVal = CALL_API(RF_PH_SHIFT_CALIB_DATA_STORE_IND, devMap, &phShiftCalibData, 0);
                    if (retVal != RL_RET_CODE_OK)
                    {
                        printf("Device map %u : Phase shifter Calibration Data Store Configuration failed with error %d \n\n",
                            deviceMap, retVal);
                        return -1;
                    }
                    else
                    {
                        printf("Device map %u : Phase shifter Calibration Data Store Configuration success \n\n", deviceMap);
                    }

					/* Save Calibration Data to a file */
					retVal = MMWL_saveCalibDataToFile(devMap);
					if (retVal != RL_RET_CODE_OK)
					{
						printf("Device map %u : Save Calibration Data to a file failed with error %d \n\n",
							devMap, retVal);
						return -1;
					}
					else
					{
						printf("Device map %u : Save Calibration Data to a file success \n\n", devMap);
					}
                    
                    /* Save Phase shifter Calibration Data to a file */
                    retVal = MMWL_savePhShiftCalibDataToFile(devMap);
                    if (retVal != RL_RET_CODE_OK)
                    {
                        printf("Device map %u : Save Phase shifter Calibration Data to a file failed with error %d \n\n",
                            devMap, retVal);
                        return -1;
                    }
                    else
                    {
                        printf("Device map %u : Save Phase shifter Calibration Data to a file success \n\n", devMap);
                    }
				}
			}
		}
	}
    return retVal;
}

/** @fn int MMWL_saveCalibDataToFile(unsigned char deviceMap)
*
*   @brief Save Calibration Data to a file.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Save Calibration Data to a file.
*/
int MMWL_saveCalibDataToFile(unsigned char deviceMap)
{
	int retVal = RL_RET_CODE_OK;
	int i,j;
	int index = 0;
	char CalibdataBuff[2500] = { 0 };

	if (deviceMap == 1)
		CalibrationDataPtr = _fsopen("CalibrationData_0.txt", "wt", _SH_DENYWR);
	else if (deviceMap == 2)
		CalibrationDataPtr = _fsopen("CalibrationData_1.txt", "wt", _SH_DENYWR);
	else if (deviceMap == 4)
		CalibrationDataPtr = _fsopen("CalibrationData_2.txt", "wt", _SH_DENYWR);
	else if (deviceMap == 8)
		CalibrationDataPtr = _fsopen("CalibrationData_3.txt", "wt", _SH_DENYWR);

	if (CalibrationDataPtr == NULL)
	{
		printf("Device map %u : Error opening CalibrationData file\n\n", deviceMap);
		return -1;

	}
	/* Copy data from all the 3 chunks */
	for (i = 0; i < 3; i++)
	{
		sprintf(CalibdataBuff + strlen(CalibdataBuff), "0x%04x\n", calibData.calibChunk[i].numOfChunk);
		sprintf(CalibdataBuff + strlen(CalibdataBuff), "0x%04x\n", calibData.calibChunk[i].chunkId);
		/* Store 224 bytes of data in each chunk in terms of 2 bytes per line */
		for (j = 0; j < 224; j+=2)
		{
			sprintf(CalibdataBuff + strlen(CalibdataBuff), "0x%02x%02x\n", calibData.calibChunk[i].calData[j+1], calibData.calibChunk[i].calData[j]);
		}
	}

	fwrite(CalibdataBuff, sizeof(char), strlen(CalibdataBuff), CalibrationDataPtr);
	fflush(CalibrationDataPtr);

	if (CalibrationDataPtr != NULL)
		fclose(CalibrationDataPtr);

	return retVal;
}

/** @fn int MMWL_savePhShiftCalibDataToFile(unsigned char deviceMap)
*
*   @brief Save Phase shifter Calibration Data to a file.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Save Phase shifter Calibration Data to a file.
*/
int MMWL_savePhShiftCalibDataToFile(unsigned char deviceMap)
{
	int retVal = RL_RET_CODE_OK;
	int i,j;
	int index = 0;
	char PhShiftCalibdataBuff[2500] = { 0 };

	if(deviceMap == 1)
		PhShiftCalibrationDataPtr = _fsopen("PhShiftCalibrationData_0.txt", "wt", _SH_DENYWR);
	else if(deviceMap == 2)
		PhShiftCalibrationDataPtr = _fsopen("PhShiftCalibrationData_1.txt", "wt", _SH_DENYWR);
	else if(deviceMap == 4)
		PhShiftCalibrationDataPtr = _fsopen("PhShiftCalibrationData_2.txt", "wt", _SH_DENYWR);
	else if(deviceMap == 8)
		PhShiftCalibrationDataPtr = _fsopen("PhShiftCalibrationData_3.txt", "wt", _SH_DENYWR);

	/* Copy data from all the 3 chunks */
	for (i = 0; i < 3; i++)
	{
		sprintf(PhShiftCalibdataBuff + strlen(PhShiftCalibdataBuff), "0x%02x\n", \
                phShiftCalibData.PhShiftcalibChunk[i].txIndex);
		sprintf(PhShiftCalibdataBuff + strlen(PhShiftCalibdataBuff), "0x%02x\n", \
                phShiftCalibData.PhShiftcalibChunk[i].calibApply);
		/* Store 128 bytes of data in each chunk in terms of 1 byte per line */
		for (j = 0; j < 128; j++)
		{
			sprintf(PhShiftCalibdataBuff + strlen(PhShiftCalibdataBuff), "0x%02x\n", \
                    phShiftCalibData.PhShiftcalibChunk[i].observedPhShiftData[j]);
		}
	}

	fwrite(PhShiftCalibdataBuff, sizeof(char), strlen(PhShiftCalibdataBuff), PhShiftCalibrationDataPtr);
	fflush(PhShiftCalibrationDataPtr);

	if (PhShiftCalibrationDataPtr != NULL)
		fclose(PhShiftCalibrationDataPtr);

	return retVal;
}

/** @fn int MMWL_LoadCalibDataFromFile(unsigned char deviceMap)
*
*   @brief Load Calibration Data from a file.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Load Calibration Data from a file.
*/
int MMWL_LoadCalibDataFromFile(unsigned char deviceMap)
{
	int retVal = RL_RET_CODE_OK;
	int index = 0;
	char CalibdataBuff[2500] = { 0 };
	char *s, buff[8], val[100];
	int i = 0;
	char readNumChunks = 0, readChunkId = 0;
	
	if(deviceMap == 1)
		CalibrationDataPtr = _fsopen("CalibrationData_0.txt", "rt", _SH_DENYRD);
	else if(deviceMap == 2)
		CalibrationDataPtr = _fsopen("CalibrationData_1.txt", "rt", _SH_DENYRD);
	else if(deviceMap == 4)
		CalibrationDataPtr = _fsopen("CalibrationData_2.txt", "rt", _SH_DENYRD);
	else if(deviceMap == 8)
		CalibrationDataPtr = _fsopen("CalibrationData_3.txt", "rt", _SH_DENYRD);

	if (CalibrationDataPtr == NULL)
	{
		printf("Device map %u : CalibrationData file does not exist or Error opening the file\n\n", deviceMap);
		return -1;
	}

	/*seek the pointer to starting of the file */
	fseek(CalibrationDataPtr, 0, SEEK_SET);

	/*parse the parameters by reading each line of the calib data file*/
	while ((readNumChunks != 3) && (readChunkId != 3))
	{
		unsigned char readDataChunks = 0;
		if ((s = fgets(buff, sizeof buff, CalibrationDataPtr)) != NULL)
		{
			/* Parse value from line */
			s = strtok(buff, "\n");
			if (s == NULL)
			{
				continue;
			}
			else
			{
				strncpy(val, s, STRINGLEN);
				calibData.calibChunk[i].numOfChunk = (rlUInt16_t)strtol(val, NULL, 0);
				readNumChunks++;
			}
		}
		if ((s = fgets(buff, sizeof buff, CalibrationDataPtr)) != NULL)
		{
			/* Parse value from line */
			s = strtok(buff, "\n");
			if (s == NULL)
			{
				continue;
			}
			else
			{
				strncpy(val, s, STRINGLEN);
				calibData.calibChunk[i].chunkId = (rlUInt16_t)strtol(val, NULL, 0);
				readChunkId++;
			}
		}
		while ((readDataChunks != 224) && ((s = fgets(buff, sizeof buff, CalibrationDataPtr)) != NULL))
		{
			/* Parse value from line */
			const char* temp = &buff[0];
			char byte1[3];
			char byte2[3];

			strncpy(byte1, temp +4, 2);
			byte1[2] = '\0';
			if (byte1 == NULL)
			{
				continue;
			}
			else
			{
				calibData.calibChunk[i].calData[readDataChunks] = (rlUInt8_t)strtol(byte1, NULL, 16);
				readDataChunks++;
			}

			strncpy(byte2, temp + 2, 2);
			byte2[2] = '\0';
			if (byte2 == NULL)
			{
				continue;
			}
			else
			{
				calibData.calibChunk[i].calData[readDataChunks] = (rlUInt8_t)strtol(byte2, NULL, 16);
				readDataChunks++;
			}
		}
		i++;
	}

	fflush(CalibrationDataPtr);

	if (CalibrationDataPtr != NULL)
		fclose(CalibrationDataPtr);

	return retVal;
}

/** @fn int MMWL_LoadPhShiftCalibDataFromFile(unsigned char deviceMap)
*
*   @brief Load Phase shifter Calibration Data from a file.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Load Phase shifter Calibration Data from a file.
*/
int MMWL_LoadPhShiftCalibDataFromFile(unsigned char deviceMap)
{
	int retVal = RL_RET_CODE_OK;
	int index = 0;
	char PhShiftCalibdataBuff[2500] = { 0 };
	char *s, buff[8], val[100];
	int i = 0;
	char readNumChunks = 0, readChunkId = 0;
    unsigned char readDataChunks;
	
	if(deviceMap == 1)
		PhShiftCalibrationDataPtr = _fsopen("PhShiftCalibrationData_0.txt", "rt", _SH_DENYRD);
	else if(deviceMap == 2)
		PhShiftCalibrationDataPtr = _fsopen("PhShiftCalibrationData_1.txt", "rt", _SH_DENYRD);
	else if(deviceMap == 4)
		PhShiftCalibrationDataPtr = _fsopen("PhShiftCalibrationData_2.txt", "rt", _SH_DENYRD);
	else if(deviceMap == 8)
		PhShiftCalibrationDataPtr = _fsopen("PhShiftCalibrationData_3.txt", "rt", _SH_DENYRD);

	if (PhShiftCalibrationDataPtr == NULL)
	{
		printf("PhShiftCalibrationData.txt does not exist or Error opening the file\n\n");
		return -1;
	}

	/*seek the pointer to starting of the file */
	fseek(PhShiftCalibrationDataPtr, 0, SEEK_SET);

	/*parse the parameters by reading each line of the phase shift calib data file*/
	while ((readNumChunks != 3) && (readChunkId != 3))
	{
		readDataChunks = 0;
		if ((s = fgets(buff, sizeof buff, PhShiftCalibrationDataPtr)) != NULL)
		{
			/* Parse value from line */
			s = strtok(buff, "\n");
			if (s == NULL)
			{
				continue;
			}
			else
			{
				strncpy(val, s, STRINGLEN);
				phShiftCalibData.PhShiftcalibChunk[i].txIndex = (rlUInt8_t)strtol(val, NULL, 0);
				readNumChunks++;
			}
		}
		if ((s = fgets(buff, sizeof buff, PhShiftCalibrationDataPtr)) != NULL)
		{
			/* Parse value from line */
			s = strtok(buff, "\n");
			if (s == NULL)
			{
				continue;
			}
			else
			{
				strncpy(val, s, STRINGLEN);
				phShiftCalibData.PhShiftcalibChunk[i].calibApply = (rlUInt8_t)strtol(val, NULL, 0);
				readChunkId++;
			}
		}
		while ((readDataChunks != 128) && ((s = fgets(buff, sizeof buff, PhShiftCalibrationDataPtr)) != NULL))
		{
			/* Parse value from line */
			const char* temp = &buff[0];
			char byte1[5];

			strncpy(byte1, temp, 4);
			byte1[4] = '\0';
			if (byte1 == NULL)
			{
				continue;
			}
			else
			{
				phShiftCalibData.PhShiftcalibChunk[i].observedPhShiftData[readDataChunks] = (rlUInt8_t)strtol(byte1, NULL, 16);
				readDataChunks++;
			}
		}
        phShiftCalibData.PhShiftcalibChunk[i].reserved = 0U;
		i++;
	}

	fflush(PhShiftCalibrationDataPtr);

	if (PhShiftCalibrationDataPtr != NULL)
		fclose(PhShiftCalibrationDataPtr);

	return retVal;
}

/** @fn int MMWL_progFiltConfig(unsigned char deviceMap)
*
*   @brief Programmable filter configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Programmable filter configuration API.
*/
int MMWL_progFiltConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    rlRfProgFiltConf_t progFiltCnfgArgs = { 0 };

    /*read progFiltCnfgArgs from config file*/
    MMWL_readProgFiltConfig(&progFiltCnfgArgs);

    printf("Device map %u : Calling rlRfSetProgFiltConfig with \ncoeffStartIdx[%d]\nprogFiltLen[%d] GHz\nprogFiltFreqShift[%d] MHz/uS \n\n",
        deviceMap, progFiltCnfgArgs.coeffStartIdx, progFiltCnfgArgs.progFiltLen, progFiltCnfgArgs.progFiltFreqShift);
	retVal = CALL_API(RF_SET_PROG_FILT_CONFIG_IND, deviceMap, &progFiltCnfgArgs, 0);
    return retVal;
}

/** @fn int MMWL_progFiltCoeffRam(unsigned char deviceMap)
*
*   @brief Programmable Filter coefficient RAM configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Programmable Filter coefficient RAM configuration API.
*/
int MMWL_progFiltCoeffRam(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    rlRfProgFiltCoeff_t progFiltCoeffCnfgArgs = { 0 };
    progFiltCoeffCnfgArgs.coeffArray[0] = -876,
    progFiltCoeffCnfgArgs.coeffArray[1] = -272,
    progFiltCoeffCnfgArgs.coeffArray[2] = 1826,
    progFiltCoeffCnfgArgs.coeffArray[3] = -395,
    progFiltCoeffCnfgArgs.coeffArray[4] = -3672,
    progFiltCoeffCnfgArgs.coeffArray[5] = 3336,
    progFiltCoeffCnfgArgs.coeffArray[6] = 15976,
    progFiltCoeffCnfgArgs.coeffArray[7] = 15976,
    progFiltCoeffCnfgArgs.coeffArray[8] = 3336,
    progFiltCoeffCnfgArgs.coeffArray[9] = -3672,
    progFiltCoeffCnfgArgs.coeffArray[10] = -395,
    progFiltCoeffCnfgArgs.coeffArray[11] = 1826,
    progFiltCoeffCnfgArgs.coeffArray[12] = -272,
    progFiltCoeffCnfgArgs.coeffArray[13] = -876,

    printf("Device map %u : Calling rlRfSetProgFiltCoeffRam with \ncoeffArray0[%d]\ncoeffArray1[%d] GHz\ncoeffArray2[%d] MHz/uS \n\n",
    deviceMap, progFiltCoeffCnfgArgs.coeffArray[0], progFiltCoeffCnfgArgs.coeffArray[1], progFiltCoeffCnfgArgs.coeffArray[2]);
	retVal = CALL_API(RF_SET_PROG_FILT_COEFF_RAM_IND, deviceMap, &progFiltCoeffCnfgArgs, 0);
    return retVal;
}

/** @fn int MMWL_profileConfig(unsigned char deviceMap)
*
*   @brief Profile configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Profile configuration API.
*/
int MMWL_profileConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK, i;
	
	if (rlDevGlobalCfgArgs.LinkAdvChirpTest == FALSE)
	{
		/*read profileCfgArgs from config file*/
		MMWL_readProfileConfig(&profileCfgArgs[0], 1U);

		printf("Device map %u : Calling rlSetProfileConfig with \nProfileId[%d]\nStart Frequency[%f] GHz\nRamp Slope[%f] MHz/uS \n\n",
			deviceMap, profileCfgArgs[0].profileId, (float)((profileCfgArgs[0].startFreqConst * 53.6441803) / (1000 * 1000 * 1000)),
			(float)(profileCfgArgs[0].freqSlopeConst * 48.2797623) / 1000.0);
		/* with this API we can configure 2 profiles (max 4 profiles) at a time */
		retVal = CALL_API(API_TYPE_C | SET_PROFILE_CONFIG_IND, deviceMap, &profileCfgArgs[0U], 1U);
	}
	else
	{
		/*read ProfileCfgArgs_AdvChirp from config file*/
		MMWL_readProfileConfig(&ProfileCfgArgs_AdvChirp[0], 4U);

		for (i = 0; i < 4; i++)
		{
			printf("Device map %u : Calling rlSetProfileConfig with \nProfileId[%d]\nStart Frequency[%f] GHz\nRamp Slope[%f] MHz/uS \n\n",
				deviceMap, ProfileCfgArgs_AdvChirp[i].profileId, (float)((ProfileCfgArgs_AdvChirp[i].startFreqConst * 53.6441803) / (1000 * 1000 * 1000)),
				(float)(ProfileCfgArgs_AdvChirp[i].freqSlopeConst * 48.2797623) / 1000.0);
		}

		/* configuring 4 profiles at a time */
		retVal = CALL_API(API_TYPE_C | SET_PROFILE_CONFIG_IND, deviceMap, &ProfileCfgArgs_AdvChirp[0U], 4U);
	}
    return retVal;
}

/** @fn int MMWL_chirpConfig(unsigned char deviceMap)
*
*   @brief Chirp configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Chirp configuration API.
*/
int MMWL_chirpConfig(unsigned char deviceMap)
{
	int retVal = RL_RET_CODE_OK;
    rlChirpCfg_t setChirpCfgArgs[2] = {0};

    /*read chirpCfgArgs from config file*/
    MMWL_readChirpConfig(&setChirpCfgArgs[0], 2);

    printf("Device map %u : Calling rlSetChirpConfig with \nProfileId[%d]\nStart Idx[%d]\nEnd Idx[%d] \n\n",
                deviceMap, setChirpCfgArgs[0].profileId, setChirpCfgArgs[0].chirpStartIdx,
                setChirpCfgArgs[0].chirpEndIdx);

	printf("Device map %u : Calling rlSetChirpConfig with \nProfileId[%d]\nStart Idx[%d]\nEnd Idx[%d] \n\n",
		deviceMap, setChirpCfgArgs[1].profileId, setChirpCfgArgs[1].chirpStartIdx,
		setChirpCfgArgs[1].chirpEndIdx);

    /* With this API we can configure max 512 chirp in one call */
	retVal = CALL_API(API_TYPE_C | SET_CHIRP_CONFIG_IND, deviceMap, &setChirpCfgArgs[0U], 2U);
    return retVal;
}

/* Ping LUT buffer offset (starts at 0 bytes and ends at 52 bytes) */
rlAdvChirpDynLUTAddrOffCfg_t advChirpDynLUTOffsetCfg1 = 
{
	.addrMaskEn = 0x3FF, /* enable for all LUT parameters */
	.lutAddressOffset[0U] = 0,
	.lutAddressOffset[1U] = 4,
	.lutAddressOffset[2U] = 12,
	.lutAddressOffset[3U] = 16,
	.lutAddressOffset[4U] = 24,
	.lutAddressOffset[5U] = 32,
	.lutAddressOffset[6U] = 36,
	.lutAddressOffset[7U] = 40,
	.lutAddressOffset[8U] = 44,
	.lutAddressOffset[9U] = 48
};
/* Pong LUT buffer offset (starts at 100 bytes and ends at 152 bytes) */
rlAdvChirpDynLUTAddrOffCfg_t advChirpDynLUTOffsetCfg2 = 
{
	.addrMaskEn = 0x3FF, /* enable for all LUT parameters */
	.lutAddressOffset[0U] = 100,
	.lutAddressOffset[1U] = 104,
	.lutAddressOffset[2U] = 112,
	.lutAddressOffset[3U] = 116,
	.lutAddressOffset[4U] = 124,
	.lutAddressOffset[5U] = 132,
	.lutAddressOffset[6U] = 136,
	.lutAddressOffset[7U] = 140,
	.lutAddressOffset[8U] = 144,
	.lutAddressOffset[9U] = 148
};

/** @fn int MMWL_advChirpConfigAll(unsigned char deviceMap)
*
*   @brief Advanced chirp configuration API.
*
*   @param[in] deviceMap - Device Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Advanced chirp configuration API.
*/
int MMWL_advChirpConfigAll(unsigned char deviceMap)
{
	int retVal = RL_RET_CODE_OK;
	int bufIdx = 0;

	rlAdvChirpCfg_t AdvChirpCfgArgs = { 0 };
	rlFillLUTParams_t rlFillLUTParamsArgs = { 0 };
	rlAdvChirpLUTProfileCfg_t AdvChirpLUTProfileCfgArgs = { 0 };
	/* profile ID LUT dither local buffer */
	rlUInt8_t ProfileCfgData[4] = { 0 };
	rlAdvChirpLUTStartFreqCfg_t AdvChirpLUTStartFreqCfgArgs = { 0 };
	/* Frequency start LUT dither local buffer */
	rlInt16_t StartFreqData[4] = { 0 }; /* Change this based on the chirp param size*/
	rlAdvChirpLUTFreqSlopeCfg_t AdvChirpLUTFreqSlopeCfgArgs = { 0 };
	/* Frequency slope LUT dither local buffer */
	rlInt8_t FreqSlopeData[4] = { 0 };
	rlAdvChirpLUTIdleTimeCfg_t AdvChirpLUTIdleTimeCfgArgs = { 0 };
	/* Idle Time LUT dither local buffer */
	rlInt16_t IdleTimeData[4] = { 0 }; /* Change this based on the chirp param size*/
	rlAdvChirpLUTADCTimeCfg_t AdvChirpLUTADCTimeCfgArgs = { 0 };
	/* ADC start Time LUT dither local buffer */
	rlInt16_t ADCStartTimeData[4] = { 0 }; /* Change this based on the chirp param size*/
	rlAdvChirpLUTTxEnCfg_t AdvChirpLUTTxEnCfgArgs = { 0 };
	/* Tx Enable LUT dither local buffer */
	rlUInt8_t TxEnCfgData[4] = { 0 };
	rlAdvChirpLUTBpmEnCfg_t AdvChirpLUTBpmEnCfgArgs = { 0 };
	/* BPM Enable LUT dither local buffer */
	rlUInt8_t BpmEnCfgData[4] = { 0 };
	rlAdvChirpLUTTx0PhShiftCfg_t AdvChirpLUTTx0PhShiftCfgArgs = { 0 };
	/* Tx0 Phase shifter LUT dither local buffer */
	rlInt8_t Tx0PhShiftData[4] = { 0 };
	rlAdvChirpLUTTx1PhShiftCfg_t AdvChirpLUTTx1PhShiftCfgArgs = { 0 };
	/* Tx1 Phase shifter LUT dither local buffer */
	rlInt8_t Tx1PhShiftData[4] = { 0 };
	rlAdvChirpLUTTx2PhShiftCfg_t AdvChirpLUTTx2PhShiftCfgArgs = { 0 };
	/* Tx2 Phase shifter LUT dither local buffer */
	rlInt8_t Tx2PhShiftData[4] = { 0 };
	rlAdvChirpLUTCfg_t rlAdvChirpLUTCfgArgs = { 0 };
	
	/* Configuring Profile (Param Index = 0) */
	/* Fixed delta dither is not supported for profile parameter */
	/* Configuring 4 unique profile index (0,1,2,3) in the generic SW LUT - LUT Reset period (4) */
	/* The new profile index is picked every chirp (update period = 1) and it will reset every 4 chirps */
	/*   Chirp    Profile   
	       0         0
		   1         1
		   2         2
		   3         3
		   4         0     and so on */
	/* LUT start address offset for the profile chirp parameter is made 0 */
	/* AdvChirpLUTData[0] is the start address offset (Offset = 0), Each data parameter is 4 bits */
	/* Number of unique LUT dither parameters (4) */
	AdvChirpCfgArgs.chirpParamIdx = 0;
	AdvChirpCfgArgs.lutResetPeriod = 4;
	AdvChirpCfgArgs.lutParamUpdatePeriod = 1; /* Make it zero if LUT index zero (PF0) to be applied for all the chirps */
	AdvChirpCfgArgs.lutPatternAddressOffset = 0;
	AdvChirpCfgArgs.numOfPatterns = 4; /* Uses 4 different profiles. Make it one, if it requires only a single profile */

	retVal = MMWL_advChirpConfig(deviceMap, &AdvChirpCfgArgs);
	if (retVal != RL_RET_CODE_OK)
	{
		printf("Device map %u : MMWL_advChirpConfig failed with error code %d \n\n", deviceMap, retVal);
		return retVal;
	}

	/* read the input param data */
	MMWL_readAdvChirpProfileConfig(&AdvChirpLUTProfileCfgArgs);
	printf("Device map %u : Saving advChirpLUTProfileConfig with \nLUTAddrOff[%d]\nProfileCfgData[0]=[%d]\nProfileCfgData[1]=[%d]\nProfileCfgData[2]=[%d]\nProfileCfgData[3]=[%d] \n\n",
		deviceMap, AdvChirpLUTProfileCfgArgs.LUTAddrOff, AdvChirpLUTProfileCfgArgs.ProfileCfgData[0], AdvChirpLUTProfileCfgArgs.ProfileCfgData[1],
		AdvChirpLUTProfileCfgArgs.ProfileCfgData[2], AdvChirpLUTProfileCfgArgs.ProfileCfgData[3]);
	/* copy the input param data from mmwaveconfig.txt to a local buffer */
	memcpy(&ProfileCfgData[0], &AdvChirpLUTProfileCfgArgs.ProfileCfgData[0], 4 * sizeof(rlUInt8_t));

	/* fill up the Chirp LUT buffer which is used later for rlSetAdvChirpLUTConfig API */
	rlFillLUTParamsArgs.chirpParamIndex = RL_LUT_CHIRP_PROFILE_VAR;
	rlFillLUTParamsArgs.chirpParamSize = AdvChirpCfgArgs.lutChirpParamSize;
	rlFillLUTParamsArgs.inputSize = AdvChirpCfgArgs.numOfPatterns;
	rlFillLUTParamsArgs.lutGlobalOffset = lutOffsetInNBytes;
	retVal = rlDevSetFillLUTBuff(&rlFillLUTParamsArgs, &ProfileCfgData[0], &AdvChirpLUTData[lutOffsetInNBytes], &lutOffsetInNBytes);

	/* Start Frequency (Param Index = 1) */
	/* Fixed start frequency delta dither (-20000) LSB's = (-20000 * 3.6 GHz) / 2^26 = -0.0010728 GHz, the delta dither
	   will accumulate every single chirp (update period = 1) and it will reset every 8 chirps */
	/* Configuring 4 unique start frequency LUT dither (-0.000001,0.000000,0.000001,-0.000001) GHz from mmwaveconfig.txt */
	/* The new start frequency LUT dither is picked every chirp (update period = 1) and it will reset every 4 chirps */
	/*   Chirp    Start Freq (from Profile) + LUT dither + Fixed delta dither
		   0         77 GHz + -0.000001 GHz + 0
		   1         77 GHz + 0.000000 GHz + -0.0010728 GHz
		   2         77 GHz + 0.000001 GHz + -0.0010728*2 GHz
		   3         77 GHz + -0.000001 GHz + -0.0010728*3 GHz
		   4         77 GHz + -0.000001 GHz + -0.0010728*4 GHz  (LUT reset period = 4)
		   5         77 GHz + 0.000000 GHz + -0.0010728*5 GHz
		   6         77 GHz + 0.000001 GHz + -0.0010728*6 GHz
		   7         77 GHz + -0.000001 GHz + -0.0010728*7 GHz
		   8         77 GHz + -0.000001 GHz + 0                 (Delta dither reset period = 8) and so on */
	/* LUT start address offset for the start frequency chirp parameter is made 4 */
	/* AdvChirpLUTData[4] is the start address offset (Offset = 4), Each data parameter is 2 bytes (used lutChirpParamSize = 1 and lutChirpParamScale = 0) */
	/* Number of unique LUT dither parameters (4) */
	memset((void *)&AdvChirpCfgArgs, 0, sizeof(rlAdvChirpCfg_t));
	AdvChirpCfgArgs.chirpParamIdx = 1;
	AdvChirpCfgArgs.resetMode = 0; /* reset at the end of frame */
	AdvChirpCfgArgs.deltaResetPeriod = 8;
	AdvChirpCfgArgs.deltaParamUpdatePeriod = 1; /* Update start frequency for each chirp */
	AdvChirpCfgArgs.sf0ChirpParamDelta = -20000; /* This value corresponds to (-20000 * 3.6 GHz) / 2^26 */
	AdvChirpCfgArgs.sf1ChirpParamDelta = 0; /* In legacy frame - SF1, SF2 and SF3 are not used */
	AdvChirpCfgArgs.lutResetPeriod = 4;
	AdvChirpCfgArgs.lutParamUpdatePeriod = 1;
	AdvChirpCfgArgs.lutPatternAddressOffset = 4;
	AdvChirpCfgArgs.numOfPatterns = 4;
	AdvChirpCfgArgs.lutChirpParamSize = 1;
	AdvChirpCfgArgs.lutChirpParamScale = 0;

	retVal = MMWL_advChirpConfig(deviceMap, &AdvChirpCfgArgs);
	if (retVal != RL_RET_CODE_OK)
	{
		printf("Device map %u : MMWL_advChirpConfig failed with error code %d \n\n", deviceMap, retVal);
		return retVal;
	}

	/* read the input param data */
	MMWL_readAdvChirpStartFreqConfig(&AdvChirpLUTStartFreqCfgArgs);
	printf("Device map %u : Saving advChirpLUTStartFreqConfig with \nLUTAddrOff[%d]\nParamSize[%d]\nParamScale[%d]\nStartFreqCfgData[0]=[%.6f] GHz\nStartFreqCfgData[1]=[%.6f] GHz\nStartFreqCfgData[2]=[%.6f] GHz\nStartFreqCfgData[3]=[%.6f] GHz \n\n",
		deviceMap, AdvChirpLUTStartFreqCfgArgs.LUTAddrOff, AdvChirpLUTStartFreqCfgArgs.ParamSize, AdvChirpLUTStartFreqCfgArgs.ParamScale, AdvChirpLUTStartFreqCfgArgs.StartFreqCfgData[0], AdvChirpLUTStartFreqCfgArgs.StartFreqCfgData[1],
		AdvChirpLUTStartFreqCfgArgs.StartFreqCfgData[2], AdvChirpLUTStartFreqCfgArgs.StartFreqCfgData[3]);
	/* copy the input param data from mmwaveconfig.txt to a local buffer */
	/* Dividing the input param data by 1 LSB . 1 LSB = ((3.6 * 10^9)/2^26) * 2^Scale Hz = (3.6 * 2^Scale)/2^26 GHz */
	StartFreqData[0] = (rlInt16_t)(((double)AdvChirpLUTStartFreqCfgArgs.StartFreqCfgData[0] * 67108864.0) / (3.6 * pow(2, AdvChirpLUTStartFreqCfgArgs.ParamScale)));
	StartFreqData[1] = (rlInt16_t)(((double)AdvChirpLUTStartFreqCfgArgs.StartFreqCfgData[1] * 67108864.0) / (3.6 * pow(2, AdvChirpLUTStartFreqCfgArgs.ParamScale)));
	StartFreqData[2] = (rlInt16_t)(((double)AdvChirpLUTStartFreqCfgArgs.StartFreqCfgData[2] * 67108864.0) / (3.6 * pow(2, AdvChirpLUTStartFreqCfgArgs.ParamScale)));
	StartFreqData[3] = (rlInt16_t)(((double)AdvChirpLUTStartFreqCfgArgs.StartFreqCfgData[3] * 67108864.0) / (3.6 * pow(2, AdvChirpLUTStartFreqCfgArgs.ParamScale)));

	/* fill up the Chirp LUT buffer which is used later for rlSetAdvChirpLUTConfig API */
	rlFillLUTParamsArgs.chirpParamIndex = RL_LUT_CHIRP_FREQ_START_VAR;
	rlFillLUTParamsArgs.chirpParamSize = AdvChirpCfgArgs.lutChirpParamSize;
	rlFillLUTParamsArgs.inputSize = AdvChirpCfgArgs.numOfPatterns;
	rlFillLUTParamsArgs.lutGlobalOffset = lutOffsetInNBytes;
	retVal = rlDevSetFillLUTBuff(&rlFillLUTParamsArgs, &(rlInt8_t)StartFreqData[0], &AdvChirpLUTData[lutOffsetInNBytes], &lutOffsetInNBytes);

	/* Frequency Slope (Param Index = 2) */
	/* Fixed slope delta dither (-10) LSB's = (-10 * 3.6 * 900 GHz) / 2^26 = -0.482797 MHz/us, the delta dither
	   will accumulate every single chirp (update period = 1) and it will reset every 4 chirps */
	/* Configuring 4 unique slope LUT dither (-0.050,0.000,-0.050,0.050) MHz/us from mmwaveconfig.txt */
	/* The new slope LUT dither is picked every chirp (update period = 1) and it will reset every 4 chirps */
	/*   Chirp    Slope (from Profile) + LUT dither + Fixed delta dither
		   0         29.982 MHz/us + -0.050 MHz/us + 0
		   1         29.982 MHz/us + 0.000 MHz/us + -0.482797 MHz/us
		   2         29.982 MHz/us + -0.050 MHz/us + -0.482797*2 MHz/us
		   3         29.982 MHz/us + 0.050 MHz/us + -0.482797*3 MHz/us
		   4         29.982 MHz/us + -0.050 MHz/us + 0     (Delta dither reset period = 4 and LUT dither reset period = 4) and so on */
	/* LUT start address offset for the slope chirp parameter is made 12 */
	/* AdvChirpLUTData[12] is the start address offset (Offset = 12), Each data parameter is 1 byte */
	/* Number of unique LUT dither parameters (4) */
	memset((void *)&AdvChirpCfgArgs, 0, sizeof(rlAdvChirpCfg_t));
	AdvChirpCfgArgs.chirpParamIdx = 2;
	AdvChirpCfgArgs.deltaResetPeriod = 4;
	AdvChirpCfgArgs.deltaParamUpdatePeriod = 1;
	AdvChirpCfgArgs.sf0ChirpParamDelta = -10;
	AdvChirpCfgArgs.lutResetPeriod = 4;
	AdvChirpCfgArgs.lutParamUpdatePeriod = 1;
	AdvChirpCfgArgs.lutPatternAddressOffset = 12;
	AdvChirpCfgArgs.numOfPatterns = 4;

	retVal = MMWL_advChirpConfig(deviceMap, &AdvChirpCfgArgs);
	if (retVal != RL_RET_CODE_OK)
	{
		printf("Device map %u : MMWL_advChirpConfig failed with error code %d \n\n", deviceMap, retVal);
		return retVal;
	}

	/* read the input param data */
	MMWL_readAdvChirpFreqSlopeConfig(&AdvChirpLUTFreqSlopeCfgArgs);
	printf("Device map %u : Saving advChirpLUTFreqSlopeConfig with \nLUTAddrOff[%d]\nFreqSlopeCfgData[0]=[%.3f] MHz/us\nFreqSlopeCfgData[1]=[%.3f] MHz/us\nFreqSlopeCfgData[2]=[%.3f] MHz/us\nFreqSlopeCfgData[3]=[%.3f] MHz/us \n\n",
		deviceMap, AdvChirpLUTFreqSlopeCfgArgs.LUTAddrOff, AdvChirpLUTFreqSlopeCfgArgs.FreqSlopeCfgData[0], AdvChirpLUTFreqSlopeCfgArgs.FreqSlopeCfgData[1],
		AdvChirpLUTFreqSlopeCfgArgs.FreqSlopeCfgData[2], AdvChirpLUTFreqSlopeCfgArgs.FreqSlopeCfgData[3]);
	/* copy the input param data from mmwaveconfig.txt to a local buffer */
    /* Dividing the input param data by 1 LSB . 1 LSB = ((3.6 * 10^9)* 900 /2^26) Hz = 48.279 KHz = 0.048279 MHz*/
	FreqSlopeData[0] = (rlInt8_t)((double)AdvChirpLUTFreqSlopeCfgArgs.FreqSlopeCfgData[0] * (67108864.0 / 3240000.0));
	FreqSlopeData[1] = (rlInt8_t)((double)AdvChirpLUTFreqSlopeCfgArgs.FreqSlopeCfgData[1] * (67108864.0 / 3240000.0));
	FreqSlopeData[2] = (rlInt8_t)((double)AdvChirpLUTFreqSlopeCfgArgs.FreqSlopeCfgData[2] * (67108864.0 / 3240000.0));
	FreqSlopeData[3] = (rlInt8_t)((double)AdvChirpLUTFreqSlopeCfgArgs.FreqSlopeCfgData[3] * (67108864.0 / 3240000.0));

	/* fill up the Chirp LUT buffer which is used later for rlSetAdvChirpLUTConfig API */
	rlFillLUTParamsArgs.chirpParamIndex = RL_LUT_CHIRP_FREQ_SLOPE_VAR;
	rlFillLUTParamsArgs.chirpParamSize = AdvChirpCfgArgs.lutChirpParamSize;
	rlFillLUTParamsArgs.inputSize = AdvChirpCfgArgs.numOfPatterns;
	rlFillLUTParamsArgs.lutGlobalOffset = lutOffsetInNBytes;
	retVal = rlDevSetFillLUTBuff(&rlFillLUTParamsArgs, &FreqSlopeData[0], &AdvChirpLUTData[lutOffsetInNBytes], &lutOffsetInNBytes);

	/* Idle time (Param Index = 3) */
	/* Fixed idle time delta dither (2) LSB's = (2 * 10 ns) = 0.02 us, the delta dither
	   will accumulate every single chirp (update period = 1) and it will reset every 4 chirps */
	/* Configuring 4 unique idle time LUT dither (0.01,0.02,0.00,0.01) us from mmwaveconfig.txt */
	/* The new idle time LUT dither is picked every chirp (update period = 1) and it will reset every 4 chirps */
	/*   Chirp    Idle time (from Profile) + LUT dither + Fixed delta dither
		   0         100 us + 0.01 us + 0
		   1         100 us + 0.02 us + 0.02 us
		   2         100 us + 0.00 us + 0.02*2 us
		   3         100 us + 0.01 us + 0.02*3 us
		   4         100 us + 0.01 us + 0    (Delta dither reset period = 4 and LUT dither reset period = 4) and so on */
	/* LUT start address offset for the idle time chirp parameter is made 16 */
	/* AdvChirpLUTData[16] is the start address offset (Offset = 16), Each data parameter is 2 bytes (used lutChirpParamSize = 0 and lutChirpParamScale = 0) */
	/* Number of unique LUT dither parameters (4) */
	memset((void *)&AdvChirpCfgArgs, 0, sizeof(rlAdvChirpCfg_t));
	AdvChirpCfgArgs.chirpParamIdx = 3;
	AdvChirpCfgArgs.deltaResetPeriod = 4;
	AdvChirpCfgArgs.deltaParamUpdatePeriod = 1;
	AdvChirpCfgArgs.sf0ChirpParamDelta = 2;
	AdvChirpCfgArgs.lutResetPeriod = 4;
	AdvChirpCfgArgs.lutParamUpdatePeriod = 1;
	AdvChirpCfgArgs.lutPatternAddressOffset = 16;
	AdvChirpCfgArgs.numOfPatterns = 4;
	AdvChirpCfgArgs.lutChirpParamScale = 0;
	AdvChirpCfgArgs.lutChirpParamSize = 0;

	retVal = MMWL_advChirpConfig(deviceMap, &AdvChirpCfgArgs);
	if (retVal != RL_RET_CODE_OK)
	{
		printf("Device map %u : MMWL_advChirpConfig failed with error code %d \n\n", deviceMap, retVal);
		return retVal;
	}

	/* read the input param data */
	MMWL_readAdvChirpIdleTimeConfig(&AdvChirpLUTIdleTimeCfgArgs);
	printf("Device map %u : Saving advChirpLUTIdleTimeConfig with \nLUTAddrOff[%d]\nParamSize[%d]\nParamScale[%d]\nIdleTimeCfgData[0]=[%.2f] us\nIdleTimeCfgData[1]=[%.2f] us\nIdleTimeCfgData[2]=[%.2f] us\nIdleTimeCfgData[3]=[%.2f] us \n\n",
		deviceMap, AdvChirpLUTIdleTimeCfgArgs.LUTAddrOff, AdvChirpLUTIdleTimeCfgArgs.ParamSize, AdvChirpLUTIdleTimeCfgArgs.ParamScale, AdvChirpLUTIdleTimeCfgArgs.IdleTimeCfgData[0], AdvChirpLUTIdleTimeCfgArgs.IdleTimeCfgData[1],
		AdvChirpLUTIdleTimeCfgArgs.IdleTimeCfgData[2], AdvChirpLUTIdleTimeCfgArgs.IdleTimeCfgData[3]);
	/* copy the input param data from mmwaveconfig.txt to a local buffer */
	/* Dividing the input param data by 1 LSB . 1 LSB = 10ns * 2^scale  = 0.01 us * 2^scale */
	IdleTimeData[0] = (rlInt16_t)(((double)AdvChirpLUTIdleTimeCfgArgs.IdleTimeCfgData[0] * 100) / pow(2, AdvChirpLUTIdleTimeCfgArgs.ParamScale));
	IdleTimeData[1] = (rlInt16_t)(((double)AdvChirpLUTIdleTimeCfgArgs.IdleTimeCfgData[1] * 100) / pow(2, AdvChirpLUTIdleTimeCfgArgs.ParamScale));
	IdleTimeData[2] = (rlInt16_t)(((double)AdvChirpLUTIdleTimeCfgArgs.IdleTimeCfgData[2] * 100) / pow(2, AdvChirpLUTIdleTimeCfgArgs.ParamScale));
	IdleTimeData[3] = (rlInt16_t)(((double)AdvChirpLUTIdleTimeCfgArgs.IdleTimeCfgData[3] * 100) / pow(2, AdvChirpLUTIdleTimeCfgArgs.ParamScale));

	/* fill up the Chirp LUT buffer which is used later for rlSetAdvChirpLUTConfig API */
	rlFillLUTParamsArgs.chirpParamIndex = RL_LUT_CHIRP_IDLE_TIME_VAR;
	rlFillLUTParamsArgs.chirpParamSize = AdvChirpCfgArgs.lutChirpParamSize;
	rlFillLUTParamsArgs.inputSize = AdvChirpCfgArgs.numOfPatterns;
	rlFillLUTParamsArgs.lutGlobalOffset = lutOffsetInNBytes;
	retVal = rlDevSetFillLUTBuff(&rlFillLUTParamsArgs, &(rlInt8_t)IdleTimeData[0], &AdvChirpLUTData[lutOffsetInNBytes], &lutOffsetInNBytes);

	/* ADC start time (Param Index = 4) */
	/* Fixed ADC start time delta dither (3) LSB's = (3 * 10 ns) = 0.03 us, the delta dither
	   will accumulate every single chirp (update period = 1) and it will reset every 4 chirps */
	/* Configuring 4 unique idle time LUT dither (0.02,0.01,0.00,0.01) us from mmwaveconfig.txt */
	/* The new idle time LUT dither is picked every chirp (update period = 1) and it will reset every 4 chirps */
	/*   Chirp    ADC start time (from Profile) + LUT dither + Fixed delta dither
		   0         6 us + 0.02 us + 0
		   1         6 us + 0.01 us + 0.03 us
		   2         6 us + 0.00 us + 0.03*2 us
		   3         6 us + 0.01 us + 0.03*3 us
		   4         6 us + 0.02 us + 0    (Delta dither reset period = 4 and LUT dither reset period = 4) and so on */
	/* LUT start address offset for the idle time chirp parameter is made 24 */
	/* AdvChirpLUTData[24] is the start address offset (Offset = 24), Each data parameter is 2 bytes (used lutChirpParamSize = 0 and lutChirpParamScale = 0) */
	/* Number of unique LUT dither parameters (4) */
	memset((void *)&AdvChirpCfgArgs, 0, sizeof(rlAdvChirpCfg_t));
	AdvChirpCfgArgs.chirpParamIdx = 4;
	AdvChirpCfgArgs.deltaResetPeriod = 4;
	AdvChirpCfgArgs.deltaParamUpdatePeriod = 1;
	AdvChirpCfgArgs.sf0ChirpParamDelta = 3;
	AdvChirpCfgArgs.lutResetPeriod = 4;
	AdvChirpCfgArgs.lutParamUpdatePeriod = 1;
	AdvChirpCfgArgs.lutPatternAddressOffset = 24;
	AdvChirpCfgArgs.numOfPatterns = 4;
	AdvChirpCfgArgs.lutChirpParamScale = 0;
	AdvChirpCfgArgs.lutChirpParamSize = 0;

	retVal = MMWL_advChirpConfig(deviceMap, &AdvChirpCfgArgs);
	if (retVal != RL_RET_CODE_OK)
	{
		printf("Device map %u : MMWL_advChirpConfig failed with error code %d \n\n", deviceMap, retVal);
		return retVal;
	}

	/* read the input param data */
	MMWL_readAdvChirpADCTimeConfig(&AdvChirpLUTADCTimeCfgArgs);
	printf("Device map %u : Saving advChirpLUTADCTimeConfig with \nLUTAddrOff[%d]\nParamSize[%d]\nParamScale[%d]\nADCTimeCfgData[0]=[%.2f] us\nADCTimeCfgData[1]=[%.2f] us\nADCTimeCfgData[2]=[%.2f] us\nADCTimeCfgData[3]=[%.2f] us \n\n",
		deviceMap, AdvChirpLUTADCTimeCfgArgs.LUTAddrOff, AdvChirpLUTADCTimeCfgArgs.ParamSize, AdvChirpLUTADCTimeCfgArgs.ParamScale, AdvChirpLUTADCTimeCfgArgs.ADCTimeCfgData[0], AdvChirpLUTADCTimeCfgArgs.ADCTimeCfgData[1],
		AdvChirpLUTADCTimeCfgArgs.ADCTimeCfgData[2], AdvChirpLUTADCTimeCfgArgs.ADCTimeCfgData[3]);
	/* copy the input param data from mmwaveconfig.txt to a local buffer */
	/* Dividing the input param data by 1 LSB . 1 LSB = 10ns * 2^scale  = 0.01 us * 2^scale */
	ADCStartTimeData[0] = (rlInt16_t)(((double)AdvChirpLUTADCTimeCfgArgs.ADCTimeCfgData[0] * 100) / pow(2, AdvChirpLUTADCTimeCfgArgs.ParamScale));
	ADCStartTimeData[1] = (rlInt16_t)(((double)AdvChirpLUTADCTimeCfgArgs.ADCTimeCfgData[1] * 100) / pow(2, AdvChirpLUTADCTimeCfgArgs.ParamScale));
	ADCStartTimeData[2] = (rlInt16_t)(((double)AdvChirpLUTADCTimeCfgArgs.ADCTimeCfgData[2] * 100) / pow(2, AdvChirpLUTADCTimeCfgArgs.ParamScale));
	ADCStartTimeData[3] = (rlInt16_t)(((double)AdvChirpLUTADCTimeCfgArgs.ADCTimeCfgData[3] * 100) / pow(2, AdvChirpLUTADCTimeCfgArgs.ParamScale));

	/* fill up the Chirp LUT buffer which is used later for rlSetAdvChirpLUTConfig API */
	rlFillLUTParamsArgs.chirpParamIndex = RL_LUT_CHIRP_ADC_START_TIME_VAR;
	rlFillLUTParamsArgs.chirpParamSize = AdvChirpCfgArgs.lutChirpParamSize;
	rlFillLUTParamsArgs.inputSize = AdvChirpCfgArgs.numOfPatterns;
	rlFillLUTParamsArgs.lutGlobalOffset = lutOffsetInNBytes;
	retVal = rlDevSetFillLUTBuff(&rlFillLUTParamsArgs, &(rlInt8_t)ADCStartTimeData[0], &AdvChirpLUTData[lutOffsetInNBytes], &lutOffsetInNBytes);

	/* Tx Enable (Param Index = 5) */
	/* Fixed delta dither is not supported for Tx Enable parameter */
	/* Configuring 4 unique Tx enable mask (7,3,1,2) in the generic SW LUT - LUT Reset period (4) */
	/* The new Tx enable mask is picked every chirp (update period = 1) and it will reset every 4 chirps */
	/*   Chirp    Tx enable mask
		   0         7
		   1         3
		   2         1
		   3         2
		   4         7     and so on */
    /* LUT start address offset for the Tx enable chirp parameter is made 32 */
    /* AdvChirpLUTData[32] is the start address offset (Offset = 32), Each data parameter is 4 bits */
    /* Number of unique LUT dither parameters (4) */
	memset((void *)&AdvChirpCfgArgs, 0, sizeof(rlAdvChirpCfg_t));
	AdvChirpCfgArgs.chirpParamIdx = 5;
	AdvChirpCfgArgs.lutResetPeriod = 4;
	AdvChirpCfgArgs.lutParamUpdatePeriod = 1;
	AdvChirpCfgArgs.lutPatternAddressOffset = 32;
	AdvChirpCfgArgs.numOfPatterns = 4;

	retVal = MMWL_advChirpConfig(deviceMap, &AdvChirpCfgArgs);
	if (retVal != RL_RET_CODE_OK)
	{
		printf("Device map %u : MMWL_advChirpConfig failed with error code %d \n\n", deviceMap, retVal);
		return retVal;
	}

	/* read the input param data */
	MMWL_readAdvChirpTxEnConfig(&AdvChirpLUTTxEnCfgArgs);
	printf("Device map %u : Saving advChirpLUTTxEnConfig with \nLUTAddrOff[%d]\nTxEnCfgData[0]=[%d]\nTxEnCfgData[1]=[%d]\nTxEnCfgData[2]=[%d]\nTxEnCfgData[3]=[%d] \n\n",
		deviceMap, AdvChirpLUTTxEnCfgArgs.LUTAddrOff, AdvChirpLUTTxEnCfgArgs.TxEnCfgData[0], AdvChirpLUTTxEnCfgArgs.TxEnCfgData[1],
		AdvChirpLUTTxEnCfgArgs.TxEnCfgData[2], AdvChirpLUTTxEnCfgArgs.TxEnCfgData[3]);
	/* copy the input param data from mmwaveconfig.txt to a local buffer */
	memcpy(&TxEnCfgData[0], &AdvChirpLUTTxEnCfgArgs.TxEnCfgData[0], 4 * sizeof(rlUInt8_t));

	/* fill up the Chirp LUT buffer which is used later for rlSetAdvChirpLUTConfig API */
	rlFillLUTParamsArgs.chirpParamIndex = RL_LUT_CHIRP_TX_EN_VAR;
	rlFillLUTParamsArgs.chirpParamSize = AdvChirpCfgArgs.lutChirpParamSize;
	rlFillLUTParamsArgs.inputSize = AdvChirpCfgArgs.numOfPatterns;
	rlFillLUTParamsArgs.lutGlobalOffset = lutOffsetInNBytes;
	retVal = rlDevSetFillLUTBuff(&rlFillLUTParamsArgs, &TxEnCfgData[0], &AdvChirpLUTData[lutOffsetInNBytes], &lutOffsetInNBytes);

	/* BPM Enable (Param Index = 6) */
	/* Fixed delta dither is not supported for BPM Enable parameter */
	/* Configuring 4 unique BPM enable mask (7,3,1,2) in the generic SW LUT - LUT Reset period (4) */
	/* The new BPM enable mask is picked every chirp (update period = 1) and it will reset every 4 chirps */
	/*   Chirp    BPM enable mask
		   0         7
		   1         3
		   2         1
		   3         2
		   4         7     and so on */
	/* LUT start address offset for the BPM enable chirp parameter is made 36 */
	/* AdvChirpLUTData[36] is the start address offset (Offset = 36), Each data parameter is 4 bits */
	/* Number of unique LUT dither parameters (4) */
	memset((void *)&AdvChirpCfgArgs, 0, sizeof(rlAdvChirpCfg_t));
	AdvChirpCfgArgs.chirpParamIdx = 6;
	AdvChirpCfgArgs.lutResetPeriod = 4;
	AdvChirpCfgArgs.lutParamUpdatePeriod = 1;
	AdvChirpCfgArgs.lutPatternAddressOffset = 36;
	AdvChirpCfgArgs.numOfPatterns = 4;

	retVal = MMWL_advChirpConfig(deviceMap, &AdvChirpCfgArgs);
	if (retVal != RL_RET_CODE_OK)
	{
		printf("Device map %u : MMWL_advChirpConfig failed with error code %d \n\n", deviceMap, retVal);
		return retVal;
	}

	/* read the input param data */
	MMWL_readAdvChirpBpmEnConfig(&AdvChirpLUTBpmEnCfgArgs);
	printf("Device map %u : Saving advChirpLUTBpmEnConfig with \nLUTAddrOff[%d]\nBpmEnCfgData[0]=[%d]\nBpmEnCfgData[1]=[%d]\nBpmEnCfgData[2]=[%d]\nBpmEnCfgData[3]=[%d] \n\n",
		deviceMap, AdvChirpLUTBpmEnCfgArgs.LUTAddrOff, AdvChirpLUTBpmEnCfgArgs.BpmEnCfgData[0], AdvChirpLUTBpmEnCfgArgs.BpmEnCfgData[1],
		AdvChirpLUTBpmEnCfgArgs.BpmEnCfgData[2], AdvChirpLUTBpmEnCfgArgs.BpmEnCfgData[3]);
	/* copy the input param data from mmwaveconfig.txt to a local buffer */
	memcpy(&BpmEnCfgData[0], &AdvChirpLUTBpmEnCfgArgs.BpmEnCfgData[0], 4 * sizeof(rlUInt8_t));

	/* fill up the Chirp LUT buffer which is used later for rlSetAdvChirpLUTConfig API */
	rlFillLUTParamsArgs.chirpParamIndex = RL_LUT_CHIRP_BPM_VAL_VAR;
	rlFillLUTParamsArgs.chirpParamSize = AdvChirpCfgArgs.lutChirpParamSize;
	rlFillLUTParamsArgs.inputSize = AdvChirpCfgArgs.numOfPatterns;
	rlFillLUTParamsArgs.lutGlobalOffset = lutOffsetInNBytes;
	retVal = rlDevSetFillLUTBuff(&rlFillLUTParamsArgs, &BpmEnCfgData[0], &AdvChirpLUTData[lutOffsetInNBytes], &lutOffsetInNBytes);

	/* TX0 Phase shifter (Param Index = 7) */
	/* Fixed TX0 phase shifter delta dither (512) LSB's = (512 * 360) degrees / 2^16 = 2.8125 degrees, the delta dither
	   will accumulate every single chirp (update period = 1) and it will reset every 4 chirps */
	/* Configuring 4 unique TX0 phase shifter LUT dither (5.625,11.250,16.875,16.875) degrees from mmwaveconfig.txt */
	/* The new TX0 phase shifter LUT dither is picked every chirp (update period = 1) and it will reset every 4 chirps */
	/*   Chirp    TX0 PS (from Profile) + LUT dither + Fixed delta dither
			0         0 deg + 5.625 deg + 0
			1         0 deg + 11.250 deg + 2.8125 deg
			2         0 deg + 16.875 deg + 2.8125*2 deg
			3         0 deg + 16.875 deg + 2.8125*3 deg
			4         0 deg + 5.625 deg + 0     (Delta dither reset period = 4 and LUT dither reset period = 4) and so on */
	/* LUT start address offset for the TX0 PS chirp parameter is made 40 */
	/* AdvChirpLUTData[40] is the start address offset (Offset = 40), Each data parameter is 1 byte */
	/* Number of unique LUT dither parameters (4) */
	memset((void *)&AdvChirpCfgArgs, 0, sizeof(rlAdvChirpCfg_t));
	AdvChirpCfgArgs.chirpParamIdx = 7;
	AdvChirpCfgArgs.deltaResetPeriod = 4;
	AdvChirpCfgArgs.deltaParamUpdatePeriod = 1;
	AdvChirpCfgArgs.sf0ChirpParamDelta = 512;
	AdvChirpCfgArgs.lutResetPeriod = 4;
	AdvChirpCfgArgs.lutParamUpdatePeriod = 1;
	AdvChirpCfgArgs.lutPatternAddressOffset = 40;
	AdvChirpCfgArgs.numOfPatterns = 4;
	AdvChirpCfgArgs.maxTxPhShiftIntDither = 0;

	retVal = MMWL_advChirpConfig(deviceMap, &AdvChirpCfgArgs);
	if (retVal != RL_RET_CODE_OK)
	{
		printf("Device map %u : MMWL_advChirpConfig failed with error code %d \n\n", deviceMap, retVal);
		return retVal;
	}

	/* read the input param data */
	MMWL_readAdvChirpTx0PhShiftConfig(&AdvChirpLUTTx0PhShiftCfgArgs);
	printf("Device map %u : Saving advChirpLUTTx0PhShiftConfig with \nLUTAddrOff[%d]\nTx0PhShiftCfgData[0]=[%.3f] deg\nTx0PhShiftCfgData[1]=[%.3f] deg\nTx0PhShiftCfgData[2]=[%.3f] deg\nTx0PhShiftCfgData[3]=[%.3f] deg \n\n",
		deviceMap, AdvChirpLUTTx0PhShiftCfgArgs.LUTAddrOff, AdvChirpLUTTx0PhShiftCfgArgs.Tx0PhShiftCfgData[0], AdvChirpLUTTx0PhShiftCfgArgs.Tx0PhShiftCfgData[1],
		AdvChirpLUTTx0PhShiftCfgArgs.Tx0PhShiftCfgData[2], AdvChirpLUTTx0PhShiftCfgArgs.Tx0PhShiftCfgData[3]);
	/* copy the input param data from mmwaveconfig.txt to a local buffer */
	/* Dividing the input param data by 1 LSB . 1 LSB = 360 / 2^6 degrees */
	Tx0PhShiftData[0] = (rlInt8_t)((AdvChirpLUTTx0PhShiftCfgArgs.Tx0PhShiftCfgData[0] * 64) / 360);
	Tx0PhShiftData[1] = (rlInt8_t)((AdvChirpLUTTx0PhShiftCfgArgs.Tx0PhShiftCfgData[1] * 64) / 360);
	Tx0PhShiftData[2] = (rlInt8_t)((AdvChirpLUTTx0PhShiftCfgArgs.Tx0PhShiftCfgData[2] * 64) / 360);
	Tx0PhShiftData[3] = (rlInt8_t)((AdvChirpLUTTx0PhShiftCfgArgs.Tx0PhShiftCfgData[3] * 64) / 360);

	/* fill up the Chirp LUT buffer which is used later for rlSetAdvChirpLUTConfig API */
	rlFillLUTParamsArgs.chirpParamIndex = RL_LUT_TX0_PHASE_SHIFT_VAR;
	rlFillLUTParamsArgs.chirpParamSize = AdvChirpCfgArgs.lutChirpParamSize;
	rlFillLUTParamsArgs.inputSize = AdvChirpCfgArgs.numOfPatterns;
	rlFillLUTParamsArgs.lutGlobalOffset = lutOffsetInNBytes;
	retVal = rlDevSetFillLUTBuff(&rlFillLUTParamsArgs, &Tx0PhShiftData[0], &AdvChirpLUTData[lutOffsetInNBytes], &lutOffsetInNBytes);

	/* TX1 Phase shifter (Param Index = 8) */
	/* Fixed TX1 phase shifter delta dither (1024) LSB's = (1024 * 360) degrees / 2^16 = 5.625 degrees, the delta dither
	   will accumulate every single chirp (update period = 1) and it will reset every 4 chirps */
	/* Configuring 4 unique TX1 phase shifter LUT dither (0.000, 5.625,0.000, 5.625) degrees from mmwaveconfig.txt */
	/* The new TX1 phase shifter LUT dither is picked every chirp (update period = 1) and it will reset every 4 chirps */
	/*   Chirp    TX1 PS (from Profile) + LUT dither + Fixed delta dither
		   0         0 deg + 0.000 deg + 0
		   1         0 deg + 5.625 deg + 5.625 deg
		   2         0 deg + 0.000 deg + 5.625*2 deg
		   3         0 deg + 5.625 deg + 5.625*3 deg
		   4         0 deg + 0.000 deg + 0     (Delta dither reset period = 4 and LUT dither reset period = 4) and so on */
	/* LUT start address offset for the TX1 PS chirp parameter is made 44 */
	/* AdvChirpLUTData[44] is the start address offset (Offset = 44), Each data parameter is 1 byte */
	/* Number of unique LUT dither parameters (4) */
	memset((void *)&AdvChirpCfgArgs, 0, sizeof(rlAdvChirpCfg_t));
	AdvChirpCfgArgs.chirpParamIdx = 8;
	AdvChirpCfgArgs.deltaResetPeriod = 4;
	AdvChirpCfgArgs.deltaParamUpdatePeriod = 1;
	AdvChirpCfgArgs.sf0ChirpParamDelta = 1024;
	AdvChirpCfgArgs.lutResetPeriod = 4;
	AdvChirpCfgArgs.lutParamUpdatePeriod = 1;
	AdvChirpCfgArgs.lutPatternAddressOffset = 44;
	AdvChirpCfgArgs.numOfPatterns = 4;
	AdvChirpCfgArgs.maxTxPhShiftIntDither = 0;

	retVal = MMWL_advChirpConfig(deviceMap, &AdvChirpCfgArgs);
	if (retVal != RL_RET_CODE_OK)
	{
		printf("Device map %u : MMWL_advChirpConfig failed with error code %d \n\n", deviceMap, retVal);
		return retVal;
	}

	/* read the input param data */
	MMWL_readAdvChirpTx1PhShiftConfig(&AdvChirpLUTTx1PhShiftCfgArgs);
	printf("Device map %u : Saving advChirpLUTTx1PhShiftConfig with \nLUTAddrOff[%d]\nTx1PhShiftCfgData[0]=[%.3f] deg\nTx1PhShiftCfgData[1]=[%.3f] deg\nTx1PhShiftCfgData[2]=[%.3f] deg\nTx1PhShiftCfgData[3]=[%.3f] deg \n\n",
		deviceMap, AdvChirpLUTTx1PhShiftCfgArgs.LUTAddrOff, AdvChirpLUTTx1PhShiftCfgArgs.Tx1PhShiftCfgData[0], AdvChirpLUTTx1PhShiftCfgArgs.Tx1PhShiftCfgData[1],
		AdvChirpLUTTx1PhShiftCfgArgs.Tx1PhShiftCfgData[2], AdvChirpLUTTx1PhShiftCfgArgs.Tx1PhShiftCfgData[3]);
	/* copy the input param data from mmwaveconfig.txt to a local buffer */
	/* Dividing the input param data by 1 LSB . 1 LSB = 360 / 2^6 degrees */
	Tx1PhShiftData[0] = (rlInt8_t)((AdvChirpLUTTx1PhShiftCfgArgs.Tx1PhShiftCfgData[0] * 64) / 360);
	Tx1PhShiftData[1] = (rlInt8_t)((AdvChirpLUTTx1PhShiftCfgArgs.Tx1PhShiftCfgData[1] * 64) / 360);
	Tx1PhShiftData[2] = (rlInt8_t)((AdvChirpLUTTx1PhShiftCfgArgs.Tx1PhShiftCfgData[2] * 64) / 360);
	Tx1PhShiftData[3] = (rlInt8_t)((AdvChirpLUTTx1PhShiftCfgArgs.Tx1PhShiftCfgData[3] * 64) / 360);

	/* fill up the Chirp LUT buffer which is used later for rlSetAdvChirpLUTConfig API */
	rlFillLUTParamsArgs.chirpParamIndex = RL_LUT_TX1_PHASE_SHIFT_VAR;
	rlFillLUTParamsArgs.chirpParamSize = AdvChirpCfgArgs.lutChirpParamSize;
	rlFillLUTParamsArgs.inputSize = AdvChirpCfgArgs.numOfPatterns;
	rlFillLUTParamsArgs.lutGlobalOffset = lutOffsetInNBytes;
	retVal = rlDevSetFillLUTBuff(&rlFillLUTParamsArgs, &Tx1PhShiftData[0], &AdvChirpLUTData[lutOffsetInNBytes], &lutOffsetInNBytes);

	/* TX2 Phase shifter (Param Index = 9) */
	/* Fixed TX2 phase shifter delta dither (2048) LSB's = (2048 * 360) degrees / 2^16 = 11.25 degrees, the delta dither
	   will accumulate every single chirp (update period = 1) and it will reset every 4 chirps */
	/* Configuring 4 unique TX2 phase shifter LUT dither (5.625,0.000, 5.625, 0.000) degrees from mmwaveconfig.txt */
	/* The new TX2 phase shifter LUT dither is picked every chirp (update period = 1) and it will reset every 4 chirps */
	/*   Chirp    TX2 PS (from Profile) + LUT dither + Fixed delta dither
		   0         0 deg + 5.625 deg + 0
		   1         0 deg + 0.000 deg + 11.25 deg
		   2         0 deg + 5.625 deg + 11.25*2 deg
		   3         0 deg + 0.000 deg + 11.25*3 deg
		   4         0 deg + 5.625 deg + 0     (Delta dither reset period = 4 and LUT dither reset period = 4) and so on */
	/* LUT start address offset for the TX2 PS chirp parameter is made 48 */
	/* AdvChirpLUTData[48] is the start address offset (Offset = 48), Each data parameter is 1 byte */
	/* Number of unique LUT dither parameters (4) */
	memset((void *)&AdvChirpCfgArgs, 0, sizeof(rlAdvChirpCfg_t));
	AdvChirpCfgArgs.chirpParamIdx = 9;
	AdvChirpCfgArgs.deltaResetPeriod = 4;
	AdvChirpCfgArgs.deltaParamUpdatePeriod = 1;
	AdvChirpCfgArgs.sf0ChirpParamDelta = 2048;
	AdvChirpCfgArgs.lutResetPeriod = 4;
	AdvChirpCfgArgs.lutParamUpdatePeriod = 1;
	AdvChirpCfgArgs.lutPatternAddressOffset = 48;
	AdvChirpCfgArgs.numOfPatterns = 4;
	AdvChirpCfgArgs.maxTxPhShiftIntDither = 0;

	retVal = MMWL_advChirpConfig(deviceMap, &AdvChirpCfgArgs);
	if (retVal != RL_RET_CODE_OK)
	{
		printf("Device map %u : MMWL_advChirpConfig failed with error code %d \n\n", deviceMap, retVal);
		return retVal;
	}

	/* read the input param data */
	MMWL_readAdvChirpTx2PhShiftConfig(&AdvChirpLUTTx2PhShiftCfgArgs);
	printf("Device map %u : Saving advChirpLUTTx2PhShiftConfig with \nLUTAddrOff[%d]\nTx2PhShiftCfgData[0]=[%.3f] deg\nTx2PhShiftCfgData[1]=[%.3f] deg\nTx2PhShiftCfgData[2]=[%.3f] deg\nTx2PhShiftCfgData[3]=[%.3f] deg \n\n",
		deviceMap, AdvChirpLUTTx2PhShiftCfgArgs.LUTAddrOff, AdvChirpLUTTx2PhShiftCfgArgs.Tx2PhShiftCfgData[0], AdvChirpLUTTx2PhShiftCfgArgs.Tx2PhShiftCfgData[1],
		AdvChirpLUTTx2PhShiftCfgArgs.Tx2PhShiftCfgData[2], AdvChirpLUTTx2PhShiftCfgArgs.Tx2PhShiftCfgData[3]);
	/* copy the input param data from mmwaveconfig.txt to a local buffer */
	/* Dividing the input param data by 1 LSB . 1 LSB = 360 / 2^6 degrees */
	Tx2PhShiftData[0] = (rlInt8_t)((AdvChirpLUTTx2PhShiftCfgArgs.Tx2PhShiftCfgData[0] * 64) / 360);
	Tx2PhShiftData[1] = (rlInt8_t)((AdvChirpLUTTx2PhShiftCfgArgs.Tx2PhShiftCfgData[1] * 64) / 360);
	Tx2PhShiftData[2] = (rlInt8_t)((AdvChirpLUTTx2PhShiftCfgArgs.Tx2PhShiftCfgData[2] * 64) / 360);
	Tx2PhShiftData[3] = (rlInt8_t)((AdvChirpLUTTx2PhShiftCfgArgs.Tx2PhShiftCfgData[3] * 64) / 360);

	/* fill up the Chirp LUT buffer which is used later for rlSetAdvChirpLUTConfig API */
	rlFillLUTParamsArgs.chirpParamIndex = RL_LUT_TX2_PHASE_SHIFT_VAR;
	rlFillLUTParamsArgs.chirpParamSize = AdvChirpCfgArgs.lutChirpParamSize;
	rlFillLUTParamsArgs.inputSize = AdvChirpCfgArgs.numOfPatterns;
	rlFillLUTParamsArgs.lutGlobalOffset = lutOffsetInNBytes;
	retVal = rlDevSetFillLUTBuff(&rlFillLUTParamsArgs, &Tx2PhShiftData[0], &AdvChirpLUTData[lutOffsetInNBytes], &lutOffsetInNBytes);

	for (int devId = 0; devId < 4; devId++)
	{
		if ((deviceMap & (1 << devId)) != 0)
		{
			unsigned char devMap = createDevMapFromDevId(devId);
			/* Save the entire locally programmed LUT data to a file for debug purposes
			   This locally programmed LUT data will be the same as RadarSS LUT at the device end */
			retVal = MMWL_saveAdvChirpLUTDataToFile(devMap);
			if (retVal != RL_RET_CODE_OK)
			{
				printf("Device map %u : MMWL_saveAdvChirpLUTDataToFile failed with error code %d \n\n", devMap, retVal);
				return retVal;
			}
			else
			{
				printf("Device map %u : MMWL_saveAdvChirpLUTDataToFile success \n\n", devMap);
			}

            /* PING BUFFER - Starts from offset 0 and ends at 52 bytes
               PONG BUFFER - Starts from offset 100 and ends at 152 bytes */
            
            for (bufIdx = 0; bufIdx < 2; bufIdx++)
            {
                /* Send the locally programmed LUT data to the device */
                rlAdvChirpLUTCfgArgs.lutAddressOffset = 100*bufIdx;
                rlAdvChirpLUTCfgArgs.numBytes = lutOffsetInNBytes;
                
                retVal = rlSetMultiAdvChirpLUTConfig(devMap, &rlAdvChirpLUTCfgArgs, &AdvChirpLUTData[0]);
                if (retVal != RL_RET_CODE_OK)
                {
                    printf("Device map %u : rlSetMultiAdvChirpLUTConfig failed with error code %d \n\n", devMap, retVal);
                    return retVal;
                }
                else
                {
                    printf("Device map %u : rlSetMultiAdvChirpLUTConfig success with lutAddressOffset = %d and numBytes = %d\n\n", devMap, rlAdvChirpLUTCfgArgs.lutAddressOffset, rlAdvChirpLUTCfgArgs.numBytes);
                }
            }
		}
	}

	return retVal;
}

/** @fn int MMWL_advChirpConfig(unsigned char deviceMap)
*
*   @brief Advanced chirp configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Advanced chirp configuration API.
*/
int MMWL_advChirpConfig(unsigned char deviceMap, rlAdvChirpCfg_t* AdvChirpCfgArgs)
{
	int retVal = RL_RET_CODE_OK;

	printf("Device map %u : Calling rlSetAdvChirpConfig with \nchirpParamIdx[%d]\nresetMode[%d]\ndeltaResetPeriod[%d]\ndeltaParamUpdatePeriod[%d]\nsf0ChirpParamDelta[%d]\nsf1ChirpParamDelta[%d]\nsf2ChirpParamDelta[%d]\nsf3ChirpParamDelta[%d]\n",
		deviceMap, AdvChirpCfgArgs->chirpParamIdx, AdvChirpCfgArgs->resetMode, AdvChirpCfgArgs->deltaResetPeriod,
		AdvChirpCfgArgs->deltaParamUpdatePeriod, AdvChirpCfgArgs->sf0ChirpParamDelta, AdvChirpCfgArgs->sf1ChirpParamDelta,
		AdvChirpCfgArgs->sf2ChirpParamDelta, AdvChirpCfgArgs->sf3ChirpParamDelta);

	printf("lutResetPeriod[%d]\nlutParamUpdatePeriod[%d]\nlutPatternAddressOffset[%d]\nnumOfPatterns[%d]\nlutBurstIndexOffset[%d]\nlutSfIndexOffset[%d]\nlutChirpParamSize[%d]\nlutChirpParamScale[%d]\nmaxTxPhShiftIntDither[%d] \n\n",
		AdvChirpCfgArgs->lutResetPeriod, AdvChirpCfgArgs->lutParamUpdatePeriod, AdvChirpCfgArgs->lutPatternAddressOffset,
		AdvChirpCfgArgs->numOfPatterns, AdvChirpCfgArgs->lutBurstIndexOffset, AdvChirpCfgArgs->lutSfIndexOffset,
		AdvChirpCfgArgs->lutChirpParamSize, AdvChirpCfgArgs->lutChirpParamScale, AdvChirpCfgArgs->maxTxPhShiftIntDither);

	retVal = CALL_API(SET_ADV_CHIRP_CONFIG_IND, deviceMap, AdvChirpCfgArgs, 0);
	return retVal;
}

/** @fn int MMWL_saveAdvChirpLUTDataToFile(unsigned char deviceMap)
*
*   @brief Save Advanced Chirp LUT Data to a file.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Save Advanced Chirp LUT Data to a file.
*/
int MMWL_saveAdvChirpLUTDataToFile(unsigned char deviceMap)
{
	int retVal = RL_RET_CODE_OK;
	int index = 0, j;
	char AdvChirpLUTDataBuff[LUT_ADVCHIRP_TABLE_SIZE*8] = { 0 };

	if (deviceMap == 1)
		AdvChirpLUTDataPtr = _fsopen("AdvChirpLUTData_0.txt", "wt", _SH_DENYWR);
	else if (deviceMap == 2)
		AdvChirpLUTDataPtr = _fsopen("AdvChirpLUTData_1.txt", "wt", _SH_DENYWR);
	else if (deviceMap == 4)
		AdvChirpLUTDataPtr = _fsopen("AdvChirpLUTData_2.txt", "wt", _SH_DENYWR);
	else if (deviceMap == 8)
		AdvChirpLUTDataPtr = _fsopen("AdvChirpLUTData_3.txt", "wt", _SH_DENYWR);

	/* Store the entire LUT data in terms of 1 byte per line */
	for (j = 0; j < LUT_ADVCHIRP_TABLE_SIZE; j++)
	{
		sprintf(AdvChirpLUTDataBuff + strlen(AdvChirpLUTDataBuff), "%d\n", AdvChirpLUTData[j]);
	}

	fwrite(AdvChirpLUTDataBuff, sizeof(char), strlen(AdvChirpLUTDataBuff), AdvChirpLUTDataPtr);
	fflush(AdvChirpLUTDataPtr);

	if (AdvChirpLUTDataPtr != NULL)
		fclose(AdvChirpLUTDataPtr);

	return retVal;
}

/** @fn int MMWL_frameConfig(unsigned char deviceMap)
*
*   @brief Frame configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Frame configuration API.
*/
int MMWL_frameConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
	unsigned char devId;
    rlFrameCfg_t frameCfgArgs = { 0 };

    /*read frameCfgArgs from config file*/
    MMWL_readFrameConfig(&frameCfgArgs);
	if (deviceMap == 1)
	{
		frameCfgArgs.triggerSelect = 1;
	}
	else
	{
		frameCfgArgs.triggerSelect = 2;
	}

    framePeriodicity = (frameCfgArgs.framePeriodicity * 5)/(1000*1000);
    frameCount = frameCfgArgs.numFrames;

	if (deviceMap == 1)
	{
		mmwl_TDA_framePeriodicity = framePeriodicity;
	}

	/* In Adv chirp context, frame start and frame end index is not used, the nuber of chirps is taken from number of loops */
	if (rlDevGlobalCfgArgs.LinkAdvChirpTest == TRUE)
	{
		frameCfgArgs.numLoops = frameCfgArgs.numLoops * (frameCfgArgs.chirpEndIdx - frameCfgArgs.chirpStartIdx + 1);
		frameCfgArgs.chirpEndIdx = 0;
		frameCfgArgs.chirpStartIdx = 0;
	}

    printf("Device map %u : Calling rlSetFrameConfig with \nStart Idx[%d]\nEnd Idx[%d]\nLoops[%d]\nPeriodicity[%d]ms \n\n",
        deviceMap, frameCfgArgs.chirpStartIdx, frameCfgArgs.chirpEndIdx,
        frameCfgArgs.numLoops, (frameCfgArgs.framePeriodicity * 5)/(1000*1000));

	retVal = CALL_API(SET_FRAME_CONFIG_IND, deviceMap, &frameCfgArgs, 0);

	for (devId = 0; devId < 4; devId++)
	{
		if ((deviceMap & (1 << devId)) != 0)
		{
			/* Height calculation */
			mmwl_TDA_height[devId] = frameCfgArgs.numLoops * (frameCfgArgs.chirpEndIdx - frameCfgArgs.chirpStartIdx + 1);
			printf("Device map %u : Calculated TDA Height is %d\n\n", deviceMap, mmwl_TDA_height[devId]);

			/* Width calculation */
			/* Count the number of Rx antenna */
			unsigned char numRxAntenna = 0;
			rlChanCfg_t rfChanCfgArgs = { 0 };
			MMWL_readChannelConfig(&rfChanCfgArgs, 0);
			while (rfChanCfgArgs.rxChannelEn != 0)
			{
				if ((rfChanCfgArgs.rxChannelEn & 0x1) == 1)
				{
					numRxAntenna++;
				}
				rfChanCfgArgs.rxChannelEn = (rfChanCfgArgs.rxChannelEn >> 1);
			}

			/* ADC format (in bytes) */
			unsigned char numValPerAdcSample = 0, numAdcBits = 0;
			rlAdcOutCfg_t adcOutCfgArgs = { 0 };
			MMWL_readAdcOutConfig(&adcOutCfgArgs);
			if (adcOutCfgArgs.fmt.b2AdcOutFmt == 1 || adcOutCfgArgs.fmt.b2AdcOutFmt == 2)
			{
				numValPerAdcSample = 2;
			}
			else
			{
				numValPerAdcSample = 1;
			}

			if (adcOutCfgArgs.fmt.b2AdcBits == 0)
			{
				numAdcBits = 12;
			}
			else if (adcOutCfgArgs.fmt.b2AdcBits == 1)
			{
				numAdcBits = 14;
			}
			else if (adcOutCfgArgs.fmt.b2AdcBits == 2)
			{
				numAdcBits = 16;
			}
			/* Number of ADC samples */
			unsigned short numAdcSamples = 0;
			if (rlDevGlobalCfgArgs.LinkAdvChirpTest == FALSE)
			{
				numAdcSamples = profileCfgArgs[0].numAdcSamples;
			}
			else
			{
				numAdcSamples = ProfileCfgArgs_AdvChirp[0].numAdcSamples;
			}

			/* Get CP and CQ value */
			unsigned short cp_data = 0, cq_val = 0;
			unsigned int cq_data = 0;
			rlDevDataPathCfg_t dataPathCfgArgs = { 0 };
			MMWL_readDataPathConfig(&dataPathCfgArgs);
			cq_val = dataPathCfgArgs.cq0TransSize + dataPathCfgArgs.cq1TransSize + dataPathCfgArgs.cq2TransSize;

			if (dataPathCfgArgs.transferFmtPkt0 == 6 || dataPathCfgArgs.transferFmtPkt0 == 9)
			{
				cp_data = 2;
				cq_data = 0;
			}
			else if (dataPathCfgArgs.transferFmtPkt0 == 54)
			{
				cp_data = 2;
				cq_data = (cq_val * 16) / numAdcBits;
			}

			mmwl_TDA_width[devId] = (((numValPerAdcSample * numAdcSamples) + cp_data) * numRxAntenna) + cq_data;
			printf("Device map %u : Calculated TDA Width is %d\n\n", deviceMap, mmwl_TDA_width[devId]);
		}
	}
    return retVal;
}

/** @fn int MMWL_advFrameConfig(unsigned char deviceMap)
*
*   @brief Advance Frame configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Frame configuration API.
*/
int MMWL_advFrameConfig(unsigned char deviceMap)
{
    int i, retVal = RL_RET_CODE_OK;
	unsigned char devId;
	unsigned int SubFramePeriodicity[4] = { 0 };
	unsigned int SubFramePeriodicityMax = 0;
    rlAdvFrameCfg_t AdvframeCfgArgs = { 0 };
    rlAdvFrameCfg_t GetAdvFrameCfgArgs = { 0 };
    /* reset frame periodicity to zero */
    framePeriodicity = 0;

    /*read frameCfgArgs from config file*/
    MMWL_readAdvFrameConfig(&AdvframeCfgArgs);
	if (deviceMap == 1)
	{
		AdvframeCfgArgs.frameSeq.triggerSelect = 1;
	}
	else
	{
		AdvframeCfgArgs.frameSeq.triggerSelect = 2;
	}

    /* Add all subframes periodicity to get whole frame periodicity */
	for (i = 0; i < AdvframeCfgArgs.frameSeq.numOfSubFrames; i++)
	{
		SubFramePeriodicity[i] = (AdvframeCfgArgs.frameSeq.subFrameCfg[i].subFramePeriodicity * 5)/(1000*1000);
		framePeriodicity = framePeriodicity + SubFramePeriodicity[i];
	}
    /* store total number of frames configured */
    frameCount = AdvframeCfgArgs.frameSeq.numFrames;
	if (deviceMap == 1)
	{
		SubFramePeriodicityMax = SubFramePeriodicity[0];
		for (i = 1; i < AdvframeCfgArgs.frameSeq.numOfSubFrames; i++)
		{
			if (SubFramePeriodicityMax < SubFramePeriodicity[i])
			{
				SubFramePeriodicityMax = SubFramePeriodicity[i];
			}
		}
		mmwl_TDA_framePeriodicity = SubFramePeriodicityMax;
	}

    printf("Device map %u : Calling rlSetAdvFrameConfig with \nnumOfSubFrames[%d]\nforceProfile[%d]\nnumFrames[%d]\ntriggerSelect[%d]ms \n\n",
        deviceMap, AdvframeCfgArgs.frameSeq.numOfSubFrames, AdvframeCfgArgs.frameSeq.forceProfile,
        AdvframeCfgArgs.frameSeq.numFrames, AdvframeCfgArgs.frameSeq.triggerSelect);

	retVal = CALL_API(SET_ADV_FRAME_CONFIG_IND, deviceMap, &AdvframeCfgArgs, 0);

	for (devId = 0; devId < 4; devId++)
	{
		if ((deviceMap & (1 << devId)) != 0)
		{
			/* Height calculation */
			mmwl_TDA_height[devId] = AdvframeCfgArgs.frameSeq.subFrameCfg->numLoops * AdvframeCfgArgs.frameSeq.subFrameCfg->numOfChirps * \
				AdvframeCfgArgs.frameSeq.subFrameCfg->numOfBurst * AdvframeCfgArgs.frameSeq.subFrameCfg->numOfBurstLoops;
			printf("Device map %u : Calculated TDA Height is %d\n\n", deviceMap, mmwl_TDA_height[devId]);

			/* Width calculation */
			/* Count the number of Rx antenna */
			unsigned char numRxAntenna = 0;
			rlChanCfg_t rfChanCfgArgs = { 0 };
			MMWL_readChannelConfig(&rfChanCfgArgs, 0);
			while (rfChanCfgArgs.rxChannelEn != 0)
			{
				if ((rfChanCfgArgs.rxChannelEn & 0x1) == 1)
				{
					numRxAntenna++;
				}
				rfChanCfgArgs.rxChannelEn = (rfChanCfgArgs.rxChannelEn >> 1);
			}

			/* ADC format (in bytes) */
			unsigned char numValPerAdcSample = 0, numAdcBits = 0;
			rlAdcOutCfg_t adcOutCfgArgs = { 0 };
			MMWL_readAdcOutConfig(&adcOutCfgArgs);
			if (adcOutCfgArgs.fmt.b2AdcOutFmt == 1 || adcOutCfgArgs.fmt.b2AdcOutFmt == 2)
			{
				numValPerAdcSample = 2;
			}
			else
			{
				numValPerAdcSample = 1;
			}

			if (adcOutCfgArgs.fmt.b2AdcBits == 0)
			{
				numAdcBits = 12;
			}
			else if (adcOutCfgArgs.fmt.b2AdcBits == 1)
			{
				numAdcBits = 14;
			}
			else if (adcOutCfgArgs.fmt.b2AdcBits == 2)
			{
				numAdcBits = 16;
			}
			/* Number of ADC samples */
			unsigned short numAdcSamples = 0;
			if (rlDevGlobalCfgArgs.LinkAdvChirpTest == FALSE)
			{
				numAdcSamples = profileCfgArgs[0].numAdcSamples;
			}
			else
			{
				numAdcSamples = ProfileCfgArgs_AdvChirp[0].numAdcSamples;
			}

			/* Get CP and CQ value */
			unsigned short cp_data = 0, cq_val = 0;
			unsigned int cq_data = 0;
			rlDevDataPathCfg_t dataPathCfgArgs = { 0 };
			MMWL_readDataPathConfig(&dataPathCfgArgs);
			cq_val = dataPathCfgArgs.cq0TransSize + dataPathCfgArgs.cq1TransSize + dataPathCfgArgs.cq2TransSize;

			if (dataPathCfgArgs.transferFmtPkt0 == 6 || dataPathCfgArgs.transferFmtPkt0 == 9)
			{
				cp_data = 2;
				cq_data = 0;
			}
			else if (dataPathCfgArgs.transferFmtPkt0 == 54)
			{
				cp_data = 2;
				cq_data = (cq_val * 16) / numAdcBits;
			}

			mmwl_TDA_width[devId] = (((numValPerAdcSample * numAdcSamples) + cp_data) * numRxAntenna) + cq_data;
			printf("Device map %u : Calculated TDA Width is %d\n\n", deviceMap, mmwl_TDA_width[devId]);
		}
	}

    if (retVal == 0)
    {
		retVal = CALL_API(GET_ADV_FRAME_CONFIG_IND, deviceMap, &GetAdvFrameCfgArgs, 0);
        if ((AdvframeCfgArgs.frameSeq.forceProfile != GetAdvFrameCfgArgs.frameSeq.forceProfile) || \
            (AdvframeCfgArgs.frameSeq.frameTrigDelay != GetAdvFrameCfgArgs.frameSeq.frameTrigDelay) || \
            (AdvframeCfgArgs.frameSeq.numFrames != GetAdvFrameCfgArgs.frameSeq.numFrames) || \
            (AdvframeCfgArgs.frameSeq.numOfSubFrames != GetAdvFrameCfgArgs.frameSeq.numOfSubFrames) || \
            (AdvframeCfgArgs.frameSeq.triggerSelect != GetAdvFrameCfgArgs.frameSeq.triggerSelect))
        {
            printf("Device map %u : MMWL_readAdvFrameConfig failed...\n\n", deviceMap);
            return retVal;
        }
    }
    return retVal;
}

/**
 *******************************************************************************
 *
 * \brief   Local function to enable the dummy input of objects from AWR143
 *
 * \param   None
 * /return  retVal   BSP_SOK if the test source is set correctly.
 *
 *******************************************************************************
*/
#if defined (ENABLE_TEST_SOURCE)
int MMWL_testSourceConfig(unsigned char deviceMap)
{
    rlTestSource_t tsArgs = {0};
    rlTestSourceEnable_t tsEnableArgs = {0};
    int retVal = RL_RET_CODE_OK;

    tsArgs.testObj[0].posX = 0;

    tsArgs.testObj[0].posY = 500;
    tsArgs.testObj[0].posZ = 0;

    tsArgs.testObj[0].velX = 0;
    tsArgs.testObj[0].velY = 0;
    tsArgs.testObj[0].velZ = 0;

    tsArgs.testObj[0].posXMin = -32700;
    tsArgs.testObj[0].posYMin = 0;
    tsArgs.testObj[0].posZMin = -32700;

    tsArgs.testObj[0].posXMax = 32700;
    tsArgs.testObj[0].posYMax = 32700;
    tsArgs.testObj[0].posZMax = 32700;

    tsArgs.testObj[0].sigLvl = 150;

    tsArgs.testObj[1].posX = 0;
    tsArgs.testObj[1].posY = 32700;
    tsArgs.testObj[1].posZ = 0;

    tsArgs.testObj[1].velX = 0;
    tsArgs.testObj[1].velY = 0;
    tsArgs.testObj[1].velZ = 0;

    tsArgs.testObj[1].posXMin = -32700;
    tsArgs.testObj[1].posYMin = 0;
    tsArgs.testObj[1].posZMin = -32700;

    tsArgs.testObj[1].posXMax = 32700;
    tsArgs.testObj[1].posYMax = 32700;
    tsArgs.testObj[1].posZMax = 32700;

    tsArgs.testObj[1].sigLvl = 948;

    tsArgs.rxAntPos[0].antPosX = 0;
    tsArgs.rxAntPos[0].antPosZ = 0;
    tsArgs.rxAntPos[1].antPosX = 32;
    tsArgs.rxAntPos[1].antPosZ = 0;
    tsArgs.rxAntPos[2].antPosX = 64;
    tsArgs.rxAntPos[2].antPosZ = 0;
    tsArgs.rxAntPos[3].antPosX = 96;
    tsArgs.rxAntPos[3].antPosZ = 0;

    printf("Device map %u : Calling rlSetTestSourceConfig with Simulated Object at X[%d]cm, Y[%d]cm, Z[%d]cm \n\n",
		deviceMap, tsArgs.testObj[0].posX, tsArgs.testObj[0].posY, tsArgs.testObj[0].posZ);

	retVal = CALL_API(SET_TEST_SOURCE_CONFIG_IND, deviceMap, &tsArgs, 0);

    tsEnableArgs.tsEnable = 1U;
	retVal = CALL_API(RF_TEST_SOURCE_ENABLE, deviceMap, &tsEnableArgs, 0);

    return retVal;
}
#endif

/** @fn int MMWL_dataPathConfig(unsigned char deviceMap)
*
*   @brief Data path configuration API. Configures CQ data size on the
*           lanes and number of samples of CQ[0-2] to br transferred.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Data path configuration API. Configures CQ data size on the
*   lanes and number of samples of CQ[0-2] to br transferred.
*/
int MMWL_dataPathConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    rlDevDataPathCfg_t dataPathCfgArgs = { 0 };

    /* read dataPathCfgArgs from config file */
    MMWL_readDataPathConfig(&dataPathCfgArgs);

    printf("Device map %u : Calling rlDeviceSetDataPathConfig with HSI Interface[%d] Selected \n\n",
            deviceMap, dataPathCfgArgs.intfSel);

    /* same API is used to configure CQ data size on the
     * lanes and number of samples of CQ[0-2] to br transferred.
     */
	retVal = CALL_API(SET_DATA_PATH_CONFIG_IND, deviceMap, &dataPathCfgArgs, 0);
    return retVal;
}

#if defined (LVDS_ENABLE)
/** @fn int MMWL_lvdsLaneConfig(unsigned char deviceMap)
*
*   @brief Lane Config API
*
*   @return Success - 0, Failure - Error Code
*
*   Lane Config API
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
int MMWL_lvdsLaneConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    rlDevLvdsLaneCfg_t lvdsLaneCfgArgs = { 0 };

    /*read lvdsLaneCfgArgs from config file*/
    MMWL_readLvdsLaneConfig(&lvdsLaneCfgArgs);

	retVal = CALL_API(SET_LVDS_LANE_CONFIG_IND, deviceMap, &lvdsLaneCfgArgs, 0);
    return retVal;
}

/** @fn int MMWL_laneConfig(unsigned char deviceMap)
*
*   @brief Lane Enable API
*
*   @return Success - 0, Failure - Error Code
*
*   Lane Enable API
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
int MMWL_laneConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    rlDevLaneEnable_t laneEnCfgArgs = { 0 };

    /*read laneEnCfgArgs from config file*/
    MMWL_readLaneConfig(&laneEnCfgArgs);

	retVal = CALL_API(SET_LANE_CONFIG_IND, deviceMap, &laneEnCfgArgs, 0);
    return retVal;
}
#else
/** @fn int MMWL_CSI2LaneConfig(unsigned char deviceMap)
*
*   @brief CSI2 Lane Config API
*
*   @return Success - 0, Failure - Error Code
*
*   CSI2 Lane Config API
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
int MMWL_CSI2LaneConfig(unsigned char deviceMap)
{
	int retVal = RL_RET_CODE_OK;
	rlDevCsi2Cfg_t CSI2LaneCfgArgs = { 0 };

	/*read CSI2LaneCfgArgs from config file*/
	MMWL_readCSI2LaneConfig(&CSI2LaneCfgArgs);

	retVal = CALL_API(SET_CSI2_CONFIG_IND, deviceMap, &CSI2LaneCfgArgs, 0);
	return retVal;
}
#endif

/** @fn int MMWL_hsiLaneConfig(unsigned char deviceMap)
*
*   @brief LVDS lane configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   LVDS lane configuration API.
*/
int MMWL_hsiLaneConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
#if defined LVDS_ENABLE
	/*lane configuration*/
    retVal = MMWL_laneConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Device map %u : LaneConfig failed with error code %d\n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("Device map %u : LaneConfig success\n\n", deviceMap);
    }
    /*LVDS lane configuration*/
    retVal = MMWL_lvdsLaneConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Device map %u : LvdsLaneConfig failed with error code %d\n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("Device map %u : LvdsLaneConfig success\n\n", deviceMap);
    }
#else
	/*CSI2 lane configuration*/
	retVal = MMWL_CSI2LaneConfig(deviceMap);
	if (retVal != RL_RET_CODE_OK)
	{
		printf("Device map %u : CSI2LaneConfig failed with error code %d\n\n",
			deviceMap, retVal);
		return -1;
	}
	else
	{
		printf("Device map %u : CSI2LaneConfig success\n\n", deviceMap);
	}
#endif
    return retVal;
}

/** @fn int MMWL_setHsiClock(unsigned char deviceMap)
*
*   @brief High Speed Interface Clock Config API
*
*   @return Success - 0, Failure - Error Code
*
*   HSI Clock Config API
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
int MMWL_setHsiClock(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    rlDevHsiClk_t hsiClkgs = { 0 };

    /*read hsiClkgs from config file*/
    MMWL_readSetHsiClock(&hsiClkgs);

    printf("Device map %u : Calling rlDeviceSetHsiClk with HSI Clock[%d] \n\n",
            deviceMap, hsiClkgs.hsiClk);

	retVal = CALL_API(SET_HSI_CLK_IND, deviceMap, &hsiClkgs, 0);
    return retVal;
}

/** @fn int MMWL_hsiDataRateConfig(unsigned char deviceMap)
*
*   @brief LVDS/CSI2 Clock Config API
*
*   @return Success - 0, Failure - Error Code
*
*   LVDS/CSI2 Clock Config API
*/
/* SourceId :  */
/* DesignId :  */
/* Requirements :  */
int MMWL_hsiDataRateConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    rlDevDataPathClkCfg_t dataPathClkCfgArgs = { 0 };

    /*read lvdsClkCfgArgs from config file*/
    MMWL_readLvdsClkConfig(&dataPathClkCfgArgs);

    printf("Device map %u : Calling rlDeviceSetDataPathClkConfig with HSI Data Rate[%d] Selected \n\n",
            deviceMap, dataPathClkCfgArgs.dataRate);

	retVal = CALL_API(SET_DATA_PATH_CLK_CONFIG_IND, deviceMap, &dataPathClkCfgArgs, 0);
    return retVal;
}

/** @fn int MMWL_hsiClockConfig(unsigned char deviceMap)
*
*   @brief Clock configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Clock configuration API.
*/
int MMWL_hsiClockConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK, readAllParams = 0;

    /*LVDS clock configuration*/
    retVal = MMWL_hsiDataRateConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Device map %u : LvdsClkConfig failed with error code %d\n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("Device map %u : MMWL_hsiDataRateConfig success\n\n", deviceMap);
    }

    /*set high speed clock configuration*/
    retVal = MMWL_setHsiClock(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Device map %u : MMWL_setHsiClock failed with error code %d\n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("Device map %u : MMWL_setHsiClock success\n\n", deviceMap);
    }

    return retVal;
}

/** @fn int MMWL_gpadcMeasConfig(unsigned char deviceMap)
*
*   @brief API to set GPADC configuration.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code.
*
*   API to set GPADC Configuration. And device will    send GPADC
*    measurement data in form of Asynchronous event over SPI to
*    Host. User needs to feed input signal on the device pins where
*    they want to read the measurement data inside the device.
*/
int MMWL_gpadcMeasConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    int timeOutCnt = 0;
    rlGpAdcCfg_t gpadcCfg = {0};

    /* enable all the sensors [0-6] to read gpADC measurement data */
    gpadcCfg.enable = 0x3F;
    /* set the number of samples device needs to collect to do the measurement */
    gpadcCfg.numOfSamples[0].sampleCnt = 32;
    gpadcCfg.numOfSamples[1].sampleCnt = 32;
    gpadcCfg.numOfSamples[2].sampleCnt = 32;
    gpadcCfg.numOfSamples[3].sampleCnt = 32;
    gpadcCfg.numOfSamples[4].sampleCnt = 32;
    gpadcCfg.numOfSamples[5].sampleCnt = 32;
    gpadcCfg.numOfSamples[6].sampleCnt = 32;

	retVal = CALL_API(SET_GPADC_CONFIG, deviceMap, &gpadcCfg, 0);

    if(retVal == RL_RET_CODE_OK)
    {
        while ((mmwl_bGpadcDataRcv & deviceMap) != deviceMap)
        {
            osiSleep(1); /*Sleep 1 msec*/
            timeOutCnt++;
            if (timeOutCnt > MMWL_API_RF_INIT_TIMEOUT)
            {
                retVal = RL_RET_CODE_RESP_TIMEOUT;
                break;
            }
        }
    }

    return retVal;
}

/** @fn int MMWL_sensorStart(unsigned char deviceMap)
*
*   @brief API to Start sensor.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   API to Start sensor.
*/
int MMWL_sensorStart(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    int timeOutCnt = 0;

	rlFrameTrigger_t data = { 0 };
	/* Start the frame */
	data.startStop = 0x1;
	mmwl_bSensorStarted = mmwl_bSensorStarted & (~deviceMap);
	retVal = CALL_API(SENSOR_START_STOP_IND, deviceMap, &data, 0);
    while ((mmwl_bSensorStarted & deviceMap) != deviceMap)
    {
        osiSleep(1); /*Sleep 1 msec*/
        timeOutCnt++;
        if (timeOutCnt > MMWL_API_RF_INIT_TIMEOUT)
        {
            retVal = RL_RET_CODE_RESP_TIMEOUT;
            break;
        }
    }
    return retVal;
}

/** @fn int MMWL_sensorStop(unsigned char deviceMap)
*
*   @brief API to Stop sensor.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   API to Stop Sensor.
*/
int MMWL_sensorStop(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK, timeOutCnt =0;
	rlFrameTrigger_t data = { 0 };
	/* Stop the frame after the current frame is over */
	data.startStop = 0;
	retVal = CALL_API(SENSOR_START_STOP_IND, deviceMap, &data, 0);
    if (retVal == RL_RET_CODE_OK)
    {
        while ((mmwl_bSensorStarted & deviceMap) == deviceMap)
        {
            osiSleep(1); /*Sleep 1 msec*/
            timeOutCnt++;
            if (timeOutCnt > MMWL_API_RF_INIT_TIMEOUT)
            {
                retVal = RL_RET_CODE_RESP_TIMEOUT;
                break;
            }
        }
    }
    return retVal;
}

/** @fn int MMWL_setContMode(unsigned char deviceMap)
*
*   @brief API to set continuous mode.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   API to set continuous mode.
*/
int MMWL_setContMode(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    rlContModeCfg_t contModeCfgArgs = { 0 };
    contModeCfgArgs.digOutSampleRate = 9000;
    contModeCfgArgs.hpfCornerFreq1 = 0;
    contModeCfgArgs.hpfCornerFreq2 = 0;
	contModeCfgArgs.startFreqConst = 1435388860; /* 77GHz */
    contModeCfgArgs.txOutPowerBackoffCode = 0;
    contModeCfgArgs.txPhaseShifter = 0;

	/* store the digital sampling rate */
	gContStreamSampleRate = contModeCfgArgs.digOutSampleRate;

    /*read contModeCfgArgs from config file*/
    MMWL_readContModeConfig(&contModeCfgArgs);

    printf("Device map %u : Calling setContMode with\n digOutSampleRate[%d]\nstartFreqConst[%d]\ntxOutPowerBackoffCode[%d]\nRXGain[%d]\n\n", \
        deviceMap, contModeCfgArgs.digOutSampleRate, contModeCfgArgs.startFreqConst, contModeCfgArgs.txOutPowerBackoffCode, \
        contModeCfgArgs.rxGain);
	retVal = CALL_API(SET_CONT_MODE_CONFIG_IND, deviceMap, &contModeCfgArgs, 0);
    return retVal;
}

/** @fn int MMWL_dynChirpEnable(unsigned char deviceMap)
*
*   @brief API to enable Dynamic chirp feature.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   API to enable Dynamic chirp feature.
*/
int MMWL_dynChirpEnable(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    rlDynChirpEnCfg_t dynChirpEnCfgArgs = { 0 };

	retVal = CALL_API(SET_DYN_CHIRP_EN_IND, deviceMap, &dynChirpEnCfgArgs, 0);
    return retVal;
}

/** @fn int MMWL_setDynChirpConfig(unsigned char deviceMap)
*
*   @brief API to config chirp dynamically.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   API to config chirp dynamically.
*/
int  MMWL_setDynChirpConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    unsigned int cnt;
    rlDynChirpCfg_t * dataDynChirp[3U] = { &dynChirpCfgArgs[0], &dynChirpCfgArgs[1], &dynChirpCfgArgs[2]};

    dynChirpCfgArgs[0].programMode = 0;

    /* Configure NR1 for 48 chirps */
    dynChirpCfgArgs[0].chirpRowSelect = 0x10;
    dynChirpCfgArgs[0].chirpSegSel = 0;
    /* Copy this dynamic chirp config to other config and update chirp segment number */
    memcpy(&dynChirpCfgArgs[1], &dynChirpCfgArgs[0], sizeof(rlDynChirpCfg_t));
    memcpy(&dynChirpCfgArgs[2], &dynChirpCfgArgs[0], sizeof(rlDynChirpCfg_t));
    /* Configure NR2 for 48 chirps */
    dynChirpCfgArgs[1].chirpRowSelect = 0x20;
    dynChirpCfgArgs[1].chirpSegSel = 1;
    /* Configure NR3 for 48 chirps */
    dynChirpCfgArgs[2].chirpRowSelect = 0x30;
    dynChirpCfgArgs[2].chirpSegSel = 2;

    for (cnt = 0; cnt < 16; cnt++)
    {
        /* Reconfiguring frequency slope for 48 chirps */
        dynChirpCfgArgs[0].chirpRow[cnt].chirpNR1 |= (((3*cnt) & 0x3FU) << 8);
        dynChirpCfgArgs[0].chirpRow[cnt].chirpNR2 |= (((3*cnt + 1) & 0x3FU) << 8);
        dynChirpCfgArgs[0].chirpRow[cnt].chirpNR3 |= (((3*cnt + 2) & 0x3FU) << 8);
        /* Reconfiguring start frequency for 48 chirps */
        dynChirpCfgArgs[1].chirpRow[cnt].chirpNR1 |= 3*cnt;
        dynChirpCfgArgs[1].chirpRow[cnt].chirpNR2 |= 3*cnt + 1;
        dynChirpCfgArgs[1].chirpRow[cnt].chirpNR3 |= 3*cnt + 2;
        /* Reconfiguring ideal time for 48 chirps */
        dynChirpCfgArgs[2].chirpRow[cnt].chirpNR1 |= 3 * cnt;
        dynChirpCfgArgs[2].chirpRow[cnt].chirpNR2 |= 3 * cnt + 1;
        dynChirpCfgArgs[2].chirpRow[cnt].chirpNR3 |= 3 * cnt + 2;
    }

    printf("Device map %u : Calling DynChirpCfg with chirpSegSel[%d]\nchirpNR1[%d]\n\n", \
        deviceMap, dynChirpCfgArgs[0].chirpSegSel, dynChirpCfgArgs[0].chirpRow[0].chirpNR1);
	retVal = CALL_API(API_TYPE_C | SET_DYN_CHIRP_CFG_IND, deviceMap, &dataDynChirp[0], 2U);
    return retVal;
}

/** @fn int  MMWL_setDynAdvChirpOffsetConfig(unsigned char deviceMap, rlAdvChirpDynLUTAddrOffCfg_t *data)
*
*   @brief API to config advnace chirp LUT offset dynamically.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   API to config advnace chirp LUT offset dynamically.
*/
int  MMWL_setDynAdvChirpOffsetConfig(unsigned char deviceMap, rlAdvChirpDynLUTAddrOffCfg_t *data)
{
    int retVal = RL_RET_CODE_OK;
    printf("Device map %u : Calling rlSetAdvChirpDynLUTAddrOffConfig with profile index LUT offset = %d\n\n", \
            deviceMap, data->lutAddressOffset[0U]);
	retVal = CALL_API(SET_ADV_CHIRP_DYN_LUT_CONFIG_IND, deviceMap, data, 0);
    return retVal;
}

/** @fn int MMWL_powerOff(unsigned char deviceMap)
*
*   @brief API to poweroff device.
*
*   @param[in] deviceMap - Device Index
*
*   @return int Success - 0, Failure - Error Code
*
*   API to poweroff device.
*/
int MMWL_powerOff(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
	mmwl_devHdl = NULL;

	if (deviceMap == 1)
	{
		retVal = rlDevicePowerOff();
		if (retVal != RL_RET_CODE_OK)
		{
			printf("Device map %u : Power Off API failed with error code %d \n\n",
				deviceMap, retVal);
			return -1;
		}
		else
		{
			printf("Device map %u : Power Off API success\n\n", deviceMap);
			mmwl_bInitComp = mmwl_bInitComp & (~deviceMap);
			mmwl_bStartComp = mmwl_bStartComp & (~deviceMap);
			mmwl_bRfInitComp = mmwl_bRfInitComp & (~deviceMap);
			DeleteCriticalSection(&rlAsyncEvent);
		}		
	}
	else
	{
		retVal = CALL_API(API_TYPE_B | REMOVE_DEVICE_IND, deviceMap, NULL, 0);
		if (retVal != RL_RET_CODE_OK)
		{
			printf("Device map %u : Power Off API failed with error code %d \n\n",
				deviceMap, retVal);
			return -1;
		}
		else
		{
			printf("Device map %u : Power Off API success\n\n", deviceMap);
			mmwl_bInitComp = mmwl_bInitComp & (~deviceMap);
			mmwl_bStartComp = mmwl_bStartComp & (~deviceMap);
			mmwl_bRfInitComp = mmwl_bRfInitComp & (~deviceMap);
		}	
	}   

    return retVal;
}

/** @fn int MMWL_lowPowerConfig(deviceMap)
*
*   @brief LowPower configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   LowPower configuration API.
*/
int MMWL_lowPowerConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;
    /* TBD - Read GUI Values */
    rlLowPowerModeCfg_t rfLpModeCfgArgs = { 0 };

    /*read rfLpModeCfgArgs from config file*/
    MMWL_readLowPowerConfig(&rfLpModeCfgArgs);

	retVal = CALL_API(SET_LOW_POWER_MODE_IND, deviceMap, &rfLpModeCfgArgs, 0);
    return retVal;
}

/** @fn int MMWL_ApllSynthBwConfig(deviceMap)
*
*   @brief APLL Synth BW configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   APLL Synth BW configuration API.
*/
int MMWL_ApllSynthBwConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;

    rlRfApllSynthBwControl_t rfApllSynthBwCfgArgs = { 0 };
    rfApllSynthBwCfgArgs.synthIcpTrim = 3;
    rfApllSynthBwCfgArgs.synthRzTrim = 8;
    rfApllSynthBwCfgArgs.apllIcpTrim = 38;
    rfApllSynthBwCfgArgs.apllRzTrimLpf = 9;
	rfApllSynthBwCfgArgs.apllRzTrimVco = 0;
    
	retVal = CALL_API(RF_SET_APLL_SYNTH_BW_CTL_CONFIG_IND, deviceMap, &rfApllSynthBwCfgArgs, 0);
    return retVal;
}

int MMWL_DevicePowerUp(unsigned int deviceMap)
{
	int retVal = RL_RET_CODE_OK;
	TDADevHandle_t TDAImpl_devHdl = NULL;
	unsigned int devId = getDevIdFromDevMap(deviceMap);
	TDAImpl_devHdl = TDAGetDeviceCtx(devId);

	int SOPmode = 4;									/* Only SOP4 is supported for cascade */

	/* Set SOP Mode for the devices */
	if (TDAImpl_devHdl != NULL)
	{
		retVal = setSOPMode(TDAImpl_devHdl, SOPmode);
		osiSleep(1); // Additional 1 msec delay 
	}
	else
	{
		printf("Device map %u : Cannot get device context\n\n", deviceMap);
		return -1;
	}

	if (retVal != RL_RET_CODE_OK)
	{
		printf("Device map %u : SOP 4 mode failed with error %d\n\n", deviceMap,
			retVal);
		return -1;
	}
	else
	{
		printf("Device map %u : SOP 4 mode successful\n\n", deviceMap);
	}

	/* Reset the devices */
	if (TDAImpl_devHdl != NULL)
	{
		retVal = resetDevice(TDAImpl_devHdl);
	}
	else
	{
		printf("Device map %u : Cannot get device context\n\n", deviceMap);
		return -1;
	}

	if (retVal != RL_RET_CODE_OK)
	{
		printf("Device map %u : Device reset failed with error %d \n\n", deviceMap,
			retVal);
		return -1;
	}
	else
	{
		printf("Device map %u : Device reset successful\n\n", deviceMap);
	}

	/*  \subsection     api_sequence1     Seq 1 - Call Power ON API
	The mmWaveLink driver initializes the internal components, creates Mutex/Semaphore,
	initializes buffers, register interrupts, bring mmWave front end out of reset.
	*/
	if (deviceMap == 1)
	{
		retVal = MMWL_powerOnMaster(deviceMap);
		if (retVal != RL_RET_CODE_OK)
		{
			printf("Device map %u : mmWave Device Power on failed with error %d \n\n",
				deviceMap, retVal);
			return -1;
		}
		else
		{
			printf("Device map %u : mmWave Device Power on success\n\n",
				deviceMap);
		}
	}
	else
	{
		retVal = CALL_API(API_TYPE_B | ADD_DEVICE_IND, deviceMap, NULL, 0);
		int timeoutCnt = 0;
		/* TBD - Wait for Power ON complete
           @Note: In case of ES1.0 sample application needs to wait for MSS CPU fault as well with some timeout.
        */
		if (RL_RET_CODE_OK == retVal)
		{
			while ((mmwl_bInitComp & deviceMap) != deviceMap)
			{
				osiSleep(1); //Sleep 1 msec
				timeoutCnt++;
				if (timeoutCnt > MMWL_API_INIT_TIMEOUT)
				{
					CALL_API(API_TYPE_B | REMOVE_DEVICE_IND, deviceMap, NULL, 0);
					retVal = RL_RET_CODE_RESP_TIMEOUT;
					break;
				}
			}
		}
		mmwl_bInitComp = mmwl_bInitComp & (~deviceMap);

		if (retVal != RL_RET_CODE_OK)
		{
			printf("Device map %u : mmWave Device Power on failed with error %d \n\n",
				deviceMap, retVal);
			return -1;
		}
		else
		{
			printf("Device map %u : mmWave Device Power on success\n\n",
				deviceMap);
		}
	}

	return retVal;
}

int MMWL_DeviceInit1(unsigned int deviceMap)
{
	int retVal = RL_RET_CODE_OK;
	/*  \subsection     api_sequence2     Seq 2 - Download FIrmware/patch (Optional)
			The mmWave device firmware is ROMed and also can be stored in External Flash. This
			step is necessary if firmware needs to be patched and patch is not stored in serial
			Flash
	*/
	if (rlDevGlobalCfgArgs.EnableFwDownload)
	{
		printf("==========================Firmware Download==========================\n\n");
		retVal = MMWL_firmwareDownload(deviceMap);
		if (retVal != RL_RET_CODE_OK)
		{
			printf("Device map %u : Firmware update failed with error %d \n\n",
				deviceMap, retVal);
			return -1;
		}
		else
		{
			printf("Device map %u : Firmware update successful\n\n",
				deviceMap);
		}
		printf("=====================================================================\n\n");
	}

	/* Change CRC Type of Async Event generated by MSS to what is being requested by user in mmwaveconfig.txt */
	retVal = MMWL_setDeviceCrcType(deviceMap);
	if (retVal != RL_RET_CODE_OK)
	{
		printf("Device map %u : CRC Type set for MasterSS failed with error code %d \n\n",
			deviceMap, retVal);
		return -1;
	}
	else
	{
		printf("Device map %u : CRC Type set for MasterSS success\n\n", deviceMap);
	}

	/*  \subsection     api_sequence3     Seq 3 - Enable the mmWave Front end (Radar/RF subsystem)
	The mmWave Front end once enabled runs boot time routines and upon completion sends asynchronous event
	to notify the result
	*/
	retVal = MMWL_rfEnable(deviceMap);
	if (retVal != RL_RET_CODE_OK)
	{
		printf("Device map %u : Radar/RF subsystem Power up failed with error %d \n\n",
			deviceMap, retVal);
		return -1;
	}
	else
	{
		printf("Device map %u : Radar/RF subsystem Power up successful\n\n", deviceMap);
	}

	/*  \subsection     api_sequence4     Seq 4 - Basic/Static Configuration
	The mmWave Front end needs to be configured for mmWave Radar operations. basic
	configuration includes Rx/Tx channel configuration, ADC configuration etc
	*/
	printf("======================Basic/Static Configuration======================\n\n");
	unsigned int cascade;
	if (deviceMap == 1)
	{
		cascade = 1;
	}
	else
	{
		cascade = 2;
	}

	/* Set which Rx and Tx channels will be enable of the device */
	retVal = MMWL_channelConfig(deviceMap, cascade);
	if (retVal != RL_RET_CODE_OK)
	{
		printf("Device map %u : Channel Config failed with error code %d\n\n",
			deviceMap, retVal);
		return -1;
	}
	else
	{
		printf("Device map %u : Channel Configuration success\n\n", deviceMap);
	}

	return retVal;
}

int MMWL_DeviceInit2(unsigned int deviceMap)
{
	int retVal = RL_RET_CODE_OK;

	retVal = MMWL_basicConfiguration(deviceMap);
	if (retVal != RL_RET_CODE_OK)
	{
		printf("Device map %u : Basic/Static configuration failed with error %d \n\n",
			deviceMap, retVal);
		return -1;
	}
	else
	{
		printf("Device map %u : Basic/Static configuration success\n\n",
			deviceMap);
	}

	/*  \subsection     api_sequence5     Seq 5 - Initializes the mmWave Front end
	The mmWave Front end once configured needs to be initialized. During initialization
	mmWave Front end performs calibration and once calibration is complete, it
	notifies the application using asynchronous event
	*/
	retVal = MMWL_rfInit(deviceMap);
	if (retVal != RL_RET_CODE_OK)
	{
		printf("Device map %u : RF Initialization/Calibration failed with error code %d \n\n",
			deviceMap, retVal);
		return -1;
	}
	else
	{
		printf("Device map %u : RF Initialization/Calibration successful\n\n", deviceMap);
	}

	return retVal;
}

int MMWL_DeviceConfig(unsigned int deviceMap)
{
	int retVal = RL_RET_CODE_OK;

	if (rlDevGlobalCfgArgs.LinkContModeTest == FALSE)
	{
		/*  \subsection     api_sequence6     Seq 6 - Configures the programmable filter */
		printf("==================Programmable Filter Configuration==================\n\n");
		retVal = MMWL_progFiltConfig(deviceMap);
		if (retVal != RL_RET_CODE_OK)
		{
			printf("Device map %u : Programmable Filter Configuration failed with error code %d \n\n",
				deviceMap, retVal);
			return -1;
		}
		else
		{
			printf("Device map %u : Programmable Filter Configuration success\n\n", deviceMap);
		}

		/*  \subsection     api_sequence7     Seq 7 - Configures the programmable filter RAM coefficients */
		retVal = MMWL_progFiltCoeffRam(deviceMap);
		if (retVal != RL_RET_CODE_OK)
		{
			printf("Device map %u : Programmable Filter coefficient RAM Configuration failed with error code %d \n\n",
				deviceMap, retVal);
			return -1;
		}
		else
		{
			printf("Device map %u : Programmable Filter coefficient RAM Configuration success\n\n", deviceMap);
		}

		/*  \subsection     api_sequence8     Seq 8 - FMCW profile configuration
		TI mmWave devices supports Frequency Modulated Continuous Wave(FMCW) Radar. User
		Need to define characteristics of FMCW signal using profile configuration. A profile
		contains information about FMCW signal such as Start Frequency (76 - 81 GHz), Ramp
		Slope (e.g 30MHz/uS). Idle Time etc. It also configures ADC samples, Sampling rate,
		Receiver gain, Filter configuration parameters

		\ Note - User can define upto 4 different profiles
		*/
		printf("======================FMCW Configuration======================\n\n");
		retVal = MMWL_profileConfig(deviceMap);
		if (retVal != RL_RET_CODE_OK)
		{
			printf("Device map %u : Profile Configuration failed with error code %d \n\n",
				deviceMap, retVal);
			return -1;
		}
		else
		{
			printf("Device map %u : Profile Configuration success\n\n", deviceMap);
		}

		if (rlDevGlobalCfgArgs.LinkAdvChirpTest == FALSE)
		{
			/*  \subsection     api_sequence9     Seq 9 - FMCW chirp configuration
			A chirp is always associated with FMCW profile from which it inherits coarse information
			about FMCW signal. Using chirp configuration user can further define fine
			variations to coarse parameters such as Start Frequency variation(0 - ~400 MHz), Ramp
			Slope variation (0 - ~3 MHz/uS), Idle Time variation etc. It also configures which transmit channels to be used
			for transmitting FMCW signal.

			\ Note - User can define upto 512 unique chirps
			*/
			retVal = MMWL_chirpConfig(deviceMap);
			if (retVal != RL_RET_CODE_OK)
			{
				printf("Device map %u : Chirp Configuration failed with error %d \n\n",
					deviceMap, retVal);
				return -1;
			}
			else
			{
				printf("Device map %u : Chirp Configuration success\n\n", deviceMap);
			}
		}
		else
		{
			retVal = MMWL_advChirpConfigAll(deviceMap);
			if (retVal != RL_RET_CODE_OK)
			{
				printf("Device map %u : Advanced Chirp Configuration failed with error %d \n\n",
					deviceMap, retVal);
				return -1;
			}
			else
			{
				printf("Device map %u : Advanced Chirp Configuration success\n\n", deviceMap);
			}
		}
	}

	/*  \subsection     api_sequence10     Seq 10 - Data Path (CSI2/LVDS) Configuration
	TI mmWave device supports CSI2 or LVDS interface for sending RAW ADC data. mmWave device
	can also send Chirp Profile and Chirp Quality data over LVDS/CSI2. User need to select
	the high speed interface and also what data it expects to receive.

	\ Note - This API is only applicable for AWR2243 when mmWaveLink driver is running on External Host
	*/
	printf("==================Data Path(LVDS/CSI2) Configuration==================\n\n");
	retVal = MMWL_dataPathConfig(deviceMap);
	if (retVal != RL_RET_CODE_OK)
	{
		printf("Device map %u : Data Path Configuration failed with error %d \n\n",
			deviceMap, retVal);
		return -1;
	}
	else
	{
		printf("Device map %u : Data Path Configuration successful\n\n", deviceMap);
	}

	/*  \subsection     api_sequence11     Seq 11 - CSI2/LVDS CLock and Data Rate Configuration
	User need to configure what data rate is required to send the data on high speed interface. For
	e.g 150mbps, 300mbps etc.
	\ Note - This API is only applicable for AWR2243 when mmWaveLink driver is running on External Host
	*/
	retVal = MMWL_hsiClockConfig(deviceMap);
	if (retVal != RL_RET_CODE_OK)
	{
		printf("Device map %u : CSI2/LVDS Clock Configuration failed with error %d \n\n",
			deviceMap, retVal);
		return -1;
	}
	else
	{
		printf("Device map %u : CSI2/LVDS Clock Configuration success\n\n", deviceMap);
	}

	/*  \subsection     api_sequence12     Seq 12 - CSI2/LVDS Lane Configuration
	User need to configure how many LVDS/CSI2 lane needs to be enabled
	\ Note - This API is only applicable for AWR2243 when mmWaveLink driver is running on External Host
	*/
	retVal = MMWL_hsiLaneConfig(deviceMap);
	if (retVal != RL_RET_CODE_OK)
	{
		printf("Device map %u : CSI2/LVDS Lane Config failed with error %d \n\n",
			deviceMap, retVal);
		return -1;
	}
	else
	{
		printf("Device map %u : CSI2/LVDS Lane Configuration success\n\n",
			deviceMap);
	}
	printf("======================================================================\n\n");

#ifdef ENABLE_TEST_SOURCE
	retVal = MMWL_testSourceConfig(deviceMap);
	if (retVal != RL_RET_CODE_OK)
	{
		printf("Device map %u : Test Source Configuration failed with error %d \n\n",
			deviceMap, retVal);
		return -1;
	}
	else
	{
		printf("Device map %u : Test source Configuration success\n\n", deviceMap);
	}
#endif

    if (rlDevGlobalCfgArgs.LinkContModeTest == TRUE)
	{
		retVal = MMWL_setContMode(deviceMap);
		if (retVal != RL_RET_CODE_OK)
		{
			printf("Device map %u : Continuous mode Config failed with error code %d \n\n",
				deviceMap, retVal);
			return -1;
		}
		else
		{
			printf("Device map %u : Continuous mode Config successful\n\n", deviceMap);
		}
	}

	return retVal;
}

int setWidthAndHeightContStream()
{
	int retVal = RL_RET_CODE_OK;
	unsigned char devId;
	mmwl_TDA_framePeriodicity = 100;
	/* Number of samples to be captured at one shot during continous streaming of AWR device.
	   User can change this according to their requirement. */
	unsigned int numAdcSamples_ContStream = 524288; /* half a million samples */

	for (devId = 0; devId < 4; devId++)
	{
		/* Width calculation */
		/* Count the number of Rx antenna */
		unsigned char numRxAntenna = 0;
		rlChanCfg_t rfChanCfgArgs = { 0 };
		MMWL_readChannelConfig(&rfChanCfgArgs, 0);
		while (rfChanCfgArgs.rxChannelEn != 0)
		{
			if ((rfChanCfgArgs.rxChannelEn & 0x1) == 1)
			{
				numRxAntenna++;
			}
			rfChanCfgArgs.rxChannelEn = (rfChanCfgArgs.rxChannelEn >> 1);
		}

		/* ADC format (in bytes) */
		unsigned char numValPerAdcSample = 0, numAdcBits = 0;
		rlAdcOutCfg_t adcOutCfgArgs = { 0 };
		MMWL_readAdcOutConfig(&adcOutCfgArgs);
		if (adcOutCfgArgs.fmt.b2AdcOutFmt == 1 || adcOutCfgArgs.fmt.b2AdcOutFmt == 2)
		{
			numValPerAdcSample = 2;
		}
		else
		{
			numValPerAdcSample = 1;
		}

		if (adcOutCfgArgs.fmt.b2AdcBits == 0)
		{
			numAdcBits = 12;
		}
		else if (adcOutCfgArgs.fmt.b2AdcBits == 1)
		{
			numAdcBits = 14;
		}
		else if (adcOutCfgArgs.fmt.b2AdcBits == 2)
		{
			numAdcBits = 16;
		}

		/* Get CP and CQ value */
		unsigned short cp_data = 0, cq_val = 0;
		unsigned int cq_data = 0;
		rlDevDataPathCfg_t dataPathCfgArgs = { 0 };
		MMWL_readDataPathConfig(&dataPathCfgArgs);
		cq_val = dataPathCfgArgs.cq0TransSize + dataPathCfgArgs.cq1TransSize + dataPathCfgArgs.cq2TransSize;

		if (dataPathCfgArgs.transferFmtPkt0 == 6 || dataPathCfgArgs.transferFmtPkt0 == 9)
		{
			cp_data = 2;
			cq_data = 0;
		}
		else if (dataPathCfgArgs.transferFmtPkt0 == 54)
		{
			cp_data = 2;
			cq_data = (cq_val * 16) / numAdcBits;
		}

		mmwl_TDA_width[devId] = (((numValPerAdcSample * numAdcSamples_ContStream) + cp_data) * numRxAntenna) + cq_data;

		/*  From TDA2 data capture card stand point, it is not aware of the type of data transmitted by the AWR.
			So, we maintain the same abstraction with the TDA and capture all of the samples requested during continous streaming  
			as if we are capturing one frame worth of data. 
			Since TDA2 data capture card can only accomodate a maximum of 4096 bytes as width, any number of bytes requested > 4096, 
			needs to be accomodated in the height parameter. */
		if (mmwl_TDA_width[devId] > 4096)
		{
			mmwl_TDA_height[devId] = (mmwl_TDA_width[devId] / 4096);
			mmwl_TDA_width[devId] = 4096;
		}
		else /* If total number of bytes to be captured is less than 4096, then height component is not required */
		{
			mmwl_TDA_height[devId] = 1;
		}

		printf("Device map %u : Calculated TDA Width is %d\n\n", (1 << devId), mmwl_TDA_width[devId]);
		printf("Device map %u : Calculated TDA Height is %d\n\n", (1 << devId), mmwl_TDA_height[devId]);

		/* Store the time to wait in ms for TDA to capture the continous streaming data from AWR */
		gContStreamTime = ((mmwl_TDA_height[devId] * mmwl_TDA_width[devId]) / gContStreamSampleRate);
	}
	return retVal;

}

int MMWL_ArmingTDA()
{
	int retVal = RL_RET_CODE_OK;
	int timeOutCnt = 0U;

	if (rlDevGlobalCfgArgs.LinkContModeTest == TRUE)
	{
		retVal = setWidthAndHeightContStream();
		if (retVal != RL_RET_CODE_OK)
		{
			printf("Device map %u : Setting width & Height in Continuous mode Config failed with error code %d \n\n",
				mmwl_TDA_DeviceMapCascadedAll, retVal);
			return -1;
		}
		else
		{
			printf("Device map %u : Setting width & Height in Continuous mode Config successful\n\n", mmwl_TDA_DeviceMapCascadedAll);
		}
	}

	/* Set width and height for all devices*/

	/* Master */
	retVal = setWidthAndHeight(1, mmwl_TDA_width[0], mmwl_TDA_height[0]);
	if (retVal != RL_RET_CODE_OK)
	{
		printf("ERROR: Device map 1 : Setting width = %u and height = %u failed with error code %d \n\n", mmwl_TDA_width[0], mmwl_TDA_height[0], retVal);
		return -1;
	}
	else
	{
		printf("INFO: Device map 1 : Setting width = %u and height = %u successful\n\n", mmwl_TDA_width[0], mmwl_TDA_height[0]);
	}

	for (int i = 0; i < 3; i++)
	{
		if (mmwl_TDA_SlavesEnabled[i] == 1)
		{
			retVal = setWidthAndHeight(1 << (i + 1), mmwl_TDA_width[i+1], mmwl_TDA_height[i+1]);
			if (retVal != RL_RET_CODE_OK)
			{
				printf("ERROR: Device map %u : Setting width = %u and height = %u failed with error code %d \n\n", 1 << (i + 1), mmwl_TDA_width[i+1], mmwl_TDA_height[i+1], retVal);
				return -1;
			}
			else
			{
				printf("INFO: Device map %u : Setting width = %u and height = %u successful\n\n", 1 << (i + 1), mmwl_TDA_width[i+1], mmwl_TDA_height[i+1]);
			}
		}
	}

	timeOutCnt = 0U;
	mmwl_bTDA_FramePeriodicityACK = 0U;
	/* Send frame periodicity for syncing the data being received at VIP ports */
	retVal = sendFramePeriodicitySync(mmwl_TDA_framePeriodicity);
	if (retVal != RL_RET_CODE_OK)
	{
		printf("ERROR: Sending framePeriodicity = %u failed with error code %d \n\n", mmwl_TDA_framePeriodicity, retVal);
		return -1;
	}
	else
	{
		printf("INFO: Sending framePeriodicity = %u successful \n\n", mmwl_TDA_framePeriodicity);
	}

	while (1)
	{
		if (mmwl_bTDA_FramePeriodicityACK == 0U)
		{
			osiSleep(1); /*Sleep 1 msec*/
			timeOutCnt++;
			if (timeOutCnt > MMWL_API_TDA_TIMEOUT)
			{
				printf("ERROR: Frame Periodicity Response from Capture Card timed out!\n\n");
				retVal = RL_RET_CODE_RESP_TIMEOUT;
				return retVal;
			}
		}
		else
		{
			break;
		}
	}

	timeOutCnt = 0U;
	mmwl_bTDA_CaptureDirectoryACK = 0U;
	/* Send session's capture directory to TDA */
	retVal = setSessionDirectory(mmwl_TDA_CaptureDirectory);
	if (retVal != RL_RET_CODE_OK)
	{
		printf("ERROR: Sending capture directory = %s failed with error code %d \n\n", mmwl_TDA_CaptureDirectory, retVal);
		return -1;
	}
	else
	{
		printf("INFO: Sending capture directory = %s successful \n\n", mmwl_TDA_CaptureDirectory);
	}

	while (1)
	{
		if (mmwl_bTDA_CaptureDirectoryACK == 0U)
		{
			osiSleep(1); /*Sleep 1 msec*/
			timeOutCnt++;
			if (timeOutCnt > MMWL_API_TDA_TIMEOUT)
			{
				printf("ERROR: Capture Directory Response from Capture Card timed out!\n\n");
				retVal = RL_RET_CODE_RESP_TIMEOUT;
				return retVal;
			}
		}
		else
		{
			break;
		}
	}

	timeOutCnt = 0U;
	mmwl_bTDA_FileAllocationACK = 0U;
	/* Send number of files to be pre-allocated to TDA */
	retVal = sendNumAllocatedFiles(mmwl_TDA_numAllocatedFiles);
	if (retVal != RL_RET_CODE_OK)
	{
		printf("ERROR: Sending pre-allocated files = %u failed with error code %d \n\n", mmwl_TDA_numAllocatedFiles, retVal);
		return -1;
	}
	else
	{
		printf("INFO: Sending pre-allocated files = %u successful \n\n", mmwl_TDA_numAllocatedFiles);
	}

	while (1)
	{
		if (mmwl_bTDA_FileAllocationACK == 0U)
		{
			osiSleep(1); /*Sleep 1 msec*/
			timeOutCnt++;
			if (timeOutCnt > MMWL_API_TDA_TIMEOUT)
			{
				printf("ERROR: File Allocation Response from Capture Card timed out!\n\n");
				retVal = RL_RET_CODE_RESP_TIMEOUT;
				return retVal;
			}
		}
		else
		{
			break;
		}
	}

	timeOutCnt = 0U;
	mmwl_bTDA_DataPackagingACK = 0U;
	/* Send enable Data packing (0 : 16-bit, 1 : 12-bit) to TDA */
	retVal = enableDataPackaging(mmwl_TDA_enableDataPacking);
	if (retVal != RL_RET_CODE_OK)
	{
		printf("ERROR: Sending enable data packing = %u failed with error code %d \n\n", mmwl_TDA_enableDataPacking, retVal);
		return -1;
	}
	else
	{
		printf("INFO: Sending enable data packing = %u successful \n\n", mmwl_TDA_enableDataPacking);
	}

	while (1)
	{
		if (mmwl_bTDA_DataPackagingACK == 0U)
		{
			osiSleep(1); /*Sleep 1 msec*/
			timeOutCnt++;
			if (timeOutCnt > MMWL_API_TDA_TIMEOUT)
			{
				printf("ERROR: Enable Data Packaging Response from Capture Card timed out!\n\n");
				retVal = RL_RET_CODE_RESP_TIMEOUT;
				return retVal;
			}
		}
		else
		{
			break;
		}
	}

	timeOutCnt = 0U;
	mmwl_bTDA_NumFramesToCaptureACK = 0U;
	/* Send number of frames to be captured by TDA */
	retVal = NumFramesToCapture(mmwl_TDA_numFramesToCapture);
	if (retVal != RL_RET_CODE_OK)
	{
		printf("ERROR: Sending number of frames to capture = %u failed with error code %d \n\n", mmwl_TDA_numFramesToCapture, retVal);
		return -1;
	}
	else
	{
		printf("INFO: Sending number of frames to capture = %u successful \n\n", mmwl_TDA_numFramesToCapture);
	}

	while (1)
	{
		if (mmwl_bTDA_NumFramesToCaptureACK == 0U)
		{
			osiSleep(1); /*Sleep 1 msec*/
			timeOutCnt++;
			if (timeOutCnt > MMWL_API_TDA_TIMEOUT)
			{
				printf("ERROR: Number of frames to be captured Response from Capture Card timed out!\n\n");
				retVal = RL_RET_CODE_RESP_TIMEOUT;
				return retVal;
			}
		}
		else
		{
			break;
		}
	}

	timeOutCnt = 0U;
	mmwl_bTDA_CreateAppACK = 0U;
	/* Notify TDA about creating the application */
	retVal = TDACreateApplication();
	if (retVal != RL_RET_CODE_OK)
	{
		printf("ERROR: Notifying TDA about creating application failed with error code %d \n\n", retVal);
		return -1;
	}
	else
	{
		printf("INFO: Notifying TDA about creating application successful \n\n");
	}

	while (1)
	{
		if (mmwl_bTDA_CreateAppACK == 0U)
		{
			osiSleep(1); /*Sleep 1 msec*/
			timeOutCnt++;
			if (timeOutCnt > MMWL_API_TDA_TIMEOUT*3)
			{
				printf("ERROR: Create Application Response from Capture Card timed out!\n\n");
				retVal = RL_RET_CODE_RESP_TIMEOUT;
				return retVal;
			}
		}
		else
		{
			break;
		}
	}

	timeOutCnt = 0U;
	mmwl_bTDA_StartRecordACK = 0U;
	/* Notify TDA about starting the frame */
	retVal = startRecord();
	if (retVal != RL_RET_CODE_OK)
	{
		printf("ERROR: Notifying TDA about start frame failed with error code %d \n\n", retVal);
		return -1;
	}
	else
	{
		printf("INFO: Notifying TDA about start frame successful \n\n");
		mmwl_bTDA_ARMDone = 1U;
	}

	while (1)
	{
		if (mmwl_bTDA_StartRecordACK == 0U)
		{
			osiSleep(1); /*Sleep 1 msec*/
			timeOutCnt++;
			if (timeOutCnt > MMWL_API_TDA_TIMEOUT)
			{
				printf("ERROR: Start Record Response from Capture Card timed out!\n\n");
				retVal = RL_RET_CODE_RESP_TIMEOUT;
				return retVal;
			}
		}
		else
		{
			break;
		}
	}

	return retVal;
}

int MMWL_ContModeEnable(unsigned int deviceMap, unsigned short contModeEn)
{
	int retVal = RL_RET_CODE_OK;

	rlDevContStreamingModeCfg_t contStreamingModeCfgArgs = { 0 };
	rlContModeEn_t contModeEnArgs = { 0 };

	/* MSS API */
	contStreamingModeCfgArgs.contStreamModeEn = contModeEn;
	retVal = CALL_API(SET_CONT_STREAM_MODE_CONFIG_IND, deviceMap, &contStreamingModeCfgArgs, 0);

	/* BSS API */
	contModeEnArgs.contModeEn = contModeEn;
	retVal = CALL_API(ENABLE_CONT_MODE_IND, deviceMap, &contModeEnArgs, 0);

	if (retVal != RL_RET_CODE_OK)
	{
		printf("Device map %u : Continuous streaming start/stop failed with error code %d \n\n",
			deviceMap, retVal);
		return -1;
	}
	else
	{
		printf("Device map %u : Continuous streaming start/stop successful \n\n",
			deviceMap);
	}

	return retVal;
}

int MMWL_StartFrame(unsigned int deviceMap)
{
	int retVal = RL_RET_CODE_OK;

	/*  \subsection     api_sequence14     Seq 14 - Start mmWave Radar Sensor
	This will trigger the mmWave Front to start transmitting FMCW signal. Raw ADC samples
	would be received from Digital front end. For AWR2243, if high speed interface is
	configured, RAW ADC data would be transmitted over CSI2/LVDS. On xWR1443/xWR1642, it can
	be processed using HW accelerator or DSP
	*/
	retVal = MMWL_sensorStart(deviceMap);
	if (retVal != RL_RET_CODE_OK)
	{
		printf("Device map %u : Sensor Start failed with error code %d \n\n",
			deviceMap, retVal);
		return -1;
	}

	return retVal;
}

int MMWL_DynConfig(unsigned int deviceMap)
{
	int retVal = RL_RET_CODE_OK;

    if (rlDevGlobalCfgArgs.LinkDynProfileTest == TRUE)
	{
		/* Host can update profile configurations dynamically while frame is ongoing.
		   This test has been added in this example to demostrate dynamic profile update feature
		   of mmWave sensor device, developer must check the validity of parameters at the system
		   level before implementing the application. */

		   /* wait for few frames worth of time before updating profile config */
		osiSleep(3 * framePeriodicity);

		/* update few of existing profile parameters */
		profileCfgArgs[0].rxGain = 158; /* 30 dB gain and 36 dB Gain target */
		profileCfgArgs[0].pfCalLutUpdate = 0x1; /* bit0: 1, bit1: 0 */
		profileCfgArgs[0].hpfCornerFreq1 = 1;
		profileCfgArgs[0].hpfCornerFreq2 = 1;
		profileCfgArgs[0].txStartTime = 2;
		profileCfgArgs[0].rampEndTime = 7000;

		/* Dynamically configure 1 profile (max 4 profiles) while frame is ongoing */
		retVal = CALL_API(API_TYPE_C | SET_PROFILE_CONFIG_IND, deviceMap, &profileCfgArgs[0U], 1U);
		if (retVal != RL_RET_CODE_OK)
		{
			printf("Device map %u : Dynamic Profile Configuration failed with error code %d \n\n",
				deviceMap, retVal);
			return -1;
		}
		else
		{
			printf("Device map %u : Dynamic Profile Configuration success\n\n", deviceMap);
		}

		/* wait for few frames worth of time before reading profile config.
		   Dynamic profile configuration will come in effect during next frame, so wait for that time
		   before reading back profile config */
		osiSleep(2 * framePeriodicity);

		/* To verify that profile configuration parameters are applied to device while frame is ongoing,
		   read back profile configurationn from device */
		retVal = CALL_API(API_TYPE_C | GET_PROFILE_CONFIG_IND, deviceMap, &profileCfgArgs[1], 0U);
		if (retVal != RL_RET_CODE_OK)
		{
			printf("Device map %u : Dynamic Get Profile Configuration failed with error code %d \n\n",
				deviceMap, retVal);
			return -1;
		}
		else
		{
			printf("Device map %u : Dynamic Get Profile Configuration success\n\n", deviceMap);
			/* compare the read back profile configuration parameters to lastly configured parameters */
			if ((profileCfgArgs[0].rxGain != profileCfgArgs[1].rxGain) || \
				(profileCfgArgs[0].hpfCornerFreq1 != profileCfgArgs[1].hpfCornerFreq1) || \
				(profileCfgArgs[0].hpfCornerFreq2 != profileCfgArgs[1].hpfCornerFreq2) || \
				(profileCfgArgs[0].txStartTime != profileCfgArgs[1].txStartTime) || \
				(profileCfgArgs[0].rampEndTime != profileCfgArgs[1].rampEndTime))
				printf("Dynamic Profile Config mismatched !!! \n\n");
			else
				printf("Dynamic profile cfg matched \n\n");
		}
	}

    if (rlDevGlobalCfgArgs.LinkDynChirpTest == TRUE)
	{
		/* wait for few frames to elapse before invoking Dynamic chirp config API to update
		   new chirp config to come in effect for next frames */

		   /* wait for few frames worth of time */
		osiSleep(3 * framePeriodicity);

		retVal = MMWL_setDynChirpConfig(deviceMap);
		if (retVal != RL_RET_CODE_OK)
		{
			printf("Device map %u : Dynamic Chirp config failed with error code %d \n\n",
				deviceMap, retVal);
			return -1;
		}
		else
		{
			printf("Device map %u : Dynamic Chirp config successful\n\n", deviceMap);
		}
		printf("======================================================================\n\n");

		retVal = MMWL_dynChirpEnable(deviceMap);
		if (retVal != RL_RET_CODE_OK)
		{
			printf("Device map %u : Dynamic Chirp Enable failed with error code %d \n\n",
				deviceMap, retVal);
			return -1;
		}
		else
		{
			printf("Device map %u : Dynamic Chirp Enable successful\n\n", deviceMap);
		}
		printf("======================================================================\n\n");

		/* wait for another few mSec so that dynamic chirp come in effect,
		 If above API reached to BSS at the end of frame then new chirp config will come in effect
		 during next frame only */
		osiSleep(2 * framePeriodicity);
	}

	return retVal;
}

int MMWL_DynAdvConfig(unsigned int deviceMap)
{
	int retVal = RL_RET_CODE_OK;

    if (rlDevGlobalCfgArgs.LinkAdvChirpTest == TRUE)
	{
		/* wait for few frames to elapse before invoking Dynamic Advance chirp offset config API 
           to update new chirp config to come in effect for next frames */

		   /* wait for few frames worth of time */
		osiSleep(3 * framePeriodicity);

        if (gDynAdvChirpLUTBufferDir == 0) /* PING Buffer */
        {
            retVal = MMWL_setDynAdvChirpOffsetConfig(deviceMap, &advChirpDynLUTOffsetCfg1);
            gDynAdvChirpLUTBufferDir = 1; /* update the buffer direction to point to the PONG buffer next time */
        }
        else /* PONG Buffer */
        {
            retVal = MMWL_setDynAdvChirpOffsetConfig(deviceMap, &advChirpDynLUTOffsetCfg2);
            gDynAdvChirpLUTBufferDir = 0; /* update the buffer direction to point to the PING buffer next time */ 
        }

		if (retVal != RL_RET_CODE_OK)
		{
			printf("Device map %u : Dynamic Advance Chirp offset config failed with error code %d \n\n",
				deviceMap, retVal);
			return -1;
		}
		else
		{
			printf("Device map %u : Dynamic Advance Chirp offset config successful\n\n", deviceMap);
		}
		printf("======================================================================\n\n");

		retVal = MMWL_dynChirpEnable(deviceMap);
		if (retVal != RL_RET_CODE_OK)
		{
			printf("Device map %u : Dynamic Chirp Enable failed with error code %d \n\n",
				deviceMap, retVal);
			return -1;
		}
		else
		{
			printf("Device map %u : Dynamic Chirp Enable successful\n\n", deviceMap);
		}
		printf("======================================================================\n\n");

		/* wait for another few mSec so that dynamic advance chirp offset come in effect,
		 If above API reached to BSS at the end of frame then new chirp config will come in effect
		 during next frame only */
		osiSleep(2 * framePeriodicity);
	}

	return retVal;
}

int MMWL_StopFrame(unsigned int deviceMap)
{
	int retVal = RL_RET_CODE_OK;

	/* Stop the frame */
	retVal = MMWL_sensorStop(deviceMap);
	if (retVal != RL_RET_CODE_OK)
	{
		if (retVal == RL_RET_CODE_FRAME_ALREADY_ENDED)
		{
			printf("Device map %u : Frame is already stopped when sensorStop CMD was issued\n\n", deviceMap);
		}
		else
		{
			printf("Device map %u : Sensor Stop failed with error code %d \n\n",
				deviceMap, retVal);
			return -1;
		}
	}
	else
	{
		printf("Device map %u : Sensor Stop successful\n\n", deviceMap);
	}

	return retVal;
}

int MMWL_DeArmingTDA()
{
	int retVal = RL_RET_CODE_OK;
	int timeOutCnt = 0;
	mmwl_bTDA_StopRecordACK = 0U;

	/* Notify TDA about stopping the frame */
	retVal = stopRecord();
	if (retVal != RL_RET_CODE_OK)
	{
		printf("ERROR: Notifying TDA about stop frame failed with error code %d \n\n", retVal);
		return -1;
	}
	else
	{
		printf("INFO: Notifying TDA about stop frame successful \n\n");
		mmwl_bTDA_ARMDone = 0U;
	}

	while (1)
	{
		if (mmwl_bTDA_StopRecordACK == 0U)
		{
			osiSleep(1); /*Sleep 1 msec*/
			timeOutCnt++;
			if (timeOutCnt > MMWL_API_TDA_TIMEOUT*2)
			{
				printf("ERROR: TDA Stop Record ACK not received!");
				retVal = RL_RET_CODE_RESP_TIMEOUT;
				return retVal;
			}
		}
		else
		{
			break;
		}
	}

	return retVal;
}

int MMWL_DeviceDeInit(unsigned int deviceMap)
{
	int retVal = RL_RET_CODE_OK;

	/* Note- Before Calling this API user must feed in input signal to device's pins,
	else device will return garbage data in GPAdc measurement over Async event.
	Measurement data is stored in 'rcvGpAdcData' structure after this API call. */
	retVal = MMWL_gpadcMeasConfig(deviceMap);
	if (retVal != RL_RET_CODE_OK)
	{
		printf("Device map %u : GPAdc measurement API failed with error code %d \n\n",
			deviceMap, retVal);
		return -1;
	}
	else
	{
		printf("Device map %u : GPAdc measurement API success\n\n", deviceMap);
	}

	return retVal;
}

int MMWL_TDAInit()
{
	int retVal = RL_RET_CODE_OK;
	int timeOutCnt = 0;

	/* Register Async event handler with TDA */
	retVal = registerTDAStatusCallback((TDA_EVENT_HANDLER)TDA_asyncEventHandler);
	if (retVal != RL_RET_CODE_OK)
	{
		printf("ERROR: Registering Async event handler with TDA failed with error %d \n\n",
			retVal);
		return -1;
	}
	else
	{
		printf("INFO: Registered Async event handler with TDA \n\n");
	}

	mmwl_bTDA_CaptureCardConnect = 0U;
	/* Connect to the TDA Capture card */
	retVal = ethernetConnect(mmwl_TDA_IPAddress, mmwl_TDA_ConfigPort, mmwl_TDA_DeviceMapCascadedAll);
	if (retVal != RL_RET_CODE_OK)
	{
		printf("ERROR: Connecting to TDA failed with error %d. Check whether the capture card is connected to the network! \n\n",
			retVal);
		return -1;
	}
	while (1)
	{
		if (mmwl_bTDA_CaptureCardConnect == 0U)
		{
			osiSleep(1); /*Sleep 1 msec*/
			timeOutCnt++;
			if (timeOutCnt > MMWL_API_TDA_TIMEOUT)
			{
				printf("ERROR: No Acknowlegment received from the capture card! \n\n");
				retVal = RL_RET_CODE_RESP_TIMEOUT;
				return retVal;
			}
		}
		else
		{
			break;
		}
	}

	if (retVal == RL_RET_CODE_OK)
	{
		printf("INFO: Connection to TDA successful! \n\n");
	}

	return retVal;
}

int MMWL_FrameConfigAll()
{
	int retVal = RL_RET_CODE_OK;
	/* Check for If Advance Frame Test is enabled */
	if (rlDevGlobalCfgArgs.LinkAdvanceFrameTest == FALSE)
	{
		/*  FMCW frame configuration */
		retVal = MMWL_frameConfig(mmwl_TDA_DeviceMapCascadedMaster);
		if (retVal != RL_RET_CODE_OK)
		{
			printf("Device map %u : Frame Configuration failed with error %d \n\n",
				mmwl_TDA_DeviceMapCascadedMaster, retVal);
			return -1;
		}
		else
		{
			printf("Device map %u : Frame Configuration success\n\n", mmwl_TDA_DeviceMapCascadedMaster);
		}

		if (mmwl_TDA_DeviceMapCascadedSlaves != 0)
		{
			retVal = MMWL_frameConfig(mmwl_TDA_DeviceMapCascadedSlaves);
			if (retVal != RL_RET_CODE_OK)
			{
				printf("Device map %u : Frame Configuration failed with error %d \n\n",
					mmwl_TDA_DeviceMapCascadedSlaves, retVal);
				return -1;
			}
			else
			{
				printf("Device map %u : Frame Configuration success\n\n", mmwl_TDA_DeviceMapCascadedSlaves);
			}
		}	
	}
	else
	{
		/*  \subsection     api_sequence13     Seq 13 - FMCW Advance frame configuration
		A frame defines sequence of chirps and how this sequence needs to be repeated over time.
		*/
		retVal = MMWL_advFrameConfig(mmwl_TDA_DeviceMapCascadedMaster);
		if (retVal != RL_RET_CODE_OK)
		{
			printf("Device map %u : Adv Frame Configuration failed with error %d \n\n",
				mmwl_TDA_DeviceMapCascadedMaster, retVal);
			return -1;
		}
		else
		{
			printf("Device map %u : Adv Frame Configuration success\n\n", mmwl_TDA_DeviceMapCascadedMaster);
		}

		if (mmwl_TDA_DeviceMapCascadedSlaves != 0)
		{
			retVal = MMWL_advFrameConfig(mmwl_TDA_DeviceMapCascadedSlaves);
			if (retVal != RL_RET_CODE_OK)
			{
				printf("Device map %u : Adv Frame Configuration failed with error %d \n\n",
					mmwl_TDA_DeviceMapCascadedSlaves, retVal);
				return -1;
			}
			else
			{
				printf("Device map %u : Adv Frame Configuration success\n\n", mmwl_TDA_DeviceMapCascadedSlaves);
			}
		}
	}
	printf("======================================================================\n\n");

	return retVal;
}

int MMWL_AssignDeviceMap()
{
	int retVal = RL_RET_CODE_OK;
	unsigned char deviceMap = rlDevGlobalCfgArgs.CascadeDeviceMap;
	unsigned char devId = 0;

	if ((deviceMap & 1) == 0)
	{
		return RL_RET_CODE_INVALID_INPUT;
	}

	for (devId = 0; devId < 4; devId++)
	{
		if ((deviceMap & (1 << devId)) != 0)
		{
			if (devId == 0)
			{
				mmwl_TDA_DeviceMapCascadedMaster |= (1 << devId);
				mmwl_TDA_DeviceMapCascadedAll |= (1 << devId);
			}
			else
			{
				mmwl_TDA_SlavesEnabled[devId - 1] = 1;
				mmwl_TDA_DeviceMapCascadedSlaves |= (1 << devId);
				mmwl_TDA_DeviceMapCascadedAll |= (1 << devId);
			}
		}
	}

	return retVal;
}

/** @fn int MMWL_App()
*
*   @brief mmWaveLink Example Application for Cascade.
*
*   @return int Success - 0, Failure - Error Code
*
*   mmWaveLink Example Application for Cascade.
*/

int MMWL_App()
{
    int retVal = RL_RET_CODE_OK;

	retVal = MMWL_openConfigFile();
	if (retVal != RL_RET_CODE_OK)
	{
		printf("ERROR: Failed to Open configuration file\n\n");
		return -1;
	}
	
	/* Read all global variable configurations from config file */
	MMWL_getGlobalConfigStatus(&rlDevGlobalCfgArgs);

	retVal = MMWL_AssignDeviceMap();
	if (retVal != RL_RET_CODE_OK)
	{
		printf("ERROR: Master should be enabled! \n\n");
		return -1;
	}

	retVal = MMWL_TDAInit();
	if (retVal != RL_RET_CODE_OK)
	{
		printf("ERROR: Failed to Connect with TDA\n\n");
		return -1;
	}

	MMWL_DevicePowerUp(mmwl_TDA_DeviceMapCascadedMaster);
	MMWL_DeviceInit1(mmwl_TDA_DeviceMapCascadedMaster);

	for (int i = 0; i < 3; i++)
	{
		if (mmwl_TDA_SlavesEnabled[i] == 1)
		{
			MMWL_DevicePowerUp(1 << (i + 1));
		}
	}

	if (mmwl_TDA_DeviceMapCascadedSlaves != 0)
	{
		MMWL_DeviceInit1(mmwl_TDA_DeviceMapCascadedSlaves);
	}
	MMWL_DeviceInit2(mmwl_TDA_DeviceMapCascadedAll);
	MMWL_DeviceConfig(mmwl_TDA_DeviceMapCascadedAll);

	if (rlDevGlobalCfgArgs.LinkContModeTest == TRUE)
	{
		/* Start continuous streaming for slaves */
		for (int i = 2; i >= 0; i--)
		{
			if (mmwl_TDA_SlavesEnabled[i] == 1)
			{
				MMWL_ContModeEnable(1 << (i + 1), 1);
			}
		}
		/* start streaming for master */
		MMWL_ContModeEnable(mmwl_TDA_DeviceMapCascadedMaster, 1);

		MMWL_ArmingTDA();

		/* Wait for the TDA to capture the requested number of samples */
		osiSleep((UINT32)(gContStreamTime)); /*Sleep in msec*/

		if (mmwl_bTDA_ARMDone == 1U)
		{
			MMWL_DeArmingTDA();
		}

		/* Let the device stream for some more time */
		osiSleep(5000);

		/* Stop continuous streaming for slaves */
		for (int i = 2; i >= 0; i--)
		{
			if (mmwl_TDA_SlavesEnabled[i] == 1)
			{
				MMWL_ContModeEnable(1 << (i + 1), 0);
			}
		}
		/* Stop streaming for master */
		MMWL_ContModeEnable(mmwl_TDA_DeviceMapCascadedMaster, 0);
	}
	else
	{
		MMWL_FrameConfigAll();

		MMWL_ArmingTDA();

		for (int i = 2; i >= 0; i--)
		{
			if (mmwl_TDA_SlavesEnabled[i] == 1)
			{
				MMWL_StartFrame(1 << (i + 1));
			}
		}
		MMWL_StartFrame(mmwl_TDA_DeviceMapCascadedMaster);

		for (int i = 2; i >= 0; i--)
		{
			if (mmwl_TDA_SlavesEnabled[i] == 1)
			{
				MMWL_DynConfig(1 << (i + 1));
			}
		}
		MMWL_DynConfig(mmwl_TDA_DeviceMapCascadedMaster);

		/* Wait for the frame end async event from all devices */
		while (mmwl_bSensorStarted != 0x0)
		{
			retVal = MMWL_DynAdvConfig(mmwl_TDA_DeviceMapCascadedAll);
			if (retVal != 0)
			{
				printf("Dynamic Advance chirp LUT offset update failed with error code %d", retVal);
				break;
			}
			osiSleep(1); /*Sleep 1 msec*/
		}

		if (mmwl_bTDA_ARMDone == 1U)
		{
			MMWL_DeArmingTDA();
		}
	}

	MMWL_DeviceDeInit(mmwl_TDA_DeviceMapCascadedAll);

	/* Switch off the device */
	for (int i = 2; i >= 0; i--)
	{
		if (mmwl_TDA_SlavesEnabled[i] == 1)
		{
			MMWL_powerOff(1 << (i + 1));
		}
	}
	MMWL_powerOff(mmwl_TDA_DeviceMapCascadedMaster);
	
	if (mmwl_bTDA_CaptureCardConnect == 1)
	{
		/* Disconnect from TDA Capture card */
		retVal = ethernetDisconnect();
		if (retVal != RL_RET_CODE_OK)
		{
			printf("ERROR: Disconnecting from TDA failed with error %d \n\n",
				retVal);
			return -1;
		}
		else
		{
			printf("INFO: Disconnected from TDA \n\n");
			mmwl_bTDA_CaptureCardConnect = 0;
		}
	}

    /* Close Configuraiton file */
    MMWL_closeConfigFile();

    return 0;
}

/** @fn int main()
*
*   @brief Main function.
*
*   @return none
*
*   Main function.
*/
void main(void)
{
    int retVal;

    printf("================= mmWaveLink Example Application ====================\n\n");
    retVal = MMWL_App();
    if(retVal == RL_RET_CODE_OK)
    {
        printf("=========== mmWaveLink Example Application execution Successful =========== \n\n");
    }
    else
    {
        printf("=========== mmWaveLink Example Application execution Failed =========== \n\n");
    }

    /* Wait for Enter click */
    getchar();
    printf("=========== mmWaveLink Example Application: Exit =========== \n\n");
}
