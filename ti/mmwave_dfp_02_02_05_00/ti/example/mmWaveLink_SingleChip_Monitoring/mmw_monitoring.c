/****************************************************************************************
* FileName     : mmw_monitoring.c
*
* Description  : This file implements mmwavelink Advanced frame and monitoring feature of mmwave sensor.
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
#include <windows.h>
#include <stdio.h>
#include <share.h>
#include <string.h>
#include <stdlib.h>
#include "mmw_monitoring.h"
#include "mmw_config.h"
#include <ti/control/mmwavelink/mmwavelink.h>
#include <math.h>
#include "mmwl_port_ftdi.h"
#include "rls_osi.h"
#include <direct.h>
#include <sys/stat.h>
#include <inttypes.h>

/* AWR2243 meta image file */
#include "firmware/xwr22xx_metaImage.h"

/****************************************************************************************
* MACRO DEFINITIONS
****************************************************************************************
*/
#define MMWL_FW_FIRST_CHUNK_SIZE (224U)
#define MMWL_FW_CHUNK_SIZE (232U)

#define MMWL_MSS_FIRMWARE_SIZE								(sizeof(mssImg))
#define MMWL_CONFIG_FILE_SIZE								(sizeof(configImg))
#define MMWL_RADARSS_FIRMWARE_SIZE							(sizeof(bssImg))
#define MMWL_META_IMG_FILE_SIZE (sizeof(metaImage))

/* Async Event Timeouts */
#define MMWL_API_INIT_TIMEOUT								(2000) /* 2 Sec */
#define MMWL_API_START_TIMEOUT								(1000) /* 1 Sec */
#define MMWL_API_RF_INIT_TIMEOUT							(1000) /* 1 Sec */

/* Time unit for calibration/monitoring, it's based on Frame count */
#define CAL_MON_TIME_UNIT									4
/* To enable TX2 */
#define ENABLE_TX2											1

#define FALSE												0
#define TRUE												1

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
static unsigned char mmwl_bRunTimeCalib = 0U;
static unsigned char mmwl_bSensorStarted = 0U;
static unsigned char mmwl_bMonTypeTrigDone[3] = { 0 };
static unsigned char mmwl_bGpadcDataRcv = 0U;
static unsigned char mmwl_bMssCpuFault = 0U;
static unsigned char mmwl_bMssEsmFault = 0U;


unsigned char gAwr2243CrcType = RL_CRC_TYPE_32BIT;

/* Global variable configurations from config file */
rlDevGlobalCfg_t rlDevGlobalCfgArgs = { 0 };
/* API based monitoring triger config */
rlMonTypeTrigCfg_t rlMonTypeTrigCfgs = { 0 };

/* monitoring report header count */
unsigned int gMonReportHdrCnt = 0U;
/* store frame periodicity */
unsigned int framePeriodicity = 0;
/* Frame count configured to AWR2243 device */
unsigned short gFrameCount = 0;

/* Strcture to store async event config */
rlRfDevCfg_t rfDevCfg = { 0x0 };

/* SPI Communication handle to AWR2243 device*/
rlComIfHdl_t mmwl_devHdl = NULL;

/* calibData is the calibration data sent by the device which needs to store to
   sFlash and will be used for factory calibration or embedded in the application itself */
rlCalibrationData_t calibData = { 0 };
rlPhShiftCalibrationData_t phShiftCalibData = { 0 };

/* File Handle for Calibration Data */
FILE *CalibrationDataPtr = NULL;
FILE *PhShiftCalibrationDataPtr = NULL;

/* File Handle for Monitoring Reports */
FILE *MonitoringReportDataPtr = NULL;

/* File Handle for Calibration Reports */
FILE *CalibrationReportDataPtr = NULL;

/* File Handle for BSS Events */
FILE *BSSEventsDataPtr = NULL;

/* File Handle for MSS Events */
FILE *MSSEventsDataPtr = NULL;

struct stat st = { 0 };

uint64_t computeCRC(uint8_t *p, uint32_t len, uint8_t width);

#define USE_SYSTEM_TIME
static void rlsGetTimeStamp(char *Tsbuffer, char *reportName)
{
#ifdef USE_SYSTEM_TIME
	SYSTEMTIME SystemTime;
	GetLocalTime(&SystemTime);
	sprintf(Tsbuffer, "\n[%02d:%02d:%02d:%03d] %s: ", SystemTime.wHour, SystemTime.wMinute, SystemTime.wSecond, SystemTime.wMilliseconds, reportName);
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
	char cBuffer[2048];
	if (TRUE)
	{
		va_list ap;
		va_start(ap, fmt);
		vsnprintf(&cBuffer[0], sizeof(cBuffer), fmt, ap);
#ifdef _CAPTURE_TO_FILE_
		if (rls_traceFp != NULL)
		{
			fwrite(cBuffer, sizeof(char), strlen(cBuffer), rls_traceFp);
			fflush(rls_traceFp);
		}
		else
		{
			rls_traceFp = _fsopen("trace.txt", "wt", _SH_DENYWR);
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
			fwrite(cBuffer, sizeof(char), strlen(cBuffer), rls_traceMmwlFp);
			fflush(rls_traceMmwlFp);
		}
		else
		{
			rls_traceMmwlFp = _fsopen("mmwavelink_log.txt", "wt", _SH_DENYWR);
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

static void WriteReport(char* data, char* reportName, unsigned char fileNum)
{
	if (fileNum == 0)
	{
		EnterCriticalSection(&rlAsyncEvent);
		/* Calibration Report */
		char cBuffer[2048];
		va_list ap;
		va_start(ap, data);
		vsnprintf(&cBuffer[0], sizeof(cBuffer), data, ap);
		if (CalibrationReportDataPtr != NULL)
		{
			char tsTime[150] = { 0 };
			rlsGetTimeStamp(tsTime, reportName);
			fwrite(tsTime, sizeof(char), strlen(tsTime), CalibrationReportDataPtr);
			fwrite(cBuffer, sizeof(char), strlen(cBuffer), CalibrationReportDataPtr);
			fflush(CalibrationReportDataPtr);
		}
		else
		{
			char ReportDir[] = "Reports";
			if (stat(ReportDir, &st) == -1) {
				_mkdir(ReportDir);
			}
			strcat(ReportDir, "\\CalibrationReport.txt");
			CalibrationReportDataPtr = _fsopen(ReportDir, "wt", _SH_DENYWR);
			char tsTime[150] = { 0 };
			rlsGetTimeStamp(tsTime, reportName);
			fwrite(tsTime, sizeof(char), strlen(tsTime), CalibrationReportDataPtr);
			fwrite(cBuffer, sizeof(char), strlen(cBuffer), CalibrationReportDataPtr);
			fflush(CalibrationReportDataPtr);
		}
		va_end(ap);
		LeaveCriticalSection(&rlAsyncEvent);
	}
	else if (fileNum == 1)
	{
		EnterCriticalSection(&rlAsyncEvent);
		/* BSS Events */
		char cBuffer[2048];
		va_list ap;
		va_start(ap, data);
		vsnprintf(&cBuffer[0], sizeof(cBuffer), data, ap);
		if (BSSEventsDataPtr != NULL)
		{
			char tsTime[150] = { 0 };
			rlsGetTimeStamp(tsTime, reportName);
			fwrite(tsTime, sizeof(char), strlen(tsTime), BSSEventsDataPtr);
			fwrite(cBuffer, sizeof(char), strlen(cBuffer), BSSEventsDataPtr);
			fflush(BSSEventsDataPtr);
		}
		else
		{
			char ReportDir[] = "Reports";
			if (stat(ReportDir, &st) == -1) {
				_mkdir(ReportDir);
			}
			strcat(ReportDir, "\\BSSEvents.txt");
			BSSEventsDataPtr = _fsopen(ReportDir, "wt", _SH_DENYWR);
			char tsTime[150] = { 0 };
			rlsGetTimeStamp(tsTime, reportName);
			fwrite(tsTime, sizeof(char), strlen(tsTime), BSSEventsDataPtr);
			fwrite(cBuffer, sizeof(char), strlen(cBuffer), BSSEventsDataPtr);
			fflush(BSSEventsDataPtr);
		}
		va_end(ap);
		LeaveCriticalSection(&rlAsyncEvent);
	}
	else if (fileNum == 2)
	{
		EnterCriticalSection(&rlAsyncEvent);
		/* MSS Events */
		char cBuffer[2048];
		va_list ap;
		va_start(ap, data);
		vsnprintf(&cBuffer[0], sizeof(cBuffer), data, ap);
		if (MSSEventsDataPtr != NULL)
		{
			char tsTime[150] = { 0 };
			rlsGetTimeStamp(tsTime, reportName);
			fwrite(tsTime, sizeof(char), strlen(tsTime), MSSEventsDataPtr);
			fwrite(cBuffer, sizeof(char), strlen(cBuffer), MSSEventsDataPtr);
			fflush(MSSEventsDataPtr);
		}
		else
		{
			if (stat("Reports", &st) == -1) {
				_mkdir("Reports");
			}
			char ReportDir[] = "Reports";
			if (stat(ReportDir, &st) == -1) {
				_mkdir(ReportDir);
			}
			strcat(ReportDir, "\\MSSEvents.txt");
			MSSEventsDataPtr = _fsopen(ReportDir, "wt", _SH_DENYWR);
			char tsTime[150] = { 0 };
			rlsGetTimeStamp(tsTime, reportName);
			fwrite(tsTime, sizeof(char), strlen(tsTime), MSSEventsDataPtr);
			fwrite(cBuffer, sizeof(char), strlen(cBuffer), MSSEventsDataPtr);
			fflush(MSSEventsDataPtr);
		}
		va_end(ap);
		LeaveCriticalSection(&rlAsyncEvent);
	}
	else if(fileNum == 3)
	{
		EnterCriticalSection(&rlAsyncEvent);
		/* Monitoring Report */
		char cBuffer[2048];
		va_list ap;
		va_start(ap, data);
		vsnprintf(&cBuffer[0], sizeof(cBuffer), data, ap);
		if (MonitoringReportDataPtr != NULL)
		{
			char tsTime[150] = { 0 };
			rlsGetTimeStamp(tsTime, reportName);
			fwrite(tsTime, sizeof(char), strlen(tsTime), MonitoringReportDataPtr);
			fwrite(cBuffer, sizeof(char), strlen(cBuffer), MonitoringReportDataPtr);
			fflush(MonitoringReportDataPtr);
		}
		else
		{
			char ReportDir[] = "Reports";
			if (stat(ReportDir, &st) == -1) {
				_mkdir(ReportDir);
			}
			strcat(ReportDir, "\\MonitoringReport.txt");
			MonitoringReportDataPtr = _fsopen(ReportDir, "wt", _SH_DENYWR);
			char tsTime[150] = { 0 };
			rlsGetTimeStamp(tsTime, reportName);
			fwrite(tsTime, sizeof(char), strlen(tsTime), MonitoringReportDataPtr);
			fwrite(cBuffer, sizeof(char), strlen(cBuffer), MonitoringReportDataPtr);
			fflush(MonitoringReportDataPtr);
		}
		va_end(ap);
		LeaveCriticalSection(&rlAsyncEvent);
	}
}
/******************************************************************************
* all function definations starts here
*******************************************************************************
*/
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
rlUInt32_t asyncEvntCnt0[32];
rlUInt32_t asyncEvntCnt1[32];
void MMWL_asyncEventHandler(rlUInt8_t deviceIndex, rlUInt16_t sbId,
    rlUInt16_t sbLen, rlUInt8_t *payload)
{
    rlUInt16_t msgId = sbId / RL_MAX_SB_IN_MSG;
    rlUInt16_t asyncSB = RL_GET_SBID_FROM_MSG(sbId, msgId);
	char buf[2048] = "";

    /* Host can receive Async Event from RADARSS/MSS */
    switch (msgId)
    {
        /* Async Event from RADARSS */
        case RL_RF_ASYNC_EVENT_MSG:
        {
            asyncEvntCnt0[asyncSB]++;
            switch (asyncSB)
            {
			case RL_RF_AE_MONITOR_TYPE_TRIGGER_DONE_SB:
			{
				rlMonTypeTrigDoneStatus_t *data = (rlMonTypeTrigDoneStatus_t*)payload;
                if (data->monTrigTypeDone == 1U) /* Type 0 done for that device */
                {
                    mmwl_bMonTypeTrigDone[0] = 1U;
                    printf("Monitor Type 0 Trigger Done Async event received \n\n");
                }
                else if (data->monTrigTypeDone == 3U) /* Type 1 done for that device */
                {
                    mmwl_bMonTypeTrigDone[1] = 1U;
                    printf("Monitor Type 1 Trigger Done Async event received \n\n");
                }
                else if (data->monTrigTypeDone == 7U) /* Type 2 done for that device */
                {
                    mmwl_bMonTypeTrigDone[2] = 1U;
                    printf("Monitor Type 2 Trigger Done Async event received \n\n");
                }
			}
			break;
			case RL_RF_AE_FRAME_TRIGGER_RDY_SB:
			{
				mmwl_bSensorStarted = 1;
				printf("Frame Start Async event\n\n");
			}
			break;
			case RL_RF_AE_FRAME_END_SB:
			{
				mmwl_bSensorStarted = 0;
				printf("Frame End Async event\n\n");
			}
			break;
			case RL_RF_AE_CPUFAULT_SB:
			{
				printf("BSS CPU Fault Async event\n\n");
				rlCpuFault_t *data = (rlCpuFault_t*)payload;
				sprintf(buf, "%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d", data->faultType, data->lineNum, \
					data->faultLR, data->faultPrevLR, data->faultSpsr, data->faultSp, data->faultAddr, data->faultErrStatus, data->faultErrSrc, \
					data->faultAxiErrType, data->faultAccType, data->faultRecovType);
				WriteReport(buf, "BSSCPUFaultStatus", 1);
				while(1);
			}
			case RL_RF_AE_ESMFAULT_SB:
			{
				printf("BSS ESM Fault Async event\n\n");
				rlBssEsmFault_t *data = (rlBssEsmFault_t*)payload;
				sprintf(buf, "0x%x, 0x%x", data->esmGrp1Err, data->esmGrp2Err);
				WriteReport(buf, "BSSESMFaultStatus", 1);
			}
			break;
			case RL_RF_AE_INITCALIBSTATUS_SB:
			{
				mmwl_bRfInitComp = 1;
                printf("RF Init Async event\n\n");
				rlRfInitComplete_t *data = (rlRfInitComplete_t*)payload;
				sprintf(buf, "0x%x, 0x%x, %d, %d", data->calibStatus, data->calibUpdate, data->temperature, data->timeStamp);
				WriteReport(buf, "RFInitCalibStatus", 0);
			}
			break;
			case RL_RF_AE_MON_TIMING_FAIL_REPORT_SB:
			{
				rlCalMonTimingErrorReportData_t *data = (rlCalMonTimingErrorReportData_t*)payload;
                printf("Cal Mon Time Unit Fail [0x%x] Async event\n\n", data->timingFailCode);
				sprintf(buf, "0x%x", data->timingFailCode);
				WriteReport(buf, "CalMonTimingFailReport", 1);
			}
			break;
			case RL_RF_AE_RUN_TIME_CALIB_REPORT_SB:
			{
				mmwl_bRunTimeCalib = 1;
                printf("Run time Calibration Async event\n\n");
				rlRfRunTimeCalibReport_t *data = (rlRfRunTimeCalibReport_t*)payload;
				sprintf(buf, "0x%x, 0x%x, %d, %d", data->calibErrorFlag, data->calibUpdateStatus, data->temperature, data->timeStamp);
				WriteReport(buf, "RunTimeCalibReport", 0);
			}
			break;
			case RL_RF_AE_DIG_LATENTFAULT_REPORT_SB:
			{
				rlDigLatentFaultReportData_t *data = (rlDigLatentFaultReportData_t*)payload;
				sprintf(buf, "0x%x", data->digMonLatentFault);
				WriteReport(buf, "DigitalLatentFaultMonitoring", 1);
			}
			break;
			case RL_RF_AE_MON_REPORT_HEADER_SB:
			{
				rlMonReportHdrData_t *data = (rlMonReportHdrData_t*)payload;
				sprintf(buf, "%d, %d", data->fttiCount, data->avgTemp);
				WriteReport(buf, "MonitoringReportHeader", 3);
				printf("Monitoring Report Header with FTTI count [%d] received\n\n", data->fttiCount);
				gMonReportHdrCnt++;
				if (gFrameCount != 0)
				{
					if (gMonReportHdrCnt == (gFrameCount / CAL_MON_TIME_UNIT))
					{
						printf("All Monitoring Reports received! \n\n");
					}
				}
			}
			break;
			case RL_RF_AE_MON_DIG_PERIODIC_REPORT_SB:
			{
				rlDigPeriodicReportData_t *data = (rlDigPeriodicReportData_t*)payload;
				sprintf(buf, "%d, %d", data->digMonPeriodicStatus, data->timeStamp);
				WriteReport(buf, "DigitalPeriodicMonitoring", 1);
			}
			break;
			case RL_RF_AE_MON_TEMPERATURE_REPORT_SB:
			{
				rlMonTempReportData_t *data = (rlMonTempReportData_t*)payload;
				sprintf(buf, "%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d", data->statusFlags, data->errorCode, data->tempValues[0], data->tempValues[1], data->tempValues[2], data->tempValues[3], \
					data->tempValues[4], data->tempValues[5], data->tempValues[6], data->tempValues[7], data->tempValues[8], data->tempValues[9], data->timeStamp);
				WriteReport(buf, "TemperatureMonitoring", 3);
			}
			break;
			case RL_RF_AE_MON_RX_GAIN_PHASE_REPORT:
			{
				rlMonRxGainPhRep_t *data = (rlMonRxGainPhRep_t*)payload;
				sprintf(buf, "%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d", \
					data->statusFlags, data->errorCode, data->profIndex, data->loopbackPowerRF1, data->loopbackPowerRF2, data->loopbackPowerRF3, \
					data->rxGainVal[0], data->rxGainVal[1], data->rxGainVal[2], data->rxGainVal[3], data->rxGainVal[4], data->rxGainVal[5], data->rxGainVal[6], data->rxGainVal[7], \
					data->rxGainVal[8], data->rxGainVal[9], data->rxGainVal[10], data->rxGainVal[11], data->rxPhaseVal[0], data->rxPhaseVal[1], data->rxPhaseVal[2], data->rxPhaseVal[3], \
					data->rxPhaseVal[4], data->rxPhaseVal[5], data->rxPhaseVal[6], data->rxPhaseVal[7], data->rxPhaseVal[8], data->rxPhaseVal[9], data->rxPhaseVal[10], data->rxPhaseVal[11], \
					data->rxNoisePower1, data->rxNoisePower2, data->timeStamp);
				WriteReport(buf, "RXGainPhaseMonitoring", 3);
			}
			break;
			case RL_RF_AE_MON_RX_NOISE_FIG_REPORT:
			{
				rlMonRxNoiseFigRep_t *data = (rlMonRxNoiseFigRep_t*)payload;
				sprintf(buf, "%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d", \
					data->statusFlags, data->errorCode, data->profIndex, data->rxNoiseFigVal[0], data->rxNoiseFigVal[1], data->rxNoiseFigVal[2], data->rxNoiseFigVal[3], data->rxNoiseFigVal[4], data->rxNoiseFigVal[5], data->rxNoiseFigVal[6], \
					data->rxNoiseFigVal[7], data->rxNoiseFigVal[8], data->rxNoiseFigVal[9], data->rxNoiseFigVal[10], data->rxNoiseFigVal[11], data->timeStamp);
				WriteReport(buf, "RXNoiseFigureMonitoring", 3);
			}
			break;
			case RL_RF_AE_MON_RX_IF_STAGE_REPORT:
			{
				rlMonRxIfStageRep_t *data = (rlMonRxIfStageRep_t*)payload;
				sprintf(buf, "%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d", \
					data->statusFlags, data->errorCode, data->profIndex, data->lpfCutOffBandEdgeDroopValRx0, data->hpfCutOffFreqEr[0], data->hpfCutOffFreqEr[1], data->hpfCutOffFreqEr[2], data->hpfCutOffFreqEr[3], data->hpfCutOffFreqEr[4], data->hpfCutOffFreqEr[5], data->hpfCutOffFreqEr[6], \
					data->hpfCutOffFreqEr[7], data->lpfCutOffStopBandAtten[0], data->lpfCutOffStopBandAtten[1], data->lpfCutOffStopBandAtten[2], data->lpfCutOffStopBandAtten[3], data->lpfCutOffStopBandAtten[4], data->lpfCutOffStopBandAtten[5], data->lpfCutOffStopBandAtten[6], data->lpfCutOffStopBandAtten[7], \
					data->rxIfaGainErVal[0], data->rxIfaGainErVal[1], data->rxIfaGainErVal[2], data->rxIfaGainErVal[3], data->rxIfaGainErVal[4], data->rxIfaGainErVal[5], data->rxIfaGainErVal[6], data->rxIfaGainErVal[7], data->ifGainExp, data->lpfCutOffBandEdgeDroopValRx[0], data->lpfCutOffBandEdgeDroopValRx[1], \
					data->lpfCutOffBandEdgeDroopValRx[2], data->lpfCutOffBandEdgeDroopValRx[3], data->lpfCutOffBandEdgeDroopValRx[4], data->lpfCutOffBandEdgeDroopValRx[5], data->timeStamp);
				WriteReport(buf, "RXIFStageMonitoring", 3);
			}
			break;
			case RL_RF_AE_MON_TX0_POWER_REPORT:
			{
				rlMonTxPowRep_t *data = (rlMonTxPowRep_t*)payload;
				sprintf(buf, "%d, %d, %d, %d, %d, %d, %d", data->statusFlags, data->errorCode, data->profIndex, data->txPowVal[0], data->txPowVal[1], data->txPowVal[2], data->timeStamp);
				WriteReport(buf, "TX0PowerMonitoring", 3);
			}
			break;
			case RL_RF_AE_MON_TX1_POWER_REPORT:
			{
				rlMonTxPowRep_t *data = (rlMonTxPowRep_t*)payload;
				sprintf(buf, "%d, %d, %d, %d, %d, %d, %d", data->statusFlags, data->errorCode, data->profIndex, data->txPowVal[0], data->txPowVal[1], data->txPowVal[2], data->timeStamp);
				WriteReport(buf, "TX1PowerMonitoring", 3);
			}
			break;
			case RL_RF_AE_MON_TX2_POWER_REPORT:
			{
				rlMonTxPowRep_t *data = (rlMonTxPowRep_t*)payload;
				sprintf(buf, "%d, %d, %d, %d, %d, %d, %d", data->statusFlags, data->errorCode, data->profIndex, data->txPowVal[0], data->txPowVal[1], data->txPowVal[2], data->timeStamp);
				WriteReport(buf, "TX2PowerMonitoring", 3);
			}
			break;
			case RL_RF_AE_MON_TX0_BALLBREAK_REPORT:
			{
				rlMonTxBallBreakRep_t *data = (rlMonTxBallBreakRep_t*)payload;
				sprintf(buf, "%d, %d, %d, %d", data->statusFlags, data->errorCode, data->txReflCoefVal, data->timeStamp);
				WriteReport(buf, "TX0BallBreakMonitoring", 3);
			}
			break;
			case RL_RF_AE_MON_TX1_BALLBREAK_REPORT:
			{
				rlMonTxBallBreakRep_t *data = (rlMonTxBallBreakRep_t*)payload;
				sprintf(buf, "%d, %d, %d, %d", data->statusFlags, data->errorCode, data->txReflCoefVal, data->timeStamp);
				WriteReport(buf, "TX1BallBreakMonitoring", 3);
			}
			break;
			default:
			{
				printf("Unhandled BSS Async event. msgId: 0x%x, asyncSB: 0x%x  \n\n", msgId, asyncSB);
			}
			break;
			}
		}
		break;
		/* async event from radarSS which sub-block IDs comes under 0x81 MsgID */
		case RL_RF_ASYNC_EVENT_1_MSG:
		{
			asyncEvntCnt1[asyncSB]++;
			switch (asyncSB)
			{
			case RL_RF_AE_MON_TX2_BALLBREAK_REPORT:
			{
				rlMonTxBallBreakRep_t *data = (rlMonTxBallBreakRep_t*)payload;
				sprintf(buf, "%d, %d, %d, %d", data->statusFlags, data->errorCode, data->txReflCoefVal, data->timeStamp);
				WriteReport(buf, "TX2BallBreakMonitoring", 3);
			}
			break;
			case RL_RF_AE_MON_TX_GAIN_MISMATCH_REPORT:
			{
				rlMonTxGainPhaMisRep_t *data = (rlMonTxGainPhaMisRep_t*)payload;
				sprintf(buf, "%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d", \
					data->statusFlags, data->errorCode, data->profIndex, data->txGainVal[0], data->txGainVal[1], data->txGainVal[2], data->txGainVal[3], data->txGainVal[4], data->txGainVal[5], data->txGainVal[6], data->txGainVal[7], \
					data->txGainVal[8], data->txPhaVal[0], data->txPhaVal[1], data->txPhaVal[2], data->txPhaVal[3], data->txPhaVal[4], data->txPhaVal[5], data->txPhaVal[6], data->txPhaVal[7], data->txPhaVal[8], data->timeStamp);
				WriteReport(buf, "TXGainPhaseMismatchMonitoring", 3);
			}
			break;
			case RL_RF_AE_MON_TX0_PH_SHIFT_REPORT:
			{
				rlMonTxPhShiftRep_t *data = (rlMonTxPhShiftRep_t*)payload;
				sprintf(buf, "%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d", data->statusFlags, data->errorCode, data->profIndex, data->phaseShifterMonVal1, data->phaseShifterMonVal2, \
					data->phaseShifterMonVal3, data->phaseShifterMonVal4, data->txPsAmplitudeVal1, data->txPsAmplitudeVal2, data->txPsAmplitudeVal3, data->txPsAmplitudeVal4, \
					data->txPsNoiseVal1, data->txPsNoiseVal2, data->txPsNoiseVal3, data->txPsNoiseVal4, data->timeStamp);
				WriteReport(buf, "Tx0PhaseShifterMonitoring", 3);
			}
			break;
			case RL_RF_AE_MON_TX1_PH_SHIFT_REPORT:
			{
				rlMonTxPhShiftRep_t *data = (rlMonTxPhShiftRep_t*)payload;
				sprintf(buf, "%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d", data->statusFlags, data->errorCode, data->profIndex, data->phaseShifterMonVal1, data->phaseShifterMonVal2, \
					data->phaseShifterMonVal3, data->phaseShifterMonVal4, data->txPsAmplitudeVal1, data->txPsAmplitudeVal2, data->txPsAmplitudeVal3, data->txPsAmplitudeVal4, \
					data->txPsNoiseVal1, data->txPsNoiseVal2, data->txPsNoiseVal3, data->txPsNoiseVal4, data->timeStamp);
				WriteReport(buf, "Tx1PhaseShifterMonitoring", 3);
			}
			break;
			case RL_RF_AE_MON_TX2_PH_SHIFT_REPORT:
			{
				rlMonTxPhShiftRep_t *data = (rlMonTxPhShiftRep_t*)payload;
				sprintf(buf, "%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d", data->statusFlags, data->errorCode, data->profIndex, data->phaseShifterMonVal1, data->phaseShifterMonVal2, \
					data->phaseShifterMonVal3, data->phaseShifterMonVal4, data->txPsAmplitudeVal1, data->txPsAmplitudeVal2, data->txPsAmplitudeVal3, data->txPsAmplitudeVal4, \
					data->txPsNoiseVal1, data->txPsNoiseVal2, data->txPsNoiseVal3, data->txPsNoiseVal4, data->timeStamp);
				WriteReport(buf, "Tx2PhaseShifterMonitoring", 3);
			}
			break;
			case RL_RF_AE_MON_SYNTHESIZER_FREQ_REPORT:
			{
				rlMonSynthFreqRep_t *data = (rlMonSynthFreqRep_t*)payload;
				sprintf(buf, "%d, %d, %d, %d, %d, %d", data->statusFlags, data->errorCode, data->profIndex, data->maxFreqErVal, data->freqFailCnt, data->timeStamp);
				WriteReport(buf, "SynthFrequencyMonitoring", 3);
			}
			break;
			case RL_RF_AE_MON_EXT_ANALOG_SIG_REPORT:
			{
				rlMonExtAnaSigRep_t *data = (rlMonExtAnaSigRep_t*)payload;
				sprintf(buf, "%d, %d, %d, %d, %d, %d, %d, %d, %d", data->statusFlags, data->errorCode, data->extAnaSigVal[0], data->extAnaSigVal[1], data->extAnaSigVal[2], \
					data->extAnaSigVal[3], data->extAnaSigVal[4], data->extAnaSigVal[5], data->timeStamp);
				WriteReport(buf, "ExtAnalogSignalsMonitoring", 3);
			}
			break;
			case RL_RF_AE_MON_TX0_INT_ANA_SIG_REPORT:
			{
				rlMonTxIntAnaSigRep_t *data = (rlMonTxIntAnaSigRep_t*)payload;
				sprintf(buf, "%d, %d, %d, %d", data->statusFlags, data->errorCode, data->profIndex, data->timeStamp);
				WriteReport(buf, "IntTX0AnalogSignalMonitoring", 3);
			}
			break;
			case RL_RF_AE_MON_TX1_INT_ANA_SIG_REPORT:
			{
				rlMonTxIntAnaSigRep_t *data = (rlMonTxIntAnaSigRep_t*)payload;
				sprintf(buf, "%d, %d, %d, %d", data->statusFlags, data->errorCode, data->profIndex, data->timeStamp);
				WriteReport(buf, "IntTX1AnalogSignalMonitoring", 3);
			}
			break;
			case RL_RF_AE_MON_TX2_INT_ANA_SIG_REPORT:
			{
				rlMonTxIntAnaSigRep_t *data = (rlMonTxIntAnaSigRep_t*)payload;
				sprintf(buf, "%d, %d, %d, %d", data->statusFlags, data->errorCode, data->profIndex, data->timeStamp);
				WriteReport(buf, "IntTX2AnalogSignalMonitoring", 3);
			}
			break;
			case RL_RF_AE_MON_RX_INT_ANALOG_SIG_REPORT:
			{
				rlMonRxIntAnaSigRep_t *data = (rlMonRxIntAnaSigRep_t*)payload;
				sprintf(buf, "%d, %d, %d, %d", data->statusFlags, data->errorCode, data->profIndex, data->timeStamp);
				WriteReport(buf, "IntRxAnalogSignalMonitoring", 3);
			}
			break;
			case RL_RF_AE_MON_PMCLKLO_INT_ANA_SIG_REPORT:
			{
				rlMonPmclkloIntAnaSigRep_t *data = (rlMonPmclkloIntAnaSigRep_t*)payload;
				sprintf(buf, "%d, %d, %d, %d, %d", data->statusFlags, data->errorCode, data->profIndex, data->sync20GPower, data->timeStamp);
				WriteReport(buf, "IntPMCLKLOAnalogSignalMonitoring", 3);
			}
			break;
			case RL_RF_AE_MON_GPADC_INT_ANA_SIG_REPORT:
			{
				rlMonGpadcIntAnaSigRep_t *data = (rlMonGpadcIntAnaSigRep_t*)payload;
				sprintf(buf, "%d, %d, %d, %d, %d", data->statusFlags, data->errorCode, data->gpadcRef1Val, data->gpadcRef2Val, data->timeStamp);
				WriteReport(buf, "IntGPADCAnalogSignalMonitoring", 3);
			}
			break;
			case RL_RF_AE_MON_PLL_CONTROL_VOLT_REPORT:
			{
				rlMonPllConVoltRep_t *data = (rlMonPllConVoltRep_t*)payload;
				sprintf(buf, "%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d", data->statusFlags, data->errorCode, data->pllContVoltVal[0], data->pllContVoltVal[1], data->pllContVoltVal[2], \
					data->pllContVoltVal[3], data->pllContVoltVal[4], data->pllContVoltVal[5], data->pllContVoltVal[6], data->pllContVoltVal[7], data->timeStamp);
				WriteReport(buf, "PLLControlVoltageSignalMonitoring", 3);
			}
			break;
			case RL_RF_AE_MON_DCC_CLK_FREQ_REPORT:
			{
				rlMonDccClkFreqRep_t *data = (rlMonDccClkFreqRep_t*)payload;
				sprintf(buf, "%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d", data->statusFlags, data->errorCode, data->freqMeasVal[0], data->freqMeasVal[1], data->freqMeasVal[2], \
					data->freqMeasVal[3], data->freqMeasVal[4], data->freqMeasVal[5], data->freqMeasVal[6], data->freqMeasVal[7], data->timeStamp);
				WriteReport(buf, "DualClockComparatorMonitoring", 3);
			}
			break;
			case RL_RF_AE_MON_RX_MIXER_IN_PWR_REPORT:
			{
				rlMonRxMixrInPwrRep_t *data = (rlMonRxMixrInPwrRep_t*)payload;
				sprintf(buf, "%d, %d, %d, %d, %d", data->statusFlags, data->errorCode, data->profIndex, data->rxMixInVolt, data->timeStamp);
				WriteReport(buf, "RxMixerInputPowerMonitoring", 3);
			}
			break;
			case RL_RF_AE_MON_SYNTH_FREQ_NONLIVE_REPORT:
			{
				rlMonSynthFreqNonLiveRep_t *data = (rlMonSynthFreqNonLiveRep_t*)payload;
				sprintf(buf, "%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d", data->statusFlags, data->errorCode, data->profIndex0, data->maxFreqErVal0, data->freqFailCnt0, data->maxFreqFailTime0, data->profIndex1, data->maxFreqErVal1, data->freqFailCnt1, data->maxFreqFailTime1,  data->timeStamp);
				WriteReport(buf, "SynthFrequencyNonLiveMonitoring", 3);
			}
			break;
			default:
			{
				printf("Unhandled BSS Async event. msgId: 0x%x, asyncSB: 0x%x  \n\n", msgId, asyncSB);
			}
			break;
			}
			break;
		}

        /* Async Event from MSS */
        case RL_DEV_ASYNC_EVENT_MSG:
        {
            switch (asyncSB)
            {
                case RL_DEV_AE_MSSPOWERUPDONE_SB:
                {
					mmwl_bInitComp = 1;
					printf("MSS Power Up Async event\n\n");
					rlInitComplete_t *data = (rlInitComplete_t*)payload;
					sprintf(buf, "%d, 0x%x, 0x%x, 0x%x, 0x%x", data->powerUpTime, data->powerUpStatus1, data->powerUpStatus2, data->bootTestStatus1, data->bootTestStatus2);
					WriteReport(buf, "MSSPowerUpDoneAsyncReport", 2);
                }
                break;
				case RL_DEV_AE_RFPOWERUPDONE_SB:
				{
					mmwl_bStartComp |= (1 << deviceIndex);
					printf("BSS Power Up Async event\n\n");
					rlStartComplete_t *data = (rlStartComplete_t*)payload;
					sprintf(buf, "0x%x, %d", data->status, data->powerUpTime);
					WriteReport(buf, "BSSRFPowerUpDoneAsyncReport", 2);
				}
				break;
				case RL_DEV_AE_MSS_CPUFAULT_SB:
				{
					mmwl_bMssCpuFault = 1;
					printf("MSS CPU Error Async event\n\n");
					rlCpuFault_t *data = (rlCpuFault_t*)payload;
					sprintf(buf, "%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d", data->faultType, data->lineNum, \
						data->faultLR, data->faultPrevLR, data->faultSpsr, data->faultSp, data->faultAddr, data->faultErrStatus, data->faultErrSrc, \
						data->faultAxiErrType, data->faultAccType, data->faultRecovType);
					WriteReport(buf, "MSSCPUFaultStatus", 2);
				}
				break;
				case RL_DEV_AE_MSS_ESMFAULT_SB:
				{
					mmwl_bMssEsmFault = 1;
					printf("MSS ESM Error Async event\n\n");
					rlMssEsmFault_t *data = (rlMssEsmFault_t*)payload;
					sprintf(buf, "0x%x, 0x%x", data->esmGrp1Err, data->esmGrp2Err);
					WriteReport(buf, "MSSESMFaultStatus", 2);
				}
				break;
				case RL_DEV_AE_MSS_BOOTERRSTATUS_SB:
				{
					mmwl_bMssBootErrStatus = 1U;
					printf("MSS Boot Error Status Async event\n\n");
					rlMssBootErrStatus_t *data = (rlMssBootErrStatus_t*)payload;
					sprintf(buf, "%d, 0x%x, 0x%x, 0x%x, 0x%x", data->powerUpTime, data->powerUpStatus1, data->powerUpStatus2, data->bootTestStatus1, data->bootTestStatus2);
					WriteReport(buf, "MSSBootErrorStatus", 2);
				}
				break;
				case RL_DEV_AE_MSS_LATENTFLT_TEST_REPORT_SB:
				{
					rlMssLatentFaultReport_t *data = (rlMssLatentFaultReport_t*)payload;
					sprintf(buf, "0x%x, 0x%x", data->testStatusFlg1, data->testStatusFlg2);
					WriteReport(buf, "MSSLatentFaultMonitoring", 2);
				}
				break;
				case RL_DEV_AE_MSS_PERIODIC_TEST_STATUS_SB:
				{
					rlMssPeriodicTestStatus_t *data = (rlMssPeriodicTestStatus_t*)payload;
					sprintf(buf, "0x%x", data->testStatusFlg);
					WriteReport(buf, "MSSPeriodicTestMonitoring", 2);
				}
				break;
				case RL_DEV_AE_MSS_RF_ERROR_STATUS_SB:
				{
					rlMssRfErrStatus_t *data = (rlMssRfErrStatus_t*)payload;
                    printf("MSS RF Error [0x%x] Status Async event\n\n", data->errStatusFlg);
					sprintf(buf, "0x%x", data->errStatusFlg);
					WriteReport(buf, "MSSRFErrorStatus", 2);
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
						printf("CRC mismatched in the received Async-Event msg \n\n");
					}
					/* Checksum mismatched in the received msg */
					else if (errTemp == RL_RET_CODE_CHKSUM_FAILED)
					{
						printf("Checksum mismatched in the received msg \n\n");
					}
					/* Polling to HostIRQ is timed out,
					   i.e. Device didn't respond to CNYS from the Host */
					else if (errTemp == RL_RET_CODE_HOSTIRQ_TIMEOUT)
					{
						printf("HostIRQ polling timed out \n\n");
					}
					/* If any of OSI call-back function returns non-zero value */
					else if (errTemp == RL_RET_CODE_RADAR_OSIF_ERROR)
					{
						printf("mmWaveLink OS_IF error \n\n");
					}
					break;
				}
			}
            break;
        }
        default:
        {
            printf("Unhandled Async Event msgId: 0x%x, asyncSB:0x%x  \n\n", msgId, asyncSB);
            break;
        }
    }
}

/** @fn int MMWL_enableDevice(unsigned char deviceIndex)
*
*   @brief Performs SOP and enables the device.
*
*   @param[in] deviceIndex
*
*   @return int Success - 0, Failure - Error Code
*
*   Power on Slave device API.
*/
int MMWL_enableDevice(unsigned char deviceIndex)
{
    int retVal = RL_RET_CODE_OK;
    /* Enable device in Functional Mode (SOP-4) */
    printf("rlDeviceEnable Callback is called by mmWaveLink for Device Index [%d]\n\n", deviceIndex);
    return rlsEnableDevice(deviceIndex);
}

/** @fn int MMWL_disableDevice(unsigned char deviceIndex)
*
*   @brief disables the device.
*
*   @param[in] deviceIndex
*
*   @return int Success - 0, Failure - Error Code
*
*   Power on Slave device API.
*/
int MMWL_disableDevice(unsigned char deviceIndex)
{
    printf("rlDeviceDisable Callback is called by mmWaveLink for Device Index [%d]\n\n", deviceIndex);
    return rlsDisableDevice(deviceIndex);
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
    clientCtx.comIfCb.rlComIfOpen = rlsCommOpen;
    clientCtx.comIfCb.rlComIfClose = rlsCommClose;
    clientCtx.comIfCb.rlComIfRead = rlsSpiRead;
    clientCtx.comIfCb.rlComIfWrite = rlsSpiWrite;

    /*   \subsection     porting_step3   Step 3 - Implement Device Control Interface
    The mmWaveLink driver internally powers on/off the mmWave device. The exact implementation of
    these interface is platform dependent, hence you need to implement below functions:
    -# rlDeviceEnable
    -# rlDeviceDisable
    -# rlRegisterInterruptHandler

    Refer to \ref rlDeviceCtrlCbs_t for interface details
    */
    clientCtx.devCtrlCb.rlDeviceDisable = MMWL_disableDevice;
    clientCtx.devCtrlCb.rlDeviceEnable = MMWL_enableDevice;
    clientCtx.devCtrlCb.rlDeviceMaskHostIrq = rlsCommIRQMask;
    clientCtx.devCtrlCb.rlDeviceUnMaskHostIrq = rlsCommIRQUnMask;
    clientCtx.devCtrlCb.rlRegisterInterruptHandler = rlsRegisterInterruptHandler;
    clientCtx.devCtrlCb.rlDeviceWaitIrqStatus = rlsDeviceWaitIrqStatus;

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
    while (mmwl_bInitComp == 0U)
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

    ret_val = rlDeviceFileDownload(deviceMap, &fileChunk, remChunks);
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
        printf("MMWL_fileDwld Fail : File Buffer is NULL \n\n\r");
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
        printf("MMWL_fileDwld Fail : Ftype: %d\n\n\r", MMWL_FILETYPE_META_IMG);
        return ret_val;
    }
    pmmwl_imgBuffer += MMWL_FW_FIRST_CHUNK_SIZE;
    mmwl_iRemChunks--;

    if(mmwl_iRemChunks > 0)
    {
        printf("Download in Progress: ");
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
				printf("MMWL_fileDwld last chunk Fail : Ftype: %d\n\n\r",
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
				printf("\n\n\r MMWL_fileDwld rem chunk Fail : Ftype: %d\n\n\r",
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
    printf("Meta Image download started for deviceMap %u\n\n",
        deviceMap);
    retVal = MMWL_fileDownload(deviceMap, MMWL_META_IMG_FILE_SIZE);
    printf("Meta Image download complete ret = %d\n\n", retVal);

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
    retVal = rlDeviceRfStart(deviceMap);
    while (mmwl_bStartComp == 0U)
    {
        osiSleep(1); /*Sleep 1 msec*/
        timeOutCnt++;
        if (timeOutCnt > MMWL_API_START_TIMEOUT)
        {
			printf("Timeout! RF Enable Status = %u\n\n", mmwl_bStartComp);
            retVal = RL_RET_CODE_RESP_TIMEOUT;
            break;
        }
    }
    mmwl_bStartComp = 0;

    if(retVal == RL_RET_CODE_OK)
    {
        rlVersion_t verArgs = {0};
		rlRfDieIdCfg_t dieId = { 0 };
        retVal = rlDeviceGetVersion(deviceMap,&verArgs);

        printf("RF Version [%2d.%2d.%2d.%2d] \nMSS version [%2d.%2d.%2d.%2d] \nmmWaveLink version [%2d.%2d.%2d.%2d]\n\n",
            verArgs.rf.fwMajor, verArgs.rf.fwMinor, verArgs.rf.fwBuild, verArgs.rf.fwDebug,
            verArgs.master.fwMajor, verArgs.master.fwMinor, verArgs.master.fwBuild, verArgs.master.fwDebug,
            verArgs.mmWaveLink.major, verArgs.mmWaveLink.minor, verArgs.mmWaveLink.build, verArgs.mmWaveLink.debug);
        printf("RF Patch Version [%2d.%2d.%2d.%2d] \nMSS Patch version [%2d.%2d.%2d.%2d]\n\n",
            verArgs.rf.patchMajor, verArgs.rf.patchMinor, ((verArgs.rf.patchBuildDebug & 0xF0) >> 4), (verArgs.rf.patchBuildDebug & 0x0F),
            verArgs.master.patchMajor, verArgs.master.patchMinor, ((verArgs.master.patchBuildDebug & 0xF0) >> 4), (verArgs.master.patchBuildDebug & 0x0F));

		retVal = rlGetRfDieId(deviceMap, &dieId);
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

    retVal = rlDeviceSetDataFmtConfig(deviceMap, &dataFmtCfgArgs);
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
    rfLdoBypassCfgArgs.ldoBypassEnable = 0x3;

    printf("Calling rlRfSetLdoBypassConfig With Bypass [%d] \n\n",
        rfLdoBypassCfgArgs.ldoBypassEnable);

    retVal = rlRfSetLdoBypassConfig(deviceMap, &rfLdoBypassCfgArgs);
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


    printf("Calling rlSetAdcOutConfig With [%d]ADC Bits and [%d]ADC Format \n\n",
        adcOutCfgArgs.fmt.b2AdcBits, adcOutCfgArgs.fmt.b2AdcOutFmt);

    retVal = rlSetAdcOutConfig(deviceMap, &adcOutCfgArgs);
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

    printf("Calling rlSetChannelConfig With [%d]Rx and [%d]Tx Channel Enabled \n\n",
           rfChanCfgArgs.rxChannelEn, rfChanCfgArgs.txChannelEn);

    retVal = rlSetChannelConfig(deviceMap, &rfChanCfgArgs);
    return retVal;
}

/** @fn int MMWL_basicConfiguration(unsigned char deviceMap, unsigned int cascade)
*
*   @brief Channel, ADC,Data format configuration API.
*
*   @param[in] deviceMap - Devic Index
*    @param[in] unsigned int cascade
*
*   @return int Success - 0, Failure - Error Code
*
*   Channel, ADC,Data format configuration API.
*/
int MMWL_basicConfiguration(unsigned char deviceMap, unsigned int cascade)
{
    int retVal = RL_RET_CODE_OK;

    /* Set which Rx and Tx channels will be enable of the device */
    retVal = MMWL_channelConfig(deviceMap, cascade);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Channel Config failed for deviceMap %u with error code %d\n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("Channel Configuration success for deviceMap %u\n\n", deviceMap);
    }
    /* ADC out data format configuration */
    retVal = MMWL_adcOutConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("AdcOut Config failed for deviceMap %u with error code %d\n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("AdcOut Configuration success for deviceMap %u\n\n", deviceMap);
    }

    /* LDO bypass configuration */
    retVal = MMWL_ldoBypassConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("LDO Bypass Config failed for deviceMap %u with error code %d\n\n",
            deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("LDO Bypass Configuration success for deviceMap %u\n\n", deviceMap);
    }

    /* Data format configuration */
    retVal = MMWL_dataFmtConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Data format Configuration failed for deviceMap %u with error code %d\n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("Data format Configuration success for deviceMap %u\n\n", deviceMap);
    }

    /* Data path configuration */
    retVal = MMWL_dataPathConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Data path Configuration failed for deviceMap %u with error code %d\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("Data path Configuration success for deviceMap %u\n\n", deviceMap);
    }

    /* low power configuration */
    retVal = MMWL_lowPowerConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Low Power Configuration failed for deviceMap %u with error %d \n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("Low Power Configuration success for deviceMap %u \n\n", deviceMap);
    }

    /* HSI (LVDS/CSI-2) clock configuration
       - This API is only applicable for AWR2243 when mmWaveLink driver is running on External Host */
    retVal = MMWL_hsiClockConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("CSI2/LVDS Clock Configuration failed for deviceMap %u with error %d \n\n",
            deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("CSI2/LVDS Clock Configuration success for deviceMap %u \n\n", deviceMap);
    }

    /* Async event direction and control configuration for RadarSS */
    retVal = MMWL_setAsyncEventDir(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("AsyncEvent Configuration failed for deviceMap %u with error code %d \n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("AsyncEvent Configuration success for deviceMap %u \n\n", deviceMap);
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
    mmwl_bRfInitComp = 0;

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
		retVal = rlRfInitCalibConfig(deviceMap, &rfCalibCfgArgs);
		if (retVal != RL_RET_CODE_OK)
		{
			printf("RF Init Calibration Configuration failed for deviceMap %u with error %d \n\n",
				deviceMap, retVal);
			return -1;
		}
		else
		{
			printf("RF Init Calibration Configuration success for deviceMap %u \n\n", deviceMap);
		}

		/* Calibration restore */
		if (rlDevGlobalCfgArgs.CalibStoreRestore == 0)
		{
			/* Load Phase shifter Calibration Data from a file */
			retVal = MMWL_LoadPhShiftCalibDataFromFile(deviceMap);
			if (retVal != RL_RET_CODE_OK)
			{
				printf("Load Phase shifter Calibration Data from a file failed for deviceMap %u with error %d \n\n",
					deviceMap, retVal);
				return -1;
			}
			else
			{
				printf("Load Phase shifter Calibration Data from a file success for deviceMap %u \n\n", deviceMap);
			}

			/* Phase shifter Calibration Data Restore Configuration */
			retVal = rlRfPhShiftCalibDataRestore(deviceMap, &phShiftCalibData);
			if (retVal != RL_RET_CODE_OK)
			{
				printf("Phase shifter Calibration Data Restore Configuration failed for deviceMap %u with error %d \n\n",
					deviceMap, retVal);
				return -1;
			}
			else
			{
				printf("Phase shifter Calibration Data Restore Configuration success for deviceMap %u \n\n", deviceMap);
			}
            
			/* Load Calibration Data from a file */
			retVal = MMWL_LoadCalibDataFromFile(deviceMap);
			if (retVal != RL_RET_CODE_OK)
			{
				printf("Load Calibration Data from a file failed for deviceMap %u with error %d \n\n",
					deviceMap, retVal);
				return -1;
			}
			else
			{
				printf("Load Calibration Data from a file success for deviceMap %u \n\n", deviceMap);
			}

			/* Calibration Data Restore Configuration */
			retVal = rlRfCalibDataRestore(deviceMap, &calibData);
			if (retVal != RL_RET_CODE_OK)
			{
				printf("Calibration Data Restore Configuration failed for deviceMap %u with error %d \n\n",
					deviceMap, retVal);
				return -1;
			}
			else
			{
				printf("Calibration Data Restore Configuration success for deviceMap %u \n\n", deviceMap);
				while (mmwl_bRfInitComp == 0U)
				{
					osiSleep(1); /*Sleep 1 msec*/
					timeOutCnt++;
					if (timeOutCnt > MMWL_API_RF_INIT_TIMEOUT)
					{
						retVal = RL_RET_CODE_RESP_TIMEOUT;
						break;
					}
				}
				mmwl_bRfInitComp = 0;
			}
		}
	}
    retVal = rlRfInit(deviceMap);
    while (mmwl_bRfInitComp == 0U)
    {
        osiSleep(1); /*Sleep 1 msec*/
        timeOutCnt++;
        if (timeOutCnt > MMWL_API_RF_INIT_TIMEOUT)
        {
            retVal = RL_RET_CODE_RESP_TIMEOUT;
            break;
        }
    }
    mmwl_bRfInitComp = 0;
	if (rlDevGlobalCfgArgs.CalibEnable == TRUE)
	{
		/* Calibration Store */
		if (rlDevGlobalCfgArgs.CalibStoreRestore == 1)
		{
			/* If all the calibration is done successfully as per above Async-event status,
			   now get the calibration data from the device */
			   /* Calibration Data Store Configuration */
			retVal = rlRfCalibDataStore(deviceMap, &calibData);
			if (retVal != RL_RET_CODE_OK)
			{
				printf("Calibration Data Store Configuration failed for deviceMap %u with error %d \n\n",
					deviceMap, retVal);
				return -1;
			}
			else
			{
				printf("Calibration Data Store Configuration success for deviceMap %u \n\n", deviceMap);
			}
            
            /* Phase shifter Calibration Data Store Configuration */
			retVal = rlRfPhShiftCalibDataStore(deviceMap, &phShiftCalibData);
			if (retVal != RL_RET_CODE_OK)
			{
				printf("Phase shifter Calibration Data Store Configuration failed for deviceMap %u with error %d \n\n",
					deviceMap, retVal);
				return -1;
			}
			else
			{
				printf("Phase shifter Calibration Data Store Configuration success for deviceMap %u \n\n", deviceMap);
			}

			/* Save Calibration Data to a file */
			retVal = MMWL_saveCalibDataToFile(deviceMap);
			if (retVal != RL_RET_CODE_OK)
			{
				printf("Save Calibration Data to a file failed for deviceMap %u with error %d \n\n",
					deviceMap, retVal);
				return -1;
			}
			else
			{
				printf("Save Calibration Data to a file success for deviceMap %u \n\n", deviceMap);
			}
            
			/* Save Phase shifter Calibration Data to a file */
			retVal = MMWL_savePhShiftCalibDataToFile(deviceMap);
			if (retVal != RL_RET_CODE_OK)
			{
				printf("Save Phase shifter Calibration Data to a file failed for deviceMap %u with error %d \n\n",
					deviceMap, retVal);
				return -1;
			}
			else
			{
				printf("Save Phase shifter Calibration Data to a file success for deviceMap %u \n\n", deviceMap);
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
	CalibrationDataPtr = _fsopen("CalibrationData.txt", "wt", _SH_DENYWR);

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
	PhShiftCalibrationDataPtr = _fsopen("PhShiftCalibrationData.txt", "wt", _SH_DENYWR);

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
	
	CalibrationDataPtr = _fsopen("CalibrationData.txt", "rt", _SH_DENYRD);

	if (CalibrationDataPtr == NULL)
	{
		printf("CalibrationData.txt does not exist or Error opening the file\n\n");
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
	
	PhShiftCalibrationDataPtr = _fsopen("PhShiftCalibrationData.txt", "rt", _SH_DENYRD);

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

    /*read profileCfgArgs from config file*/
    MMWL_readProgFiltConfig(&progFiltCnfgArgs);

    printf("Calling rlRfSetProgFiltConfig with \ncoeffStartIdx[%d]\nprogFiltLen[%d] GHz\nprogFiltFreqShift[%d] MHz/uS \n\n",
        progFiltCnfgArgs.coeffStartIdx, progFiltCnfgArgs.progFiltLen, progFiltCnfgArgs.progFiltFreqShift);
    retVal = rlRfSetProgFiltConfig(deviceMap, &progFiltCnfgArgs);
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

    printf("Calling rlRfSetProgFiltCoeffRam with \ncoeffArray0[%d]\ncoeffArray1[%d] GHz\ncoeffArray2[%d] MHz/uS \n\n",
    progFiltCoeffCnfgArgs.coeffArray[0], progFiltCoeffCnfgArgs.coeffArray[1], progFiltCoeffCnfgArgs.coeffArray[2]);
    retVal = rlRfSetProgFiltCoeffRam(deviceMap, &progFiltCoeffCnfgArgs);
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
    int retVal = RL_RET_CODE_OK;
    double endFreqConst;
	rlProfileCfg_t profileCfgArgs = { 0 };

    /*read profileCfgArgs from config file*/
    MMWL_readProfileConfig(&profileCfgArgs);

    endFreqConst = (double)(profileCfgArgs.startFreqConst * 53.6441803 + (profileCfgArgs.rampEndTime * 10) * \
                   (profileCfgArgs.freqSlopeConst * 48.2797623))/53.6441803;
    if ((profileCfgArgs.startFreqConst >= 1435388860U) && ((unsigned int)endFreqConst <= 1509954515U))
    {
        /* If start frequency is in between 77GHz to 81GHz use VCO2 */
        profileCfgArgs.pfVcoSelect = 0x02;
    }
    else
    {
        /* If start frequency is in between 76GHz to 78GHz use VCO1 */
        profileCfgArgs.pfVcoSelect = 0x00;
    }

    printf("Calling rlSetProfileConfig with \nProfileId[%d]\nStart Frequency[%f] GHz\nIdle Time[%f]\nADC start time[%f]\nRamp end time[%f]\nRamp Slope[%f] MHz/uS\nTX Start time[%f]\nSampling rate[%f] \n\n",
        profileCfgArgs.profileId, (float)((profileCfgArgs.startFreqConst * 53.6441803)/(1000*1000*1000)), (float)profileCfgArgs.idleTimeConst,
		(float)profileCfgArgs.adcStartTimeConst, (float)profileCfgArgs.rampEndTime, (float)(profileCfgArgs.freqSlopeConst * 48.2797623)/1000.0, (float)profileCfgArgs.txStartTime,
		(float)profileCfgArgs.digOutSampleRate);
    retVal = rlSetProfileConfig(deviceMap, 1U, &profileCfgArgs);
    return retVal;
}

/** @fn int MMMWL_setCalMonConfig(unsigned char deviceMap)
*
*   @brief Calibration monitoring time unit and freaquency limit configuration
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Calibration monitoring time unit and freaquency limit configuration API.
*/
int32_t MMMWL_setCalMonConfig(unsigned char deviceMap)
{
	int32_t         retVal;
	rlRfCalMonFreqLimitConf_t data2 = { 0 };
	rlRfCalMonTimeUntConf_t data = { 0 };

	/* set Frequency max and min limit */
	data2.freqLimitLow = 770U;
	data2.freqLimitHigh = 810U;

	/* set monitoring Unit etc. */
	data.calibMonTimeUnit = CAL_MON_TIME_UNIT;
	data.devId = 0U;
	data.numOfCascadeDev = 1U;
	data.monitoringMode = 1U; /* Host-based monitoring trigger */
	data.reserved = 0U;

    retVal = rlRfSetCalMonTimeUnitConfig(RL_DEVICE_MAP_CASCADED_1,&data);
    if (retVal != 0)
    {
        printf("Error: Unable to rlRfSetCalMonTimeUnitConfig [Error %d]\n", retVal);
        return -1;
    }

    retVal = rlRfSetCalMonFreqLimitConfig(RL_DEVICE_MAP_CASCADED_1,
        (rlRfCalMonFreqLimitConf_t*)&data2);

    if (retVal != 0)
    {
        printf("Error: Unable to rlRfSetCalMonFreqLimitConfig[Error %d]\n", retVal);
        return -1;
    }

    return 0;
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
    rlChirpCfg_t chirpCfgArgs = {0};

    /*read chirpCfgArgs from config file*/
    MMWL_readChirpConfig(&chirpCfgArgs);

    printf("Calling rlSetChirpConfig with \nProfileId[%d]\nStart Idx[%d]\nEnd Idx[%d] \n",
        chirpCfgArgs.profileId, chirpCfgArgs.chirpStartIdx,
        chirpCfgArgs.chirpEndIdx);
    retVal = rlSetChirpConfig(deviceMap, 1U, &chirpCfgArgs);
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
    rlFrameCfg_t frameCfgArgs = { 0 };

    /*read frameCfgArgs from config file*/
    MMWL_readFrameConfig(&frameCfgArgs);

    framePeriodicity = (frameCfgArgs.framePeriodicity * 5)/(1000*1000);
    gFrameCount = frameCfgArgs.numFrames;

    printf("Calling rlSetFrameConfig with \nStart Idx[%d]\nEnd Idx[%d]\nLoops[%d]\nPeriodicity[%d]ms \n\n",
        frameCfgArgs.chirpStartIdx, frameCfgArgs.chirpEndIdx,
        frameCfgArgs.numLoops, (frameCfgArgs.framePeriodicity * 5)/(1000*1000));

    retVal = rlSetFrameConfig(deviceMap, &frameCfgArgs);
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
    rlAdvFrameCfg_t AdvframeCfgArgs = { 0 };
    rlAdvFrameCfg_t GetAdvFrameCfgArgs = { 0 };
    /* reset frame periodicity to zero */
    framePeriodicity = 0;

    /*read frameCfgArgs from config file*/
    MMWL_readAdvFrameConfig(&AdvframeCfgArgs);
    /* Add all subframes periodicity to get whole frame periodicity */
	for (i = 0; i < AdvframeCfgArgs.frameSeq.numOfSubFrames; i++)
	{
		framePeriodicity += AdvframeCfgArgs.frameSeq.subFrameCfg[i].subFramePeriodicity;
	}

    framePeriodicity = (framePeriodicity * 5)/(1000*1000);
    /* store total number of frames configured */
    gFrameCount = AdvframeCfgArgs.frameSeq.numFrames;

    printf("Calling rlSetAdvFrameConfig with \nnumOfSubFrames[%d]\nforceProfile[%d]\nnumFrames[%d]\ntriggerSelect[%d]ms \n\n",
        AdvframeCfgArgs.frameSeq.numOfSubFrames, AdvframeCfgArgs.frameSeq.forceProfile,
        AdvframeCfgArgs.frameSeq.numFrames, AdvframeCfgArgs.frameSeq.triggerSelect);

    retVal = rlSetAdvFrameConfig(deviceMap, &AdvframeCfgArgs);
    if (retVal == 0)
    {
        retVal = rlGetAdvFrameConfig(deviceMap, &GetAdvFrameCfgArgs);
        if ((AdvframeCfgArgs.frameSeq.forceProfile != GetAdvFrameCfgArgs.frameSeq.forceProfile) || \
            (AdvframeCfgArgs.frameSeq.frameTrigDelay != GetAdvFrameCfgArgs.frameSeq.frameTrigDelay) || \
            (AdvframeCfgArgs.frameSeq.numFrames != GetAdvFrameCfgArgs.frameSeq.numFrames) || \
            (AdvframeCfgArgs.frameSeq.numOfSubFrames != GetAdvFrameCfgArgs.frameSeq.numOfSubFrames) || \
            (AdvframeCfgArgs.frameSeq.triggerSelect != GetAdvFrameCfgArgs.frameSeq.triggerSelect))
        {
            printf("MMWL_readAdvFrameConfig failed...\n\n");
            return retVal;
        }
    }
    return retVal;
}

/** @fn int MMWL_setAsyncEventDir(unsigned char deviceMap)
*
*   @brief Update async event message direction and CRC type of Async event
*           from AWR2243 radarSS to Host
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Update async event message direction and CRC type of Async event
*   from AWR2243 radarSS to Host
*/
int MMWL_setAsyncEventDir(unsigned char  deviceMap)
{
    int32_t         retVal;
    /* set global and monitoring async event direction to Host */
    rfDevCfg.aeDirection = 0x05;
    /* Set the CRC type of Async event received from radarSS */
    rfDevCfg.aeCrcConfig = gAwr2243CrcType;
    retVal = rlRfSetDeviceCfg(deviceMap, &rfDevCfg);

    /* Sanity Check: Was the mmWave link successful? */
    if (retVal != 0)
    {
        /* Error: Link reported an issue. */
        printf("Error: rlSetAsyncEventDir retVal=%d\n\n", retVal);
        return -1;
    }

    return 0;
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
    retVal = rlDeviceSetMiscConfig(deviceMap, &devMiscCfg);

    /* Sanity Check: Was the mmWave link successful? */
    if (retVal != 0)
    {
        /* Error: Link reported an issue. */
        printf("Error: rlDeviceSetMiscConfig retVal=%d\n\n", retVal);
        return -1;
    }

    return 0;
}

/** @fn int MMWL_rlRfAnaMonConfig(unsigned char deviceMap)
*
*   @brief consolidated configuration of all ana mon.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Sets the consolidated configuration of all analog monitoring..
*/
int MMWL_rlRfAnaMonConfig(unsigned char deviceMap)
{
    int32_t         retVal;
	rlMonAnaEnables_t rlAnaMonCnfgArgs = { 0 };

	/* read rlAnaMonCnfgArgs from config file */
	MMWL_readAnaMonConfig(&rlAnaMonCnfgArgs);

	printf("Calling rlRfAnaMonConfig with \nenMask[0x%x]\nldoVmonScEn[0x%x] \n\n",
		rlAnaMonCnfgArgs.enMask, rlAnaMonCnfgArgs.ldoVmonScEn);

    /* Analog monitoring configuration */
    retVal = rlRfAnaMonConfig(RL_DEVICE_MAP_CASCADED_1, &rlAnaMonCnfgArgs);
    return retVal;
}

/** @fn int MMWL_setRfTempMonConfig(unsigned char deviceMap)
*
*   @brief Temperature monitoring configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Temperature monitoring configuration API.
*/
int MMWL_setRfTempMonConfig(unsigned char deviceMap)
{
    int32_t         retVal;
    rlTempMonConf_t rlTempMonCnfgArgs = { 0 };

	/* read rlTempMonCnfgArgs from config file */
	MMWL_readTempMonConfig(&rlTempMonCnfgArgs);

	printf("Calling rlRfTempMonConfig with \nreportMode[%d]\nanaTempThreshMin[%d]\nanaTempThreshMax[%d]\ndigTempThreshMin[%d]\ndigTempThreshMax[%d]\ntempDiffThresh[%d] \n\n",
		rlTempMonCnfgArgs.reportMode, rlTempMonCnfgArgs.anaTempThreshMin, rlTempMonCnfgArgs.anaTempThreshMax, 
		rlTempMonCnfgArgs.digTempThreshMin, rlTempMonCnfgArgs.digTempThreshMax, rlTempMonCnfgArgs.tempDiffThresh);

    /* Temperature monitoring configuration */
    retVal = rlRfTempMonConfig(RL_DEVICE_MAP_CASCADED_1, &rlTempMonCnfgArgs);
    return retVal;
}

/** @fn int MMWL_setRfRxGainPhaMonConfig(unsigned char deviceMap)
*
*   @brief RX Gain and Phase Monitoring configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   RX Gain and Phase Monitoring configuration API.
*/
int MMWL_setRfRxGainPhaMonConfig(unsigned char deviceMap)
{
    int32_t         retVal;
    rlRxGainPhaseMonConf_t rlRxGainPhaMonCnfgArgs = { 0 };

	/* read rlRxGainPhaMonCnfgArgs from config file */
	MMWL_readRxGainPhaMonConfig(&rlRxGainPhaMonCnfgArgs);

	/* Enable Freq Dither for RF */
	rlRxGainPhaMonCnfgArgs.rfFreqBitMask |= (1 << 3);
	rlRxGainPhaMonCnfgArgs.rf1rf2FreqDitherLimits = 0xE0D0F0E0;
	rlRxGainPhaMonCnfgArgs.rf3FreqDitherLimits = 0xC0B0;

	printf("Calling rlRfRxGainPhMonConfig with \nprofileIndx[%d]\nrfFreqBitMask[%d]\nreportMode[%d]\ntxSel[%d]\nrxGainAbsThresh[%d]\nrxGainMismatchErrThresh[%d]\nrxGainFlatnessErrThresh[%d]\nrxGainPhaseMismatchErrThresh[%d]\nrxGainMismatchOffsetVal[%d]\nrxGainPhaseMismatchOffsetVal[%d] \n\n",
		rlRxGainPhaMonCnfgArgs.profileIndx, rlRxGainPhaMonCnfgArgs.rfFreqBitMask, rlRxGainPhaMonCnfgArgs.reportMode, rlRxGainPhaMonCnfgArgs.txSel, rlRxGainPhaMonCnfgArgs.rxGainAbsThresh,
		rlRxGainPhaMonCnfgArgs.rxGainMismatchErrThresh, rlRxGainPhaMonCnfgArgs.rxGainFlatnessErrThresh, rlRxGainPhaMonCnfgArgs.rxGainPhaseMismatchErrThresh,
		rlRxGainPhaMonCnfgArgs.rxGainMismatchOffsetVal[0][0], rlRxGainPhaMonCnfgArgs.rxGainPhaseMismatchOffsetVal[0][0]);

    /* RX Gain and Phase Monitoring configuration */
    retVal = rlRfRxGainPhMonConfig(RL_DEVICE_MAP_CASCADED_1, &rlRxGainPhaMonCnfgArgs);
    return retVal;
}

/** @fn int MMWL_setRfRxNoiseMonConfig(unsigned char deviceMap)
*
*   @brief RX Noise Monitoring configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   RX Noise Monitoring configuration API.
*/
int32_t MMWL_setRfRxNoiseMonConfig(unsigned char deviceMap)
{
    int32_t         retVal;
    rlRxNoiseMonConf_t rlRxNoiseMonCnfgArgs = { 0 };

	/* read rlRxNoiseMonCnfgArgs from config file */
	MMWL_readRxNoiseMonConfig(&rlRxNoiseMonCnfgArgs);

	printf("Calling rlRfRxNoiseMonConfig with \nprofileIndx[%d]\nreportMode[%d]\nrfFreqBitMask[%d]\nnoiseThresh[%d] \n\n",
		rlRxNoiseMonCnfgArgs.profileIndx, rlRxNoiseMonCnfgArgs.reportMode, rlRxNoiseMonCnfgArgs.rfFreqBitMask, rlRxNoiseMonCnfgArgs.noiseThresh);

    /* RX Noise Monitoring configuration */
	retVal = rlRfRxNoiseMonConfig(RL_DEVICE_MAP_CASCADED_1, &rlRxNoiseMonCnfgArgs);
    return retVal;
}

/** @fn int MMWL_setRfRxIfStageMonConfig(unsigned char deviceMap)
*
*   @brief RX IF Stage Monitoring configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   RX IF Stage Monitoring configuration API.
*/
int32_t MMWL_setRfRxIfStageMonConfig(unsigned char deviceMap)
{
    int32_t         retVal;
    rlRxIfStageMonConf_t rlRxIfStageMonCnfgArgs = { 0 };

	/* read rlRxIfStageMonCnfgArgs from config file */
	MMWL_readRxIfStageMonConfig(&rlRxIfStageMonCnfgArgs);

	printf("Calling rlRfRxIfStageMonConfig with \nprofileIndx[%d]\nreportMode[%d]\nhpfCutoffErrThresh[%d]\nlpfCutoffBandEdgeDroopThresh[%d]\nlpfCutoffStopbandAttenThresh[%d]\nifaGainErrThresh[%d] \n\n",
		rlRxIfStageMonCnfgArgs.profileIndx, rlRxIfStageMonCnfgArgs.reportMode, rlRxIfStageMonCnfgArgs.hpfCutoffErrThresh, rlRxIfStageMonCnfgArgs.lpfCutoffBandEdgeDroopThresh,
		rlRxIfStageMonCnfgArgs.lpfCutoffStopBandAttenThresh, rlRxIfStageMonCnfgArgs.ifaGainErrThresh);

    /* RX IF Stage Monitoring configuration */
    retVal = rlRfRxIfStageMonConfig(RL_DEVICE_MAP_CASCADED_1, &rlRxIfStageMonCnfgArgs);
    return retVal;
}

/** @fn int MMWL_setRfTxPowMonConfig(unsigned char deviceMap)
*
*   @brief TX Power Monitoring configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   TX Power Monitoring configuration API.
*/
int32_t MMWL_setRfTxPowMonConfig(unsigned char deviceMap)
{
    int32_t         retVal;

	rlTxPowMonConf_t rlTx0PowerMonCnfgArgs = { 0 };
	/* read rlTx0BallbreakMonCnfgArgs from config file */
	MMWL_readTxPowerMonConfig(&rlTx0PowerMonCnfgArgs);

	rlTxPowMonConf_t rlTx1PowerMonCnfgArgs = { 0 };
	/* read rlTx1PowerMonCnfgArgs from config file */
	MMWL_readTxPowerMonConfig(&rlTx1PowerMonCnfgArgs);

#if (ENABLE_TX2)
	rlTxPowMonConf_t rlTx2PowerMonCnfgArgs = { 0 };
	/* read rlTx2PowerMonCnfgArgs from config file */
	MMWL_readTxPowerMonConfig(&rlTx2PowerMonCnfgArgs);
#else
	rlTxPowMonConf_t rlTx2PowerMonCnfgArgs = NULL;
#endif

    rlAllTxPowMonConf_t data = { &rlTx0PowerMonCnfgArgs, &rlTx1PowerMonCnfgArgs, &rlTx2PowerMonCnfgArgs };

	printf("Calling rlRfTxPowrMonConfig with \nprofileIndx[%d]\nreportMode[%d]\nrfFreqBitMask[%d]\ntxPowAbsErrThresh[%d]\ntxPowFlatnessErrThresh[%d] \n\n",
		rlTx0PowerMonCnfgArgs.profileIndx, rlTx0PowerMonCnfgArgs.reportMode, rlTx0PowerMonCnfgArgs.rfFreqBitMask,
		rlTx0PowerMonCnfgArgs.txPowAbsErrThresh, rlTx0PowerMonCnfgArgs.txPowFlatnessErrThresh);

    /* TX Power Monitoring configuration */
    retVal = rlRfTxPowrMonConfig(RL_DEVICE_MAP_CASCADED_1, &data);
    return retVal;
}

/** @fn int MMWL_setRfTxBallbreakMonConfig(unsigned char deviceMap)
*
*   @brief TX Ballbreak Monitoring configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   TX Ballbreak Monitoring configuration API.
*/
int32_t MMWL_setRfTxBallbreakMonConfig(unsigned char deviceMap)
{
    int32_t         retVal;

	rlTxBallbreakMonConf_t rlTx0BallbreakMonCnfgArgs = { 0 };
	/* read rlTx0BallbreakMonCnfgArgs from config file */
	MMWL_readTxBallbreakMonConfig(&rlTx0BallbreakMonCnfgArgs);

	rlTxBallbreakMonConf_t rlTx1BallbreakMonCnfgArgs = { 0 };
	/* read rlTx1BallbreakMonCnfgArgs from config file */
	MMWL_readTxBallbreakMonConfig(&rlTx1BallbreakMonCnfgArgs);

#if (ENABLE_TX2)
	rlTxBallbreakMonConf_t rlTx2BallbreakMonCnfgArgs = { 0 };
	/* read rlTx2BallbreakMonCnfgArgs from config file */
	MMWL_readTxBallbreakMonConfig(&rlTx2BallbreakMonCnfgArgs);
#else
	rlTxBallbreakMonConf_t rlTx2BallbreakMonCnfgArgs = NULL;
#endif
    rlAllTxBallBreakMonCfg_t data = { &rlTx0BallbreakMonCnfgArgs, &rlTx1BallbreakMonCnfgArgs, &rlTx2BallbreakMonCnfgArgs };

	printf("Calling rlRfTxBallbreakMonConfig with \nreportMode[%d]\ntxReflCoeffMagThresh[%d] \n\n",
		rlTx0BallbreakMonCnfgArgs.reportMode, rlTx0BallbreakMonCnfgArgs.txReflCoeffMagThresh);

    /* TX Ballbreak Monitoring configuration */
    retVal = rlRfTxBallbreakMonConfig(RL_DEVICE_MAP_CASCADED_1, &data);
    return retVal;
}

/** @fn int MMWL_setRfTxGainPhaMonConfig(unsigned char deviceMap)
*
*   @brief TX Gain and Phase Monitoring configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   TX Gain and Phase Monitoring configuration API.
*/
int MMWL_setRfTxGainPhaMonConfig(unsigned char deviceMap)
{
	int32_t         retVal;
	rlTxGainPhaseMismatchMonConf_t rlTxGainPhaMonCnfgArgs = { 0 };

	/* read rlTxGainPhaMonCnfgArgs from config file */
	MMWL_readTxGainPhaMonConfig(&rlTxGainPhaMonCnfgArgs);

	printf("Calling rlTxGainPhaMonCnfgArgs with \nprofileIndx[%d]\nrfFreqBitMask[%d]\ntxEn[%d]\nrxEn[%d]\nreportMode[%d]\ntxGainMismatchThresh[%d]\ntxPhaseMismatchThresh[%d]\ntxGainMismatchOffsetVal[%d]\ntxPhaseMismatchOffsetVal[%d] \n\n",
		rlTxGainPhaMonCnfgArgs.profileIndx, rlTxGainPhaMonCnfgArgs.rfFreqBitMask, rlTxGainPhaMonCnfgArgs.txEn, rlTxGainPhaMonCnfgArgs.rxEn, rlTxGainPhaMonCnfgArgs.reportMode,
		rlTxGainPhaMonCnfgArgs.txGainMismatchThresh, rlTxGainPhaMonCnfgArgs.txPhaseMismatchThresh, rlTxGainPhaMonCnfgArgs.txGainMismatchOffsetVal[0][0], rlTxGainPhaMonCnfgArgs.txPhaseMismatchOffsetVal[0][0]);

	/* TX Gain and Phase Monitoring configuration */
	retVal = rlRfTxGainPhaseMismatchMonConfig(RL_DEVICE_MAP_CASCADED_1, &rlTxGainPhaMonCnfgArgs);
	return retVal;
}

/** @fn int MMWL_setRfTxPhaseShifterMonConfig(unsigned char deviceMap)
*
*   @brief TX Phase shifter Monitoring configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   TX Phase shifter Monitoring configuration API.
*/
int32_t MMWL_setRfTxPhaseShifterMonConfig(unsigned char deviceMap)
{
	int32_t         retVal;

	rlTxPhShiftMonConf_t rlTx0PhShiftMonCnfgArgs = { 0 };
	/* read rlTx0PhShiftMonCnfgArgs from config file */
	MMWL_readTxPhaseShifterMonConfig(&rlTx0PhShiftMonCnfgArgs);

	rlTxPhShiftMonConf_t rlTx1PhShiftMonCnfgArgs = { 0 };
	/* read rlTx1PhShiftMonCnfgArgs from config file */
	MMWL_readTxPhaseShifterMonConfig(&rlTx1PhShiftMonCnfgArgs);

#if (ENABLE_TX2)
	rlTxPhShiftMonConf_t rlTx2PhShiftMonCnfgArgs = { 0 };
	/* read rlTx2PhShiftMonCnfgArgs from config file */
	MMWL_readTxPhaseShifterMonConfig(&rlTx2PhShiftMonCnfgArgs);
#else
	rlTxPhShiftMonConf_t rlTx2PhShiftMonCnfgArgs = NULL;
#endif
	rlAllTxPhShiftMonConf_t data = { &rlTx0PhShiftMonCnfgArgs, &rlTx1PhShiftMonCnfgArgs, &rlTx2PhShiftMonCnfgArgs };

	printf("Calling rlRfTxPhShiftMonConfig with \nprofileIndx[%d]\nreportMode[%d]\nphShifterMonCfg[%d]\nrxEn[%d]\nmonChirpSlope[%d]\nphShifterIncVal1[%d]\nphShifterIncVal2[%d]\nphShifterIncVal3[%d]\nphShifterIncVal4[%d]\nphShifterMon1[%d]\nphShifterMon2[%d]\nphShifterMon3[%d]\nphShifterMon4[%d]\ntxPhaseErrorThresh[%d]\ntxAmplErrorThresh[%d] \n\n",
		rlTx0PhShiftMonCnfgArgs.profileIndx, rlTx0PhShiftMonCnfgArgs.reportMode, rlTx0PhShiftMonCnfgArgs.phShifterMonCfg, rlTx0PhShiftMonCnfgArgs.rxEn, rlTx0PhShiftMonCnfgArgs.monChirpSlope,
		rlTx0PhShiftMonCnfgArgs.phShifterIncVal1, rlTx0PhShiftMonCnfgArgs.phShifterIncVal2, rlTx0PhShiftMonCnfgArgs.phShifterIncVal3, rlTx0PhShiftMonCnfgArgs.phShifterIncVal4, rlTx0PhShiftMonCnfgArgs.phShifterMon1,
		rlTx0PhShiftMonCnfgArgs.phShifterMon2, rlTx0PhShiftMonCnfgArgs.phShifterMon3, rlTx0PhShiftMonCnfgArgs.phShifterMon4, rlTx0PhShiftMonCnfgArgs.txPhaseErrorThresh, rlTx0PhShiftMonCnfgArgs.txAmplErrorThresh);

	/* TX Phase shifter Monitoring configuration */
	retVal = rlRfTxPhShiftMonConfig(RL_DEVICE_MAP_CASCADED_1, &data);
	return retVal;
}

/** @fn int MMWL_setRfTxIntAnaSignalMonConfig(unsigned char deviceMap)
*
*   @brief  TX Internal Analog Signals Monitoring configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   TX Internal Analog Signals Monitoring configuration API.
*/
int32_t MMWL_setRfTxIntAnaSignalMonConfig(unsigned char deviceMap)
{
    int32_t         retVal;

    rlTxIntAnaSignalsMonConf_t rlTx0IntAnaSignalMonCnfgArgs = { 0 };
	/* read rlTx0IntAnaSignalMonCnfgArgs from config file */
	MMWL_readTxIntAnaSignalMonConfig(&rlTx0IntAnaSignalMonCnfgArgs);

    rlTxIntAnaSignalsMonConf_t rlTx1IntAnaSignalMonCnfgArgs = { 0 };
	/* read rlTx1IntAnaSignalMonCnfgArgs from config file */
	MMWL_readTxIntAnaSignalMonConfig(&rlTx1IntAnaSignalMonCnfgArgs);

#if (ENABLE_TX2)
	rlTxIntAnaSignalsMonConf_t rlTx2IntAnaSignalMonCnfgArgs = { 0 };
	/* read rlTx2IntAnaSignalMonCnfgArgs from config file */
	MMWL_readTxIntAnaSignalMonConfig(&rlTx2IntAnaSignalMonCnfgArgs);
#else
	rlTxIntAnaSignalsMonConf_t rlTx2IntAnaSignalMonCnfgArgs = NULL;
#endif
    rlAllTxIntAnaSignalsMonConf_t data = { &rlTx0IntAnaSignalMonCnfgArgs, &rlTx1IntAnaSignalMonCnfgArgs, &rlTx2IntAnaSignalMonCnfgArgs };

	printf("Calling rlRfTxIntAnaSignalsMonConfig with \nprofileIndx[%d]\nreportMode[%d]\ntxPhShiftDacMonThresh[%d] \n\n",
		rlTx0IntAnaSignalMonCnfgArgs.profileIndx, rlTx0IntAnaSignalMonCnfgArgs.reportMode, rlTx0IntAnaSignalMonCnfgArgs.txPhShiftDacMonThresh);

    /* TX Internal Analog Signals Monitoring configuration */
    retVal = rlRfTxIntAnaSignalsMonConfig(RL_DEVICE_MAP_CASCADED_1, &data);
    return retVal;
}

/** @fn int MMWL_setRfRxIntAnaSignalMonConfig(unsigned char deviceMap)
*
*   @brief  RX Internal Analog Signals Monitoring configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   RX Internal Analog Signals Monitoring configuration API.
*/
int32_t MMWL_setRfRxIntAnaSignalMonConfig(unsigned char deviceMap)
{
    int32_t         retVal;
    rlRxIntAnaSignalsMonConf_t rlRxIntAnaSignalMonCnfgArgs = { 0 };

	/* read rlRxIntAnaSignalMonCnfgArgs from config file */
	MMWL_readRxIntAnaSignalMonConfig(&rlRxIntAnaSignalMonCnfgArgs);

	printf("Calling rlRfRxIntAnaSignalsMonConfig with \nprofileIndx[%d]\nreportMode[%d]\n \n\n",
		rlRxIntAnaSignalMonCnfgArgs.profileIndx, rlRxIntAnaSignalMonCnfgArgs.reportMode);

    /* RX Internal Analog Signals Monitoring configuration */
    retVal = rlRfRxIntAnaSignalsMonConfig(RL_DEVICE_MAP_CASCADED_1, &rlRxIntAnaSignalMonCnfgArgs);
    return retVal;
}

/** @fn int MMWL_setRfPmClkLoIntAnaSignalsMonConfig(unsigned char deviceMap)
*
*   @brief PMClock configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   PM, CLK, LO Internal Analog Signals Monitoring configuration API.
*/
int32_t MMWL_setRfPmClkLoIntAnaSignalsMonConfig(unsigned char deviceMap)
{
    int32_t         retVal;
    rlPmClkLoIntAnaSignalsMonConf_t rlPmClkLoIntAnaSignalsMonCnfgArgs = { 0 };

	/* read rlPmClkLoIntAnaSignalsMonCnfgArgs from config file */
	MMWL_readPmClkLoIntAnaSignalsMonConfig(&rlPmClkLoIntAnaSignalsMonCnfgArgs);

	printf("Calling rlRfPmClkLoIntAnaSignalsMonConfig with \nprofileIndx[%d]\nreportMode[%d]\nsync20GMinThresh[%d]\nsync20GMaxThresh[%d]\nsync20GSigSel[%d] \n\n",
		rlPmClkLoIntAnaSignalsMonCnfgArgs.profileIndx, rlPmClkLoIntAnaSignalsMonCnfgArgs.reportMode, rlPmClkLoIntAnaSignalsMonCnfgArgs.sync20GMinThresh,
		rlPmClkLoIntAnaSignalsMonCnfgArgs.sync20GMaxThresh, rlPmClkLoIntAnaSignalsMonCnfgArgs.sync20GSigSel);

    /* PM, CLK, LO Internal Analog Signals Monitoring configuration */
    retVal = rlRfPmClkLoIntAnaSignalsMonConfig(RL_DEVICE_MAP_CASCADED_1, &rlPmClkLoIntAnaSignalsMonCnfgArgs);
    return retVal;
}

/** @fn int MMWL_setRfGPADCMonConfig(unsigned char deviceMap)
*
*   @brief GPADC monitoring configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   GPADC monitoring configuration API.
*/
int32_t MMWL_setRfGPADCMonConfig(unsigned char deviceMap)
{
    int32_t         retVal;
    rlGpadcIntAnaSignalsMonConf_t rlGpadcMonCnfgArgs = { 0 };
	rlGpadcMonCnfgArgs.reportMode = 2U;

	printf("Calling rlRfGpadcIntAnaSignalsMonConfig with \nreportMode[%d] \n\n", rlGpadcMonCnfgArgs.reportMode);

    /* GPADC monitoring configuration */
	retVal = rlRfGpadcIntAnaSignalsMonConfig(RL_DEVICE_MAP_CASCADED_1, &rlGpadcMonCnfgArgs);
    return retVal;
}

/** @fn int MMWL_setRfPllContrlVoltMonConfig(unsigned char deviceMap)
*
*   @brief APLL and Synthesizer�s control voltage signals monitoring configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   APLL and Synthesizer�s control voltage signals monitoring configuration API.
*/
int32_t MMWL_setRfPllContrlVoltMonConfig(unsigned char deviceMap)
{
    int32_t         retVal;
    rlPllContrVoltMonConf_t rlPllContrlVoltMonCnfgArgs = { 0 };

	/* read rlPllContrlVoltMonCnfgArgs from config file */
	MMWL_readPllContrlVoltMonConfig(&rlPllContrlVoltMonCnfgArgs);

	printf("Calling rlRfPllContrlVoltMonConfig with \nreportMode[%d]\nsignalEnables[%d] \n\n",
		rlPllContrlVoltMonCnfgArgs.reportMode, rlPllContrlVoltMonCnfgArgs.signalEnables);

    /* APLL and Synthesizer�s control voltage signals monitoring configuration */
    retVal = rlRfPllContrlVoltMonConfig(RL_DEVICE_MAP_CASCADED_1, &rlPllContrlVoltMonCnfgArgs);
    return retVal;
}

/** @fn int MMWL_setRfSynthFreqMonConfig(unsigned char deviceMap)
*
*   @brief Synth Freq Monitoring configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Synth Freq Monitoring configuration API.
*/
int32_t MMWL_setRfSynthFreqMonConfig(unsigned char deviceMap)
{
    int32_t         retVal;
    rlSynthFreqMonConf_t rlSynthFreqMonCnfgArgs = { 0 };

	/* read rlSynthFreqMonCnfgArgs from config file */
	MMWL_readSynthFreqMonConfig(&rlSynthFreqMonCnfgArgs);

	printf("Calling rlRfSynthFreqMonConfig with \nprofileIndx[%d]\nreportMode[%d]\nfreqErrThresh[%d]\nmonStartTime[%d]\nmonitorMode[%d] \n\n",
		rlSynthFreqMonCnfgArgs.profileIndx, rlSynthFreqMonCnfgArgs.reportMode, rlSynthFreqMonCnfgArgs.freqErrThresh,
		rlSynthFreqMonCnfgArgs.monStartTime, rlSynthFreqMonCnfgArgs.monitorMode);

    /* Synth Freq Monitoring configuration */
    retVal = rlRfSynthFreqMonConfig(RL_DEVICE_MAP_CASCADED_1, &rlSynthFreqMonCnfgArgs);
    return retVal;
}

/** @fn int MMWL_setRfDualClkCompMonConfig(unsigned char deviceMap)
*
*   @brief DCC based clock frequency monitoring configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   DCC based clock frequency monitoring configuration API.
*/
int32_t MMWL_setRfDualClkCompMonConfig(unsigned char deviceMap)
{
    int32_t         retVal;

    rlDualClkCompMonConf_t rlDualClkCompMonCnfgArgs = { 0 };

	/* read rlDualClkCompMonCnfgArgs from config file */
	MMWL_readDualClkCompMonConfig(&rlDualClkCompMonCnfgArgs);

	printf("Calling rlRfDualClkCompMonConfig with \nreportMode[%d]\ndccPairEnables[%d] \n\n",
		rlDualClkCompMonCnfgArgs.reportMode, rlDualClkCompMonCnfgArgs.dccPairEnables);

    /* DCC based clock frequency monitoring configuration */
    retVal = rlRfDualClkCompMonConfig(RL_DEVICE_MAP_CASCADED_1, &rlDualClkCompMonCnfgArgs);
    return retVal;
}

/** @fn int MMWL_setRfRxMixMonConfig(unsigned char deviceMap)
*
*   @brief Rx Mixer monitoring configuration API.
*
*   @param[in] deviceMap - Devic Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Rx Mixer monitoring configuration API.
*/
int32_t MMWL_setRfRxMixMonConfig(unsigned char deviceMap)
{
    int32_t         retVal;

    rlRxMixInPwrMonConf_t rlRxMixMonCnfgArgs = { 0 };

	/* read rlRxMixMonCnfgArgs from config file */
	MMWL_readRxMixMonConfig(&rlRxMixMonCnfgArgs);

	printf("Calling rlRfRxMixerInPwrConfig with \nprofileIndx[%d]\nreportMode[%d]\ntxEnable[%d]\nthresholds[%d] \n\n",
		rlRxMixMonCnfgArgs.profileIndx, rlRxMixMonCnfgArgs.reportMode,
        rlRxMixMonCnfgArgs.txEnable, rlRxMixMonCnfgArgs.thresholds);

    /* Rx Mixer monitoring configuration */
	retVal = rlRfRxMixerInPwrConfig(RL_DEVICE_MAP_CASCADED_1, &rlRxMixMonCnfgArgs);
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

    printf("Calling rlSetTestSourceConfig with Simulated Object at X[%d]cm, Y[%d]cm, Z[%d]cm \n\n",
            tsArgs.testObj[0].posX, tsArgs.testObj[0].posY, tsArgs.testObj[0].posZ);

    retVal = rlSetTestSourceConfig(deviceMap, &tsArgs);

    tsEnableArgs.tsEnable = 1U;
    retVal = rlTestSourceEnable(deviceMap, &tsEnableArgs);

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

    printf("Calling rlDeviceSetDataPathConfig with HSI Interface[%d] Selected \n\n",
            dataPathCfgArgs.intfSel);

    /* same API is used to configure CQ data size on the
     * lanes and number of samples of CQ[0-2] to br transferred.
     */
    retVal = rlDeviceSetDataPathConfig(deviceMap, &dataPathCfgArgs);
    return retVal;
}

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

    retVal = rlDeviceSetLvdsLaneConfig(deviceMap, &lvdsLaneCfgArgs);
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

    retVal = rlDeviceSetLaneConfig(deviceMap, &laneEnCfgArgs);
    return retVal;
}

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
    /*lane configuration*/
    retVal = MMWL_laneConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("LaneConfig failed for deviceMap %u with error code %d\n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("LaneConfig success for deviceMap %u\n\n", deviceMap);
    }
    /*LVDS lane configuration*/
    retVal = MMWL_lvdsLaneConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("LvdsLaneConfig failed for deviceMap %u with error code %d\n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("LvdsLaneConfig success for deviceMap %u\n\n", deviceMap);
    }
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

    printf("Calling rlDeviceSetHsiClk with HSI Clock[%d] \n\n",
            hsiClkgs.hsiClk);

    retVal = rlDeviceSetHsiClk(deviceMap, &hsiClkgs);
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

    printf("Calling rlDeviceSetDataPathClkConfig with HSI Data Rate[%d] Selected \n\n",
            dataPathClkCfgArgs.dataRate);

    retVal = rlDeviceSetDataPathClkConfig(deviceMap, &dataPathClkCfgArgs);
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
        printf("LvdsClkConfig failed for deviceMap %u with error code %d\n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("MMWL_hsiDataRateConfig success for deviceMap %u\n\n", deviceMap);
    }

    /*set high speed clock configuration*/
    retVal = MMWL_setHsiClock(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("MMWL_setHsiClock failed for deviceMap %u with error code %d\n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("MMWL_setHsiClock success for deviceMap %u\n\n", deviceMap);
    }

    return retVal;
}

/** @fn int MMWL_MonTypeTrigConf()
*
*   @brief API to provide Monitor Type Trigger.
*
*   @return int Success - 0, Failure - Error Code
*
*   API to provide Monitor Type Trigger.
*/
int MMWL_MonTypeTrigConf()
{
	int retVal = RL_RET_CODE_OK;
	int timeOutCnt = 0;
	/* Wait for Trigger Type 0 Monitors completion */
	while (mmwl_bMonTypeTrigDone[0] == 0)
	{
		osiSleep(1); /*Sleep 1 msec*/
		timeOutCnt++;
		if (timeOutCnt > MMWL_API_RF_INIT_TIMEOUT)
		{
			retVal = RL_RET_CODE_RESP_TIMEOUT;
			break;
		}
	}

	if (retVal != RL_RET_CODE_OK)
	{
		printf("Monitor Trigger Type 0 monitors - failed with error code = %d \n\n", retVal);
	}

	/* Trigger Type 1 monitors */
	mmwl_bMonTypeTrigDone[1] = 0U;
	rlMonTypeTrigCfgs.monTrigTypeEn = 2;
	retVal = rlMonTypeTrigConfig(1, &rlMonTypeTrigCfgs);
    timeOutCnt = 0;
	/* Wait for Trigger Type 1 Monitors completion */
	while (mmwl_bMonTypeTrigDone[1] == 0U)
	{
		osiSleep(1); /*Sleep 1 msec*/
		timeOutCnt++;
		if (timeOutCnt > MMWL_API_RF_INIT_TIMEOUT)
		{
			retVal = RL_RET_CODE_RESP_TIMEOUT;
			break;
		}
	}

	if (retVal != RL_RET_CODE_OK)
	{
		printf("Monitor Trigger Type 1 monitors - failed with error code = %d \n\n", retVal);
	}

	/* Trigger Type 2 monitors */
	mmwl_bMonTypeTrigDone[2] = 0U;
	rlMonTypeTrigCfgs.monTrigTypeEn = 4;
	retVal = rlMonTypeTrigConfig(1, &rlMonTypeTrigCfgs);
    timeOutCnt = 0;
	/* Wait for Trigger Type 2 Monitors completion */
	while (mmwl_bMonTypeTrigDone[2] == 0U)
	{
		osiSleep(1); /*Sleep 1 msec*/
		timeOutCnt++;
		if (timeOutCnt > MMWL_API_RF_INIT_TIMEOUT)
		{
			retVal = RL_RET_CODE_RESP_TIMEOUT;
			break;
		}
	}
    
	if (retVal != RL_RET_CODE_OK)
	{
		printf("Monitor Trigger Type 2 monitors - failed with error code = %d \n\n", retVal);
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
    mmwl_bSensorStarted = 0U;
	retVal = rlFrameStartStop(deviceMap, &data);
    while (mmwl_bSensorStarted == 0U)
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
    retVal = rlFrameStartStop(deviceMap, &data);
    if (retVal == RL_RET_CODE_OK)
	{
        while (mmwl_bSensorStarted == 1U)
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

	retVal = rlDevicePowerOff();
	if (retVal != RL_RET_CODE_OK)
	{
		printf("Power Off API failed with error code %d \n\n", retVal);
		return -1;
	}
	else
	{
		printf("Power Off API success\n\n");
		mmwl_bInitComp = 0;
		mmwl_bStartComp = 0;
		mmwl_bRfInitComp = 0;
		DeleteCriticalSection(&rlAsyncEvent);

		if (MonitoringReportDataPtr != NULL)
		{
			fclose(MonitoringReportDataPtr);
			MonitoringReportDataPtr = NULL;
		}
		if (CalibrationReportDataPtr != NULL)
		{
			fclose(CalibrationReportDataPtr);
			CalibrationReportDataPtr = NULL;
		}
		if (MSSEventsDataPtr != NULL)
		{
			fclose(MSSEventsDataPtr);
			MSSEventsDataPtr = NULL;
		}
		if (BSSEventsDataPtr != NULL)
		{
			fclose(BSSEventsDataPtr);
			BSSEventsDataPtr = NULL;
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

    retVal = rlSetLowPowerModeConfig(deviceMap, &rfLpModeCfgArgs);
    return retVal;
}

int MMWL_MonitoringConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK;

    /* consolidated configuration of all analog monitoring
       This API SB sets the consolidated configuration of all analog monitoring
    */
    retVal = MMWL_rlRfAnaMonConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Consolidated Monitoring config failed for deviceMap %u with error %d \n\n",
            deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("Consolidated Monitoring config success for deviceMap %u \n\n", deviceMap);
    }

    /* This API sets the monitoring periodicity based on frame count (FTTI) */
    retVal = MMMWL_setCalMonConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("MMMWL_setCalMonConfig failed for deviceMap %u with error code %d \n\n",
            deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("MMMWL_setCalMonConfig success for deviceMap %u \n\n", deviceMap);
    }

    /*   This API SB sets the Temperature sensor configuration    */
    retVal = MMWL_setRfTempMonConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("MMWL_setRfTempMonConfig failed for deviceMap %u with error %d \n\n",
            deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("MMWL_setRfTempMonConfig success for deviceMap %u \n\n",
            deviceMap);
    }

    /*   This API sets RX Gain and Phase Monitoring  configuration    */
    retVal = MMWL_setRfRxGainPhaMonConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("MMWL_setRfRxGainPhaMonConfig failed for deviceMap %u with error %d \n\n",
            deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("MMWL_setRfRxGainPhaMonConfig success for deviceMap %u \n\n",
            deviceMap);
    }

    /*   This API sets RX Noise Monitoring configuration    */
    retVal = MMWL_setRfRxNoiseMonConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Device map %u : MMWL_setRfRxNoiseMonConfig failed with error %d \n\n",
            deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("Device map %u : MMWL_setRfRxNoiseMonConfig success \n\n",
            deviceMap);
    }

    /*   This API sets RX IF Stage Monitoring configuration    */
    retVal = MMWL_setRfRxIfStageMonConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("MMWL_setRfRxIfStageMonConfig failed for deviceMap %u with error %d \n\n",
            deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("MMWL_setRfRxIfStageMonConfig success for deviceMap %u \n\n",
            deviceMap);
    }

    /*   This API sets TX Power Monitoring configuration    */
    retVal = MMWL_setRfTxPowMonConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("MMWL_setRfTxPowMonConfig failed for deviceMap %u with error %d \n\n",
            deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("MMWL_setRfTxPowMonConfig success for deviceMap %u \n\n",
            deviceMap);
    }

    /*   This API sets TX Ballbreak Monitoring  configuration    */
    retVal = MMWL_setRfTxBallbreakMonConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("MMWL_setRfTxBallbreakMonConfig failed for deviceMap %u with error %d \n\n",
            deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("MMWL_setRfTxBallbreakMonConfig success for deviceMap %u \n\n",
            deviceMap);
    }

	/*   This API sets TX Gain Phase Mistmatch Monitoring  configuration    */
	retVal = MMWL_setRfTxGainPhaMonConfig(deviceMap);
	if (retVal != RL_RET_CODE_OK)
	{
		printf("MMWL_setRfTxGainPhaMonConfig failed for deviceMap %u with error %d \n\n",
			deviceMap, retVal);
		return -1;
	}
	else
	{
		printf("MMWL_setRfTxGainPhaMonConfig success for deviceMap %u \n\n",
			deviceMap);
	}

	/*   This API sets TX Phase shifter Monitoring configuration    */
	retVal = MMWL_setRfTxPhaseShifterMonConfig(deviceMap);
	if (retVal != RL_RET_CODE_OK)
	{
		printf("MMWL_setRfTxPhaseShifterMonConfig failed for deviceMap %u with error %d \n\n",
			deviceMap, retVal);
		return -1;
	}
	else
	{
		printf("MMWL_setRfTxPhaseShifterMonConfig success for deviceMap %u \n\n",
			deviceMap);
	}

    /*   This API sets Synth Freq Monitoring configuration    */
    retVal = MMWL_setRfSynthFreqMonConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Device map %u : MMWL_setRfSynthFreqMonConfig failed with error %d \n\n",
            deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("Device map %u : MMWL_setRfSynthFreqMonConfig success \n\n",
            deviceMap);
    }

    /*   This API sets TX Internal Analog Signals Monitoring configuration    */
    retVal = MMWL_setRfTxIntAnaSignalMonConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("MMWL_setRfTxIntAnaSignalMonConfig failed for deviceMap %u with error %d \n\n",
            deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("MMWL_setRfTxIntAnaSignalMonConfig success for deviceMap %u \n\n",
            deviceMap);
    }


    /*   This API sets RX Internal Analog Signals Monitoring configuration    */
    retVal = MMWL_setRfRxIntAnaSignalMonConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("MMWL_setRfRxIntAnaSignalMonConfig failed for deviceMap %u with error %d \n\n",
            deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("MMWL_setRfRxIntAnaSignalMonConfig success for deviceMap %u \n\n",
            deviceMap);
    }

    /*   This API sets PM, CLK, LO Internal Analog Signals Monitoring configuration    */
    retVal = MMWL_setRfPmClkLoIntAnaSignalsMonConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("MMWL_setRfPmClkLoIntAnaSignalsMonConfig failed for deviceMap %u with error %d \n\n",
            deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("MMWL_setRfPmClkLoIntAnaSignalsMonConfig success for deviceMap %u \n\n",
            deviceMap);
    }

    /*   This API sets GPADC monitoring configuration    */
    retVal = MMWL_setRfGPADCMonConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Device map %u : MMWL_setRfGPADCMonConfig failed with error %d \n\n",
            deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("Device map %u : MMWL_setRfGPADCMonConfig success \n\n",
            deviceMap);
    }

    /*   This API sets APLL and Synthesizer�s control voltage signals monitoring configuration    */
    retVal = MMWL_setRfPllContrlVoltMonConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("MMWL_setRfPllContrlVoltMonConfig failed for deviceMap %u with error %d \n\n",
            deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("MMWL_setRfPllContrlVoltMonConfig success for deviceMap %u \n\n",
            deviceMap);
    }

    /*   This API sets DCC based clock frequency monitoring configuration    */
    retVal = MMWL_setRfDualClkCompMonConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Device map %u : MMWL_setRfDualClkCompMonConfig failed with error %d \n\n",
            deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("Device map %u : MMWL_setRfDualClkCompMonConfig success \n\n",
            deviceMap);
    }
    
    /*   This API sets RX Mixer monitoring configuration    */
    retVal = MMWL_setRfRxMixMonConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Device map %u : MMWL_setRfRxMixMonConfig failed with error %d \n\n",
            deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("Device map %u : MMWL_setRfRxMixMonConfig success \n\n",
            deviceMap);
    }

    printf("======================================================================\n\n");


    return retVal;
}

int MML_RunTimeCalibConfig(unsigned char deviceMap)
{
    int retVal = RL_RET_CODE_OK, timeOutCnt = 0;
    rlRunTimeCalibConf_t runTimeCalibCfg = {0};

	runTimeCalibCfg.oneTimeCalibEnMask = 0x610; /* disbale PD cal */
	runTimeCalibCfg.periodicCalibEnMask = 0x0; /* Clear periodic enable mask */
	runTimeCalibCfg.calibPeriodicity = 0x0; /* Set periodicity to zero */
	runTimeCalibCfg.reportEn = 0x1; /* enable report */
	runTimeCalibCfg.txPowerCalMode = 0x1; /* OLPC only */
	runTimeCalibCfg.CalTempIdxOverrideEn = 0x7; /* all overrides */
	runTimeCalibCfg.CalTempIdxTx = 7; /* 30-40 degrees */
	runTimeCalibCfg.CalTempIdxRx = 7; /* 30-40 degrees */
	runTimeCalibCfg.CalTempIdxLodist = 7; /* 30-40 degrees */

    mmwl_bRunTimeCalib = 0U;
    retVal = rlRfRunTimeCalibConfig(deviceMap, &runTimeCalibCfg);

    while (mmwl_bRunTimeCalib == 0U)
    {
        osiSleep(1); /*Sleep 1 msec*/
        timeOutCnt++;
        if (timeOutCnt > MMWL_API_RF_INIT_TIMEOUT)
        {
            retVal = RL_RET_CODE_RESP_TIMEOUT;
            break;
        }
    }
    mmwl_bRunTimeCalib = 0U;
	return retVal;
}

/** @fn int MMWL_SOPControl(deviceMap)
*
*   @brief SOP mode configuration API.
*
*   @param[in] deviceMap - Device Index
*
*   @return int Success - 0, Failure - Error Code
*
*   SOP mode configuration API.
*/
int MMWL_SOPControl(unsigned char deviceMap)
{
	int retVal = RL_RET_CODE_OK;
	/* Set SOP 4 mode */
	int SOPmode = 4;
	rlsDevHandle_t rlImpl_devHdl = NULL;

	rlImpl_devHdl = rlsGetDeviceCtx(0);
	if (rlImpl_devHdl != NULL)
	{
		retVal = rlsOpenGenericGpioIf(rlImpl_devHdl);
		retVal = rlsSOPControl(rlImpl_devHdl, SOPmode);
		retVal = rlsOpenBoardControlIf(rlImpl_devHdl);
	}
	else
	{
		retVal = RL_RET_CODE_INVALID_STATE_ERROR;
	}
	return retVal;
}

/** @fn int MMWL_ResetDevice(deviceMap)
*
*   @brief Device Reset configuration API.
*
*   @param[in] deviceMap - Device Index
*
*   @return int Success - 0, Failure - Error Code
*
*   Device Reset configuration API.
*/
int MMWL_ResetDevice(unsigned char deviceMap)
{
	int retVal = RL_RET_CODE_OK;
	/* Reset the devices */
	rlsDevHandle_t rlImpl_devHdl = NULL;

	rlImpl_devHdl = rlsGetDeviceCtx(0);
	if (rlImpl_devHdl != NULL)
	{
		retVal = rlsFullReset(rlImpl_devHdl, 0);
		retVal = rlsFullReset(rlImpl_devHdl, 1);
		retVal = rlsCloseBoardControlIf(rlImpl_devHdl);
		retVal = rlsCloseGenericGpioIf(rlImpl_devHdl);
	}
	else
	{
		retVal = RL_RET_CODE_INVALID_STATE_ERROR;
	}
	return retVal;
}

/** @fn int MMWL_App()
*
*   @brief mmWaveLink Monitoring Application for Single Chip.
*
*   @return int Success - 0, Failure - Error Code
*
*   mmWaveLink Monitoring Application for Single Chip.
*/

int MMWL_App()
{
    int retVal = RL_RET_CODE_OK;
    unsigned char deviceMap = RL_DEVICE_MAP_CASCADED_1;

    retVal = MMWL_openConfigFile();
    if (retVal != RL_RET_CODE_OK)
    {
        printf("failed to Open configuration file\n\n");
        return -1;
    }

	/* Read all global variable configurations from config file */
	MMWL_getGlobalConfigStatus(&rlDevGlobalCfgArgs);

	retVal = MMWL_SOPControl(deviceMap);
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

	retVal = MMWL_ResetDevice(deviceMap);
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
    retVal = MMWL_powerOnMaster(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("mmWave Device Power on failed for deviceMap %u with error %d \n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("mmWave Device Power on success for deviceMap %u \n\n",
                deviceMap);
    }

    /*  \subsection     api_sequence2     Seq 2 - Download Firmware/patch (Optional)
    The mmWave device firmware is ROMed and also can be stored in External Flash. This
    step is necessary if firmware needs to be patched and patch is not stored in serial
    Flash
    */
	if (rlDevGlobalCfgArgs.EnableFwDownload)
	{
		printf("========================== Firmware Download ==========================\n\n");
		retVal = MMWL_firmwareDownload(deviceMap);
		if (retVal != RL_RET_CODE_OK)
		{
			printf("Firmware update failed for deviceMap %u with error %d \n\n",
				deviceMap, retVal);
			return -1;
		}
		else
		{
			printf("Firmware update successful for deviceMap %u \n\n",
				deviceMap);
		}
		printf("=====================================================================\n\n");
	}

    /* Change CRC Type of Async Event generated by MSS to what is being requested by user in mmwaveconfig.txt */
    retVal = MMWL_setDeviceCrcType(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("CRC Type set for MasterSS failed for deviceMap %u with error code %d \n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("CRC Type set for MasterSS success for deviceMap %u \n\n", deviceMap);
    }

    /*  \subsection     api_sequence3     Seq 3 - Enable the mmWave Front end (Radar/RF subsystem)
    The mmWave Front end once enabled runs boot time routines and upon completion sends asynchronous event
    to notify the result
    */
    retVal = MMWL_rfEnable(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Radar/RF subsystem Power up failed for deviceMap %u with error %d \n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("Radar/RF subsystem Power up successful for deviceMap %u \n\n", deviceMap);
    }

    /*  \subsection     api_sequence4     Seq 4 - Basic/Static Configuration
    The mmWave Front end needs to be configured for mmWave Radar operations. basic
    configuration includes Rx/Tx channel configuration, ADC configuration etc
    */
    printf("======================Basic/Static Configuration======================\n\n");
    retVal = MMWL_basicConfiguration(deviceMap, 0);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Basic/Static configuration failed for deviceMap %u with error %d \n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("Basic/Static configuration success for deviceMap %u \n\n",
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
        printf("RF Initialization/Calibration failed for deviceMap %u with error code %d \n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("RF Initialization/Calibration successful for deviceMap %u \n\n", deviceMap);
    }

    /*  \subsection     api_sequence6     Seq 6 - Configures the programmable filter */
    printf("==================Programmable Filter Configuration==================\n\n");
    retVal = MMWL_progFiltConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Programmable Filter Configuration failed for deviceMap %u with error code %d \n\n",
            deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("Programmable Filter Configuration success for deviceMap %u \n\n", deviceMap);
    }

    /*  \subsection     api_sequence7     Seq 7 - Configures the programmable filter RAM coefficients */
    retVal = MMWL_progFiltCoeffRam(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Programmable Filter coefficient RAM Configuration failed for deviceMap %u with error code %d \n\n",
            deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("Programmable Filter coefficient RAM Configuration success for deviceMap %u \n\n", deviceMap);
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
        printf("Profile Configuration failed for deviceMap %u with error code %d \n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("Profile Configuration success for deviceMap %u \n\n", deviceMap);
    }

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
        printf("Chirp Configuration failed for deviceMap %u with error %d \n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("Chirp Configuration success for deviceMap %u \n\n", deviceMap);
    }

    /*  \subsection     api_sequence10     Seq 10 - Configure RF one time and periodic
    calibration of various RF/Analog aspects and trigger those. After this configuration, one
    time calibration report arrives immediately and periodic calibration report will arrive when
    frame starts in form of async event at every FTTI interval (here CAL_MON_TIME_UNIT) which
    is unit of frame count.
    @Note- This API must call after profileConfig API is called */
    retVal = MML_RunTimeCalibConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Run Time calibration failed for deviceMap %u with error code %d \n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("Run Time calibration successful for deviceMap %u \n\n", deviceMap);
    }

#ifdef ENABLE_TEST_SOURCE
    retVal = MMWL_testSourceConfig(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Test Source Configuration failed for deviceMap %u with error %d \n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("Test source Configuration success for deviceMap %u \n\n", deviceMap);
    }
#endif

    /* Check for If Advance Frame Test is enabled */
    if(rlDevGlobalCfgArgs.LinkAdvanceFrameTest == FALSE)
    {
        /*  \subsection     api_sequence11     Seq 11 - FMCW frame configuration
        A frame defines sequence of chirps and how this sequence needs to be repeated over time.
        */
        retVal = MMWL_frameConfig(deviceMap);
        if (retVal != RL_RET_CODE_OK)
        {
            printf("Frame Configuration failed for deviceMap %u with error %d \n\n",
                deviceMap, retVal);
            return -1;
        }
        else
        {
            printf("Frame Configuration success for deviceMap %u \n\n", deviceMap);
        }
    }
    else
    {
        /*  \subsection     api_sequence12     Seq 12 - FMCW Advance frame configuration
        A frame defines sequence of chirps and how this sequence needs to be repeated over time.
        */
        retVal = MMWL_advFrameConfig(deviceMap);

        if (retVal != RL_RET_CODE_OK)
        {
            printf("Adv Frame Configuration failed for deviceMap %u with error %d \n\n",
                deviceMap, retVal);
            return -1;
        }
        else
        {
            printf("Adv Frame Configuration success for deviceMap %u \n\n", deviceMap);
        }
    }
    printf("======================================================================\n\n");

    /*  \subsection     api_sequence13     Seq 13 - Configure monitoring time and frequncy unit
    and along with this enable misc monitoring feature of device.
    After this configuration when frame starts device will send monitoring data in form of
    async event at every FTTI interval (here CAL_MON_TIME_UNIT) which is unit of frame count */
    MMWL_MonitoringConfig(deviceMap);

	/* Trigger type 0 monitor before the frame start */
	mmwl_bMonTypeTrigDone[0U] = 0U;
	rlMonTypeTrigCfgs.monTrigTypeEn = 1;
	rlMonTypeTrigConfig(deviceMap, &rlMonTypeTrigCfgs);

    /*  \subsection     api_sequence14     Seq 14 - Start mmWave Radar Sensor
    This will trigger the mmWave Front to start transmitting FMCW signal. Raw ADC samples
    would be received from Digital front end. For AWR2243, if high speed interface is
    configured, RAW ADC data would be transmitted over CSI2/LVDS. On xWR1443/xWR1642, it can
    be processed using HW accelerator or DSP.
    After this API call, device will start sending monitoring data in form of Async Event message.
    */
    retVal = MMWL_sensorStart(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Sensor Start failed for deviceMap %u with error code %d \n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("Sensor Start successful for deviceMap %u \n\n", deviceMap);
    }
    printf("======================================================================\n\n");

    /* wait till Host receives all monitoring report where number and periodicity of monitoring report is configured
    from rlRfSetCalMonTimeUnitConfig API. */
	int fttiLoopCount = 0;
	int exp_header_cnt = gFrameCount / CAL_MON_TIME_UNIT;
    while (fttiLoopCount != exp_header_cnt)
    {
		/* Wait for type 0, type 1 and type 2 monitors to complete */
		MMWL_MonTypeTrigConf(deviceMap);
		/* Don't trigger for the last FTTI */
		if (fttiLoopCount != (exp_header_cnt - 1))
		{
			/* Trigger Type 0 monitors before the next FTTI starts */
			mmwl_bMonTypeTrigDone[0U] = 0U;
			rlMonTypeTrigCfgs.monTrigTypeEn = 1;
			rlMonTypeTrigConfig(deviceMap, &rlMonTypeTrigCfgs);
		}
		fttiLoopCount++;
    }

	/* Wait for the frame end async end from the device */
	while (mmwl_bSensorStarted != 0x0U)
	{
		osiSleep(1); /*Sleep 1 msec*/
	}

	/* Buffer time to receive any remaining reports/async events */
	osiSleep(framePeriodicity);

    /* Switch off the device */
    retVal = MMWL_powerOff(deviceMap);
    if (retVal != RL_RET_CODE_OK)
    {
        printf("Device power off failed for deviceMap %u with error code %d \n\n",
                deviceMap, retVal);
        return -1;
    }
    else
    {
        printf("Device power off success for deviceMap %u \n\n", deviceMap);
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

    printf("=========== mmWaveLink Monitoring Application =========== \n\n");
    retVal = MMWL_App();
    if(retVal == RL_RET_CODE_OK)
    {
        printf("=========== mmWaveLink Monitoring Application execution Successful=========== \n\n");
    }
    else
    {
        printf("=========== mmWaveLink Monitoring Application execution Failed =========== \n\n");
    }

    /* Wait for Enter click */
    getchar();
    printf("=========== mmWaveLink Monitoring Application: Exit =========== \n\n");
}