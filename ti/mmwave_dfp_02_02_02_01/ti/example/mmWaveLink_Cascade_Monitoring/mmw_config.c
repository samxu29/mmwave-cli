/****************************************************************************************
* FileName     : mmw_config.c
*
* Description  : This file reads the mmwave configuration from config file.
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
#include "mmw_config.h"

/****************************************************************************************
* MACRO DEFINITIONS
****************************************************************************************
*/

/******************************************************************************
* GLOBAL VARIABLES/DATA-TYPES DEFINITIONS
******************************************************************************
*/

/* File pointer for config file*/
FILE *mmwl_configfPtr = NULL;

/******************************************************************************
* Function Definitions
*******************************************************************************
*/

/** @fn char *MMWL_trim(char * s)
*
*   @brief get rid of trailing and leading whitespace along with "\n"
*
*   @param[in] s - String pointer which needs to be trimed
*
*   @return int Success - 0, Failure - Error Code
*
*   get rid of trailing and leading whitespace along with "\n"
*/
char *MMWL_trim(char * s)
{
    /* Initialize start, end pointers */
    char *s1 = s, *s2 = &s[strlen(s) - 1];

    /* Trim and delimit right side */
    while ((isspace(*s2)) && (s2 >= s1))
        s2--;
    *(s2 + 1) = '\0';

    /* Trim left side */
    while ((isspace(*s1)) && (s1 < s2))
        s1++;

    /* Copy finished string */
    strcpy(s, s1);
    return s;
}

/** @fn void MMWL_getGlobalConfigStatus(rlDevGlobalCfg_t *rlDevGlobalCfgArgs)
*
*   @brief Read all global variable configurations from config file.
*
*   @param[in] rlDevGlobalCfg_t *rlDevGlobalCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*/
void MMWL_getGlobalConfigStatus(rlDevGlobalCfg_t *rlDevGlobalCfgArgs)
{
	char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
	int retVal = RL_RET_CODE_OK;
	unsigned int readAllParams = 0;
	/*seek the pointer to starting of the file so that
			we dont miss any parameter*/
	fseek(mmwl_configfPtr, 0, SEEK_SET);
	/*parse the parameters by reading each line of the config file*/
	while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
		&& (readAllParams == 0))
	{
		/* Skip blank lines and comments */
		if (buff[0] == '\n' || buff[0] == '#')
		{
			continue;
		}

		/* Parse name/value pair from line */
		s = strtok(buff, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(name, s, STRINGLEN);
		}
		s = strtok(NULL, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(value, s, STRINGLEN);
		}
		MMWL_trim(value);

		if (strcmp(name, "LinkAdvanceFrameTest") == 0)
			rlDevGlobalCfgArgs->LinkAdvanceFrameTest = atoi(value);

		if (strcmp(name, "EnableFwDownload") == 0)
			rlDevGlobalCfgArgs->EnableFwDownload = atoi(value);

		if (strcmp(name, "CalibEnable") == 0)
			rlDevGlobalCfgArgs->CalibEnable = atoi(value);

		if (strcmp(name, "CalibStoreRestore") == 0)
			rlDevGlobalCfgArgs->CalibStoreRestore = atoi(value);

		if (strcmp(name, "CascadeDeviceMap") == 0)
		{
			rlDevGlobalCfgArgs->CascadeDeviceMap = atoi(value);
			readAllParams = 1;
		}
	}
}

/** @fn void MMWL_readPowerOnMaster(rlClientCbs_t *clientCtx)
*
*   @brief Read rlClientCbs_t params from config file.
*
*   @param[in] rlClientCbs_t *clientCtx
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read rlDevicePowerOn configuration params
*/
void MMWL_readPowerOnMaster(rlClientCbs_t *clientCtx)
{
    int readAllParams = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
    /*seek the pointer to starting of the file so that 
            we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL) 
             && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "crcType") == 0)
        {
            clientCtx->crcType = (rlCrcType_t)atoi(value);
        }
        
        if (strcmp(name, "ackTimeout") == 0)
        {
            clientCtx->ackTimeout = atoi(value);
            readAllParams = 1;
        }
    }
}

/** @fn void MMWL_readChannelConfig(rlChanCfg_t *rfChanCfgArgs, 
*                            unsigned short cascade)
*
*   @brief Read rlChanCfg_t params from config file.
*
*   @param[in] rlChanCfg_t *rfChanCfgArgs
*    @param[in] unsigned short cascade
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read channel configuration params
*/
void MMWL_readChannelConfig(rlChanCfg_t *rfChanCfgArgs, 
                            unsigned short cascade)
{
    int readAllParams = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
    /*seek the pointer to starting of the file so that 
            we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL) 
             && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "channelTx") == 0)
            rfChanCfgArgs->txChannelEn = atoi(value);

        if (strcmp(name, "channelRx") == 0)
            rfChanCfgArgs->rxChannelEn = atoi(value);

        if (strcmp(name, "cascading") == 0)
        {
            rfChanCfgArgs->cascading = atoi(value);
            readAllParams = 1;
        }
    }
    rfChanCfgArgs->cascading = cascade;
}

/** @fn void MMWL_readAdcOutConfig(rlAdcOutCfg_t *adcOutCfgArgs)
*
*   @brief Read rlAdcOutCfg_t params from config file.
*
*   @param[in] rlAdcOutCfg_t *adcOutCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read ADC configuration params
*/
void MMWL_readAdcOutConfig(rlAdcOutCfg_t *adcOutCfgArgs)
{
    int readAllParams = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
    /*seek the pointer to starting of the file so that 
            we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
             && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "adcBits") == 0)
            adcOutCfgArgs->fmt.b2AdcBits = atoi(value);

        if (strcmp(name, "adcFormat") == 0)
        {
            adcOutCfgArgs->fmt.b2AdcOutFmt = atoi(value);
            readAllParams = 1;
        }
    }
}

/** @fn void MMWL_readDataFmtConfig(rlDevDataFmtCfg_t *dataFmtCfgArgs)
*
*   @brief Read rlDevDataFmtCfg_t params from config file.
*
*   @param[in] rlDevDataFmtCfg_t *dataFmtCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read data format configuration params
*/
void MMWL_readDataFmtConfig(rlDevDataFmtCfg_t *dataFmtCfgArgs)
{
    int readAllParams = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
    /*seek the pointer to starting of the file so that 
            we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
             && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "rxChanEn") == 0)
            dataFmtCfgArgs->rxChannelEn = atoi(value);

        if (strcmp(name, "adcBitsD") == 0)
            dataFmtCfgArgs->adcBits = atoi(value);

        if (strcmp(name, "adcFmt") == 0)
            dataFmtCfgArgs->adcFmt = atoi(value);

        if (strcmp(name, "iqSwapSel") == 0)
            dataFmtCfgArgs->iqSwapSel = atoi(value);

        if (strcmp(name, "chInterleave") == 0)
        {
            dataFmtCfgArgs->chInterleave = atoi(value);
            readAllParams = 1;
        }
    }
}

/** @fn void MMWL_readLowPowerConfig(rlLowPowerModeCfg_t *rfLpModeCfgArgs)
*
*   @brief Read rlLowPowerModeCfg_t params from config file.
*
*   @param[in] rlLowPowerModeCfg_t *rfLpModeCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read low power configuration params
*/
void MMWL_readLowPowerConfig(rlLowPowerModeCfg_t *rfLpModeCfgArgs)
{
    int readAllParams = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
    /*seek the pointer to starting of the file so that 
            we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
             && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "lpAdcMode") == 0)
        {
            rfLpModeCfgArgs->lpAdcMode = atoi(value);
            readAllParams = 1;
        }
    }
}

/** @fn void MMWL_readDataPathConfig(rlDevDataPathCfg_t *dataPathCfgArgs)
*
*   @brief Read rlDevDataPathCfg_t params from config file.
*
*   @param[in] rlDevDataPathCfg_t *dataPathCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read data path configuration params
*/
void MMWL_readDataPathConfig(rlDevDataPathCfg_t *dataPathCfgArgs)
{
    int readAllParams = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
    /*seek the pointer to starting of the file so that 
            we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
             && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "intfSel") == 0)
            dataPathCfgArgs->intfSel = atoi(value);

        if (strcmp(name, "transferFmtPkt0") == 0)
            dataPathCfgArgs->transferFmtPkt0 = atoi(value);

        if (strcmp(name, "transferFmtPkt1") == 0)
            dataPathCfgArgs->transferFmtPkt1 = atoi(value);

        if (strcmp(name, "cqConfig") == 0)
            dataPathCfgArgs->cqConfig = atoi(value);
        
        if (strcmp(name, "cq0TransSize") == 0)
            dataPathCfgArgs->cq0TransSize = atoi(value);
        
        if (strcmp(name, "cq1TransSize") == 0)
            dataPathCfgArgs->cq1TransSize = atoi(value);
        
        if (strcmp(name, "cq2TransSize") == 0)
            dataPathCfgArgs->cq2TransSize = atoi(value);
    }
}

/** @fn void MMWL_readLvdsClkConfig(rlDevDataPathClkCfg_t *lvdsClkCfgArgs)
*
*   @brief Read rlDevDataPathClkCfg_t params from config file.
*
*   @param[in] rlDevDataPathClkCfg_t *lvdsClkCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read LVDS clock configuration params
*/
void MMWL_readLvdsClkConfig(rlDevDataPathClkCfg_t *lvdsClkCfgArgs)
{
    int readAllParams = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
    /*seek the pointer to starting of the file so that
    we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
             && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "laneClk") == 0)
            lvdsClkCfgArgs->laneClkCfg = atoi(value);

        if (strcmp(name, "dataRate") == 0)
        {
            lvdsClkCfgArgs->dataRate = atoi(value);
            readAllParams = 1;
        }
    }
}

/** @fn void MMWL_readSetHsiClock(rlDevHsiClk_t *hsiClkgs)
*
*   @brief Read rlDevHsiClk_t params from config file.
*
*   @param[in] rlDevHsiClk_t *hsiClkgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read data path clock configuration params
*/
void MMWL_readSetHsiClock(rlDevHsiClk_t *hsiClkgs)
{
    int readAllParams = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
    /*seek the pointer to starting of the file so that
    we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
             && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "hsiClk") == 0)
        {
            hsiClkgs->hsiClk = atoi(value);
            readAllParams = 1;
        }
    }
}

#if defined LVDS_ENABLE
/** @fn void MMWL_readLaneConfig(rlDevLaneEnable_t *laneEnCfgArgs)
*
*   @brief Read rlDevLaneEnable_t params from config file.
*
*   @param[in] rlDevLaneEnable_t *laneEnCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read LVDS/CSI2 lane configuration params
*/
void MMWL_readLaneConfig(rlDevLaneEnable_t *laneEnCfgArgs)
{
    int readAllParams = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
    /*seek the pointer to starting of the file so that
    we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
             && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "laneEn") == 0)
        {
            laneEnCfgArgs->laneEn = atoi(value);
            readAllParams = 1;
        }
    }
}

/** @fn void MMWL_readLvdsLaneConfig(rlDevLvdsLaneCfg_t *lvdsLaneCfgArgs)
*
*   @brief Read rlDevLvdsLaneCfg_t params from config file.
*
*   @param[in] rlDevLvdsLaneCfg_t *lvdsLaneCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read LVDS specific configuration params
*/
void MMWL_readLvdsLaneConfig(rlDevLvdsLaneCfg_t *lvdsLaneCfgArgs)
{
    int readAllParams = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
    /*seek the pointer to starting of the file so that
    we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
             && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "laneFmtMap") == 0)
            lvdsLaneCfgArgs->laneFmtMap = atoi(value);

        if (strcmp(name, "laneParamCfg") == 0)
        {
            lvdsLaneCfgArgs->laneParamCfg = atoi(value);
            readAllParams = 1;
        }
    }
}
#else
/** @fn void MMWL_readCSI2LaneConfig(rlDevCsi2Cfg_t *CSI2LaneCfgArgs)
*
*   @brief Read rlDevCsi2Cfg_t params from config file.
*
*   @param[in] rlDevCsi2Cfg_t *CSI2LaneCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read CSI2 specific configuration params
*/
void MMWL_readCSI2LaneConfig(rlDevCsi2Cfg_t *CSI2LaneCfgArgs)
{
	int readAllParams = 0;
	char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
	/*seek the pointer to starting of the file so that
	we dont miss any parameter*/
	fseek(mmwl_configfPtr, 0, SEEK_SET);
	/*parse the parameters by reading each line of the config file*/
	while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
		&& (readAllParams == 0))
	{
		/* Skip blank lines and comments */
		if (buff[0] == '\n' || buff[0] == '#')
		{
			continue;
		}

		/* Parse name/value pair from line */
		s = strtok(buff, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(name, s, STRINGLEN);
		}
		s = strtok(NULL, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(value, s, STRINGLEN);
		}
		MMWL_trim(value);

		if (strcmp(name, "lanePosPolSel") == 0)
		{
			CSI2LaneCfgArgs->lanePosPolSel = atoi(value);
			readAllParams = 1;
		}
	}
}
#endif

/** @fn void MMWL_readProfileConfig(rlProfileCfg_t *profileCfgArgs)
*
*   @brief Read rlProfileCfg_t params from config file.
*
*   @param[in] rlProfileCfg_t *profileCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read profile configuration
*/
void MMWL_readProfileConfig(rlProfileCfg_t *profileCfgArgs)
{
    int readAllParams = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN], *ptr;
    /*seek the pointer to starting of the file so that
    we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
             && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "profileId") == 0)
            profileCfgArgs->profileId = atoi(value);

        if (strcmp(name, "pfVcoSelect") == 0)
            profileCfgArgs->pfVcoSelect = atoi(value);

        if (strcmp(name, "startFreqConst") == 0)
            profileCfgArgs->startFreqConst = strtoul(value, &ptr, 10);

        if (strcmp(name, "idleTimeConst") == 0)
            profileCfgArgs->idleTimeConst = strtoul(value, &ptr, 10);

        if (strcmp(name, "adcStartTimeConst") == 0)
            profileCfgArgs->adcStartTimeConst = strtoul(value, &ptr, 10);

        if (strcmp(name, "rampEndTime") == 0)
            profileCfgArgs->rampEndTime = strtoul(value, &ptr, 10);

        if (strcmp(name, "txOutPowerBackoffCode") == 0)
            profileCfgArgs->txOutPowerBackoffCode = strtoul(value, &ptr, 10);

        if (strcmp(name, "txPhaseShifter") == 0)
            profileCfgArgs->txPhaseShifter = strtoul(value, &ptr, 10);

        if (strcmp(name, "freqSlopeConst") == 0)
            profileCfgArgs->freqSlopeConst = atoi(value);

        if (strcmp(name, "txStartTime") == 0)
            profileCfgArgs->txStartTime = atoi(value);

        if (strcmp(name, "numAdcSamples") == 0)
            profileCfgArgs->numAdcSamples = atoi(value);

        if (strcmp(name, "digOutSampleRate") == 0)
            profileCfgArgs->digOutSampleRate = atoi(value);

        if (strcmp(name, "hpfCornerFreq1") == 0)
            profileCfgArgs->hpfCornerFreq1 = atoi(value);

        if (strcmp(name, "hpfCornerFreq2") == 0)
            profileCfgArgs->hpfCornerFreq2 = atoi(value);

        if (strcmp(name, "rxGain") == 0)
        {
            profileCfgArgs->rxGain = atoi(value);
            readAllParams = 1;
        }
    }
}

/** @fn void MMWL_readChirpConfig(rlChirpCfg_t *chirpCfgArgs)
*
*   @brief Read rlChirpCfg_t params from config file.
*

*   @param[in] rlChirpCfg_t *chirpCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read chirp configuration params
*/

void MMWL_readChirpConfig(rlChirpCfg_t *chirpCfgArgs)
{
    int readAllParams = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN], *ptr;
    /*seek the pointer to starting of the file so that
    we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
             && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "chirpStartIdx") == 0)
            chirpCfgArgs->chirpStartIdx = atoi(value);

        if (strcmp(name, "chirpEndIdx") == 0)
            chirpCfgArgs->chirpEndIdx = atoi(value);

        if (strcmp(name, "profileIdCPCFG") == 0)
            chirpCfgArgs->profileId = atoi(value);

        if (strcmp(name, "startFreqVar") == 0)
            chirpCfgArgs->startFreqVar = strtoul(value, &ptr, 10);

        if (strcmp(name, "freqSlopeVar") == 0)
            chirpCfgArgs->freqSlopeVar = atoi(value);

        if (strcmp(name, "idleTimeVar") == 0)
            chirpCfgArgs->idleTimeVar = atoi(value);

        if (strcmp(name, "adcStartTimeVar") == 0)
            chirpCfgArgs->adcStartTimeVar = atoi(value);

        if (strcmp(name, "txEnable") == 0)
        {
            chirpCfgArgs->txEnable = atoi(value);
            readAllParams = 1;
        }
    }
}

/** @fn void MMWL_readFrameConfig(rlFrameCfg_t *frameCfgArgs)
*
*   @brief Read rlFrameCfg_t params from config file.
*
*   @param[in] rlFrameCfg_t *frameCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read frame configuration params
*/
void MMWL_readFrameConfig(rlFrameCfg_t *frameCfgArgs)
{
    int readAllParams = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN], *ptr;
    /*seek the pointer to starting of the file so that
    we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
             && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "chirpStartIdxFCF") == 0)
            frameCfgArgs->chirpStartIdx = atoi(value);

        if (strcmp(name, "chirpEndIdxFCF") == 0)
            frameCfgArgs->chirpEndIdx = atoi(value);

        if (strcmp(name, "frameCount") == 0)
            frameCfgArgs->numFrames = atoi(value);

        if (strcmp(name, "loopCount") == 0)
            frameCfgArgs->numLoops = atoi(value);

        if (strcmp(name, "periodicity") == 0)
            frameCfgArgs->framePeriodicity = strtoul(value, &ptr, 10);

        if (strcmp(name, "triggerDelay") == 0)
            frameCfgArgs->frameTriggerDelay = strtoul(value, &ptr, 10);

        if (strcmp(name, "numAdcSamples") == 0)
            frameCfgArgs->numAdcSamples = atoi(value);

        if (strcmp(name, "triggerSelect") == 0)
        {
            frameCfgArgs->triggerSelect = atoi(value);
            readAllParams = 1;
        }
    }
}

/** @fn void MMWL_readAdvFrameConfig(rlAdvFrameCfg_t *rlAdvFrameCfgArgs)
*
*   @brief Read rlAdvFrameCfg_t params from config file.
*
*   @param[in] rlAdvFrameCfg_t *rlAdvFrameCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read frame configuration params
*/
void MMWL_readAdvFrameConfig(rlAdvFrameCfg_t *rlAdvFrameCfgArgs)
{
    int readAllParams = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN], *ptr;
    unsigned char subFrameCfgCnt = 0, numsubframe = 0, advframe_flag = 0;
    /*seek the pointer to starting of the file so that
    we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
        && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "numOfSubFrames") == 0){
            rlAdvFrameCfgArgs->frameSeq.numOfSubFrames = atoi(value);
            subFrameCfgCnt = rlAdvFrameCfgArgs->frameSeq.numOfSubFrames;

            rlAdvFrameCfgArgs->frameData.numSubFrames = atoi(value);
            numsubframe = rlAdvFrameCfgArgs->frameData.numSubFrames;

            advframe_flag = 1;
        }

        if (strcmp(name, "forceProfile") == 0)
            rlAdvFrameCfgArgs->frameSeq.forceProfile = atoi(value);

        if (strcmp(name, "numFrames") == 0)
            rlAdvFrameCfgArgs->frameSeq.numFrames = atoi(value);

        if (strcmp(name, "loopBackCfg") == 0)
            rlAdvFrameCfgArgs->frameSeq.loopBackCfg = atoi(value);

        if (strcmp(name, "triggerSelect") == 0)
            rlAdvFrameCfgArgs->frameSeq.triggerSelect = atoi(value);

        if (strcmp(name, "frameTrigDelay") == 0)
            rlAdvFrameCfgArgs->frameSeq.frameTrigDelay = strtoul(value, &ptr, 10);

        if (strcmp(name, "forceProfileIdx") == 0)
            rlAdvFrameCfgArgs->frameSeq.subFrameCfg[--subFrameCfgCnt].forceProfileIdx = atoi(value);

        if (strcmp(name, "chirpStartIdxAF") == 0)
            if (advframe_flag == 1){
                advframe_flag = 0;
                rlAdvFrameCfgArgs->frameSeq.subFrameCfg[subFrameCfgCnt].chirpStartIdxOffset = atoi(value);
            }

        if (strcmp(name, "numOfChirps") == 0)
            rlAdvFrameCfgArgs->frameSeq.subFrameCfg[subFrameCfgCnt].numOfChirps = atoi(value);

        if (strcmp(name, "numLoops") == 0)
            rlAdvFrameCfgArgs->frameSeq.subFrameCfg[subFrameCfgCnt].numLoops = atoi(value);

        if (strcmp(name, "burstPeriodicity") == 0)
            rlAdvFrameCfgArgs->frameSeq.subFrameCfg[subFrameCfgCnt].burstPeriodicity = strtoul(value, &ptr, 10);

        if (strcmp(name, "chirpStartIdxOffset") == 0)
            rlAdvFrameCfgArgs->frameSeq.subFrameCfg[subFrameCfgCnt].chirpStartIdxOffset = atoi(value);

        if (strcmp(name, "numOfBurst") == 0)
            rlAdvFrameCfgArgs->frameSeq.subFrameCfg[subFrameCfgCnt].numOfBurst = atoi(value);

        if (strcmp(name, "numOfBurstLoops") == 0)
            rlAdvFrameCfgArgs->frameSeq.subFrameCfg[subFrameCfgCnt].numOfBurstLoops = atoi(value);

        if (strcmp(name, "subFramePeriodicity") == 0)
            rlAdvFrameCfgArgs->frameSeq.subFrameCfg[subFrameCfgCnt].subFramePeriodicity = strtoul(value, &ptr, 10);

        if (strcmp(name, "numAdcSamplesAF") == 0)
            rlAdvFrameCfgArgs->frameData.subframeDataCfg[--numsubframe].numAdcSamples = atoi(value);

        if (strcmp(name, "numChirpsInDataPacket") == 0)
        {
            rlAdvFrameCfgArgs->frameData.subframeDataCfg[numsubframe].numChirpsInDataPacket = atoi(value);

            /* Total number of chirps in one subframe */
            rlAdvFrameCfgArgs->frameData.subframeDataCfg[numsubframe].totalChirps =
                (rlAdvFrameCfgArgs->frameSeq.subFrameCfg[subFrameCfgCnt].numOfChirps *
                rlAdvFrameCfgArgs->frameSeq.subFrameCfg[subFrameCfgCnt].numLoops *
                rlAdvFrameCfgArgs->frameSeq.subFrameCfg[subFrameCfgCnt].numOfBurst *
                rlAdvFrameCfgArgs->frameSeq.subFrameCfg[subFrameCfgCnt].numOfBurstLoops);

            if (numsubframe == 0)
                readAllParams = 1;
        }

    }
}

/** @fn void MMWL_readContModeConfig(rlContModeCfg_t * rlContModeCfgArgs)
*
*   @brief Read rlContModeCfg_t params from config file.
*
*   @param[in] rlContModeCfg_t * rlContModeCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read continuous mode configuration params
*/
void MMWL_readContModeConfig(rlContModeCfg_t * rlContModeCfgArgs)
{
    int readAllParams = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
    /*seek the pointer to starting of the file so that
    we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
        && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "vcoSelect") == 0)
            rlContModeCfgArgs->vcoSelect= atoi(value);

        if (strcmp(name, "contModeRxGain") == 0)
            rlContModeCfgArgs->rxGain = atoi(value);
    }

}

/** @fn void MMWL_readDynChirpConfig(rlDynChirpCfg_t* rldynChirpCfgArgs)
*
*   @brief Read rlDynChirpCfg_t params from config file.
*
*   @param[in] rlDynChirpCfg_t* rldynChirpCfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read dynamic chirp configuration params
*/
void MMWL_readDynChirpConfig(rlDynChirpCfg_t* rldynChirpCfgArgs)
{
    int readAllParams = 0;
    int chirpRowCnt = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
    /*seek the pointer to starting of the file so that
    we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
        && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "chirpRowSel") == 0)
            rldynChirpCfgArgs->chirpRowSelect = atoi(value);

        if (strcmp(name, "chirpSegSel") == 0)
            rldynChirpCfgArgs->chirpSegSel = atoi(value);

        if (strcmp(name, "chirpNR1") == 0)
        {
            for (chirpRowCnt = 0; chirpRowCnt < 16; chirpRowCnt++)
                rldynChirpCfgArgs->chirpRow[chirpRowCnt].chirpNR1 = atoi(value);
        }
        if (strcmp(name, "chirpNR2") == 0)
        {
            for (chirpRowCnt = 0; chirpRowCnt < 16; chirpRowCnt++)
                rldynChirpCfgArgs->chirpRow[chirpRowCnt].chirpNR2 = atoi(value);
        }
        if (strcmp(name, "chirpNR3") == 0)
        {
            for (chirpRowCnt = 0; chirpRowCnt < 16; chirpRowCnt++)
                rldynChirpCfgArgs->chirpRow[chirpRowCnt].chirpNR3 = atoi(value);
        }
    }

}

/** @fn void MMWL_readProgFiltConfig(rlRfProgFiltConf_t* rlProgFiltCnfgArgs)
*
*   @brief Read rlRfProgFiltConf_t params from config file.
*
*   @param[in] rlRfProgFiltConf_t* rlProgFiltCnfgArgs
*
*   @return int Success - 0, Failure - Error Code
*
*   API to read programmabe filter configuration params
*/
void MMWL_readProgFiltConfig(rlRfProgFiltConf_t* rlProgFiltCnfgArgs)
{
    int readAllParams = 0;
    char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
    /*seek the pointer to starting of the file so that
    we dont miss any parameter*/
    fseek(mmwl_configfPtr, 0, SEEK_SET);
    /*parse the parameters by reading each line of the config file*/
    while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
        && (readAllParams == 0))
    {
        /* Skip blank lines and comments */
        if (buff[0] == '\n' || buff[0] == '#')
        {
            continue;
        }

        /* Parse name/value pair from line */
        s = strtok(buff, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(name, s, STRINGLEN);
        }
        s = strtok(NULL, "=");
        if (s == NULL)
        {
            continue;
        }
        else
        {
            strncpy(value, s, STRINGLEN);
        }
        MMWL_trim(value);

        if (strcmp(name, "profileId") == 0)
            rlProgFiltCnfgArgs->profileId = atoi(value);

        if (strcmp(name, "coeffStartIdx") == 0)
            rlProgFiltCnfgArgs->coeffStartIdx = atoi(value);

        if (strcmp(name, "progFiltLen") == 0)
            rlProgFiltCnfgArgs->progFiltLen = atoi(value);

        if (strcmp(name, "progFiltFreqShift") == 0)
        {
            rlProgFiltCnfgArgs->progFiltFreqShift = atoi(value);
            readAllParams = 1;
        }
    }
}

void MMWL_readAnaMonConfig(rlMonAnaEnables_t* rlAnaMonCnfgArgs)
{
	int readAllParams = 0;
	char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
	/*seek the pointer to starting of the file so that
	we dont miss any parameter*/
	fseek(mmwl_configfPtr, 0, SEEK_SET);
	/*parse the parameters by reading each line of the config file*/
	while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
		&& (readAllParams == 0))
	{
		/* Skip blank lines and comments */
		if (buff[0] == '\n' || buff[0] == '#')
		{
			continue;
		}

		/* Parse name/value pair from line */
		s = strtok(buff, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(name, s, STRINGLEN);
		}
		s = strtok(NULL, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(value, s, STRINGLEN);
		}
		MMWL_trim(value);

		if (strcmp(name, "AnaMon_enMask") == 0)
			rlAnaMonCnfgArgs->enMask = atoi(value);

		if (strcmp(name, "AnaMon_ldoVmonScEn") == 0)
		{
			rlAnaMonCnfgArgs->ldoVmonScEn = atoi(value);
			readAllParams = 1;
		}
	}
}

void MMWL_readTempMonConfig(rlTempMonConf_t* rlTempMonCnfgArgs)
{
	int readAllParams = 0;
	char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
	/*seek the pointer to starting of the file so that
	we dont miss any parameter*/
	fseek(mmwl_configfPtr, 0, SEEK_SET);
	/*parse the parameters by reading each line of the config file*/
	while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
		&& (readAllParams == 0))
	{
		/* Skip blank lines and comments */
		if (buff[0] == '\n' || buff[0] == '#')
		{
			continue;
		}

		/* Parse name/value pair from line */
		s = strtok(buff, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(name, s, STRINGLEN);
		}
		s = strtok(NULL, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(value, s, STRINGLEN);
		}
		MMWL_trim(value);

		if (strcmp(name, "TempMon_reportMode") == 0)
			rlTempMonCnfgArgs->reportMode = atoi(value);

		if (strcmp(name, "TempMon_anaTempThreshMin") == 0)
			rlTempMonCnfgArgs->anaTempThreshMin = atoi(value);

		if (strcmp(name, "TempMon_anaTempThreshMax") == 0)
			rlTempMonCnfgArgs->anaTempThreshMax = atoi(value);

		if (strcmp(name, "TempMon_digTempThreshMin") == 0)
			rlTempMonCnfgArgs->digTempThreshMin = atoi(value);

		if (strcmp(name, "TempMon_digTempThreshMax") == 0)
			rlTempMonCnfgArgs->digTempThreshMax = atoi(value);

		if (strcmp(name, "TempMon_tempDiffThresh") == 0)
		{
			rlTempMonCnfgArgs->tempDiffThresh = atoi(value);
			readAllParams = 1;
		}
	}
}

void MMWL_readRxNoiseMonConfig(rlRxNoiseMonConf_t* rlRxNoiseMonCnfgArgs)
{
	int readAllParams = 0;
	char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
	/*seek the pointer to starting of the file so that
	we dont miss any parameter*/
	fseek(mmwl_configfPtr, 0, SEEK_SET);
	/*parse the parameters by reading each line of the config file*/
	while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
		&& (readAllParams == 0))
	{
		/* Skip blank lines and comments */
		if (buff[0] == '\n' || buff[0] == '#')
		{
			continue;
		}

		/* Parse name/value pair from line */
		s = strtok(buff, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(name, s, STRINGLEN);
		}
		s = strtok(NULL, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(value, s, STRINGLEN);
		}
		MMWL_trim(value);

		if (strcmp(name, "RxNoiseMon_profileIndx") == 0)
			rlRxNoiseMonCnfgArgs->profileIndx = atoi(value);

		if (strcmp(name, "RxNoiseMon_reportMode") == 0)
			rlRxNoiseMonCnfgArgs->reportMode = atoi(value);

		if (strcmp(name, "RxNoiseMon_rfFreqBitMask") == 0)
			rlRxNoiseMonCnfgArgs->rfFreqBitMask = atoi(value);

		if (strcmp(name, "RxNoiseMon_noiseThresh") == 0)
		{
			rlRxNoiseMonCnfgArgs->noiseThresh = atoi(value);
			readAllParams = 1;
		}
	}
}

void MMWL_readRxIfStageMonConfig(rlRxIfStageMonConf_t* rlRxIfStageMonCnfgArgs)
{
	int readAllParams = 0;
	char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
	/*seek the pointer to starting of the file so that
	we dont miss any parameter*/
	fseek(mmwl_configfPtr, 0, SEEK_SET);
	/*parse the parameters by reading each line of the config file*/
	while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
		&& (readAllParams == 0))
	{
		/* Skip blank lines and comments */
		if (buff[0] == '\n' || buff[0] == '#')
		{
			continue;
		}

		/* Parse name/value pair from line */
		s = strtok(buff, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(name, s, STRINGLEN);
		}
		s = strtok(NULL, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(value, s, STRINGLEN);
		}
		MMWL_trim(value);

		if (strcmp(name, "RxIfStageMon_profileIndx") == 0)
			rlRxIfStageMonCnfgArgs->profileIndx = atoi(value);

		if (strcmp(name, "RxIfStageMon_reportMode") == 0)
			rlRxIfStageMonCnfgArgs->reportMode = atoi(value);

		if (strcmp(name, "RxIfStageMon_hpfCutoffErrThresh") == 0)
			rlRxIfStageMonCnfgArgs->hpfCutoffErrThresh = atoi(value);

		if (strcmp(name, "RxIfStageMon_lpfCutoffBandEdgeDroopThresh") == 0)
			rlRxIfStageMonCnfgArgs->lpfCutoffBandEdgeDroopThresh = atoi(value);

		if (strcmp(name, "RxIfStageMon_lpfCutoffStopBandAttenThresh") == 0)
			rlRxIfStageMonCnfgArgs->lpfCutoffStopBandAttenThresh = atoi(value);

		if (strcmp(name, "RxIfStageMon_ifaGainErrThresh") == 0)
		{
			rlRxIfStageMonCnfgArgs->ifaGainErrThresh = atoi(value);
			readAllParams = 1;
		}
	}
}

void MMWL_readTxPowerMonConfig(rlTxPowMonConf_t* rlTxPowerMonCnfgArgs)
{
	int readAllParams = 0;
	char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
	/*seek the pointer to starting of the file so that
	we dont miss any parameter*/
	fseek(mmwl_configfPtr, 0, SEEK_SET);
	/*parse the parameters by reading each line of the config file*/
	while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
		&& (readAllParams == 0))
	{
		/* Skip blank lines and comments */
		if (buff[0] == '\n' || buff[0] == '#')
		{
			continue;
		}

		/* Parse name/value pair from line */
		s = strtok(buff, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(name, s, STRINGLEN);
		}
		s = strtok(NULL, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(value, s, STRINGLEN);
		}
		MMWL_trim(value);

		if (strcmp(name, "TxPowerMon_profileIndx") == 0)
			rlTxPowerMonCnfgArgs->profileIndx = atoi(value);

		if (strcmp(name, "TxPowerMon_rfFreqBitMask") == 0)
			rlTxPowerMonCnfgArgs->rfFreqBitMask = atoi(value);

		if (strcmp(name, "TxPowerMon_reportMode") == 0)
			rlTxPowerMonCnfgArgs->reportMode = atoi(value);

		if (strcmp(name, "TxPowerMon_txPowAbsErrThresh") == 0)
			rlTxPowerMonCnfgArgs->txPowAbsErrThresh = atoi(value);

		if (strcmp(name, "TxPowerMon_txPowFlatnessErrThresh") == 0)
		{
			rlTxPowerMonCnfgArgs->txPowFlatnessErrThresh = atoi(value);
			readAllParams = 1;
		}
	}
}

void MMWL_readTxBallbreakMonConfig(rlTxBallbreakMonConf_t* rlTxBallbreakMonCnfgArgs)
{
	int readAllParams = 0;
	char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
	/*seek the pointer to starting of the file so that
	we dont miss any parameter*/
	fseek(mmwl_configfPtr, 0, SEEK_SET);
	/*parse the parameters by reading each line of the config file*/
	while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
		&& (readAllParams == 0))
	{
		/* Skip blank lines and comments */
		if (buff[0] == '\n' || buff[0] == '#')
		{
			continue;
		}

		/* Parse name/value pair from line */
		s = strtok(buff, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(name, s, STRINGLEN);
		}
		s = strtok(NULL, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(value, s, STRINGLEN);
		}
		MMWL_trim(value);

		if (strcmp(name, "TxBallbreakMon_reportMode") == 0)
			rlTxBallbreakMonCnfgArgs->reportMode = atoi(value);

		if (strcmp(name, "TxBallbreakMon_txReflCoeffMagThresh") == 0)
		{
			rlTxBallbreakMonCnfgArgs->txReflCoeffMagThresh = atoi(value);
			readAllParams = 1;
		}
	}
}

void MMWL_readTxPhaseShifterMonConfig(rlTxPhShiftMonConf_t* rlTxPhShiftMonCnfgArgs)
{
	int readAllParams = 0;
	char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
	/*seek the pointer to starting of the file so that
	we dont miss any parameter*/
	fseek(mmwl_configfPtr, 0, SEEK_SET);
	/*parse the parameters by reading each line of the config file*/
	while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
		&& (readAllParams == 0))
	{
		/* Skip blank lines and comments */
		if (buff[0] == '\n' || buff[0] == '#')
		{
			continue;
		}

		/* Parse name/value pair from line */
		s = strtok(buff, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(name, s, STRINGLEN);
		}
		s = strtok(NULL, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(value, s, STRINGLEN);
		}
		MMWL_trim(value);

		if (strcmp(name, "TxPhShiftMon_profileIndx") == 0)
			rlTxPhShiftMonCnfgArgs->profileIndx = atoi(value);

		if (strcmp(name, "TxPhShiftMon_reportMode") == 0)
			rlTxPhShiftMonCnfgArgs->reportMode = atoi(value);

		if (strcmp(name, "TxPhShiftMon_phShifterMonCfg") == 0)
			rlTxPhShiftMonCnfgArgs->phShifterMonCfg = atoi(value);

		if (strcmp(name, "TxPhShiftMon_rxEn") == 0)
			rlTxPhShiftMonCnfgArgs->rxEn = atoi(value);

		if (strcmp(name, "TxPhShiftMon_monChirpSlope") == 0)
			rlTxPhShiftMonCnfgArgs->monChirpSlope = atoi(value);

		if (strcmp(name, "TxPhShiftMon_phShifterIncVal1") == 0)
			rlTxPhShiftMonCnfgArgs->phShifterIncVal1 = atoi(value);

		if (strcmp(name, "TxPhShiftMon_phShifterIncVal2") == 0)
			rlTxPhShiftMonCnfgArgs->phShifterIncVal2 = atoi(value);

		if (strcmp(name, "TxPhShiftMon_phShifterIncVal3") == 0)
			rlTxPhShiftMonCnfgArgs->phShifterIncVal3 = atoi(value);

		if (strcmp(name, "TxPhShiftMon_phShifterIncVal4") == 0)
			rlTxPhShiftMonCnfgArgs->phShifterIncVal4 = atoi(value);

		if (strcmp(name, "TxPhShiftMon_phShifterMon1") == 0)
			rlTxPhShiftMonCnfgArgs->phShifterMon1 = atoi(value);

		if (strcmp(name, "TxPhShiftMon_phShifterMon2") == 0)
			rlTxPhShiftMonCnfgArgs->phShifterMon2 = atoi(value);

		if (strcmp(name, "TxPhShiftMon_phShifterMon3") == 0)
			rlTxPhShiftMonCnfgArgs->phShifterMon3 = atoi(value);

		if (strcmp(name, "TxPhShiftMon_phShifterMon4") == 0)
			rlTxPhShiftMonCnfgArgs->phShifterMon4 = atoi(value);

		if (strcmp(name, "TxPhShiftMon_txPhaseErrorThresh") == 0)
			rlTxPhShiftMonCnfgArgs->txPhaseErrorThresh = atoi(value);

		if (strcmp(name, "TxPhShiftMon_txAmplErrorThresh") == 0)
		{
			rlTxPhShiftMonCnfgArgs->txAmplErrorThresh = atoi(value);
			readAllParams = 1;
		}
	}
}

void MMWL_readTxIntAnaSignalMonConfig(rlTxIntAnaSignalsMonConf_t* rlTxIntAnaSignalMonCnfgArgs)
{
	int readAllParams = 0;
	char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
	/*seek the pointer to starting of the file so that
	we dont miss any parameter*/
	fseek(mmwl_configfPtr, 0, SEEK_SET);
	/*parse the parameters by reading each line of the config file*/
	while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
		&& (readAllParams == 0))
	{
		/* Skip blank lines and comments */
		if (buff[0] == '\n' || buff[0] == '#')
		{
			continue;
		}

		/* Parse name/value pair from line */
		s = strtok(buff, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(name, s, STRINGLEN);
		}
		s = strtok(NULL, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(value, s, STRINGLEN);
		}
		MMWL_trim(value);

		if (strcmp(name, "TxIntAnaSignalMon_profileIndx") == 0)
			rlTxIntAnaSignalMonCnfgArgs->profileIndx = atoi(value);

		if (strcmp(name, "TxIntAnaSignalMon_reportMode") == 0)
			rlTxIntAnaSignalMonCnfgArgs->reportMode = atoi(value);

		if (strcmp(name, "TxIntAnaSignalMon_txPhShiftDacMonThresh") == 0)
		{
			rlTxIntAnaSignalMonCnfgArgs->txPhShiftDacMonThresh = atoi(value);
			readAllParams = 1;
		}
	}
}

void MMWL_readRxIntAnaSignalMonConfig(rlRxIntAnaSignalsMonConf_t* rlRxIntAnaSignalMonCnfgArgs)
{
	int readAllParams = 0;
	char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
	/*seek the pointer to starting of the file so that
	we dont miss any parameter*/
	fseek(mmwl_configfPtr, 0, SEEK_SET);
	/*parse the parameters by reading each line of the config file*/
	while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
		&& (readAllParams == 0))
	{
		/* Skip blank lines and comments */
		if (buff[0] == '\n' || buff[0] == '#')
		{
			continue;
		}

		/* Parse name/value pair from line */
		s = strtok(buff, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(name, s, STRINGLEN);
		}
		s = strtok(NULL, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(value, s, STRINGLEN);
		}
		MMWL_trim(value);

		if (strcmp(name, "RxIntAnaSignalMon_profileIndx") == 0)
			rlRxIntAnaSignalMonCnfgArgs->profileIndx = atoi(value);

		if (strcmp(name, "RxIntAnaSignalMon_reportMode") == 0)
		{
			rlRxIntAnaSignalMonCnfgArgs->reportMode = atoi(value);
			readAllParams = 1;
		}
	}
}

void MMWL_readPmClkLoIntAnaSignalsMonConfig(rlPmClkLoIntAnaSignalsMonConf_t* rlPmClkLoIntAnaSignalsMonCnfgArgs)
{
	int readAllParams = 0;
	char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
	/*seek the pointer to starting of the file so that
	we dont miss any parameter*/
	fseek(mmwl_configfPtr, 0, SEEK_SET);
	/*parse the parameters by reading each line of the config file*/
	while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
		&& (readAllParams == 0))
	{
		/* Skip blank lines and comments */
		if (buff[0] == '\n' || buff[0] == '#')
		{
			continue;
		}

		/* Parse name/value pair from line */
		s = strtok(buff, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(name, s, STRINGLEN);
		}
		s = strtok(NULL, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(value, s, STRINGLEN);
		}
		MMWL_trim(value);

		if (strcmp(name, "PmClkLoIntAnaSignalsMon_profileIndx") == 0)
			rlPmClkLoIntAnaSignalsMonCnfgArgs->profileIndx = atoi(value);

		if (strcmp(name, "PmClkLoIntAnaSignalsMon_reportMode") == 0)
			rlPmClkLoIntAnaSignalsMonCnfgArgs->reportMode = atoi(value);

		if (strcmp(name, "PmClkLoIntAnaSignalsMon_sync20GSigSel") == 0)
			rlPmClkLoIntAnaSignalsMonCnfgArgs->sync20GSigSel = atoi(value);

		if (strcmp(name, "PmClkLoIntAnaSignalsMon_sync20GMinThresh") == 0)
			rlPmClkLoIntAnaSignalsMonCnfgArgs->sync20GMinThresh = atoi(value);

		if (strcmp(name, "PmClkLoIntAnaSignalsMon_sync20GMaxThresh") == 0)
		{
			rlPmClkLoIntAnaSignalsMonCnfgArgs->sync20GMaxThresh = atoi(value);
			readAllParams = 1;
		}
	}
}

void MMWL_readPllContrlVoltMonConfig(rlPllContrVoltMonConf_t* rlPllContrlVoltMonCnfgArgs)
{
	int readAllParams = 0;
	char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
	/*seek the pointer to starting of the file so that
	we dont miss any parameter*/
	fseek(mmwl_configfPtr, 0, SEEK_SET);
	/*parse the parameters by reading each line of the config file*/
	while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
		&& (readAllParams == 0))
	{
		/* Skip blank lines and comments */
		if (buff[0] == '\n' || buff[0] == '#')
		{
			continue;
		}

		/* Parse name/value pair from line */
		s = strtok(buff, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(name, s, STRINGLEN);
		}
		s = strtok(NULL, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(value, s, STRINGLEN);
		}
		MMWL_trim(value);

		if (strcmp(name, "PllContrlVoltMon_reportMode") == 0)
			rlPllContrlVoltMonCnfgArgs->reportMode = atoi(value);

		if (strcmp(name, "PllContrlVoltMon_signalEnables") == 0)
		{
			rlPllContrlVoltMonCnfgArgs->signalEnables = atoi(value);
			readAllParams = 1;
		}
	}
}

void MMWL_readSynthFreqMonConfig(rlSynthFreqMonConf_t* rlSynthFreqMonCnfgArgs)
{
	int readAllParams = 0;
	char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
	/*seek the pointer to starting of the file so that
	we dont miss any parameter*/
	fseek(mmwl_configfPtr, 0, SEEK_SET);
	/*parse the parameters by reading each line of the config file*/
	while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
		&& (readAllParams == 0))
	{
		/* Skip blank lines and comments */
		if (buff[0] == '\n' || buff[0] == '#')
		{
			continue;
		}

		/* Parse name/value pair from line */
		s = strtok(buff, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(name, s, STRINGLEN);
		}
		s = strtok(NULL, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(value, s, STRINGLEN);
		}
		MMWL_trim(value);

		if (strcmp(name, "SynthFreqMon_profileIndx") == 0)
			rlSynthFreqMonCnfgArgs->profileIndx = atoi(value);

		if (strcmp(name, "SynthFreqMon_reportMode") == 0)
			rlSynthFreqMonCnfgArgs->reportMode = atoi(value);

		if (strcmp(name, "SynthFreqMon_freqErrThresh") == 0)
			rlSynthFreqMonCnfgArgs->freqErrThresh = atoi(value);

		if (strcmp(name, "SynthFreqMon_monStartTime") == 0)
			rlSynthFreqMonCnfgArgs->monStartTime = atoi(value);

		if (strcmp(name, "SynthFreqMon_monitorMode") == 0)
			rlSynthFreqMonCnfgArgs->monitorMode = atoi(value);

		if (strcmp(name, "SynthFreqMon_VcoMonEn") == 0)
		{
			rlSynthFreqMonCnfgArgs->vcoMonEn = atoi(value);
			readAllParams = 1;
		}
	}
}

void MMWL_readRxMixMonConfig(rlRxMixInPwrMonConf_t* rlRxMixMonCnfgArgs)
{
	int readAllParams = 0;
	char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
	/*seek the pointer to starting of the file so that
	we dont miss any parameter*/
	fseek(mmwl_configfPtr, 0, SEEK_SET);
	/*parse the parameters by reading each line of the config file*/
	while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
		&& (readAllParams == 0))
	{
		/* Skip blank lines and comments */
		if (buff[0] == '\n' || buff[0] == '#')
		{
			continue;
		}

		/* Parse name/value pair from line */
		s = strtok(buff, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(name, s, STRINGLEN);
		}
		s = strtok(NULL, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(value, s, STRINGLEN);
		}
		MMWL_trim(value);

		if (strcmp(name, "RxMixMon_profileIndx") == 0)
			rlRxMixMonCnfgArgs->profileIndx = atoi(value);

		if (strcmp(name, "RxMixMon_reportMode") == 0)
			rlRxMixMonCnfgArgs->reportMode = atoi(value);

		if (strcmp(name, "RxMixMon_txEnable") == 0)
			rlRxMixMonCnfgArgs->txEnable = atoi(value);

		if (strcmp(name, "RxMixMon_thresholds") == 0)
		{
			rlRxMixMonCnfgArgs->thresholds = atoi(value);
			readAllParams = 1;
		}
	}
}

void MMWL_readDualClkCompMonConfig(rlDualClkCompMonConf_t* rlDualClkCompMonCnfgArgs)
{
	int readAllParams = 0;
	char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
	/*seek the pointer to starting of the file so that
	we dont miss any parameter*/
	fseek(mmwl_configfPtr, 0, SEEK_SET);
	/*parse the parameters by reading each line of the config file*/
	while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
		&& (readAllParams == 0))
	{
		/* Skip blank lines and comments */
		if (buff[0] == '\n' || buff[0] == '#')
		{
			continue;
		}

		/* Parse name/value pair from line */
		s = strtok(buff, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(name, s, STRINGLEN);
		}
		s = strtok(NULL, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(value, s, STRINGLEN);
		}
		MMWL_trim(value);

		if (strcmp(name, "DualClkCompMon_reportMode") == 0)
			rlDualClkCompMonCnfgArgs->reportMode = atoi(value);

		if (strcmp(name, "DualClkCompMon_dccPairEnables") == 0)
		{
			rlDualClkCompMonCnfgArgs->dccPairEnables = atoi(value);
			readAllParams = 1;
		}
	}
}

void MMWL_readRxGainPhaMonConfig(rlRxGainPhaseMonConf_t* rlRxGainPhaMonCnfgArgs)
{
	int readAllParams = 0;
	char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
	/*seek the pointer to starting of the file so that
	we dont miss any parameter*/
	fseek(mmwl_configfPtr, 0, SEEK_SET);
	/*parse the parameters by reading each line of the config file*/
	while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
		&& (readAllParams == 0))
	{
		/* Skip blank lines and comments */
		if (buff[0] == '\n' || buff[0] == '#')
		{
			continue;
		}

		/* Parse name/value pair from line */
		s = strtok(buff, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(name, s, STRINGLEN);
		}
		s = strtok(NULL, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(value, s, STRINGLEN);
		}
		MMWL_trim(value);

		if (strcmp(name, "RxGainPhaMon_profileIndx") == 0)
			rlRxGainPhaMonCnfgArgs->profileIndx = atoi(value);

		if (strcmp(name, "RxGainPhaMon_rfFreqBitMask") == 0)
			rlRxGainPhaMonCnfgArgs->rfFreqBitMask = atoi(value);

		if (strcmp(name, "RxGainPhaMon_reportMode") == 0)
			rlRxGainPhaMonCnfgArgs->reportMode = atoi(value);

		if (strcmp(name, "RxGainPhaMon_txSel") == 0)
			rlRxGainPhaMonCnfgArgs->txSel = atoi(value);

		if (strcmp(name, "RxGainPhaMon_rxGainAbsThresh") == 0)
			rlRxGainPhaMonCnfgArgs->rxGainAbsThresh = atoi(value);

		if (strcmp(name, "RxGainPhaMon_rxGainMismatchErrThresh") == 0)
			rlRxGainPhaMonCnfgArgs->rxGainMismatchErrThresh = atoi(value);

		if (strcmp(name, "RxGainPhaMon_rxGainFlatnessErrThresh") == 0)
			rlRxGainPhaMonCnfgArgs->rxGainFlatnessErrThresh = atoi(value);

		if (strcmp(name, "RxGainPhaMon_rxGainPhaseMismatchErrThresh") == 0)
			rlRxGainPhaMonCnfgArgs->rxGainPhaseMismatchErrThresh = atoi(value);

		if (strcmp(name, "RxGainPhaMon_rxGainMismatchOffsetVal") == 0)
		{
			int32_t i, j;
			for (i = 0; i <= 3; i++)
			{
				for (j = 0; j <= 2; j++)
				{
					rlRxGainPhaMonCnfgArgs->rxGainMismatchOffsetVal[i][j] = atoi(value);
				}
			}			
		}

		if (strcmp(name, "RxGainPhaMon_rxGainPhaseMismatchOffsetVal") == 0)
		{
			int32_t k, l;
			for (k = 0; k <= 3; k++)
			{
				for (l = 0; l <= 2; l++)
				{
					rlRxGainPhaMonCnfgArgs->rxGainPhaseMismatchOffsetVal[k][l] = atoi(value);
				}
			}
			readAllParams = 1;
		}
	}
}

void MMWL_readTxGainPhaMonConfig(rlTxGainPhaseMismatchMonConf_t* rlTxGainPhaMonCnfgArgs)
{
	int readAllParams = 0;
	char *s, buff[256], name[STRINGLEN], value[STRINGLEN];
	/*seek the pointer to starting of the file so that
	we dont miss any parameter*/
	fseek(mmwl_configfPtr, 0, SEEK_SET);
	/*parse the parameters by reading each line of the config file*/
	while (((s = fgets(buff, sizeof buff, mmwl_configfPtr)) != NULL)
		&& (readAllParams == 0))
	{
		/* Skip blank lines and comments */
		if (buff[0] == '\n' || buff[0] == '#')
		{
			continue;
		}

		/* Parse name/value pair from line */
		s = strtok(buff, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(name, s, STRINGLEN);
		}
		s = strtok(NULL, "=");
		if (s == NULL)
		{
			continue;
		}
		else
		{
			strncpy(value, s, STRINGLEN);
		}
		MMWL_trim(value);

		if (strcmp(name, "TxGainPhaMon_profileIndx") == 0)
			rlTxGainPhaMonCnfgArgs->profileIndx = atoi(value);

		if (strcmp(name, "TxGainPhaMon_rfFreqBitMask") == 0)
			rlTxGainPhaMonCnfgArgs->rfFreqBitMask = atoi(value);

		if (strcmp(name, "TxGainPhaMon_txEn") == 0)
			rlTxGainPhaMonCnfgArgs->txEn = atoi(value);

		if (strcmp(name, "TxGainPhaMon_rxEn") == 0)
			rlTxGainPhaMonCnfgArgs->rxEn = atoi(value);

		if (strcmp(name, "TxGainPhaMon_reportMode") == 0)
			rlTxGainPhaMonCnfgArgs->reportMode = atoi(value);

		if (strcmp(name, "TxGainPhaMon_monChirpSlope") == 0)
			rlTxGainPhaMonCnfgArgs->monChirpSlope = atoi(value);

		if (strcmp(name, "TxGainPhaMon_txGainMismatchThresh") == 0)
			rlTxGainPhaMonCnfgArgs->txGainMismatchThresh = atoi(value);

		if (strcmp(name, "TxGainPhaMon_txPhaseMismatchThresh") == 0)
			rlTxGainPhaMonCnfgArgs->txPhaseMismatchThresh = atoi(value);

		if (strcmp(name, "TxGainPhaMon_txGainMismatchOffsetVal") == 0)
		{
			int32_t i, j;
			for (i = 0; i <= 2; i++)
			{
				for (j = 0; j <= 2; j++)
				{
					rlTxGainPhaMonCnfgArgs->txGainMismatchOffsetVal[i][j] = atoi(value);
				}
			}
		}

		if (strcmp(name, "TxGainPhaMon_txPhaseMismatchOffsetVal") == 0)
		{
			int32_t k, l;
			for (k = 0; k <= 2; k++)
			{
				for (l = 0; l <= 2; l++)
				{
					rlTxGainPhaMonCnfgArgs->txPhaseMismatchOffsetVal[k][l] = atoi(value);
				}
			}
			readAllParams = 1;
		}
	}
}

/** @fn int MMWL_openConfigFile()
*
*   @brief Opens MMWave config file
*
*   @return int Success - 0, Failure - Error Code
*
*   Opens MMWave config file
*/
int MMWL_openConfigFile()
{
    /*open config file to read parameters*/
    if (mmwl_configfPtr == NULL)
    {
        mmwl_configfPtr = fopen("mmwaveconfig.txt", "r");
        if (mmwl_configfPtr == NULL)
        {
            printf("failed to open config file\n");
            return -1;
        }
    }
    return 0;
}

/** @fn void MMWL_closeConfigFile()
*
*   @brief Close MMWave config file
*
*   Close MMWave config file
*/
void MMWL_closeConfigFile()
{
    /* Close config file */
    fclose(mmwl_configfPtr);
    mmwl_configfPtr = NULL;
}

