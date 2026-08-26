/****************************************************************************************
* FileName     : mmw_config.h
*
* Description  : This file implements mmwave link example application.
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

/******************************************************************************
* MACROS
*******************************************************************************
*/
/*string length for reading from config file*/
#define STRINGLEN 100

/*! \brief
* Global Configuration Structure
*/
typedef struct rlDevGlobalCfg
{
	/**
	 * @brief  Advanced frame test enable/disable
	 *         1 - Enable; 0 - Disable
	 */
	unsigned char LinkAdvanceFrameTest;
	/**
	 * @brief  Continuous mode test enable/disable
	 *         1 - Enable; 0 - Disable
	 */
	unsigned char LinkContModeTest;
	/**
	 * @brief  Dynamic chirp test enable/disable
	 *         1 - Enable; 0 - Disable
	 */
	unsigned char LinkDynChirpTest;
	/**
	 * @brief  Dynamic profile test enable/disable
	 *         1 - Enable; 0 - Disable
	 */
	unsigned char LinkDynProfileTest;
	/**
	 * @brief  Advanced chirp test enable/disable
	 *         1 - Enable; 0 - Disable
	 */
	unsigned char LinkAdvChirpTest;
	/**
	 * @brief  Firmware download enable/disable
	 *         1 - Enable; 0 - Disable
	 */
	unsigned char EnableFwDownload;
	/**
	 * @brief  mmwavelink logging enable/disable
	 *         1 - Enable; 0 - Disable
	 */
	unsigned char EnableMmwlLogging;
	/**
	 * @brief  Calibration enable/disable
	 *         To perform calibration store/restore
	 *         1 - Enable; 0 - Disable
	 */
	unsigned char CalibEnable;
	/**
	 * @brief  Calibration Store/Restore
	 *         If CalibEnable = 1, then whether to store/restore
	 *         1 - Store; 0 - Restore
	 */
	unsigned char CalibStoreRestore;
	/**
	 * @brief  Bitmap of the Devices to be enabled
               1 - Master ; 2 - Slave1 ; 4 - Slave2 ; 8 - Slave3
	 */
	unsigned char CascadeDeviceMap;
	/**
	 * @brief  Transport mode
	 *         1 - I2C; 0 - SPI
	 */
	unsigned char TransferMode;

} rlDevGlobalCfg_t;

/******************************************************************************
* PARSE FUNCTION DECLARATION
******************************************************************************
*/

/*read PowerOn configurations*/
void MMWL_readPowerOnMaster(rlClientCbs_t *clientCtx);

/*read frame configurations*/
void MMWL_readFrameConfig(rlFrameCfg_t *frameCfgArgs);

/*read Advance frame configurations*/
void MMWL_readAdvFrameConfig(rlAdvFrameCfg_t *rlAdvFrameCfgArgs);

/*read chirp configurations*/
void MMWL_readChirpConfig(rlChirpCfg_t *chirpCfgArgs);

/*read profile configurations*/
void MMWL_readProfileConfig(rlProfileCfg_t *profileCfgArgs);

/*read lvds lane configurations*/
void MMWL_readLvdsLaneConfig(rlDevLvdsLaneCfg_t *lvdsLaneCfgArgs);

/*read lane configurations*/
void MMWL_readLaneConfig(rlDevLaneEnable_t *laneEnCfgArgs);

/*read CSI2 lane configurations*/
void MMWL_readCSI2LaneConfig(rlDevCsi2Cfg_t *CSI2LaneCfgArgs);

/*read high speed clock configurations*/
void MMWL_readSetHsiClock(rlDevHsiClk_t *hsiClkgs);

/*read LVDS clock configuration*/
void MMWL_readLvdsClkConfig(rlDevDataPathClkCfg_t *lvdsClkCfgArgs);

/*read data path configurations*/
void MMWL_readDataPathConfig(rlDevDataPathCfg_t *dataPathCfgArgs);

/*read low power configurations*/
void MMWL_readLowPowerConfig(rlLowPowerModeCfg_t *rfLpModeCfgArgs);

/*read data format configuration*/
void MMWL_readDataFmtConfig(rlDevDataFmtCfg_t *dataFmtCfgArgs);

/*read AdcOut configuration*/
void MMWL_readAdcOutConfig(rlAdcOutCfg_t *adcOutCfgArgs);

/*read channel config parameters*/
void MMWL_readChannelConfig(rlChanCfg_t *rfChanCfgArgs, 
                            unsigned short cascade);

/*read the parameters for power on master*/
void MMWL_readPowerOnMaster(rlClientCbs_t *clientCtx);

/*Read all global variable configurations from config file*/
void MMWL_getGlobalConfigStatus(rlDevGlobalCfg_t *rlDevGlobalCfgArgs);

/*read continuous mode config args*/
void MMWL_readContModeConfig(rlContModeCfg_t * rlContModeCfgArgs);

/*read Dynamic Chirp config args*/
void MMWL_readDynChirpConfig(rlDynChirpCfg_t* rldynChirpCfgArgs);

/*read Programmable filter config args*/
void MMWL_readProgFiltConfig(rlRfProgFiltConf_t* rlProgFiltCnfgArgs);

/*read Analog montoring enable config args*/
void MMWL_readAnaMonConfig(rlMonAnaEnables_t* rlAnaMonCnfgArgs);

/*read Temp monitoring config args*/
void MMWL_readTempMonConfig(rlTempMonConf_t* rlTempMonCnfgArgs);

/*read Rx Gain phase monitoring config args*/
void MMWL_readRxGainPhaMonConfig(rlRxGainPhaseMonConf_t* rlRxGainPhaMonCnfgArgs);

/*read Rx Noise monitoring config args*/
void MMWL_readRxNoiseMonConfig(rlRxNoiseMonConf_t* rlRxNoiseMonCnfgArgs);

/*read Rx IF Stage monitoring config args*/
void MMWL_readRxIfStageMonConfig(rlRxIfStageMonConf_t* rlRxIfStageMonCnfgArgs);

/*read Tx Power monitoring config args*/
void MMWL_readTxPowerMonConfig(rlTxPowMonConf_t* rlTxPowerMonCnfgArgs);

/*read Tx ball break monitoring config args*/
void MMWL_readTxBallbreakMonConfig(rlTxBallbreakMonConf_t* rlTxBallbreakMonCnfgArgs);

/*read Tx Gain phase monitoring config args*/
void MMWL_readTxGainPhaMonConfig(rlTxGainPhaseMismatchMonConf_t* rlTxGainPhaMonCnfgArgs);

/*read Tx Phase shifter monitoring config args*/
void MMWL_readTxPhaseShifterMonConfig(rlTxPhShiftMonConf_t* rlTxPhShiftMonCnfgArgs);

/*read Tx Internal Analog signal monitoring config args*/
void MMWL_readTxIntAnaSignalMonConfig(rlTxIntAnaSignalsMonConf_t* rlTxIntAnaSignalMonCnfgArgs);

/*read Rx Internal Analog signal monitoring config args*/
void MMWL_readRxIntAnaSignalMonConfig(rlRxIntAnaSignalsMonConf_t* rlRxIntAnaSignalMonCnfgArgs);

/*read PMCLKO Internal Analog signal monitoring config args*/
void MMWL_readPmClkLoIntAnaSignalsMonConfig(rlPmClkLoIntAnaSignalsMonConf_t* rlPmClkLoIntAnaSignalsMonCnfgArgs);

/*read PLL Control voltage monitoring config args*/
void MMWL_readPllContrlVoltMonConfig(rlPllContrVoltMonConf_t* rlPllContrlVoltMonCnfgArgs);

/*read Synth Frequency monitoring config args*/
void MMWL_readSynthFreqMonConfig(rlSynthFreqMonConf_t* rlSynthFreqMonCnfgArgs);

/*read Dual clock comparator monitoring config args*/
void MMWL_readDualClkCompMonConfig(rlDualClkCompMonConf_t* rlDualClkCompMonCnfgArgs);

/*read Rx mixer monitoring config args*/
void MMWL_readRxMixMonConfig(rlRxMixInPwrMonConf_t* rlRxMixMonCnfgArgs);

/*get rid of trailing and leading whitespace along with "\n"*/
char *MMWL_trim(char * s);

/* Open Configuration file in read mode */
int MMWL_openConfigFile();

/* Close Configuration file */
void MMWL_closeConfigFile();

/*Trim the string trailing and leading whitespace*/
char *MMWL_trim(char * s);
