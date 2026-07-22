#from libc.stdio cimport printf as DEBUG_PRINT
from libc.stdio cimport printf
from libc.stdint cimport uint8_t, int8_t,int16_t,uint16_t, int32_t, uint32_t
from libc.math cimport ceil
from libc.string cimport strncpy
import time

cdef extern from "ti/mmwave/mmwave.h":
    '''
    FILE* rls_traceF = NULL;
    void CloseTraceFile() {
    if (rls_traceF != NULL) {
        fclose(rls_traceF);
        rls_traceF = NULL;
    }
    }
    '''
    ctypedef struct rlProfileCfg_t:
        uint16_t profileId
        uint8_t pfVcoSelect
        uint8_t pfCalLutUpdate
        uint32_t startFreqConst
        uint32_t idleTimeConst
        uint32_t adcStartTimeConst
        uint32_t rampEndTime
        uint32_t txOutPowerBackoffCode
        uint32_t txPhaseShifter
        int16_t freqSlopeConst
        int16_t txStartTime
        uint16_t numAdcSamples
        uint16_t digOutSampleRate
        uint8_t hpfCornerFreq1
        uint8_t hpfCornerFreq2
        uint16_t txCalibEnCfg
        uint16_t rxGain
        uint16_t reserved

    ctypedef struct rlFrameCfg_t:
        uint16_t reserved0
        uint16_t chirpStartIdx
        uint16_t chirpEndIdx
        uint16_t numLoops
        uint16_t numFrames
        uint16_t numAdcSamples
        uint32_t framePeriodicity
        uint16_t triggerSelect
        uint16_t reserved1
        uint32_t frameTriggerDelay

    ctypedef struct rlChirpCfg_t:
        uint16_t chirpStartIdx
        uint16_t chirpEndIdx
        uint16_t profileId
        uint16_t reserved
        uint32_t startFreqVar
        uint16_t freqSlopeVar
        uint16_t idleTimeVar
        uint16_t adcStartTimeVar
        uint16_t txEnable
    
    ctypedef struct rlChanCfg_t:
        uint16_t rxChannelEn
        uint16_t txChannelEn
        uint16_t cascading
        uint16_t cascadingPinoutCfg
    
    ctypedef struct rlAdcBitFormat_t:
        uint32_t b2AdcBits
        uint32_t b6Reserved0
        uint32_t b8FullScaleReducFctr
        uint32_t b2AdcOutFmt
        uint32_t b14Reserved1

    ctypedef struct rlAdcOutCfg_t:
        rlAdcBitFormat_t fmt
        uint16_t reserved0
        uint16_t reserved1

    ctypedef struct rlDevDataFmtCfg_t:
        uint16_t rxChannelEn
        uint16_t adcBits
        uint16_t adcFmt
        uint8_t iqSwapSel
        uint8_t chInterleave
        uint32_t reserved
    
    ctypedef struct rlRfLdoBypassCfg_t:
        uint16_t ldoBypassEnable
        uint8_t supplyMonIrDrop
        uint8_t ioSupplyIndicator
    
    ctypedef struct rlLowPowerModeCfg_t:
        uint16_t reserved
        uint16_t lpAdcMode
    
    ctypedef struct rlRfMiscConf_t:
        uint32_t miscCtl
        uint32_t reserved
    
    ctypedef struct rlDevDataPathCfg_t:
        uint8_t intfSel
        uint8_t transferFmtPkt0
        uint8_t transferFmtPkt1
        uint8_t cqConfig
        uint8_t cq0TransSize
        uint8_t cq1TransSize
        uint8_t reserved

    ctypedef struct rlDevDataPathClkCfg_t:
        uint8_t laneClkCfg
        uint8_t dataRate
        uint16_t reserved

    ctypedef struct rlDevHsiClk_t:
        uint16_t hsiClk
        uint16_t reserved

    ctypedef struct rlDevCsi2Cfg_t:
        uint32_t lanePosPolSel
        uint8_t lineStartEndDis
        uint8_t reserved0
        uint16_t reserved0
    
    ctypedef struct rlTdaArmCfg_t:
        unsigned int framePeriodicity
        unsigned char* captureDirectory
        unsigned int numberOfFilesToAllocate
        unsigned int dataPacking
        unsigned int numberOfFramesToCapture

    int MMWL_chirpConfig(unsigned char deviceMap, rlChirpCfg_t chirpCfgArgs)
    unsigned int createDevMapFromDevId(unsigned char devId)
    int MMWL_DevicePowerUp(unsigned char deviceMap, uint32_t rlClientCbsTimeout, uint32_t sopTimeout)
    int MMWL_firmwareDownload(unsigned char deviceMap)
    int MMWL_setDeviceCrcType(unsigned char deviceMap)
    int MMWL_rfEnable(unsigned char deviceMap)
    int MMWL_channelConfig(unsigned char deviceMap, unsigned short cascade, rlChanCfg_t rfChanCfgArgs)
    int MMWL_adcOutConfig(unsigned char deviceMap, rlAdcOutCfg_t adcOutCfgArgs)
    int MMWL_RFDeviceConfig(unsigned char deviceMap)
    int MMWL_ldoBypassConfig(unsigned char deviceMap, rlRfLdoBypassCfg_t rfLdoBypassCfgArgs)
    int MMWL_dataFmtConfig(unsigned char deviceMap, rlDevDataFmtCfg_t dataFmtCfgArgs)
    int MMWL_lowPowerConfig(unsigned char deviceMap, rlLowPowerModeCfg_t rfLpModeCfgArgs)
    int MMWL_ApllSynthBwConfig(unsigned char deviceMap)
    int MMWL_setMiscConfig(unsigned char deviceMap, rlRfMiscConf_t miscCfg)
    int MMWL_rfInit(unsigned char deviceMap)
    int MMWL_dataPathConfig(unsigned char deviceMap, rlDevDataPathCfg_t datapathCfgArgs)
    int MMWL_hsiClockConfig(unsigned char deviceMap, rlDevDataPathClkCfg_t datapathClkCfgArgs, rlDevHsiClk_t hisClkgs)
    int MMWL_CSI2LaneConfig(unsigned char deviceMap, rlDevCsi2Cfg_t CSI2LaneCfgArgs)
    int MMWL_profileConfig(unsigned char deviceMap, rlProfileCfg_t profileCfgArgs)
    int MMWL_frameConfig(unsigned char deviceMap, rlFrameCfg_t frameCfgArgs, rlChanCfg_t channelCfgArgs, rlAdcOutCfg_t adcOutCfgArgs, rlDevDataPathCfg_t datapathCfgArgs, rlProfileCfg_t profileCfgArgs)
    int MMWL_AssignDeviceMap(unsigned char deviceMap,uint8_t* masterMap,uint8_t* slavesMap)
    int MMWL_ArmingTDA(rlTdaArmCfg_t tdaArmCfgArgs)
    int MMWL_StartFrame(unsigned char deviceMap)
    int MMWL_StopFrame(unsigned char deviceMap)
    int MMWL_DeArmingTDA()
    int MMWL_TDAInit(unsigned char *ipAddr , unsigned int port,uint8_t deviceMap)




# Program name, version, and other constants
cdef char* PROG_NAME = b"mmwcas"             # Name of the program
cdef char* PROG_VERSION = b"0.1"             # Program version
cdef char* PROG_COPYRIGHT = b"Copyright (C) 2024"
#DEBUG_PRINT = printf             # Debug print function

cdef int RL_RET_CODE_OK = 0               # Return code for success

# Development environment flag and other constants
cdef int DEV_ENV = 1
# Filled exclusively by mmw_set_config() from radar_configs/*.toml.
# Left at 0 / empty so calling mmw_init() without a complete TOML fails loudly
# instead of silently programming the radar with stale hardcoded values.
cdef int NUM_CHIRPS = 0
_tx_antenna_table = []
_profile_id_per_chirp = []
_config_applied = False   # True after a successful mmw_set_config() call

# Mbps -> rlDevDataPathClkCfg_t.dataRate field code (rl_device.h). DDR-only
# rates (600/400/225/150) have no SDR equivalent per the SDK header docs.
_DATA_RATE_CODE_DDR = {600: 1, 450: 2, 400: 3, 300: 4, 225: 5, 150: 6}
_DATA_RATE_CODE_SDR = {450: 2, 300: 4}

# Mbps -> rlDevHsiClk_t.hsiClk field code (rl_device.h), per lane clock mode.
_HSI_CLK_CODE_DDR = {900: 0xD, 600: 0x9, 450: 0x5, 400: 0x1, 300: 0xA, 225: 0x6, 150: 0xB}
_HSI_CLK_CODE_SDR = {900: 0x5, 600: 0xA, 450: 0x6, 400: 0x2, 300: 0xB, 225: 0x7}


def _lookup_data_rate(mbps, lane_clk_cfg):
    """Map a requested lane data rate (Mbps) + laneClkCfg (0=SDR, 1=DDR) to
    the (dataRate, hsiClk) field codes documented in rl_device.h. Raises
    ValueError for combinations the SDK doesn't support (e.g. 600 Mbps in
    SDR mode - DDR-only per rlDevDataPathClkCfg_t's doc comment)."""
    is_ddr = bool(lane_clk_cfg)
    rate_table = _DATA_RATE_CODE_DDR if is_ddr else _DATA_RATE_CODE_SDR
    hsi_table = _HSI_CLK_CODE_DDR if is_ddr else _HSI_CLK_CODE_SDR
    if mbps not in rate_table:
        mode = "DDR" if is_ddr else "SDR"
        raise ValueError(
            f"Unsupported dataRate_Mbps={mbps} for laneClkCfg={lane_clk_cfg} ({mode}). "
            f"Valid {mode} rates: {sorted(rate_table)}"
        )
    return rate_table[mbps], hsi_table[mbps]


cdef char* CRED=b"\e[0;31m"    # Terminal code for regular red text
cdef char* CGREEN=b"\e[0;32m"    # Terminal code for regular greed text
cdef char* CRESET=b"\e[0m"       # Clear reset terminal color

cdef int TRUE = 1


# Device configuration struct
ctypedef struct devConfig_t:
    uint8_t deviceMap         # Device Map (1: Master, 2: Slave1, 4: Slave2, 8: Slave3)
    uint8_t masterMap         # Master device map (value: 1)
    uint8_t slavesMap         # Slave devices map (value: 14)

    rlFrameCfg_t frameCfg

    # Profile configuration - PATCHED: 3 separate profiles instead of 1,
    # matching mimo.c (idle time differs per chirp: 175us/7us/7us)
    rlProfileCfg_t profileCfg[3]

    # Chirp configuration
    rlChirpCfg_t chirpCfg

    # Channel configuration
    rlChanCfg_t channelCfg

    # ADC output configuration
    rlAdcOutCfg_t adcOutCfg

    # Data format configuration
    rlDevDataFmtCfg_t dataFmtCfg

    # LDO Bypass configuration
    rlRfLdoBypassCfg_t ldoCfg

    # Low Power Mode configuration
    rlLowPowerModeCfg_t lpmCfg

    # Miscellaneous configuration
    rlRfMiscConf_t miscCfg

    # Datapath configuration
    rlDevDataPathCfg_t datapathCfg

    # Datapath clock configuration
    rlDevDataPathClkCfg_t datapathClkCfg

    # High Speed Clock configuration
    rlDevHsiClk_t hsClkCfg

    # CSI2 configuration
    rlDevCsi2Cfg_t csi2LaneCfg

"""! \brief
* C-struct memory the mmWaveLink SDK consumes.
*
* These are intentionally ZEROED - they are NOT a fallback radar config.
* radar_configs/*.toml (via radar_config.py -> mmw_set_config) is the only
* source of RF/geometry values. mmw_set_config() requires every field it
* applies; a missing TOML key raises ValueError instead of silently using
* a hardcoded default. mmw_init() refuses to run until mmw_set_config()
* has succeeded once (_config_applied).
*
* Encoding applied by mmw_set_config() (same scale factors as TI docs):
*   startFreqConst: 1 LSB = 53.6441803 Hz
*   freqSlopeConst: 1 LSB = 48.2797623 kHz/us
*   idleTimeConst / adcStartTimeConst / rampEndTime / txStartTime: 1 LSB = 10ns
*   framePeriodicity / frameTriggerDelay: 1 LSB = 5ns
*/
"""
# Zero-initialized C structs - filled exclusively by mmw_set_config() from TOML.
cdef rlProfileCfg_t profileCfgArgs0
cdef rlProfileCfg_t profileCfgArgs1
cdef rlProfileCfg_t profileCfgArgs2
cdef rlFrameCfg_t frameCfgArgs
cdef rlChirpCfg_t chirpCfgArgs
cdef rlChanCfg_t channelCfgArgs
cdef rlAdcOutCfg_t adcOutCfgArgs
cdef rlDevDataFmtCfg_t dataFmtCfgArgs
cdef rlRfLdoBypassCfg_t ldoCfgArgs
cdef rlLowPowerModeCfg_t lpmCfgArgs
cdef rlRfMiscConf_t miscCfgArgs
cdef rlDevDataPathCfg_t datapathCfgArgs
cdef rlDevDataPathClkCfg_t datapathClkCfgArgs
cdef rlDevHsiClk_t hsClkCfgArgs
cdef rlDevCsi2Cfg_t csi2LaneCfgArgs

"""
|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|       | Dev 1 | Dev 1 | Dev 1 | Dev 2 | Dev 2 | Dev 2 | Dev 3 | Dev 3 | Dev 3 | Dev 4 | Dev 4 | Dev 4 |
| Chirp |  TX0  |  TX1  |  TX2  |  TX 0 |  TX1  |  TX2  |  TX0  |  TX1  |  TX2  |  TX0  |  TX1  |  TX2  |
|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     1 |
|     1 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     1 |     0 |
|     2 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     1 |     0 |     0 |
|     3 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     1 |     0 |     0 |     0 |
|     4 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     1 |     0 |     0 |     0 |     0 |
|     5 |     0 |     0 |     0 |     0 |     0 |     0 |     1 |     0 |     0 |     0 |     0 |     0 |
|     6 |     0 |     0 |     0 |     0 |     0 |     1 |     0 |     0 |     0 |     0 |     0 |     0 |
|     7 |     0 |     0 |     0 |     0 |     1 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |
|     8 |     0 |     0 |     0 |     1 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |
|     9 |     0 |     0 |     1 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |
|    10 |     0 |     1 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |
|    11 |     1 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |     0 |
|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
"""


cpdef uint32_t configureMimoChirp(uint8_t devId, rlChirpCfg_t chirpCfg):
    """@brief MIMO Chirp configuration
    #* @param devId Device ID (0: master, 1: slave1, 2: slave2, 3: slave3)
    #* @param chirpCfg Initital chirp configuration
    #* @return uint32_t Configuration status

    Antenna geometry (which TX antenna, if any, transmits on each chirp) and
    the chirp->profile mapping are driven entirely by the module-level
    _tx_antenna_table / _profile_id_per_chirp (see above) - populated from
    config_dict["mimo"]["chirp"] (radar_configs/*.toml) by mmw_set_config().
    _tx_antenna_table[devId][chirpIdx] is the TX antenna index to enable for
    that device on that chirp, or -1 for RX-only (same indexing/semantics as
    utility.export_config_to_json()'s txAntennaTable handling, so the
    .mmwave.json sidecar always matches what actually got programmed).
    """
    cdef int status = 0
    cdef int i
    cdef int8_t txAnt
    dev_row = _tx_antenna_table[devId] if devId < len(_tx_antenna_table) else []

    for i in range(NUM_CHIRPS):
        txAnt = dev_row[i] if i < len(dev_row) else -1

        # Update chirp configuration
        chirpCfg.chirpStartIdx = i
        chirpCfg.chirpEndIdx = i
        # Select the profile matching this chirp index (per profileIdPerChirp
        # - default chirp0->profile0 with 175us idle, chirp1/2->profile1/2
        # with 7us idle) - same profile used across ALL devices for a given
        # chirp index, matching mimo.c.
        chirpCfg.profileId = _profile_id_per_chirp[i] if i < len(_profile_id_per_chirp) else i
        if txAnt < 0:
            chirpCfg.txEnable = 0x00
        else:
            chirpCfg.txEnable = (1 << txAnt)

        # Configure chirp and update status
        status += MMWL_chirpConfig(createDevMapFromDevId(devId), chirpCfg)

        # Print debug info
        printf(b"[CHIRP CONFIG] dev %u, chirp idx %u, profileId %u, txEnable 0x%02x, status: %d\n",
               devId, i, chirpCfg.profileId, chirpCfg.txEnable, status)
        if status != 0:
            printf(b"Configuration of chirp %d failed!\n", i)
            break

    return status

cdef void check(int status, char* success_msg, char* error_msg,
                unsigned char deviceMap, uint8_t is_required):
    """@brief Check status and print error or success message
    @param status Status value returned by a function
    @param success_msg Success message to print when status is 0
    @param error_msg Error message to print in case of error
    @param deviceMap Device map the check if related to
    @param is_required Indicates if the checking stage is required. if so,the program exits in case of failure.
    @return uint32_t Configuration status

    @note: Status is considered successful when the status integer is 0.
    Any other value is considered a failure.
    """
    # Debug info printed under DEV_ENV
    if DEV_ENV:
        printf(b"STATUS %4d | DEV MAP: %2u | ", status, deviceMap)

    # Check status
    if status == RL_RET_CODE_OK:
        if DEV_ENV:
            #printf(CGREEN)
            printf(success_msg)
            #printf(CRESET)
            printf("\n")
        return
    else:
        if DEV_ENV:
            #printf(CRED)
            printf(error_msg)
            #printf(CRESET)
            printf("\n")
        
        # If is_required is non-zero, exit the program
        if is_required != 0:
            exit(status)


cdef int32_t initMaster(rlChanCfg_t channelCfg,rlAdcOutCfg_t adcOutCfg):
    cdef unsigned int masterId = 0
    cdef unsigned int masterMap = 1U << masterId
    cdef int status = 0
    channelCfg.cascading = 1
    status += MMWL_DevicePowerUp(masterMap, 1000, 1000)
    check(status,
        b"[MASTER] Power up successful!",
        b"[MASTER] Error: Failed to power up device!", masterMap, TRUE)

    status += MMWL_firmwareDownload(masterMap)
    check(status,
        b"[MASTER] Firmware successfully uploaded!",
        b"[MASTER] Error: Firmware upload failed!", masterMap, TRUE)

    status += MMWL_setDeviceCrcType(masterMap)
    check(status,
        b"[MASTER] CRC type has been set!",
        b"[MASTER] Error: Unable to set CRC type!", masterMap, TRUE)

    status += MMWL_rfEnable(masterMap)
    check(status,
        b"[MASTER] RF successfully enabled!",
        b"[MASTER] Error: Failed to enable master RF", masterMap, TRUE)

    status += MMWL_channelConfig(masterMap, channelCfg.cascading, channelCfg)
    check(status,
        b"[MASTER] Channels successfully configured!",
        b"[MASTER] Error: Channels configuration failed!", masterMap, TRUE)

    status += MMWL_adcOutConfig(masterMap, adcOutCfg)
    check(status,
        b"[MASTER] ADC output format successfully configured!",
        b"[MASTER] Error: ADC output format configuration failed!", masterMap, TRUE)

    check(status,
        b"[MASTER] Init completed with sucess",
        b"[MASTER] Init completed with error", masterMap, TRUE)
    return status

cdef int32_t initSlaves(rlChanCfg_t channelCfg, rlAdcOutCfg_t adcOutCfg):
    cdef int status = 0
    cdef uint8_t slavesMap = (1 << 1) | (1 << 2) | (1 << 3)
    cdef unsigned int slaveMap

    # slave chip
    channelCfg.cascading = 2

    for slaveId in range(1,4):
        slaveMap = 1 << slaveId

        status += MMWL_DevicePowerUp(slaveMap, 1000, 1000)
        check(status,
            b"[SLAVE] Power up successful!",
            b"[SLAVE] Error: Failed to power up device!", slaveMap, TRUE)

    #Config of all slaves together
    status += MMWL_firmwareDownload(slavesMap)
    check(status,
        b"[SLAVE] Firmware successfully uploaded!",
        b"[SLAVE] Error: Firmware upload failed!", slavesMap, TRUE)

    status += MMWL_setDeviceCrcType(slavesMap)
    check(status,
        b"[SLAVE] CRC type has been set!",
        b"[SLAVE] Error: Unable to set CRC type!", slavesMap, TRUE)

    status += MMWL_rfEnable(slavesMap)
    check(status,
        b"[SLAVE] RF successfully enabled!",
        b"[SLAVE] Error: Failed to enable master RF", slavesMap, TRUE)

    status += MMWL_channelConfig(slavesMap, channelCfg.cascading,channelCfg)
    check(status,
        b"[SLAVE] Channels successfully configured!",
        b"[SLAVE] Error: Channels configuration failed!", slavesMap, TRUE)

    status += MMWL_adcOutConfig(slavesMap, adcOutCfg)
    check(status,
        b"[SLAVE] ADC output format successfully configured!",
        b"[SLAVE] Error: ADC output format configuration failed!", slavesMap, TRUE)

    check(status,
        b"[SLAVE] Init completed with sucess",
        b"[SLAVE] Init completed with error", slavesMap, TRUE)
    return status

cdef uint32_t configure (devConfig_t config):
    cdef int status = 0
    cdef int devId = 0
    status += initMaster(config.channelCfg, config.adcOutCfg)
    status += initSlaves(config.channelCfg, config.adcOutCfg)

    status += MMWL_RFDeviceConfig(config.deviceMap)
    check(status,
        b"[ALL] RF deivce configured!",
        b"[ALL] RF device configuration failed!", config.deviceMap, TRUE)

    status += MMWL_ldoBypassConfig(config.deviceMap, config.ldoCfg)
    check(status,
        b"[ALL] LDO Bypass configuration successful!",
        b"[ALL] LDO Bypass configuration failed!", config.deviceMap, TRUE)

    status += MMWL_dataFmtConfig(config.deviceMap, config.dataFmtCfg)
    check(status,
        b"[ALL] Data format configuration successful!",
        b"[ALL] Data format configuration failed!", config.deviceMap, TRUE)

    status += MMWL_lowPowerConfig(config.deviceMap, config.lpmCfg)
    check(status,
        b"[ALL] Low Power Mode configuration successful!",
        b"[ALL] Low Power Mode configuration failed!", config.deviceMap, TRUE)

    status += MMWL_ApllSynthBwConfig(config.deviceMap)
    status += MMWL_setMiscConfig(config.deviceMap, config.miscCfg)
    status += MMWL_rfInit(config.deviceMap)
    check(status,
        b"[ALL] RF successfully initialized!",
        b"[ALL] RF init failed!", config.deviceMap, TRUE)

    status += MMWL_dataPathConfig(config.deviceMap, config.datapathCfg)
    status += MMWL_hsiClockConfig(config.deviceMap, config.datapathClkCfg, config.hsClkCfg)
    status += MMWL_CSI2LaneConfig(config.deviceMap, config.csi2LaneCfg)
    check(status,
        b"[ALL] Datapath configuration successful!",
        b"[ALL] Datapath configuration failed!", config.deviceMap, TRUE)

    # PATCHED: 3 separate profile configs instead of 1, matching the 3
    # different idle times used per chirp (175us/7us/7us)
    status += MMWL_profileConfig(config.deviceMap, config.profileCfg[0])
    status += MMWL_profileConfig(config.deviceMap, config.profileCfg[1])
    status += MMWL_profileConfig(config.deviceMap, config.profileCfg[2])
    check(status,
        b"[ALL] Profile configuration successful!",
        b"[ALL] Profile configuration failed!", config.deviceMap, TRUE)

    # MIMO Chirp configuration
    for devId in range(4):
        status += configureMimoChirp(devId, config.chirpCfg)

    check(status,
        b"[ALL] Chirp configuration successful!",
        b"[ALL] Chirp configuration failed!", config.deviceMap, TRUE)

    #Master frame config.
    status += MMWL_frameConfig(
        config.masterMap,
        config.frameCfg,
        config.channelCfg,
        config.adcOutCfg,
        config.datapathCfg,
        config.profileCfg[0]   # PATCHED: array now; [0] fine since only
                                # numAdcSamples is used here, identical across profiles
    )
    check(status,
        b"[MASTER] Frame configuration completed!",
        b"[MASTER] Frame configuration failed!", config.masterMap, TRUE)

    #Slaves frame config
    status += MMWL_frameConfig(
        config.slavesMap,
        config.frameCfg,
        config.channelCfg,
        config.adcOutCfg,  
        config.datapathCfg,
        config.profileCfg[0]   # PATCHED: array now; [0] fine, see note above
    )
    check(status,
        b"[SLAVE] Frame configuration completed!",
        b"[SLAVE] Frame configuration failed!", config.slavesMap, TRUE)

    return status

cdef devConfig_t config


def _require(section, key, section_name):
    """Fetch section[key] or raise a clear ValueError naming the missing TOML key."""
    if key not in section:
        raise ValueError(
            f"Radar config missing required field '{section_name}.{key}'. "
            f"Add it to radar_configs/*.toml (see radar_configs/default.toml)."
        )
    return section[key]


def _require_section(mimo, name):
    """Fetch mimo[name] or raise naming the missing [mimo.<name>] TOML table."""
    if name not in mimo:
        raise ValueError(
            f"Radar config missing required section [mimo.{name}]. "
            f"Add it to radar_configs/*.toml (see radar_configs/default.toml)."
        )
    return mimo[name]


cpdef mmw_set_config(dict configdict):
    """Apply a complete radar_configs/*.toml dict to the C structs the SDK uses.

    Every field this function programs is REQUIRED. Missing keys raise
    ValueError - there are no silent hardcoded fallbacks. Call this before
    mmw_init().
    """
    global config, NUM_CHIRPS, _tx_antenna_table, _profile_id_per_chirp, _config_applied
    cdef int pIdx
    cdef dict mimo, profile, frame, channel, chirp, adc_out, low_power, misc, ldo, datapath

    if "mimo" not in configdict:
        raise ValueError(
            "Radar config missing top-level 'mimo' table. "
            "Pass a dict loaded from radar_configs/*.toml."
        )
    mimo = configdict["mimo"]

    # Start from zeroed module-level structs (not RF defaults).
    config.deviceMap = 1|(1<<1)|(1<<2)|(1<<3)
    MMWL_AssignDeviceMap(config.deviceMap, &config.masterMap, &config.slavesMap)
    config.frameCfg = frameCfgArgs
    config.profileCfg[0] = profileCfgArgs0
    config.profileCfg[1] = profileCfgArgs1
    config.profileCfg[2] = profileCfgArgs2
    config.chirpCfg = chirpCfgArgs
    config.channelCfg = channelCfgArgs
    config.csi2LaneCfg = csi2LaneCfgArgs
    config.datapathCfg = datapathCfgArgs
    config.datapathClkCfg = datapathClkCfgArgs
    config.hsClkCfg = hsClkCfgArgs
    config.ldoCfg = ldoCfgArgs
    config.lpmCfg = lpmCfgArgs
    config.miscCfg = miscCfgArgs
    config.adcOutCfg = adcOutCfgArgs
    config.dataFmtCfg = dataFmtCfgArgs

    # ── profile (required; applied to all 3 profile slots) ──────────────
    profile = _require_section(mimo, "profile")
    idle_times = _require(profile, "idleTimes", "mimo.profile")
    if len(idle_times) != 3:
        raise ValueError(
            f"mimo.profile.idleTimes must have exactly 3 entries "
            f"(one per profileCfg slot), got {len(idle_times)}"
        )
    for pIdx in range(3):
        config.profileCfg[pIdx].profileId = <uint16_t>pIdx
        config.profileCfg[pIdx].startFreqConst = <uint32_t>(ceil(
            _require(profile, "startFrequency", "mimo.profile") * 1e9 / 53.644))
        config.profileCfg[pIdx].freqSlopeConst = <int16_t>(ceil(
            _require(profile, "frequencySlope", "mimo.profile") * 1e3 / 48.279))
        config.profileCfg[pIdx].adcStartTimeConst = <uint32_t>(ceil(
            _require(profile, "adcStartTime", "mimo.profile") * 1e2))
        config.profileCfg[pIdx].rampEndTime = <uint32_t>(ceil(
            _require(profile, "rampEndTime", "mimo.profile") * 1e2))
        config.profileCfg[pIdx].txStartTime = <int16_t>(ceil(
            _require(profile, "txStartTime", "mimo.profile") * 1e2))
        config.profileCfg[pIdx].numAdcSamples = <uint16_t>(
            _require(profile, "numAdcSamples", "mimo.profile"))
        config.profileCfg[pIdx].digOutSampleRate = <uint16_t>(
            _require(profile, "adcSamplingFrequency", "mimo.profile"))
        config.profileCfg[pIdx].rxGain = <uint16_t>(
            _require(profile, "rxGain", "mimo.profile"))
        config.profileCfg[pIdx].hpfCornerFreq1 = <uint8_t>(
            _require(profile, "hpfCornerFreq1", "mimo.profile"))
        config.profileCfg[pIdx].hpfCornerFreq2 = <uint8_t>(
            _require(profile, "hpfCornerFreq2", "mimo.profile"))
        config.profileCfg[pIdx].pfVcoSelect = <uint8_t>(
            _require(profile, "pfVcoSelect", "mimo.profile"))
        config.profileCfg[pIdx].pfCalLutUpdate = <uint8_t>(
            _require(profile, "pfCalLutUpdate", "mimo.profile"))
        config.profileCfg[pIdx].txOutPowerBackoffCode = <uint32_t>(
            _require(profile, "txOutPowerBackoffCode", "mimo.profile"))
        config.profileCfg[pIdx].txPhaseShifter = <uint32_t>(
            _require(profile, "txPhaseShifter", "mimo.profile"))
        # 1 LSB = 10ns -> value = us * 100
        config.profileCfg[pIdx].idleTimeConst = <uint32_t>(ceil(idle_times[pIdx] * 1e2))

    # ── frame ───────────────────────────────────────────────────────────
    frame = _require_section(mimo, "frame")
    config.frameCfg.numFrames = <uint16_t>(_require(frame, "numFrames", "mimo.frame"))
    config.frameCfg.numLoops = <uint16_t>(_require(frame, "numLoops", "mimo.frame"))
    config.frameCfg.framePeriodicity = <uint32_t>(ceil(
        _require(frame, "framePeriodicity", "mimo.frame") * 2e5))  # 1LSB = 5ns
    config.frameCfg.chirpStartIdx = <uint16_t>(_require(frame, "chirpStartIdx", "mimo.frame"))
    config.frameCfg.chirpEndIdx = <uint16_t>(_require(frame, "chirpEndIdx", "mimo.frame"))
    config.frameCfg.frameTriggerDelay = <uint32_t>(ceil(
        float(_require(frame, "frameTriggerDelay", "mimo.frame")) * 2e5))
    # triggerSelectMaster/Slave are consumed by .mmwave.json export only
    # (frameCfg.triggerSelect is set per-device inside MMWL_frameConfig).
    _require(frame, "triggerSelectMaster", "mimo.frame")
    _require(frame, "triggerSelectSlave", "mimo.frame")

    # ── channel ─────────────────────────────────────────────────────────
    channel = _require_section(mimo, "channel")
    config.channelCfg.rxChannelEn = <uint16_t>(_require(channel, "rxChannelEn", "mimo.channel"))
    config.channelCfg.txChannelEn = <uint16_t>(_require(channel, "txChannelEn", "mimo.channel"))

    # ── chirp / antenna geometry ────────────────────────────────────────
    chirp = _require_section(mimo, "chirp")
    NUM_CHIRPS = <int>(_require(chirp, "numChirps", "mimo.chirp"))
    if NUM_CHIRPS <= 0:
        raise ValueError("mimo.chirp.numChirps must be > 0")
    _profile_id_per_chirp = list(_require(chirp, "profileIdPerChirp", "mimo.chirp"))
    _tx_antenna_table = [list(row) for row in _require(chirp, "txAntennaTable", "mimo.chirp")]
    if len(_profile_id_per_chirp) != NUM_CHIRPS:
        raise ValueError(
            f"mimo.chirp.profileIdPerChirp length ({len(_profile_id_per_chirp)}) "
            f"must equal numChirps ({NUM_CHIRPS})"
        )
    if len(_tx_antenna_table) < 4:
        raise ValueError(
            f"mimo.chirp.txAntennaTable must have one row per device "
            f"(need >= 4), got {len(_tx_antenna_table)}"
        )
    for row in _tx_antenna_table[:4]:
        if len(row) != NUM_CHIRPS:
            raise ValueError(
                f"each mimo.chirp.txAntennaTable row must have numChirps "
                f"({NUM_CHIRPS}) entries, got {len(row)}"
            )
    # Fine-dither vars: only 0.0 is supported today (encoding for non-zero
    # is not wired). Keys are still required so a missing TOML field fails.
    for _var_key in ("startFreqVar_MHz", "freqSlopeVar_KHz_usec",
                     "idleTimeVar_usec", "adcStartTimeVar_usec"):
        _var_val = _require(chirp, _var_key, "mimo.chirp")
        if float(_var_val) != 0.0:
            raise ValueError(
                f"mimo.chirp.{_var_key}={_var_val}: non-zero chirp fine-dither "
                f"vars are not yet supported (must be 0.0)"
            )
    config.chirpCfg.startFreqVar = 0
    config.chirpCfg.freqSlopeVar = 0
    config.chirpCfg.idleTimeVar = 0
    config.chirpCfg.adcStartTimeVar = 0

    # ── adcOut ──────────────────────────────────────────────────────────
    adc_out = _require_section(mimo, "adcOut")
    config.adcOutCfg.fmt.b2AdcBits = <uint32_t>(_require(adc_out, "adcBits", "mimo.adcOut"))
    config.adcOutCfg.fmt.b8FullScaleReducFctr = <uint32_t>(
        _require(adc_out, "fullScaleReducFctr", "mimo.adcOut"))
    config.adcOutCfg.fmt.b2AdcOutFmt = <uint32_t>(_require(adc_out, "adcOutFmt", "mimo.adcOut"))

    # ── lowPower / misc / ldo ───────────────────────────────────────────
    low_power = _require_section(mimo, "lowPower")
    config.lpmCfg.lpAdcMode = <uint16_t>(_require(low_power, "lpAdcMode", "mimo.lowPower"))

    misc = _require_section(mimo, "misc")
    config.miscCfg.miscCtl = <uint32_t>(_require(misc, "miscCtl", "mimo.misc"))

    ldo = _require_section(mimo, "ldo")
    config.ldoCfg.ldoBypassEnable = <uint16_t>(_require(ldo, "ldoBypassEnable", "mimo.ldo"))
    config.ldoCfg.supplyMonIrDrop = <uint8_t>(_require(ldo, "supplyMonIrDrop", "mimo.ldo"))
    config.ldoCfg.ioSupplyIndicator = <uint8_t>(_require(ldo, "ioSupplyIndicator", "mimo.ldo"))

    # ── datapath / CSI2 / HSI clock ─────────────────────────────────────
    datapath = _require_section(mimo, "datapath")
    config.dataFmtCfg.iqSwapSel = <uint8_t>(_require(datapath, "iqSwapSel", "mimo.datapath"))
    config.dataFmtCfg.chInterleave = <uint8_t>(_require(datapath, "chInterleave", "mimo.datapath"))
    config.datapathCfg.intfSel = <uint8_t>(_require(datapath, "intfSel", "mimo.datapath"))
    config.datapathCfg.transferFmtPkt0 = <uint8_t>(
        _require(datapath, "transferFmtPkt0", "mimo.datapath"))
    config.datapathCfg.transferFmtPkt1 = <uint8_t>(
        _require(datapath, "transferFmtPkt1", "mimo.datapath"))
    config.datapathCfg.cqConfig = <uint8_t>(_require(datapath, "cqConfig", "mimo.datapath"))
    config.datapathCfg.cq0TransSize = <uint8_t>(
        _require(datapath, "cq0TransSize", "mimo.datapath"))
    config.datapathCfg.cq1TransSize = <uint8_t>(
        _require(datapath, "cq1TransSize", "mimo.datapath"))
    # cq2TransSize: required in TOML for documentation / .mmwave.json parity,
    # but rlDevDataPathCfg_t has no matching field - not applied to the chip.
    _require(datapath, "cq2TransSize", "mimo.datapath")
    config.csi2LaneCfg.lineStartEndDis = <uint8_t>(
        _require(datapath, "lineStartEndDis", "mimo.datapath"))
    config.csi2LaneCfg.lanePosPolSel = <uint32_t>(
        _require(datapath, "lanePosPolSel", "mimo.datapath"))
    config.datapathClkCfg.laneClkCfg = <uint8_t>(
        _require(datapath, "laneClkCfg", "mimo.datapath"))
    rate_code, hsi_clk_code = _lookup_data_rate(
        int(_require(datapath, "dataRate_Mbps", "mimo.datapath")),
        int(config.datapathClkCfg.laneClkCfg),
    )
    config.datapathClkCfg.dataRate = <uint8_t>(rate_code)
    config.hsClkCfg.hsiClk = <uint16_t>(hsi_clk_code)

    # NOTE: mimo.rfInit.calibEnMask is intentionally NOT applied here -
    # MMWL_rfInit() takes no calibEnMask parameter in this build. Optional
    # in TOML; written to .mmwave.json for documentation only when present.

    config.frameCfg.numAdcSamples = 2 * config.profileCfg[0].numAdcSamples
    config.dataFmtCfg.rxChannelEn = config.channelCfg.rxChannelEn
    config.dataFmtCfg.adcBits = config.adcOutCfg.fmt.b2AdcBits
    config.dataFmtCfg.adcFmt = config.adcOutCfg.fmt.b2AdcOutFmt

    _config_applied = True
    return 0

cpdef int mmw_init(
    str ip_addr="192.168.33.180",
    int port = 5001,
    ):
    cdef int status = 0
    cdef bytes ip_addr_bytes = ip_addr.encode('utf-8')
    if not _config_applied:
        raise RuntimeError(
            "mmw_set_config() must be called with a complete radar_configs/*.toml "
            "dict before mmw_init() - C-struct defaults are zeroed and will not "
            "program a working radar."
        )
    status = MMWL_TDAInit(ip_addr_bytes,port,config.deviceMap)
    check(status,
        b"[MMWCAS-DSP] TDA Connected!",
        b"[MMWCAS-DSP] Couldn't connect to TDA board!", 32, TRUE)

    configure(config) 
    return status

cpdef int mmw_reconfigure_frame_count(int num_frames):
    """@brief Re-run ONLY the frame configuration RPC (MMWL_frameConfig) for
    * master + slaves with an updated numFrames, leaving profile/chirp/RF-init/
    * datapath untouched. This makes the RF chips themselves hard-stop after
    * exactly num_frames frames, instead of relying on host-side wall-clock
    * timing (time.sleep + mmw_stop_frame()) to cut the capture off - which is
    * subject to RPC/network jitter and does not guarantee an exact frame count.
    *
    * Used by --interactive mode (mimo.py) so each prompt's frame count is
    * applied to the actual chip config, not just the TDA arm/host wait,
    * without paying the cost of a full mmw_init()/configure() (firmware
    * re-download, RF re-init, etc.) between captures.
    *
    * @num_frames Exact number of frames the chips should record (0 = infinite,
    *             same semantics as frame.numFrames in radar_configs/*.toml).
    * @return int Aggregate status (0 on full success)
    """
    cdef int status = 0
    if not _config_applied:
        raise RuntimeError(
            "mmw_set_config() must be called before mmw_reconfigure_frame_count()."
        )
    config.frameCfg.numFrames = <uint16_t>num_frames

    status += MMWL_frameConfig(
        config.masterMap,
        config.frameCfg,
        config.channelCfg,
        config.adcOutCfg,
        config.datapathCfg,
        config.profileCfg[0]
    )
    check(status,
        b"[MASTER] Frame count reconfigured!",
        b"[MASTER] Frame count reconfiguration failed!", config.masterMap, TRUE)

    status += MMWL_frameConfig(
        config.slavesMap,
        config.frameCfg,
        config.channelCfg,
        config.adcOutCfg,
        config.datapathCfg,
        config.profileCfg[0]
    )
    check(status,
        b"[SLAVE] Frame count reconfigured!",
        b"[SLAVE] Frame count reconfiguration failed!", config.slavesMap, TRUE)

    return status

cpdef int mmw_arming_tda(str capture_path, int num_frames=-1):
    """@brief Prepare the TDA board and notify TDA about the start of recording
    * @capture_path capture path setup to arm the TDA for recording
    * @num_frames frames for TDA to record (-1 = use config.frameCfg.numFrames;
    *             0 = unlimited until stop). Used by interactive mode so each
    *             capture can request a different length without reconfiguring RF.
    * @return int
    """
    cdef int status = 0
    cdef char capture_path_buf[256]
    cdef bytes capture_path_bytes = f"/mnt/ssd/{capture_path}".encode('utf-8')
    if len(capture_path_bytes) >= sizeof(capture_path_buf):
        printf(b"[MMWCAS] ERROR: capture_path too long!\n")
        return -1
    strncpy(capture_path_buf, capture_path_bytes, sizeof(capture_path_buf) - 1)
    capture_path_buf[sizeof(capture_path_buf) - 1] = b'\0'

    cdef unsigned int frame_period_ms = (config.frameCfg.framePeriodicity * 5) // (1000 * 1000)

    cdef rlTdaArmCfg_t tdaCfg
    tdaCfg.captureDirectory = <unsigned char*>capture_path_buf
    tdaCfg.framePeriodicity = frame_period_ms
    tdaCfg.numberOfFilesToAllocate = 0
    if num_frames < 0:
        tdaCfg.numberOfFramesToCapture = config.frameCfg.numFrames
    else:
        tdaCfg.numberOfFramesToCapture = <unsigned int>num_frames
    tdaCfg.dataPacking = 0              # 0: 16-bit | 1: 12-bit

    status = MMWL_ArmingTDA(tdaCfg)
    check(status,
        b"[MMWCAS-DSP] Arming TDA",
        b"[MMWCAS-DSP] TDA Arming failed!", 32, 0)
    return status

cpdef int mmw_start_frame():
    """
    Start framing: slaves first (arms their wait-for-hardware-sync state),
    THEN master last, with a settle delay in between - see the hardware-sync
    note in Cascade_Configuration_Capture_Test10s.lua. The master's software
    trigger is what actually fires the RF sweep + sync pulse the slaves are
    waiting for; without the settle delay the master can fire before the
    slaves finish arming, and EVERY RPC call still reports status 0 (arm,
    start, stop, de-arm all "succeed") while /mnt/ssd/<capture_dir> ends up
    with 0-byte .bin files. A previous version of this function looped over
    all 4 devices back-to-back with no delay between slaves and master,
    which silently produced empty captures - fixed to match the working
    LUA sequence.
    """
    cdef int status = 0
    cdef int i
    # Slaves first (bits 8, 4, 2 = slave3, slave2, slave1): arm hw-sync wait state
    for i in range(3, 0, -1):
        status += MMWL_StartFrame(1 << i)

    time.sleep(0.1)  # let slaves finish arming before master fires the sync pulse

    # Master last (bit 1): fires RF sweep + sync pulse
    status += MMWL_StartFrame(1)

    check(status,
        b"[MMWCAS-RF] Framing ...",
        b"[MMWCAS-RF] Failed to initiate framing!", 
        config.deviceMap, 
        0)
    return status

cdef int RL_RET_CODE_FRAME_ALREADY_ENDED = 21  # see ti/mmwavelink/mmwavelink.h

cpdef int mmw_stop_frame():
    """
    Stop framing on all 4 devices (3, 2, 1, 0 order).

    Since mmw_reconfigure_frame_count() makes frameCfg.numFrames finite for
    every capture, each RF chip auto-stops itself once it has emitted its
    last frame - it does not wait around for an explicit STOP. By the time
    the host's wait_s margin has elapsed and this runs, a device that
    finished on its own legitimately answers RL_RET_CODE_FRAME_ALREADY_ENDED
    (21) to STOP, not a real error (mmw_stop_frame() is then effectively a
    no-op / safety net rather than what actually ends the capture). Treat
    that specific per-device code as success; still surface any other
    non-zero status as a genuine failure.
    """
    cdef int status = 0
    cdef int i
    cdef int dev_status
    for i in range(3, -1, -1):
        dev_status = MMWL_StopFrame(1 << i)
        if dev_status != RL_RET_CODE_FRAME_ALREADY_ENDED:
            status += dev_status

    check(status,
        b"[MMWCAS-RF] Stopped Frame ...",
        b"[MMWCAS-RF] Failed to stop frame!", 
        config.deviceMap, 
        0)
    return status

cpdef int mmw_dearming_tda():
    cdef int status = 0
    status = MMWL_DeArmingTDA()
    check(status,
        b"[MMWCAS-RF] Stop recording",
        b"[MMWCAS-RF] Failed to de-arm TDA board!", 32, 0)
    return status
