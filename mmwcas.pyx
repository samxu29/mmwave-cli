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
cdef int NUM_CHIRPS = 3    # PATCHED: was 12 (full 4-device MIMO). Matches mimo.c -
                           # 3 chirps, TX0/TX1/TX2 on a single device only.

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
* Profile config API parameters. A profile contains coarse parameters of FMCW chirp such as
* start frequency, chirp slope, ramp time, idle time etc. Fine dithering values need
* to be programmed in chirp configuration \ref rlChirpCfg_t
* \note Maximum of 4 profiles can be configured.
*
* PATCHED: 3 separate profiles instead of 1, matching mimo.c exactly
* (Cascade_Configuration_Capture_Ready2ArmTrigger.lua geometry):
*   - startFreq=77GHz, slope=60MHz/us, adcStart=6us, rampEnd=65us,
*     256 samples @ 8000ksps, rxGain=48dB  -- SAME across all 3
*   - idleTime DIFFERS per chirp: profile0=175us, profile1=7us, profile2=7us
*
* Encoding (same scale factors as the original default, verified against it):
*   startFreqConst: 1 LSB = 53.6441803 Hz  -> 77GHz unchanged (1435384036)
*   freqSlopeConst: 1 LSB = 48.2797623 kHz/us -> 60MHz/us = round(60000/48.2797623) = 1243
*   idleTimeConst / adcStartTimeConst / rampEndTime: 1 LSB = 10ns -> value = us * 100
*/
"""
cdef rlProfileCfg_t profileCfgArgs0=rlProfileCfg_t(
    profileId = 0,
    pfVcoSelect = 0x02,
    pfCalLutUpdate = 0,
    startFreqConst = 1435384036,   # 77GHz
    freqSlopeConst = 1243,         # 60 MHz/us
    idleTimeConst = 17500,         # 175us
    adcStartTimeConst = 600,       # 6us
    rampEndTime = 6500,            # 65us
    txOutPowerBackoffCode = 0x0,
    txPhaseShifter = 0x0,
    txStartTime = 0x0,             # 0us
    numAdcSamples = 256,
    digOutSampleRate = 8000,       # 8000 ksps
    hpfCornerFreq1 = 0x0,
    hpfCornerFreq2 = 0x0,
    rxGain = 48,
)

cdef rlProfileCfg_t profileCfgArgs1=rlProfileCfg_t(
    profileId = 1,
    pfVcoSelect = 0x02,
    pfCalLutUpdate = 0,
    startFreqConst = 1435384036,   # 77GHz
    freqSlopeConst = 1243,         # 60 MHz/us
    idleTimeConst = 700,           # 7us
    adcStartTimeConst = 600,       # 6us
    rampEndTime = 6500,            # 65us
    txOutPowerBackoffCode = 0x0,
    txPhaseShifter = 0x0,
    txStartTime = 0x0,
    numAdcSamples = 256,
    digOutSampleRate = 8000,
    hpfCornerFreq1 = 0x0,
    hpfCornerFreq2 = 0x0,
    rxGain = 48,
)

cdef rlProfileCfg_t profileCfgArgs2=rlProfileCfg_t(
    profileId = 2,
    pfVcoSelect = 0x02,
    pfCalLutUpdate = 0,
    startFreqConst = 1435384036,   # 77GHz
    freqSlopeConst = 1243,         # 60 MHz/us
    idleTimeConst = 700,           # 7us
    adcStartTimeConst = 600,       # 6us
    rampEndTime = 6500,            # 65us
    txOutPowerBackoffCode = 0x0,
    txPhaseShifter = 0x0,
    txStartTime = 0x0,
    numAdcSamples = 256,
    digOutSampleRate = 8000,
    hpfCornerFreq1 = 0x0,
    hpfCornerFreq2 = 0x0,
    rxGain = 48,
)

"""! \brief
* Frame config API parameters - PATCHED to match mimo.c's geometry (3 chirps,
* 255 loops, 100ms periodicity), matching Cascade_Configuration_Capture_Ready2ArmTrigger.lua
"""
cdef rlFrameCfg_t frameCfgArgs=rlFrameCfg_t(
    chirpStartIdx = 0,
    chirpEndIdx = 2,                # PATCHED: was 11 (12-chirp scheme), now 3 chirps
    numFrames = 0,                  # (0 for infinite)
    numLoops = 255,                  # PATCHED: was 16, now 255 (matches nchirp_loops)
    numAdcSamples = 2 * 256,        # Complex samples (for I and Q siganls)
    frameTriggerDelay = 0x0,
    framePeriodicity = 20000000,    # 100ms | 1LSB = 5ns
)

"""! \brief
* Chirp config API parameters. This structure contains fine dithering to coarse profile
* defined in \ref rlProfileCfg_t. It also includes the selection of Transmitter and
* binary phase modulation for a chirp.\n
* @note : One can define upto 512 unique chirps.These chirps need to be included in
*         frame configuration structure \ref rlFrameCfg_t to create FMCW frame
"""
cdef rlChirpCfg_t chirpCfgArgs = rlChirpCfg_t(
    chirpStartIdx = 0,
    chirpEndIdx = 0,
    profileId = 0,
    txEnable = 0x00,
    adcStartTimeVar = 0,
    idleTimeVar = 0,
    startFreqVar = 0,
    freqSlopeVar = 0,
)

"""! \brief
* Rx/Tx Channel Configuration
"""
cdef rlChanCfg_t channelCfgArgs = rlChanCfg_t(
    rxChannelEn = 0x0F,      # Enable all 4 RX Channels
    txChannelEn = 0x07,      # Enable all 3 TX Channels
    cascading = 0x02,        # Slave
)

# Manual initialization to ensure correct values
cdef rlAdcOutCfg_t adcOutCfgArgs
adcOutCfgArgs.fmt.b2AdcBits = 2           # 16-bit
adcOutCfgArgs.fmt.b6Reserved0 = 0
adcOutCfgArgs.fmt.b8FullScaleReducFctr = 0
adcOutCfgArgs.fmt.b2AdcOutFmt = 1         # Complex
adcOutCfgArgs.fmt.b14Reserved1 = 0
adcOutCfgArgs.reserved0 = 0
adcOutCfgArgs.reserved1 = 0

"""! \brief
* mmwave radar data format config
"""
cdef rlDevDataFmtCfg_t dataFmtCfgArgs = rlDevDataFmtCfg_t(
    iqSwapSel = 0,           # I first
    chInterleave = 0,        # Interleaved mode
    rxChannelEn = 0xF,       # All RX antenna enabled
    adcFmt = 1,              # Complex
    adcBits = 2,             # 16-bit ADC
)

"""! \brief
* Radar RF LDO bypass enable/disable configuration
"""
cdef rlRfLdoBypassCfg_t ldoCfgArgs = rlRfLdoBypassCfg_t(
    ldoBypassEnable = 3,       # RF LDO disabled, PA LDO disabled
    ioSupplyIndicator = 0,
    supplyMonIrDrop = 0,
)

"""! \brief
* Power saving mode configuration
"""
cdef rlLowPowerModeCfg_t lpmCfgArgs = rlLowPowerModeCfg_t(
    lpAdcMode = 0,             # Regular ADC power mode
)

"""! \brief
* Radar RF Miscconfiguration
"""
cdef rlRfMiscConf_t miscCfgArgs = rlRfMiscConf_t(
    miscCtl = 1,               # Enable Per chirp phase shifter
)

"""! \brief
* mmwave radar data path config.
"""
cdef rlDevDataPathCfg_t datapathCfgArgs = rlDevDataPathCfg_t(
    intfSel = 0,               # CSI2 intrface
    transferFmtPkt0 = 1,       # ADC data only
    transferFmtPkt1 = 0,       # Suppress packet 1
)

"""! \brief
* DataPath clock configuration
"""
cdef rlDevDataPathClkCfg_t datapathClkCfgArgs = rlDevDataPathClkCfg_t(
    laneClkCfg = 1,            # DDR Clock
    dataRate = 1,              # 600Mbps
)

"""! \brief
* mmwave radar high speed clock configuration
"""
cdef rlDevHsiClk_t hsClkCfgArgs = rlDevHsiClk_t(
    hsiClk = 0x09,             # DDR 600Mbps
)

"""! \brief
* CSI2 configuration
"""
cdef rlDevCsi2Cfg_t csi2LaneCfgArgs = rlDevCsi2Cfg_t(
    lineStartEndDis = 0,       # Enable
    lanePosPolSel = 0x35421,   # 0b 0011 0101 0100 0010 0001,
)

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


cdef int8_t is_in_table(uint8_t value, uint8_t[:] table, uint8_t size):
    '''@brief Check if a value is in the table provided in argument
    #* @param value Value to look for in the table
    #* @param table Table defining the search context
    #* @param size Size of the table
    #* @return int8_t
    #* Return the index where the match has been found. -1 if not found
    '''
    cdef uint8_t i
    for i in range(size):
        if table[i] == value:
            return i
    return -1


cpdef uint32_t configureMimoChirp(uint8_t devId, rlChirpCfg_t chirpCfg):
    """@brief MIMO Chirp configuration
    #* @param devId Device ID (0: master, 1: slave1, 2: slave2, 3: slave3)
    #* @param chirpCfg Initital chirp configuration
    #* @return uint32_t Configuration status

    PATCHED TX table: matches mimo.c exactly - ONLY Dev4 (slave3, devId 3)
    transmits, one TX antenna per chirp (TX0 on chirp0, TX1 on chirp1, TX2
    on chirp2). Dev1/Dev2/Dev3 are 100% RX-only on every chirp (rows are all
    0xFF, an invalid sentinel that never matches a real chirp index 0-2).

    Original (unpatched) table implemented TI's full 12-chirp, 4-device MIMO
    scheme where every device transmits on 3 of 12 chirps.
    """
    cdef uint8_t[4][3] chripTxTable=[
        [0xFF, 0xFF, 0xFF],  # Dev1 - Master: RX only, never transmits
        [0xFF, 0xFF, 0xFF],  # Dev2 - Slave1: RX only, never transmits
        [0xFF, 0xFF, 0xFF],  # Dev3 - Slave2: RX only, never transmits
        [0, 1, 2],           # Dev4 - Slave3: TX0/TX1/TX2 on chirp0/1/2
    ]

    cdef int status = 0
    cdef uint8_t i
    cdef int8_t txIdx

    for i in range(NUM_CHIRPS):
        txIdx = is_in_table(i, chripTxTable[devId], 3)

        # Update chirp configuration
        chirpCfg.chirpStartIdx = i
        chirpCfg.chirpEndIdx = i
        # PATCHED: select the profile matching this chirp index (chirp0->profile0
        # with 175us idle, chirp1/2->profile1/2 with 7us idle) - same profile
        # used across ALL devices for a given chirp index, matching mimo.c.
        chirpCfg.profileId = i
        if txIdx < 0:
            chirpCfg.txEnable = 0x00
        else:
            chirpCfg.txEnable = (1 << txIdx)

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

cpdef mmw_set_config(dict configdict):
    global config
    config.deviceMap = 1|(1<<1)|(1<<2)|(1<<3)
    MMWL_AssignDeviceMap(config.deviceMap, &config.masterMap, &config.slavesMap)
    config.frameCfg = frameCfgArgs
    config.profileCfg[0] = profileCfgArgs0  # PATCHED: 3 profiles instead of 1
    config.profileCfg[1] = profileCfgArgs1
    config.profileCfg[2] = profileCfgArgs2
    config.chirpCfg = chirpCfgArgs
    config.channelCfg = channelCfgArgs
    config.csi2LaneCfg = csi2LaneCfgArgs
    config.datapathCfg = datapathCfgArgs
    config.datapathClkCfg=datapathClkCfgArgs
    config.hsClkCfg = hsClkCfgArgs
    config.ldoCfg = ldoCfgArgs
    config.lpmCfg = lpmCfgArgs
    config.miscCfg = miscCfgArgs
    config.adcOutCfg = adcOutCfgArgs

    cdef dict mimo,profile,frame,channel
    if "mimo" in configdict:
        mimo = configdict["mimo"]
        if "profile" in mimo: # [PROFILE CONFIGURATION]
            # NOTE: idle time is intentionally NOT configurable here - it's
            # fixed per-profile (175us/7us/7us) to match mimo.c's PATCHED
            # 3-profile geometry. All other fields below are applied
            # identically to all 3 profile slots (SAME across all 3, like
            # mimo.c's profileCfgArgs0/1/2).
            profile = mimo["profile"]
            for pIdx in range(3):
                if "startFrequency" in profile: # Chirp start frequency in GHz
                    config.profileCfg[pIdx].startFreqConst = <uint32_t>(ceil(profile["startFrequency"]*1e9/53.644)) # 1LSB = 53.644 Hz
                if "frequencySlope" in profile: # Frequency slope in MHz/us
                    config.profileCfg[pIdx].freqSlopeConst = <int16_t>(ceil(profile["frequencySlope"]*1e3/48.279)) # 1LSB = 48.279 kHz/us
                if "adcStartTime" in profile:# ADC start time in us
                    config.profileCfg[pIdx].adcStartTimeConst = <uint32_t>(ceil(profile["adcStartTime"]*1e2)) # 1LSB = 10ns
                if "rampEndTime" in profile:# Chirp ramp end time in us
                    config.profileCfg[pIdx].rampEndTime = <uint32_t>(ceil(profile["rampEndTime"]*1e2)) # 1LSB = 10ns
                if "txStartTime" in profile:# TX starttime in us
                    config.profileCfg[pIdx].txStartTime = <uint16_t>(ceil(profile["txStartTime"]*1e2)) # 1LSB = 10ns
                if "numAdcSamples" in profile:# Number of ADC samples per chirp
                    config.profileCfg[pIdx].numAdcSamples = <uint16_t>(profile["numAdcSamples"])
                if "adcSamplingFrequency" in profile:# ADC sampling frequency in ksps
                    config.profileCfg[pIdx].digOutSampleRate = <uint16_t>(profile["adcSamplingFrequency"])
                if "rxGain" in profile:# rxGain in dB
                    config.profileCfg[pIdx].rxGain = <uint16_t>(profile["rxGain"])
                if "hpfCornerFreq1" in profile: # hpfCornerFreq1
                    config.profileCfg[pIdx].hpfCornerFreq1 = <uint8_t>(profile["hpfCornerFreq1"])
                if "hpfCornerFreq2" in profile: # hpfCornerFreq2
                    config.profileCfg[pIdx].hpfCornerFreq2 = <uint8_t>(profile["hpfCornerFreq2"])
            
        if "frame" in mimo: # [FRAME CONFIGURATION]
            frame = mimo["frame"]
            if "numFrames" in frame: # Number of frames to record
                config.frameCfg.numFrames = <uint16_t>(frame["numFrames"])
            if "numLoops" in frame: # Number of chirp loop per frame
                config.frameCfg.numLoops = <uint16_t>(frame["numLoops"])
            if "framePeriodicity" in frame: # Frame periodicity in ms
                config.frameCfg.framePeriodicity = <uint32_t>(ceil(frame["framePeriodicity"]*2e5)) # 1LSB = 5ns
        if "channel" in mimo:# [CHANNEL CONFIGURATION]
            channel = mimo["channel"]
            if "rxChannelEn" in channel: # RX Channel configuration
                config.channelCfg.rxChannelEn = <uint16_t>(channel["rxChannelEn"])
            if "txChannelEn" in channel: # TX Channel configuration
                config.channelCfg.txChannelEn = <uint16_t>(channel["txChannelEn"])
        config.frameCfg.numAdcSamples = 2 * config.profileCfg[0].numAdcSamples
        config.dataFmtCfg.rxChannelEn = config.channelCfg.rxChannelEn
        
    config.dataFmtCfg.rxChannelEn = channelCfgArgs.rxChannelEn
    config.dataFmtCfg.adcBits = adcOutCfgArgs.fmt.b2AdcBits
    config.dataFmtCfg.adcFmt = adcOutCfgArgs.fmt.b2AdcOutFmt
    return 0

cpdef int mmw_init(
    str ip_addr="192.168.33.180",
    int port = 5001,
    ):
    cdef int status = 0
    cdef bytes ip_addr_bytes = ip_addr.encode('utf-8')
    status = MMWL_TDAInit(ip_addr_bytes,port,config.deviceMap)
    check(status,
        b"[MMWCAS-DSP] TDA Connected!",
        b"[MMWCAS-DSP] Couldn't connect to TDA board!", 32, TRUE)

    configure(config) 
    return status

cpdef int mmw_arming_tda(str capture_path):
    """@brief Prepare the TDA board and notify TDA about the start of recording
    * @capture_path capture path setup to arm the TDA for recording 
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
    tdaCfg.numberOfFramesToCapture = 0  # config.frameCfg.numFrames
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

cpdef int mmw_stop_frame():
    cdef int status = 0
    cdef int i
    # Stop devices sequentially (3, 2, 1, 0)
    for i in range(3, -1, -1):
        status += MMWL_StopFrame(1 << i)
    
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
