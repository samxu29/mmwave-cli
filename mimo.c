/**
 * @file mimo.c
 * @author AMOUSSOU Z. Kenneth (www.gitlab.com/azinke)
 * @brief MMWave Radar configuration and control tool
 *
 * @note: Only MIMO setup is supported for now
 *
 * The MMWCAS-RF-EVM revision E has AWR2243 radar chips
 *
 * Approximate default configuration (generated uing mmWave Sensing Estimator):
 *
 *  Max Detectable Range  : ~80m
 *  Range resolution      : ~31cm
 *  May Velocity          : ~6.49 km/h
 *  Velocity resolution   : ~0.4 km/h
 *
 * @version 0.1
 * @date 2022-07-21
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "mimo.h"
#include "toml/config.h"

/******************************
 *      CONFIGURATIONS
 ******************************/

/**
 * PATCHED: 3 separate profiles instead of 1, matching
 * Cascade_Configuration_Capture_Ready2ArmTrigger.lua exactly:
 *   - startFreq=77GHz, slope=60MHz/us, adcStart=6us, rampEnd=65us,
 *     256 samples @ 4400ksps, rxGain=48dB  -- SAME across all 3
 *     (2026-07-22: digOutSampleRate corrected 8000->4400 to match the
 *     AWR1843 reference config's SAMPLE_RATE)
 *   - idleTime DIFFERS per chirp: profile0=175us, profile1=7us, profile2=7us
 *
 * Encoding (same scale factors as the original default, verified against it):
 *   startFreqConst: 1 LSB = 53.6441803 Hz  -> 77GHz unchanged from default (1435384036)
 *   freqSlopeConst: 1 LSB = 48.2797623 kHz/us -> 60MHz/us = round(60000/48.2797623) = 1243
 *   idleTimeConst / adcStartTimeConst / rampEndTime: 1 LSB = 10ns -> value = us * 100
 */
const rlProfileCfg_t profileCfgArgs0 = {
  .profileId = 0,
  .pfVcoSelect = 0x02,
  .startFreqConst = 1435384036,   // 77GHz
  .freqSlopeConst = 1243,         // 60 MHz/us
  .idleTimeConst = 17500,         // 175us
  .adcStartTimeConst = 600,       // 6us
  .rampEndTime = 6500,            // 65us
  .txOutPowerBackoffCode = 0x0,
  .txPhaseShifter = 0x0,
  .txStartTime = 0x0,
  .numAdcSamples = 256,
  .digOutSampleRate = 4400,
  .hpfCornerFreq1 = 0x0,
  .hpfCornerFreq2 = 0x0,
  .rxGain = 48,
};

const rlProfileCfg_t profileCfgArgs1 = {
  .profileId = 1,
  .pfVcoSelect = 0x02,
  .startFreqConst = 1435384036,   // 77GHz
  .freqSlopeConst = 1243,         // 60 MHz/us
  .idleTimeConst = 700,           // 7us
  .adcStartTimeConst = 600,       // 6us
  .rampEndTime = 6500,            // 65us
  .txOutPowerBackoffCode = 0x0,
  .txPhaseShifter = 0x0,
  .txStartTime = 0x0,
  .numAdcSamples = 256,
  .digOutSampleRate = 4400,
  .hpfCornerFreq1 = 0x0,
  .hpfCornerFreq2 = 0x0,
  .rxGain = 48,
};

const rlProfileCfg_t profileCfgArgs2 = {
  .profileId = 2,
  .pfVcoSelect = 0x02,
  .startFreqConst = 1435384036,   // 77GHz
  .freqSlopeConst = 1243,         // 60 MHz/us
  .idleTimeConst = 700,           // 7us
  .adcStartTimeConst = 600,       // 6us
  .rampEndTime = 6500,            // 65us
  .txOutPowerBackoffCode = 0x0,
  .txPhaseShifter = 0x0,
  .txStartTime = 0x0,
  .numAdcSamples = 256,
  .digOutSampleRate = 4400,
  .hpfCornerFreq1 = 0x0,
  .hpfCornerFreq2 = 0x0,
  .rxGain = 48,
};

/** Frame config - PATCHED to match the Lua script's geometry (3 chirps, 255
 * loops, 100ms periodicity), but numFrames=0 (infinite) so --time alone
 * controls capture duration - no recompile needed to change duration. */
const rlFrameCfg_t frameCfgArgs = {
  .chirpStartIdx = 0,
  .chirpEndIdx = 2,                // PATCHED: was 11 (12-chirp scheme), now 3 chirps
  .numFrames = 0,                  // PATCHED: infinite framing - stops only when
                                    // the explicit StopFrame arrives (governed by
                                    // --time), never auto-completes on its own
  .numLoops = 255,                 // PATCHED: was 16, now 255 (matches nchirp_loops)
  .numAdcSamples = 2 * 256,        // Complex samples (for I and Q signals)
  .frameTriggerDelay = 0x0,
  .framePeriodicity = 20000000,    // 100ms | 1LSB = 5ns (unchanged, already matched)
};

/** Chirps config */
rlChirpCfg_t chirpCfgArgs = {
  .chirpStartIdx = 0,
  .chirpEndIdx = 0,
  .profileId = 0,
  .txEnable = 0x00,
  .adcStartTimeVar = 0,
  .idleTimeVar = 0,
  .startFreqVar = 0,
  .freqSlopeVar = 0,
};

/** Channel config */
rlChanCfg_t channelCfgArgs = {
  .rxChannelEn = 0x0F,      // Enable all 4 RX Channels
  .txChannelEn = 0x07,      // Enable all 3 TX Channels
  .cascading = 0x02,        // Slave
};

/** ADC output config */
rlAdcOutCfg_t adcOutCfgArgs = {
  .fmt = {
    .b2AdcBits = 2,           // 16-bit ADC
    .b2AdcOutFmt = 1,         // Complex values
    .b8FullScaleReducFctr = 0,
  }
};

/** Data format config */
rlDevDataFmtCfg_t dataFmtCfgArgs = {
  .iqSwapSel = 0,           // I first
  .chInterleave = 0,        // Interleaved mode
  .rxChannelEn = 0xF,       // All RX antenna enabled
  .adcFmt = 1,              // Complex
  .adcBits = 2,             // 16-bit ADC
};

/** LDO Bypass config */
rlRfLdoBypassCfg_t ldoCfgArgs = {
  .ldoBypassEnable = 3,       // RF LDO disabled, PA LDO disabled
  .ioSupplyIndicator = 0,
  .supplyMonIrDrop = 0,
};

/** Low Power Mode config */
rlLowPowerModeCfg_t lpmCfgArgs = {
  .lpAdcMode = 0,             // Regular ADC power mode
};

/** Miscellaneous config */
rlRfMiscConf_t miscCfgArgs = {
  .miscCtl = 1,               // Enable Per chirp phase shifter
};

/** Datapath config */
rlDevDataPathCfg_t datapathCfgArgs = {
  .intfSel = 0,               // CSI2 intrface
  .transferFmtPkt0 = 1,       // ADC data only
  .transferFmtPkt1 = 0,       // Suppress packet 1
};

/** Datapath clock config */
rlDevDataPathClkCfg_t datapathClkCfgArgs = {
  .laneClkCfg = 1,            // DDR Clock
  .dataRate = 1,              // 600Mbps
};

/** High speed clock config */
rlDevHsiClk_t hsClkCfgArgs = {
  .hsiClk = 0x09,             // DDR 600Mbps
};

/** CSI2 config */
rlDevCsi2Cfg_t csi2LaneCfgArgs = {
  .lineStartEndDis = 0,       // Enable
  .lanePosPolSel = 0x35421,   // 0b 0011 0101 0100 0010 0001,
};



/*
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
*/


/**
 * @brief Check if a value is in the table provided in argument
 *
 * @param value Value to look for in the table
 * @param table Table defining the search context
 * @param size Size of the table
 * @return int8_t
 *      Return the index where the match has been found. -1 if not found
 */
int8_t is_in_table(uint8_t value, uint8_t *table, uint8_t size) {
  for (uint8_t i = 0; i < size; i++) {
    if (table[i] == value) return i;
  }
  return -1;
}


/**
 * @brief MIMO Chirp configuration
 *
 * @param devId Device ID (0: master, 1: slave1, 2: slave2, 3: slave3)
 * @param chirpCfg Initital chirp configuration
 * @return uint32_t Configuration status
 */
uint32_t configureMimoChirp(uint8_t devId, rlChirpCfg_t chirpCfg) {
  /**
   * PATCHED TX table: ONLY Dev4 (slave3, devId 3) transmits, one TX antenna
   * per chirp (TX0 on chirp0, TX1 on chirp1, TX2 on chirp2), chosen to match
   * the physical antenna placement of this deployment. Dev1/Dev2/Dev3
   * (master, slave1, slave2) are 100% RX-only on every chirp - their rows
   * are all 0xFF (invalid sentinel, never matches a real chirp index 0-2),
   * so is_in_table always returns -1 for them and txEnable stays 0x00 on
   * every chirp.
   *
   * NOTE: "master" here only refers to the mmWaveLink sync/LO-distribution
   * role (rlChanCfg_t.cascading, set in initMaster()/initSlaves()) - it has
   * no bearing on which chip's chirpCfg.txEnable is set, so any one of the
   * 4 devices (including a slave) can be the sole transmitter, as here.
   *
   * Previously (see git history) it was Dev1 (master) that transmitted -
   * flip which row below holds {0,1,2} to move the active TX antenna to a
   * different physical board.
   *
   * Original (unpatched) table implemented TI's full 12-chirp, 4-device MIMO
   * scheme where every device transmits on 3 of 12 chirps - a fundamentally
   * different antenna configuration than this setup uses.
   */
  const uint8_t chripTxTable [4][3] = {
    {0xFF, 0xFF, 0xFF}, // Dev1 - Master: RX only, never transmits
    {0xFF, 0xFF, 0xFF}, // Dev2 - Slave1: RX only, never transmits
    {0xFF, 0xFF, 0xFF}, // Dev3 - Slave2: RX only, never transmits
    {0, 1, 2},          // Dev4 - Slave3: TX0/TX1/TX2 on chirp0/1/2 respectively
  };
  int status = 0;

  for (uint8_t i = 0; i < NUM_CHIRPS; i++) {
    int8_t txIdx = is_in_table(i, chripTxTable[devId], 3);

    // Update chirp config
    chirpCfg.chirpStartIdx = i;
    chirpCfg.chirpEndIdx = i;
    // PATCHED: select the profile matching this chirp index (chirp0->profile0
    // with 175us idle, chirp1/2->profile1/2 with 7us idle) - same profile
    // used across ALL devices for a given chirp index, matching the Lua
    // script's ar1.ProfileConfig_mult(deviceMapOverall, ...) applying each
    // profile identically to master+slaves, with only TX enable differing.
    chirpCfg.profileId = i;
    if (txIdx < 0) chirpCfg.txEnable = 0x00;
    else chirpCfg.txEnable = (1 << txIdx);  // txIdx = position in the matched
                                              // row = the local TX antenna
                                              // index (0/1/2) to enable
    status += MMWL_chirpConfig(createDevMapFromDevId(devId), chirpCfg);
    DEBUG_PRINT("[CHIRP CONFIG] dev %u, chirp idx %u, profileId %u, txEnable 0x%02x, status: %d\n",
                devId, i, chirpCfg.profileId, chirpCfg.txEnable, status);
    if (status != 0) {
      DEBUG_PRINT("Configuration of chirp %d failed!\n", i);
      break;
    }
  }
  return status;
}

/**
 * @brief Check status and print error or success message
 *
 * @param status Status value returned by a function
 * @param success_msg Success message to print when status is 0
 * @param error_msg Error message to print in case of error
 * @param deviceMap Device map the check if related to
 * @param is_required Indicates if the checking stage is required. if so,
 *                    the program exits in case of failure.
 * @return uint32_t Configuration status
 *
 * @note: Status is considered successful when the status integer is 0.
 * Any other value is considered a failure.
 */
void check(int status, const char *success_msg, const char *error_msg,
      unsigned char deviceMap, uint8_t is_required) {
#if DEV_ENV
  printf("STATUS %4d | DEV MAP: %2u | ", status, deviceMap);
#endif
  if (status == RL_RET_CODE_OK) {
#if DEV_ENV
    printf(CGREEN);
    printf(success_msg);
    printf(CRESET);
    printf("\n");
#endif
    return;
  } else {
#if DEV_ENV
    printf(CRED);
    printf(error_msg);
    printf(CRESET);
    printf("\n");
#endif
    if (is_required != 0) exit(status);
  }
}


int32_t initMaster(rlChanCfg_t channelCfg, rlAdcOutCfg_t adcOutCfg) {
  const unsigned int masterId = 0;
  const unsigned int masterMap = 1 << masterId;
  int status = 0;

  // master chip
  channelCfg.cascading = 1;

  status += MMWL_DevicePowerUp(masterMap, 1000, 1000);
  check(status,
    "[MASTER] Power up successful!",
    "[MASTER] Error: Failed to power up device!", masterMap, TRUE);

  status += MMWL_firmwareDownload(masterMap);
  check(status,
    "[MASTER] Firmware successfully uploaded!",
    "[MASTER] Error: Firmware upload failed!", masterMap, TRUE);

  status += MMWL_setDeviceCrcType(masterMap);
  check(status,
    "[MASTER] CRC type has been set!",
    "[MASTER] Error: Unable to set CRC type!", masterMap, TRUE);

  status += MMWL_rfEnable(masterMap);
  check(status,
    "[MASTER] RF successfully enabled!",
    "[MASTER] Error: Failed to enable master RF", masterMap, TRUE);

  status += MMWL_channelConfig(masterMap, channelCfg.cascading, channelCfg);
  check(status,
    "[MASTER] Channels successfully configured!",
    "[MASTER] Error: Channels configuration failed!", masterMap, TRUE);

  status += MMWL_adcOutConfig(masterMap, adcOutCfg);
  check(status,
    "[MASTER] ADC output format successfully configured!",
    "[MASTER] Error: ADC output format configuration failed!", masterMap, TRUE);

  check(status,
    "[MASTER] Init completed with sucess\n",
    "[MASTER] Init completed with error", masterMap, TRUE);
  return status;
}


int32_t initSlaves(rlChanCfg_t channelCfg, rlAdcOutCfg_t adcOutCfg) {
  int status = 0;
  uint8_t slavesMap = (1 << 1) | (1 << 2) | (1 << 3);

  // slave chip
  channelCfg.cascading = 2;

  for (uint8_t slaveId = 1; slaveId < 4; slaveId++) {
    unsigned int slaveMap = 1 << slaveId;

    status += MMWL_DevicePowerUp(slaveMap, 1000, 1000);
    check(status,
      "[SLAVE] Power up successful!",
      "[SLAVE] Error: Failed to power up device!", slaveMap, TRUE);
  }

  //Config of all slaves together
  status += MMWL_firmwareDownload(slavesMap);
  check(status,
    "[SLAVE] Firmware successfully uploaded!",
    "[SLAVE] Error: Firmware upload failed!", slavesMap, TRUE);

  status += MMWL_setDeviceCrcType(slavesMap);
  check(status,
    "[SLAVE] CRC type has been set!",
    "[SLAVE] Error: Unable to set CRC type!", slavesMap, TRUE);

  status += MMWL_rfEnable(slavesMap);
  check(status,
    "[SLAVE] RF successfully enabled!",
    "[SLAVE] Error: Failed to enable master RF", slavesMap, TRUE);

  status += MMWL_channelConfig(slavesMap, channelCfg.cascading, channelCfg);
  check(status,
    "[SLAVE] Channels successfully configured!",
    "[SLAVE] Error: Channels configuration failed!", slavesMap, TRUE);

  status += MMWL_adcOutConfig(slavesMap, adcOutCfg);
  check(status,
    "[SLAVE] ADC output format successfully configured!",
    "[SLAVE] Error: ADC output format configuration failed!", slavesMap, TRUE);

  check(status,
    "[SLAVE] Init completed with sucess\n",
    "[SLAVE] Init completed with error", slavesMap, TRUE);
  return status;
}


uint32_t configure (devConfig_t config) {
  int status = 0;
  status += initMaster(config.channelCfg, config.adcOutCfg);
  status += initSlaves(config.channelCfg, config.adcOutCfg);

  status += MMWL_RFDeviceConfig(config.deviceMap);
  check(status,
    "[ALL] RF deivce configured!",
    "[ALL] RF device configuration failed!", config.deviceMap, TRUE);

  status += MMWL_ldoBypassConfig(config.deviceMap, config.ldoCfg);
  check(status,
    "[ALL] LDO Bypass configuration successful!",
    "[ALL] LDO Bypass configuration failed!", config.deviceMap, TRUE);

  status += MMWL_dataFmtConfig(config.deviceMap, config.dataFmtCfg);
  check(status,
    "[ALL] Data format configuration successful!",
    "[ALL] Data format configuration failed!", config.deviceMap, TRUE);

  status += MMWL_lowPowerConfig(config.deviceMap, config.lpmCfg);
  check(status,
    "[ALL] Low Power Mode configuration successful!",
    "[ALL] Low Power Mode configuration failed!", config.deviceMap, TRUE);

  status += MMWL_ApllSynthBwConfig(config.deviceMap);
  status += MMWL_setMiscConfig(config.deviceMap, config.miscCfg);
  status += MMWL_rfInit(config.deviceMap);
  check(status,
    "[ALL] RF successfully initialized!",
    "[ALL] RF init failed!", config.deviceMap, TRUE);

  status += MMWL_dataPathConfig(config.deviceMap, config.datapathCfg);
  status += MMWL_hsiClockConfig(config.deviceMap, config.datapathClkCfg, config.hsClkCfg);
  status += MMWL_CSI2LaneConfig(config.deviceMap, config.csi2LaneCfg);
  check(status,
    "[ALL] Datapath configuration successful!",
    "[ALL] Datapath configuration failed!", config.deviceMap, TRUE);

  // PATCHED: 3 separate profile configs instead of 1, matching the 3
  // different idle times used per chirp (175us/7us/7us)
  status += MMWL_profileConfig(config.deviceMap, config.profileCfg[0]);
  status += MMWL_profileConfig(config.deviceMap, config.profileCfg[1]);
  status += MMWL_profileConfig(config.deviceMap, config.profileCfg[2]);
  check(status,
    "[ALL] Profile configuration successful!",
    "[ALL] Profile configuration failed!", config.deviceMap, TRUE);

  // MIMO Chirp configuration
  for (uint8_t devId = 0; devId < 4; devId++) {
    status += configureMimoChirp(devId, config.chirpCfg);
  }
  check(status,
    "[ALL] Chirp configuration successful!",
    "[ALL] Chirp configuration failed!", config.deviceMap, TRUE);

  // Master frame config.
  status += MMWL_frameConfig(
    config.masterMap,
    config.frameCfg,
    config.channelCfg,
    config.adcOutCfg,
    config.datapathCfg,
    config.profileCfg[0]   // PATCHED: array now; [0] fine since only
                            // numAdcSamples is used here, identical across profiles
  );
  check(status,
    "[MASTER] Frame configuration completed!",
    "[MASTER] Frame configuration failed!", config.masterMap, TRUE);

  // Slaves frame config
  status += MMWL_frameConfig(
    config.slavesMap,
    config.frameCfg,
    config.channelCfg,
    config.adcOutCfg,
    config.datapathCfg,
    config.profileCfg[0]   // PATCHED: array now; [0] fine, see note above
  );
  check(status,
    "[SLAVE] Frame configuration completed!",
    "[SLAVE] Frame configuration failed!", config.slavesMap, TRUE);

  check(status,
    "[MIMO] Configuration completed!\n",
    "[MIMO] Configuration completed with error!", config.deviceMap, TRUE);
}


/**
 * @brief Routine to close trace file
 * 
 */
FILE* rls_traceF = NULL;
void CloseTraceFile() {
  if (rls_traceF != NULL) {
    fclose(rls_traceF);
    rls_traceF = NULL;
  }
}

// Pointer to the CLI option parser
parser_t *g_parser = NULL;

/**
 * Print program version
 */
void print_version() {
  printf(PROG_NAME " version " PROG_VERSION ", " PROG_COPYRIGHT "\n");
  exit(0);
}

/**
 * @brief Print CLI options help and exit
 */
void help() {
  print_help(g_parser);
  exit(0);
}

/**
 * @brief Free the parser to cleanup any dynamically allocated memory
 */
void cleanup() {
  free_parser(g_parser);
}

/**
 * @brief Called when the user presses CTRL+C
 *
 * This aim to explicitly call the exit function so that
 * dynamically allocated memory could be freed
 */
void signal_handler () {
  exit(1);
}


/**
 * @brief Arm the TDA, trigger a frame, wait, then stop and de-arm.
 *
 * Does NOT touch the RF chip configuration (profiles/chirps/frame geometry)
 * - only the per-capture arm/trigger/stop sequence. This is what makes it
 * safe to call repeatedly within a single process (see --interactive) to
 * record multiple experiments back-to-back without re-running the full
 * MMWL_TDAInit + configure() sequence each time.
 *
 * @param capture_dir_name Directory name (relative to /mnt/ssd/) where this
 *                          capture's raw data will be stored on the TDA
 * @param duration_ms How long to let the frame run, in milliseconds
 * @param is_required If TRUE, exit(status) on the first failed stage
 *                    (matches the original one-shot --record behavior).
 *                    If FALSE, failures are reported but the function
 *                    returns so the caller (e.g. the interactive loop)
 *                    can keep going.
 * @return int Aggregate status (0 on full success)
 */
int run_capture(unsigned char *capture_dir_name, float duration_ms, uint8_t is_required) {
  int status = 0;
  unsigned char path[128];
  strcpy(path, "/mnt/ssd/");
  strcat(path, capture_dir_name);

  rlTdaArmCfg_t tdaCfg = {
    .captureDirectory = path,
    .framePeriodicity = (frameCfgArgs.framePeriodicity * 5)/(1000*1000),
    .numberOfFilesToAllocate = 0,
    .numberOfFramesToCapture = 0, // TDA follows the frame config's own count
    .dataPacking = 0, // 0: 16-bit | 1: 12-bit
  };

  // Arm TDA
  status = MMWL_ArmingTDA(tdaCfg);
  check(status,
    "[MMWCAS-DSP] Arming TDA",
    "[MMWCAS-DSP] TDA Arming failed!\n", 32, is_required);

  msleep(2000);

  /**
   * Start framing: slaves first (arms their wait-for-hardware-sync state),
   * THEN master last, with a settle delay in between - see the hardware-sync
   * note in Cascade_Configuration_Capture_Test10s.lua. The master's software
   * trigger is what actually fires the RF sweep + sync pulse the slaves are
   * waiting for; without the settle delay the master can fire before the
   * slaves finish arming, and EVERY RPC call still reports STATUS 0 (arm,
   * start, stop, de-arm all "succeed") while /mnt/ssd/<capture_dir> ends up
   * with 0-byte .bin files - this bit us for real, see git history. A
   * previous version of this function looped over all 4 devices back-to-back
   * with no delay (the comment here claimed "slaves first, master last" but
   * the code never actually separated the two groups with a pause) - fixed
   * to match the working LUA sequence below.
   *
   * On repeated back-to-back captures within the same live connection
   * (--interactive), a device occasionally misses its Frame-Start ACK window
   * (RL_RET_CODE_RESP_TIMEOUT, -8) because the RF chips haven't fully settled
   * from the previous de-arm yet - this never showed up in the original
   * one-shot flow because a fresh process + full MMWL_TDAInit()/configure()
   * naturally gave the hardware 30-60s to settle. Retry once after a longer
   * cool-down before giving up, instead of failing the whole capture on a
   * timing fluke.
   */
  int start_status = 0;
  for (int i = 3; i >= 1; i--) {
    start_status += MMWL_StartFrame(1U << i);   // slaves: arm hw-sync wait state
  }
  msleep(100);                                   // let slaves finish arming
  start_status += MMWL_StartFrame(1U);           // master: fires RF sweep + sync pulse
  if (start_status != 0) {
    DEBUG_PRINT("WARNING: Framing failed to start cleanly (status %d) - "
                "stopping, cooling down, and retrying once\n\n", start_status);
    for (int i = 3; i >= 0; i--) {
      MMWL_StopFrame(1U << i);
    }
    msleep(3000);
    start_status = 0;
    for (int i = 3; i >= 1; i--) {
      start_status += MMWL_StartFrame(1U << i);
    }
    msleep(100);
    start_status += MMWL_StartFrame(1U);
  }
  status += start_status;
  check(status,
    "[MMWCAS-RF] Framing ...",
    "[MMWCAS-RF] Failed to initiate framing!\n", 15, is_required);

  msleep((unsigned long int)duration_ms);

  // Stop framing
  for (int i = 3; i >= 0; i--) {
    status += MMWL_StopFrame(1U << i);
  }

  status += MMWL_DeArmingTDA();
  check(status,
    "[MMWCAS-RF] Stop recording",
    "[MMWCAS-RF] Failed to de-arm TDA board!\n", 32, is_required);

  // Extra settle time before the caller may arm again (--interactive)
  msleep(3000);

  return status;
}


/**
 * @brief Application entry point
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main (int argc, char *argv[]) {
  DEBUG_PRINT("MMWave EVM configuration and control application\n");
  unsigned char default_ip_addr[] = "192.168.33.180";
  unsigned int default_port = 5001U;
  unsigned char default_capture_directory[64];
  sprintf(default_capture_directory, "%s_%lu", "MMWL_Capture", (unsigned long int)time(NULL));
  int status = 0;
  float default_recording_duration = 1.0;   // min

  /**
   * static: g_parser (set below) and every option_t added to it via add_arg()
   * are dereferenced by cleanup(), which only runs via atexit() AFTER main()
   * has already returned. Stack-local storage for these would make that a
   * use-after-return (UB) - this was segfaulting on exit after every capture,
   * right after a clean run_capture(), which meant the process died via
   * SIGSEGV instead of exiting normally - never giving the TDA's Ethernet
   * capture-manager connection a clean close, which is the most likely
   * explanation for the TDA getting stuck (STATUS 0 everywhere, 0-byte
   * .bin files) until a full board reboot.
   */
  static parser_t parser;
  parser = init_parser(
    PROG_NAME,
    "Configuration and control tool for TI MMWave cascade Evaluation Module"
  );
  g_parser = &parser;

  atexit(cleanup);  // Call the cleanup function before exiting the program
  signal(SIGINT, signal_handler);  // Catch CTRL+C to enable memory deallocation

  // NOTE: .default_value is assigned below, not in the initializer - it
  // points at a stack-local variable (address not known until runtime),
  // and static-storage initializers must be compile-time constants.
  static option_t opt_capturedir = {
    .args = "-d",
    .argl = "--capture-dir",
    .help = "Name of the director where to store recordings on the DSP board",
    .type = OPT_STR,
  };
  opt_capturedir.default_value = default_capture_directory;
  add_arg(&parser, &opt_capturedir);

  static option_t opt_port = {
    .args = "-p",
    .argl = "--port",
    .help = "Port number the DSP board server app is listening on",
    .type = OPT_INT,
  };
  opt_port.default_value = &default_port;
  add_arg(&parser, &opt_port);

  static option_t opt_ipaddr = {
    .args = "-i",
    .argl = "--ip-addr",
    .help = "IP Address of the MMWCAS DSP evaluation module",
    .type = OPT_STR,
  };
  opt_ipaddr.default_value = default_ip_addr;
  add_arg(&parser, &opt_ipaddr);

  static option_t opt_config = {
    .args = "-c",
    .argl = "--configure",
    .help = "Configure the MMWCAS-RF-EVM board",
    .type = OPT_BOOL,
  };
  add_arg(&parser, &opt_config);

  static option_t opt_record = {
    .args = "-r",
    .argl = "--record",
    .help = "Trigger data recording. This assumes that configuration is completed.",
    .type = OPT_BOOL,
  };
  add_arg(&parser, &opt_record);

  static option_t opt_interactive = {
    .args = "-I",
    .argl = "--interactive",
    .help = "Configure once (requires --configure), then repeatedly prompt for an "
            "experiment name and record - no reconfiguration between captures. "
            "At the prompt, type '<name>' or '<name> <seconds>' to capture, or "
            "'quit'/blank to exit.",
    .type = OPT_BOOL,
  };
  add_arg(&parser, &opt_interactive);

  static option_t opt_record_duration = {
    .args = "-t",
    .argl = "--time",
    .help = "Indicate how long the recording should last in minutes. Default: 1 min",
    .type = OPT_FLOAT,
  };
  opt_record_duration.default_value = &default_recording_duration;
  add_arg(&parser, &opt_record_duration);

  static option_t opt_config_file = {
    .args = "-f",
    .argl = "--cfg",
    .help = "TOML Configuration file. Overwrite the default config when provided",
    .type = OPT_STR,
    .default_value = NULL,
  };
  add_arg(&parser, &opt_config_file);

  static option_t opt_help = {
    .args = "-h",
    .argl = "--help",
    .help = "Print CLI option help and exit.",
    .type = OPT_BOOL,
    .default_value = NULL,
    .callback = help,
  };
  add_arg(&parser, &opt_help);

  static option_t opt_version = {
    .args = "-v",
    .argl = "--version",
    .help = "Print program version and exit.",
    .type = OPT_BOOL,
    .callback = print_version,
  };
  add_arg(&parser, &opt_version);

  parse(&parser, argc, argv);

  // Print help
  if ((unsigned char*)get_option(&parser, "help") != NULL) {
    print_help(&parser);
    exit(0);
  }

  unsigned char *ip_addr = (unsigned char*)get_option(&parser, "ip-addr");
  unsigned int port = *(unsigned int*)get_option(&parser, "port");
  unsigned char *capture_directory = (unsigned char*)get_option(&parser, "capture-dir");
  /* Record CLI option possible values are:
   *  - start: To start a recording and exit
   *  - stop: Stop a recording and exit
   *  - oneshot: Start a recording, wait for it's complemention and stop it.
   */
  unsigned char *record = (unsigned char*)get_option(&parser, "record");
  float record_duration = *(float*)get_option(&parser, "time");
  record_duration *= 60 * 1000;  // convert into milliseconds

  unsigned char *config_filename = (unsigned char*)get_option(&parser, "cfg");

  // Configuration
  devConfig_t config;

  /*  Device map:  master | slave 1  | slave 2  | slave 3 */
  config.deviceMap =  1   | (1 << 1) | (1 << 2) | (1 << 3);
  MMWL_AssignDeviceMap(config.deviceMap, &config.masterMap, &config.slavesMap);

  config.frameCfg = frameCfgArgs;
  config.profileCfg[0] = profileCfgArgs0;  // PATCHED: 3 profiles instead of 1
  config.profileCfg[1] = profileCfgArgs1;
  config.profileCfg[2] = profileCfgArgs2;
  config.chirpCfg = chirpCfgArgs;
  config.adcOutCfg = adcOutCfgArgs;
  config.dataFmtCfg = dataFmtCfgArgs;
  config.channelCfg = channelCfgArgs;
  config.csi2LaneCfg = csi2LaneCfgArgs;
  config.datapathCfg = datapathCfgArgs;
  config.datapathClkCfg = datapathClkCfgArgs;
  config.hsClkCfg = hsClkCfgArgs;
  config.ldoCfg = ldoCfgArgs;
  config.lpmCfg = lpmCfgArgs;
  config.miscCfg = miscCfgArgs;

  if (config_filename != NULL) {
    // Read parameters from config file
    read_config(config_filename, &config);
  }

  /**
   * @note: The adcOutCfg is used to overwrite the dataFmtCfg
   *
   * In a unified config file, it'll make for sense to have a single
   * source of truth for the ADC data format. And therefore use the
   * same data for setting both.
   */
  config.dataFmtCfg.rxChannelEn = channelCfgArgs.rxChannelEn;
  config.dataFmtCfg.adcBits = adcOutCfgArgs.fmt.b2AdcBits;
  config.dataFmtCfg.adcFmt = adcOutCfgArgs.fmt.b2AdcOutFmt;

  unsigned char *configured = (unsigned char *)get_option(&parser, "configure");

  if (configured != NULL) {
    // Connect to TDA
    status = MMWL_TDAInit(ip_addr, port, config.deviceMap);
    check(status,
      "[MMWCAS-DSP] TDA Connected!",
      "[MMWCAS-DSP] Couldn't connect to TDA board!\n", 32, TRUE);

    // Start configuration
    configure(config);
    msleep(2000);
  }

  if (record != NULL) {
    run_capture(capture_directory, record_duration, TRUE);
  }

  if ((unsigned char *)get_option(&parser, "interactive") != NULL) {
    if (configured == NULL) {
      DEBUG_PRINT(
        "ERROR: --interactive requires --configure in the same invocation "
        "(the TDA connection is only established during configuration)\n\n"
      );
      printf("Error: --interactive must be combined with --configure.\n");
      return 1;
    }

    printf("\n=== Interactive multi-capture mode ===\n");
    printf("Radar is configured and the connection to the TDA is live.\n");
    printf("At the prompt, type an experiment name to arm + record, e.g.:\n");
    printf("  bridge_test          (uses --time value: %.2f min)\n", record_duration / (60.0f * 1000.0f));
    printf("  bridge_test 30       (30 seconds instead)\n");
    printf("Type 'quit'/'exit' or leave blank to stop.\n\n");

    char line[128];
    while (1) {
      printf("experiment> ");
      fflush(stdout);
      if (fgets(line, sizeof(line), stdin) == NULL) break;

      size_t len = strlen(line);
      while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) {
        line[--len] = '\0';
      }
      if (len == 0) break;
      if (strcmp(line, "quit") == 0 || strcmp(line, "exit") == 0) break;

      char exp_name[96] = {0};
      float duration_s = -1.0f;
      int parsed = sscanf(line, "%95s %f", exp_name, &duration_s);
      float duration_ms = (parsed >= 2 && duration_s > 0)
        ? (duration_s * 1000.0f)
        : record_duration;

      unsigned char dirname[128];
      sprintf((char*)dirname, "%s_%lu", exp_name, (unsigned long int)time(NULL));

      printf("\n>>> Capturing '%s' for %.1fs ...\n", dirname, duration_ms / 1000.0f);
      status = run_capture(dirname, duration_ms, FALSE);
      if (status == 0) {
        printf(">>> Capture '%s' completed successfully.\n\n", dirname);
      } else {
        printf(">>> Capture '%s' finished with errors (status %d). Continuing...\n\n", dirname, status);
      }
    }

    printf("Exiting interactive mode.\n");
  }
  return 0;
}
