import subprocess
import time


def _ls_capture_dir(remote_path, tda_ip):
    """Single SSH `ls` attempt against remote_path. Returns (success, file_count, files)."""
    ssh_cmd = [
        "ssh",
        "-oHostKeyAlgorithms=+ssh-rsa",
        "-oPubkeyAcceptedAlgorithms=+ssh-rsa",
        "-oStrictHostKeyChecking=no",
        "-oConnectTimeout=10",
        f"root@{tda_ip}",
        f"ls -lh {remote_path} 2>/dev/null || echo 'DIRECTORY_NOT_FOUND'"
    ]

    result = subprocess.run(
        ssh_cmd,
        capture_output=True,
        text=True,
        timeout=15
    )

    if result.returncode != 0 or "DIRECTORY_NOT_FOUND" in result.stdout:
        return False, 0, []

    output_lines = result.stdout.strip().split('\n')

    # Filter out directory entries (., ..) and get actual files
    files = []
    for line in output_lines:
        if line and not line.startswith('total') and not line.endswith(' .') and not line.endswith(' ..'):
            parts = line.split()
            if len(parts) >= 9:
                # Extract filename (last part) and size
                filename = parts[-1]
                size_str = parts[4]
                files.append((filename, size_str))

    return (len(files) > 0), len(files), files


def check_captured_files(capture_dir, tda_ip="192.168.33.180", retries=4, retry_delay=2.0):
    """
    Check if files were actually captured on the TDA board via SSH.

    mmw_dearming_tda() only confirms the TDA firmware accepted the stop
    command - it does NOT wait for buffered frames to actually flush to
    /mnt/ssd/. Immediately after a capture, the directory can transiently
    exist-but-be-empty (or not exist yet) while the flush is still in
    flight, especially for longer captures. So this retries with a short
    delay before declaring failure, instead of a single immediate `ls`.

    Args:
        capture_dir: The capture directory name
        tda_ip: IP address of the TDA board (default: 192.168.33.180)
        retries: Number of SSH `ls` attempts before giving up (default: 4)
        retry_delay: Seconds to wait between attempts (default: 2.0)

    Returns:
        tuple: (success: bool, file_count: int, file_list: list)
    """
    remote_path = f"/mnt/ssd/{capture_dir}"

    print(f"\n{'='*60}")
    print(f"Checking captured files on TDA board...")
    print(f"{'='*60}")
    print(f"Remote path: {remote_path}")

    try:
        for attempt in range(1, retries + 1):
            success, file_count, files = _ls_capture_dir(remote_path, tda_ip)

            if success:
                print(f"\n SUCCESS: Found {file_count} file(s) in capture directory"
                      + (f" (attempt {attempt}/{retries})" if attempt > 1 else ""))
                print(f"\nCaptured files:")
                print(f"{'-'*60}")
                for filename, size in files:
                    print(f"  {filename:50s} {size:>8s}")
                print(f"{'-'*60}")
                return True, file_count, files

            if attempt < retries:
                print(f"\n No files yet (attempt {attempt}/{retries}) - TDA may still be "
                      f"flushing to disk, retrying in {retry_delay}s...")
                time.sleep(retry_delay)

        print(f"\n WARNING: Directory not found or empty after {retries} attempts!")
        print(f"   Path: {remote_path}")
        print(f"   This usually means data capture failed.")
        return False, 0, []

    except subprocess.TimeoutExpired:
        print(f"\n ERROR: SSH connection timeout!")
        print(f"   Could not connect to TDA board at {tda_ip}")
        return False, 0, []
    except Exception as e:
        print(f"\n ERROR: Failed to check files via SSH: {e}")
        return False, 0, []


def upload_files_to_tda(local_paths, capture_dir, tda_ip="192.168.33.180"):
    """
    SCP local metadata files (IR timestamps .npy, .mmwave.json config) up
    into /mnt/ssd/<capture_dir>/ on the TDA board, so they sit alongside the
    raw .bin capture data and travel together through the existing
    SCP-download-then-delete step in pipeline.py, instead of needing a
    separate correlation step downstream.

    local_paths: iterable of local file paths (None/missing entries are
    skipped, e.g. when IR logging was disabled).

    Returns True if the upload succeeded, False otherwise (never raises -
    a failed upload here shouldn't fail the whole capture).
    """
    existing_paths = [p for p in local_paths if p and os.path.exists(p)]
    if not existing_paths:
        return False

    remote_path = f"/mnt/ssd/{capture_dir}/"
    scp_cmd = [
        "scp", "-O",
        "-oHostKeyAlgorithms=+ssh-rsa",
        "-oPubkeyAcceptedAlgorithms=+ssh-rsa",
        "-oStrictHostKeyChecking=no",
        "-oConnectTimeout=10",
        *existing_paths,
        f"root@{tda_ip}:{remote_path}",
    ]
    try:
        result = subprocess.run(scp_cmd, capture_output=True, text=True, timeout=30)
        if result.returncode != 0:
            print(f"\n WARNING: Failed to upload metadata files to TDA: {result.stderr.strip()}")
            return False
        print(f"\n Uploaded {len(existing_paths)} metadata file(s) to {tda_ip}:{remote_path}"
              f" ({', '.join(os.path.basename(p) for p in existing_paths)})")
        return True
    except subprocess.TimeoutExpired:
        print(f"\n WARNING: SCP upload to TDA timed out.")
        return False
    except Exception as e:
        print(f"\n WARNING: Failed to upload metadata files to TDA: {e}")
        return False


import json
import os
import sys
from datetime import datetime


def _as_hex(value):
    """Format an int (or already-hex str) as Studio-style '0x..' string."""
    if isinstance(value, str):
        return value if value.lower().startswith("0x") else f"0x{int(value, 0):X}"
    return f"0x{int(value):X}"


def export_config_to_json(config_dict, filename, num_devices=4):
    """
    Create mmwave.json from config_dict (Studio-compatible structure).

    All RF / chirp / frame / datapath fields are taken from config_dict so the
    sidecar matches what was (or will be) programmed - no parallel hardcodes.
    """
    mimo = config_dict["mimo"]
    profile = mimo["profile"]
    frame = mimo["frame"]
    channel = mimo["channel"]
    chirp = mimo["chirp"]
    rf_init = mimo["rfInit"]
    adc_out = mimo["adcOut"]
    low_power = mimo["lowPower"]
    misc = mimo["misc"]
    ldo = mimo["ldo"]
    datapath = mimo["datapath"]

    idle_times = profile["idleTimes"]
    num_profiles = len(idle_times)
    num_chirps = chirp["numChirps"]
    profile_id_per_chirp = chirp["profileIdPerChirp"]
    tx_antenna_table = chirp["txAntennaTable"]

    if len(profile_id_per_chirp) != num_chirps:
        raise ValueError("chirp.profileIdPerChirp length must equal chirp.numChirps")
    if len(tx_antenna_table) < num_devices:
        raise ValueError("chirp.txAntennaTable must have one row per device")
    for row in tx_antenna_table[:num_devices]:
        if len(row) != num_chirps:
            raise ValueError("each chirp.txAntennaTable row must have numChirps entries")

    json_output = {
        "configGenerator": {
            "createdBy": "mmWavel CLI Python Script",
            "createdOn": datetime.now().astimezone().isoformat(),
            "isConfigIntermediate": 1
        },
        "currentVersion": {
            "jsonCfgVersion": {"major": 0, "minor": 4, "patch": 0},
            "DFPVersion": {"major": 2, "minor": 2, "patch": 0},
            "SDKVersion": {"major": 3, "minor": 3, "patch": 0},
            "mmwavelinkVersion": {"major": 2, "minor": 2, "patch": 0}
        },
        "lastBackwardCompatibleVersion": {
            "DFPVersion": {"major": 2, "minor": 1, "patch": 0},
            "SDKVersion": {"major": 3, "minor": 0, "patch": 0},
            "mmwavelinkVersion": {"major": 2, "minor": 1, "patch": 0}
        },
        "regulatoryRestrictions": {
            "frequencyRangeBegin_GHz": 77,
            "frequencyRangeEnd_GHz": 81,
            "maxBandwidthAllowed_MHz": 4000,
            "maxTransmitPowerAllowed_dBm": 12
        },
        "systemConfig": {
            "summary": "This is a comments field not passed to device",
            "sceneParameters": {
                "ambientTemperature_degC": 20,
                "maxDetectableRange_m": 10,
                "rangeResolution_cm": 5,
                "maxVelocity_kmph": 26,
                "velocityResolution_kmph": 2,
                "measurementRate": 10,
                "typicalDetectedObjectRCS": 1.0
            }
        },
        "mmWaveDevices": []
    }

    for devId in range(num_devices):
        chirps = []
        for chirpIdx in range(num_chirps):
            tx_ant = tx_antenna_table[devId][chirpIdx]
            tx_enable = (1 << int(tx_ant)) if tx_ant is not None and int(tx_ant) >= 0 else 0
            chirps.append({
                "rlChirpCfg_t": {
                    "chirpStartIdx": chirpIdx,
                    "chirpEndIdx": chirpIdx,
                    "profileId": profile_id_per_chirp[chirpIdx],
                    "startFreqVar_MHz": float(chirp["startFreqVar_MHz"]),
                    "freqSlopeVar_KHz_usec": float(chirp["freqSlopeVar_KHz_usec"]),
                    "idleTimeVar_usec": float(chirp["idleTimeVar_usec"]),
                    "adcStartTimeVar_usec": float(chirp["adcStartTimeVar_usec"]),
                    "txEnable": _as_hex(tx_enable)
                }
            })

        trigger_select = (
            frame["triggerSelectMaster"] if devId == 0 else frame["triggerSelectSlave"]
        )

        device_config = {
            "mmWaveDeviceId": devId,
            "rfConfig": {
                "waveformType": "legacyFrameChirp",
                "MIMOScheme": "TDM",
                "rlCalibrationDataFile": "",
                "rlChanCfg_t": {
                    "rxChannelEn": _as_hex(channel["rxChannelEn"]),
                    "txChannelEn": _as_hex(channel["txChannelEn"]),
                    "cascading": 1 if devId == 0 else 2,
                    "cascadingPinoutCfg": "0x0"
                },
                "rlAdcOutCfg_t": {
                    "fmt": {
                        "b2AdcBits": adc_out["adcBits"],
                        "b8FullScaleReducFctr": adc_out["fullScaleReducFctr"],
                        "b2AdcOutFmt": adc_out["adcOutFmt"]
                    }
                },
                "rlLowPowerModeCfg_t": {
                    "lpAdcMode": low_power["lpAdcMode"]
                },
                "rlProfiles": [{
                    "rlProfileCfg_t": {
                        "profileId": pIdx,
                        "pfVcoSelect": _as_hex(profile["pfVcoSelect"]),
                        "pfCalLutUpdate": _as_hex(profile["pfCalLutUpdate"]),
                        "startFreqConst_GHz": profile["startFrequency"],
                        "idleTimeConst_usec": idle_times[pIdx],
                        "adcStartTimeConst_usec": profile["adcStartTime"],
                        "rampEndTime_usec": profile["rampEndTime"],
                        "txOutPowerBackoffCode": _as_hex(profile["txOutPowerBackoffCode"]),
                        "txPhaseShifter": _as_hex(profile["txPhaseShifter"]),
                        "freqSlopeConst_MHz_usec": profile["frequencySlope"],
                        "txStartTime_usec": profile["txStartTime"],
                        "numAdcSamples": profile["numAdcSamples"],
                        "digOutSampleRate": float(profile["adcSamplingFrequency"]),
                        "hpfCornerFreq1": profile["hpfCornerFreq1"],
                        "hpfCornerFreq2": profile["hpfCornerFreq2"],
                        "rxGain_dB": _as_hex(profile["rxGain"])
                    }
                } for pIdx in range(num_profiles)],
                "rlChirps": chirps,
                "rlRfInitCalConf_t": {
                    "calibEnMask": _as_hex(rf_init["calibEnMask"])
                },
                "rlFrameCfg_t": {
                    "chirpEndIdx": frame["chirpEndIdx"],
                    "chirpStartIdx": frame["chirpStartIdx"],
                    "numLoops": frame["numLoops"],
                    "numFrames": frame["numFrames"],
                    "framePeriodicity_msec": float(frame["framePeriodicity"]),
                    "triggerSelect": trigger_select,
                    "frameTriggerDelay": float(frame["frameTriggerDelay"])
                },
                "rlBpmChirps": [],
                "rlRfMiscConf_t": {
                    "miscCtl": str(misc["miscCtl"])
                },
                "rlRfPhaseShiftCfgs": [],
                "rlRfProgFiltConfs": [],
                "rlRfLdoBypassCfg_t": {
                    "ldoBypassEnable": ldo["ldoBypassEnable"],
                    "supplyMonIrDrop": ldo["supplyMonIrDrop"],
                    "ioSupplyIndicator": ldo["ioSupplyIndicator"]
                },
                "rlLoopbackBursts": [],
                "rlDynChirpCfgs": [],
                "rlDynPerChirpPhShftCfgs": []
            },
            "rawDataCaptureConfig": {
                "rlDevDataFmtCfg_t": {
                    "iqSwapSel": datapath["iqSwapSel"],
                    "chInterleave": datapath["chInterleave"]
                },
                "rlDevDataPathCfg_t": {
                    "intfSel": datapath["intfSel"],
                    "transferFmtPkt0": _as_hex(datapath["transferFmtPkt0"]),
                    "transferFmtPkt1": _as_hex(datapath["transferFmtPkt1"]),
                    "cqConfig": datapath["cqConfig"],
                    "cq0TransSize": datapath["cq0TransSize"],
                    "cq1TransSize": datapath["cq1TransSize"],
                    "cq2TransSize": datapath["cq2TransSize"]
                },
                "rlDevDataPathClkCfg_t": {
                    "laneClkCfg": datapath["laneClkCfg"],
                    "dataRate_Mbps": datapath["dataRate_Mbps"]
                },
                "rlDevCsi2Cfg_t": {
                    "lanePosPolSel": _as_hex(datapath["lanePosPolSel"]),
                    "lineStartEndDis": datapath["lineStartEndDis"]
                }
            },
            "monitoringConfig": {}
        }
        json_output["mmWaveDevices"].append(device_config)

    json_output["processingChainConfig"] = {
        "detectionChain": {
            "name": "TI_GenericChain",
            "detectionLoss": 1,
            "systemLoss": 1,
            "implementationMargin": 2,
            "detectionSNR": 12,
            "theoreticalRxAntennaGain": 9,
            "theoreticalTxAntennaGain": 9
        }
    }

    try:
        with open(filename, 'w') as f:
            json.dump(json_output, f, indent=2)
        print(f"  > Successfully saved configuration to {filename}")
    except IOError as e:
        print(f"ERROR: Failed to save JSON file: {e}", file=sys.stderr)

def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    global shutdown_flag
    print("\n\n  Interrupt received (Ctrl+C). Finishing current capture and shutting down...")
    shutdown_flag = True
