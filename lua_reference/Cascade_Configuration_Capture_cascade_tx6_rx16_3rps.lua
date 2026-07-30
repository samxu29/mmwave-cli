--[[
Cascade_Configuration_Capture_cascade_tx6_rx16_3rps.lua

mmWave Studio capture matching radar_configs/cascade_tx6_rx16_3rps.toml.

Built from Cascade_Configuration_Capture_cascade_tx3_rx16_.lua's 4-device
scaffolding (all of Dev1-4 enabled/connected) with the RF timing and chirp
geometry swapped to match the TOML exactly.

GEOMETRY (all 4 devices RX; 2 of them also TX - "tx6" from 6 chirps, not
6 TX devices):
  Dev1 (master)  : RX on, TX off
  Dev2 (slave1)  : RX on, TX0/TX1/TX2 on chirp0/1/2
  Dev3 (slave2)  : RX on, TX off
  Dev4 (slave3)  : RX on, TX0/TX1/TX2 on chirp3/4/5
  -> deviceMapOverall = 1+2+4+8 = 15; all 4 devices' master_*/slave1_*/
     slave2_*/slave3_* .bin files present. 16 physical RX (4 devices x 4 RX).

RF (doubled-period 3rps spinning schedule):
  3 profiles (reused across both TX devices via profileIdPerChirp),
  idle 7/7/7 us, ramp 40 us, slope 95 MHz/us, 160 samples @ 4850 ksps,
  rxGain 48, numLoops 255, period 76 ms (double cascade_tx3_rx8_3rps.lua's
  38 ms - see DATA RATE note below for why).

  6 chirps total: chirp0-2 = Dev2 TX0/1/2 (profile 0/1/2), chirp3-5 = Dev4
  TX0/1/2 (profile 0/1/2 again - same 3 profiles reused, not 6 separate
  ones, matching radar_configs/cascade_tx6_rx16_3rps.toml's
  profileIdPerChirp = [0,1,2,0,1,2]).

DATA RATE: 160 x 6 x 255 x 16 RX x 4 bytes / 0.076s ~= 206 MB/s. This config
is EXPECTED to show mid-capture frame drops under Studio and CLI alike -
mechanism (B) tracks the radar preset, not the host tool (see mimo.py's
FRAME DROPS docstring; prefer radar_configs/cascade_baseline.toml for a clean
reference). The point of this A/B is to compare Studio vs CLI drop PATTERN
at the same nominal config.

DEFAULT CAPTURE LENGTH: 300 frames (~22.8 s @ 76 ms). Edit
nframes_master/slave below if you want a shorter/longer run.

AFTER THE CAPTURE:
  python3 parse_idx.py --fetch <capture_directory> --frames 300 --period-ms 76

BEFORE RUNNING - edit if needed:
  1. metaImagePath  (F/W path on the Windows Studio PC)
  2. TDA_IPAddress  (default 192.168.33.180)
  3. nframes_master / nframes_slave
--]]

----------------------------------------User Constants--------------------------------------------

dev_list          =    {1, 2, 4, 8}       -- Device map bit values
RadarDevice       =    {1, 1, 1, 1}       -- {dev1, dev2, dev3, dev4} - all 4 enabled
cascade_mode_list =    {1, 2, 2, 2}       -- 0: Single chip, 1: Master, 2: Slave

-- F/W Download Path (edit for your Studio PC)
metaImagePath            =   "C:\\ti\\mmwave_dfp_02_02_02_01\\firmware\\xwr22xx_metaImage.bin"
-- metaImagePath            =   "C:\\ti\\mmwave_dfp_02_02_00_02\\firmware\\xwr22xx_metaImage.bin"

TDA_IPAddress     =   "192.168.33.180"

-- 1=master, 2=slave1, 4=slave2, 8=slave3
deviceMapOverall  =   RadarDevice[1] + (RadarDevice[2]*2) + (RadarDevice[3]*4) + (RadarDevice[4]*8)
deviceMapSlaves   =   (RadarDevice[2]*2) + (RadarDevice[3]*4) + (RadarDevice[4]*8)  -- = 14 (Dev2+Dev3+Dev4)
deviceMapMaster   =   RadarDevice[1]     -- = 1

-- Per-device maps used for the per-chirp TX gating below (matches
-- radar_configs/cascade_tx6_rx16_3rps.toml's txAntennaTable exactly)
devMap_Dev2       =   dev_list[2]        -- = 2  (slave1 - TX on chirp0-2)
devMap_Dev4       =   dev_list[4]        -- = 8  (slave3 - TX on chirp3-5)
devMap_RxOnlyAll  =   dev_list[1] + dev_list[3]  -- = 5  (Dev1 master + Dev3 slave2, TX off every chirp)

WriteToLog(string.format(
    "deviceMapOverall=%d (expect 15), deviceMapSlaves=%d (expect 14), deviceMapMaster=%d\n",
    deviceMapOverall, deviceMapSlaves, deviceMapMaster), "blue")

------------------------------------------- Sensor Configuration ------------------------------------------------

local start_freq                =   77       -- GHz
local slope                     =   95       -- MHz/us
local adc_start_time            =   6        -- us
local adc_samples               =   160
local sample_freq               =   4850     -- ksps
local ramp_end_time             =   40       -- us
local rx_gain                   =   48       -- dB

local idle_time_p0              =   7        -- us
local idle_time_p1              =   7        -- us
local idle_time_p2              =   7        -- us

local start_chirp_tx            =   0
local end_chirp_tx              =   5        -- 6 chirps (0-5)
local nchirp_loops              =   255
local nframes_master            =   300      -- ~22.8 s @ 76 ms; edit as needed
local nframes_slave             =   300
local Inter_Frame_Interval      =   76       -- ms - doubled from cascade_tx3_rx8_3rps.lua's 38ms
local trigger_delay             =   0        -- us

------------------------------ API Configuration ------------------------------------------------

WriteToLog("Setting up Studio for Cascade (cascade_tx6_rx16_3rps)...\n", "blue")

if(0 == ar1.ConnectTDA(TDA_IPAddress, 5001, deviceMapOverall)) then
    WriteToLog("ConnectTDA Successful\n", "green")
else
    WriteToLog("ConnectTDA Failed\n", "red")
    return -1
end

if(0 == ar1.selectCascadeMode(1)) then
    WriteToLog("selectCascadeMode Successful\n", "green")
else
    WriteToLog("selectCascadeMode Failed\n", "red")
    return -1
end

WriteToLog("Setting up Studio for Cascade ended..\n", "blue")

--Master Initialization

if (0 == ar1.SOPControl_mult(1, 4)) then
    WriteToLog("Master : SOP Reset Successful\n", "green")
else
    WriteToLog("Master : SOP Reset Failed\n", "red")
    return -1
end

if (0 == ar1.PowerOn_mult(1, 0, 1000, 0, 0)) then
    WriteToLog("Master : SPI Connection Successful\n", "green")
else
    WriteToLog("Master : SPI Connection Failed\n", "red")
    return -1
end

if (0 == ar1.DownloadBssFwOvSPI_mult(1, metaImagePath)) then
    WriteToLog("Master : FW Download Successful\n", "green")
else
    WriteToLog("Master : FW Download Failed\n", "red")
    return -1
end

if (0 == ar1.RfEnable_mult(1)) then
    WriteToLog("Master : RF Power Up Successful\n", "green")
else
    WriteToLog("Master : RF Power Up Failed\n", "red")
    return -1
end

-- Master: all 4 RX, all 3 TX channels enabled at channel level (TX gates are
-- per-chirp below), cascade mode = 1 (master)
if (0 == ar1.ChanNAdcConfig_mult(1,1,1,1,1,1,1,1,2,1,0,1)) then
    WriteToLog("Master : Channel & ADC Configuration Successful\n", "green")
else
    WriteToLog("Master : Channel & ADC Configuration Failed\n", "red")
    return -2
end

-- Slaves Initialization (Dev2, Dev3, Dev4 - all enabled, RadarDevice={1,1,1,1})

for i=2,table.getn(RadarDevice) do
    if ((RadarDevice[1]==1) and (RadarDevice[i]==1)) then

        if (0 == ar1.SOPControl_mult(dev_list[i], 4)) then
            WriteToLog("Device "..i.." : SOP Reset Successful\n", "green")
        else
            WriteToLog("Device "..i.." : SOP Reset Failed\n", "red")
            return -1
        end

        if (0 == ar1.AddDevice(dev_list[i])) then
            WriteToLog("Device "..i.." : SPI Connection Successful\n", "green")
        else
            WriteToLog("Device "..i.." : SPI Connection Failed\n", "red")
            return -1
        end

    end
end

if (0 == ar1.DownloadBssFwOvSPI_mult(deviceMapSlaves, metaImagePath)) then
    WriteToLog("Slaves : FW Download Successful\n", "green")
else
    WriteToLog("Slaves : FW Download Failed\n", "red")
    return -1
end

if (0 == ar1.RfEnable_mult(deviceMapSlaves)) then
    WriteToLog("Slaves : RF Power Up Successful\n", "green")
else
    WriteToLog("Slaves : RF Power Up Failed\n", "red")
    return -1
end

-- Slave(s): cascade mode = 2
if (0 == ar1.ChanNAdcConfig_mult(deviceMapSlaves,1,1,1,1,1,1,1,2,1,0,2)) then
    WriteToLog("Slaves : Channel & ADC Configuration Successful\n", "green")
else
    WriteToLog("Slaves : Channel & ADC Configuration Failed\n", "red")
    return -2
end

-- All active devices together

if (0 == ar1.RfLdoBypassConfig_mult(deviceMapOverall, 3)) then
    WriteToLog("LDO Bypass Successful\n", "green")
else
    WriteToLog("LDO Bypass failed\n", "red")
    return -2
end

if (0 == ar1.LPModConfig_mult(deviceMapOverall,0, 0)) then
    WriteToLog("Low Power Mode Configuration Successful\n", "green")
else
    WriteToLog("Low Power Mode Configuration failed\n", "red")
    return -2
end

if (0 == ar1.SetMiscConfig_mult(deviceMapOverall, 1, 0, 0, 0)) then
    WriteToLog("Misc Control Configuration Successful\n", "green")
else
    WriteToLog("Misc Control Configuration failed\n", "red")
    return -2
end

if (0 == ar1.RfInitCalibConfig_mult(deviceMapOverall, 1, 1, 1, 1, 1, 1, 1, 65537)) then
    WriteToLog("RF Init Calibration Successful\n", "green")
else
    WriteToLog("RF Init Calibration failed\n", "red")
    return -2
end

if (0 == ar1.RfInit_mult(deviceMapOverall)) then
    WriteToLog("RF Init Successful\n", "green")
else
    WriteToLog("RF Init failed\n", "red")
    return -2
end

---------------------------Data Configuration----------------------------------

if (0 == ar1.DataPathConfig_mult(deviceMapOverall, 0, 1, 0)) then
    WriteToLog("Data Path Configuration Successful\n", "green")
else
    WriteToLog("Data Path Configuration failed\n", "red")
    return -3
end

if (0 == ar1.LvdsClkConfig_mult(deviceMapOverall, 1, 1)) then
    WriteToLog("Clock Configuration Successful\n", "green")
else
    WriteToLog("Clock Configuration failed\n", "red")
    return -3
end

if (0 == ar1.CSI2LaneConfig_mult(deviceMapOverall, 1, 0, 2, 0, 4, 0, 5, 0, 3, 0, 0)) then
    WriteToLog("CSI2 Configuration Successful\n", "green")
else
    WriteToLog("CSI2 Configuration failed\n", "red")
    return -3
end

---------------------------Sensor Configuration-------------------------

if (0 == ar1.ProfileConfig_mult(deviceMapOverall, 0, start_freq, idle_time_p0, adc_start_time, ramp_end_time,
                                0, 0, 0, 0, 0, 0, slope, 0, adc_samples, sample_freq, 0, 0, rx_gain)) then
    WriteToLog("Profile 0 Configuration successful\n", "green")
else
    WriteToLog("Profile 0 Configuration failed\n", "red")
    return -4
end

if (0 == ar1.ProfileConfig_mult(deviceMapOverall, 1, start_freq, idle_time_p1, adc_start_time, ramp_end_time,
                                0, 0, 0, 0, 0, 0, slope, 0, adc_samples, sample_freq, 0, 0, rx_gain)) then
    WriteToLog("Profile 1 Configuration successful\n", "green")
else
    WriteToLog("Profile 1 Configuration failed\n", "red")
    return -4
end

if (0 == ar1.ProfileConfig_mult(deviceMapOverall, 2, start_freq, idle_time_p2, adc_start_time, ramp_end_time,
                                0, 0, 0, 0, 0, 0, slope, 0, adc_samples, sample_freq, 0, 0, rx_gain)) then
    WriteToLog("Profile 2 Configuration successful\n", "green")
else
    WriteToLog("Profile 2 Configuration failed\n", "red")
    return -4
end

-- Chirp geometry matches cascade_tx6_rx16_3rps.toml's txAntennaTable:
--   Dev1 master + Dev3 slave2 : TX off on EVERY chirp (RX only, all 6 chirps)
--   Dev2 slave1 : TX0/TX1/TX2 on chirp 0/1/2, TX off on chirp 3/4/5
--   Dev4 slave3 : TX off on chirp 0/1/2, TX0/TX1/TX2 on chirp 3/4/5
-- profileIdPerChirp = [0,1,2,0,1,2] - chirps 0-2 and 3-5 reuse the SAME
-- 3 profiles (not 6 separate ones).

-- Chirp 0 / Profile 0 - Dev2 TX0 active
if (0 == ar1.ChirpConfig_mult(devMap_RxOnlyAll, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)) then
    WriteToLog("Chirp 0 (Dev1+Dev3, RX only) Configuration successful\n", "green")
else
    WriteToLog("Chirp 0 (Dev1+Dev3, RX only) Configuration failed\n", "red")
    return -4
end
if (0 == ar1.ChirpConfig_mult(devMap_Dev4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)) then
    WriteToLog("Chirp 0 (Dev4, TX off) Configuration successful\n", "green")
else
    WriteToLog("Chirp 0 (Dev4, TX off) Configuration failed\n", "red")
    return -4
end
if (0 == ar1.ChirpConfig_mult(devMap_Dev2, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0)) then
    WriteToLog("Chirp 0 (Dev2 TX0) Configuration successful\n", "green")
else
    WriteToLog("Chirp 0 (Dev2 TX0) Configuration failed\n", "red")
    return -4
end

-- Chirp 1 / Profile 1 - Dev2 TX1 active
if (0 == ar1.ChirpConfig_mult(devMap_RxOnlyAll, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0)) then
    WriteToLog("Chirp 1 (Dev1+Dev3, RX only) Configuration successful\n", "green")
else
    WriteToLog("Chirp 1 (Dev1+Dev3, RX only) Configuration failed\n", "red")
    return -4
end
if (0 == ar1.ChirpConfig_mult(devMap_Dev4, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0)) then
    WriteToLog("Chirp 1 (Dev4, TX off) Configuration successful\n", "green")
else
    WriteToLog("Chirp 1 (Dev4, TX off) Configuration failed\n", "red")
    return -4
end
if (0 == ar1.ChirpConfig_mult(devMap_Dev2, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0)) then
    WriteToLog("Chirp 1 (Dev2 TX1) Configuration successful\n", "green")
else
    WriteToLog("Chirp 1 (Dev2 TX1) Configuration failed\n", "red")
    return -4
end

-- Chirp 2 / Profile 2 - Dev2 TX2 active
if (0 == ar1.ChirpConfig_mult(devMap_RxOnlyAll, 2, 2, 2, 0, 0, 0, 0, 0, 0, 0)) then
    WriteToLog("Chirp 2 (Dev1+Dev3, RX only) Configuration successful\n", "green")
else
    WriteToLog("Chirp 2 (Dev1+Dev3, RX only) Configuration failed\n", "red")
    return -4
end
if (0 == ar1.ChirpConfig_mult(devMap_Dev4, 2, 2, 2, 0, 0, 0, 0, 0, 0, 0)) then
    WriteToLog("Chirp 2 (Dev4, TX off) Configuration successful\n", "green")
else
    WriteToLog("Chirp 2 (Dev4, TX off) Configuration failed\n", "red")
    return -4
end
if (0 == ar1.ChirpConfig_mult(devMap_Dev2, 2, 2, 2, 0, 0, 0, 0, 0, 0, 1)) then
    WriteToLog("Chirp 2 (Dev2 TX2) Configuration successful\n", "green")
else
    WriteToLog("Chirp 2 (Dev2 TX2) Configuration failed\n", "red")
    return -4
end

-- Chirp 3 / Profile 0 (reused) - Dev4 TX0 active
if (0 == ar1.ChirpConfig_mult(devMap_RxOnlyAll, 3, 3, 0, 0, 0, 0, 0, 0, 0, 0)) then
    WriteToLog("Chirp 3 (Dev1+Dev3, RX only) Configuration successful\n", "green")
else
    WriteToLog("Chirp 3 (Dev1+Dev3, RX only) Configuration failed\n", "red")
    return -4
end
if (0 == ar1.ChirpConfig_mult(devMap_Dev2, 3, 3, 0, 0, 0, 0, 0, 0, 0, 0)) then
    WriteToLog("Chirp 3 (Dev2, TX off) Configuration successful\n", "green")
else
    WriteToLog("Chirp 3 (Dev2, TX off) Configuration failed\n", "red")
    return -4
end
if (0 == ar1.ChirpConfig_mult(devMap_Dev4, 3, 3, 0, 0, 0, 0, 0, 1, 0, 0)) then
    WriteToLog("Chirp 3 (Dev4 TX0) Configuration successful\n", "green")
else
    WriteToLog("Chirp 3 (Dev4 TX0) Configuration failed\n", "red")
    return -4
end

-- Chirp 4 / Profile 1 (reused) - Dev4 TX1 active
if (0 == ar1.ChirpConfig_mult(devMap_RxOnlyAll, 4, 4, 1, 0, 0, 0, 0, 0, 0, 0)) then
    WriteToLog("Chirp 4 (Dev1+Dev3, RX only) Configuration successful\n", "green")
else
    WriteToLog("Chirp 4 (Dev1+Dev3, RX only) Configuration failed\n", "red")
    return -4
end
if (0 == ar1.ChirpConfig_mult(devMap_Dev2, 4, 4, 1, 0, 0, 0, 0, 0, 0, 0)) then
    WriteToLog("Chirp 4 (Dev2, TX off) Configuration successful\n", "green")
else
    WriteToLog("Chirp 4 (Dev2, TX off) Configuration failed\n", "red")
    return -4
end
if (0 == ar1.ChirpConfig_mult(devMap_Dev4, 4, 4, 1, 0, 0, 0, 0, 0, 1, 0)) then
    WriteToLog("Chirp 4 (Dev4 TX1) Configuration successful\n", "green")
else
    WriteToLog("Chirp 4 (Dev4 TX1) Configuration failed\n", "red")
    return -4
end

-- Chirp 5 / Profile 2 (reused) - Dev4 TX2 active
if (0 == ar1.ChirpConfig_mult(devMap_RxOnlyAll, 5, 5, 2, 0, 0, 0, 0, 0, 0, 0)) then
    WriteToLog("Chirp 5 (Dev1+Dev3, RX only) Configuration successful\n", "green")
else
    WriteToLog("Chirp 5 (Dev1+Dev3, RX only) Configuration failed\n", "red")
    return -4
end
if (0 == ar1.ChirpConfig_mult(devMap_Dev2, 5, 5, 2, 0, 0, 0, 0, 0, 0, 0)) then
    WriteToLog("Chirp 5 (Dev2, TX off) Configuration successful\n", "green")
else
    WriteToLog("Chirp 5 (Dev2, TX off) Configuration failed\n", "red")
    return -4
end
if (0 == ar1.ChirpConfig_mult(devMap_Dev4, 5, 5, 2, 0, 0, 0, 0, 0, 0, 1)) then
    WriteToLog("Chirp 5 (Dev4 TX2) Configuration successful\n", "green")
else
    WriteToLog("Chirp 5 (Dev4 TX2) Configuration failed\n", "red")
    return -4
end

-- Frame Configuration
if (0 == ar1.FrameConfig_mult(1, start_chirp_tx, end_chirp_tx, nframes_master, nchirp_loops,
                              Inter_Frame_Interval, trigger_delay, 1)) then
    WriteToLog("Master : Frame Configuration successful\n", "green")
else
    WriteToLog("Master : Frame Configuration failed\n", "red")
end

if (0 == ar1.FrameConfig_mult(deviceMapSlaves, start_chirp_tx, end_chirp_tx, nframes_slave, nchirp_loops,
                              Inter_Frame_Interval, trigger_delay, 2)) then
    WriteToLog("Slaves : Frame Configuration successful\n", "green")
else
    WriteToLog("Slaves : Frame Configuration failed\n", "red")
end

---------------------------Capture Configuration-------------------------
-- Match current CLI arming where it matters for the drop A/B:
--   n_files_allocation = 1  (CLI pre-allocates; Studio used to leave this 0)
--   data_packaging     = 0  (16-bit)
--   num_frames_to_capture = 0  (TDA follows FrameConfig's 300)

local timestamp           =   os.date("%Y%m%d_%H%M%S")
capture_directory         =   "studio_tx6_rx16_3rps_" .. timestamp
n_files_allocation        =   1      -- match CLI pre-alloc; set 0 to match old Studio default
data_packaging            =   0      -- 0: 16-bit, 1: 12-bit
num_frames_to_capture     =   0      -- 0: TDA follows FrameConfig frame count (300)
stop_frame_mode           =   0

WriteToLog("Recording basename set to: " .. capture_directory .. "\n", "blue")
WriteToLog(string.format(
    "Arming TDA: files=%d packing=%d frames_arg=%d (RF FrameConfig=%d @ %dms)\n",
    n_files_allocation, data_packaging, num_frames_to_capture,
    nframes_master, Inter_Frame_Interval), "blue")

if (0 == ar1.TDACaptureCard_StartRecord_mult(deviceMapOverall, n_files_allocation, data_packaging, capture_directory, num_frames_to_capture)) then
    WriteToLog("TDA ARM Successful\n", "green")
else
    WriteToLog("TDA ARM Failed\n", "red")
    return -5
end

RSTD.Sleep(1000)

-- Slaves first (wait for HW sync), then master (software trigger + sync pulse)
if (0 == ar1.StartFrame_mult(deviceMapSlaves)) then
    WriteToLog("Slaves : Start Frame (armed, waiting for HW sync) Successful\n", "green")
else
    WriteToLog("Slaves : Start Frame Failed\n", "red")
    return -5
end

RSTD.Sleep(100)

if (0 == ar1.StartFrame_mult(deviceMapMaster)) then
    WriteToLog("Master : Start Frame (triggers RF + sync pulse) Successful\n", "green")
else
    WriteToLog("Master : Start Frame Failed\n", "red")
    return -5
end

WriteToLog("Capture running: " .. nframes_master .. " frames at " .. Inter_Frame_Interval .. "ms - expect ~"
    .. string.format("%.1f", nframes_master * Inter_Frame_Interval / 1000) .. "s total\n", "blue")

RSTD.Sleep((nframes_master * Inter_Frame_Interval) + 5000)

WriteToLog("Capture complete. Directory on TDA: /mnt/ssd/" .. capture_directory .. "\n", "blue")
WriteToLog("On the Pi, count frames with:\n", "blue")
WriteToLog("  python3 parse_idx.py --fetch " .. capture_directory
    .. " --frames " .. nframes_master .. " --period-ms " .. Inter_Frame_Interval .. "\n", "blue")
WriteToLog("Expect master_*/slave1_*/slave2_*/slave3_* .bin files (all 4 devices active).\n", "blue")
