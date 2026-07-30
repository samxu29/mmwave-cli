--[[
Cascade_Configuration_Capture_cascade_baseline_devrx_test.lua

mmWave Studio capture matching radar_configs/cascade_baseline_devrx_test.toml.

Same RF as cascade_baseline, but Dev4 (slave3) has ALL RX off -> device
DROPPED from the cascade (RadarDevice[4]=0). Dev4's baseline chirps are
removed; remaining 9 chirps are remapped to 0-8 (Dev3/Dev2/Dev1 TX).
Pair with CLI:
  python3 mimo.py --radar-config cascade_baseline_devrx_test --frames 300 \
      --exp-name cli_cascade_baseline_devrx_test

GEOMETRY:
  Dev1+Dev2+Dev3 enabled (12 RX); Dev4 dropped
  chirp 0-2  -> Dev3 TX2/TX1/TX0
  chirp 3-5  -> Dev2 TX2/TX1/TX0
  chirp 6-8  -> Dev1 TX2/TX1/TX0
  -> deviceMapOverall = 1+2+4 = 7; master_*/slave1_*/slave2_* only

RF: slope 79, idle 5, 256 @ 8000 ksps, 64 loops, 100 ms (baseline).

AFTER THE CAPTURE:
  python3 parse_idx.py --fetch <capture_directory> --frames 300 --period-ms 100

BEFORE RUNNING - edit if needed:
  1. metaImagePath
  2. TDA_IPAddress
  3. nframes_master / nframes_slave
--]]

----------------------------------------User Constants--------------------------------------------

dev_list          =    {1, 2, 4, 8}
RadarDevice       =    {1, 1, 1, 0}       -- Dev4 dropped (RX off)
cascade_mode_list =    {1, 2, 2, 2}

metaImagePath            =   "C:\\ti\\mmwave_dfp_02_02_02_01\\firmware\\xwr22xx_metaImage.bin"
TDA_IPAddress     =   "192.168.33.180"

deviceMapOverall  =   RadarDevice[1] + (RadarDevice[2]*2) + (RadarDevice[3]*4) + (RadarDevice[4]*8)  -- = 7
deviceMapSlaves   =   (RadarDevice[2]*2) + (RadarDevice[3]*4) + (RadarDevice[4]*8)                 -- = 6
deviceMapMaster   =   RadarDevice[1]

devMap_Dev1       =   dev_list[1]   -- 1
devMap_Dev2       =   dev_list[2]   -- 2
devMap_Dev3       =   dev_list[3]   -- 4

WriteToLog(string.format(
    "deviceMapOverall=%d (expect 7), deviceMapSlaves=%d (expect 6)\n",
    deviceMapOverall, deviceMapSlaves), "blue")

------------------------------------------- Sensor Configuration ------------------------------------------------

local start_freq                =   77
local slope                     =   79
local idle_time                 =   5
local adc_start_time            =   6
local adc_samples               =   256
local sample_freq               =   8000
local ramp_end_time             =   40
local rx_gain                   =   48

local start_chirp_tx            =   0
local end_chirp_tx              =   8        -- 9 chirps
local nchirp_loops              =   64
local nframes_master            =   300
local nframes_slave             =   300
local Inter_Frame_Interval      =   100
local trigger_delay             =   0

------------------------------ API Configuration ------------------------------------------------

WriteToLog("Setting up Studio for Cascade (cascade_baseline_devrx_test)...\n", "blue")

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

if (0 == ar1.ChanNAdcConfig_mult(1,1,1,1,1,1,1,1,2,1,0,1)) then
    WriteToLog("Master : Channel & ADC Configuration Successful\n", "green")
else
    WriteToLog("Master : Channel & ADC Configuration Failed\n", "red")
    return -2
end

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

if (0 == ar1.ChanNAdcConfig_mult(deviceMapSlaves,1,1,1,1,1,1,1,2,1,0,2)) then
    WriteToLog("Slaves : Channel & ADC Configuration Successful\n", "green")
else
    WriteToLog("Slaves : Channel & ADC Configuration Failed\n", "red")
    return -2
end

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

-- Single profile (matches cascade_baseline / TOML profileId 0 on every chirp)
if (0 == ar1.ProfileConfig_mult(deviceMapOverall, 0, start_freq, idle_time, adc_start_time, ramp_end_time,
                                0, 0, 0, 0, 0, 0, slope, 0, adc_samples, sample_freq, 0, 0, rx_gain)) then
    WriteToLog("Profile 0 Configuration successful\n", "green")
else
    WriteToLog("Profile 0 Configuration failed\n", "red")
    return -4
end

-- Chirp geometry (profileId always 0):
--   0-2 Dev3 TX2/1/0; 3-5 Dev2 TX2/1/0; 6-8 Dev1 TX2/1/0
local function chirp_ok(map, idx, tx0, tx1, tx2, label)
    if (0 == ar1.ChirpConfig_mult(map, idx, idx, 0, 0, 0, 0, 0, tx0, tx1, tx2)) then
        WriteToLog(label .. " successful\n", "green")
        return true
    end
    WriteToLog(label .. " failed\n", "red")
    return false
end

-- Chirp 0: Dev3 TX2
if not chirp_ok(devMap_Dev1, 0, 0, 0, 0, "Chirp 0 Dev1 RX") then return -4 end
if not chirp_ok(devMap_Dev2, 0, 0, 0, 0, "Chirp 0 Dev2 RX") then return -4 end
if not chirp_ok(devMap_Dev3, 0, 0, 0, 1, "Chirp 0 Dev3 TX2") then return -4 end
-- Chirp 1: Dev3 TX1
if not chirp_ok(devMap_Dev1, 1, 0, 0, 0, "Chirp 1 Dev1 RX") then return -4 end
if not chirp_ok(devMap_Dev2, 1, 0, 0, 0, "Chirp 1 Dev2 RX") then return -4 end
if not chirp_ok(devMap_Dev3, 1, 0, 1, 0, "Chirp 1 Dev3 TX1") then return -4 end
-- Chirp 2: Dev3 TX0
if not chirp_ok(devMap_Dev1, 2, 0, 0, 0, "Chirp 2 Dev1 RX") then return -4 end
if not chirp_ok(devMap_Dev2, 2, 0, 0, 0, "Chirp 2 Dev2 RX") then return -4 end
if not chirp_ok(devMap_Dev3, 2, 1, 0, 0, "Chirp 2 Dev3 TX0") then return -4 end
-- Chirp 3: Dev2 TX2
if not chirp_ok(devMap_Dev1, 3, 0, 0, 0, "Chirp 3 Dev1 RX") then return -4 end
if not chirp_ok(devMap_Dev2, 3, 0, 0, 1, "Chirp 3 Dev2 TX2") then return -4 end
if not chirp_ok(devMap_Dev3, 3, 0, 0, 0, "Chirp 3 Dev3 RX") then return -4 end
-- Chirp 4: Dev2 TX1
if not chirp_ok(devMap_Dev1, 4, 0, 0, 0, "Chirp 4 Dev1 RX") then return -4 end
if not chirp_ok(devMap_Dev2, 4, 0, 1, 0, "Chirp 4 Dev2 TX1") then return -4 end
if not chirp_ok(devMap_Dev3, 4, 0, 0, 0, "Chirp 4 Dev3 RX") then return -4 end
-- Chirp 5: Dev2 TX0
if not chirp_ok(devMap_Dev1, 5, 0, 0, 0, "Chirp 5 Dev1 RX") then return -4 end
if not chirp_ok(devMap_Dev2, 5, 1, 0, 0, "Chirp 5 Dev2 TX0") then return -4 end
if not chirp_ok(devMap_Dev3, 5, 0, 0, 0, "Chirp 5 Dev3 RX") then return -4 end
-- Chirp 6: Dev1 TX2
if not chirp_ok(devMap_Dev1, 6, 0, 0, 1, "Chirp 6 Dev1 TX2") then return -4 end
if not chirp_ok(devMap_Dev2, 6, 0, 0, 0, "Chirp 6 Dev2 RX") then return -4 end
if not chirp_ok(devMap_Dev3, 6, 0, 0, 0, "Chirp 6 Dev3 RX") then return -4 end
-- Chirp 7: Dev1 TX1
if not chirp_ok(devMap_Dev1, 7, 0, 1, 0, "Chirp 7 Dev1 TX1") then return -4 end
if not chirp_ok(devMap_Dev2, 7, 0, 0, 0, "Chirp 7 Dev2 RX") then return -4 end
if not chirp_ok(devMap_Dev3, 7, 0, 0, 0, "Chirp 7 Dev3 RX") then return -4 end
-- Chirp 8: Dev1 TX0
if not chirp_ok(devMap_Dev1, 8, 1, 0, 0, "Chirp 8 Dev1 TX0") then return -4 end
if not chirp_ok(devMap_Dev2, 8, 0, 0, 0, "Chirp 8 Dev2 RX") then return -4 end
if not chirp_ok(devMap_Dev3, 8, 0, 0, 0, "Chirp 8 Dev3 RX") then return -4 end

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

local timestamp           =   os.date("%Y%m%d_%H%M%S")
capture_directory         =   "studio_cascade_baseline_devrx_test_" .. timestamp
n_files_allocation        =   1
data_packaging            =   0
num_frames_to_capture     =   0
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

if (0 == ar1.StartFrame_mult(deviceMapSlaves)) then
    WriteToLog("Slaves : Start Frame Successful\n", "green")
else
    WriteToLog("Slaves : Start Frame Failed\n", "red")
    return -5
end

RSTD.Sleep(100)

if (0 == ar1.StartFrame_mult(deviceMapMaster)) then
    WriteToLog("Master : Start Frame Successful\n", "green")
else
    WriteToLog("Master : Start Frame Failed\n", "red")
    return -5
end

WriteToLog("Capture running: " .. nframes_master .. " frames at " .. Inter_Frame_Interval .. "ms\n", "blue")
RSTD.Sleep((nframes_master * Inter_Frame_Interval) + 5000)

WriteToLog("Capture complete. Directory on TDA: /mnt/ssd/" .. capture_directory .. "\n", "blue")
WriteToLog("  python3 parse_idx.py --fetch " .. capture_directory
    .. " --frames " .. nframes_master .. " --period-ms " .. Inter_Frame_Interval .. "\n", "blue")
WriteToLog("Expect master_*/slave1_*/slave2_* only (Dev4 dropped).\n", "blue")
