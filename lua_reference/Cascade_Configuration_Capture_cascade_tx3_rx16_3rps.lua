--[[
Cascade_Configuration_Capture_cascade_tx3_rx16_3rps.lua

mmWave Studio capture matching radar_configs/cascade_tx3_rx16_3rps.toml.

GEOMETRY (same as the TOML):
  Dev1–3 : RX on, TX off
  Dev4   : RX on, TX0/TX1/TX2 on chirps 0/1/2
  -> deviceMapOverall = 15; master + slave1 + slave2 + slave3 .bin files

RF (same as the TOML / CLI):
  3 profiles, idle 5/5/5 us, ramp 40 us, slope 99.987 MHz/us
  160 samples @ 4850 ksps, rxGain 48, numLoops 255, period 37 ms
  (~27 Hz frame rate - "3rps" spinning-radar spacing)

  Active chirp time ≈ (5+40)*3 * 255 ≈ 34.4 ms per 37 ms frame (~93% duty).

  WRITE RATE WARNING: 4 devices x 4 RX at this schedule needs ~212 MB/s, well
  above the TDA SSD's measured ~92 MB/s. Expect heavy frame drops on long
  captures unless you shorten length or drop devices/RX. Use parse_idx.py.

DEFAULT CAPTURE LENGTH: 300 frames (~11.1 s). Edit nframes_master/slave below.

AFTER THE CAPTURE:
  python3 parse_idx.py --fetch <capture_directory> --frames 300 --period-ms 37

BEFORE RUNNING - edit if needed:
  1. metaImagePath  (F/W path on the Windows Studio PC)
  2. TDA_IPAddress  (default 192.168.33.180)
  3. nframes_master / nframes_slave
--]]

----------------------------------------User Constants--------------------------------------------

dev_list          =    {1, 2, 4, 8}
RadarDevice       =    {1, 1, 1, 1}       -- all 4 devices - matches rxChannelEn = 0x0F
cascade_mode_list =    {1, 2, 2, 2}

metaImagePath            =   "C:\\ti\\mmwave_dfp_02_02_02_01\\firmware\\xwr22xx_metaImage.bin"
-- metaImagePath            =   "C:\\ti\\mmwave_dfp_02_02_00_02\\firmware\\xwr22xx_metaImage.bin"

TDA_IPAddress     =   "192.168.33.180"

deviceMapOverall  =   RadarDevice[1] + (RadarDevice[2]*2) + (RadarDevice[3]*4) + (RadarDevice[4]*8)  -- 15
deviceMapSlaves   =   (RadarDevice[2]*2) + (RadarDevice[3]*4) + (RadarDevice[4]*8)                 -- 14
deviceMapMaster   =   RadarDevice[1]     -- 1
-- Chirp TX is Dev4-only; Dev2+Dev3 stay RX-only (do NOT ChirpConfig TX onto deviceMapSlaves)
deviceMapSlaveRxOnly = 2 + 4             -- Dev2 + Dev3
deviceMapDev4        = 8                 -- Dev4 / slave3

WriteToLog(string.format(
    "deviceMapOverall=%d (expect 15), slaves=%d, master=%d, Dev4=%d\n",
    deviceMapOverall, deviceMapSlaves, deviceMapMaster, deviceMapDev4), "blue")

------------------------------------------- Sensor Configuration ------------------------------------------------

local start_freq                =   77       -- GHz
local slope                     =   99.987   -- MHz/us
local adc_start_time            =   6        -- us
local adc_samples               =   160
local sample_freq               =   4850     -- ksps
local ramp_end_time             =   40       -- us
local rx_gain                   =   48       -- dB

local idle_time_p0              =   5        -- us
local idle_time_p1              =   5        -- us
local idle_time_p2              =   5        -- us

local start_chirp_tx            =   0
local end_chirp_tx              =   2
local nchirp_loops              =   255
local nframes_master            =   300      -- ~11.1 s @ 37 ms; edit as needed
local nframes_slave             =   300
local Inter_Frame_Interval      =   37       -- ms
local trigger_delay             =   0        -- us

------------------------------ API Configuration ------------------------------------------------

WriteToLog("Setting up Studio for Cascade (cascade_tx3_rx16_3rps)...\n", "blue")

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

if (0 == ar1.ChanNAdcConfig_mult(1,1,1,1,1,1,1,1,2,1,0,1)) then
    WriteToLog("Master : Channel & ADC Configuration Successful\n", "green")
else
    WriteToLog("Master : Channel & ADC Configuration Failed\n", "red")
    return -2
end

-- Slaves Initialization (Dev2, Dev3, Dev4)

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

-- All devices together

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

-- Chirp geometry matches cascade_tx3_rx16_3rps.toml txAntennaTable:
--   Dev1 / Dev2 / Dev3 : TX off
--   Dev4               : TX0 / TX1 / TX2 on chirp 0 / 1 / 2

-- Chirp 0 / Profile 0
if (0 == ar1.ChirpConfig_mult(deviceMapMaster, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)) then
    WriteToLog("Chirp 0 (Dev1, TX off) Configuration successful\n", "green")
else
    WriteToLog("Chirp 0 (Dev1, TX off) Configuration failed\n", "red")
    return -4
end
if (0 == ar1.ChirpConfig_mult(deviceMapSlaveRxOnly, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)) then
    WriteToLog("Chirp 0 (Dev2+Dev3, TX off) Configuration successful\n", "green")
else
    WriteToLog("Chirp 0 (Dev2+Dev3, TX off) Configuration failed\n", "red")
    return -4
end
if (0 == ar1.ChirpConfig_mult(deviceMapDev4, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0)) then
    WriteToLog("Chirp 0 (Dev4 TX0) Configuration successful\n", "green")
else
    WriteToLog("Chirp 0 (Dev4 TX0) Configuration failed\n", "red")
    return -4
end

-- Chirp 1 / Profile 1
if (0 == ar1.ChirpConfig_mult(deviceMapMaster, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0)) then
    WriteToLog("Chirp 1 (Dev1, TX off) Configuration successful\n", "green")
else
    WriteToLog("Chirp 1 (Dev1, TX off) Configuration failed\n", "red")
    return -4
end
if (0 == ar1.ChirpConfig_mult(deviceMapSlaveRxOnly, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0)) then
    WriteToLog("Chirp 1 (Dev2+Dev3, TX off) Configuration successful\n", "green")
else
    WriteToLog("Chirp 1 (Dev2+Dev3, TX off) Configuration failed\n", "red")
    return -4
end
if (0 == ar1.ChirpConfig_mult(deviceMapDev4, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0)) then
    WriteToLog("Chirp 1 (Dev4 TX1) Configuration successful\n", "green")
else
    WriteToLog("Chirp 1 (Dev4 TX1) Configuration failed\n", "red")
    return -4
end

-- Chirp 2 / Profile 2
if (0 == ar1.ChirpConfig_mult(deviceMapMaster, 2, 2, 2, 0, 0, 0, 0, 0, 0, 0)) then
    WriteToLog("Chirp 2 (Dev1, TX off) Configuration successful\n", "green")
else
    WriteToLog("Chirp 2 (Dev1, TX off) Configuration failed\n", "red")
    return -4
end
if (0 == ar1.ChirpConfig_mult(deviceMapSlaveRxOnly, 2, 2, 2, 0, 0, 0, 0, 0, 0, 0)) then
    WriteToLog("Chirp 2 (Dev2+Dev3, TX off) Configuration successful\n", "green")
else
    WriteToLog("Chirp 2 (Dev2+Dev3, TX off) Configuration failed\n", "red")
    return -4
end
if (0 == ar1.ChirpConfig_mult(deviceMapDev4, 2, 2, 2, 0, 0, 0, 0, 0, 0, 1)) then
    WriteToLog("Chirp 2 (Dev4 TX2) Configuration successful\n", "green")
else
    WriteToLog("Chirp 2 (Dev4 TX2) Configuration failed\n", "red")
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

local timestamp           =   os.date("%Y%m%d_%H%M%S")
capture_directory         =   "studio_tx3_rx16_3rps_" .. timestamp
n_files_allocation        =   1      -- match CLI pre-alloc; set 0 for old Studio default
data_packaging            =   0      -- 0: 16-bit, 1: 12-bit
num_frames_to_capture     =   0      -- 0: TDA follows FrameConfig frame count
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
WriteToLog("Expect master + slave1 + slave2 + slave3 *_0000_*.bin files.\n", "blue")
