--[[
Cascade_Configuration_Capture_cascade_tx5_rx16_robotic.lua

mmWave Studio capture matching radar_configs/cascade_tx5_rx16_robotic.toml
(UR3e robotic-arm SAR baseline). Built from
Cascade_Configuration_Capture_cascade_tx6_rx16_3rps.lua's 4-device
scaffolding, with RF timing, chirp geometry, and frame timing swapped to
match the TOML exactly.

GEOMETRY (sparse subset of the TX7 global numbering: TX4, TX6, TX8, TX10,
TX12 - matches the TOML's txAntennaTable):
  Dev1 (master) : RX on, TX off (every chirp)
  Dev2 (slave1) : RX on, TX0/TX2 on chirp0/1  (global TX4/TX6)
  Dev3 (slave2) : RX on, TX1     on chirp2    (global TX8, middle device)
  Dev4 (slave3) : RX on, TX0/TX2 on chirp3/4  (global TX10/TX12)
  -> deviceMapOverall = 1+2+4+8 = 15; all 4 devices' master_*/slave1_*/
     slave2_*/slave3_* .bin files present. 16 physical RX (4 devices x 4 RX).
  Unlike cascade_tx6_rx16_3rps (only 2 of 4 devices ever transmit), THREE
  devices (Dev2, Dev3, Dev4) each transmit on a subset of the 5 chirps -
  only Dev1 (master) is RX-only on every chirp.

RF / FRAME TIMING - PURE SAR IMAGING, NOT MIMO AoA (see
radar_configs/cascade_tx5_rx16_robotic.toml's header for the full
lambda/4 derivation; summary below):
  The radar body itself moves (mounted on the UR3e TCP), so successive
  LOOPS are samples of a mechanically-scanned synthetic aperture. Round-trip
  phase from a displacement d is phi = 4*pi*d/lambda, so consecutive loop
  positions must stay within lambda/4 (0.9734 mm at 77 GHz start freq) of
  each other to avoid phase wrap - the sub-mm Permanent-Scatterer
  displacement/vibration processing in this repo depends on that.

  UR3e arm speed used for this bound: 0.1 m/s (100 mm/s) - the ACTIVE,
  uncommented tuning in robotic_control/motion_*.py (the "HIGH SPEED"/
  "AGGRESSIVE" 1.0-1.5 m/s alternates are commented out, not used here;
  UR3e's absolute hw ceiling is 3 m/s, ~30x this baseline's speed).

  Design: fire all 5 TX in ONE tight ~235us burst per loop (idle 7/7/7 us,
  ramp 40 us - antennas effectively co-located in time/space, negligible
  0.0235mm of arm travel across the whole burst), then spend ALL the
  lambda/4 budget in the GAP BETWEEN loops (Inter_Frame_Interval), not
  spread inside one. nchirp_loops = 1 so loop == frame == one SAR aperture
  sample; Inter_Frame_Interval = 9 ms (~92% of the 9.73ms budget = lambda/4
  / 0.1 m/s) -> loop-to-loop displacement = 0.9mm, lambda/4 margin = 1.08x
  (intentionally tight, not oversampled).

  TRUE RF duty cycle (5*40us / 9ms) is only ~2.2% - inherent to sampling
  this sparsely at this arm speed, not a config mistake. If robotic_control
  is ever switched to a faster commented-out speed, Inter_Frame_Interval
  (and possibly nchirp_loops) must be recomputed - see the TOML's ARM SPEED
  <-> framePeriodicity REFERENCE table for 0.5/1.0/1.5/3.0 m/s values.

DATA RATE: 160 x 5 x 1 x 16 RX x 4 bytes / 0.009s ~= 5.7 MB/s (deliberately
low - this preset trades throughput for tight lambda/4 spatial sampling,
not a frame-drop risk profile like the spinning-blade presets).

DEFAULT CAPTURE LENGTH: 300 frames (~2.7 s @ 9 ms). This is almost
certainly too short for a real UR3e moveL scan - set nframes_master/slave
to match your actual scan duration (e.g. a 0.3m linear move at 0.1 m/s
takes 3s -> ~334 frames; pad for accel/decel).

AFTER THE CAPTURE:
  python3 parse_idx.py --fetch <capture_directory> --frames <N> --period-ms 9
  python3 check_timestamp.py <capture_directory>   -- if IR/robot sync markers were logged

BEFORE RUNNING - edit if needed:
  1. metaImagePath  (F/W path on the Windows Studio PC)
  2. TDA_IPAddress  (default 192.168.33.180)
  3. nframes_master / nframes_slave (see DEFAULT CAPTURE LENGTH above)
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
-- radar_configs/cascade_tx5_rx16_robotic.toml's txAntennaTable exactly)
devMap_Dev2       =   dev_list[2]        -- = 2  (slave1 - TX on chirp0/1)
devMap_Dev3       =   dev_list[3]        -- = 4  (slave2 - TX on chirp2 only)
devMap_Dev4       =   dev_list[4]        -- = 8  (slave3 - TX on chirp3/4)

-- "Everyone else off" groups per chirp - three devices cycle TX here (not
-- two, unlike cascade_tx6_rx16_3rps), so the off-group differs by chirp:
devMap_OffFor01   =   dev_list[1] + dev_list[3] + dev_list[4]  -- = 13 (Dev1+Dev3+Dev4) - chirp0/1 (Dev2 active)
devMap_OffFor2    =   dev_list[1] + dev_list[2] + dev_list[4]  -- = 11 (Dev1+Dev2+Dev4) - chirp2   (Dev3 active)
devMap_OffFor34   =   dev_list[1] + dev_list[2] + dev_list[3]  -- =  7 (Dev1+Dev2+Dev3) - chirp3/4 (Dev4 active)

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

-- Per-profile idle times - baseline TIGHT values (unchanged from
-- cascade_tx5_rx16_5rps.toml). All 5 TX fire as one tight ~235us burst per
-- loop; the lambda/4 budget is spent between loops (Inter_Frame_Interval
-- below), not by inflating idle here. See header derivation above.
local idle_time_p0              =   7        -- us
local idle_time_p1              =   7        -- us
local idle_time_p2              =   7        -- us

local start_chirp_tx            =   0
local end_chirp_tx              =   4        -- 5 chirps (0-4)
local nchirp_loops              =   1        -- loop == frame == one SAR aperture sample (was 255)
local nframes_master            =   300      -- ~2.7 s @ 9 ms - edit to match actual scan duration, see header
local nframes_slave             =   300
local Inter_Frame_Interval      =   9        -- ms - loop-to-loop/frame-to-frame gap, close to lambda/4 @ 0.1 m/s
local trigger_delay             =   0        -- us

------------------------------ API Configuration ------------------------------------------------

WriteToLog("Setting up Studio for Cascade (cascade_tx5_rx16_robotic)...\n", "blue")

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

-- Chirp geometry matches cascade_tx5_rx16_robotic.toml's txAntennaTable
-- and profileIdPerChirp = [0, 2, 1, 0, 2]:
--   Dev1 master        : TX off on EVERY chirp (RX only, all 5 chirps)
--   Dev2 slave1        : TX0 on chirp0 (profile0), TX2 on chirp1 (profile2)
--   Dev3 slave2        : TX1 on chirp2 (profile1) only
--   Dev4 slave3        : TX0 on chirp3 (profile0), TX2 on chirp4 (profile2)
-- Only 3 profiles for 5 chirps - profile0 and profile2 are each reused
-- twice (chirp0/chirp3 share profile0, chirp1/chirp4 share profile2).

-- Chirp 0 / Profile 0 - Dev2 TX0 active
if (0 == ar1.ChirpConfig_mult(devMap_OffFor01, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)) then
    WriteToLog("Chirp 0 (Dev1+Dev3+Dev4, RX only) Configuration successful\n", "green")
else
    WriteToLog("Chirp 0 (Dev1+Dev3+Dev4, RX only) Configuration failed\n", "red")
    return -4
end
if (0 == ar1.ChirpConfig_mult(devMap_Dev2, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0)) then
    WriteToLog("Chirp 0 (Dev2 TX0) Configuration successful\n", "green")
else
    WriteToLog("Chirp 0 (Dev2 TX0) Configuration failed\n", "red")
    return -4
end

-- Chirp 1 / Profile 2 - Dev2 TX2 active
if (0 == ar1.ChirpConfig_mult(devMap_OffFor01, 1, 1, 2, 0, 0, 0, 0, 0, 0, 0)) then
    WriteToLog("Chirp 1 (Dev1+Dev3+Dev4, RX only) Configuration successful\n", "green")
else
    WriteToLog("Chirp 1 (Dev1+Dev3+Dev4, RX only) Configuration failed\n", "red")
    return -4
end
if (0 == ar1.ChirpConfig_mult(devMap_Dev2, 1, 1, 2, 0, 0, 0, 0, 0, 0, 1)) then
    WriteToLog("Chirp 1 (Dev2 TX2) Configuration successful\n", "green")
else
    WriteToLog("Chirp 1 (Dev2 TX2) Configuration failed\n", "red")
    return -4
end

-- Chirp 2 / Profile 1 - Dev3 TX1 active (middle device)
if (0 == ar1.ChirpConfig_mult(devMap_OffFor2, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0)) then
    WriteToLog("Chirp 2 (Dev1+Dev2+Dev4, RX only) Configuration successful\n", "green")
else
    WriteToLog("Chirp 2 (Dev1+Dev2+Dev4, RX only) Configuration failed\n", "red")
    return -4
end
if (0 == ar1.ChirpConfig_mult(devMap_Dev3, 2, 2, 1, 0, 0, 0, 0, 0, 1, 0)) then
    WriteToLog("Chirp 2 (Dev3 TX1) Configuration successful\n", "green")
else
    WriteToLog("Chirp 2 (Dev3 TX1) Configuration failed\n", "red")
    return -4
end

-- Chirp 3 / Profile 0 (reused) - Dev4 TX0 active
if (0 == ar1.ChirpConfig_mult(devMap_OffFor34, 3, 3, 0, 0, 0, 0, 0, 0, 0, 0)) then
    WriteToLog("Chirp 3 (Dev1+Dev2+Dev3, RX only) Configuration successful\n", "green")
else
    WriteToLog("Chirp 3 (Dev1+Dev2+Dev3, RX only) Configuration failed\n", "red")
    return -4
end
if (0 == ar1.ChirpConfig_mult(devMap_Dev4, 3, 3, 0, 0, 0, 0, 0, 1, 0, 0)) then
    WriteToLog("Chirp 3 (Dev4 TX0) Configuration successful\n", "green")
else
    WriteToLog("Chirp 3 (Dev4 TX0) Configuration failed\n", "red")
    return -4
end

-- Chirp 4 / Profile 2 (reused) - Dev4 TX2 active
if (0 == ar1.ChirpConfig_mult(devMap_OffFor34, 4, 4, 2, 0, 0, 0, 0, 0, 0, 0)) then
    WriteToLog("Chirp 4 (Dev1+Dev2+Dev3, RX only) Configuration successful\n", "green")
else
    WriteToLog("Chirp 4 (Dev1+Dev2+Dev3, RX only) Configuration failed\n", "red")
    return -4
end
if (0 == ar1.ChirpConfig_mult(devMap_Dev4, 4, 4, 2, 0, 0, 0, 0, 0, 0, 1)) then
    WriteToLog("Chirp 4 (Dev4 TX2) Configuration successful\n", "green")
else
    WriteToLog("Chirp 4 (Dev4 TX2) Configuration failed\n", "red")
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
--   num_frames_to_capture = 0  (TDA follows FrameConfig's nframes_master)

local timestamp           =   os.date("%Y%m%d_%H%M%S")
capture_directory         =   "studio_tx5_rx16_robotic_" .. timestamp
n_files_allocation        =   1      -- match CLI pre-alloc; set 0 to match old Studio default
data_packaging            =   0      -- 0: 16-bit, 1: 12-bit
num_frames_to_capture     =   0      -- 0: TDA follows FrameConfig frame count (nframes_master)
stop_frame_mode           =   0

WriteToLog("Recording basename set to: " .. capture_directory .. "\n", "blue")
WriteToLog(string.format(
    "Arming TDA: files=%d packing=%d frames_arg=%d (RF FrameConfig=%d @ %dms, %d loop(s)/frame)\n",
    n_files_allocation, data_packaging, num_frames_to_capture,
    nframes_master, Inter_Frame_Interval, nchirp_loops), "blue")

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
WriteToLog("START THE UR3e MOVE (robotic_control/*.py, 0.1 m/s) NOW if not already running - "
    .. "this capture is timing-critical to the arm's motion.\n", "yellow")

RSTD.Sleep((nframes_master * Inter_Frame_Interval) + 5000)

WriteToLog("Capture complete. Directory on TDA: /mnt/ssd/" .. capture_directory .. "\n", "blue")
WriteToLog("On the Pi, count frames with:\n", "blue")
WriteToLog("  python3 parse_idx.py --fetch " .. capture_directory
    .. " --frames " .. nframes_master .. " --period-ms " .. Inter_Frame_Interval .. "\n", "blue")
WriteToLog("Expect master_*/slave1_*/slave2_*/slave3_* .bin files (all 4 devices active).\n", "blue")
