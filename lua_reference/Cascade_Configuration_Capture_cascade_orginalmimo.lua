--[[
Sequence being followed

A. CONFIGURATION
1. Connecting to TDA
1. Configuring Master from SOP till Channel Config
2. Configuring Slave (i) sequentially from SOP till SPI Connect. i = 1, 2, 3
3. Configuring Slaves together from F/W download till Channel Config
4. Configuring all devices together from LDO Bypass till Frame Config

Modified from Cascade_Configuration_TestSource.lua, then re-synced to match
Cascade_Configuration_MIMO.lua's sensor settings (this update):
- Test source REMOVED (no synthetic targets, real RF only)
- SENSOR CONFIG NOW MATCHES Cascade_Configuration_MIMO.lua's full 2x12 cascade
  MIMO scheme (previously this file only chirped the master's TX0/TX1/TX2 in
  a 3-chirp TDM test with slaves fully off - that mismatch is what this
  update fixes):
    - Single Profile 0 (idle_time=5us, ramp_end_time=40us, slope=79 MHz/us -
      exact values from Cascade_Configuration_MIMO.lua), instead of the old
      3 profiles with different idle times.
    - Chirp Config: 12 chirps (0-11) via the Configure_Chirps(i) function,
      ported verbatim from Cascade_Configuration_MIMO.lua - one TX antenna
      enabled per chirp per device (Dev1 TX0/TX1/TX2 -> chirps 11/10/9;
      Dev2 -> chirps 8/7/6; Dev3 -> chirps 5/4/3; Dev4 -> chirps 2/1/0),
      giving all 12 unique TX combinations across the 4-device cascade.
    - Frame Config chirp range updated to 0-11 (was 0-2), nchirp_loops = 64
      (was 255) - both taken from Cascade_Configuration_MIMO.lua to match
      the 12-chirp sequence.
- Frame trigger: Master = Software (1), Slaves = Hardware sync from master (2)
  (slaves are ALWAYS hardware-synced off the master's internal distribution -
  this is fixed cascade architecture, not a user-selectable "software" option)
- nframes_master/slave = 300, Inter_Frame_Interval = 100 ms -> 30s total capture
  (frame count/interval govern capture DURATION, not the MIMO antenna scheme)
- Capture/ARM/Trigger section (timestamped capture_directory,
  TDACaptureCard_StartRecord_mult ARM, slaves-then-master StartFrame_mult
  trigger order) is UNCHANGED - already working, kept as-is.

NOTE:
Update the following in the script accordingly before running
1. metaImage F/W path on line ~40
2. TDA Host Board IP Address on line ~47
--]]

----------------------------------------User Constants--------------------------------------------

dev_list          =    {1, 2, 4, 8}       -- Device map
RadarDevice       =    {1, 1, 1, 1}       -- {dev1, dev2, dev3, dev4}, 1: Enable, 0: Disable
cascade_mode_list =    {1, 2, 2, 2}       -- 0: Single chip, 1: Master, 2: Slave

-- F/W Download Path
-- metaImagePath  =   RSTD.BrowseForFile(RSTD.GetSettingsPath(), "bin", "Browse to .bin file")
-- For 2243 ES1.1 devices
metaImagePath            =   "C:\\ti\\mmwave_dfp_02_02_02_01\\firmware\\xwr22xx_metaImage.bin"
-- For 2243 ES1.0 devices
-- metaImagePath            =   "C:\\ti\\mmwave_dfp_02_02_00_02\\firmware\\xwr22xx_metaImage.bin"

-- IP Address for the TDA2 Host Board
TDA_IPAddress     =   "192.168.33.180"

-- Device map of all the devices to be enabled by TDA
-- 1 - master ; 2- slave1 ; 4 - slave2 ; 8 - slave3

deviceMapOverall  =   RadarDevice[1] + (RadarDevice[2]*2) + (RadarDevice[3]*4) + (RadarDevice[4]*8)
deviceMapSlaves   =   (RadarDevice[2]*2) + (RadarDevice[3]*4) + (RadarDevice[4]*8)
deviceMapMaster   =   RadarDevice[1]     -- = 1, isolated for per-device chirp calls

------------------------------------------- Sensor Configuration ------------------------------------------------
--[[
Single profile (Profile 0), shared by all 12 chirps - values ported verbatim
from Cascade_Configuration_MIMO.lua:
  start_freq 77GHz, slope 79 MHz/us, idle_time 5us, ramp_end_time 40us
  (77 + 79*40us = 80.16GHz, under ~81GHz VCO ceiling)
Chirp Config (12 chirps, one TX enabled per chirp per device) is implemented
by the Configure_Chirps(i) function below - see API Configuration section.
--]]

-- RF params (matching Cascade_Configuration_MIMO.lua exactly)
local start_freq                =   77     -- GHz
local slope                     =   79     -- MHz/us
local idle_time                 =   5      -- us
local adc_start_time            =   6      -- us
local adc_samples               =   256    -- Number of samples per chirp
local sample_freq               =   8000   -- ksps
local ramp_end_time             =   40     -- us
local rx_gain                   =   48     -- dB

-- Frame configuration
local start_chirp_tx            =   0
local end_chirp_tx              =   11     -- 12 chirps (0-11), matching Cascade_Configuration_MIMO.lua
local nchirp_loops              =   64     -- Number of chirp loops per frame (matching Cascade_Configuration_MIMO.lua)
local nframes_master            =   300    -- Number of Frames for Master (300 x 100ms = 30s total capture)
local nframes_slave             =   300    -- Number of Frames for Slaves
local Inter_Frame_Interval      =   100    -- ms
local trigger_delay             =   0      -- us

--[[
Chirp Configuration - ported verbatim from Cascade_Configuration_MIMO.lua.
12 chirps, one TX antenna enabled per chirp per device, giving all 12 unique
TX combinations across the 4-device cascade (2x12 MIMO):

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
--]]
-- ar1.ChirpConfig_mult(RadarDeviceId, chirpStartIdx, chirpEndIdx, profileId, startFreqVar,
--                      freqSlopeVar, idleTimeVar, adcStartTimeVar, tx0Enable, tx1Enable, tx2Enable)

function Configure_Chirps(i)

    if (i == 1) then
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)) then WriteToLog("Device "..i.." : Chirp 0 Configuration successful\n", "green") else WriteToLog("Device "..i.." : Chirp 0 Configuration failed\n", "red") return -4 end
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 1, 1, 0, 0, 0, 0, 0, 0, 0, 0)) then WriteToLog("Device "..i.." : Chirp 1 Configuration successful\n", "green") else WriteToLog("Device "..i.." : Chirp 1 Configuration failed\n", "red") return -4 end
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 2, 2, 0, 0, 0, 0, 0, 0, 0, 0)) then WriteToLog("Device "..i.." : Chirp 2 Configuration successful\n", "green") else WriteToLog("Device "..i.." : Chirp 2 Configuration failed\n", "red") return -4 end
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 3, 3, 0, 0, 0, 0, 0, 0, 0, 0)) then WriteToLog("Device "..i.." : Chirp 3 Configuration successful\n", "green") else WriteToLog("Device "..i.." : Chirp 3 Configuration failed\n", "red") return -4 end
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 4, 4, 0, 0, 0, 0, 0, 0, 0, 0)) then WriteToLog("Device "..i.." : Chirp 4 Configuration successful\n", "green") else WriteToLog("Device "..i.." : Chirp 4 Configuration failed\n", "red") return -4 end
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 5, 5, 0, 0, 0, 0, 0, 0, 0, 0)) then WriteToLog("Device "..i.." : Chirp 5 Configuration successful\n", "green") else WriteToLog("Device "..i.." : Chirp 5 Configuration failed\n", "red") return -4 end
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 6, 6, 0, 0, 0, 0, 0, 0, 0, 0)) then WriteToLog("Device "..i.." : Chirp 6 Configuration successful\n", "green") else WriteToLog("Device "..i.." : Chirp 6 Configuration failed\n", "red") return -4 end
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 7, 7, 0, 0, 0, 0, 0, 0, 0, 0)) then WriteToLog("Device "..i.." : Chirp 7 Configuration successful\n", "green") else WriteToLog("Device "..i.." : Chirp 7 Configuration failed\n", "red") return -4 end
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 8, 8, 0, 0, 0, 0, 0, 0, 0, 0)) then WriteToLog("Device "..i.." : Chirp 8 Configuration successful\n", "green") else WriteToLog("Device "..i.." : Chirp 8 Configuration failed\n", "red") return -4 end
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 9, 9, 0, 0, 0, 0, 0, 0, 0, 1)) then WriteToLog("Device "..i.." : Chirp 9 Configuration successful\n", "green") else WriteToLog("Device "..i.." : Chirp 9 Configuration failed\n", "red") return -4 end
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 10, 10, 0, 0, 0, 0, 0, 0, 1, 0)) then WriteToLog("Device "..i.." : Chirp 10 Configuration successful\n", "green") else WriteToLog("Device "..i.." : Chirp 10 Configuration failed\n", "red") return -4 end
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 11, 11, 0, 0, 0, 0, 0, 1, 0, 0)) then WriteToLog("Device "..i.." : Chirp 11 Configuration successful\n", "green") else WriteToLog("Device "..i.." : Chirp 11 Configuration failed\n", "red") return -4 end

    elseif (i == 2) then
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)) then WriteToLog("Device "..i.." : Chirp 0 Configuration successful\n", "green") else WriteToLog("Device "..i.." : Chirp 0 Configuration failed\n", "red") return -4 end
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 1, 1, 0, 0, 0, 0, 0, 0, 0, 0)) then WriteToLog("Device "..i.." : Chirp 1 Configuration successful\n", "green") else WriteToLog("Device "..i.." : Chirp 1 Configuration failed\n", "red") return -4 end
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 2, 2, 0, 0, 0, 0, 0, 0, 0, 0)) then WriteToLog("Device "..i.." : Chirp 2 Configuration successful\n", "green") else WriteToLog("Device "..i.." : Chirp 2 Configuration failed\n", "red") return -4 end
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 3, 3, 0, 0, 0, 0, 0, 0, 0, 0)) then WriteToLog("Device "..i.." : Chirp 3 Configuration successful\n", "green") else WriteToLog("Device "..i.." : Chirp 3 Configuration failed\n", "red") return -4 end
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 4, 4, 0, 0, 0, 0, 0, 0, 0, 0)) then WriteToLog("Device "..i.." : Chirp 4 Configuration successful\n", "green") else WriteToLog("Device "..i.." : Chirp 4 Configuration failed\n", "red") return -4 end
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 5, 5, 0, 0, 0, 0, 0, 0, 0, 0)) then WriteToLog("Device "..i.." : Chirp 5 Configuration successful\n", "green") else WriteToLog("Device "..i.." : Chirp 5 Configuration failed\n", "red") return -4 end
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 6, 6, 0, 0, 0, 0, 0, 0, 0, 1)) then WriteToLog("Device "..i.." : Chirp 6 Configuration successful\n", "green") else WriteToLog("Device "..i.." : Chirp 6 Configuration failed\n", "red") return -4 end
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 7, 7, 0, 0, 0, 0, 0, 0, 1, 0)) then WriteToLog("Device "..i.." : Chirp 7 Configuration successful\n", "green") else WriteToLog("Device "..i.." : Chirp 7 Configuration failed\n", "red") return -4 end
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 8, 8, 0, 0, 0, 0, 0, 1, 0, 0)) then WriteToLog("Device "..i.." : Chirp 8 Configuration successful\n", "green") else WriteToLog("Device "..i.." : Chirp 8 Configuration failed\n", "red") return -4 end
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 9, 9, 0, 0, 0, 0, 0, 0, 0, 0)) then WriteToLog("Device "..i.." : Chirp 9 Configuration successful\n", "green") else WriteToLog("Device "..i.." : Chirp 9 Configuration failed\n", "red") return -4 end
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 10, 10, 0, 0, 0, 0, 0, 0, 0, 0)) then WriteToLog("Device "..i.." : Chirp 10 Configuration successful\n", "green") else WriteToLog("Device "..i.." : Chirp 10 Configuration failed\n", "red") return -4 end
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 11, 11, 0, 0, 0, 0, 0, 0, 0, 0)) then WriteToLog("Device "..i.." : Chirp 11 Configuration successful\n", "green") else WriteToLog("Device "..i.." : Chirp 11 Configuration failed\n", "red") return -4 end

    elseif (i == 3) then
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)) then WriteToLog("Device "..i.." : Chirp 0 Configuration successful\n", "green") else WriteToLog("Device "..i.." : Chirp 0 Configuration failed\n", "red") return -4 end
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 1, 1, 0, 0, 0, 0, 0, 0, 0, 0)) then WriteToLog("Device "..i.." : Chirp 1 Configuration successful\n", "green") else WriteToLog("Device "..i.." : Chirp 1 Configuration failed\n", "red") return -4 end
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 2, 2, 0, 0, 0, 0, 0, 0, 0, 0)) then WriteToLog("Device "..i.." : Chirp 2 Configuration successful\n", "green") else WriteToLog("Device "..i.." : Chirp 2 Configuration failed\n", "red") return -4 end
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 3, 3, 0, 0, 0, 0, 0, 0, 0, 1)) then WriteToLog("Device "..i.." : Chirp 3 Configuration successful\n", "green") else WriteToLog("Device "..i.." : Chirp 3 Configuration failed\n", "red") return -4 end
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 4, 4, 0, 0, 0, 0, 0, 0, 1, 0)) then WriteToLog("Device "..i.." : Chirp 4 Configuration successful\n", "green") else WriteToLog("Device "..i.." : Chirp 4 Configuration failed\n", "red") return -4 end
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 5, 5, 0, 0, 0, 0, 0, 1, 0, 0)) then WriteToLog("Device "..i.." : Chirp 5 Configuration successful\n", "green") else WriteToLog("Device "..i.." : Chirp 5 Configuration failed\n", "red") return -4 end
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 6, 6, 0, 0, 0, 0, 0, 0, 0, 0)) then WriteToLog("Device "..i.." : Chirp 6 Configuration successful\n", "green") else WriteToLog("Device "..i.." : Chirp 6 Configuration failed\n", "red") return -4 end
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 7, 7, 0, 0, 0, 0, 0, 0, 0, 0)) then WriteToLog("Device "..i.." : Chirp 7 Configuration successful\n", "green") else WriteToLog("Device "..i.." : Chirp 7 Configuration failed\n", "red") return -4 end
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 8, 8, 0, 0, 0, 0, 0, 0, 0, 0)) then WriteToLog("Device "..i.." : Chirp 8 Configuration successful\n", "green") else WriteToLog("Device "..i.." : Chirp 8 Configuration failed\n", "red") return -4 end
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 9, 9, 0, 0, 0, 0, 0, 0, 0, 0)) then WriteToLog("Device "..i.." : Chirp 9 Configuration successful\n", "green") else WriteToLog("Device "..i.." : Chirp 9 Configuration failed\n", "red") return -4 end
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 10, 10, 0, 0, 0, 0, 0, 0, 0, 0)) then WriteToLog("Device "..i.." : Chirp 10 Configuration successful\n", "green") else WriteToLog("Device "..i.." : Chirp 10 Configuration failed\n", "red") return -4 end
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 11, 11, 0, 0, 0, 0, 0, 0, 0, 0)) then WriteToLog("Device "..i.." : Chirp 11 Configuration successful\n", "green") else WriteToLog("Device "..i.." : Chirp 11 Configuration failed\n", "red") return -4 end

    elseif (i == 4) then
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 0, 0, 0, 0, 0, 0, 0, 0, 0, 1)) then WriteToLog("Device "..i.." : Chirp 0 Configuration successful\n", "green") else WriteToLog("Device "..i.." : Chirp 0 Configuration failed\n", "red") return -4 end
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 1, 1, 0, 0, 0, 0, 0, 0, 1, 0)) then WriteToLog("Device "..i.." : Chirp 1 Configuration successful\n", "green") else WriteToLog("Device "..i.." : Chirp 1 Configuration failed\n", "red") return -4 end
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 2, 2, 0, 0, 0, 0, 0, 1, 0, 0)) then WriteToLog("Device "..i.." : Chirp 2 Configuration successful\n", "green") else WriteToLog("Device "..i.." : Chirp 2 Configuration failed\n", "red") return -4 end
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 3, 3, 0, 0, 0, 0, 0, 0, 0, 0)) then WriteToLog("Device "..i.." : Chirp 3 Configuration successful\n", "green") else WriteToLog("Device "..i.." : Chirp 3 Configuration failed\n", "red") return -4 end
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 4, 4, 0, 0, 0, 0, 0, 0, 0, 0)) then WriteToLog("Device "..i.." : Chirp 4 Configuration successful\n", "green") else WriteToLog("Device "..i.." : Chirp 4 Configuration failed\n", "red") return -4 end
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 5, 5, 0, 0, 0, 0, 0, 0, 0, 0)) then WriteToLog("Device "..i.." : Chirp 5 Configuration successful\n", "green") else WriteToLog("Device "..i.." : Chirp 5 Configuration failed\n", "red") return -4 end
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 6, 6, 0, 0, 0, 0, 0, 0, 0, 0)) then WriteToLog("Device "..i.." : Chirp 6 Configuration successful\n", "green") else WriteToLog("Device "..i.." : Chirp 6 Configuration failed\n", "red") return -4 end
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 7, 7, 0, 0, 0, 0, 0, 0, 0, 0)) then WriteToLog("Device "..i.." : Chirp 7 Configuration successful\n", "green") else WriteToLog("Device "..i.." : Chirp 7 Configuration failed\n", "red") return -4 end
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 8, 8, 0, 0, 0, 0, 0, 0, 0, 0)) then WriteToLog("Device "..i.." : Chirp 8 Configuration successful\n", "green") else WriteToLog("Device "..i.." : Chirp 8 Configuration failed\n", "red") return -4 end
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 9, 9, 0, 0, 0, 0, 0, 0, 0, 0)) then WriteToLog("Device "..i.." : Chirp 9 Configuration successful\n", "green") else WriteToLog("Device "..i.." : Chirp 9 Configuration failed\n", "red") return -4 end
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 10, 10, 0, 0, 0, 0, 0, 0, 0, 0)) then WriteToLog("Device "..i.." : Chirp 10 Configuration successful\n", "green") else WriteToLog("Device "..i.." : Chirp 10 Configuration failed\n", "red") return -4 end
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 11, 11, 0, 0, 0, 0, 0, 0, 0, 0)) then WriteToLog("Device "..i.." : Chirp 11 Configuration successful\n", "green") else WriteToLog("Device "..i.." : Chirp 11 Configuration failed\n", "red") return -4 end

    end

end

------------------------------ API Configuration ------------------------------------------------

WriteToLog("Setting up Studio for Cascade started..\n", "blue")

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

-- SOP Mode Configuration
if (0 == ar1.SOPControl_mult(1, 4)) then
    WriteToLog("Master : SOP Reset Successful\n", "green")
else
    WriteToLog("Master : SOP Reset Failed\n", "red")
    return -1
end

-- SPI Connect
if (0 == ar1.PowerOn_mult(1, 0, 1000, 0, 0)) then
    WriteToLog("Master : SPI Connection Successful\n", "green")
else
    WriteToLog("Master : SPI Connection Failed\n", "red")
    return -1
end

-- Firmware Download. (SOP 4 - MetaImage)
if (0 == ar1.DownloadBssFwOvSPI_mult(1, metaImagePath)) then
    WriteToLog("Master : FW Download Successful\n", "green")
else
    WriteToLog("Master : FW Download Failed\n", "red")
    return -1
end

-- RF Power Up
if (0 == ar1.RfEnable_mult(1)) then
    WriteToLog("Master : RF Power Up Successful\n", "green")
else
    WriteToLog("Master : RF Power Up Failed\n", "red")
    return -1
end

-- Channel & ADC Configuration
if (0 == ar1.ChanNAdcConfig_mult(1,1,1,1,1,1,1,1,2,1,0,1)) then
    WriteToLog("Master : Channel & ADC Configuration Successful\n", "green")
else
    WriteToLog("Master : Channel & ADC Configuration Failed\n", "red")
    return -2
end

-- Slaves Initialization

for i=2,table.getn(RadarDevice) do
    local status    =    0
    if ((RadarDevice[1]==1) and (RadarDevice[i]==1)) then

        -- SOP Mode Configuration
        if (0 == ar1.SOPControl_mult(dev_list[i], 4)) then
            WriteToLog("Device "..i.." : SOP Reset Successful\n", "green")
        else
            WriteToLog("Device "..i.." : SOP Reset Failed\n", "red")
            return -1
        end

        -- SPI Connect
        if (0 == ar1.AddDevice(dev_list[i])) then
            WriteToLog("Device "..i.." : SPI Connection Successful\n", "green")
        else
            WriteToLog("Device "..i.." : SPI Connection Failed\n", "red")
            return -1
        end

    end
end

-- Firmware Download. (SOP 4 - MetaImage)
if (0 == ar1.DownloadBssFwOvSPI_mult(deviceMapSlaves, metaImagePath)) then
    WriteToLog("Slaves : FW Download Successful\n", "green")
else
    WriteToLog("Slaves : FW Download Failed\n", "red")
    return -1
end

-- RF Power Up
if (0 == ar1.RfEnable_mult(deviceMapSlaves)) then
    WriteToLog("Slaves : RF Power Up Successful\n", "green")
else
    WriteToLog("Slaves : RF Power Up Failed\n", "red")
    return -1
end

-- Channel & ADC Configuration
if (0 == ar1.ChanNAdcConfig_mult(deviceMapSlaves,1,1,1,1,1,1,1,2,1,0,2)) then
    WriteToLog("Slaves : Channel & ADC Configuration Successful\n", "green")
else
    WriteToLog("Slaves : Channel & ADC Configuration Failed\n", "red")
    return -2
end

-- All devices together

-- LDO configuration
if (0 == ar1.RfLdoBypassConfig_mult(deviceMapOverall, 3)) then
    WriteToLog("LDO Bypass Successful\n", "green")
else
    WriteToLog("LDO Bypass failed\n", "red")
    return -2
end

-- Low Power Mode Configuration
if (0 == ar1.LPModConfig_mult(deviceMapOverall,0, 0)) then
    WriteToLog("Low Power Mode Configuration Successful\n", "green")
else
    WriteToLog("Low Power Mode Configuration failed\n", "red")
    return -2
end

-- Miscellaneous Control Configuration
if (0 == ar1.SetMiscConfig_mult(deviceMapOverall, 1, 0, 0, 0)) then
    WriteToLog("Misc Control Configuration Successful\n", "green")
else
    WriteToLog("Misc Control Configuration failed\n", "red")
    return -2
end

-- RF Init Calibration Configuration
if (0 == ar1.RfInitCalibConfig_mult(deviceMapOverall, 1, 1, 1, 1, 1, 1, 1, 65537)) then
    WriteToLog("RF Init Calibration Successful\n", "green")
else
    WriteToLog("RF Init Calibration failed\n", "red")
    return -2
end

-- RF Init
if (0 == ar1.RfInit_mult(deviceMapOverall)) then
    WriteToLog("RF Init Successful\n", "green")
else
    WriteToLog("RF Init failed\n", "red")
    return -2
end

---------------------------Data Configuration----------------------------------

-- Data path Configuration
if (0 == ar1.DataPathConfig_mult(deviceMapOverall, 0, 1, 0)) then
    WriteToLog("Data Path Configuration Successful\n", "green")
else
    WriteToLog("Data Path Configuration failed\n", "red")
    return -3
end

-- Clock Configuration
if (0 == ar1.LvdsClkConfig_mult(deviceMapOverall, 1, 1)) then
    WriteToLog("Clock Configuration Successful\n", "green")
else
    WriteToLog("Clock Configuration failed\n", "red")
    return -3
end

-- CSI2 Configuration
if (0 == ar1.CSI2LaneConfig_mult(deviceMapOverall, 1, 0, 2, 0, 4, 0, 5, 0, 3, 0, 0)) then
    WriteToLog("CSI2 Configuration Successful\n", "green")
else
    WriteToLog("CSI2 Configuration failed\n", "red")
    return -3
end

-- NOTE: Test Source section intentionally removed - real RF only, no synthetic targets.

---------------------------Sensor Configuration-------------------------

-- Profile 0 (shared by all 12 chirps): matches Cascade_Configuration_MIMO.lua
if (0 == ar1.ProfileConfig_mult(deviceMapOverall, 0, start_freq, idle_time, adc_start_time, ramp_end_time,
                                0, 0, 0, 0, 0, 0, slope, 0, adc_samples, sample_freq, 0, 0, rx_gain)) then
    WriteToLog("Profile 0 Configuration successful\n", "green")
else
    WriteToLog("Profile 0 Configuration failed\n", "red")
    return -4
end

-- Chirp Configuration: 12 chirps (0-11), per-device TX table from Cascade_Configuration_MIMO.lua
-- (see Configure_Chirps(i) function defined above, in the Sensor Configuration locals block)
for i=1,table.getn(RadarDevice) do
    if ((RadarDevice[1]==1) and (RadarDevice[i]==1)) then
        Configure_Chirps(i)
    end
end

-- Frame Configuration
-- Master: trigger = 1 (Software) - you trigger the frame yourself via ar1.StartFrame_mult / GUI "Trigger Frame"
if (0 == ar1.FrameConfig_mult(1, start_chirp_tx, end_chirp_tx, nframes_master, nchirp_loops,
                              Inter_Frame_Interval, trigger_delay, 1)) then
    WriteToLog("Master : Frame Configuration successful\n", "green")
else
    WriteToLog("Master : Frame Configuration failed\n", "red")
end
-- Slaves: trigger = 2 (Hardware sync from master's internal distribution - NOT user-selectable,
-- this is required cascade architecture, independent of the master's software/hardware choice above)
if (0 == ar1.FrameConfig_mult(deviceMapSlaves, start_chirp_tx, end_chirp_tx, nframes_slave, nchirp_loops,
                              Inter_Frame_Interval, trigger_delay, 2)) then
    WriteToLog("Slaves : Frame Configuration successful\n", "green")
else
    WriteToLog("Slaves : Frame Configuration failed\n", "red")
end

---------------------------Capture Configuration (from Cascade_Capture.lua)-------------------------
-- Variable names/values below match TI's shipped Cascade_Capture.lua structure
-- (confirmed against multiple mmwave_studio_03_00_00_14 cascade logs/forum threads).

local timestamp          =   os.date("%Y%m%d_%H%M%S")
capture_directory         =   "calib5s_1orginalmimo_" .. timestamp   -- recommended to change between captures - timestamp does this automatically
n_files_allocation        =   0      -- 0: auto-allocate based on capture size
data_packaging            =   0      -- 0: 16-bit, 1: 12-bit
num_frames_to_capture     =   0      -- 0: default - TDA follows FrameConfig's own frame count (300, set above).
                                      -- CONFIRMED via TDA log: this is the actual 5th argument to the call below
                                      -- ("Sending Number of frames to capture with value of X to TDA..").
                                      -- Do NOT put framing_type here - that was the bug in the last run: framing_type=1
                                      -- landed in this slot, TDA was told to capture exactly 1 frame, and the directory
                                      -- came back empty even though the sensor itself ran the full 10s.
stop_frame_mode           =   0      -- 0: Frame boundary, 2: Sub-frame boundary, 3: Burst boundary, 4: HW/Sub-frame triggered

WriteToLog("Recording basename set to: " .. capture_directory .. "\n", "blue")

if (0 == ar1.TDACaptureCard_StartRecord_mult(deviceMapOverall, n_files_allocation, data_packaging, capture_directory, num_frames_to_capture)) then
    WriteToLog("TDA ARM Successful\n", "green")
else
    WriteToLog("TDA ARM Failed\n", "red")
    return -5
end

RSTD.Sleep(1000)   -- allow TDA ARM to fully settle before triggering

-- Function to start/stop frame (confirmed pattern from TI's Cascade_Capture.lua)
function Framing_Control(Device_ID, En1_Dis0)
    local status = 0
    if (En1_Dis0 == 1) then
        status = ar1.StartFrame_mult(dev_list[Device_ID])   -- Start Trigger Frame
        if (status == 0) then
            WriteToLog("Device "..Device_ID.." : Start Frame Successful\n", "green")
        else
            WriteToLog("Device "..Device_ID.." : Start Frame Failed\n", "red")
            return -5
        end
    else
        status = ar1.StopFrame_mult(dev_list[Device_ID], stop_frame_mode)   -- Stop Trigger Frame
        if (status == 0) then
            WriteToLog("Device "..Device_ID.." : Stop Frame Successful\n", "green")
        else
            WriteToLog("Device "..Device_ID.." : Stop Frame Failed\n", "red")
            return -5
        end
    end
    return 0
end

-- Software-trigger sequence, per TI's documented order for this exact "empty capture" symptom:
-- trigger SLAVES first (arms their wait-for-hardware-sync state), THEN master last
-- (master's software trigger is what actually generates the RF sweep + sync pulse the
-- slaves are waiting for). Triggering only the master, as before, left slaves never
-- armed to listen - TDA ARM/Frame Ended still report success, but nothing gets written.

if (0 == ar1.StartFrame_mult(deviceMapSlaves)) then
    WriteToLog("Slaves : Start Frame (armed, waiting for HW sync) Successful\n", "green")
else
    WriteToLog("Slaves : Start Frame Failed\n", "red")
    return -5
end

RSTD.Sleep(100)   -- brief settle so slaves are confirmed armed before master fires the sync pulse

if (0 == ar1.StartFrame_mult(deviceMapMaster)) then
    WriteToLog("Master : Start Frame (triggers RF + sync pulse) Successful\n", "green")
else
    WriteToLog("Master : Start Frame Failed\n", "red")
    return -5
end

WriteToLog("Capture running: " .. nframes_master .. " frames at " .. Inter_Frame_Interval .. "ms - expect ~"
    .. (nframes_master * Inter_Frame_Interval / 1000) .. "s total\n", "blue")

-- Wait for the full capture duration plus margin - FrameConfig's own frame count (nframes_master)
-- governs when the device stops; TDA capture (num_frames_to_capture=0) follows that automatically.
RSTD.Sleep((nframes_master * Inter_Frame_Interval) + 3000)

WriteToLog("Capture complete. Check capture_directory: " .. capture_directory .. "\n", "blue")

-- If you need to abort early or the auto-stop doesn't trigger, uncomment:
-- ar1.StopFrame_mult(deviceMapOverall, stop_frame_mode)
