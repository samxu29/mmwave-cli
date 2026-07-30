--[[
Cascade_Configuration_Capture_cascade_baseline.lua

mmWave Studio capture matching radar_configs/cascade_baseline.toml.

Built from Cascade_Configuration_MIMO.lua (TI stock 12-chirp / 4-device MIMO)
plus the TDA arm/start/wait capture block used by the other
Cascade_Configuration_Capture_*.lua scripts. Pair with CLI:
  python3 mimo.py --radar-config cascade_baseline --frames 300 --exp-name cli_cascade_baseline

GEOMETRY (TI stock MIMO - all 4 devices TX + RX):
  chirp 0-2  -> Dev4 TX2/TX1/TX0
  chirp 3-5  -> Dev3 TX2/TX1/TX0
  chirp 6-8  -> Dev2 TX2/TX1/TX0
  chirp 9-11 -> Dev1 TX2/TX1/TX0
  -> 12 chirps, 16 physical RX, deviceMapOverall = 15

RF (matches cascade_baseline.toml / Cascade_Configuration_MIMO.lua):
  1 profile (profileId 0), idle 5 us, ramp 40 us, slope 79 MHz/us,
  256 samples @ 8000 ksps, rxGain 48, numLoops 64, period 100 ms.

DATA RATE: 256 x 12 x 64 x 16 RX x 4 bytes / 0.100 s ~= 126 MB/s. Mid-capture
drops track the radar preset (see mimo.py FRAME DROPS); this TI stock MIMO
schedule is the known-clean CLI reference vs cascade_tx6_rx16_3rps.

DEFAULT CAPTURE LENGTH: 300 frames (~30 s @ 100 ms). Edit
nframes_master/slave below if you want a shorter/longer run.

AFTER THE CAPTURE:
  python3 parse_idx.py --fetch <capture_directory> --frames 300 --period-ms 100

BEFORE RUNNING - edit if needed:
  1. metaImagePath  (F/W path on the Windows Studio PC)
  2. TDA_IPAddress  (default 192.168.33.180)
  3. nframes_master / nframes_slave
--]]

----------------------------------------User Constants--------------------------------------------

dev_list          =    {1, 2, 4, 8}       -- Device map
RadarDevice       =    {1, 1, 1, 1}       -- {dev1, dev2, dev3, dev4}, 1: Enable, 0: Disable
cascade_mode_list =    {1, 2, 2, 2}       -- 0: Single chip, 1: Master, 2: Slave

-- F/W Download Path (edit for your Studio PC)
metaImagePath            =   "C:\\ti\\mmwave_dfp_02_02_02_01\\firmware\\xwr22xx_metaImage.bin"
-- metaImagePath            =   "C:\\ti\\mmwave_dfp_02_02_00_02\\firmware\\xwr22xx_metaImage.bin"

TDA_IPAddress     =   "192.168.33.180"

-- 1=master ; 2=slave1 ; 4=slave2 ; 8=slave3
deviceMapOverall  =   RadarDevice[1] + (RadarDevice[2]*2) + (RadarDevice[3]*4) + (RadarDevice[4]*8)
deviceMapSlaves   =   (RadarDevice[2]*2) + (RadarDevice[3]*4) + (RadarDevice[4]*8)
deviceMapMaster   =   RadarDevice[1]

test_source_enable  =   0      -- 0: Disable, 1: Enable

------------------------------------------- Sensor Configuration ------------------------------------------------

-- Profile configuration (matches radar_configs/cascade_baseline.toml)
local profile_indx              =   0
local start_freq                =   77     -- GHz
local slope                     =   79     -- MHz/us
local idle_time                 =   5      -- us
local adc_start_time            =   6      -- us
local adc_samples               =   256
local sample_freq               =   8000   -- ksps
local ramp_end_time             =   40     -- us
local rx_gain                   =   48     -- dB
local tx0OutPowerBackoffCode    =   0
local tx1OutPowerBackoffCode    =   0
local tx2OutPowerBackoffCode    =   0
local tx0PhaseShifter           =   0
local tx1PhaseShifter           =   0
local tx2PhaseShifter           =   0
local txStartTimeUSec           =   0
local hpfCornerFreq1            =   0      -- 0: 175KHz
local hpfCornerFreq2            =   0      -- 0: 350KHz

-- Frame configuration (matches cascade_baseline.toml; 300 frames for Studio/CLI A/B)
local start_chirp_tx            =   0
local end_chirp_tx              =   11
local nchirp_loops              =   64
local nframes_master            =   300    -- ~30 s @ 100 ms; edit as needed
local nframes_slave             =   300
local Inter_Frame_Interval      =   100    -- ms
local trigger_delay             =   0      -- us
local trig_list                 =   {1,2,2,2} -- 1: Software trigger, 2: Hardware trigger

--[[
Function to configure the chirps specific to a device
12 chirps are configured below, individually for each AWR device

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
-- Note: The syntax for this API is:
-- ar1.ChirpConfig_mult(RadarDeviceId, chirpStartIdx, chirpEndIdx, profileId, startFreqVar, 
--                      freqSlopeVar, idleTimeVar, adcStartTimeVar, tx0Enable, tx1Enable, tx2Enable)

function Configure_Chirps(i) 
    
    if (i == 1) then
            
            -- Chirp 0
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)) then
                WriteToLog("Device "..i.." : Chirp 0 Configuration successful\n", "green")
            else
                WriteToLog("Device "..i.." : Chirp 0 Configuration failed\n", "red")
                return -4
            end
            
            -- Chirp 1
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 1, 1, 0, 0, 0, 0, 0, 0, 0, 0)) then
                WriteToLog("Device "..i.." : Chirp 1 Configuration successful\n", "green")
            else
                WriteToLog("Device "..i.." : Chirp 1 Configuration failed\n", "red")
                return -4
            end
            
            -- Chirp 2
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 2, 2, 0, 0, 0, 0, 0, 0, 0, 0)) then
                WriteToLog("Device "..i.." : Chirp 2 Configuration successful\n", "green")
            else
                WriteToLog("Device "..i.." : Chirp 2 Configuration failed\n", "red")
                return -4
            end
            
            -- Chirp 3
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 3, 3, 0, 0, 0, 0, 0, 0, 0, 0)) then
                WriteToLog("Device "..i.." : Chirp 3 Configuration successful\n", "green")
            else
                WriteToLog("Device "..i.." : Chirp 3 Configuration failed\n", "red")
                return -4
            end
            
            -- Chirp 4
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 4, 4, 0, 0, 0, 0, 0, 0, 0, 0)) then
                WriteToLog("Device "..i.." : Chirp 4 Configuration successful\n", "green")
            else
                WriteToLog("Device "..i.." : Chirp 4 Configuration failed\n", "red")
                return -4
            end
            
            -- Chirp 5
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 5, 5, 0, 0, 0, 0, 0, 0, 0, 0)) then
                WriteToLog("Device "..i.." : Chirp 5 Configuration successful\n", "green")
            else
                WriteToLog("Device "..i.." : Chirp 5 Configuration failed\n", "red")
                return -4
            end
            
            -- Chirp 6
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 6, 6, 0, 0, 0, 0, 0, 0, 0, 0)) then
                WriteToLog("Device "..i.." : Chirp 6 Configuration successful\n", "green")
            else
                WriteToLog("Device "..i.." : Chirp 6 Configuration failed\n", "red")
                return -4
            end
            
            -- Chirp 7
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 7, 7, 0, 0, 0, 0, 0, 0, 0, 0)) then
                WriteToLog("Device "..i.." : Chirp 7 Configuration successful\n", "green")
            else
                WriteToLog("Device "..i.." : Chirp 7 Configuration failed\n", "red")
                return -4
            end
            
            -- Chirp 8
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 8, 8, 0, 0, 0, 0, 0, 0, 0, 0)) then
                WriteToLog("Device "..i.." : Chirp 8 Configuration successful\n", "green")
            else
                WriteToLog("Device "..i.." : Chirp 8 Configuration failed\n", "red")
                return -4
            end
            
            -- Chirp 9
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 9, 9, 0, 0, 0, 0, 0, 0, 0, 1)) then
                WriteToLog("Device "..i.." : Chirp 9 Configuration successful\n", "green")
            else
                WriteToLog("Device "..i.." : Chirp 9 Configuration failed\n", "red")
                return -4
            end
            
            -- Chirp 10
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 10, 10, 0, 0, 0, 0, 0, 0, 1, 0)) then
                WriteToLog("Device "..i.." : Chirp 10 Configuration successful\n", "green")
            else
                WriteToLog("Device "..i.." : Chirp 10 Configuration failed\n", "red")
                return -4
            end
            
            -- Chirp 11
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 11, 11, 0, 0, 0, 0, 0, 1, 0, 0)) then
                WriteToLog("Device "..i.." : Chirp 11 Configuration successful\n", "green")
            else
                WriteToLog("Device "..i.." : Chirp 11 Configuration failed\n", "red")
                return -4
            end
        
    elseif (i == 2) then
    
            -- Chirp 0
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)) then
                WriteToLog("Device "..i.." : Chirp 0 Configuration successful\n", "green")
            else
                WriteToLog("Device "..i.." : Chirp 0 Configuration failed\n", "red")
                return -4
            end
            
            -- Chirp 1
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 1, 1, 0, 0, 0, 0, 0, 0, 0, 0)) then
                WriteToLog("Device "..i.." : Chirp 1 Configuration successful\n", "green")
            else
                WriteToLog("Device "..i.." : Chirp 1 Configuration failed\n", "red")
                return -4
            end
            
            -- Chirp 2
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 2, 2, 0, 0, 0, 0, 0, 0, 0, 0)) then
                WriteToLog("Device "..i.." : Chirp 2 Configuration successful\n", "green")
            else
                WriteToLog("Device "..i.." : Chirp 2 Configuration failed\n", "red")
                return -4
            end
            
            -- Chirp 3
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 3, 3, 0, 0, 0, 0, 0, 0, 0, 0)) then
                WriteToLog("Device "..i.." : Chirp 3 Configuration successful\n", "green")
            else
                WriteToLog("Device "..i.." : Chirp 3 Configuration failed\n", "red")
                return -4
            end
            
            -- Chirp 4
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 4, 4, 0, 0, 0, 0, 0, 0, 0, 0)) then
                WriteToLog("Device "..i.." : Chirp 4 Configuration successful\n", "green")
            else
                WriteToLog("Device "..i.." : Chirp 4 Configuration failed\n", "red")
                return -4
            end
            
            -- Chirp 5
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 5, 5, 0, 0, 0, 0, 0, 0, 0, 0)) then
                WriteToLog("Device "..i.." : Chirp 5 Configuration successful\n", "green")
            else
                WriteToLog("Device "..i.." : Chirp 5 Configuration failed\n", "red")
                return -4
            end
            
            -- Chirp 6
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 6, 6, 0, 0, 0, 0, 0, 0, 0, 1)) then
                WriteToLog("Device "..i.." : Chirp 6 Configuration successful\n", "green")
            else
                WriteToLog("Device "..i.." : Chirp 6 Configuration failed\n", "red")
                return -4
            end
            
            -- Chirp 7
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 7, 7, 0, 0, 0, 0, 0, 0, 1, 0)) then
                WriteToLog("Device "..i.." : Chirp 7 Configuration successful\n", "green")
            else
                WriteToLog("Device "..i.." : Chirp 7 Configuration failed\n", "red")
                return -4
            end
            
            -- Chirp 8
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 8, 8, 0, 0, 0, 0, 0, 1, 0, 0)) then
                WriteToLog("Device "..i.." : Chirp 8 Configuration successful\n", "green")
            else
                WriteToLog("Device "..i.." : Chirp 8 Configuration failed\n", "red")
                return -4
            end
            
            -- Chirp 9
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 9, 9, 0, 0, 0, 0, 0, 0, 0, 0)) then
                WriteToLog("Device "..i.." : Chirp 9 Configuration successful\n", "green")
            else
                WriteToLog("Device "..i.." : Chirp 9 Configuration failed\n", "red")
                return -4
            end
            
            -- Chirp 10
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 10, 10, 0, 0, 0, 0, 0, 0, 0, 0)) then
                WriteToLog("Device "..i.." : Chirp 10 Configuration successful\n", "green")
            else
                WriteToLog("Device "..i.." : Chirp 10 Configuration failed\n", "red")
                return -4
            end
            
            -- Chirp 11
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 11, 11, 0, 0, 0, 0, 0, 0, 0, 0)) then
                WriteToLog("Device "..i.." : Chirp 11 Configuration successful\n", "green")
            else
                WriteToLog("Device "..i.." : Chirp 11 Configuration failed\n", "red")
                return -4
            end
            
    elseif (i == 3) then
    
            -- Chirp 0
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)) then
                WriteToLog("Device "..i.." : Chirp 0 Configuration successful\n", "green")
            else
                WriteToLog("Device "..i.." : Chirp 0 Configuration failed\n", "red")
                return -4
            end
            
            -- Chirp 1
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 1, 1, 0, 0, 0, 0, 0, 0, 0, 0)) then
                WriteToLog("Device "..i.." : Chirp 1 Configuration successful\n", "green")
            else
                WriteToLog("Device "..i.." : Chirp 1 Configuration failed\n", "red")
                return -4
            end
            
            -- Chirp 2
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 2, 2, 0, 0, 0, 0, 0, 0, 0, 0)) then
                WriteToLog("Device "..i.." : Chirp 2 Configuration successful\n", "green")
            else
                WriteToLog("Device "..i.." : Chirp 2 Configuration failed\n", "red")
                return -4
            end
            
            -- Chirp 3
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 3, 3, 0, 0, 0, 0, 0, 0, 0, 1)) then
                WriteToLog("Device "..i.." : Chirp 3 Configuration successful\n", "green")
            else
                WriteToLog("Device "..i.." : Chirp 3 Configuration failed\n", "red")
                return -4
            end
            
            -- Chirp 4
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 4, 4, 0, 0, 0, 0, 0, 0, 1, 0)) then
                WriteToLog("Device "..i.." : Chirp 4 Configuration successful\n", "green")
            else
                WriteToLog("Device "..i.." : Chirp 4 Configuration failed\n", "red")
                return -4
            end
            
            -- Chirp 5
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 5, 5, 0, 0, 0, 0, 0, 1, 0, 0)) then
                WriteToLog("Device "..i.." : Chirp 5 Configuration successful\n", "green")
            else
                WriteToLog("Device "..i.." : Chirp 5 Configuration failed\n", "red")
                return -4
            end
            
            -- Chirp 6
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 6, 6, 0, 0, 0, 0, 0, 0, 0, 0)) then
                WriteToLog("Device "..i.." : Chirp 6 Configuration successful\n", "green")
            else
                WriteToLog("Device "..i.." : Chirp 6 Configuration failed\n", "red")
                return -4
            end
            
            -- Chirp 7
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 7, 7, 0, 0, 0, 0, 0, 0, 0, 0)) then
                WriteToLog("Device "..i.." : Chirp 7 Configuration successful\n", "green")
            else
                WriteToLog("Device "..i.." : Chirp 7 Configuration failed\n", "red")
                return -4
            end
            
            -- Chirp 8
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 8, 8, 0, 0, 0, 0, 0, 0, 0, 0)) then
                WriteToLog("Device "..i.." : Chirp 8 Configuration successful\n", "green")
            else
                WriteToLog("Device "..i.." : Chirp 8 Configuration failed\n", "red")
                return -4
            end
            
            -- Chirp 9
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 9, 9, 0, 0, 0, 0, 0, 0, 0, 0)) then
                WriteToLog("Device "..i.." : Chirp 9 Configuration successful\n", "green")
            else
                WriteToLog("Device "..i.." : Chirp 9 Configuration failed\n", "red")
                return -4
            end
            
            -- Chirp 10
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 10, 10, 0, 0, 0, 0, 0, 0, 0, 0)) then
                WriteToLog("Device "..i.." : Chirp 10 Configuration successful\n", "green")
            else
                WriteToLog("Device "..i.." : Chirp 10 Configuration failed\n", "red")
                return -4
            end
            
            -- Chirp 11
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 11, 11, 0, 0, 0, 0, 0, 0, 0, 0)) then
                WriteToLog("Device "..i.." : Chirp 11 Configuration successful\n", "green")
            else
                WriteToLog("Device "..i.." : Chirp 11 Configuration failed\n", "red")
                return -4
            end
            
    elseif (i == 4) then
    
            -- Chirp 0
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 0, 0, 0, 0, 0, 0, 0, 0, 0, 1)) then
                WriteToLog("Device "..i.." : Chirp 0 Configuration successful\n", "green")
            else
                WriteToLog("Device "..i.." : Chirp 0 Configuration failed\n", "red")
                return -4
            end
            
            -- Chirp 1
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 1, 1, 0, 0, 0, 0, 0, 0, 1, 0)) then
                WriteToLog("Device "..i.." : Chirp 1 Configuration successful\n", "green")
            else
                WriteToLog("Device "..i.." : Chirp 1 Configuration failed\n", "red")
                return -4
            end
            
            -- Chirp 2
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 2, 2, 0, 0, 0, 0, 0, 1, 0, 0)) then
                WriteToLog("Device "..i.." : Chirp 2 Configuration successful\n", "green")
            else
                WriteToLog("Device "..i.." : Chirp 2 Configuration failed\n", "red")
                return -4
            end
            
            -- Chirp 3
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 3, 3, 0, 0, 0, 0, 0, 0, 0, 0)) then
                WriteToLog("Device "..i.." : Chirp 3 Configuration successful\n", "green")
            else
                WriteToLog("Device "..i.." : Chirp 3 Configuration failed\n", "red")
                return -4
            end
            
            -- Chirp 4
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 4, 4, 0, 0, 0, 0, 0, 0, 0, 0)) then
                WriteToLog("Device "..i.." : Chirp 4 Configuration successful\n", "green")
            else
                WriteToLog("Device "..i.." : Chirp 4 Configuration failed\n", "red")
                return -4
            end
            
            -- Chirp 5
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 5, 5, 0, 0, 0, 0, 0, 0, 0, 0)) then
                WriteToLog("Device "..i.." : Chirp 5 Configuration successful\n", "green")
            else
                WriteToLog("Device "..i.." : Chirp 5 Configuration failed\n", "red")
                return -4
            end
            
            -- Chirp 6
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 6, 6, 0, 0, 0, 0, 0, 0, 0, 0)) then
                WriteToLog("Device "..i.." : Chirp 6 Configuration successful\n", "green")
            else
                WriteToLog("Device "..i.." : Chirp 6 Configuration failed\n", "red")
                return -4
            end
            
            -- Chirp 7
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 7, 7, 0, 0, 0, 0, 0, 0, 0, 0)) then
                WriteToLog("Device "..i.." : Chirp 7 Configuration successful\n", "green")
            else
                WriteToLog("Device "..i.." : Chirp 7 Configuration failed\n", "red")
                return -4
            end
            
            -- Chirp 8
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 8, 8, 0, 0, 0, 0, 0, 0, 0, 0)) then
                WriteToLog("Device "..i.." : Chirp 8 Configuration successful\n", "green")
            else
                WriteToLog("Device "..i.." : Chirp 8 Configuration failed\n", "red")
                return -4
            end
            
            -- Chirp 9
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 9, 9, 0, 0, 0, 0, 0, 0, 0, 0)) then
                WriteToLog("Device "..i.." : Chirp 9 Configuration successful\n", "green")
            else
                WriteToLog("Device "..i.." : Chirp 9 Configuration failed\n", "red")
                return -4
            end
            
            -- Chirp 10
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 10, 10, 0, 0, 0, 0, 0, 0, 0, 0)) then
                WriteToLog("Device "..i.." : Chirp 10 Configuration successful\n", "green")
            else
                WriteToLog("Device "..i.." : Chirp 10 Configuration failed\n", "red")
                return -4
            end
            
            -- Chirp 11
            if (0 == ar1.ChirpConfig_mult(dev_list[i], 11, 11, 0, 0, 0, 0, 0, 0, 0, 0)) then
                WriteToLog("Device "..i.." : Chirp 11 Configuration successful\n", "green")
            else
                WriteToLog("Device "..i.." : Chirp 11 Configuration failed\n", "red")
                return -4
            end
        
    end

end
 
------------------------------ API Configuration ------------------------------------------------
    
-- 1. Connection to TDA. 2. Selecting Cascade/Single Chip.  3. Selecting 2-chip/4-chip

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
          
-- Including this depends on the type of board being used.
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

-- Edit this API to enable/disable the boot time calibration. Enabled by default.
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

----------------------------Test Source Configuration------------------------------
-- This is useful for initial bringup.
-- Each device is configured with a test object at a different location.
    
if(test_source_enable == 1) then
    
    if(RadarDevice[1] == 1) then
        -- Object at 5 m with x = 4m and y = 3m
        if (0 == ar1.SetTestSource_mult(1, 4, 3, 0, 0, 0, 0, -327, 0, -327, 327, 327, 327, -2.5, 327, 327, 0, 
                 0, 0, 0, -327, 0, -327, 327, 327, 327, -95, 0, 0, 0.5, 0, 1, 0, 1.5, 0, 0, 0, 0, 0, 0, 0, 0)) then
            WriteToLog("Device 1 : Test Source Configuration Successful\n", "green")
        else
            WriteToLog("Device 1 : Test Source Configuration failed\n", "red")
            return -3
        end
    end
    
    if(RadarDevice[2] == 1) then        
        -- Object at 5 m with x = 3m and y = 4m
        if (0 == ar1.SetTestSource_mult(2, 3, 4, 0, 0, 0, 0, -327, 0, -327, 327, 327, 327, -2.5, 327, 327, 0, 
                 0, 0, 0, -327, 0, -327, 327, 327, 327, -95, 0, 0, 0.5, 0, 1, 0, 1.5, 0, 0, 0, 0, 0, 0, 0, 0)) then
            WriteToLog("Device 2 : Test Source Configuration Successful\n", "green")
        else
            WriteToLog("Device 2 : Test Source Configuration failed\n", "red")
            return -3
        end
    end
    
    if(RadarDevice[3] == 1) then         
        -- Object at 13 m with x = 12m and y = 5m
        if (0 == ar1.SetTestSource_mult(4, 12, 5, 0, 0, 0, 0, -327, 0, -327, 327, 327, 327, -2.5, 327, 327, 0, 
                 0, 0, 0, -327, 0, -327, 327, 327, 327, -95, 0, 0, 0.5, 0, 1, 0, 1.5, 0, 0, 0, 0, 0, 0, 0, 0)) then
            WriteToLog("Device 3 : Test Source Configuration Successful\n", "green")
        else
            WriteToLog("Device 3 : Test Source Configuration failed\n", "red")
            return -3
        end
    end
    
    if(RadarDevice[4] == 1) then        
        -- Object at 13 m with x = 5m and y = 12m
        if (0 == ar1.SetTestSource_mult(8, 5, 12, 0, 0, 0, 0, -327, 0, -327, 327, 327, 327, -2.5, 327, 327, 0, 
                 0, 0, 0, -327, 0, -327, 327, 327, 327, -95, 0, 0, 0.5, 0, 1, 0, 1.5, 0, 0, 0, 0, 0, 0, 0, 0)) then
            WriteToLog("Device 4 : Test Source Configuration Successful\n", "green")
        else
            WriteToLog("Device 4 : Test Source Configuration failed\n", "red")
            return -3
        end
    end
       
end           

---------------------------Sensor Configuration-------------------------

-- Profile Configuration
if (0 == ar1.ProfileConfig_mult(deviceMapOverall, 0, start_freq, idle_time, adc_start_time, ramp_end_time, 
                                0, 0, 0, 0, 0, 0, slope, 0, adc_samples, sample_freq, 0, 0, rx_gain)) then
    WriteToLog("Profile Configuration successful\n", "green")
else
    WriteToLog("Profile Configuration failed\n", "red")
    return -4
end

-- Chirp Configuration 
for i=1,table.getn(RadarDevice) do    
    if ((RadarDevice[1]==1) and (RadarDevice[i]==1)) then
        Configure_Chirps(i)                
    end
end

-- Enabling/ Disabling Test Source
if(test_source_enable == 1) then
    ar1.EnableTestSource_mult(deviceMapOverall, 1)
    WriteToLog("Enabling Test Source Configuration successful\n", "green")
end

-- Frame Configuration
-- Master
if (0 == ar1.FrameConfig_mult(1,start_chirp_tx,end_chirp_tx,nframes_master, nchirp_loops,
                              Inter_Frame_Interval, trigger_delay, 1)) then
    WriteToLog("Master : Frame Configuration successful\n", "green")
else
    WriteToLog("Master : Frame Configuration failed\n", "red")
end
-- Slaves
if (0 == ar1.FrameConfig_mult(deviceMapSlaves,start_chirp_tx,end_chirp_tx,nframes_slave, nchirp_loops,
                              Inter_Frame_Interval, trigger_delay, 2)) then
    WriteToLog("Slaves : Frame Configuration successful\n", "green")
else
    WriteToLog("Slaves : Frame Configuration failed\n", "red")
end

---------------------------Capture Configuration-------------------------
-- Match CLI arming where it matters for the drop A/B:
--   n_files_allocation = 1  (CLI pre-allocates; Studio used to leave this 0)
--   data_packaging     = 0  (16-bit)
--   num_frames_to_capture = 0  (TDA follows FrameConfig's nframes)

local timestamp           =   os.date("%Y%m%d_%H%M%S")
capture_directory         =   "studio_cascade_baseline_" .. timestamp
n_files_allocation        =   1
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