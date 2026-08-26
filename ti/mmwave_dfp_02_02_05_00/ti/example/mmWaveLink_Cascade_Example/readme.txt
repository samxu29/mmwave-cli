***************************************************************************************
*       This application showcases the configuration and ADC data capture of          *
*               AWR2243 ES 1.1 mmWave Cascade chip with TDA2XX board                  *
***************************************************************************************
 
How to run:
    1. Connect AWR2243 ES 1.1 Cascade board with TDA2XX board.
    2. Connect the PC to the TDA2XX board via Ethernet cable.
    3. Run mmwavelink_example.exe.
    
Execution flow of the application:
    1. Application sets the Master in SOP4 mode.
    2. Downloads the meta image over SPI for Master.
    3. Application sets all the Slave devices in SOP4 mode.
    4. Downloads the meta image over SPI for all the Slaves.
    5. API parameters for all the commands are read from mmwaveconfig.txt.
    6. During framing, the data is captured in the TDA2XX SSD under the folder "MMWL_Capture" (under /mnt/ssd/ directory).
    7. Once the capture is complete, user can retrieve the data from the SSD using WinSCP application.

Note:
    1. To modify and re-run the application, use Visual Studio based project provided in the same directory.
    2. "trace.txt" file is created which logs all the SPI communication commands.
    3. Application by default has all the devices enabled for this example. User can control the devices to be configured via
       CascadeDeviceMap field in mmwaveconfig.txt.
    4. "CalibrationData.txt" file is created which stores the calibration data. When Calibration restore is issued,
       it makes use of the data present in this file.
    5. "PhShiftCalibrationData.txt" file is created which stores the phase shifter calibration data. 
       When phase shifter Calibration restore is issued, it makes use of the data present in this file.
    6. "AdvChirpLUTData.txt" file is created which stores the locally programmed LUT data that is sent to RadarSS
       to populate the LUT at the device end.
