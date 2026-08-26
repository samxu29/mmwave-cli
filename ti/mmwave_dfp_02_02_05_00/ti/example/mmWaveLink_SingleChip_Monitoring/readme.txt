***************************************************************************************
  This application showcases the monitoring features of AWR2243 ES 1.0/ES1.1 mmWave 
   device and mmWaveLink APIs usage on External Host environment for the same.         
***************************************************************************************

How to run:
    1. Connect AWR2243 ES 1.0/ES 1.1 boosterpack and DCA1000 EVM to PC.
    2. Erase sFlash before running this application.
    3. Run mmwavelink_monitoring.exe.
    

Execution flow of the application:
    1. Application sets the device in SOP4 mode.
    2. Downloads the meta image over SPI.
    3. API parameters for all the commands are read from mmwaveconfig.txt.
    4. The example by-default uses the Host based trigger monitoring mode instead of the Auto trigger monitoring mode.
    5. Once the device starts framing, the async event reports from the device are logged
       into 4 different files under the "Reports" directory.
       a. MonitoringReport.txt -> This report logs all other async events coming from the device specifically the analog and the digital monitors.
       b. CalibrationReport.txt -> This report logs all the async events related to the calibrations.
       c. MSSEvents.txt -> This report logs all the async events related to the MSS.
       d. BSSEvents.txt -> This report logs all the async events related to the BSS.
    
Note:
    1. To modify and re-run the application, use Visual Studio based project provided in the same directory.
    2. "trace.txt" file is created which logs all the SPI communication commands.
    3. Advanced frame feature is based on global tag 'gLinkAdvanceFrameTest'.
    4. "CalibrationData.txt" file is created which stores the calibration data. When Calibration restore is issued,
       it makes use of the data present in this file.
    5. "PhShiftCalibrationData.txt" file is created which stores the phase shifter calibration data. 
       When phase shifter Calibration restore is issued, it makes use of the data present in this file.
