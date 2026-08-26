***************************************************************************************
*                   This application showcases the monitoring of                      *
*               AWR2243 ES 1.1 mmWave Cascade chip with TDA2XX board                  *
***************************************************************************************

How to run:
    1. Connect AWR2243 ES 1.1 Cascade board with TDA2XX board.
    2. Connect the PC to the TDA2XX board via Ethernet cable.
    3. Run mmwavelink_monitoring.exe.
    

Execution flow of the application:
    1. Application sets the Master in SOP4 mode.
    2. Downloads the meta image over SPI for Master.
    3. Application sets all the Slave devices in SOP4 mode.
    4. Downloads the meta image over SPI for all the Slaves.
    5. API parameters for all the commands are read from mmwaveconfig.txt.
    6. The example uses the Host based trigger monitoring mode instead of Auto trigger monitoring mode.
    7. Once the device starts framing, the async event reports from the device are logged
       into 4 different files under the "Reports" directory separately for each device.
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
    6. Application by default has the Parallel-SPI feature enabled.
    7. Application by default has all the devices enabled for this example. User can control the devices to be configured via
       DEVICE_MAP_CASCADED macro.

