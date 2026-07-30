***************************************************************************************
* This application showcases the feature of SPI to sFlash image download on a         *
*                 AWR2243 ES 1.0/ES1.1 mmWave device.                                 *
***************************************************************************************

Note:
    This application supports both SPI and I2C mode of operation.
    Refer to mmwaveconfig.txt on choosing the mode of operation.
 
How to run:
    1. Connect AWR2243 ES 1.0/ES 1.1 boosterpack and DCA1000 EVM to PC.
    2. Erase sFlash before running this application.
    3. Run mmwavelink_sflash_example.exe.
    
Execution flow of the application:
    1. Application sets the device in SOP4 mode.
    2. Downloads the meta image to sFlash over SPI.
    3. API parameters for all the commands are read from mmwaveconfig.txt.

Note:
    1. To modify and re-run the application, use Visual Studio based project provided in the same directory.
    2. "trace.txt" file is created which logs all the SPI communication commands.
    3. Once the flashing operation is successful, it is not necessary to download the meta image in the subsequent power cycles.
       In order to skip the firmware download for the subsequent iterations, make EnableFwDownload=1 in mmwaveconfig.txt.    
    4. "CalibrationData.txt" file is created which stores the calibration data. When Calibration restore is issued,
       it makes use of the data present in this file.
    5. "PhShiftCalibrationData.txt" file is created which stores the phase shifter calibration data. 
       When phase shifter Calibration restore is issued, it makes use of the data present in this file.
    6. "AdvChirpLUTData.txt" file is created which stores the locally programmed LUT data that is sent to RadarSS
       to populate the LUT at the device end.
