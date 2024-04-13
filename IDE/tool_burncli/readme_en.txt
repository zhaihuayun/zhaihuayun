Welcome to use the BurnToolCLI command line tool!
The main function of this tool is to burn images to the flash.
This tool applies to the Windows OS and Linux OS. The JRE is integrated and therefore does not need to be installed.

1. Description About the Function of Burning Images
The procedures are as follows:

(1) Enter the directory where BurnToolCLI is located at the command line.
(2) Enter the command to burn images according to the XML file.
Local serial port:
                                                 <chipName>     <mode>      <comport>   <baud>       <chipType>               [burnFilePath]             [logFileParentPath]
   .\jre\bin\java -jar burntoolcli.jar --burn  -n chipName  -m  serial        COM1      115200   -t    0x200       -b     D:\image\Allinone.bin    -lp       D:\log
Serial port server:
                                                 <chipName>     <mode>      <serialServerIp>  <serialServerPort>       <chipType>               [burnFilePath]             [logFileParentPath]
   .\jre\bin\java -jar burntoolcli.jar --burn  -n chipName  -m  serial   -s       x.x.x.x        10003           -t       0x200       -b     D:\image\Allinone.bin    -lp       D:\log

   chipName              Chip name
   mode                  Transmission mode
   comport               Serial port number
   baud                  Baud rate for serial port burning
   chipType              Chip type
   burnFilePath          Burning File Path
   logFileParentPath     Create the parent path of the log folder
   serialServerIp        IP address of the serial port server
   serialServerPort      Port number of the serial port server

(5) The following information is displayed: SerialPort has been connected. Please power off, then power on the device. If it doesn't work, please try to power on again. Then manually power on the tool and start burning.

FQA
Description of the error codes returned when the burning fails:
Error code  1: The command parameters are incorrect.
Error code  2: Failed to import the file. For example, the config file, XML file, or program file fails to be imported.
Error code  3: Failed to enable the serial port or I2C device.
Error code  4: Failed to burn the fastboot.
Error code  5: Failed to burn the partition file.
Error code  6: Failed to create the NAND burner image.
Error code  7: Failed to create the eMMC burner image.
Error code  8: null pointer exception
Error code  9: open socket failed
Error code  10: erase failed
Error code  11: Data receiving timed out
Error code  12: An I/O exception occurs
Error code  13: A running exception occurred.