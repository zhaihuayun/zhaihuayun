欢迎使用BurnToolCLI命令行工具!
此工具功能主要是：烧写镜像到Flash上。
支持Windows和linux操作系统，JRE已集成无需安装。

1.烧写镜像功能说明
步骤如下：
(1)在命令行下，进入burntoolcli所在目录；
(2)输入命令烧写
本地串口：
                                                 <chipName>     <mode>      <comport>   <baud>       <chipType>               [burnFilePath]             [logFileParentPath]
   .\jre\bin\java -jar burntoolcli.jar --burn  -n chipName  -m  serial        COM1      115200   -t    0x200       -b     D:\image\Allinone.bin    -lp       D:\log        -dt   20000
串口服务器：
                                                 <chipName>     <mode>      <serialServerIp>  <serialServerPort>       <chipType>               [burnFilePath]             [logFileParentPath]
   .\jre\bin\java -jar burntoolcli.jar --burn  -n chipName  -m  serial   -s       x.x.x.x        10003           -t       0x200       -b     D:\image\Allinone.bin    -lp       D:\log

   chipName              芯片名
   mode                  传输方式
   comport               串口号
   baud                  串口烧写的波特率
   chipType              芯片类型
   burnFilePath          烧写文件路径
   logFileParentPath     创建log文件夹的父路径
   serialServerIp        串口服务器的ip
   serialServerPort      串口服务器的端口号

(3)出现如下打印：SerialPort has been connented, Please power off, then power on the device.If it doesn't work, please try to repower on. 请手动上电，开始烧写。

FQA：
烧写失败返回错误码说明：
错误码 1：烧写的命令参数错误
错误码 2：导入文件失败，例如导入config文件，xml文件，program文件失败
错误码 3：打开串口/I2C设备失败
错误码 4：烧写fastboot失败
错误码 5：烧写分区文件失败
错误码 6：制作Nand烧片器镜像失败
错误码 7：制作Emmc烧片器镜像失败
错误码 8：空指针异常
错误码 9: 打开socket失败
错误码 10: 擦除失败
错误码 11: 接收数据超时
错误码 12: 发生io异常
错误码 13: 发生运行异常