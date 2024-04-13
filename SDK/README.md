Solarec 1.0.0.6
# 目录整体结构<a name="ZH-CN_TOPIC_0000001324634060"></a>

**表 1**  目录整体结构说明

|文件夹名|描述|
|--|--|
|application|应用sample目录，存放各级sample和客户主程序入口。|
|board|板级特性目录，存放板级的软件特性。|
|build|编译构建目录，存放编译相关的脚本和配置信息。|
|chip|支持芯片目录，存放当前SDK支持的芯片系列的代码。|
|drivers|驱动目录，存放当前SDK支持的IP驱动。|
|generatecode|IDE自动生成代码相关目录。|
|middleware|中间件目录，存放当前SDK版本支持的中间件。|
|document|文档目录，存放用户使用指导手册。|


# application<a name="ZH-CN_TOPIC_0000001159757668"></a>

**表 2**  application目录结构说明

|文件夹名|描述|
|--|--|
|board_sample|存放板级抽象sample代码。|
|drivers_sample|存放驱动sample代码。|
|middleware_sample|存放中间件sample代码。|
|user|main.c 存放路径。|







## board\_sample<a name="ZH-CN_TOPIC_0000001375993845"></a>

**表 3**  board\_sample目录结构说明

|文件夹名|描述|
|--|--|
|dimming|呼吸灯功能示例。|
|key|按键检查功能示例。|
|led|数码管功能示例。|
|pulses|gpio发送pwm波功能示例。|
|softserial|gpio实现串口通信功能示例。|


## drivers\_sample<a name="ZH-CN_TOPIC_0000001341348849"></a>

























### acmp<a name="ZH-CN_TOPIC_0000001290560712"></a>

**表 4**  acmp目录结构说明

|文件夹名|描述|
|--|--|
|sample_acmp|比较器使用示例。|


### adc<a name="ZH-CN_TOPIC_0000001341668617"></a>

**表 5**  adc目录结构说明

|文件夹名|描述|
|--|--|
|sample_adc_associative_trigger_of_apt|APT触发ADC。|
|sample_adc_continue_trigger|ADC连续采样。|
|sample_adc_over_sample|ADC过采样。|
|sample_adc_single_trigger|ADC单次采样。|
|sample_adc_single_trigger_dma|ADC单次采样带DMA。|
|sample_adc_single_trigger_it|ADC单次采样带中断。|
|sample_adc_sync_sample|ADC同步采样。|
|sample_adc_sync_sample_dma|ADC同步采样带DMA。|
|sample_adc_sync_sample_it|ADC同步采样带中断。|


### apt<a name="ZH-CN_TOPIC_0000001290840460"></a>

**表 6**  apt目录结构说明

|文件夹名|描述|
|--|--|
|sample_apt_single_resistor|APT单电阻采样示例，仅在U相触发ADC采样信号。|
|sample_apt_three_resistor|APT三电阻采样示例，在U、V和W相都触发ADC采样信号。|


### can<a name="ZH-CN_TOPIC_0000001389612044"></a>

**表 7**  can目录结构说明

|文件夹名|描述|
|--|--|
|sample_can_send_receive|CAN发送和接收数据示例。|


### capm<a name="ZH-CN_TOPIC_0000001291040088"></a>

**表 8**  capm目录结构说明

|文件夹名|描述|
|--|--|
|capm_hall_sample|CAPM读取霍尔传感器值示例。|


### cfd<a name="ZH-CN_TOPIC_0000001289188524"></a>

**表 9**  cfd目录结构说明

|文件夹名|描述|
|--|--|
|sample_cfd_check_error|cfd注入错误前后监测目标时钟异常功能。|


### cmm<a name="ZH-CN_TOPIC_0000001341548701"></a>

**表 10**  cmm目录结构说明

|文件夹名|描述|
|--|--|
|sample_cmm_check_error|cmm注入错误前后监测目标时钟异常功能。|


### crc<a name="ZH-CN_TOPIC_0000001289028884"></a>

**表 11**  crc目录结构说明

|文件夹名|描述|
|--|--|
|sample_crc_check|测试CRC不同算法和输入有效位宽，生成并校验crc值。|
|sample_crc_gen|计算并生成CRC数值。|
|sample_crc_load|通过load初始值将xmodem算法改为ccit-false算法并校验结果。|


### dac<a name="ZH-CN_TOPIC_0000001343119853"></a>

**表 12**  dac目录结构说明

|文件夹名|描述|
|--|--|
|sample_dac|DAC电压输出到管脚示例。|


### dma<a name="ZH-CN_TOPIC_0000001341468201"></a>

**表 13**  dma目录结构说明

|文件夹名|描述|
|--|--|
|sample_dma_list_transfer|DMA链式传输。|
|sample_dma_list_transfer_continue|DMA链式传输实现连续功能。|
|sample_dma_mem_to_mem|DMA内存到内存传输。|
|sample_dma_mem_to_per|DMA内存到外设传输。|
|sample_dma_per_to_mem|DMA外设到内存传输。|
|sample_dma_per_to_per|DMA外设到外设传输。|


### flash<a name="ZH-CN_TOPIC_0000001288708996"></a>

**表 14**  flash目录结构说明

|文件夹名|描述|
|--|--|
|sample_flash_blocking|阻塞模式操作flash。|
|sample_flash_interrupt|中断方式操作flash。|


### gpio<a name="ZH-CN_TOPIC_0000001288708992"></a>

**表 15**  gpio目录结构说明

|文件夹名|描述|
|--|--|
|sample_gpio_circle|GPIO环回测试电平和方向属性。|
|sample_gpio_interrupt|测试GPIO不同中断类型。|
|sample_gpio_key|GPIO用作按键功能。|
|sample_gpio_led|GPIO周期控制led亮灭功能。|


### gpt<a name="ZH-CN_TOPIC_0000001341548693"></a>

**表 16**  gpt目录结构说明

|文件夹名|描述|
|--|--|
|sample_gpt_simplerun|gpt产生PWM波形。|


### i2c<a name="ZH-CN_TOPIC_0000001341668613"></a>

**表 17**  i2c目录结构说明

|文件夹名|描述|
|--|--|
|sample_i2c_blocking_stlm75|使用阻塞的方式读写温度传感器。|
|sample_i2c_interrupt_stlm75|使用中断的方式读写温度传感器。|
|sample_i2c_dma_stlm75|使用dma方式读写温度传感器。|


### iocmg<a name="ZH-CN_TOPIC_0000001426670030"></a>

**表 18**  iocmg目录结构说明

|文件夹名|描述|
|--|--|
|iolist_sample|iocmg初始化管脚列表的属性配置功能。|


### pga<a name="ZH-CN_TOPIC_0000001343000405"></a>

**表 19**  pga目录结构说明

|文件夹名|描述|
|--|--|
|sample_pga|PGA内部电阻放大示例。|
|sample_pga_extra_resistor|PGA外部电阻放大示例。|


### pmc<a name="ZH-CN_TOPIC_0000001290988348"></a>

**表 20**  pmc目录结构说明

|文件夹名|描述|
|--|--|
|sample_pmc_pvd|PMC掉电检测示例。|
|sample_pmc_wakeup|PMC定时器唤醒示例。|


### qdm<a name="ZH-CN_TOPIC_0000001343320549"></a>

**表 21**  qdm目录结构说明

|文件夹名|描述|
|--|--|
|sample_qdm_m|QDM使用M法读取电机转速的示例。|
|sample_qdm_mt|QDM使用MT法读取电机转速的示例。|


### spi<a name="ZH-CN_TOPIC_0000001341468197"></a>

**表 22**  spi目录机构说明

|文件夹名|描述|
|--|--|
|sample_spi_blocking_kta7953|使用阻塞方式读写ADC。|
|sample_spi_dma_kta7953|使用dma方式读写ADC。|
|sample_spi_interrupt_kta7953|使用中断方式读写ADC。|
|sample_spi_microwire_master|演示如何使用microwire master。|
|sample_spi_microwire_slave|演示如何使用microwire slave。|
|sample_spi_slave|演示如何使用motorola spi slaver。|


### timer<a name="ZH-CN_TOPIC_0000001289188520"></a>

**表 23**  timer目录结构说明

|文件夹名|描述|
|--|--|
|sample_timer_interrupt|timer定时触发中断，执行用户串口打印。|


### tsensor<a name="ZH-CN_TOPIC_0000001289937942"></a>

**表 24**  tsensor目录结构说明

|文件夹名|描述|
|--|--|
|sample_tsensor|tsensor对器件结温采样。|


### uart<a name="ZH-CN_TOPIC_0000001341348853"></a>

**表 25**  uart目录结构说明

|文件夹名|描述|
|--|--|
|sample_uart_blocking_rx|UART阻塞接收。|
|sample_uart_blocking_tx|UART阻塞发送。|
|sample_uart_dma_rx|UART带DMA接收。|
|sample_uart_dma_tx|UART带DMA发送。|
|sample_uart_interrupt_tx_after_rx|UART中断接收数据之后，再中断发送此数据。|
|sample_uart_interrupt_rx|UART中断接收。|
|sample_uart_interrupt_tx|UART中断发送。|
|sample_uart_dma_tx_dma_rx_simultaneously|UART全双工模式，DMA同时发送和接收。|
|sample_uart_dma_tx_int_rx_simultaneously|UART全双工模式，DMA发送的同时，中断接收。|
|sample_uart_int_tx_dma_rx_simultaneously|UART全双工模式，中断发送的同时，DMA接收。|
|sample_uart_int_tx_int_rx_simultaneously|UART全双工模式，中断发送的同时，中断接收。|
|sample_uart_dma_rx_cyclically_stored|UART使用DMA循环搬运数据到指定内存。|
|sample_uart_single_wire_communication|UART单线通信示例。|


### wdg<a name="ZH-CN_TOPIC_0000001288868916"></a>

**表 26**  wdg目录机构说明

|文件夹名|描述|
|--|--|
|sample_wdg_reset|测试wdg不喂狗复位功能。|
|sample_iwdg_reset|测试iwdg不喂狗复位功能。|


## middleware\_sample<a name="ZH-CN_TOPIC_0000001375634057"></a>

**表 27**  middleware\_sample目录机构说明

|文件夹名|描述|
|--|--|
|mcs_65ldemo|电机控制算法在AD101HDMA_VER.B板的示例。|
|mcs_65demo|电机控制算法在AD105HDMA_VER.B板的示例。|
|pmsm_sensorless_1shunt_foc|永磁同步电机单电阻采样无感FOC应用。|
|pmsm_sensorless_2shunt_foc|永磁同步电机双电阻采样无感FOC应用。|


## user<a name="ZH-CN_TOPIC_0000001324953936"></a>

客户主程序的入口文件存放在此目录下。

# board<a name="ZH-CN_TOPIC_0000001375834373"></a>

**表 28**  board目录结构说明

|文件夹名|描述|
|--|--|
|dimming|使用GPT和TIMER实现呼吸灯功能。|
|key|按键检查功能。|
|led|数码管功能。|
|pulses|使用gpio发送pwm波功能。|
|softserial|使用gpio实现串口通信功能。|


# build<a name="ZH-CN_TOPIC_0000001204999115"></a>

**表 29**  build目录

|文件夹名|文件名|描述|
|--|--|--|
|__pycache__|-|执行python脚本产生的缓存文件。|
|config|-|指定编译工具链和工具链对应的默认编译参数。|
|toolchain|-|编译工具链配置模板。|
|-|.gn|内容为指定BUILDCONFIG.gn路径，以及指定根构建目标BUILD.gn的路径。|
|-|BUILD.gn|由build_gn.py脚本自动生成，此文件为根构建目标文件。|
|-|build.py|完成编译命令解析、编译环境检测、执行编译等工作。|
|-|build_check.py|编译前合法性检查脚本。|
|-|build_gn.py|解析userconfig.json的编译内容，完成自动构建编译脚本的工作。|
|-|config.ini|保存编译工具链路径和gn、ninja编译命令的调用。|
|-|ide_entry.py|从SDK分离生成工程的脚本。|
|-|packet_create.py|打包生成all-in-one（loader.bin + target.bin）文件的脚本。|


**表 30**  config目录

|文件夹名|文件名|描述|
|--|--|--|
|hcc|-|存放hcc编译器默认编译选项和最终编译选项。|
|hcc_fpu|-|存放hcc浮点编译器默认编译选项和最终编译选项。|
|-|BUILDCONFIG.gn|内容为指定需要使用的编译工具链。|


**表 31**  toolchain目录

|文件夹名|文件名|描述|
|--|--|--|
|-|BUILD.gn|使用config.gni中的工具链配置模板配置编译工具链命令。|
|-|config.gni|内容为工具链的编译命令模板。|


# chip<a name="ZH-CN_TOPIC_0000001205277643"></a>

**表 32**  chip目录结构说明

|文件夹名|文件夹名/文件名|描述|
|--|--|--|
|3065h/3061h|-|存放与芯片配置相关的代码文件，比如IP版本、特性宏、中断和时钟等。|
|chipinit/|存放芯片早期初始化代码，在客户程序运行前执行。|
|fotp/|存放fotp区域的读接口。|
|ip_crg/|与芯片耦合的IP时钟配置。|
|iomap/|与芯片相关的IO管脚复选功能定义。|
|baseaddr.h|芯片的寄存器地址信息。|
|chipinc.h|芯片头文集合，客户包含此头文件即可使用全部芯片信息。|
|codecopy.json|非IDE环境下分离工程信息。|
|flash.lds|芯片分区信息。|
|info.h|芯片cycle信息。|
|interrupt_ip.h|芯片中断号分配信息。|
|ioconfig.h|芯片管脚复用寄存器信息。|
|locktype.h|芯片预定义的锁信息，暂无使用。|
|startup.S|程序初始化代码。|
|sysctrl.h|系统控制寄存器信息。|
|systick.h|系统systick信息。|
|target|userconfig.json|存放默认的编译参数。|


# document<a name="ZH-CN_TOPIC_0000001515244654"></a>

**表 33**  documet目录结构说明

|文件夹名|描述|
|--|--|
|hardware|存放硬件生态板的用户手册。|
|software|存放Solrec软件用户手册。|
|tools|存放IDE、调试器等工具用户手册。|


# drivers<a name="ZH-CN_TOPIC_0000001205117667"></a>

**表 34**  drivers目录结构说明

|文件夹名|描述|
|--|--|
|acmp|存放比较器驱动代码。|
|adc|存放adc驱动代码。|
|apt|存放apt驱动代码。|
|base|存放基础枚举定义和通用函数接口。|
|can|存放can驱动代码。|
|capm|存放capm驱动代码。|
|cfd|存放cfd驱动代码。|
|cmm|存放cmm驱动代码。|
|crc|存放crc驱动代码。|
|crg|存放crg驱动代码。|
|dac|存放dac驱动代码。|
|debug|存放调试相关代码。|
|dma|存放dma驱动代码。|
|flash|存放flash驱动代码。|
|gpio|存放gpio驱动代码。|
|gpt|存放gpt驱动代码。|
|i2c|存放i2c驱动代码。|
|pga|存放pga驱动代码。|
|pmc|存放pmc驱动代码。|
|qdm|存放qdm驱动代码。|
|spi|存放spi驱动代码。|
|timer|存放定时器驱动代码。|
|tsensor|存放tsensor驱动代码。|
|uart|存放串口驱动代码。|
|wdg|存放看门狗驱动代码。|


>**说明：** 
>模块目录下的v0/v1/...后缀目录代表不同的模块IP，各IP不具有升级关系。




























## acmp<a name="ZH-CN_TOPIC_0000001343200261"></a>

**表 35**  acmp目录结构说明

|文件夹名|描述|
|--|--|
|common|存放与IP无关的acmp模块头文件。|
|acmp_v0|存放与IP相关的acmp模块头文件和源码文件。|


## adc<a name="ZH-CN_TOPIC_0000001290065524"></a>

**表 36**  adc目录结构说明

|文件夹名|描述|
|--|--|
|common|存放与IP无关的adc模块头文件。|
|adc_v0|存放与IP相关的adc模块头文件和源码文件。|


## apt<a name="ZH-CN_TOPIC_0000001290720620"></a>

**表 37**  apt目录结构说明

|文件夹名|描述|
|--|--|
|common|存放与IP无关的，apt模块头文件。|
|apt_v0|存放与IP相关的，apt模块头文件和源码文件。|


## base<a name="ZH-CN_TOPIC_0000001159439130"></a>

**表 38**  base目录结构说明

|文件夹名|描述|
|--|--|
|common|存放base模块的头文件信息。|
|base_v0|存放base基础定义和通用功能函数以及中断接口。|


## can<a name="ZH-CN_TOPIC_0000001376683229"></a>

**表 39**  can目录结构说明

|文件夹名|描述|
|--|--|
|common|存放与IP无关的，can模块头文件。|
|can_v0|存放与IP相关的，can模块头文件和源码文件。|


## capm<a name="ZH-CN_TOPIC_0000001290840464"></a>

**表 40**  capm目录结构说明

|文件夹名|描述|
|--|--|
|common|存放与IP无关的，capm模块头文件。|
|capm_v0|存放与IP相关的，capm模块头文件和源码文件。|


## cfd<a name="ZH-CN_TOPIC_0000001291750040"></a>

**表 41**  cfd目录结构说明

|文件夹名|描述|
|--|--|
|common|存放与IP无关的，cfd模块头文件。|
|cfd_v0|存放与IP相关的，cfd模块头文件和源码文件。|


## cmm<a name="ZH-CN_TOPIC_0000001344670165"></a>

**表 42**  cmm目录结构说明

|文件夹名|描述|
|--|--|
|common|存放与IP无关的，cmm模块头文件。|
|cmm_v0|存放与IP相关的，cmm模块头文件和源码文件。|


## crc<a name="ZH-CN_TOPIC_0000001291590396"></a>

**表 43**  crc目录结构说明

|文件夹名|描述|
|--|--|
|common|存放与IP无关的，crc模块头文件。|
|crc_v0|存放与IP相关的，crc模块头文件和源码文件。|


## crg<a name="ZH-CN_TOPIC_0000001342672721"></a>

**表 44**  crg目录结构说明

|文件夹名|描述|
|--|--|
|common|存放与IP无关的crg模块头文件。|
|crg_v0|存放与IP相关的crg模块头文件和源码文件。|


## dac<a name="ZH-CN_TOPIC_0000001291040092"></a>

**表 45**  dac目录结构说明

|文件夹名|描述|
|--|--|
|common|存放与IP无关的，dac模块头文件。|
|dac_v0|存放与IP相关的，dac模块头文件和源码文件。|


## debug<a name="ZH-CN_TOPIC_0000001159917622"></a>

**表 46**  debug目录结构说明

|文件夹名|描述|
|--|--|
|inc|调试模块头文件。|
|src|调试模块源代码。|


## dma<a name="ZH-CN_TOPIC_0000001290386184"></a>

**表 47**  dma目录结构说明

|文件夹名|描述|
|--|--|
|common|存放与IP无关的dma模块头文件。|
|dma_v0|存放与IP相关的串口模块头文件和源码文件。|


## flash<a name="ZH-CN_TOPIC_0000001341459477"></a>

**表 48**  flash目录结构说明

|文件夹名|描述|
|--|--|
|common|存放与IP无关的，flash模块头文件。|
|flash_v0|存放与IP相关的，flash模块头文件和源码文件。|


## gpio<a name="ZH-CN_TOPIC_0000001159757670"></a>

**表 49**  gpio目录结构说明

|文件夹名|描述|
|--|--|
|common|存放与IP无关的，gpio模块头文件。|
|gpio_v0|存放与IP相关的，gpio模块头文件和源码文件。|


## gpt<a name="ZH-CN_TOPIC_0000001290073060"></a>

**表 50**  gpt目录结构说明

|文件夹名|描述|
|--|--|
|common|存放与IP无关的gpt模块头文件。|
|gpt_v0|存放与IP相关的gpt模块头文件和源码文件。|


## i2c<a name="ZH-CN_TOPIC_0000001204999117"></a>

**表 51**  i2c目录结构说明

|文件夹名|描述|
|--|--|
|common|存放与IP无关的i2c模块头文件。|
|i2c_v0|存放与IP相关的i2c模块头文件和源码文件。|


## iocmg<a name="ZH-CN_TOPIC_0000001476429749"></a>

**表 52**  iocmg目录结构说明

|文件夹名|描述|
|--|--|
|common|存放与IP无关的，iocmg模块头文件。|
|iocmg_v0|存放与IP相关的，iocmg模块头文件和源码文件。|


## pga<a name="ZH-CN_TOPIC_0000001343320553"></a>

**表 53**  pga目录结构说明

|文件夹名|描述|
|--|--|
|common|存放与IP无关的，pga模块头文件。|
|pga_v0|存放与IP相关的，pga模块头文件和源码文件。|


## pmc<a name="ZH-CN_TOPIC_0000001343627749"></a>

**表 54**  pmc目录结构说明

|文件夹名|描述|
|--|--|
|common|存放与IP无关的，pmc模块头文件。|
|pmc_v0|存放与IP相关的，pmc模块头文件和源码文件。|


## qdm<a name="ZH-CN_TOPIC_0000001290560716"></a>

**表 55**  qdm目录结构说明

|文件夹名|描述|
|--|--|
|common|存放与IP无关的，qdm模块头文件。|
|qdm_v0|存放与IP相关的，qdm模块头文件和源码文件。|


## spi<a name="ZH-CN_TOPIC_0000001205277645"></a>

**表 56**  spi目录结构说明

|文件夹名|描述|
|--|--|
|common|存放与IP无关的spi模块头文件。|
|spi_v0|存放与IP相关的spi模块头文件和源码文件。|


## timer<a name="ZH-CN_TOPIC_0000001159599142"></a>

**表 57**  timer目录结构说明

|文件夹名|描述|
|--|--|
|common|存放与IP无关的timer模块头文件。|
|timer_v0|存放与IP相关的timer模块头文件和源码文件。|


## tsensor<a name="ZH-CN_TOPIC_0000001342666281"></a>

**表 58**  tsensor目录结构说明

|文件夹名|描述|
|--|--|
|common|存放与IP无关的tsensor模块头文件。|
|tsensor_v0|存放与IP相关的tsensor模块头文件和源码文件。|


## uart<a name="ZH-CN_TOPIC_0000001205117669"></a>

**表 59**  uart目录结构说明

|文件夹名|描述|
|--|--|
|common|存放与IP无关的uart模块头文件。|
|uart_v0|存放与IP相关的uart模块头文件和源码文件。|


## wdg<a name="ZH-CN_TOPIC_0000001344629761"></a>

**表 60**  wdg目录结构说明

|文件夹名|描述|
|--|--|
|common|存放与IP无关的，看门狗模块头文件。|
|wdg_v0|存放与IP相关的，看门狗模块头文件和源码文件。|


# generatecode<a name="ZH-CN_TOPIC_0000001375634061"></a>

存放IDE根据图形化配置界面自动生成系统初始化代码的初始文件。

**表 61**  generatecode目录结构说明

|文件名|描述文件|
|--|--|
|feature.h|宏定义文件，定义了参数检查、打印开关等宏。|
|main.h|main.c的头文件。|
|system_init.c|驱动初始化代码。|


# middleware<a name="ZH-CN_TOPIC_0000001159439132"></a>

**表 62**  middleware目录结构说明

|文件夹名|描述文件|
|--|--|
|thirdparty|第三方中间件。|
|control_library|电机控制FOC算法，此算法做为电机功能验证使用，不做商用质量保证。如需商用请充分验证。|


# bundle.json<a name="ZH-CN_TOPIC_0000001344758118"></a>

**表 63**  bundle.json的文件说明

|文件名|描述文件|
|--|--|
|bundle.json|IDE编译组件的编译命令配置文件。|


# 缩略语<a name="ZH-CN_TOPIC_0000001376842437"></a>

|缩略语|英文|中文|
|--|--|--|
|ADC|Analog-to-digital converter|模拟数字转换器|
|APT|Advance Pluse timer|高级脉冲定时器|
|ACMP|Analog Comparator|模拟比较器|
|BGA|Ball Grid Array|球栅阵列|
|BASE|Base|基础定义|
|CRG|Clock Reset and Generator|时钟复位生成器|
|CAN|Controller Area Network|控制器局域网|
|CRC|Cyclic Redundancy Check|循环冗余校验|
|CAPM|Capture Module|捕捉器|
|CFD|Clock Failure Detector|时钟失效检测|
|CMM|Clock Monitor Module|时钟频率监测|
|CVE|Common Vulnerabilities and Exposures system|常见漏洞和暴露系统|
|DAC|Digital-to-analog converter|数字模拟转换器|
|DMA|Direct Memory Access|内存直接访问|
|GPIO|General Purpose Input/Output|通用目的输入输出接口|
|GPT|General Pluse timer|通用脉冲定时器|
|I2C master|Inter Integrated-Circuit Master|I2C主机|
|I2C Slave|Inter Integrated-Circuit Slave|I2C从机|
|PMU|Power Manager Unit|电源管理单元|
|PWM|Pulse Width Modulation|脉冲宽度调制|
|PGA|Programmable Gain Amplifier|可编程增益放大器|
|PMP|Physical Memory Protection|物理内存保护|
|QDM|Quadrature Decoder Module|正交解码器|
|ROM|Read-Only Memory|只读存储器|
|SPI|Serial Peripheral Interface|串行外设接口|
|UART|Universal Asynchronous Receiver/Transmitter|通用异步接收发送设备|
|WDG|Watch Dog timer|看门狗|