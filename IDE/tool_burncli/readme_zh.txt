��ӭʹ��BurnToolCLI�����й���!
�˹��߹�����Ҫ�ǣ���д����Flash�ϡ�
֧��Windows��linux����ϵͳ��JRE�Ѽ������谲װ��

1.��д������˵��
�������£�
(1)���������£�����burntoolcli����Ŀ¼��
(2)����������д
���ش��ڣ�
                                                 <chipName>     <mode>      <comport>   <baud>       <chipType>               [burnFilePath]             [logFileParentPath]
   .\jre\bin\java -jar burntoolcli.jar --burn  -n chipName  -m  serial        COM1      115200   -t    0x200       -b     D:\image\Allinone.bin    -lp       D:\log        -dt   20000
���ڷ�������
                                                 <chipName>     <mode>      <serialServerIp>  <serialServerPort>       <chipType>               [burnFilePath]             [logFileParentPath]
   .\jre\bin\java -jar burntoolcli.jar --burn  -n chipName  -m  serial   -s       x.x.x.x        10003           -t       0x200       -b     D:\image\Allinone.bin    -lp       D:\log

   chipName              оƬ��
   mode                  ���䷽ʽ
   comport               ���ں�
   baud                  ������д�Ĳ�����
   chipType              оƬ����
   burnFilePath          ��д�ļ�·��
   logFileParentPath     ����log�ļ��еĸ�·��
   serialServerIp        ���ڷ�������ip
   serialServerPort      ���ڷ������Ķ˿ں�

(3)�������´�ӡ��SerialPort has been connented, Please power off, then power on the device.If it doesn't work, please try to repower on. ���ֶ��ϵ磬��ʼ��д��

FQA��
��дʧ�ܷ��ش�����˵����
������ 1����д�������������
������ 2�������ļ�ʧ�ܣ����絼��config�ļ���xml�ļ���program�ļ�ʧ��
������ 3���򿪴���/I2C�豸ʧ��
������ 4����дfastbootʧ��
������ 5����д�����ļ�ʧ��
������ 6������Nand��Ƭ������ʧ��
������ 7������Emmc��Ƭ������ʧ��
������ 8����ָ���쳣
������ 9: ��socketʧ��
������ 10: ����ʧ��
������ 11: �������ݳ�ʱ
������ 12: ����io�쳣
������ 13: ���������쳣