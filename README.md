# LED-Master

使用STM32CubeIDE制作，注释可能有点乱


ide文件在Led-Master ide.rar


其他的就是AD制作的电路板


芯片引脚本来乱做的，STM32F103C8T6说明手册都没看，后面全部推翻重搞，才把对应功能ADC那些玩意接上


在选择OLED的时候用了SH1106而不是CH1116,搞了半天才明白驱动库都差不多，只要改一下地址就行了，当时让我恼了好久


还有这个DS18B20开始搞了个坏的，到处找驱动库结果都是0.00，换了一个就好了，这个也恼了我2.5天


还有         GPIOA->CRL&=0xf7ffffff;
        	  GPIOA->CRL|=0x08000000;
           来把PA7在GPIO口和TIM3_CH1里切换也挺有意思的

           
时间树那边没把HSI切到HSE和晶振的Debug忘调成Serial Wire把芯片搞得无法写入数据了（笑哭）


另外花时间多的就是网站的搭建和蓝牙串口的适配，其他基本操作代码编写跟着keysking学的





题目：
一、任务
设计一个系统，完成下面的任务。允许使用的单片机（包括核心板）：STM单片机、Arduino系统（包括esp32、esp8266）、51单片机。其他方案自由发挥。
二、要求
每个小问得分取实现部分的最高分。总分200分
1.	作品要求
	提交的作品在洞洞板上焊接搭建（5分）
	提交的作品在PCB上焊接搭建，PCB上要求有个人标志，比如logo，二维码，不用写学号。（15分）
2.	系统供电
	使用成品降压模块，把输入电压+9V降压，为整个系统供电。（5分）
	自制降压电路，把输入电压+9V降压，为整个系统供电。（10分）
	自制降压电路，把输入电压+9V降压，为整个系统供电，且自制电路有防反接功能、工作指示灯和电路开关（15分）
	其他。测评时可以把至多2通道可调直流电源调至需要供电的电压继续测试。（0分）
3.	按键点灯
模式一：灯（1.1）上电时常灭，若按下按键，灯1亮起，若松开按键则熄灭。
模式二：灯（1.2）上电时常亮，若按下按键，灯1熄灭，若松开按键则亮起
模式三：每按下一次按键后，灯（1.3）的亮灭状态颠倒一次。
	完成一个模式（5分）
	使用三个及以下灯完成三个模式，不同模式之间可以手动切换（10分）
4.	闪烁灯
	灯（2）以2s为周期亮灭。（5分）
	灯（2）的亮灭周期可调（10分）
	完成前两条的基础上添加灯（3），灯的亮灭周期可调，且与灯（2）互不影响。（15分）
	使用两种及以上不同方法实现此小问所有功能。（20分）
5.	可变亮度灯
	灯（5）的亮度可调（5分）
	灯（6）做成呼吸灯效果（10分）
	灯（6）做成呼吸灯，且周期可调（15分）
	使用两种及以上不同方法实现此小问所有功能。（20分）
6.	电压指示灯
	使用电位器搭建一个分压电路，输出电压为0-3.3V可调，并预留测试端口（5分）
	若电位器电路部分输出的分压电路大于1.5V，灯（4）亮起，反之熄灭。（10分）
	在前一问的基础上，灯（4）亮灭的阈值可调（15分）
7.	声音检测灯。使用一个声音传感器（允许使用成品模块）
	声音超过一定阈值时，灯（7）亮起，反之熄灭。声音的大小可以用Spectroid.apk标定，且阈值可调（5分）
	灯的亮度随着声音大小变化，声音越大灯亮度越亮，亮度变化明显，并且亮度变化没有明显延迟。同时声音的大小也可以显示在屏幕上（显示大、中、小三档即算完成要求）（10分）
8.	温度检测灯。使用一个温度传感器（允许使用成品模块）
	当温度超过一定阈值时，灯（8）亮起，反之熄灭。此外阈值可调（5分）
	可以把温度显示自制系统自带的屏幕上（10分）
9.	RGB灯
	点亮一个彩色灯，并可以选择红、绿、蓝、白四种颜色（5分）
	点亮一个彩色灯，并可以选择任意颜色（10分）
10.	远程点灯
	使用USB转串口模块，使用串口助手完成所有关于点灯的设置（包括开关、阈值等）（10分）
	使用WIFI或蓝牙完成所有关于点灯的设置（包括开关、阈值等）（15分）
11.	状态显示
	把上述所有点灯状态和设置选项显示在自制系统自带的屏幕上。（10分）
12.	节能灯。上面提到的所有灯可重复利用，使用尽可能少的灯完成所有点灯的要求（包括RGB灯）
	未完成所有关于点灯的要求（0分）
	使用6个及以上灯完成所有关于点灯的要求（5分）
	使用2—5个灯完成所有关于点灯的要求（10分）
	只使用一个灯完成所有关于点灯的要求，第4问闪烁灯两个灯互不干扰还有4.5.中的不同方法可以用一个彩灯的两个不同颜色来完成（20分）


结果整的mode有十二个
制作电路板和完成一系列点灯的要求,1：按下按钮一亮蓝灯 2：按下按钮一蓝灯熄灭  3：按一下调整蓝灯亮灭 4：调整绿色灯闪烁周期  5：调整红色灯闪烁周期 6：调整蓝灯亮度  7：调整蓝色呼吸灯周期 8：调整电压输出与红灯阈值  9：声音监测与呼吸灯亮灭 10：测温与阈值亮起红灯  11：亮起各颜色的小灯 12：查看有无人搞我 13：检测
脉搏 14：查看二维码
