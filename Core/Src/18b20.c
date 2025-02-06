#include "18b20.h"

/****************************************************************************
函数名：delay_us
功能：微秒级延时
输入：延时数据
输出：无
返回值：无
备注：
****************************************************************************/
void delay_us(uint32_t time) {
    time *= 10;
    while (time) time--;
}

/****************************************************************************
函数名：DS18B20_IO_IN
功能：使DS18B20_DQ引脚变为输入模式
输入：无
输出：无
返回值：无
备注：DQ引脚为PB5
****************************************************************************/
void DS18B20_IO_IN(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Pin = GPIO_PIN_5;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/****************************************************************************
函数名：DS18B20_IO_OUT
功能：使DS18B20_DQ引脚变为推挽输出模式
输入：无
输出：无
返回值：无
备注：DQ引脚为PB5
****************************************************************************/
void DS18B20_IO_OUT(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Pin = GPIO_PIN_5;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/***************************************************************************
函数名：DS18B20_Rst
功能：发送复位信号
输入: 无
输出：无
返回值：无
备注：
***************************************************************************/
void DS18B20_Rst(void) {
    DS18B20_IO_OUT(); // 引脚输出模式

    // 拉低总线并延时750us
    DS18B20_DQ_OUT_LOW;
    delay_us(750);

    // 释放总线为高电平并延时等待15~60us
    DS18B20_DQ_OUT_HIGH;
    delay_us(15);
}

/***************************************************************************
函数名：DS18B20_Check
功能：检测DS18B20返回的存在脉冲
输入: 无
输出：无
返回值：0:成功  1：失败   2:释放总线失败
备注：
***************************************************************************/
uint8_t DS18B20_Check(void) {
    uint8_t retry = 0; // 定义一个脉冲持续时间
    DS18B20_IO_IN();   // 引脚设为输入模式

    while (DS18B20_DQ_IN && retry < 200) {
        retry++;
        delay_us(1);
    }

    if (retry >= 200) return 1; // 超时失败
    retry = 0;

    // 判断DS18B20是否释放总线
    while (!DS18B20_DQ_IN && retry < 240) {
        retry++;
        delay_us(1);
    }

    if (retry >= 240) return 2; // 释放总线失败
    return 0; // 成功
}

/***************************************************************************
函数名：DS18B20_Write_Byte
功能：向DS18B20写一个字节
输入: 要写入的字节
输出：无
返回值：无
备注：
***************************************************************************/
void DS18B20_Write_Byte(uint8_t data) {
    uint8_t j, databit;
    DS18B20_IO_OUT();

    for (j = 1; j <= 8; j++) {
        databit = data & 0x01; // 取数据最低位
        data = data >> 1;      // 右移一位

        if (databit) { // 当前位写1
            DS18B20_DQ_OUT_LOW;
            delay_us(2);
            DS18B20_DQ_OUT_HIGH;
            delay_us(60);
        } else { // 当前位写0
            DS18B20_DQ_OUT_LOW;
            delay_us(60);
            DS18B20_DQ_OUT_HIGH;
            delay_us(2);
        }
    }
}

/***************************************************************************
函数名：DS18B20_Read_Bit
功能：向DS18B20读一个位
输入: 无
输出：无
返回值：读入数据
备注：
***************************************************************************/
uint8_t DS18B20_Read_Bit(void) {
    uint8_t data;
    DS18B20_IO_OUT();

    DS18B20_DQ_OUT_LOW;
    delay_us(2);
    DS18B20_DQ_OUT_HIGH;
    DS18B20_IO_IN();
    delay_us(12);

    if (DS18B20_DQ_IN) data = 1;
    else data = 0;

    delay_us(50);
    return data;
}

/***************************************************************************
函数名：DS18B20_Read_Byte
功能：向DS18B20读一个字节
输入: 无
输出：无
返回值：读入数据
备注：
***************************************************************************/
uint8_t DS18B20_Read_Byte(void) {
    uint8_t i, j, data = 0;

    for (i = 1; i <= 8; i++) {
        j = DS18B20_Read_Bit();
        data = (j << 7) | (data >> 1); // 将1/0写入最高位
    }
    return data;
}

/***************************************************************************
函数名：DS18B20_Start
功能：DS18B20开启
输入: 无
输出：无
返回值：无
备注：
***************************************************************************/
void DS18B20_Start(void) {
    DS18B20_Rst();
    DS18B20_Check();
    DS18B20_Write_Byte(0xCC); // 跳过ROM
    DS18B20_Write_Byte(0x44); // 温度变换命令
}

/***************************************************************************
函数名：DS18B20_Init
功能：DS18B20初始化
输入: 无
输出：无
返回值：无
备注：
***************************************************************************/
uint8_t DS18B20_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Pin = GPIO_PIN_5;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

    DS18B20_Rst();
    return DS18B20_Check();
}

/***************************************************************************
函数名：DS18B20_Read_Temperature
功能：读取一次温度
输入: 无
输出：无
返回值：读取到的温度数据
备注：适用于总线上只有一个DS18B20的情况
***************************************************************************/
short DS18B20_Get_Temperature() {
    uint8_t TL, TH;
    short temperature;

    DS18B20_Start();
    DS18B20_Rst();
    if (DS18B20_Check() != 0) {
        printf("DS18B20 Check Failed!\n");
        return -1000; // 返回一个错误值
    }
    DS18B20_Write_Byte(0xCC); // 跳过ROM
    DS18B20_Write_Byte(0xBE); // 读暂存器

    TL = DS18B20_Read_Byte(); // 低八位
    TH = DS18B20_Read_Byte(); // 高八位

    // 打印原始数据
    printf("TH: %02X, TL: %02X\n", TH, TL);

    // 判断温度值是否为负数
    if (TH > 0x70) {
        TH = ~TH;
        TL = ~TL;
        temperature = -(TH << 8 | TL) * 0.0625;
    } else {
        temperature = (TH << 8 | TL) * 0.0625;
    }

    return temperature;
}
