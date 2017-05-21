/********************************************
龙丘MC9S12XS128多功能开发板 
Designed by Chiu Sir
E-mail:chiusir@yahoo.cn
软件版本:V1.1
最后更新:2011年9月18日
相关信息参考下列地址：
网站：  http://www.lqist.cn
淘宝店：http://shop36265907.taobao.com
------------------------------------
Code Warrior 5.0/1
Target : MC9S12XS128
Crystal: 16.000Mhz
busclock:16.000MHz
pllclock:32.000MHz    
 
使用说明：
OLED电源使用3.3V。   
----------------
G    电源地
3.3V 接3.3V电源
D0   PORTA_PA14  
D1   PORTA_PA15
RST  PORTA_PA16 
DC   PORTA_PA17
CS   已接地，不用接
============================================
OLED电源使用5V。   
----------------
G    电源地
3.3V 接5V电源，电源跟模块之间串接100欧姆电阻，并加3.3V钳位二极管
D0   PORTA_PA14 单片机跟模块之间串接1k-3.3k电阻 
D1   PORTA_PA15 单片机跟模块之间串接1k-3.3k电阻 
RST  PORTA_PA16 单片机跟模块之间串接1k-3.3k电阻 
DC   PORTA_PA17 单片机跟模块之间串接1k-3.3k电阻 
CS   已接地，不用接     
============================================     
如果用户使用的是5V单片机，请看用户手册，切勿烧毁模块！  
============================================*/
#include "includes.h"
#include "LQ12864.h"

//GPIOA_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(14));	//IO口输出低电平，亮
//GPIOA_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(14));	//IO口输出高电平，灭
//GPIOA_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(15));	//IO口输出低电平，亮
//GPIOA_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(15));	//IO口输出高电平，灭
//GPIOA_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(16));	//IO口输出低电平，亮
//GPIOA_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(16));	//IO口输出高电平，灭
//GPIOA_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(17));	//IO口输出低电平，亮
//GPIOA_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(17));	//IO口输出高电平，灭

#define XLevelL		0x00
#define XLevelH		0x10
#define XLevel		((XLevelH&0x0F)*16+XLevelL)
#define Max_Column	128
#define Max_Row		  64
#define	Brightness	0xCF 

/*
4线SPI使用说明：
VBT 供内部DC-DC电压，3.3~4.3V，如果使用5V电压，为保险起见串一个100~500欧的电阻
VCC 供内部逻辑电压 1.8~6V   
GND 地

BS0 低电平
BS1 低电平
BS2 低电平

CS  片选管脚
DC  命令数据选择管脚
RES 模块复位管脚 
D0（SCLK） ，时钟脚，由MCU控制
D1（MOSI） ，主输出从输入数据脚，由MCU控制

D2 悬空      
D3-D7 ， 低电平 ， 也可悬空，但最好设为低电平
RD  低电平 ，也可悬空，但最好设为低电平
RW  低电平 ，也可悬空，但最好设为低电平
RD  低电平 ，也可悬空，但最好设为低电平   
*/

#define X_WIDTH 128
#define Y_WIDTH 64
//======================================
const byte F6x8[][6] =
{
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   // sp
    { 0x00, 0x00, 0x00, 0x2f, 0x00, 0x00 },   // !
    { 0x00, 0x00, 0x07, 0x00, 0x07, 0x00 },   // "
    { 0x00, 0x14, 0x7f, 0x14, 0x7f, 0x14 },   // #
    { 0x00, 0x24, 0x2a, 0x7f, 0x2a, 0x12 },   // $
    { 0x00, 0x62, 0x64, 0x08, 0x13, 0x23 },   // %
    { 0x00, 0x36, 0x49, 0x55, 0x22, 0x50 },   // &
    { 0x00, 0x00, 0x05, 0x03, 0x00, 0x00 },   // '
    { 0x00, 0x00, 0x1c, 0x22, 0x41, 0x00 },   // (
    { 0x00, 0x00, 0x41, 0x22, 0x1c, 0x00 },   // )
    { 0x00, 0x14, 0x08, 0x3E, 0x08, 0x14 },   // *
    { 0x00, 0x08, 0x08, 0x3E, 0x08, 0x08 },   // +
    { 0x00, 0x00, 0x00, 0xA0, 0x60, 0x00 },   // ,
    { 0x00, 0x08, 0x08, 0x08, 0x08, 0x08 },   // -
    { 0x00, 0x00, 0x60, 0x60, 0x00, 0x00 },   // .
    { 0x00, 0x20, 0x10, 0x08, 0x04, 0x02 },   // /
    { 0x00, 0x3E, 0x51, 0x49, 0x45, 0x3E },   // 0
    { 0x00, 0x00, 0x42, 0x7F, 0x40, 0x00 },   // 1
    { 0x00, 0x42, 0x61, 0x51, 0x49, 0x46 },   // 2
    { 0x00, 0x21, 0x41, 0x45, 0x4B, 0x31 },   // 3
    { 0x00, 0x18, 0x14, 0x12, 0x7F, 0x10 },   // 4
    { 0x00, 0x27, 0x45, 0x45, 0x45, 0x39 },   // 5
    { 0x00, 0x3C, 0x4A, 0x49, 0x49, 0x30 },   // 6
    { 0x00, 0x01, 0x71, 0x09, 0x05, 0x03 },   // 7
    { 0x00, 0x36, 0x49, 0x49, 0x49, 0x36 },   // 8
    { 0x00, 0x06, 0x49, 0x49, 0x29, 0x1E },   // 9
    { 0x00, 0x00, 0x36, 0x36, 0x00, 0x00 },   // :
    { 0x00, 0x00, 0x56, 0x36, 0x00, 0x00 },   // ;
    { 0x00, 0x08, 0x14, 0x22, 0x41, 0x00 },   // <
    { 0x00, 0x14, 0x14, 0x14, 0x14, 0x14 },   // =
    { 0x00, 0x00, 0x41, 0x22, 0x14, 0x08 },   // >
    { 0x00, 0x02, 0x01, 0x51, 0x09, 0x06 },   // ?
    { 0x00, 0x32, 0x49, 0x59, 0x51, 0x3E },   // @
    { 0x00, 0x7C, 0x12, 0x11, 0x12, 0x7C },   // A
    { 0x00, 0x7F, 0x49, 0x49, 0x49, 0x36 },   // B
    { 0x00, 0x3E, 0x41, 0x41, 0x41, 0x22 },   // C
    { 0x00, 0x7F, 0x41, 0x41, 0x22, 0x1C },   // D
    { 0x00, 0x7F, 0x49, 0x49, 0x49, 0x41 },   // E
    { 0x00, 0x7F, 0x09, 0x09, 0x09, 0x01 },   // F
    { 0x00, 0x3E, 0x41, 0x49, 0x49, 0x7A },   // G
    { 0x00, 0x7F, 0x08, 0x08, 0x08, 0x7F },   // H
    { 0x00, 0x00, 0x41, 0x7F, 0x41, 0x00 },   // I
    { 0x00, 0x20, 0x40, 0x41, 0x3F, 0x01 },   // J
    { 0x00, 0x7F, 0x08, 0x14, 0x22, 0x41 },   // K
    { 0x00, 0x7F, 0x40, 0x40, 0x40, 0x40 },   // L
    { 0x00, 0x7F, 0x02, 0x0C, 0x02, 0x7F },   // M
    { 0x00, 0x7F, 0x04, 0x08, 0x10, 0x7F },   // N
    { 0x00, 0x3E, 0x41, 0x41, 0x41, 0x3E },   // O
    { 0x00, 0x7F, 0x09, 0x09, 0x09, 0x06 },   // P
    { 0x00, 0x3E, 0x41, 0x51, 0x21, 0x5E },   // Q
    { 0x00, 0x7F, 0x09, 0x19, 0x29, 0x46 },   // R
    { 0x00, 0x46, 0x49, 0x49, 0x49, 0x31 },   // S
    { 0x00, 0x01, 0x01, 0x7F, 0x01, 0x01 },   // T
    { 0x00, 0x3F, 0x40, 0x40, 0x40, 0x3F },   // U
    { 0x00, 0x1F, 0x20, 0x40, 0x20, 0x1F },   // V
    { 0x00, 0x3F, 0x40, 0x38, 0x40, 0x3F },   // W
    { 0x00, 0x63, 0x14, 0x08, 0x14, 0x63 },   // X
    { 0x00, 0x07, 0x08, 0x70, 0x08, 0x07 },   // Y
    { 0x00, 0x61, 0x51, 0x49, 0x45, 0x43 },   // Z
    { 0x00, 0x00, 0x7F, 0x41, 0x41, 0x00 },   // [
    { 0x00, 0x55, 0x2A, 0x55, 0x2A, 0x55 },   // 55
    { 0x00, 0x00, 0x41, 0x41, 0x7F, 0x00 },   // ]
    { 0x00, 0x04, 0x02, 0x01, 0x02, 0x04 },   // ^
    { 0x00, 0x40, 0x40, 0x40, 0x40, 0x40 },   // _
    { 0x00, 0x00, 0x01, 0x02, 0x04, 0x00 },   // '
    { 0x00, 0x20, 0x54, 0x54, 0x54, 0x78 },   // a
    { 0x00, 0x7F, 0x48, 0x44, 0x44, 0x38 },   // b
    { 0x00, 0x38, 0x44, 0x44, 0x44, 0x20 },   // c
    { 0x00, 0x38, 0x44, 0x44, 0x48, 0x7F },   // d
    { 0x00, 0x38, 0x54, 0x54, 0x54, 0x18 },   // e
    { 0x00, 0x08, 0x7E, 0x09, 0x01, 0x02 },   // f
    { 0x00, 0x18, 0xA4, 0xA4, 0xA4, 0x7C },   // g
    { 0x00, 0x7F, 0x08, 0x04, 0x04, 0x78 },   // h
    { 0x00, 0x00, 0x44, 0x7D, 0x40, 0x00 },   // i
    { 0x00, 0x40, 0x80, 0x84, 0x7D, 0x00 },   // j
    { 0x00, 0x7F, 0x10, 0x28, 0x44, 0x00 },   // k
    { 0x00, 0x00, 0x41, 0x7F, 0x40, 0x00 },   // l
    { 0x00, 0x7C, 0x04, 0x18, 0x04, 0x78 },   // m
    { 0x00, 0x7C, 0x08, 0x04, 0x04, 0x78 },   // n
    { 0x00, 0x38, 0x44, 0x44, 0x44, 0x38 },   // o
    { 0x00, 0xFC, 0x24, 0x24, 0x24, 0x18 },   // p
    { 0x00, 0x18, 0x24, 0x24, 0x18, 0xFC },   // q
    { 0x00, 0x7C, 0x08, 0x04, 0x04, 0x08 },   // r
    { 0x00, 0x48, 0x54, 0x54, 0x54, 0x20 },   // s
    { 0x00, 0x04, 0x3F, 0x44, 0x40, 0x20 },   // t
    { 0x00, 0x3C, 0x40, 0x40, 0x20, 0x7C },   // u
    { 0x00, 0x1C, 0x20, 0x40, 0x20, 0x1C },   // v
    { 0x00, 0x3C, 0x40, 0x30, 0x40, 0x3C },   // w
    { 0x00, 0x44, 0x28, 0x10, 0x28, 0x44 },   // x
    { 0x00, 0x1C, 0xA0, 0xA0, 0xA0, 0x7C },   // y
    { 0x00, 0x44, 0x64, 0x54, 0x4C, 0x44 },   // z
    { 0x14, 0x14, 0x14, 0x14, 0x14, 0x14 }    // horiz lines
};
const byte F14x16_Idx[] = 
{
	"北京龙丘邱科技开发板智能车首选液晶室温度按键次电压"
};
const byte F14x16[] = {  
0x20,0x20,0x20,0x20,0xFF,0x00,0x00,0x00,0xFF,0x40,0x20,0x30,0x18,0x10,
0x30,0x18,0x08,0x04,0x7F,0x00,0x00,0x00,0x3F,0x40,0x40,0x40,0x40,0x78,//北0
0x04,0x04,0xE4,0x24,0x24,0x25,0x26,0x24,0x24,0x24,0xE4,0x06,0x04,0x00,
0x20,0x10,0x19,0x0D,0x41,0x81,0x7F,0x01,0x01,0x05,0x0D,0x38,0x10,0x00,//京1
0x10,0x10,0x10,0x10,0x10,0xFF,0x10,0xF0,0x12,0x1C,0xD0,0x10,0x10,0x10,
0x40,0x20,0x10,0x0C,0x03,0x10,0x08,0x3F,0x42,0x41,0x40,0x40,0x40,0x70,//龙2
0x00,0x00,0xFE,0x84,0x84,0x84,0x84,0x82,0x82,0x82,0x82,0xC0,0x80,0x00,
0x20,0x20,0x3F,0x20,0x20,0x20,0x20,0x20,0x3F,0x20,0x20,0x20,0x30,0x20,//丘3
0x00,0xFC,0x24,0x24,0xE2,0x22,0x22,0x00,0xFE,0x02,0x22,0x52,0x8E,0x02,
0x10,0x1F,0x10,0x08,0x0F,0x08,0x08,0x00,0xFF,0x00,0x08,0x08,0x10,0x0F,//邱4
0x12,0x92,0x72,0xFE,0x51,0x91,0x00,0x22,0xCC,0x00,0x00,0xFF,0x00,0x00,
0x02,0x01,0x00,0xFF,0x00,0x04,0x04,0x04,0x02,0x02,0x02,0xFF,0x01,0x01,//科5
0x08,0x88,0xFF,0x48,0x28,0x00,0xC8,0x48,0x48,0x7F,0x48,0xC8,0x48,0x08,
0x41,0x80,0x7F,0x00,0x40,0x40,0x20,0x13,0x0C,0x0C,0x12,0x21,0x60,0x20,//技6
0x42,0x42,0x42,0x42,0xFE,0x42,0x42,0x42,0x42,0xFE,0x42,0x42,0x42,0x42,
0x40,0x20,0x10,0x0C,0x03,0x00,0x00,0x00,0x00,0x7F,0x00,0x00,0x00,0x00,//开7
0x10,0x3E,0x10,0x10,0xF0,0x9F,0x90,0x90,0x92,0x94,0x1C,0x10,0x10,0x10,
0x20,0x10,0x88,0x87,0x41,0x46,0x28,0x10,0x28,0x27,0x40,0xC0,0x40,0x00,//发8
0x10,0xD0,0xFF,0x50,0x90,0x00,0xFE,0x62,0xA2,0x22,0x21,0xA1,0x61,0x00,
0x03,0x00,0x7F,0x00,0x11,0x0E,0x41,0x20,0x11,0x0A,0x0E,0x31,0x60,0x20,//板9
0x14,0x13,0x92,0x7E,0x32,0x52,0x92,0x00,0x7C,0x44,0x44,0x44,0x7C,0x00,
0x01,0x01,0x00,0xFF,0x49,0x49,0x49,0x49,0x49,0x49,0xFF,0x00,0x00,0x00,//智10
0xB8,0x97,0x92,0x90,0x94,0xB8,0x10,0x00,0x7F,0x48,0x48,0x44,0x74,0x20,
0xFF,0x0A,0x0A,0x4A,0x8A,0x7F,0x00,0x00,0x3F,0x44,0x44,0x42,0x72,0x20,//能11
0x04,0x84,0xC4,0xA4,0x9C,0x87,0x84,0xF4,0x84,0x84,0x84,0x84,0x84,0x00,
0x04,0x04,0x04,0x04,0x04,0x04,0x04,0xFF,0x04,0x04,0x04,0x04,0x04,0x04,//车12
0x04,0x04,0xE4,0x25,0x26,0x34,0x2C,0x24,0x24,0x26,0xE5,0x04,0x04,0x04,
0x00,0x00,0x7F,0x25,0x25,0x25,0x25,0x25,0x25,0x25,0x7F,0x00,0x00,0x00,//首13
0x40,0x42,0xCC,0x00,0x50,0x4E,0xC8,0x48,0x7F,0xC8,0x48,0x48,0x40,0x00,
0x40,0x20,0x1F,0x20,0x48,0x46,0x41,0x40,0x40,0x47,0x48,0x48,0x4E,0x40,//选14
0x61,0x06,0xE0,0x18,0x84,0xE4,0x1C,0x84,0x65,0xBE,0x24,0xA4,0x64,0x04,
0x04,0xFF,0x00,0x01,0x00,0xFF,0x41,0x21,0x12,0x0C,0x1B,0x61,0xC0,0x40,//液15
0x00,0x00,0x00,0x7E,0x2A,0x2A,0x2A,0x2A,0x2A,0x2A,0x7E,0x00,0x00,0x00,
0x7F,0x25,0x25,0x25,0x25,0x7F,0x00,0x00,0x7F,0x25,0x25,0x25,0x25,0x7F,//晶16
0x10,0x2C,0x24,0xA4,0x64,0x25,0x26,0x24,0x24,0xA4,0x24,0x34,0x2C,0x04,
0x40,0x48,0x49,0x49,0x49,0x49,0x7F,0x49,0x49,0x49,0x4B,0x48,0x40,0x40,//室17
0x21,0x86,0x70,0x00,0x7E,0x4A,0x4A,0x4A,0x4A,0x4A,0x7E,0x00,0x00,0x00,
0xFE,0x01,0x40,0x7F,0x41,0x41,0x7F,0x41,0x41,0x7F,0x41,0x41,0x7F,0x40,//温18
0x00,0xFC,0x04,0x24,0x24,0xFC,0xA5,0xA6,0xA4,0xFC,0x24,0x24,0x24,0x04,
0x60,0x1F,0x80,0x80,0x42,0x46,0x2A,0x12,0x12,0x2A,0x26,0x42,0xC0,0x40,//度19
0x10,0x10,0xFF,0x90,0x50,0x98,0x88,0x88,0xE9,0x8E,0x88,0x88,0x98,0x88,
0x42,0x81,0x7F,0x00,0x40,0x40,0x26,0x25,0x18,0x08,0x16,0x31,0x60,0x20,//按20
0x30,0xEF,0x28,0x28,0x44,0x64,0xDC,0x10,0x54,0xFF,0x54,0x54,0x7C,0x10,
0x01,0x7F,0x21,0x51,0x22,0x14,0x0F,0x14,0x25,0x3F,0x45,0x45,0x45,0x44,//键21
0x02,0x1C,0xC0,0x30,0x4C,0x30,0x0F,0x08,0xF8,0x08,0x08,0x28,0x18,0x08,
0x5E,0x43,0x20,0x20,0x10,0x08,0x04,0x03,0x01,0x06,0x08,0x30,0x60,0x20,//次22
0x00,0xF8,0x48,0x48,0x48,0x48,0xFF,0x48,0x48,0x48,0x48,0xF8,0x00,0x00,
0x00,0x0F,0x04,0x04,0x04,0x04,0x3F,0x44,0x44,0x44,0x44,0x4F,0x40,0x70,//电23
0x00,0xFE,0x02,0x42,0x42,0x42,0x42,0xFA,0x42,0x42,0x42,0x62,0x42,0x02,
0x18,0x27,0x20,0x20,0x20,0x20,0x20,0x3F,0x20,0x21,0x2E,0x24,0x20,0x20,//压24  
};

//======================================================
// 128X64I液晶底层驱动[8X16]字体库
// 设计者: powerint
// 描  述: [8X16]西文字符的字模数据 (纵向取模,字节倒序)
// !"#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\]^_`abcdefghijklmnopqrstuvwxyz{|}~
//======================================================
const byte F8X16[]=
{
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,// 0
  0x00,0x00,0x00,0xF8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x33,0x30,0x00,0x00,0x00,//!1
  0x00,0x10,0x0C,0x06,0x10,0x0C,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//"2
  0x40,0xC0,0x78,0x40,0xC0,0x78,0x40,0x00,0x04,0x3F,0x04,0x04,0x3F,0x04,0x04,0x00,//#3
  0x00,0x70,0x88,0xFC,0x08,0x30,0x00,0x00,0x00,0x18,0x20,0xFF,0x21,0x1E,0x00,0x00,//$4
  0xF0,0x08,0xF0,0x00,0xE0,0x18,0x00,0x00,0x00,0x21,0x1C,0x03,0x1E,0x21,0x1E,0x00,//%5
  0x00,0xF0,0x08,0x88,0x70,0x00,0x00,0x00,0x1E,0x21,0x23,0x24,0x19,0x27,0x21,0x10,//&6
  0x10,0x16,0x0E,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//'7
  0x00,0x00,0x00,0xE0,0x18,0x04,0x02,0x00,0x00,0x00,0x00,0x07,0x18,0x20,0x40,0x00,//(8
  0x00,0x02,0x04,0x18,0xE0,0x00,0x00,0x00,0x00,0x40,0x20,0x18,0x07,0x00,0x00,0x00,//)9
  0x40,0x40,0x80,0xF0,0x80,0x40,0x40,0x00,0x02,0x02,0x01,0x0F,0x01,0x02,0x02,0x00,//*10
  0x00,0x00,0x00,0xF0,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x1F,0x01,0x01,0x01,0x00,//+11
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0xB0,0x70,0x00,0x00,0x00,0x00,0x00,//,12
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x01,0x01,0x01,0x01,//-13
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x30,0x00,0x00,0x00,0x00,0x00,//.14
  0x00,0x00,0x00,0x00,0x80,0x60,0x18,0x04,0x00,0x60,0x18,0x06,0x01,0x00,0x00,0x00,///15
  0x00,0xE0,0x10,0x08,0x08,0x10,0xE0,0x00,0x00,0x0F,0x10,0x20,0x20,0x10,0x0F,0x00,//016
  0x00,0x10,0x10,0xF8,0x00,0x00,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,//117
  0x00,0x70,0x08,0x08,0x08,0x88,0x70,0x00,0x00,0x30,0x28,0x24,0x22,0x21,0x30,0x00,//218
  0x00,0x30,0x08,0x88,0x88,0x48,0x30,0x00,0x00,0x18,0x20,0x20,0x20,0x11,0x0E,0x00,//319
  0x00,0x00,0xC0,0x20,0x10,0xF8,0x00,0x00,0x00,0x07,0x04,0x24,0x24,0x3F,0x24,0x00,//420
  0x00,0xF8,0x08,0x88,0x88,0x08,0x08,0x00,0x00,0x19,0x21,0x20,0x20,0x11,0x0E,0x00,//521
  0x00,0xE0,0x10,0x88,0x88,0x18,0x00,0x00,0x00,0x0F,0x11,0x20,0x20,0x11,0x0E,0x00,//622
  0x00,0x38,0x08,0x08,0xC8,0x38,0x08,0x00,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x00,//723
  0x00,0x70,0x88,0x08,0x08,0x88,0x70,0x00,0x00,0x1C,0x22,0x21,0x21,0x22,0x1C,0x00,//824
  0x00,0xE0,0x10,0x08,0x08,0x10,0xE0,0x00,0x00,0x00,0x31,0x22,0x22,0x11,0x0F,0x00,//925
  0x00,0x00,0x00,0xC0,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x30,0x00,0x00,0x00,//:26
  0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x60,0x00,0x00,0x00,0x00,//;27
  0x00,0x00,0x80,0x40,0x20,0x10,0x08,0x00,0x00,0x01,0x02,0x04,0x08,0x10,0x20,0x00,//<28
  0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x00,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x00,//=29
  0x00,0x08,0x10,0x20,0x40,0x80,0x00,0x00,0x00,0x20,0x10,0x08,0x04,0x02,0x01,0x00,//>30
  0x00,0x70,0x48,0x08,0x08,0x08,0xF0,0x00,0x00,0x00,0x00,0x30,0x36,0x01,0x00,0x00,//?31
  0xC0,0x30,0xC8,0x28,0xE8,0x10,0xE0,0x00,0x07,0x18,0x27,0x24,0x23,0x14,0x0B,0x00,//@32
  0x00,0x00,0xC0,0x38,0xE0,0x00,0x00,0x00,0x20,0x3C,0x23,0x02,0x02,0x27,0x38,0x20,//A33
  0x08,0xF8,0x88,0x88,0x88,0x70,0x00,0x00,0x20,0x3F,0x20,0x20,0x20,0x11,0x0E,0x00,//B34
  0xC0,0x30,0x08,0x08,0x08,0x08,0x38,0x00,0x07,0x18,0x20,0x20,0x20,0x10,0x08,0x00,//C35
  0x08,0xF8,0x08,0x08,0x08,0x10,0xE0,0x00,0x20,0x3F,0x20,0x20,0x20,0x10,0x0F,0x00,//D36
  0x08,0xF8,0x88,0x88,0xE8,0x08,0x10,0x00,0x20,0x3F,0x20,0x20,0x23,0x20,0x18,0x00,//E37
  0x08,0xF8,0x88,0x88,0xE8,0x08,0x10,0x00,0x20,0x3F,0x20,0x00,0x03,0x00,0x00,0x00,//F38
  0xC0,0x30,0x08,0x08,0x08,0x38,0x00,0x00,0x07,0x18,0x20,0x20,0x22,0x1E,0x02,0x00,//G39
  0x08,0xF8,0x08,0x00,0x00,0x08,0xF8,0x08,0x20,0x3F,0x21,0x01,0x01,0x21,0x3F,0x20,//H40
  0x00,0x08,0x08,0xF8,0x08,0x08,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,//I41
  0x00,0x00,0x08,0x08,0xF8,0x08,0x08,0x00,0xC0,0x80,0x80,0x80,0x7F,0x00,0x00,0x00,//J42
  0x08,0xF8,0x88,0xC0,0x28,0x18,0x08,0x00,0x20,0x3F,0x20,0x01,0x26,0x38,0x20,0x00,//K43
  0x08,0xF8,0x08,0x00,0x00,0x00,0x00,0x00,0x20,0x3F,0x20,0x20,0x20,0x20,0x30,0x00,//L44
  0x08,0xF8,0xF8,0x00,0xF8,0xF8,0x08,0x00,0x20,0x3F,0x00,0x3F,0x00,0x3F,0x20,0x00,//M45
  0x08,0xF8,0x30,0xC0,0x00,0x08,0xF8,0x08,0x20,0x3F,0x20,0x00,0x07,0x18,0x3F,0x00,//N46
  0xE0,0x10,0x08,0x08,0x08,0x10,0xE0,0x00,0x0F,0x10,0x20,0x20,0x20,0x10,0x0F,0x00,//O47
  0x08,0xF8,0x08,0x08,0x08,0x08,0xF0,0x00,0x20,0x3F,0x21,0x01,0x01,0x01,0x00,0x00,//P48
  0xE0,0x10,0x08,0x08,0x08,0x10,0xE0,0x00,0x0F,0x18,0x24,0x24,0x38,0x50,0x4F,0x00,//Q49
  0x08,0xF8,0x88,0x88,0x88,0x88,0x70,0x00,0x20,0x3F,0x20,0x00,0x03,0x0C,0x30,0x20,//R50
  0x00,0x70,0x88,0x08,0x08,0x08,0x38,0x00,0x00,0x38,0x20,0x21,0x21,0x22,0x1C,0x00,//S51
  0x18,0x08,0x08,0xF8,0x08,0x08,0x18,0x00,0x00,0x00,0x20,0x3F,0x20,0x00,0x00,0x00,//T52
  0x08,0xF8,0x08,0x00,0x00,0x08,0xF8,0x08,0x00,0x1F,0x20,0x20,0x20,0x20,0x1F,0x00,//U53
  0x08,0x78,0x88,0x00,0x00,0xC8,0x38,0x08,0x00,0x00,0x07,0x38,0x0E,0x01,0x00,0x00,//V54
  0xF8,0x08,0x00,0xF8,0x00,0x08,0xF8,0x00,0x03,0x3C,0x07,0x00,0x07,0x3C,0x03,0x00,//W55
  0x08,0x18,0x68,0x80,0x80,0x68,0x18,0x08,0x20,0x30,0x2C,0x03,0x03,0x2C,0x30,0x20,//X56
  0x08,0x38,0xC8,0x00,0xC8,0x38,0x08,0x00,0x00,0x00,0x20,0x3F,0x20,0x00,0x00,0x00,//Y57
  0x10,0x08,0x08,0x08,0xC8,0x38,0x08,0x00,0x20,0x38,0x26,0x21,0x20,0x20,0x18,0x00,//Z58
  0x00,0x00,0x00,0xFE,0x02,0x02,0x02,0x00,0x00,0x00,0x00,0x7F,0x40,0x40,0x40,0x00,//[59
  0x00,0x0C,0x30,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x06,0x38,0xC0,0x00,//\60
  0x00,0x02,0x02,0x02,0xFE,0x00,0x00,0x00,0x00,0x40,0x40,0x40,0x7F,0x00,0x00,0x00,//]61
  0x00,0x00,0x04,0x02,0x02,0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//^62
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,//_63
  0x00,0x02,0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//`64
  0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x19,0x24,0x22,0x22,0x22,0x3F,0x20,//a65
  0x08,0xF8,0x00,0x80,0x80,0x00,0x00,0x00,0x00,0x3F,0x11,0x20,0x20,0x11,0x0E,0x00,//b66
  0x00,0x00,0x00,0x80,0x80,0x80,0x00,0x00,0x00,0x0E,0x11,0x20,0x20,0x20,0x11,0x00,//c67
  0x00,0x00,0x00,0x80,0x80,0x88,0xF8,0x00,0x00,0x0E,0x11,0x20,0x20,0x10,0x3F,0x20,//d68
  0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x1F,0x22,0x22,0x22,0x22,0x13,0x00,//e69
  0x00,0x80,0x80,0xF0,0x88,0x88,0x88,0x18,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,//f70
  0x00,0x00,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x6B,0x94,0x94,0x94,0x93,0x60,0x00,//g71
  0x08,0xF8,0x00,0x80,0x80,0x80,0x00,0x00,0x20,0x3F,0x21,0x00,0x00,0x20,0x3F,0x20,//h72
  0x00,0x80,0x98,0x98,0x00,0x00,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,//i73
  0x00,0x00,0x00,0x80,0x98,0x98,0x00,0x00,0x00,0xC0,0x80,0x80,0x80,0x7F,0x00,0x00,//j74
  0x08,0xF8,0x00,0x00,0x80,0x80,0x80,0x00,0x20,0x3F,0x24,0x02,0x2D,0x30,0x20,0x00,//k75
  0x00,0x08,0x08,0xF8,0x00,0x00,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,//l76
  0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x00,0x20,0x3F,0x20,0x00,0x3F,0x20,0x00,0x3F,//m77
  0x80,0x80,0x00,0x80,0x80,0x80,0x00,0x00,0x20,0x3F,0x21,0x00,0x00,0x20,0x3F,0x20,//n78
  0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x1F,0x20,0x20,0x20,0x20,0x1F,0x00,//o79
  0x80,0x80,0x00,0x80,0x80,0x00,0x00,0x00,0x80,0xFF,0xA1,0x20,0x20,0x11,0x0E,0x00,//p80
  0x00,0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x0E,0x11,0x20,0x20,0xA0,0xFF,0x80,//q81
  0x80,0x80,0x80,0x00,0x80,0x80,0x80,0x00,0x20,0x20,0x3F,0x21,0x20,0x00,0x01,0x00,//r82
  0x00,0x00,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x33,0x24,0x24,0x24,0x24,0x19,0x00,//s83
  0x00,0x80,0x80,0xE0,0x80,0x80,0x00,0x00,0x00,0x00,0x00,0x1F,0x20,0x20,0x00,0x00,//t84
  0x80,0x80,0x00,0x00,0x00,0x80,0x80,0x00,0x00,0x1F,0x20,0x20,0x20,0x10,0x3F,0x20,//u85
  0x80,0x80,0x80,0x00,0x00,0x80,0x80,0x80,0x00,0x01,0x0E,0x30,0x08,0x06,0x01,0x00,//v86
  0x80,0x80,0x00,0x80,0x00,0x80,0x80,0x80,0x0F,0x30,0x0C,0x03,0x0C,0x30,0x0F,0x00,//w87
  0x00,0x80,0x80,0x00,0x80,0x80,0x80,0x00,0x00,0x20,0x31,0x2E,0x0E,0x31,0x20,0x00,//x88
  0x80,0x80,0x80,0x00,0x00,0x80,0x80,0x80,0x80,0x81,0x8E,0x70,0x18,0x06,0x01,0x00,//y89
  0x00,0x80,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x21,0x30,0x2C,0x22,0x21,0x30,0x00,//z90
  0x00,0x00,0x00,0x00,0x80,0x7C,0x02,0x02,0x00,0x00,0x00,0x00,0x00,0x3F,0x40,0x40,//{91
  0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,//|92
  0x00,0x02,0x02,0x7C,0x80,0x00,0x00,0x00,0x00,0x40,0x40,0x3F,0x00,0x00,0x00,0x00,//}93
  0x00,0x06,0x01,0x01,0x02,0x02,0x04,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//~94

};
//数据水平，字节垂直
byte longqiu96x64[768] = {
/* 
  0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
  0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X80,0X80,0X80,0XC0,0XC0,0XC0,
  0XC0,0XC0,0X60,0X60,0X60,0X60,0X60,0X70,0X70,0X70,0X30,0X30,0X30,0X30,0X30,0X30,
  0X30,0X30,0X30,0X30,0X30,0X30,0X30,0X70,0X70,0X60,0X60,0X60,0X60,0X60,0XE0,0XC0,
  0XC0,0XC0,0XC0,0X80,0X80,0X80,0X80,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
  0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
  0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X80,0X80,0XC0,0XE0,0X60,0X30,
  0X38,0X18,0X1C,0X0C,0X0E,0X06,0X06,0X03,0X03,0X03,0X01,0X01,0X01,0X01,0X00,0X00,
  0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
  0X80,0XC0,0XC0,0X80,0X00,0X00,0X00,0X00,0X00,0X80,0XC0,0XE0,0XE0,0XE0,0X60,0X60,
  0XE0,0XE0,0XC0,0X81,0X01,0X01,0X03,0X03,0X03,0X07,0X06,0X0E,0X0C,0X1C,0X18,0X38,
  0X30,0X70,0X60,0XC0,0XC0,0X80,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
  0X00,0X00,0X00,0X80,0XC0,0XF0,0X38,0X9C,0X8E,0XC7,0XC3,0XE1,0XF1,0XB0,0X00,0X00,
  0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0XC0,0XE0,0XF0,0X70,0X78,0XF0,0XF0,0XE0,
  0X80,0X30,0XF8,0XF8,0XF8,0XF8,0X1C,0X3C,0XFC,0XF8,0XF0,0X00,0XF0,0XF8,0XFC,0XFC,
  0XCF,0XCF,0XFF,0XFF,0XFC,0XF8,0X70,0XFC,0XFF,0XFF,0XFF,0X0F,0X01,0X70,0X70,0X30,
  0X00,0XFF,0XFF,0XFF,0XFF,0XFC,0X00,0XE0,0XE0,0XEE,0XEE,0X0E,0X00,0XC0,0XC0,0X80,
  0X80,0X00,0X00,0X00,0X01,0X03,0X03,0X06,0X0C,0X3C,0X70,0XE0,0XC0,0X00,0X00,0X00,
  0X00,0XF0,0XFE,0X1F,0X03,0X00,0X00,0X01,0X01,0X01,0X03,0X0F,0X1F,0X7F,0XFE,0XFC,
  0XF0,0XE0,0X80,0X80,0X86,0XFE,0XFC,0XF0,0X87,0X1F,0X1E,0X78,0XF8,0XF8,0X7D,0X7F,
  0X1F,0X20,0X3F,0X3F,0X3F,0X3F,0X30,0X00,0X3F,0X3F,0X3F,0X7C,0XF0,0XF9,0XFF,0XFF,
  0XFF,0XCF,0X9F,0XFF,0XFB,0XF8,0X70,0X03,0X0F,0X1F,0X1F,0X3E,0X3C,0X36,0X76,0X7E,
  0X7E,0XFF,0XFF,0XEF,0X47,0X61,0XFC,0XFF,0XFF,0XCF,0XE3,0XF8,0XFE,0XFF,0X9F,0X87,
  0XE3,0XF0,0XFE,0XFC,0X3C,0X0C,0X00,0X00,0X00,0X00,0X00,0X01,0X07,0X7F,0XFC,0X80,
  0X00,0X1F,0XFF,0XF0,0X80,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X03,
  0X1F,0X1F,0X0F,0X0F,0X07,0X03,0X03,0X01,0X00,0X00,0X00,0X00,0X00,0X00,0XC0,0XC0,
  0X00,0X00,0X00,0XC0,0XC0,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X01,0X03,0X03,
  0X03,0X03,0X03,0X01,0X01,0X00,0X00,0X80,0X00,0X00,0X00,0X00,0X00,0X80,0X80,0XC0,
  0XC0,0X80,0X01,0X01,0X00,0X00,0X00,0X00,0X00,0X00,0X01,0X03,0X03,0X03,0X07,0X0F,
  0X0F,0X0F,0X01,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0XC0,0XFE,0X7F,0X03,
  0X00,0X00,0X00,0X03,0X07,0X1E,0X38,0X70,0XE0,0XC0,0X80,0X00,0X00,0X00,0X00,0X00,
  0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X06,0X86,0X82,0XC2,0X62,0X3A,0X8F,0X43,
  0XFA,0XEE,0X32,0X3A,0X1A,0X02,0X02,0X02,0X00,0X00,0X00,0X00,0X00,0X1C,0X1C,0X1C,
  0X00,0X00,0X00,0X00,0X00,0X00,0X03,0XFF,0XFF,0X09,0X09,0X09,0X09,0XF9,0XF9,0X09,
  0X08,0X08,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
  0X00,0X00,0X00,0X00,0X00,0X80,0XC0,0XC0,0X60,0X38,0X1C,0X0F,0X07,0X01,0X00,0X00,
  0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X01,0X01,0X03,0X07,0X0E,0X0C,0X18,
  0X18,0X30,0X30,0X60,0X60,0XC0,0XC2,0XC3,0X83,0X83,0X81,0X00,0X00,0X00,0X00,0X00,
  0X01,0X03,0X03,0X07,0X07,0X06,0X0E,0X06,0X07,0X03,0X00,0X00,0X00,0X00,0X00,0X00,
  0X00,0X00,0X03,0X03,0X03,0X03,0X03,0X03,0X03,0X03,0X03,0X03,0X03,0X03,0X03,0X03,
  0X03,0X07,0X07,0X07,0X06,0X06,0X80,0X80,0X80,0XC0,0XC0,0XE0,0X60,0X70,0X30,0X38,
  0X18,0X1C,0X0E,0X06,0X07,0X03,0X01,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
  0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
  0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X01,0X01,0X01,0X03,0X03,0X03,0X07,0X06,0X06,
  0X06,0X06,0X0E,0X0C,0X0C,0X0C,0X0C,0X0C,0X0C,0X1C,0X1C,0X1C,0X18,0X18,0X18,0X18,
  0X18,0X18,0X18,0X18,0X1C,0X1C,0X1C,0X0C,0X0C,0X0C,0X0C,0X0C,0X0C,0X0C,0X06,0X06,
  0X06,0X06,0X06,0X03,0X03,0X03,0X01,0X01,0X01,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
  0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
*/
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X80,0X80,0XC0,0XC0,0XC0,0X60,0X60,0X60,0X30,0X30,0X30,0X38,0X18,0X18,
0X18,0X18,0X0C,0X0C,0X0C,0X0C,0X0C,0X0E,0X0E,0X0E,0X06,0X06,0X06,0X06,0X06,0X06,
0X06,0X06,0X06,0X06,0X06,0X06,0X06,0X0E,0X0E,0X0C,0X0C,0X0C,0X0C,0X0C,0X1C,0X18,
0X18,0X18,0X18,0X30,0X30,0X30,0X70,0X60,0X60,0XE0,0XC0,0XC0,0X80,0X80,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X80,0XC0,0XE0,0X70,0X30,0X38,0X1C,0X0C,0X06,
0X07,0X03,0X03,0X01,0X01,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X80,0X80,0X80,0X00,0X00,0X00,0X00,0X00,0X80,0X80,
0XF0,0XF8,0XF8,0XF0,0X80,0X00,0X00,0X80,0XE0,0XF0,0XF8,0XFC,0X3C,0X1C,0X0C,0X0C,
0X1C,0XFC,0XF8,0XF0,0XE0,0X80,0X00,0X00,0X00,0XC0,0XC0,0XC1,0X01,0X03,0X03,0X07,
0X06,0X0E,0X0C,0X18,0X38,0X70,0X60,0XC0,0X80,0X80,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0XC0,0XF0,0X78,0X1E,0X07,0X33,0X31,0X38,0X78,0XFC,0XFE,0XF6,0XC0,0X80,
0X00,0X00,0X00,0X00,0XC0,0XC0,0X80,0X00,0XF8,0XFC,0XDE,0X0E,0X0F,0X1E,0XBE,0XFC,
0XF0,0X06,0XFF,0XFF,0XFF,0XFF,0X03,0X07,0XFF,0XFF,0XFE,0X80,0X1E,0X3F,0XFF,0XFF,
0XF9,0XF9,0XFF,0XFF,0X7F,0X1F,0X0E,0X7F,0XFF,0XFF,0XFF,0XC1,0X80,0XCE,0XCE,0XC6,
0XC0,0XFF,0XFF,0XFF,0XFF,0X3F,0X80,0XFC,0XFC,0XFD,0X7D,0X01,0XC0,0XF8,0XF8,0XF0,
0X70,0X00,0XC0,0X80,0X80,0X80,0X00,0X00,0X01,0X07,0X0E,0X3C,0XF8,0XE0,0X80,0X00,
0X00,0XFE,0XFF,0X03,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X01,0X03,0X0F,0X1F,0X7F,
0XFE,0XFC,0XF0,0XF0,0XF0,0X7F,0X7F,0X3E,0X10,0X03,0X03,0X0F,0X1F,0X1F,0X0F,0X0F,
0X03,0X04,0X07,0X07,0X07,0X07,0X06,0X00,0X07,0X07,0X07,0X0F,0X1E,0X3F,0X7F,0X7F,
0X7F,0X79,0X73,0X3F,0X3F,0X1F,0X0E,0X00,0X01,0X03,0X03,0X07,0X07,0X06,0X0E,0X0F,
0X0F,0X1F,0X3F,0X3D,0X08,0X0C,0X1F,0X1F,0X1F,0X19,0X3C,0X7F,0X7F,0X7F,0XF3,0XF0,
0XFC,0XFE,0X3F,0X1F,0X07,0X01,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0XCF,0XFF,0X70,
0X00,0X03,0X1F,0X7E,0XF0,0XC0,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X03,0X03,0X01,0X01,0X00,0X00,0X00,0X00,0XC0,0XC0,0X40,0X40,0X40,0X40,0XF8,0X78,
0X40,0XC0,0X40,0X58,0X58,0X40,0X40,0X40,0X00,0X00,0X00,0X00,0X00,0X80,0X80,0X80,
0X00,0X00,0X00,0X00,0X00,0X00,0X60,0XF0,0XE0,0X20,0X20,0X20,0X20,0X30,0X30,0X38,
0X18,0X10,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X01,
0X01,0X01,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X80,0XE0,0XF8,0X3F,0X0F,0X00,
0X00,0X00,0X00,0X00,0X00,0X03,0X07,0X0E,0X1C,0X38,0X30,0X60,0XE0,0XC0,0X80,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X40,0X60,0X60,0X70,0X30,0X18,0X0C,0X07,0X11,0X08,
0X3F,0X7D,0X66,0XE7,0XE3,0XC0,0XC0,0XC0,0XE0,0X60,0X00,0X00,0X00,0X03,0X03,0X03,
0X00,0X00,0X60,0X60,0X60,0X60,0X60,0X7F,0X7F,0X61,0X61,0X61,0X61,0X7F,0X7F,0X61,
0X61,0XE1,0XE0,0XE0,0XC0,0XC0,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X80,0XC0,0XC0,0XE0,0X70,0X38,0X18,0X0C,0X07,0X03,0X01,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X01,0X01,0X03,
0X03,0X06,0X06,0X0C,0X0C,0X18,0X18,0X38,0X30,0X30,0X70,0X60,0X60,0XE0,0XC0,0XC0,
0XC0,0XC0,0XC0,0X80,0X80,0X80,0X81,0X80,0X80,0X80,0X80,0X80,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X80,0X80,0X80,0X80,0X80,0X80,0X80,0X80,0X80,0X80,0XC0,0XC0,
0XC0,0XC0,0XC0,0X60,0X60,0X60,0X30,0X30,0X30,0X18,0X18,0X1C,0X0C,0X0E,0X06,0X07,
0X03,0X03,0X01,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X01,0X01,0X01,0X01,0X01,0X01,0X01,0X03,0X03,0X03,0X03,0X03,0X03,0X03,
0X03,0X03,0X03,0X03,0X03,0X03,0X03,0X01,0X01,0X01,0X01,0X01,0X01,0X01,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,

};
const byte LIBLOGO60x58[480] = { 
  0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
  0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
  0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
  0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
  0X00,0X00,0XC0,0XE0,0XF8,0XFC,0XFE,0X7F,0X3F,0X0F,0X0F,0X07,0X07,0X07,0X87,0XC7,
  0XC7,0XC7,0XE7,0XEF,0XFF,0XFF,0XFF,0XEF,0XE7,0XE7,0XE7,0XE7,0XE7,0XEF,0XFF,0XFF,
  0XFF,0XEF,0XE7,0XE7,0XE7,0XE7,0XE7,0XF7,0X07,0X0F,0X3F,0X7F,0XFF,0XFE,0XFC,0XF0,
  0XC0,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0XD8,0XDC,0XDF,0XDF,0XDF,
  0XDF,0XD7,0X01,0X00,0X00,0X00,0X00,0X00,0X00,0X3E,0X7F,0XFF,0XFF,0XE3,0XE1,0XE1,
  0XFF,0XFF,0XFF,0XE0,0XE0,0XE0,0XE0,0XE0,0XE0,0XE0,0XFF,0XFF,0XFF,0XE0,0XE0,0XE0,
  0XE0,0XC1,0X83,0X07,0X00,0X00,0X00,0X00,0X01,0XD7,0XDF,0XDF,0XDF,0XDF,0XDC,0XD8,
  0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X01,0X03,0X0F,0X1F,0X3F,0XFF,0XFF,0XFE,0XF8,
  0XF0,0XE0,0XC0,0X80,0X00,0X00,0XF8,0XF0,0XE0,0XC0,0XC0,0XC0,0XFF,0XFF,0XFF,0XC0,
  0XC0,0XC0,0XC0,0XC0,0XC0,0XC0,0XFF,0XFF,0XFF,0XC0,0XE0,0XE1,0XF3,0XFF,0X7F,0X3F,
  0X00,0XC0,0XE0,0XF8,0XFE,0XFF,0XFF,0XFF,0X3F,0X1F,0X07,0X01,0X00,0X00,0X00,0X00,
  0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X01,0X07,0X0F,0X3F,0X7F,0XFF,
  0XFE,0XFC,0XF9,0XF3,0XC1,0X81,0X01,0X01,0XFF,0XFF,0XFF,0X01,0X01,0X01,0X01,0X01,
  0X01,0X01,0XFF,0XFF,0XFF,0X03,0X01,0X81,0XC0,0XF0,0XFC,0XFE,0XFF,0XFF,0X7F,0X3F,
  0X0F,0X07,0X03,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
  0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X03,0X07,0X0F,0X3F,
  0X7F,0XFF,0XFF,0XFF,0XFB,0XF3,0XE3,0XC3,0X03,0X03,0X03,0X03,0X03,0X83,0XC3,0XF3,
  0XF3,0XFB,0XFF,0XFF,0XFF,0X7F,0X3F,0X0F,0X03,0X01,0X00,0X00,0X00,0X00,0X00,0X00,
  0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
  0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X03,0X07,
  0X07,0X1F,0X3F,0X7F,0XFF,0XFE,0XFC,0XFC,0XFE,0XFF,0XFF,0X7F,0X3F,0X0F,0X07,0X03,
  0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
  0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
  0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
  0X00,0X03,0X07,0X0F,0X0F,0X07,0X01,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
  0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
}; 
void LCD_WrDat(byte data)
{
	byte i=8;
	//LCD_CS=0;;
	GPIOE_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(3));;;;
        asm("nop"); 
      GPIOE_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(0));;;;
      asm("nop");    
  while(i--)
  {
    if(data&0x80){GPIOE_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(1));;;;}
    else{GPIOE_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(1));;;;}
    GPIOE_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(0)); 
    asm("nop");;;;
		//asm("nop");            
    GPIOE_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(0));;;;;    
    data<<=1;    
  }
	//LCD_CS=1;
}
void LCD_WrCmd(byte cmd)
{
	byte i=8;
	
	//LCD_CS=0;;
  GPIOE_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(3));;;;;
  GPIOE_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(0));;;;;
  //asm("nop");   
  while(i--)
  {
    if(cmd&0x80){GPIOE_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(1));;;;;}
    else{GPIOE_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(1));;;;;;}
    GPIOE_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(0));;;;;
    asm("nop");;;;
		//asm("nop");             
    GPIOE_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(0));;;;;    
    cmd<<=1;;;;;   
  } 	
	//LCD_CS=1;
}
void LCD_Set_Pos(byte x, byte y)
{ 
  LCD_WrCmd(0xb0+y);
  LCD_WrCmd(((x&0xf0)>>4)|0x10);
  LCD_WrCmd((x&0x0f)|0x01); 
} 
void LCD_Fill(byte bmp_data)
{
	byte y,x;
	
	for(y=0;y<8;y++)
	{
		LCD_WrCmd(0xb0+y);
		LCD_WrCmd(0x01);
		LCD_WrCmd(0x10);
		for(x=0;x<X_WIDTH;x++)
			LCD_WrDat(bmp_data);
	}
}
void LCD_CLS(void)
{
	byte y,x;	
	for(y=0;y<8;y++)
	{
		LCD_WrCmd(0xb0+y);
		LCD_WrCmd(0x01);
		LCD_WrCmd(0x10); 
		for(x=0;x<X_WIDTH;x++)
			LCD_WrDat(0);
	}
}
void LCD_DLY_ms(word ms)
{                         
  word a;
  while(ms)
  {
    a=13350;
    while(a--);
    ms--;
  }
  return;
}

void LCD_Init(void)        
{
        //设置PORTA pin14,pin15为GPIO口 
	PORTE_PCR0=(0|PORT_PCR_MUX(1));
	PORTE_PCR1=(0|PORT_PCR_MUX(1)); 
	PORTE_PCR2=(0|PORT_PCR_MUX(1));
	PORTE_PCR3=(0|PORT_PCR_MUX(1)); 
	
	//设置PORTA pin14,pin15为输出方向;pin16,pin17为输入方向
	GPIOE_PDDR=GPIO_PDDR_PDD(GPIO_PIN(0)|GPIO_PIN(1)|GPIO_PIN(2)|GPIO_PIN(3));
	
  
	GPIOE_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(0));
	//LCD_CS=1;	//预制SLK和SS为高电平   	
	
	GPIOE_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(2));
	LCD_DLY_ms(50);
	GPIOE_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(2));

  LCD_WrCmd(0xae);//--turn off oled panel
  LCD_WrCmd(0x00);//---set low column address
  LCD_WrCmd(0x10);//---set high column address
  LCD_WrCmd(0x40);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
  LCD_WrCmd(0x81);//--set contrast control register
  LCD_WrCmd(0xcf); // Set SEG Output Current Brightness
  LCD_WrCmd(0xa1);//--Set SEG/Column Mapping     0xa0左右反置 0xa1正常
  LCD_WrCmd(0xc8);//Set COM/Row Scan Direction   0xc0上下反置 0xc8正常
  LCD_WrCmd(0xa6);//--set normal display
  LCD_WrCmd(0xa8);//--set multiplex ratio(1 to 64)
  LCD_WrCmd(0x3f);//--1/64 duty
  LCD_WrCmd(0xd3);//-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
  LCD_WrCmd(0x00);//-not offset
  LCD_WrCmd(0xd5);//--set display clock divide ratio/oscillator frequency
  LCD_WrCmd(0x80);//--set divide ratio, Set Clock as 100 Frames/Sec
  LCD_WrCmd(0xd9);//--set pre-charge period
  LCD_WrCmd(0xf1);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
  LCD_WrCmd(0xda);//--set com pins hardware configuration
  LCD_WrCmd(0x12);
  LCD_WrCmd(0xdb);//--set vcomh
  LCD_WrCmd(0x40);//Set VCOM Deselect Level
  LCD_WrCmd(0x20);//-Set Page Addressing Mode (0x00/0x01/0x02)
  LCD_WrCmd(0x02);//
  LCD_WrCmd(0x8d);//--set Charge Pump enable/disable
  LCD_WrCmd(0x14);//--set(0x10) disable
  LCD_WrCmd(0xa4);// Disable Entire Display On (0xa4/0xa5)
  LCD_WrCmd(0xa6);// Disable Inverse Display On (0xa6/a7) 
  LCD_WrCmd(0xaf);//--turn on oled panel
  LCD_Fill(0x00);  //初始清屏
  LCD_Set_Pos(0,0);  
	
} 
//==============================================================
//函数名： void LCD_PutPixel(byte x,byte y)
//功能描述：绘制一个点（x,y）
//参数：真实坐标值(x,y),x的范围0～127，y的范围0～64
//返回：无
//==============================================================
void LCD_PutPixel(byte x,byte y)
{
	byte data1;  //data1当前点的数据 
	 
        LCD_Set_Pos(x,y); 
	data1 = 0x01<<(y%8); 	
	LCD_WrCmd(0xb0+(y>>3));
	LCD_WrCmd(((x&0xf0)>>4)|0x10);
	LCD_WrCmd((x&0x0f)|0x00);
	LCD_WrDat(data1); 	 	
}
//==============================================================
//函数名： void LCD_Rectangle(byte x1,byte y1,
//                   byte x2,byte y2,byte color,byte gif)
//功能描述：绘制一个实心矩形
//参数：左上角坐标（x1,y1）,右下角坐标（x2，y2）
//      其中x1、x2的范围0～127，y1，y2的范围0～63，即真实坐标值
//返回：无
//==============================================================
void LCD_Rectangle(byte x1,byte y1,byte x2,byte y2,byte gif)
{
	byte n; 
		
	LCD_Set_Pos(x1,y1>>3);
	for(n=x1;n<=x2;n++)
	{
		LCD_WrDat(0x01<<(y1%8)); 			
		if(gif == 1) 	LCD_DLY_ms(50);
	}  
	LCD_Set_Pos(x1,y2>>3);
  for(n=x1;n<=x2;n++)
	{
		LCD_WrDat(0x01<<(y2%8)); 			
		if(gif == 1) 	LCD_DLY_ms(5);
	}
	
}  
//==============================================================
//函数名：LCD_P6x8Str(byte x,byte y,byte *p)
//功能描述：写入一组标准ASCII字符串
//参数：显示的位置（x,y），y为页范围0～7，要显示的字符串
//返回：无
//==============================================================  
void LCD_P6x8Str(byte x,byte y,byte ch[])
{
  byte c=0,i=0,j=0;      
  while (ch[j]!='\0')
  {    
    c =ch[j]-32;
    if(x>126){x=0;y++;}
    LCD_Set_Pos(x,y);    
  	for(i=0;i<6;i++)     
  	  LCD_WrDat(F6x8[c][i]);  
  	x+=6;
  	j++;
  }
}
//==============================================================
//函数名：LCD_P8x16Str(byte x,byte y,byte *p)
//功能描述：写入一组标准ASCII字符串
//参数：显示的位置（x,y），y为页范围0～7，要显示的字符串
//返回：无
//==============================================================  
void LCD_P8x16Str(byte x,byte y,byte ch[])
{
  byte c=0,i=0,j=0;
        
  while (ch[j]!='\0')
  {    
    c =ch[j]-32;
    if(x>120){x=0;y++;}
    LCD_Set_Pos(x,y);    
  	for(i=0;i<8;i++)     
  	  LCD_WrDat(F8X16[c*16+i]);
  	LCD_Set_Pos(x,y+1);    
  	for(i=0;i<8;i++)     
  	  LCD_WrDat(F8X16[c*16+i+8]);  
  	x+=8;
  	j++;
  }
}
//输出汉字字符串
void LCD_P14x16Str(byte x,byte y,byte ch[])
{
	byte wm=0,ii = 0;
	word adder=1; 
	
	while(ch[ii] != '\0')
	{
  	wm = 0;
  	adder = 1;
  	while(F14x16_Idx[wm] > 127)
  	{
  		if(F14x16_Idx[wm] == ch[ii])
  		{
  			if(F14x16_Idx[wm + 1] == ch[ii + 1])
  			{
  				adder = wm * 14;
  				break;
  			}
  		}
  		wm += 2;			
  	}
  	if(x>118){x=0;y++;}
  	LCD_Set_Pos(x , y); 
  	if(adder != 1)// 显示汉字					
  	{
  		LCD_Set_Pos(x , y);
  		for(wm = 0;wm < 14;wm++)               
  		{
  			LCD_WrDat(F14x16[adder]);	
  			adder += 1;
  		}      
  		LCD_Set_Pos(x,y + 1); 
  		for(wm = 0;wm < 14;wm++)          
  		{
  			LCD_WrDat(F14x16[adder]);
  			adder += 1;
  		}   		
  	}
  	else			  //显示空白字符			
  	{
  		ii += 1;
      LCD_Set_Pos(x,y);
  		for(wm = 0;wm < 16;wm++)
  		{
  				LCD_WrDat(0);
  		}
  		LCD_Set_Pos(x,y + 1);
  		for(wm = 0;wm < 16;wm++)
  		{   		
  				LCD_WrDat(0);	
  		}
  	}
  	x += 14;
  	ii += 2;
	}
}
//输出汉字和字符混合字符串
void LCD_Print(byte x, byte y, byte ch[])
{
	byte ch2[3];
	byte ii=0;        
	while(ch[ii] != '\0')
	{
		if(ch[ii] > 127)
		{
			ch2[0] = ch[ii];
	 		ch2[1] = ch[ii + 1];
			ch2[2] = '\0';			//汉字为两个字节
			LCD_P14x16Str(x , y, ch2);	//显示汉字
			x += 14;
			ii += 2;
		}
		else
		{
			ch2[0] = ch[ii];	
			ch2[1] = '\0';			//字母占一个字节
			LCD_P8x16Str(x , y , ch2);	//显示字母
			x += 8;
			ii+= 1;
		}
	}
} 

//==============================================================
//函数名： void Draw_BMP(byte x,byte y)
//功能描述：显示BMP图片128×64
//参数：起始点坐标(x,y),x的范围0～127，y为页的范围0～7
//返回：无
//==============================================================
void Draw_BMP(byte x0,byte y0,byte x1,byte y1,byte bmp[])
{ 	
  word ii=0;
  byte x,y;
  
  if(y1%8==0) y=y1/8;      
  else y=y1/8+1;
	for(y=y0;y<=y1;y++)
	{
		LCD_Set_Pos(x0,y);				
    for(x=x0;x<x1;x++)
	    {      
	    	LCD_WrDat(bmp[ii++]);	    	
	    }
	}
}





void LCD_P6x8Str_5(byte x,byte y,int16 date)
{
  byte c=0,i=0,j=0,ch[5]={0};
  ch[0]=date/10000+'0';
  date%=10000;
  ch[1]=date/1000+'0';
  date%=1000;
  ch[2]=date/100+'0';
  date%=100;
  ch[3]=date/10+'0';
  date%=10;
  ch[4]=date+'0';
  while (ch[j]!='\0')
  {
    c =ch[j]-32;
    if(x>126){x=0;y++;}
    LCD_Set_Pos(x,y);
  	for(i=0;i<6;i++)
  	  LCD_WrDat(F6x8[c][i]);
  	x+=6;
  	j++;
  }
}

void LCD_P6x8Str_4(byte x,byte y,int16 date)
{
  byte c=0,i=0,j=0,ch[4]={0};
  
  ch[0]=date/1000+'0';
  date%=1000;
  ch[1]=date/100+'0';
  date%=100;
  ch[2]=date/10+'0';
  date%=10;
  ch[3]=date+'0';
  while (ch[j]!='\0')
  {
    c =ch[j]-32;
    if(x>126){x=0;y++;}
    LCD_Set_Pos(x,y);
  	for(i=0;i<6;i++)
  	  LCD_WrDat(F6x8[c][i]);
  	x+=6;
  	j++;
  }
}

void LCD_P6x8Str_3(byte x,byte y,int16 date)
{
  byte c=0,i=0,j=0,ch[3]={0};
  ch[0]=date/100+'0';
  date%=100;
  ch[1]=date/10+'0';
  date%=10;
  ch[2]=date+'0';
  while (ch[j]!='\0')
  {
    c =ch[j]-32;
    if(x>126){x=0;y++;}
    LCD_Set_Pos(x,y);
  	for(i=0;i<6;i++)
  	  LCD_WrDat(F6x8[c][i]);
  	x+=6;
  	j++;
  }
}


void LCD_P6x8Str_FuHao_3(byte x,byte y,signed char date)
{
  byte c=0,i=0,j=0,ch[3]={0};
  if(date<0)
  {
    date=-date;
    LCD_Set_Pos(x,y);
    for(i=0;i<6;i++) LCD_WrDat(F6x8[13][i]);
  }
  ch[0]=date/100+'0';
  date%=100;
  ch[1]=date/10+'0';
  date%=10;
  ch[2]=date+'0';
  while (ch[j]!='\0')
  {
    c =ch[j]-32;
    if(x>126){x=0;y++;}
    LCD_Set_Pos(x+6,y);
  	for(i=0;i<6;i++)
  	  LCD_WrDat(F6x8[c][i]);
  	x+=6;
  	j++;
  }
}


void LCD_P6x8Str_1(byte x,byte y,signed char date)     //带符号的一位显示
{
  byte c=0,i=0;
  if(date<0)
  {
    date=-date;
    LCD_Set_Pos(x,y);
    for(i=0;i<6;i++) LCD_WrDat(F6x8[13][i]);
  }
  c=date-32+'0';
  LCD_Set_Pos(x+6,y);
  for(i=0;i<6;i++) LCD_WrDat(F6x8[c][i]);
}

void LCD_CLS2(byte y)
{
  byte x;
  LCD_WrCmd(0xb0+y);
  LCD_WrCmd(0x01);
  LCD_WrCmd(0x10); 
  for(x=0;x<X_WIDTH;x++)LCD_WrDat(0);
			
	
}



