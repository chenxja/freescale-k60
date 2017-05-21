///////////////////////////////////////////////////////////////////////////////
//                                                                            /
// IAR ANSI C/C++ Compiler V6.10.1.52143/W32 for ARM    30/Mar/2012  16:33:17 /
// Copyright 1999-2010 IAR Systems AB.                                        /
//                                                                            /
//    Cpu mode     =  thumb                                                   /
//    Endian       =  little                                                  /
//    Source file  =  E:\kinetis_IAR\19_LQ_DMAov7620\src\LCD.c                /
//    Command line =  E:\kinetis_IAR\19_LQ_DMAov7620\src\LCD.c -D IAR -D      /
//                    TWR_K60N512 -lCN E:\kinetis_IAR\19_LQ_DMAov7620\bin\Fla /
//                    sh\List\ -lB E:\kinetis_IAR\19_LQ_DMAov7620\bin\Flash\L /
//                    ist\ -o E:\kinetis_IAR\19_LQ_DMAov7620\bin\Flash\Obj\   /
//                    --no_cse --no_unroll --no_inline --no_code_motion       /
//                    --no_tbaa --no_clustering --no_scheduling --debug       /
//                    --endian=little --cpu=Cortex-M4 -e --fpu=None           /
//                    --dlib_config "C:\Program Files (x86)\IAR               /
//                    Systems\Embedded Workbench                              /
//                    6.0\arm\INC\c\DLib_Config_Normal.h" -I                  /
//                    E:\kinetis_IAR\19_LQ_DMAov7620\src\Sources\H\ -I        /
//                    E:\kinetis_IAR\19_LQ_DMAov7620\src\Sources\H\Component_ /
//                    H\ -I E:\kinetis_IAR\19_LQ_DMAov7620\src\Sources\H\Fram /
//                    e_H\ -Ol --use_c++_inline                               /
//    List file    =  E:\kinetis_IAR\19_LQ_DMAov7620\bin\Flash\List\LCD.s     /
//                                                                            /
//                                                                            /
///////////////////////////////////////////////////////////////////////////////

        NAME LCD

        PUBLIC F14x16
        PUBLIC F14x16_Idx
        PUBLIC F8X16
        PUBLIC LCD_DLY_ms
        PUBLIC LCD_Fill
        PUBLIC LCD_Init
        PUBLIC LCD_P14x16Str
        PUBLIC LCD_P8x16Str
        PUBLIC LCD_P8x16chr
        PUBLIC LCD_Set_Pos
        PUBLIC LCD_WrCmd
        PUBLIC LCD_WrDat
        PUBLIC displaypara
        PUBLIC pusher0

// E:\kinetis_IAR\19_LQ_DMAov7620\src\LCD.c
//    1 /********************************************************   
//    2 【平    台】龙丘K60X256多功能开发板
//    3 【编    写】龙丘
//    4 【Designed】by Chiu Sir
//    5 【E-mail  】chiusir@yahoo.cn
//    6 【软件版本】V1.0
//    7 【最后更新】2012年1月3日
//    8 【相关信息参考下列地址】
//    9 【网    站】http://www.lqist.cn
//   10 【淘宝店铺】http://shop36265907.taobao.com
//   11 ------------------------------------------------
//   12 【dev.env.】CodeWarrior 10.1/IAR
//   13 【Target  】K60X256
//   14 【Crystal 】50.000Mhz
//   15 【busclock】???.000MHz
//   16 【pllclock】100.000MHz    
//   17 ***************************
//   18 ------------------------------------
//   19   使用说明：
//   20 下载程序后LED交替闪烁
//   21 锁相环137.5M测试程序
//   22   
//   23  *********************************************************/
//   24 #include "MK60N512VMD100.h"  
//   25 #define A0_MASK 0x00000100
//   26 #define RESET_MASK 0x00000400
//   27 #define X_WIDTH 128
//   28 #define Y_WIDTH 64
//   29 typedef unsigned char		uint8;  /*  8 bits */
//   30 typedef unsigned short int	uint16; /* 16 bits */
//   31 typedef unsigned long int	uint32; /* 32 bits */
//   32 typedef char			int8;   /*  8 bits */
//   33 typedef short int	        int16;  /* 16 bits */
//   34 typedef int		        int32;  /* 32 bits */

        SECTION `.data`:DATA:REORDER:NOROOT(2)
//   35 uint32 pusher0=0x00010000;
pusher0:
        DATA
        DC32 65536
//   36 //======================================================
//   37 // 128X64I液晶底层驱动[8X16]字体库
//   38 // 设计者: powerint
//   39 // 描  述: [8X16]西文字符的字模数据 (纵向取模,字节倒序)
//   40 // !"#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\]^_`abcdefghijklmnopqrstuvwxyz{|}~
//   41 //======================================================
//   42 const uint8 F8X16[]=
//   43 {
//   44 	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,// 0
//   45   0x00,0x00,0x00,0xF8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x33,0x30,0x00,0x00,0x00,//!1
//   46   0x00,0x10,0x0C,0x06,0x10,0x0C,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//"2
//   47   0x40,0xC0,0x78,0x40,0xC0,0x78,0x40,0x00,0x04,0x3F,0x04,0x04,0x3F,0x04,0x04,0x00,//#3
//   48   0x00,0x70,0x88,0xFC,0x08,0x30,0x00,0x00,0x00,0x18,0x20,0xFF,0x21,0x1E,0x00,0x00,//$4
//   49   0xF0,0x08,0xF0,0x00,0xE0,0x18,0x00,0x00,0x00,0x21,0x1C,0x03,0x1E,0x21,0x1E,0x00,//%5
//   50   0x00,0xF0,0x08,0x88,0x70,0x00,0x00,0x00,0x1E,0x21,0x23,0x24,0x19,0x27,0x21,0x10,//&6
//   51   0x10,0x16,0x0E,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//'7
//   52   0x00,0x00,0x00,0xE0,0x18,0x04,0x02,0x00,0x00,0x00,0x00,0x07,0x18,0x20,0x40,0x00,//(8
//   53   0x00,0x02,0x04,0x18,0xE0,0x00,0x00,0x00,0x00,0x40,0x20,0x18,0x07,0x00,0x00,0x00,//)9
//   54   0x40,0x40,0x80,0xF0,0x80,0x40,0x40,0x00,0x02,0x02,0x01,0x0F,0x01,0x02,0x02,0x00,//*10
//   55   0x00,0x00,0x00,0xF0,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x1F,0x01,0x01,0x01,0x00,//+11
//   56   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0xB0,0x70,0x00,0x00,0x00,0x00,0x00,//,12
//   57   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x01,0x01,0x01,0x01,//-13
//   58   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x30,0x00,0x00,0x00,0x00,0x00,//.14
//   59   0x00,0x00,0x00,0x00,0x80,0x60,0x18,0x04,0x00,0x60,0x18,0x06,0x01,0x00,0x00,0x00,///15
//   60   0x00,0xE0,0x10,0x08,0x08,0x10,0xE0,0x00,0x00,0x0F,0x10,0x20,0x20,0x10,0x0F,0x00,//016
//   61   0x00,0x10,0x10,0xF8,0x00,0x00,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,//117
//   62   0x00,0x70,0x08,0x08,0x08,0x88,0x70,0x00,0x00,0x30,0x28,0x24,0x22,0x21,0x30,0x00,//218
//   63   0x00,0x30,0x08,0x88,0x88,0x48,0x30,0x00,0x00,0x18,0x20,0x20,0x20,0x11,0x0E,0x00,//319
//   64   0x00,0x00,0xC0,0x20,0x10,0xF8,0x00,0x00,0x00,0x07,0x04,0x24,0x24,0x3F,0x24,0x00,//420
//   65   0x00,0xF8,0x08,0x88,0x88,0x08,0x08,0x00,0x00,0x19,0x21,0x20,0x20,0x11,0x0E,0x00,//521
//   66   0x00,0xE0,0x10,0x88,0x88,0x18,0x00,0x00,0x00,0x0F,0x11,0x20,0x20,0x11,0x0E,0x00,//622
//   67   0x00,0x38,0x08,0x08,0xC8,0x38,0x08,0x00,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x00,//723
//   68   0x00,0x70,0x88,0x08,0x08,0x88,0x70,0x00,0x00,0x1C,0x22,0x21,0x21,0x22,0x1C,0x00,//824
//   69   0x00,0xE0,0x10,0x08,0x08,0x10,0xE0,0x00,0x00,0x00,0x31,0x22,0x22,0x11,0x0F,0x00,//925
//   70   0x00,0x00,0x00,0xC0,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x30,0x00,0x00,0x00,//:26
//   71   0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x60,0x00,0x00,0x00,0x00,//;27
//   72   0x00,0x00,0x80,0x40,0x20,0x10,0x08,0x00,0x00,0x01,0x02,0x04,0x08,0x10,0x20,0x00,//<28
//   73   0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x00,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x00,//=29
//   74   0x00,0x08,0x10,0x20,0x40,0x80,0x00,0x00,0x00,0x20,0x10,0x08,0x04,0x02,0x01,0x00,//>30
//   75   0x00,0x70,0x48,0x08,0x08,0x08,0xF0,0x00,0x00,0x00,0x00,0x30,0x36,0x01,0x00,0x00,//?31
//   76   0xC0,0x30,0xC8,0x28,0xE8,0x10,0xE0,0x00,0x07,0x18,0x27,0x24,0x23,0x14,0x0B,0x00,//@32
//   77   0x00,0x00,0xC0,0x38,0xE0,0x00,0x00,0x00,0x20,0x3C,0x23,0x02,0x02,0x27,0x38,0x20,//A33
//   78   0x08,0xF8,0x88,0x88,0x88,0x70,0x00,0x00,0x20,0x3F,0x20,0x20,0x20,0x11,0x0E,0x00,//B34
//   79   0xC0,0x30,0x08,0x08,0x08,0x08,0x38,0x00,0x07,0x18,0x20,0x20,0x20,0x10,0x08,0x00,//C35
//   80   0x08,0xF8,0x08,0x08,0x08,0x10,0xE0,0x00,0x20,0x3F,0x20,0x20,0x20,0x10,0x0F,0x00,//D36
//   81   0x08,0xF8,0x88,0x88,0xE8,0x08,0x10,0x00,0x20,0x3F,0x20,0x20,0x23,0x20,0x18,0x00,//E37
//   82   0x08,0xF8,0x88,0x88,0xE8,0x08,0x10,0x00,0x20,0x3F,0x20,0x00,0x03,0x00,0x00,0x00,//F38
//   83   0xC0,0x30,0x08,0x08,0x08,0x38,0x00,0x00,0x07,0x18,0x20,0x20,0x22,0x1E,0x02,0x00,//G39
//   84   0x08,0xF8,0x08,0x00,0x00,0x08,0xF8,0x08,0x20,0x3F,0x21,0x01,0x01,0x21,0x3F,0x20,//H40
//   85   0x00,0x08,0x08,0xF8,0x08,0x08,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,//I41
//   86   0x00,0x00,0x08,0x08,0xF8,0x08,0x08,0x00,0xC0,0x80,0x80,0x80,0x7F,0x00,0x00,0x00,//J42
//   87   0x08,0xF8,0x88,0xC0,0x28,0x18,0x08,0x00,0x20,0x3F,0x20,0x01,0x26,0x38,0x20,0x00,//K43
//   88   0x08,0xF8,0x08,0x00,0x00,0x00,0x00,0x00,0x20,0x3F,0x20,0x20,0x20,0x20,0x30,0x00,//L44
//   89   0x08,0xF8,0xF8,0x00,0xF8,0xF8,0x08,0x00,0x20,0x3F,0x00,0x3F,0x00,0x3F,0x20,0x00,//M45
//   90   0x08,0xF8,0x30,0xC0,0x00,0x08,0xF8,0x08,0x20,0x3F,0x20,0x00,0x07,0x18,0x3F,0x00,//N46
//   91   0xE0,0x10,0x08,0x08,0x08,0x10,0xE0,0x00,0x0F,0x10,0x20,0x20,0x20,0x10,0x0F,0x00,//O47
//   92   0x08,0xF8,0x08,0x08,0x08,0x08,0xF0,0x00,0x20,0x3F,0x21,0x01,0x01,0x01,0x00,0x00,//P48
//   93   0xE0,0x10,0x08,0x08,0x08,0x10,0xE0,0x00,0x0F,0x18,0x24,0x24,0x38,0x50,0x4F,0x00,//Q49
//   94   0x08,0xF8,0x88,0x88,0x88,0x88,0x70,0x00,0x20,0x3F,0x20,0x00,0x03,0x0C,0x30,0x20,//R50
//   95   0x00,0x70,0x88,0x08,0x08,0x08,0x38,0x00,0x00,0x38,0x20,0x21,0x21,0x22,0x1C,0x00,//S51
//   96   0x18,0x08,0x08,0xF8,0x08,0x08,0x18,0x00,0x00,0x00,0x20,0x3F,0x20,0x00,0x00,0x00,//T52
//   97   0x08,0xF8,0x08,0x00,0x00,0x08,0xF8,0x08,0x00,0x1F,0x20,0x20,0x20,0x20,0x1F,0x00,//U53
//   98   0x08,0x78,0x88,0x00,0x00,0xC8,0x38,0x08,0x00,0x00,0x07,0x38,0x0E,0x01,0x00,0x00,//V54
//   99   0xF8,0x08,0x00,0xF8,0x00,0x08,0xF8,0x00,0x03,0x3C,0x07,0x00,0x07,0x3C,0x03,0x00,//W55
//  100   0x08,0x18,0x68,0x80,0x80,0x68,0x18,0x08,0x20,0x30,0x2C,0x03,0x03,0x2C,0x30,0x20,//X56
//  101   0x08,0x38,0xC8,0x00,0xC8,0x38,0x08,0x00,0x00,0x00,0x20,0x3F,0x20,0x00,0x00,0x00,//Y57
//  102   0x10,0x08,0x08,0x08,0xC8,0x38,0x08,0x00,0x20,0x38,0x26,0x21,0x20,0x20,0x18,0x00,//Z58
//  103   0x00,0x00,0x00,0xFE,0x02,0x02,0x02,0x00,0x00,0x00,0x00,0x7F,0x40,0x40,0x40,0x00,//[59
//  104   0x00,0x0C,0x30,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x06,0x38,0xC0,0x00,//\60
//  105   0x00,0x02,0x02,0x02,0xFE,0x00,0x00,0x00,0x00,0x40,0x40,0x40,0x7F,0x00,0x00,0x00,//]61
//  106   0x00,0x00,0x04,0x02,0x02,0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//^62
//  107   0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,//_63
//  108   0x00,0x02,0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//`64
//  109   0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x19,0x24,0x22,0x22,0x22,0x3F,0x20,//a65
//  110   0x08,0xF8,0x00,0x80,0x80,0x00,0x00,0x00,0x00,0x3F,0x11,0x20,0x20,0x11,0x0E,0x00,//b66
//  111   0x00,0x00,0x00,0x80,0x80,0x80,0x00,0x00,0x00,0x0E,0x11,0x20,0x20,0x20,0x11,0x00,//c67
//  112   0x00,0x00,0x00,0x80,0x80,0x88,0xF8,0x00,0x00,0x0E,0x11,0x20,0x20,0x10,0x3F,0x20,//d68
//  113   0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x1F,0x22,0x22,0x22,0x22,0x13,0x00,//e69
//  114   0x00,0x80,0x80,0xF0,0x88,0x88,0x88,0x18,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,//f70
//  115   0x00,0x00,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x6B,0x94,0x94,0x94,0x93,0x60,0x00,//g71
//  116   0x08,0xF8,0x00,0x80,0x80,0x80,0x00,0x00,0x20,0x3F,0x21,0x00,0x00,0x20,0x3F,0x20,//h72
//  117   0x00,0x80,0x98,0x98,0x00,0x00,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,//i73
//  118   0x00,0x00,0x00,0x80,0x98,0x98,0x00,0x00,0x00,0xC0,0x80,0x80,0x80,0x7F,0x00,0x00,//j74
//  119   0x08,0xF8,0x00,0x00,0x80,0x80,0x80,0x00,0x20,0x3F,0x24,0x02,0x2D,0x30,0x20,0x00,//k75
//  120   0x00,0x08,0x08,0xF8,0x00,0x00,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,//l76
//  121   0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x00,0x20,0x3F,0x20,0x00,0x3F,0x20,0x00,0x3F,//m77
//  122   0x80,0x80,0x00,0x80,0x80,0x80,0x00,0x00,0x20,0x3F,0x21,0x00,0x00,0x20,0x3F,0x20,//n78
//  123   0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x1F,0x20,0x20,0x20,0x20,0x1F,0x00,//o79
//  124   0x80,0x80,0x00,0x80,0x80,0x00,0x00,0x00,0x80,0xFF,0xA1,0x20,0x20,0x11,0x0E,0x00,//p80
//  125   0x00,0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x0E,0x11,0x20,0x20,0xA0,0xFF,0x80,//q81
//  126   0x80,0x80,0x80,0x00,0x80,0x80,0x80,0x00,0x20,0x20,0x3F,0x21,0x20,0x00,0x01,0x00,//r82
//  127   0x00,0x00,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x33,0x24,0x24,0x24,0x24,0x19,0x00,//s83
//  128   0x00,0x80,0x80,0xE0,0x80,0x80,0x00,0x00,0x00,0x00,0x00,0x1F,0x20,0x20,0x00,0x00,//t84
//  129   0x80,0x80,0x00,0x00,0x00,0x80,0x80,0x00,0x00,0x1F,0x20,0x20,0x20,0x10,0x3F,0x20,//byte5
//  130   0x80,0x80,0x80,0x00,0x00,0x80,0x80,0x80,0x00,0x01,0x0E,0x30,0x08,0x06,0x01,0x00,//v86
//  131   0x80,0x80,0x00,0x80,0x00,0x80,0x80,0x80,0x0F,0x30,0x0C,0x03,0x0C,0x30,0x0F,0x00,//w87
//  132   0x00,0x80,0x80,0x00,0x80,0x80,0x80,0x00,0x00,0x20,0x31,0x2E,0x0E,0x31,0x20,0x00,//x88
//  133   0x80,0x80,0x80,0x00,0x00,0x80,0x80,0x80,0x80,0x81,0x8E,0x70,0x18,0x06,0x01,0x00,//y89
//  134   0x00,0x80,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x21,0x30,0x2C,0x22,0x21,0x30,0x00,//z90
//  135   0x00,0x00,0x00,0x00,0x80,0x7C,0x02,0x02,0x00,0x00,0x00,0x00,0x00,0x3F,0x40,0x40,//{91
//  136   0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,//|92
//  137   0x00,0x02,0x02,0x7C,0x80,0x00,0x00,0x00,0x00,0x40,0x40,0x3F,0x00,0x00,0x00,0x00,//}93
//  138   0x00,0x06,0x01,0x01,0x02,0x02,0x04,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//~94
//  139   0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xff,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xff,//95
//  140   0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00 ,//96
//  141 
//  142 };
//  143 
//  144 const uint8 F14x16_Idx[] = 
//  145 {
//  146 	"北京联合大学实训基地潘峰智能车按键秒"
//  147 };
//  148 
//  149 /**************************************************
//  150 PCTOLCD2002
//  151 新宋体，宽14，高16，阴码，逆向，行列模式，c51格式，无行前缀和后缀
//  152 ***************************************************/
//  153 const uint8 F14x16[] =
//  154 {  
//  155   0x00,0x00,0x40,0x40,0x40,0xFC,0x00,0x00,0xFC,0x80,0x40,0x20,0x20,0x00,
//  156   0x00,0x30,0x10,0x10,0x08,0x7F,0x00,0x00,0x7F,0x40,0x40,0x40,0x78,0x00,/*"北",0*/
//  157   0x00,0x00,0x10,0xF0,0x50,0x50,0x4A,0x54,0x50,0x50,0xF0,0x08,0x08,0x00,
//  158   0x00,0x40,0x20,0x13,0x0A,0x42,0xFE,0x02,0x02,0x0A,0x13,0x60,0x00,0x00,/*"京",1*/
//  159   0x00,0x00,0xF8,0x48,0xF8,0x04,0x24,0x26,0x28,0xF0,0x2C,0x22,0x80,0x00,
//  160   0x00,0x10,0x1F,0x12,0x7F,0x89,0x41,0x21,0x1D,0x07,0x19,0x21,0x41,0x00,/*"联",2*/
//  161   0x00,0x00,0x80,0x40,0xA0,0x90,0x8E,0x88,0x90,0xA0,0x40,0xC0,0x80,0x00,
//  162   0x00,0x01,0x00,0x7E,0x44,0x44,0x44,0x44,0x44,0x3C,0x01,0x00,0x00,0x00,/*"合",3*/
//  163   0x00,0x20,0x20,0x20,0x20,0x20,0xFE,0xA0,0x20,0x20,0x20,0x20,0x20,0x00,
//  164   0x00,0x80,0x40,0x20,0x10,0x0C,0x03,0x03,0x04,0x18,0x30,0x60,0x40,0x00,/*"大",4*/
//  165   0x00,0xE0,0x20,0xA4,0x98,0xA0,0x9C,0xA8,0x90,0xA8,0xA6,0x60,0x20,0x00,
//  166   0x00,0x00,0x08,0x08,0x08,0x48,0x7C,0x0A,0x09,0x08,0x08,0x04,0x00,0x00,/*"学",5*/
//  167   0x00,0x20,0x38,0x10,0x30,0xD0,0x0A,0xD4,0x30,0x10,0x10,0x10,0x18,0x00,
//  168   0x00,0x84,0x84,0x85,0x47,0x24,0x14,0x0F,0x14,0x24,0x24,0x44,0x04,0x00,/*"实",0*/
//  169   0x00,0x40,0x44,0xCC,0x00,0x00,0xFC,0x00,0xF8,0x00,0x00,0xFE,0x00,0x00,
//  170   0x00,0x00,0x00,0x7F,0x28,0x18,0x07,0x00,0x0F,0x00,0x00,0x7F,0x00,0x00,/*"训",1*/
//  171   0x00,0x08,0x08,0x08,0xFE,0xA8,0xA8,0xA8,0xA8,0xFC,0x08,0x08,0x08,0x00,
//  172   0x00,0x12,0x4A,0x46,0x53,0x52,0x7E,0x52,0x52,0x4D,0x4A,0x4A,0x19,0x00,/*"基",2*/
//  173   0x00,0x40,0xFC,0x40,0x40,0x80,0xF8,0x80,0xFE,0x40,0x20,0xF0,0x00,0x00,
//  174   0x00,0x20,0x1F,0x10,0x10,0x08,0x7F,0x40,0x4F,0x40,0x44,0x47,0x78,0x00,/*"地",3*/
//  175   0x00,0x40,0x84,0x60,0x40,0x2C,0xD4,0x44,0xFC,0x74,0xCA,0x40,0x20,0x00,
//  176   0x00,0x08,0x7F,0x04,0x02,0xFF,0x54,0x54,0x7D,0x54,0x7E,0x03,0x01,0x00,/*"潘",4*/
//  177   0x00,0xF0,0x00,0xFC,0xE0,0x20,0x10,0x8C,0x5A,0xA8,0x58,0x8C,0x80,0x00,
//  178   0x00,0x1F,0x08,0x0F,0x06,0x01,0x01,0x22,0x2A,0x7F,0x2A,0x25,0x10,0x00,/*"峰",5*/
//  179   0x00,0x30,0x28,0xA6,0x78,0x68,0xA4,0x28,0xFC,0x88,0x88,0xF8,0x00,0x00,
//  180   0x00,0x02,0x01,0x00,0x7E,0x52,0x53,0x52,0x53,0x52,0x7E,0x00,0x00,0x00,/*"智",6*/
//  181   0x00,0x70,0xA8,0xA4,0xA0,0xE8,0x30,0x00,0x7C,0x90,0x88,0x88,0x60,0x00,
//  182   0x00,0x00,0x7F,0x0A,0x4A,0x7F,0x00,0x00,0x7E,0x48,0x44,0x42,0x70,0x00,/*"能",7*/
//  183   0x00,0x08,0x08,0xC8,0x38,0x0C,0xCA,0x28,0x08,0x08,0x88,0x08,0x08,0x00,
//  184   0x00,0x08,0x08,0x08,0x09,0x09,0x0F,0x79,0x09,0x09,0x08,0x08,0x08,0x00,/*"车",8*/
//  185   0x00,0x00,0x20,0xFE,0x90,0x20,0x18,0x90,0x6E,0x10,0x10,0x30,0x18,0x00,
//  186   0x00,0x42,0x42,0x7F,0x80,0x81,0x47,0x49,0x31,0x19,0x17,0x21,0x40,0x00,/*"按",9*/
//  187   0x40,0x30,0xCE,0x50,0x48,0xE8,0x98,0xA8,0xA8,0xFE,0xA8,0xF8,0x10,0x00,
//  188   0x00,0x02,0x7F,0xA2,0x52,0x24,0x1E,0x29,0x4A,0x7F,0x4A,0x49,0x44,0x00,/*"键",10*/
//  189   0x00,0x00,0x48,0xFC,0x44,0x20,0x80,0x70,0x00,0xFE,0x00,0x20,0xE0,0x00,
//  190   0x08,0x04,0x03,0x7F,0x81,0x81,0x41,0x40,0x20,0x17,0x08,0x07,0x02,0x00,/*"秒",11*/  
//  191 };
//  192 
//  193 

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  194 void LCD_WrCmd(unsigned char cmd)
//  195 {
//  196 	GPIOC_PDOR&=~A0_MASK;//命令
LCD_WrCmd:
        LDR.N    R1,??DataTable6  ;; 0x400ff080
        LDR      R1,[R1, #+0]
        BICS     R1,R1,#0x100
        LDR.N    R2,??DataTable6  ;; 0x400ff080
        STR      R1,[R2, #+0]
//  197 	SPI0_PUSHR=pusher0+cmd;
        LDR.N    R1,??DataTable6_1
        LDR      R1,[R1, #+0]
        UXTAB    R0,R1,R0
        LDR.N    R1,??DataTable6_2  ;; 0x4002c034
        STR      R0,[R1, #+0]
//  198 	while((SPI0_SR&SPI_SR_TCF_MASK)==0);//等待传送完成
??LCD_WrCmd_0:
        LDR.N    R0,??DataTable6_3  ;; 0x4002c02c
        LDR      R0,[R0, #+0]
        CMP      R0,#+0
        BPL.N    ??LCD_WrCmd_0
//  199 	SPI0_SR|=SPI_SR_TCF_MASK;//清除标志	
        LDR.N    R0,??DataTable6_3  ;; 0x4002c02c
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x80000000
        LDR.N    R1,??DataTable6_3  ;; 0x4002c02c
        STR      R0,[R1, #+0]
//  200 }
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  201 void LCD_WrDat(unsigned char data)
//  202 {
//  203 	GPIOC_PDOR|=A0_MASK;//数据
LCD_WrDat:
        LDR.N    R1,??DataTable6  ;; 0x400ff080
        LDR      R1,[R1, #+0]
        MOV      R2,#+256
        ORRS     R1,R2,R1
        LDR.N    R2,??DataTable6  ;; 0x400ff080
        STR      R1,[R2, #+0]
//  204 	SPI0_PUSHR=pusher0+data;
        LDR.N    R1,??DataTable6_1
        LDR      R1,[R1, #+0]
        UXTAB    R0,R1,R0
        LDR.N    R1,??DataTable6_2  ;; 0x4002c034
        STR      R0,[R1, #+0]
//  205 	while((SPI0_SR&SPI_SR_TCF_MASK)==0);//等待传送完成
??LCD_WrDat_0:
        LDR.N    R0,??DataTable6_3  ;; 0x4002c02c
        LDR      R0,[R0, #+0]
        CMP      R0,#+0
        BPL.N    ??LCD_WrDat_0
//  206 	SPI0_SR|=SPI_SR_TCF_MASK;//清除标志	
        LDR.N    R0,??DataTable6_3  ;; 0x4002c02c
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x80000000
        LDR.N    R1,??DataTable6_3  ;; 0x4002c02c
        STR      R0,[R1, #+0]
//  207 }
        BX       LR               ;; return
//  208 

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  209 void LCD_Set_Pos(uint8 x, uint8 y)
//  210 { 
LCD_Set_Pos:
        PUSH     {R4,LR}
        MOVS     R4,R0
//  211   LCD_WrCmd(0xb0+y);
        SUBS     R0,R1,#+80
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        BL       LCD_WrCmd
//  212   LCD_WrCmd(((x&0xf0)>>4)|0x10);
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LSRS     R0,R4,#+4
        ORRS     R0,R0,#0x10
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        BL       LCD_WrCmd
//  213   LCD_WrCmd((x&0x0f)); 
        ANDS     R0,R4,#0xF
        BL       LCD_WrCmd
//  214 }
        POP      {R4,PC}          ;; return
//  215 
//  216  

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  217 void LCD_Fill(uint8 bmp_data)
//  218 {
LCD_Fill:
        PUSH     {R4-R6,LR}
        MOVS     R4,R0
//  219 	uint8 y,x;
//  220 	
//  221 	for(y=0;y<8;y++)
        MOVS     R5,#+0
        B.N      ??LCD_Fill_0
//  222 	{
//  223 		LCD_WrCmd(0xb0+y);
//  224 		LCD_WrCmd(0x01);
//  225 		LCD_WrCmd(0x10);
//  226 		for(x=0;x<X_WIDTH;x++)
//  227 			LCD_WrDat(bmp_data);
??LCD_Fill_1:
        MOVS     R0,R4
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        BL       LCD_WrDat
        ADDS     R6,R6,#+1
??LCD_Fill_2:
        UXTB     R6,R6            ;; ZeroExt  R6,R6,#+24,#+24
        CMP      R6,#+128
        BCC.N    ??LCD_Fill_1
        ADDS     R5,R5,#+1
??LCD_Fill_0:
        UXTB     R5,R5            ;; ZeroExt  R5,R5,#+24,#+24
        CMP      R5,#+8
        BCS.N    ??LCD_Fill_3
        SUBS     R0,R5,#+80
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        BL       LCD_WrCmd
        MOVS     R0,#+1
        BL       LCD_WrCmd
        MOVS     R0,#+16
        BL       LCD_WrCmd
        MOVS     R6,#+0
        B.N      ??LCD_Fill_2
//  228 	}
//  229 }
??LCD_Fill_3:
        POP      {R4-R6,PC}       ;; return
//  230 
//  231 
//  232 
//  233 

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  234 void LCD_DLY_ms(uint16 ms)
//  235 {                         
LCD_DLY_ms:
        B.N      ??LCD_DLY_ms_0
//  236   uint16 a;
//  237   while(ms)
//  238   {
//  239     a=1335;
??LCD_DLY_ms_1:
        MOVW     R1,#+1335
//  240     while(a--);
??LCD_DLY_ms_2:
        MOVS     R2,R1
        SUBS     R1,R2,#+1
        UXTH     R2,R2            ;; ZeroExt  R2,R2,#+16,#+16
        CMP      R2,#+0
        BNE.N    ??LCD_DLY_ms_2
//  241     ms--;
        SUBS     R0,R0,#+1
//  242   }
??LCD_DLY_ms_0:
        UXTH     R0,R0            ;; ZeroExt  R0,R0,#+16,#+16
        CMP      R0,#+0
        BNE.N    ??LCD_DLY_ms_1
//  243   return;
        BX       LR               ;; return
//  244 }

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  245 void LCD_Init(void)        
//  246 {
LCD_Init:
        PUSH     {R7,LR}
//  247 	LCD_DLY_ms(350);
        MOV      R0,#+350
        BL       LCD_DLY_ms
//  248 		
//  249 	GPIOC_PDOR&=~RESET_MASK;
        LDR.N    R0,??DataTable6  ;; 0x400ff080
        LDR      R0,[R0, #+0]
        BICS     R0,R0,#0x400
        LDR.N    R1,??DataTable6  ;; 0x400ff080
        STR      R0,[R1, #+0]
//  250 	LCD_DLY_ms(900);
        MOV      R0,#+900
        BL       LCD_DLY_ms
//  251 	GPIOC_PDOR|=RESET_MASK;
        LDR.N    R0,??DataTable6  ;; 0x400ff080
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x400
        LDR.N    R1,??DataTable6  ;; 0x400ff080
        STR      R0,[R1, #+0]
//  252 	LCD_DLY_ms(600); 	
        MOV      R0,#+600
        BL       LCD_DLY_ms
//  253 	LCD_WrCmd(0xaf);	//0xaf  LCD On
        MOVS     R0,#+175
        BL       LCD_WrCmd
//  254 	LCD_DLY_ms(600);
        MOV      R0,#+600
        BL       LCD_DLY_ms
//  255 	LCD_WrCmd(0x2F);	//0x2f  设置上电控制模式
        MOVS     R0,#+47
        BL       LCD_WrCmd
//  256 	LCD_DLY_ms(200);  	
        MOVS     R0,#+200
        BL       LCD_DLY_ms
//  257 	LCD_WrCmd(0x81);	//0x81  电量设置模式（显示亮度）
        MOVS     R0,#+129
        BL       LCD_WrCmd
//  258 	LCD_WrCmd(0x20);	//指令数据0x0000~0x003f  	
        MOVS     R0,#+32
        BL       LCD_WrCmd
//  259 	LCD_WrCmd(0x24);	//0x24  V5内部电压调节电阻设置
        MOVS     R0,#+36
        BL       LCD_WrCmd
//  260 	LCD_WrCmd(0xa2);	//0xa2 5V,0xa1 3V3     LCD偏压设置   	
        MOVS     R0,#+162
        BL       LCD_WrCmd
//  261 	LCD_WrCmd(0xc8);	//0xc8 正常,0xc0 倒置  Com 扫描方式设置
        MOVS     R0,#+200
        BL       LCD_WrCmd
//  262 	LCD_WrCmd(0xa0);	//0xa0 正常,0xa1 反向  Segment方向选择
        MOVS     R0,#+160
        BL       LCD_WrCmd
//  263 	LCD_WrCmd(0xa4);	//0xa4  全屏点亮/变暗指令
        MOVS     R0,#+164
        BL       LCD_WrCmd
//  264 	LCD_WrCmd(0xa6);	//0xa6  正向反向显示控制指令   	
        MOVS     R0,#+166
        BL       LCD_WrCmd
//  265 	LCD_WrCmd(0xac);	//0xac  关闭静态指示器
        MOVS     R0,#+172
        BL       LCD_WrCmd
//  266 	LCD_WrCmd(0x00);	//指令数据    	
        MOVS     R0,#+0
        BL       LCD_WrCmd
//  267 	LCD_WrCmd(0x40);	//0x40	设置显示起始行对应RAM
        MOVS     R0,#+64
        BL       LCD_WrCmd
//  268 	LCD_Fill(0x00);  //初始清屏
        MOVS     R0,#+0
        BL       LCD_Fill
//  269 	LCD_Set_Pos(0,0); 	
        MOVS     R1,#+0
        MOVS     R0,#+0
        BL       LCD_Set_Pos
//  270 }
        POP      {R0,PC}          ;; return
//  271 
//  272 //==============================================================
//  273 //函数名：LCD_P8x16chr(byte x,byte  y,byte ch)
//  274 //功能描述：写入一个标准ASCII字符
//  275 //参数：显示的位置（x,y），y为页范围0～7，要显示的字符
//  276 //返回：无
//  277 //==============================================================

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  278 void LCD_P8x16chr(uint8 x,uint8  y,uint8 ch) 
//  279 {
LCD_P8x16chr:
        PUSH     {R3-R7,LR}
        MOVS     R4,R0
        MOVS     R5,R1
//  280   uint8 i;
//  281   uint8 c ;
//  282   c=ch-32;  
        SUBS     R6,R2,#+32
//  283   LCD_Set_Pos(x,y);
        MOVS     R1,R5
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        MOVS     R0,R4
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        BL       LCD_Set_Pos
//  284   GPIOC_PDOR|=A0_MASK;//数据
        LDR.N    R0,??DataTable6  ;; 0x400ff080
        LDR      R0,[R0, #+0]
        MOV      R1,#+256
        ORRS     R0,R1,R0
        LDR.N    R1,??DataTable6  ;; 0x400ff080
        STR      R0,[R1, #+0]
//  285   for(i=0;i<8;i++)//显示字符上半部分
        MOVS     R7,#+0
        B.N      ??LCD_P8x16chr_0
//  286   {
//  287 	  LCD_WrDat(F8X16[c*16+i]); 	  
??LCD_P8x16chr_1:
        UXTB     R6,R6            ;; ZeroExt  R6,R6,#+24,#+24
        LSLS     R0,R6,#+4
        UXTAB    R0,R0,R7
        ADR.W    R1,F8X16
        LDRB     R0,[R0, R1]
        BL       LCD_WrDat
//  288   }  
        ADDS     R7,R7,#+1
??LCD_P8x16chr_0:
        UXTB     R7,R7            ;; ZeroExt  R7,R7,#+24,#+24
        CMP      R7,#+8
        BCC.N    ??LCD_P8x16chr_1
//  289   LCD_Set_Pos(x,y+1);    
        ADDS     R1,R5,#+1
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        MOVS     R0,R4
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        BL       LCD_Set_Pos
//  290   for(i=0;i<8;i++) //显示字符下半部分    
        MOVS     R7,#+0
        B.N      ??LCD_P8x16chr_2
//  291   	  LCD_WrDat(F8X16[c*16+i+8]); 
??LCD_P8x16chr_3:
        UXTB     R6,R6            ;; ZeroExt  R6,R6,#+24,#+24
        LSLS     R0,R6,#+4
        UXTAB    R0,R0,R7
        ADR.W    R1,F8X16
        ADDS     R0,R0,R1
        LDRB     R0,[R0, #+8]
        BL       LCD_WrDat
        ADDS     R7,R7,#+1
??LCD_P8x16chr_2:
        UXTB     R7,R7            ;; ZeroExt  R7,R7,#+24,#+24
        CMP      R7,#+8
        BCC.N    ??LCD_P8x16chr_3
//  292 }
        POP      {R0,R4-R7,PC}    ;; return

        SECTION `.text`:CODE:NOROOT(2)
        DATA
??DataTable6:
        DC32     0x400ff080

        SECTION `.text`:CODE:NOROOT(2)
        DATA
??DataTable6_1:
        DC32     pusher0

        SECTION `.text`:CODE:NOROOT(2)
        DATA
??DataTable6_2:
        DC32     0x4002c034

        SECTION `.text`:CODE:NOROOT(2)
        DATA
??DataTable6_3:
        DC32     0x4002c02c
//  293 
//  294 //==============================================================
//  295 //函数名：LCD_P8x16Str(byte x,byte y,byte *p)
//  296 //功能描述：写入一组标准ASCII字符串
//  297 //参数：显示的位置（x,y），y为页范围0～7，要显示的字符串
//  298 //返回：无
//  299 //==============================================================  

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  300 void LCD_P8x16Str(uint8 x,uint8 y,char *ch)
//  301 {
LCD_P8x16Str:
        PUSH     {R3-R9,LR}
        MOVS     R4,R0
        MOVS     R5,R1
        MOVS     R6,R2
//  302 	uint8 c=0,i=0,j=0;
        MOVS     R7,#+0
        MOVS     R8,#+0
        MOVS     R9,#+0
        B.N      ??LCD_P8x16Str_0
//  303         
//  304   while (ch[j]!='\0')
//  305   {    
//  306     c =ch[j]-32;
//  307     if(x>120){x=0;y++;}
//  308     LCD_Set_Pos(x,y);    
//  309   	for(i=0;i<8;i++)     
//  310   	  LCD_WrDat(F8X16[c*16+i]);
//  311   	LCD_Set_Pos(x,y+1);    
//  312   	for(i=0;i<8;i++)     
//  313   	  LCD_WrDat(F8X16[c*16+i+8]);  
??LCD_P8x16Str_1:
        UXTB     R7,R7            ;; ZeroExt  R7,R7,#+24,#+24
        LSLS     R0,R7,#+4
        UXTAB    R0,R0,R8
        ADR.W    R1,F8X16
        ADDS     R0,R0,R1
        LDRB     R0,[R0, #+8]
        BL       LCD_WrDat
        ADDS     R8,R8,#+1
??LCD_P8x16Str_2:
        UXTB     R8,R8            ;; ZeroExt  R8,R8,#+24,#+24
        CMP      R8,#+8
        BCC.N    ??LCD_P8x16Str_1
//  314   	x+=8;
        ADDS     R4,R4,#+8
//  315   	j++;
        ADDS     R9,R9,#+1
??LCD_P8x16Str_0:
        UXTB     R9,R9            ;; ZeroExt  R9,R9,#+24,#+24
        LDRB     R0,[R9, R6]
        CMP      R0,#+0
        BEQ.N    ??LCD_P8x16Str_3
        UXTB     R9,R9            ;; ZeroExt  R9,R9,#+24,#+24
        LDRB     R0,[R9, R6]
        SUBS     R7,R0,#+32
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+121
        BCC.N    ??LCD_P8x16Str_4
        MOVS     R4,#+0
        ADDS     R5,R5,#+1
??LCD_P8x16Str_4:
        MOVS     R1,R5
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        MOVS     R0,R4
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        BL       LCD_Set_Pos
        MOVS     R8,#+0
        B.N      ??LCD_P8x16Str_5
??LCD_P8x16Str_6:
        UXTB     R7,R7            ;; ZeroExt  R7,R7,#+24,#+24
        LSLS     R0,R7,#+4
        UXTAB    R0,R0,R8
        ADR.W    R1,F8X16
        LDRB     R0,[R0, R1]
        BL       LCD_WrDat
        ADDS     R8,R8,#+1
??LCD_P8x16Str_5:
        UXTB     R8,R8            ;; ZeroExt  R8,R8,#+24,#+24
        CMP      R8,#+8
        BCC.N    ??LCD_P8x16Str_6
        ADDS     R1,R5,#+1
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        MOVS     R0,R4
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        BL       LCD_Set_Pos
        MOVS     R8,#+0
        B.N      ??LCD_P8x16Str_2
//  316   }
//  317 }
??LCD_P8x16Str_3:
        POP      {R0,R4-R9,PC}    ;; return
//  318 //输出汉字字符串

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  319 void LCD_P14x16Str(uint8 x,uint8 y,uint8 ch[])
//  320 {
LCD_P14x16Str:
        PUSH     {R3-R9,LR}
        MOVS     R6,R0
        MOVS     R4,R1
        MOVS     R5,R2
//  321 	uint8 wm=0,ii = 0;
        MOVS     R9,#+0
        MOVS     R7,#+0
//  322 	uint16 adder=1; 
        MOVS     R8,#+1
        B.N      ??LCD_P14x16Str_0
//  323 	
//  324 	while(ch[ii] != '\0')
//  325 	{
//  326   	wm = 0;
//  327   	adder = 1;
//  328   	while(F14x16_Idx[wm] > 127)
//  329   	{
//  330   		if(F14x16_Idx[wm] == ch[ii])
//  331   		{
//  332   			if(F14x16_Idx[wm + 1] == ch[ii + 1])
//  333   			{
//  334   				adder = wm * 14;
//  335   				break;
//  336   			}
//  337   		}
//  338   		wm += 2;			
//  339   	}
//  340   	if(x>118){x=0;y++;}
//  341   	LCD_Set_Pos(x , y); 
//  342   	if(adder != 1)// 显示汉字					
//  343   	{
//  344   		LCD_Set_Pos(x , y);
//  345   		for(wm = 0;wm < 14;wm++)               
//  346   		{
//  347   			LCD_WrDat(F14x16[adder]);	
//  348   			adder += 1;
//  349   		}      
//  350   		LCD_Set_Pos(x,y + 1); 
//  351   		for(wm = 0;wm < 14;wm++)          
//  352   		{
//  353   			LCD_WrDat(F14x16[adder]);
//  354   			adder += 1;
//  355   		}   		
//  356   	}
//  357   	else			  //显示空白字符			
//  358   	{
//  359   		ii += 1;
//  360       LCD_Set_Pos(x,y);
//  361   		for(wm = 0;wm < 16;wm++)
//  362   		{
//  363   				LCD_WrDat(0);
//  364   		}
//  365   		LCD_Set_Pos(x,y + 1);
//  366   		for(wm = 0;wm < 16;wm++)
//  367   		{   		
//  368   				LCD_WrDat(0);	
??LCD_P14x16Str_1:
        MOVS     R0,#+0
        BL       LCD_WrDat
//  369   		}
        ADDS     R9,R9,#+1
??LCD_P14x16Str_2:
        UXTB     R9,R9            ;; ZeroExt  R9,R9,#+24,#+24
        CMP      R9,#+16
        BCC.N    ??LCD_P14x16Str_1
//  370   	}
//  371   	x += 14;
??LCD_P14x16Str_3:
        ADDS     R6,R6,#+14
//  372   	ii += 2;
        ADDS     R7,R7,#+2
??LCD_P14x16Str_0:
        UXTB     R7,R7            ;; ZeroExt  R7,R7,#+24,#+24
        LDRB     R0,[R7, R5]
        CMP      R0,#+0
        BEQ.W    ??LCD_P14x16Str_4
        MOVS     R9,#+0
        MOVS     R8,#+1
        B.N      ??LCD_P14x16Str_5
??LCD_P14x16Str_6:
        ADDS     R9,R9,#+2
??LCD_P14x16Str_5:
        UXTB     R9,R9            ;; ZeroExt  R9,R9,#+24,#+24
        ADR.W    R0,F14x16_Idx
        LDRB     R0,[R9, R0]
        CMP      R0,#+128
        BCC.N    ??LCD_P14x16Str_7
        UXTB     R9,R9            ;; ZeroExt  R9,R9,#+24,#+24
        ADR.W    R0,F14x16_Idx
        LDRB     R0,[R9, R0]
        UXTB     R7,R7            ;; ZeroExt  R7,R7,#+24,#+24
        LDRB     R1,[R7, R5]
        CMP      R0,R1
        BNE.N    ??LCD_P14x16Str_6
        UXTB     R9,R9            ;; ZeroExt  R9,R9,#+24,#+24
        ADR.W    R0,F14x16_Idx
        ADDS     R0,R9,R0
        LDRB     R0,[R0, #+1]
        UXTB     R7,R7            ;; ZeroExt  R7,R7,#+24,#+24
        ADDS     R1,R7,R5
        LDRB     R1,[R1, #+1]
        CMP      R0,R1
        BNE.N    ??LCD_P14x16Str_6
        UXTB     R9,R9            ;; ZeroExt  R9,R9,#+24,#+24
        MOVS     R0,#+14
        MUL      R8,R0,R9
??LCD_P14x16Str_7:
        UXTB     R6,R6            ;; ZeroExt  R6,R6,#+24,#+24
        CMP      R6,#+119
        BCC.N    ??LCD_P14x16Str_8
        MOVS     R6,#+0
        ADDS     R4,R4,#+1
??LCD_P14x16Str_8:
        MOVS     R1,R4
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        MOVS     R0,R6
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        BL       LCD_Set_Pos
        UXTH     R8,R8            ;; ZeroExt  R8,R8,#+16,#+16
        CMP      R8,#+1
        BEQ.N    ??LCD_P14x16Str_9
        MOVS     R1,R4
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        MOVS     R0,R6
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        BL       LCD_Set_Pos
        MOVS     R9,#+0
        B.N      ??LCD_P14x16Str_10
??LCD_P14x16Str_11:
        UXTH     R8,R8            ;; ZeroExt  R8,R8,#+16,#+16
        ADR.W    R0,F14x16
        LDRB     R0,[R8, R0]
        BL       LCD_WrDat
        ADDS     R8,R8,#+1
        ADDS     R9,R9,#+1
??LCD_P14x16Str_10:
        UXTB     R9,R9            ;; ZeroExt  R9,R9,#+24,#+24
        CMP      R9,#+14
        BCC.N    ??LCD_P14x16Str_11
        ADDS     R1,R4,#+1
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        MOVS     R0,R6
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        BL       LCD_Set_Pos
        MOVS     R9,#+0
??LCD_P14x16Str_12:
        UXTB     R9,R9            ;; ZeroExt  R9,R9,#+24,#+24
        CMP      R9,#+14
        BCS.N    ??LCD_P14x16Str_3
        UXTH     R8,R8            ;; ZeroExt  R8,R8,#+16,#+16
        ADR.W    R0,F14x16
        LDRB     R0,[R8, R0]
        BL       LCD_WrDat
        ADDS     R8,R8,#+1
        ADDS     R9,R9,#+1
        B.N      ??LCD_P14x16Str_12
??LCD_P14x16Str_9:
        ADDS     R7,R7,#+1
        MOVS     R1,R4
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        MOVS     R0,R6
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        BL       LCD_Set_Pos
        MOVS     R9,#+0
        B.N      ??LCD_P14x16Str_13
??LCD_P14x16Str_14:
        MOVS     R0,#+0
        BL       LCD_WrDat
        ADDS     R9,R9,#+1
??LCD_P14x16Str_13:
        UXTB     R9,R9            ;; ZeroExt  R9,R9,#+24,#+24
        CMP      R9,#+16
        BCC.N    ??LCD_P14x16Str_14
        ADDS     R1,R4,#+1
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        MOVS     R0,R6
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        BL       LCD_Set_Pos
        MOVS     R9,#+0
        B.N      ??LCD_P14x16Str_2
//  373 	}
//  374 }
??LCD_P14x16Str_4:
        POP      {R0,R4-R9,PC}    ;; return

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  375 void displaypara(int val,unsigned char row,unsigned char x) 
//  376 {
displaypara:
        PUSH     {R4,R5,LR}
        SUB      SP,SP,#+12
        MOVS     R4,R1
        MOVS     R5,R2
//  377   char n[5];
//  378   if(val<0)//负数 
        CMP      R0,#+0
        BPL.N    ??displaypara_0
//  379   {
//  380     n[4]='-'-48;
        MOVS     R1,#+253
        STRB     R1,[SP, #+4]
//  381     val=-val;
        RSBS     R0,R0,#+0
        B.N      ??displaypara_1
//  382   } 
//  383   else
//  384   n[4]=(char)(val/10000); 
??displaypara_0:
        MOVW     R1,#+10000
        SDIV     R1,R0,R1
        STRB     R1,[SP, #+4]
//  385   n[0]=(char)(val%10);
??displaypara_1:
        MOVS     R1,#+10
        SDIV     R2,R0,R1
        MLS      R1,R1,R2,R0
        STRB     R1,[SP, #+0]
//  386   n[1]=(char)((val/10)%10);
        MOVS     R1,#+10
        SDIV     R1,R0,R1
        MOVS     R2,#+10
        SDIV     R3,R1,R2
        MLS      R1,R2,R3,R1
        STRB     R1,[SP, #+1]
//  387   n[2]=(char)((val/100)%10);
        MOVS     R1,#+100
        SDIV     R1,R0,R1
        MOVS     R2,#+10
        SDIV     R3,R1,R2
        MLS      R1,R2,R3,R1
        STRB     R1,[SP, #+2]
//  388   n[3]=(char)((val/1000)%10);
        MOV      R1,#+1000
        SDIV     R0,R0,R1
        MOVS     R1,#+10
        SDIV     R2,R0,R1
        MLS      R0,R1,R2,R0
        STRB     R0,[SP, #+3]
//  389     
//  390   LCD_P8x16chr(x,row,n[4]+48);
        LDRB     R0,[SP, #+4]
        ADDS     R2,R0,#+48
        UXTB     R2,R2            ;; ZeroExt  R2,R2,#+24,#+24
        MOVS     R1,R4
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        MOVS     R0,R5
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        BL       LCD_P8x16chr
//  391   LCD_P8x16chr(x+8,row,n[3]+48);
        LDRB     R0,[SP, #+3]
        ADDS     R2,R0,#+48
        UXTB     R2,R2            ;; ZeroExt  R2,R2,#+24,#+24
        MOVS     R1,R4
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        ADDS     R0,R5,#+8
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        BL       LCD_P8x16chr
//  392   LCD_P8x16chr(x+16,row,n[2]+48);
        LDRB     R0,[SP, #+2]
        ADDS     R2,R0,#+48
        UXTB     R2,R2            ;; ZeroExt  R2,R2,#+24,#+24
        MOVS     R1,R4
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        ADDS     R0,R5,#+16
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        BL       LCD_P8x16chr
//  393   LCD_P8x16chr(x+24,row,n[1]+48);
        LDRB     R0,[SP, #+1]
        ADDS     R2,R0,#+48
        UXTB     R2,R2            ;; ZeroExt  R2,R2,#+24,#+24
        MOVS     R1,R4
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        ADDS     R0,R5,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        BL       LCD_P8x16chr
//  394   LCD_P8x16chr(x+32,row,n[0]+48);
        LDRB     R0,[SP, #+0]
        ADDS     R2,R0,#+48
        UXTB     R2,R2            ;; ZeroExt  R2,R2,#+24,#+24
        MOVS     R1,R4
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        ADDS     R0,R5,#+32
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        BL       LCD_P8x16chr
//  395 }
        POP      {R0-R2,R4,R5,PC}  ;; return

        SECTION `.text`:CODE:NOROOT(2)
        DATA
F8X16:
        ; Initializer data, 1552 bytes
        DC8 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        DC8 0, 0, 0, 0, 0, 0, 0, 0, 0, 248
        DC8 0, 0, 0, 0, 0, 0, 0, 51, 48, 0
        DC8 0, 0, 0, 16, 12, 6, 16, 12, 6, 0
        DC8 0, 0, 0, 0, 0, 0, 0, 0, 64, 192
        DC8 120, 64, 192, 120, 64, 0, 4, 63, 4, 4
        DC8 63, 4, 4, 0, 0, 112, 136, 252, 8, 48
        DC8 0, 0, 0, 24, 32, 255, 33, 30, 0, 0
        DC8 240, 8, 240, 0, 224, 24, 0, 0, 0, 33
        DC8 28, 3, 30, 33, 30, 0, 0, 240, 8, 136
        DC8 112, 0, 0, 0, 30, 33, 35, 36, 25, 39
        DC8 33, 16, 16, 22, 14, 0, 0, 0, 0, 0
        DC8 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        DC8 0, 224, 24, 4, 2, 0, 0, 0, 0, 7
        DC8 24, 32, 64, 0, 0, 2, 4, 24, 224, 0
        DC8 0, 0, 0, 64, 32, 24, 7, 0, 0, 0
        DC8 64, 64, 128, 240, 128, 64, 64, 0, 2, 2
        DC8 1, 15, 1, 2, 2, 0, 0, 0, 0, 240
        DC8 0, 0, 0, 0, 1, 1, 1, 31, 1, 1
        DC8 1, 0, 0, 0, 0, 0, 0, 0, 0, 0
        DC8 128, 176, 112, 0, 0, 0, 0, 0, 0, 0
        DC8 0, 0, 0, 0, 0, 0, 0, 1, 1, 1
        DC8 1, 1, 1, 1, 0, 0, 0, 0, 0, 0
        DC8 0, 0, 0, 48, 48, 0, 0, 0, 0, 0
        DC8 0, 0, 0, 0, 128, 96, 24, 4, 0, 96
        DC8 24, 6, 1, 0, 0, 0, 0, 224, 16, 8
        DC8 8, 16, 224, 0, 0, 15, 16, 32, 32, 16
        DC8 15, 0, 0, 16, 16, 248, 0, 0, 0, 0
        DC8 0, 32, 32, 63, 32, 32, 0, 0, 0, 112
        DC8 8, 8, 8, 136, 112, 0, 0, 48, 40, 36
        DC8 34, 33, 48, 0, 0, 48, 8, 136, 136, 72
        DC8 48, 0, 0, 24, 32, 32, 32, 17, 14, 0
        DC8 0, 0, 192, 32, 16, 248, 0, 0, 0, 7
        DC8 4, 36, 36, 63, 36, 0, 0, 248, 8, 136
        DC8 136, 8, 8, 0, 0, 25, 33, 32, 32, 17
        DC8 14, 0, 0, 224, 16, 136, 136, 24, 0, 0
        DC8 0, 15, 17, 32, 32, 17, 14, 0, 0, 56
        DC8 8, 8, 200, 56, 8, 0, 0, 0, 0, 63
        DC8 0, 0, 0, 0, 0, 112, 136, 8, 8, 136
        DC8 112, 0, 0, 28, 34, 33, 33, 34, 28, 0
        DC8 0, 224, 16, 8, 8, 16, 224, 0, 0, 0
        DC8 49, 34, 34, 17, 15, 0, 0, 0, 0, 192
        DC8 192, 0, 0, 0, 0, 0, 0, 48, 48, 0
        DC8 0, 0, 0, 0, 0, 128, 0, 0, 0, 0
        DC8 0, 0, 128, 96, 0, 0, 0, 0, 0, 0
        DC8 128, 64, 32, 16, 8, 0, 0, 1, 2, 4
        DC8 8, 16, 32, 0, 64, 64, 64, 64, 64, 64
        DC8 64, 0, 4, 4, 4, 4, 4, 4, 4, 0
        DC8 0, 8, 16, 32, 64, 128, 0, 0, 0, 32
        DC8 16, 8, 4, 2, 1, 0, 0, 112, 72, 8
        DC8 8, 8, 240, 0, 0, 0, 0, 48, 54, 1
        DC8 0, 0, 192, 48, 200, 40, 232, 16, 224, 0
        DC8 7, 24, 39, 36, 35, 20, 11, 0, 0, 0
        DC8 192, 56, 224, 0, 0, 0, 32, 60, 35, 2
        DC8 2, 39, 56, 32, 8, 248, 136, 136, 136, 112
        DC8 0, 0, 32, 63, 32, 32, 32, 17, 14, 0
        DC8 192, 48, 8, 8, 8, 8, 56, 0, 7, 24
        DC8 32, 32, 32, 16, 8, 0, 8, 248, 8, 8
        DC8 8, 16, 224, 0, 32, 63, 32, 32, 32, 16
        DC8 15, 0, 8, 248, 136, 136, 232, 8, 16, 0
        DC8 32, 63, 32, 32, 35, 32, 24, 0, 8, 248
        DC8 136, 136, 232, 8, 16, 0, 32, 63, 32, 0
        DC8 3, 0, 0, 0, 192, 48, 8, 8, 8, 56
        DC8 0, 0, 7, 24, 32, 32, 34, 30, 2, 0
        DC8 8, 248, 8, 0, 0, 8, 248, 8, 32, 63
        DC8 33, 1, 1, 33, 63, 32, 0, 8, 8, 248
        DC8 8, 8, 0, 0, 0, 32, 32, 63, 32, 32
        DC8 0, 0, 0, 0, 8, 8, 248, 8, 8, 0
        DC8 192, 128, 128, 128, 127, 0, 0, 0, 8, 248
        DC8 136, 192, 40, 24, 8, 0, 32, 63, 32, 1
        DC8 38, 56, 32, 0, 8, 248, 8, 0, 0, 0
        DC8 0, 0, 32, 63, 32, 32, 32, 32, 48, 0
        DC8 8, 248, 248, 0, 248, 248, 8, 0, 32, 63
        DC8 0, 63, 0, 63, 32, 0, 8, 248, 48, 192
        DC8 0, 8, 248, 8, 32, 63, 32, 0, 7, 24
        DC8 63, 0, 224, 16, 8, 8, 8, 16, 224, 0
        DC8 15, 16, 32, 32, 32, 16, 15, 0, 8, 248
        DC8 8, 8, 8, 8, 240, 0, 32, 63, 33, 1
        DC8 1, 1, 0, 0, 224, 16, 8, 8, 8, 16
        DC8 224, 0, 15, 24, 36, 36, 56, 80, 79, 0
        DC8 8, 248, 136, 136, 136, 136, 112, 0, 32, 63
        DC8 32, 0, 3, 12, 48, 32, 0, 112, 136, 8
        DC8 8, 8, 56, 0, 0, 56, 32, 33, 33, 34
        DC8 28, 0, 24, 8, 8, 248, 8, 8, 24, 0
        DC8 0, 0, 32, 63, 32, 0, 0, 0, 8, 248
        DC8 8, 0, 0, 8, 248, 8, 0, 31, 32, 32
        DC8 32, 32, 31, 0, 8, 120, 136, 0, 0, 200
        DC8 56, 8, 0, 0, 7, 56, 14, 1, 0, 0
        DC8 248, 8, 0, 248, 0, 8, 248, 0, 3, 60
        DC8 7, 0, 7, 60, 3, 0, 8, 24, 104, 128
        DC8 128, 104, 24, 8, 32, 48, 44, 3, 3, 44
        DC8 48, 32, 8, 56, 200, 0, 200, 56, 8, 0
        DC8 0, 0, 32, 63, 32, 0, 0, 0, 16, 8
        DC8 8, 8, 200, 56, 8, 0, 32, 56, 38, 33
        DC8 32, 32, 24, 0, 0, 0, 0, 254, 2, 2
        DC8 2, 0, 0, 0, 0, 127, 64, 64, 64, 0
        DC8 0, 12, 48, 192, 0, 0, 0, 0, 0, 0
        DC8 0, 1, 6, 56, 192, 0, 0, 2, 2, 2
        DC8 254, 0, 0, 0, 0, 64, 64, 64, 127, 0
        DC8 0, 0, 0, 0, 4, 2, 2, 2, 4, 0
        DC8 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        DC8 0, 0, 0, 0, 0, 0, 128, 128, 128, 128
        DC8 128, 128, 128, 128, 0, 2, 2, 4, 0, 0
        DC8 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        DC8 0, 0, 128, 128, 128, 128, 0, 0, 0, 25
        DC8 36, 34, 34, 34, 63, 32, 8, 248, 0, 128
        DC8 128, 0, 0, 0, 0, 63, 17, 32, 32, 17
        DC8 14, 0, 0, 0, 0, 128, 128, 128, 0, 0
        DC8 0, 14, 17, 32, 32, 32, 17, 0, 0, 0
        DC8 0, 128, 128, 136, 248, 0, 0, 14, 17, 32
        DC8 32, 16, 63, 32, 0, 0, 128, 128, 128, 128
        DC8 0, 0, 0, 31, 34, 34, 34, 34, 19, 0
        DC8 0, 128, 128, 240, 136, 136, 136, 24, 0, 32
        DC8 32, 63, 32, 32, 0, 0, 0, 0, 128, 128
        DC8 128, 128, 128, 0, 0, 107, 148, 148, 148, 147
        DC8 96, 0, 8, 248, 0, 128, 128, 128, 0, 0
        DC8 32, 63, 33, 0, 0, 32, 63, 32, 0, 128
        DC8 152, 152, 0, 0, 0, 0, 0, 32, 32, 63
        DC8 32, 32, 0, 0, 0, 0, 0, 128, 152, 152
        DC8 0, 0, 0, 192, 128, 128, 128, 127, 0, 0
        DC8 8, 248, 0, 0, 128, 128, 128, 0, 32, 63
        DC8 36, 2, 45, 48, 32, 0, 0, 8, 8, 248
        DC8 0, 0, 0, 0, 0, 32, 32, 63, 32, 32
        DC8 0, 0, 128, 128, 128, 128, 128, 128, 128, 0
        DC8 32, 63, 32, 0, 63, 32, 0, 63, 128, 128
        DC8 0, 128, 128, 128, 0, 0, 32, 63, 33, 0
        DC8 0, 32, 63, 32, 0, 0, 128, 128, 128, 128
        DC8 0, 0, 0, 31, 32, 32, 32, 32, 31, 0
        DC8 128, 128, 0, 128, 128, 0, 0, 0, 128, 255
        DC8 161, 32, 32, 17, 14, 0, 0, 0, 0, 128
        DC8 128, 128, 128, 0, 0, 14, 17, 32, 32, 160
        DC8 255, 128, 128, 128, 128, 0, 128, 128, 128, 0
        DC8 32, 32, 63, 33, 32, 0, 1, 0, 0, 0
        DC8 128, 128, 128, 128, 128, 0, 0, 51, 36, 36
        DC8 36, 36, 25, 0, 0, 128, 128, 224, 128, 128
        DC8 0, 0, 0, 0, 0, 31, 32, 32, 0, 0
        DC8 128, 128, 0, 0, 0, 128, 128, 0, 0, 31
        DC8 32, 32, 32, 16, 63, 32, 128, 128, 128, 0
        DC8 0, 128, 128, 128, 0, 1, 14, 48, 8, 6
        DC8 1, 0, 128, 128, 0, 128, 0, 128, 128, 128
        DC8 15, 48, 12, 3, 12, 48, 15, 0, 0, 128
        DC8 128, 0, 128, 128, 128, 0, 0, 32, 49, 46
        DC8 14, 49, 32, 0, 128, 128, 128, 0, 0, 128
        DC8 128, 128, 128, 129, 142, 112, 24, 6, 1, 0
        DC8 0, 128, 128, 128, 128, 128, 128, 0, 0, 33
        DC8 48, 44, 34, 33, 48, 0, 0, 0, 0, 0
        DC8 128, 124, 2, 2, 0, 0, 0, 0, 0, 63
        DC8 64, 64, 0, 0, 0, 0, 255, 0, 0, 0
        DC8 0, 0, 0, 0, 255, 0, 0, 0, 0, 2
        DC8 2, 124, 128, 0, 0, 0, 0, 64, 64, 63
        DC8 0, 0, 0, 0, 0, 6, 1, 1, 2, 2
        DC8 4, 4, 0, 0, 0, 0, 0, 0, 0, 0
        DC8 0, 255, 255, 255, 255, 255, 255, 255, 0, 255
        DC8 255, 255, 255, 255, 255, 255, 255, 0, 0, 0
        DC8 0, 0, 0, 0, 255, 0, 0, 0, 0, 0
        DC8 0, 0

        SECTION `.text`:CODE:NOROOT(2)
        DATA
F14x16_Idx:
        ; Initializer data, 40 bytes
        DC8 177, 177, 190, 169, 193, 170, 186, 207, 180, 243
        DC8 209, 167, 202, 181, 209, 181, 187, 249, 181, 216
        DC8 197, 203, 183, 229, 214, 199, 196, 220, 179, 181
        DC8 176, 180, 188, 252, 195, 235, 0, 0, 0, 0

        SECTION `.text`:CODE:NOROOT(2)
        DATA
F14x16:
        ; Initializer data, 504 bytes
        DC8 0, 0, 64, 64, 64, 252, 0, 0, 252, 128
        DC8 64, 32, 32, 0, 0, 48, 16, 16, 8, 127
        DC8 0, 0, 127, 64, 64, 64, 120, 0, 0, 0
        DC8 16, 240, 80, 80, 74, 84, 80, 80, 240, 8
        DC8 8, 0, 0, 64, 32, 19, 10, 66, 254, 2
        DC8 2, 10, 19, 96, 0, 0, 0, 0, 248, 72
        DC8 248, 4, 36, 38, 40, 240, 44, 34, 128, 0
        DC8 0, 16, 31, 18, 127, 137, 65, 33, 29, 7
        DC8 25, 33, 65, 0, 0, 0, 128, 64, 160, 144
        DC8 142, 136, 144, 160, 64, 192, 128, 0, 0, 1
        DC8 0, 126, 68, 68, 68, 68, 68, 60, 1, 0
        DC8 0, 0, 0, 32, 32, 32, 32, 32, 254, 160
        DC8 32, 32, 32, 32, 32, 0, 0, 128, 64, 32
        DC8 16, 12, 3, 3, 4, 24, 48, 96, 64, 0
        DC8 0, 224, 32, 164, 152, 160, 156, 168, 144, 168
        DC8 166, 96, 32, 0, 0, 0, 8, 8, 8, 72
        DC8 124, 10, 9, 8, 8, 4, 0, 0, 0, 32
        DC8 56, 16, 48, 208, 10, 212, 48, 16, 16, 16
        DC8 24, 0, 0, 132, 132, 133, 71, 36, 20, 15
        DC8 20, 36, 36, 68, 4, 0, 0, 64, 68, 204
        DC8 0, 0, 252, 0, 248, 0, 0, 254, 0, 0
        DC8 0, 0, 0, 127, 40, 24, 7, 0, 15, 0
        DC8 0, 127, 0, 0, 0, 8, 8, 8, 254, 168
        DC8 168, 168, 168, 252, 8, 8, 8, 0, 0, 18
        DC8 74, 70, 83, 82, 126, 82, 82, 77, 74, 74
        DC8 25, 0, 0, 64, 252, 64, 64, 128, 248, 128
        DC8 254, 64, 32, 240, 0, 0, 0, 32, 31, 16
        DC8 16, 8, 127, 64, 79, 64, 68, 71, 120, 0
        DC8 0, 64, 132, 96, 64, 44, 212, 68, 252, 116
        DC8 202, 64, 32, 0, 0, 8, 127, 4, 2, 255
        DC8 84, 84, 125, 84, 126, 3, 1, 0, 0, 240
        DC8 0, 252, 224, 32, 16, 140, 90, 168, 88, 140
        DC8 128, 0, 0, 31, 8, 15, 6, 1, 1, 34
        DC8 42, 127, 42, 37, 16, 0, 0, 48, 40, 166
        DC8 120, 104, 164, 40, 252, 136, 136, 248, 0, 0
        DC8 0, 2, 1, 0, 126, 82, 83, 82, 83, 82
        DC8 126, 0, 0, 0, 0, 112, 168, 164, 160, 232
        DC8 48, 0, 124, 144, 136, 136, 96, 0, 0, 0
        DC8 127, 10, 74, 127, 0, 0, 126, 72, 68, 66
        DC8 112, 0, 0, 8, 8, 200, 56, 12, 202, 40
        DC8 8, 8, 136, 8, 8, 0, 0, 8, 8, 8
        DC8 9, 9, 15, 121, 9, 9, 8, 8, 8, 0
        DC8 0, 0, 32, 254, 144, 32, 24, 144, 110, 16
        DC8 16, 48, 24, 0, 0, 66, 66, 127, 128, 129
        DC8 71, 73, 49, 25, 23, 33, 64, 0, 64, 48
        DC8 206, 80, 72, 232, 152, 168, 168, 254, 168, 248
        DC8 16, 0, 0, 2, 127, 162, 82, 36, 30, 41
        DC8 74, 127, 74, 73, 68, 0, 0, 0, 72, 252
        DC8 68, 32, 128, 112, 0, 254, 0, 32, 224, 0
        DC8 8, 4, 3, 127, 129, 129, 65, 64, 32, 23
        DC8 8, 7, 2, 0

        SECTION __DLIB_PERTHREAD:DATA:REORDER:NOROOT(0)

        SECTION __DLIB_PERTHREAD_init:DATA:REORDER:NOROOT(0)

        END
// 
//     4 bytes in section .data
// 3 336 bytes in section .text
// 
// 3 336 bytes of CODE memory
//     4 bytes of DATA memory
//
//Errors: none
//Warnings: 1
