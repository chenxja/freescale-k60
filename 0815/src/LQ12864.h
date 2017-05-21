#ifndef _LQOLED_H
#define _LQOLED_H
#include "includes.h"

#define byte uint8
#define word uint16
#define GPIO_PIN_MASK      0x1Fu    //0x1f=31,����λ��Ϊ0--31��Ч
#define GPIO_PIN(x)        (((1)<<(x & GPIO_PIN_MASK)))  //�ѵ�ǰλ��1

 extern byte longqiu96x64[768];
 void LCD_Init(void);
 void LCD_CLS(void);           //����
 void LCD_P6x8Str(byte x,byte y,byte ch[]);     //6*8�ַ�
 void LCD_P8x16Str(byte x,byte y,byte ch[]);
 void LCD_P14x16Str(byte x,byte y,byte ch[]);
 void LCD_Print(byte x, byte y, byte ch[]);
 void LCD_PutPixel(byte x,byte y);
 void LCD_Rectangle(byte x1,byte y1,byte x2,byte y2,byte gif);
 void Draw_BMP(byte x0,byte y0,byte x1,byte y1,byte bmp[]); 
 void LCD_Fill(byte dat);             
 
 
 
 void LCD_P6x8Str_5(byte x,byte y,int16 date);
 void LCD_P6x8Str_4(byte x,byte y,int16 date);
 void LCD_P6x8Str_3(byte x,byte y,int16 date);
 void LCD_P6x8Str_FuHao_3(byte x,byte y,signed char date);
 void LCD_P6x8Str_1(byte x,byte y,signed char date);
 void LCD_CLS2(byte y);       //ĳһ�е�����
 
#endif
