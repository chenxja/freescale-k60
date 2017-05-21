//使用B0~B7采集数字摄像头OV7620 8位灰度输入
//A24引脚设置为GPIO模式，下降沿中断,场中断
//B10引脚设置为GPIO模式，上升沿中断,行中断
//B11引脚设置为GPIO模式，像素同步脉冲pclk，4分频后，上升沿触发DMA请求
//共采集H行，每行V个点，dma传送到video数组
//本代码仅供个人学习使用，请勿用于其他用途
//舵机5V电压250Hz    电机10KHz  3.3V   摄像头5V供电 

/********************************************************/
        //给某个引脚设置高低电平的方法
        //PORTD_PCR10=PORT_PCR_MUX(1);                 选择GPIO模式
        //GPIOD_PDDR=GPIO_PDDR_PDD(GPIO_PIN(10));      选择输出模式
        //GPIOD_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(10));  高电平
        //GPIOA_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(14));  低电平
        /********************************************************/
#include <stdio.h>
#include "includes.h"
#include "MK60N512VMD100.h"
#include "uart.h"
#include "keyboard.h"
#include "IR.h"
#include "LQ12864.h" 
#include "lptmr.h"
#define READword(address)     ((unsigned int)(*(volatile unsigned int *)(address)))
#define FLEXNVM_startline_ADDR 0X10000000
#define FLEXRAM_startline_ADDR 0X14000000
#define PIT0 0
#define PIT1 1
#define PIT2 2
#define PIT3 3
#define GPIO_PIN_MASK      0x1Fu    //0x1f=31,限制位数为0--31有效
#define GPIO_PIN(x)        (((1)<<(x & GPIO_PIN_MASK)))  //把当前位置1                
#define endlineROW  169        //OV7620，每场240行
#define  IMG_ROWS 55
#define  IMG_COLS 160
#define  video_Middle  77/76//91偏左//85

int16 Servo_Middle=8151;//8151;   //中间  8155
int16 Servo_Left=9320;    //最左10475   250Hz  44%
int16 Servo_Right=7035;    //最右15575   250Hz  30.53%7035

#define  unclear  -100
#define  ok        1
#define  fail      0
#define  position_mid   77      //中心位置

int tu_flag=0;
volatile int P_timer=0;

unsigned int row=0;//摄像头行计数，最大240
volatile uint8 ImageData[IMG_ROWS][IMG_COLS];//存放数据数组
volatile uint8 ImageData2[IMG_ROWS][IMG_COLS];
volatile uint16 ption[160];
volatile int position[IMG_ROWS];//存放中心线
volatile int backup[IMG_ROWS];//存放中心线
volatile uint8 Very_straight=0;//这个需要清零
volatile uint8 ramp_flag=0;//这个需要清零
volatile int16 Judge_startline=150;
unsigned int imagerow=0;//采集行计数，最大H
int16 dijihang=0;
unsigned int const data_table[IMG_ROWS]={ 1,   3,   5,   7, 9,//2 
                                   11,  13,  15,  17,  19,    //2
                                   21,  23,  25,  27,  29,    //2
                                   31,  34,  37,  40,  43,    //3
                                   46,  49,  52,  55,  58,    //3
                                   61,  64,  67,  70,  73,    //3
                                   76,  79,  82,  85,  88,    //3
                                   92,  96, 100, 104, 108,    //4
                                   112, 116, 120, 124, 128,    //4
                                   132, 136, 140, 144, 148,    //4
                                   152, 156, 160, 164, 168     //4         
                                   };


const unsigned char Wspeed_diff[2][60]={{100,100,100,100,100,100,100,100,100,100,  //差速表
                                          99,99,99,99, 98, 98, 98, 97, 97, 97,     /////////自己注释
                                          //96, 96, 96, 96, 95, 95, 94, 94, 94, 93,
                                          97,96,96, 96, 95, 94, 93, 92,91,90,
                                         //93, 93, 91, 91, 89, 89, 87, 87, 85, 85, 
                                         89,88,87,86,85,84,83,82,81,80,
                                          //, 83, 81, 81, 79, 79, 77, 77, 75, 75,
                                          79,78,77,76,75,74,73,72,71,70, 
                                        // 73, 73, 71, 71, 69, 67, 65, 63, 61, 59
                                         69,68,67,66,65,64,63,62,61,60
                                        },                                                                     
                                       { 
                                       //100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
                                       100, 100, 100, 100, 100,100,100,100,100, 100,
                                       //101, 101, 101, 101, 101, 101, 101, 101, 101, 103,
                                        //104, 105, 106, 107, 108, 109, 110, 111, 112, 113,
                                        101, 102, 103, 104, 105, 106, 107, 108, 109, 110,
                                        //114, 115, 116, 117, 118, 119, 120, 121, 122, 123,
                                         111, 112, 113, 114, 115, 117,119, 121, 122, 124,
                                        //125, 127, 129, 131, 133, 135, 137, 139, 141, 143,
                                        126,128,130,132,134,136,138,140,142,144,
                                        // 128, 132, 137, 142, 148, 154, 160, 166, 170, 175,
                                         146,148,150, 152,153,154,155,157,159,161,
                                         163,165,167,168,169,170,172,174,176,178
                                       }
                                      }; 

  //找根参数b
//int b=48;
   uint8 black2_a1 = 145;//135 140
  uint8 black2_a2 = 155; //145 145
  uint8 black2_a3 = 145;// 135 140
  uint8 black2_b1 = 165;//150 135
  uint8 black2_b2 = 165;//150 140
  uint8 black2_b3 = 165;//150 135
  uint8 black2_c1 = 168;//145 154   132
  uint8 black2_c2 = 169;//158 158     138
  uint8 black2_c3 = 168;// 145 154     132
  uint8 black2_d1 = 162;//152 157  130
  uint8 black2_d2 = 163;//153 158  142
  uint8 black2_d3 = 162; //152 157  130
  
  
  
  uint8 black_a1 = 135;//140
  uint8 black_a2 =135;//145
  uint8 black_a3 = 135;//140
  uint8 black_b1 = 150;//135
  uint8 black_b2 = 150;//140
  uint8 black_b3 = 150;//135
  uint8 black_c1 = 145;//154   132
  uint8 black_c2 = 158;//158     138
  uint8 black_c3 = 145;//154     132
  uint8 black_d1 = 152;//157  130
  uint8 black_d2 = 153;//158  142
  uint8 black_d3 = 152;//157  130
uint8 row_F[IMG_ROWS];//该行采集完成标志
char startline;//起始行
char endline;//结束行
char startline_F;//发现起始行
char endline_F;//发现结束行

uint8 LCD_P1=0,LCD_P2=0,LCD_P3=0;
volatile uint16 pulse=0,pulse2=0;
volatile uint8 finish=0;
 int start_position=0;


volatile uint8 sai_dao_lei_xing=0;

volatile int16 low_0=0,mid_0=0,high_0=0;

volatile int16 low=0,mid=0,high=0;
volatile int16 topline=0;
volatile int16 low_old=0,mid_old=0,high_old=0;
volatile int16 position_now=0,position_diff=0,position_now_low=0;
volatile int16 Servo_value=0,Servo_value_old=13015;//这里Servo_value_old的初始值要设置的和Servo_Middle一样，是用来限定的
volatile int16 low_now=0,mid_now=0,high_now=0;

volatile int16 low_now_0=0,mid_now_0=0,high_now_0=0;
volatile int16 END_0=0;

volatile uint8 car_speed=0;
volatile int16 speed_Very_straight,speed_top,speed_b,speed_m,speed_s,speed_ms;

volatile int16 time_m=0,speed_now=0;
volatile int16 s_old=0;
//volatile int16 position_now_P=0,position_now_D=0;

////////////模糊控制///////////////
#define MP_S      400//110  260  360
#define MP_M      440//180  320  400
#define MP_B      480//240  360  440

#define MI_S      6//6 4
#define MI_M      9//10 8 
#define MI_B      12//15  10

#define  motor_p      70//   45 50 55 60 65


//////////////////////舵机参数////////////////////////////

uint8 P_High=12;//uint8 P_High=11;//27  29
uint8 D_High=6;//uint8 D_High=0;//18  15
uint8 H_High=17;//uint8 H_High=20;
uint8 T_High=29;//41//uint8 T_High=25;


uint8 P_Mid=10;//18//15//12//13
uint8 D_Mid=16;//18//13//11//9//10
uint8 H_Mid=15;//uint8 H_Mid=15;
uint8 T_Mid=45;//uint8 T_Mid=50;


uint8 P_Mid1=10;
uint8 D_Mid1=21;
uint8 H_Mid1=30;
uint8 T_Mid1=45;


uint8 P_Low=25;//21//18//15//14//
uint8 D_Low=41;//21//15//11//13
uint8 H_Low=29;//H_Low=26;
uint8 T_Low=50;//T_Low=50;

#include "keyboard_PD.h"

/*******************point结构体*********************/
struct point_position{
  volatile byte x;
  volatile byte y;
  volatile byte i;//
  volatile byte j;//
}point;
/*******************point结构体END*********************/
struct PID_proportion{
  volatile int16 P;
  volatile int16 I;
  volatile int16 D;
}Servo,Motor;

struct range
{
  int16 head;
  int16 tail;
}servo;

struct membership_grade
{
  int16 s;
  int16 m;
  int16 b;
}sub;


/***********************系统、总线频率设置**********************/
void pllinit180M(void)
{
	uint32_t temp_reg;
        //使能IO端口时钟    
    SIM_SCGC5 |= (SIM_SCGC5_PORTA_MASK
                              | SIM_SCGC5_PORTB_MASK
                              | SIM_SCGC5_PORTC_MASK
                              | SIM_SCGC5_PORTD_MASK
                              | SIM_SCGC5_PORTE_MASK );
    //这里处在默认的FEI模式
    //首先移动到FBE模式
    MCG_C2 = 0;  
    //MCG_C2 = MCG_C2_RANGE(2) | MCG_C2_HGO_MASK | MCG_C2_EREFS_MASK;
    //初始化晶振后释放锁定状态的振荡器和GPIO
    SIM_SCGC4 |= SIM_SCGC4_LLWU_MASK;
    LLWU_CS |= LLWU_CS_ACKISO_MASK;
    
    //选择外部晶振，参考分频器，清IREFS来启动外部晶振
    //011 If RANGE = 0, Divide Factor is 8; for all other RANGE values, Divide Factor is 256.
    MCG_C1 = MCG_C1_CLKS(2) | MCG_C1_FRDIV(3);
    
    //等待晶振稳定	    
    //while (!(MCG_S & MCG_S_OSCINIT_MASK)){}              //等待锁相环初始化结束
    while (MCG_S & MCG_S_IREFST_MASK){}                  //等待时钟切换到外部参考时钟
    while (((MCG_S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) != 0x2){}
    //进入FBE模式,
    MCG_C5 = MCG_C5_PRDIV(0x0e);//分频在2~4MHz之间，分频后频率=晶振频率/(PRDIV+1) 3.3MHz                
    MCG_C6 = 0x0;//确保MCG_C6处于复位状态，禁止LOLIE、PLL、和时钟控制器，清PLL VCO分频器
    temp_reg = FMC_PFAPR;//保存FMC_PFAPR当前的值
    FMC_PFAPR |= FMC_PFAPR_M7PFD_MASK | FMC_PFAPR_M6PFD_MASK | FMC_PFAPR_M5PFD_MASK
                     | FMC_PFAPR_M4PFD_MASK | FMC_PFAPR_M3PFD_MASK | FMC_PFAPR_M2PFD_MASK
                     | FMC_PFAPR_M1PFD_MASK | FMC_PFAPR_M0PFD_MASK; //通过M&PFD置位M0PFD来禁止预取功能   
    ///设置系统分频器
    //MCG=PLL, core = MCG, bus = MCG/3, FlexBus = MCG/3, Flash clock= MCG/8
    SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(3) 
                 | SIM_CLKDIV1_OUTDIV3(2) | SIM_CLKDIV1_OUTDIV4(7);       
    FMC_PFAPR = temp_reg;//从新存FMC_PFAPR的原始值 
    //设置VCO分频器，使能PLL为100MHz, LOLIE=0, PLLS=1, CME=0, VDIV=26
    MCG_C6 = MCG_C6_PLLS_MASK | MCG_C6_VDIV(30);  //VDIV = 31 (x55)
                                                  //VDIV = 26 (x50)
    while (!(MCG_S & MCG_S_PLLST_MASK)){}; // wait for PLL status bit to set    
    while (!(MCG_S & MCG_S_LOCK_MASK)){}; // Wait for LOCK bit to set    
    //进入PBE模式    
    //通过清零CLKS位来进入PEE模式
    // CLKS=0, FRDIV=3, IREFS=0, IRCLKEN=0, IREFSTEN=0
    MCG_C1 &= ~MCG_C1_CLKS_MASK;
    //等待时钟状态位更新
    while (((MCG_S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) != 0x3){};
}
/***********************系统、总线频率设置END**********************/

/************************舵机控制输出PWM模块***********************/
void hw_FTM1_init(void)
    {      	
      //SIM_SOPT4|=SIM_SOPT4_FTM1FLT0_MASK;        
      /* Turn on all port clocks */
      SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
        
      /* Enable the function on PTA8 */
      PORTA_PCR8 = PORT_PCR_MUX(0x3)| PORT_PCR_DSE_MASK;; // FTM is alt3 function for this pin         
    
      SIM_SCGC6|=SIM_SCGC6_FTM1_MASK;         //使能FTM1时钟      
      
      //change MSnB = 1  
      FTM1_C0SC |= FTM_CnSC_ELSB_MASK;
      FTM1_C0SC &= ~FTM_CnSC_ELSA_MASK;
      FTM1_C0SC |= FTM_CnSC_MSB_MASK;     
      
      //FTM1_SC = FTM_SC_PS(0) | FTM_SC_CLKS(1);
      //FTM1_SC=0X0F;     
      FTM1_SC = 0xb; //not enable the interrupt mask向上计数模式
      //FTM1_SC=0X1F;       //BIT5  0 FTM counter operates in up counting mode.
                            //1 FTM counter operates in up-down counting mode.      
      //BIT43 FTM1_SC|=FTM1_SC_CLKS_MASK;
                            //00 No clock selected (This in effect disables the FTM counter.)
                            //01 System clock
                            //10 Fixed frequency clock
                            //11 External clock
      //BIT210 FTM1_SC|=FTM1_SC_PS_MASK; 
                            //100M          MOD=2000;     MOD=4000;   MOD=1000; 
                            //000 Divide by 1---12KHZ     6K          24k
                            //001 Divide by 2--- 6KHZ
                            //010 Divide by 4--- 3K
                            //011 Divide by 8--- 1.5K
                            //100 Divide by 16---750
                            //101 Divide by 32---375
                            //110 Divide by 64---187.5HZ
                            //111 Divide by 128--93.75hz             
      
      FTM1_MODE |= FTM_MODE_WPDIS_MASK;      
       //BIT1   Initialize the Channels Output
      //FTMEN is bit 0, need to set to zero so DECAPEN can be set to 0
      FTM1_MODE &= ~1;
       //BIT0   FTM Enable
       //0 Only the TPM-compatible registers (first set of registers) can be used without any restriction. Do not use the FTM-specific registers.
       //1 All registers including the FTM-specific registers (second set of registers) are available for use with no restrictions.
      
      FTM1_OUTMASK=0xFE;   //0 Channel output is not masked. It continues to operate normally.
                           //1 Channel output is masked. It is forced to its inactive state.
      
      FTM1_COMBINE=0;      //Function for Linked Channels (FTMx_COMBINE)
      FTM1_OUTINIT=0;
      FTM1_EXTTRIG=0;      //FTM External Trigger (FTMx_EXTTRIG)
      FTM1_POL=0;          //Channels Polarity (FTMx_POL)
                           //0 The channel polarity is active high.
                           //1 The channel polarity is active low.     
      //Set Edge Aligned PWM
      FTM1_QDCTRL &=~FTM_QDCTRL_QUADEN_MASK;
      //QUADEN is Bit 1, Set Quadrature Decoder Mode (QUADEN) Enable to 0,   (disabled)
      //FTM0_SC = 0x16; //Center Aligned PWM Select = 0, sets FTM Counter to operate in up counting mode,
      //it is field 5 of FTMx_SC (status control) - also setting the pre-scale bits here
      
      FTM1_INVCTRL=0;     //反转控制
      FTM1_SWOCTRL=0;     //软件输出控制F TM Software Output Control (FTMx_SWOCTRL)
      FTM1_PWMLOAD=0;     //FTM PWM Load
                          //BIT9: 0 Loading updated values is disabled.
                          //1 Loading updated values is enabled.
      FTM1_CNTIN=0;        //Counter Initial Value      
      FTM1_MOD=62500;//56250       //Modulo value,The EPWM period is determined by (MOD - CNTIN + 0x0001) 
                           //采用龙丘时钟初始化函数，可以得到4分频的频率，系统60M频率时，PWM频率是15M,以此类推
                           //PMW频率=X系统频率/4/(2^FTM1_SC_PS)/FTM1_MOD
      FTM1_C0V=Servo_Middle;        //设置 the pulse width(duty cycle) is determined by (CnV - CNTIN).
      FTM1_CNT=0;          //只有低16位可用
}
/************************舵机控制输出PWM模块END***********************/

/*********************电机正反转  C1口C3口输出PWM波*************/
void hw_FTM0_init(void)
    {      	
      
      SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK;
        
      
      PORTC_PCR1 = PORT_PCR_MUX(0x4)| PORT_PCR_DSE_MASK; // FTM is alt4 function for this pin         
      PORTC_PCR4 = PORT_PCR_MUX(0x4)| PORT_PCR_DSE_MASK;
      PORTC_PCR2 = PORT_PCR_MUX(0x4)| PORT_PCR_DSE_MASK;
      
      PORTD_PCR4 = PORT_PCR_MUX(0x4)| PORT_PCR_DSE_MASK; // FTM is alt4 function for this pin         
      PORTD_PCR5 = PORT_PCR_MUX(0x4)| PORT_PCR_DSE_MASK;
      PORTD_PCR6 = PORT_PCR_MUX(0x4)| PORT_PCR_DSE_MASK;
      
      SIM_SCGC6|=SIM_SCGC6_FTM0_MASK;         //使能FTM0时钟      
      
      
      //电机1
      //C0口 
      FTM0_C0SC |= FTM_CnSC_ELSB_MASK;//ELSB=1，ELSA=0，左对齐，先高后低
      FTM0_C0SC &= ~FTM_CnSC_ELSA_MASK;
      FTM0_C0SC |= FTM_CnSC_MSB_MASK;    //MSB=1，模式选择边沿对齐
      //C3口
      FTM0_C3SC |= FTM_CnSC_ELSB_MASK;
      FTM0_C3SC &= ~FTM_CnSC_ELSA_MASK;
      FTM0_C3SC |= FTM_CnSC_MSB_MASK;
      //C1口 电机控制长期高电压
      FTM0_C1SC |= FTM_CnSC_ELSB_MASK;
      FTM0_C1SC &= ~FTM_CnSC_ELSA_MASK;
      FTM0_C1SC |= FTM_CnSC_MSB_MASK;    
      
      //电机2
       FTM0_C4SC |= FTM_CnSC_ELSB_MASK;//ELSB=1，ELSA=0，左对齐，先高后低
      FTM0_C4SC &= ~FTM_CnSC_ELSA_MASK;
      FTM0_C4SC |= FTM_CnSC_MSB_MASK;    //MSB=1，模式选择边沿对齐
      //C3口
      FTM0_C5SC |= FTM_CnSC_ELSB_MASK;
      FTM0_C5SC &= ~FTM_CnSC_ELSA_MASK;
      FTM0_C5SC |= FTM_CnSC_MSB_MASK;
      //C1口 电机控制长期高电压
      FTM0_C6SC |= FTM_CnSC_ELSB_MASK;
      FTM0_C6SC &= ~FTM_CnSC_ELSA_MASK;
      FTM0_C6SC |= FTM_CnSC_MSB_MASK;    
      FTM0_SC = 0xC; //not enable the interrupt mask向上计数模式
                     //中断禁止，时钟源来自总线时钟60MHz，16分频后得到3.75MHz，CLKS=01，PS=100,CPWMS=0，递增计数
      FTM0_MODE |= FTM_MODE_WPDIS_MASK;  //写保护禁止    
       //BIT1   Initialize the Channels Output
      //FTMEN is bit 0, need to set to zero so DECAPEN can be set to 0
      FTM0_MODE &= ~1;  //FTM0使能
       //BIT0   FTM Enable
       //0 Only the TPM-compatible registers (first set of registers) can be used without any restriction. Do not use the FTM-specific registers.
       //1 All registers including the FTM-specific registers (second set of registers) are available for use with no restrictions.
      
      FTM0_OUTMASK=0x84;   //0 Channel output is not masked. It continues to operate normally.
                           //1 Channel output is masked. It is forced to its inactive state.
                           //通道3 1 0输出，其他通道屏蔽   
      FTM0_COMBINE=0;      //Function for Linked Channels (FTMx_COMBINE)//DECAPEN=0，双边沿捕捉禁止，COMBINE=0，不级联
      FTM0_OUTINIT=0;
      FTM0_EXTTRIG=0;      //FTM External Trigger (FTMx_EXTTRIG)
      FTM0_POL=0;          //Channels Polarity (FTMx_POL)
                           //0 The channel polarity is active high.
                           //1 The channel polarity is active low.     
      //Set Edge Aligned PWM
      FTM0_QDCTRL &=~FTM_QDCTRL_QUADEN_MASK;//禁止正交解码模式
      //QUADEN is Bit 1, Set Quadrature Decoder Mode (QUADEN) Enable to 0,   (disabled)
      //FTM0_SC = 0x16; //Center Aligned PWM Select = 0, sets FTM Counter to operate in up counting mode,
      //it is field 5 of FTMx_SC (status control) - also setting the pre-scale bits here
      
      FTM0_INVCTRL=0;     //反转控制
      FTM0_SWOCTRL=0;     //软件输出控制F TM Software Output Control (FTMx_SWOCTRL)
      FTM0_PWMLOAD=0;     //FTM PWM Load
                          //BIT9: 0 Loading updated values is disabled.
                          //1 Loading updated values is enabled.
      FTM0_CNTIN=0;        //Counter Initial Value  初始技术值为0    
      FTM0_MOD=375;       //375为10K 
                           //采用龙丘时钟初始化函数，可以得到4分频的频率，系统60M频率时，PWM频率是15M,以此类推
                           //PMW频率=X系统频率/4/(2^FTM1_SC_PS)/FTM1_MOD
      FTM0_C0V=0;//20;        //反转
      FTM0_C1V=375;        //持续高电平
      FTM0_C3V=0;//80;           //正转
      
      FTM0_C4V=0;//100;        //正转
      FTM0_C5V=375;        //持续高电平
      FTM0_C6V=0;//20;           //反转
      
      FTM0_CNT=0;          //只有低16位可用
}
/*********************电机正反转  C1口C3口输出PWM波END*************/

/****************FTM正交分解，用于电机1测脉冲数的   A10口*************/
void FTM2_QUAD_Init(void)  
{  
  PORTA_PCR10    =  PORT_PCR_MUX(6);             // 设置引脚A10引脚为FTM2_PHA功能  
  PORTA_PCR11    =  PORT_PCR_MUX(6);             // 设置引脚A11引脚为FTM2_PHB功能  
  PORT_PCR_REG(PORTA_BASE_PTR, 10) |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK ; //开弱上拉
  PORT_PCR_REG(PORTA_BASE_PTR, 11) |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK ; //开弱上拉
  SIM_SCGC3     |=  SIM_SCGC3_FTM2_MASK;                // 使能FTM2时钟  
  FTM2_MODE     |=  FTM_MODE_WPDIS_MASK;                // 写保护禁止  
  FTM2_QDCTRL   |=  FTM_QDCTRL_QUADMODE_MASK;          // AB相同时确定方向和计数值  
  FTM2_CNTIN     =  0;                                 // FTM0计数器初始值为0  
  FTM2_MOD       =  65535;                                 // 结束值  
  FTM2_QDCTRL   |=  FTM_QDCTRL_QUADEN_MASK;             // 启用FTM2正交解码模式  
  FTM2_MODE     |=  FTM_MODE_FTMEN_MASK;                // FTM2EN=1    
  FTM2_CNT       =  0;  
}  
/****************FTM正交分解，用于测脉冲数的   A10口END*************/
//电机2 DMA测脉冲数B18

/**************************IO口初始化***********************/
void IO_Init()
{
	/* 打开各个端口的时钟源 */
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | 
	SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;
	PORTA_PCR14=PORT_PCR_MUX(1);//A14引脚设置为GPIO模式
	PORTA_PCR15=PORT_PCR_MUX(1);//A15引脚设置为GPIO模式
	PORTA_PCR16=PORT_PCR_MUX(1);//A16引脚设置为GPIO模式
	PORTA_PCR17=PORT_PCR_MUX(1);//A17引脚设置为GPIO模式
	
	//B0~B7设为GPIO输入模式，连接ov7260的8位灰度输入
	PORTB_PCR0=PORT_PCR_MUX(1);//B0引脚设置为GPIO模式
	PORTB_PCR1=PORT_PCR_MUX(1);//B1引脚设置为GPIO模式
	PORTB_PCR2=PORT_PCR_MUX(1);//B2引脚设置为GPIO模式
	PORTB_PCR3=PORT_PCR_MUX(1);//B3引脚设置为GPIO模式
	PORTB_PCR4=PORT_PCR_MUX(1);//B4引脚设置为GPIO模式
	PORTB_PCR5=PORT_PCR_MUX(1);//B5引脚设置为GPIO模式
	PORTB_PCR6=PORT_PCR_MUX(1);//B6引脚设置为GPIO模式
	PORTB_PCR7=PORT_PCR_MUX(1);//B7引脚设置为GPIO模式
	GPIOB_PDDR&=0XFFFFFF00;//B0~B7设置为输入，数字摄像头8位灰度输入
	
	PORTA_PCR24=PORT_PCR_MUX(1)|PORT_PCR_IRQC(10);//A24引脚设置为GPIO模式，下降沿中断,场中断
	
	PORTB_PCR10=PORT_PCR_MUX(1)|PORT_PCR_IRQC(9);//B10引脚设置为GPIO模式，上升沿中断,行中断
	
	PORTB_PCR11=PORT_PCR_MUX(1)|PORT_PCR_IRQC(1);//B11引脚设置为GPIO模式，上升沿触发DMA请求
	
        PORTC_PCR9=PORT_PCR_MUX(1)|PORT_PCR_IRQC(10);//C9引脚设置为GPIO模式，下降沿中断,红外遥控停车中断
        
        
        
	GPIOA_PDDR=GPIO_PDDR_PDD(GPIO_PIN(14)|GPIO_PIN(16)|GPIO_PIN(17)|GPIO_PIN(26)|GPIO_PIN(28)); //A14为输出模式  P4灯闪烁证明进入PIT中断给脉冲计时
        
        PORTA_PCR28=PORT_PCR_MUX(1);//A17引脚设置为GPIO模式
        PORTA_PCR26=PORT_PCR_MUX(1);//A17引脚设置为GPIO模式
	//PORTC_PCR8=PORT_PCR_MUX(1);//C8引脚设置为GPIO模式，液晶命令/数据选择引脚
	//PORTC_PCR10=PORT_PCR_MUX(1);//C10引脚设置为GPIO模式，液晶复位引脚
	//GPIOC_PDDR|=0X00000500;//C8,C10设置为输出
	//GPIOA_PDDR|=0X0003E000;//A14~A17设置为输出
	//GPIOA_PCOR|=0X0003E000;//初始低电平输出
	//PORTD_PCR2 = PORT_PCR_MUX(1)|PORT_PCR_IRQC(0x9);//IRQ|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK;
	//PORTD_PCR2 = PORT_PCR_MUX(1)|PORT_PCR_IRQC(0x1);//DMA|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK;
	//D2口GPIO功能，上升沿中断，PE启用拉电阻，PS上拉电阻
}
/**************************IO口初始化END***********************/


/******************与上位机相连，用于看图像的*****************/
void SCI(void)
{
  volatile uint8 *uc_FramePoint;
  uint8 a=0;
  uint16 b=0;   
  
  uart_send1(UART0,0x01);
  
  for(a=0;a<IMG_ROWS;a++)
  {
    for(b=0;b<IMG_COLS;b++)
    {
      uc_FramePoint=ImageData[a]+b;
      uart_send1(UART0,*uc_FramePoint);
    }
  }
 
  uart_send1(UART0,0x01);
  
  PORTA_PCR24|=PORT_PCR_ISF_MASK;//清除中断标志
}
/******************与上位机相连，用于看图像的END*****************/



/********************串口模块波特率的设置*********************/
//UART0初始化，使用PTD6为UART0_RX,PTD7为UART0_TX   D6接RXD  D7接TXD
//波特率：115200
void UART0_Init(void)
{
	unsigned long uartclk_khz=180000;//时钟180MHz
	unsigned long baud=115200;//波特率256000
	unsigned short sbr,brfa;
	PORTB_PCR16|=PORT_PCR_MUX(3);//将D6引脚设置为模式3，即UART0_RX
	PORTB_PCR17|=PORT_PCR_MUX(3);//将D7引脚设置为模式3，即UART0_TX
	SIM_SCGC4|=SIM_SCGC4_UART0_MASK;//开启UART0时钟
	sbr=(unsigned short)((uartclk_khz*1000)/(baud*16));//计算并设置波特率
	
	UART0_BDH=(unsigned char)((sbr&0x1F00)>>8);
	UART0_BDL=(unsigned char)(sbr&0x00FF);
	brfa = (((uartclk_khz*32000)/(baud*16))-(sbr*32));
	UART0_C4 = (unsigned char)(brfa & 0x001F);
	UART0_C2|=(UART_C2_TE_MASK|UART_C2_RE_MASK);
	UART0_C1=0;	
}
/********************串口模块波特率的设置END*********************/



/****************************DMA初始化*****************************/
//DMA初始化 使用PLCK经过4分频后与B11相连用于DMA外部中断
void DMA0_Init(void)
{
	SIM_SCGC6|=SIM_SCGC6_DMAMUX_MASK;//打开DMA多路复用器时钟
	SIM_SCGC7|=SIM_SCGC7_DMA_MASK;//打开DMA模块时钟
	DMAMUX_CHCFG0=DMAMUX_CHCFG_SOURCE(50);//DMA通道0对应50号DMA请求，即PORTB	
	
	DMA_TCD0_CITER_ELINKNO=DMA_CITER_ELINKNO_CITER(IMG_COLS);//当前主循环次数,采集点数
	DMA_TCD0_BITER_ELINKNO=DMA_BITER_ELINKNO_BITER(IMG_COLS);//起始主循环次数，采集点数
	DMA_TCD0_SADDR=(uint32)&GPIOB_PDIR;//设置源地址GPIO口，PORTB
	DMA_TCD0_SOFF=0;//每次传送源地址不变
	//DMA_TCD1_NBYTES_MLOFFYES=DMA_NBYTES_MLOFFYES_NBYTES(1)+DMA_NBYTES_MLOFFNO_SMLOE_MASK+DMA_NBYTES_MLOFFYES_MLOFF(-4);//传送4字节
	DMA_TCD0_NBYTES_MLNO=DMA_NBYTES_MLNO_NBYTES(1);//每次读取一字节
	DMA_TCD0_SLAST=0;//主循环结束后源地址0回写tcd
	DMA_TCD0_DLASTSGA=0;//主循环结束后目的地址0回写tcd
	DMA_TCD0_DADDR=(uint32)ImageData;//设置目的地址，video数组第一个元素
	DMA_TCD0_DOFF=1;//每次写目的地址加1
	DMA_TCD0_ATTR=DMA_ATTR_SSIZE(0)+DMA_ATTR_DSIZE(0);//源数据宽度8bit，目的数据宽度8bit
	DMA_TCD0_CSR=DMA_CSR_DREQ_MASK;//DMA通道0主循环结束后停止硬件请求
	DMA_TCD0_CSR|=DMA_CSR_INTMAJOR_MASK;//使能DMA0中断
	DMAMUX_CHCFG0|=DMAMUX_CHCFG_ENBL_MASK;//DMA通道0使能
	
	//DMA_TCD1_CSR|=DMA_CSR_INTMAJOR_MASK;//使能DMA中断
	//DMA_TCD0_CSR|=DMA_CSR_startline_MASK;
}
/****************************DMA初始化END*****************************/



//==========================================================================
//函数名称: hw_pit_init                                                         
//函数返回: 无                                          
//参数说明: pitno:表示pit通道号。  
//			timeout:表示定时溢出开始递减的值          
//功能概要: 设置相关寄存器的值        
//==========================================================================
    void hw_pit_init(uint8 pitno,uint32 timeout)
    {
      SIM_SCGC6|=SIM_SCGC6_PIT_MASK;              //使能PIT时钟
      PIT_MCR&=~(PIT_MCR_MDIS_MASK);              //调试模式下禁止
      PIT_MCR|=PIT_MCR_FRZ_MASK;                  //使能PIT模块时钟
      PIT_LDVAL(pitno)=timeout;                   //设置周期
      PIT_TCTRL(pitno)|=PIT_TCTRL_TEN_MASK;       //使能pit模块运行
      PIT_TCTRL(pitno)&=~(PIT_TCTRL_TIE_MASK);    //关pit中断
    }
   

/*void binaryzation(){
 uint8 i=0,j=0;
  for(i=0;i<IMG_ROWS;i++)
  {
    if(i<8)
    {
      for(j=0;j<IMG_COLS;j++)
      {
        if(j<50)
        {
          if(ImageData[i][j]<black_d1)ImageData[i][j]=0;
          else ImageData[i][j]=255;
        }
        else if(j<110)
        {
          if(ImageData[i][j]<black_d2)ImageData[i][j]=0;
          else ImageData[i][j]=255;
        }
        else
        {
          if(ImageData[i][j]<black_d3)ImageData[i][j]=0;
          else ImageData[i][j]=255;
        }
      }
    }
    else if(i<20)
    {
      for(j=0;j<IMG_COLS;j++)
      {
        if(j<50)
        {
          if(ImageData[i][j]<black_c1)ImageData[i][j]=0;
          else ImageData[i][j]=255;
        }
        else if(j<110)
        {
          if(ImageData[i][j]<black_c2)ImageData[i][j]=0;
          else ImageData[i][j]=255;
        }
        else
        {
          if(ImageData[i][j]<black_c3)ImageData[i][j]=0;
          else ImageData[i][j]=255;
        }
      }
    }
    else if(i<34)
    {
      for(j=0;j<IMG_COLS;j++)
      {
        if(j<50)
        {
          if(ImageData[i][j]<black_b1)ImageData[i][j]=0;
          else ImageData[i][j]=255;
        }
        else if(j<110)
        {
          if(ImageData[i][j]<black_b2)ImageData[i][j]=0;
          else ImageData[i][j]=255;
        }
        else
        {
          if(ImageData[i][j]<black_b3)ImageData[i][j]=0;
          else ImageData[i][j]=255;
        }
      }
    }
    else
    {
      for(j=0;j<IMG_COLS;j++)
      {
        if(j<50)
        {
          if(ImageData[i][j]<black_a1)ImageData[i][j]=0;
          else ImageData[i][j]=255;
        }
        else if(j<110)
        {
          if(ImageData[i][j]<black_a2)ImageData[i][j]=0;
          else ImageData[i][j]=255;
        }
        else
        {
          if(ImageData[i][j]<black_a3)ImageData[i][j]=0;
          else ImageData[i][j]=255;
        }
      }
    }
  }
}*/
//==========================================================================
//函数名: enable_pit_interrupt                                                     
//函数返回: 无                                              
//参数说明: pitno: 表示pit通道号      
//功能概要: 开接收引脚的IRQ中断                                                                                                     
//==========================================================================
    void enable_pit_interrupt(uint8 pitno)
    {
      PIT_TCTRL(pitno)|=(PIT_TCTRL_TIE_MASK); //开pit中断
      switch(pitno)
      {
      case 0:
        enable_irq(68);			      //开接收引脚的IRQ中断
        break;
      case 1:
        enable_irq(69);			      //开接收引脚的IRQ中断
        break;
      case 2:
        enable_irq(70);			      //开接收引脚的IRQ中断
        break;
      case 3:
        enable_irq(71);			      //开接收引脚的IRQ中断
        break;
      }
    }



//==========================================================================
//函数名: disable_pit_interrupt                                                     
//函数返回: 无                                              
//参数说明: pitno: 表示pit通道号      
//功能概要: 开接收引脚的IRQ中断                                                                                                     
//==========================================================================
    void disable_pit_interrupt(uint8 pitno)
    {
      PIT_TCTRL(pitno)&=~(PIT_TCTRL_TIE_MASK);//关pit中断
      switch(pitno)
      {
      case 0:
        disable_irq(68);	              //关接收引脚的IRQ中断
        break;
      case 1:
        disable_irq(69);		      //关接收引脚的IRQ中断
        break;
      case 2:
        disable_irq(70);		      //关接收引脚的IRQ中断
        break;
      case 3:
        disable_irq(71);		      //关接收引脚的IRQ中断
        break;
      }
    }



//*************************中断区*****************************//
//DMA外部中断   B11口
void DMA_CHO_ISR(void)
{
        DisableInterrupts;
        //DMA_INT&=~(DMA_INT_INT0_MASK);
	DMA_INT|=DMA_INT_INT0_MASK;//清除通道0中断
	//putstr("DMA complete！");
	//GPIOA_PTOR|=0X0003E000;//A14~A17设置为输出
	row_F[imagerow]=1;//采集完成标志
	imagerow++;	
        EnableInterrupts;   
}


void porta_isr(void)//场中断，A24，下降沿中断
{ 
  DisableInterrupts;
  //PORTA_ISFR = 0xFFFFFFFF;  //Clear Port A ISR flags
  PORTA_PCR24|=PORT_PCR_ISF_MASK;//清除中断标志
  DMA0_Init();
  enable_irq(0);//使能DMA通道0完成中断
  row=0;//初始化行
  imagerow=0;//初始化采集行
  enable_irq (88);//使能B口中断 ，B10行中断
  EnableInterrupts;
  
  
}


void portb_isr(void)//行中断，B10，上升沿中断
{
  DisableInterrupts;
  
  PORTB_PCR10|=PORT_PCR_ISF_MASK;//清除中断标志位
  row++; //行计数
  
  //DMA_ERQ=0x1;//使能通道0硬件DMA请求    查看整场图像的时候用的
  if(row==data_table[imagerow])//如果当前行数据应该采集
  {
    
    DMA_ERQ|=DMA_ERQ_ERQ0_MASK;//使能通道0硬件DMA请求  
  } 
 
   else if(row>=endlineROW) //一场完成，关闭行中断
  {  
    DMA_ERQ=0x00;
    disable_irq(0);
    disable_irq(88);
    //binaryzation();
  // SCI();
    finish=1; 
    GPIOA_PDOR=GPIOA_PDOR^(GPIO_PDOR_PDO(GPIO_PIN(28)));//对引脚某位进行取反操作
  }
  EnableInterrupts;
  
}

void pit1_isr(void)
{
  PIT_TFLG(1)|=PIT_TFLG_TIF_MASK;
  enable_pit_interrupt(1);
  timer++;
}
  

void portc_isr(void)
{
  DisableInterrupts;
  
  PORTC_PCR9|=PORT_PCR_ISF_MASK;//清除中断标志位
  //light_change(Light_Run_PORT,Light_Run3);
  ir_deal();
  EnableInterrupts;
}


void pit0_isr(void)
{  
  pulse=FTM2_CNT;
  FTM2_CNT=0;
  pulse+=20;
  pulse2=get_counter_value();
  lptmr_counter_clean();
  pulse=(pulse+pulse2)/2;
  PIT_TFLG(0)|=PIT_TFLG_TIF_MASK;       //清标志
  enable_pit_interrupt(0);
}

void pit2_isr(void)
{
  P_timer++;
  PIT_TFLG(2)|=PIT_TFLG_TIF_MASK;
  enable_pit_interrupt(2);
}

/******************************************************************/
//***********************中断区结束*********************************



/*****************延时函数1s*****************/
void delays(int s)
{
  int m=0,i=0,j=0;
  for(m=0;m<s;m++)
  {
    for(i=0;i<20000;i++)
    {
      for(j=0;j<1000;j++);
    }
  }
}
void delayms(int z)
{
  int x,y;
  for(x=z;x>0;x--)
    for(y=110;y>0;y--);
}

int16 abs(int16 a)
{
  if(a>=0)return a;
  
  else return -a;
  
}


/******************************************************************/
int16 limit(int16 value,int16 top,int16 bottom)
{
  if(value>top) value=top;
  if(value<bottom) value=bottom;
  return value;
}

int16 max(int16 value1,int16 value2)
{
  if(value1>value2)
    return value1;
  else 
    return value2;
}

int16 min(int16 value1,int16 value2)
{
  if(value1<value2)
    return value1;
  else 
    return value2;
}


//void set_speed(int16 speed_low)
//{
//
//  FTM0_C0V=limit(188+speed_low/2,375,0);
//  FTM0_C3V=limit(188-speed_low/2,375,0);
//}

int set_speed(int speed_low)
{ 
  //int back_1,back_2;
  long int speed_1,speed_2;
  int speed_diff=0;

  speed_diff=Servo_value-Servo_Middle;
  if(speed_diff>0)
  {      //左转
    speed_diff=speed_diff/20;//25
    if(speed_diff>=60)
     speed_diff=59;
    
    speed_1=(long)speed_low*Wspeed_diff[1][speed_diff]/100;
    speed_2=(long)speed_low*Wspeed_diff[0][speed_diff]/100;  //+
    

    FTM0_C0V=limit(188+speed_2/2,375,0);
    FTM0_C3V=limit(188-speed_2/2,375,0);
    
    FTM0_C4V=limit(188-speed_1/2,375,0);
    FTM0_C5V=375;
    FTM0_C6V=limit(188+speed_1/2,375,0);
    
 }else { 
                //右转
     speed_diff=abs(speed_diff)/19;
    if(speed_diff>=60)
      speed_diff=59;
    speed_1=(long)speed_low*Wspeed_diff[1][speed_diff]/100;
    speed_2=(long)speed_low*Wspeed_diff[0][speed_diff]/100; 

     FTM0_C0V=limit(188+speed_1/2,375,0);//198;
    FTM0_C3V=limit(188-speed_1/2,375,0);//0;

    FTM0_C4V=limit(188-speed_2/2,375,0);//0;
     FTM0_C5V=375;
    FTM0_C6V=limit(188+speed_2/2,375,0);//190;
 } 
  return ok;
}
  

int16 subjection_k(int16 *f,int16 i)
{
  int16 value;
  
  if(i>1)
   value=subjection_k(f+1,i-1);
  else
   value=*f; 
   value=max(value,*(f+1));
  return value;
}


/************************读取速度************************/
void get_speed(void)
{
  pulse=FTM2_CNT;
  FTM2_CNT=0;
  //if(pulse==0)FTM0_C1V=0;     //如果电机没转那么电机就关掉
 //pulse2=get_counter_value();
//lptmr_counter_clean(); 
}
/************************读取速度END************************/

    
/*************加权平均法对中心线的滤波平滑处理*************/
void center_filter(void)
{
  char code_coe[5]={1,2,3,3,4};
  char code_coe_sum=13;
  int16 i=0,j=0,center_sum=0;
  for(i=52;i>1;i--)
  {
    if(position[i-2]>0)
    {
      center_sum=0;
      for(j=0;j<5;j++)center_sum+=position[j+i-2]*code_coe[j];
      position[i]=(int)(center_sum/code_coe_sum);
    }
  }  
}
/*************加权平均法对中心线的滤波平滑处理*END************/


void sub_e(uint8 value,uint8 S_end,uint8 M_start,uint8 M_end,uint8 B_start)//sub_e(E,35,30,50,45);
{
  if(value<=M_start)
  {
    sub.s=100;
    sub.m=0;
    sub.b=0;
  }
  if(value<S_end&&value>M_start)
  {
    sub.s=100*(S_end-value)/(S_end-M_start);
    sub.m=100*(value-M_start)/(S_end-M_start);
    sub.b=0;
  }
  if(value>=S_end&&value<=B_start)
  {
    sub.s=0;
    sub.m=100;
    sub.b=0;
  }
  if(value<M_end&&value>B_start)
  {
    sub.s=0;
    sub.m=100*(M_end-value)/(M_end-B_start);
    sub.b=100*(value-B_start)/(M_end-B_start);
  }
  if(value>=M_end)
  {
    sub.s=0;
    sub.m=0;
    sub.b=100;
  }
}


int16 motor_fuzzy(int16 e,int16 ec,int16 S_end,int16 M_start,int16 M_end,int16 B_start,int16 Sc_end,int16 Mc_start,int16 Mc_end,int16 Bc_start)
{
  volatile int16 ib=0,im=0,is=0,iz=0;
  volatile int16 Se=0,Me=0,Be=0;
  volatile int16 Sc=0,Mc=0,Bc=0;
  int16 vb[6]={0};
  int16 vm[6]={0};
  int16 vs[6]={0};
  int16 vz[6]={0};
  volatile long Ks=0,Km=0,Kb=0,Kz=0;
  int16 *f;
  Motor.P=0;
  Motor.I=0;

  e=abs(e);
  ec=abs(ec);
  
  sub_e(e,S_end,M_start,M_end,B_start);
 
  Se=sub.s;
  Me=sub.m;
  Be=sub.b;

  sub_e(ec,Sc_end,Mc_start,Mc_end,Bc_start);
 
  Sc=sub.s;
  Mc=sub.m;
  Bc=sub.b;


  if(Se&&Sc)  {vm[im]=min(Se,Sc);im++;}  //P建表
  if(Se&&Mc)  {vs[is]=min(Se,Mc);is++;}  
  if(Se&&Bc)  {vs[is]=min(Se,Bc);is++;}
  if(Me&&Sc)  {vm[im]=min(Me,Sc);im++;}
  if(Me&&Mc)  {vm[im]=min(Me,Mc);im++;}
  if(Me&&Bc)  {vs[is]=min(Me,Bc);is++;}
  if(Be&&Sc)  {vb[ib]=min(Be,Sc);ib++;}
  if(Be&&Mc)  {vb[ib]=min(Be,Mc);ib++;} 
  if(Be&&Bc)  {vm[im]=min(Be,Bc);im++;}


  if(vs[0]){
  f=&vs[0];
  Ks=subjection_k(f,is);
  }
  if(vm[0]){
  f=&vm[0];
  Km=subjection_k(f,im); 
  }
  if(vb[0]){
  f=&vb[0];
  Kb=subjection_k(f,ib);
  }

  Motor.P=(MP_B*Kb+MP_M*Km+MP_S*Ks)/(Kb+Km+Ks);

  
  for(is=0;is<6;is++){
    vs[is]=0;
    vm[is]=0;
    vb[is]=0;
  }
  is=0;
  im=0;
  ib=0;
  Ks=0;Km=0;Kb=0;Kz=0;
  ///////////////////////////////////////I参数
  if(Se&&Sc)  {vb[ib]=min(Se,Sc);ib++;}  //建表
  if(Se&&Mc)  {vb[ib]=min(Se,Mc);ib++;}
  if(Se&&Bc)  {vm[im]=min(Se,Bc);im++;}
  if(Me&&Sc)  {vz[iz]=min(Me,Sc);iz++;}
  if(Me&&Mc)  {vs[is]=min(Me,Mc);is++;}
  if(Me&&Bc)  {vs[is]=min(Me,Bc);is++;}
  if(Be&&Sc)  {vz[iz]=min(Be,Sc);iz++;}
  if(Be&&Mc)  {vz[iz]=min(Be,Mc);iz++;} 
  if(Be&&Bc)  {vz[iz]=min(Be,Bc);iz++;}

  if(vz[0]){
  f=&vz[0];
  Kz=subjection_k(f,iz);
  }
  if(vs[0]){
  f=&vs[0];
  Ks=subjection_k(f,is);
  }
  if(vm[0]){
  f=&vm[0];
  Km=subjection_k(f,im);
  }
  if(vb[0]){
  f=&vb[0];
  Kb=subjection_k(f,ib);
  }
  
  Motor.I=(MI_B*Kb+MI_M*Km+MI_S*Ks)/(Kb+Km+Ks+Kz);

}
//speed_fuzzy(E,Ec,35,30,50,45,3,1,7,5)
int16 speed_fuzzy(int16 e,int16 ec,int16 s_end,int16 m_start,int16 b_start,int16 m_end,int16 sc_end,int16 mc_start,int16 bc_start,int16 mc_end)
{
  volatile int16 ib=0,im=0,is=0,exspeed=0;
  volatile int16 Se=0,Me=0,Be=0;
  volatile int16 Sc=0,Mc=0,Bc=0;
  int16 vb[8]={0};
  int16 vm[8]={0};
  int16 vs[8]={0};
  volatile long Ks=0,Km=0,Kb=0;
  int16 *f;
  
  sub_e(e,s_end,m_start,b_start,m_end); //变化差
  Se=sub.s;
  Me=sub.m;
  Be=sub.b;
  
  sub_e(abs(ec),sc_end,mc_start,bc_start,mc_end);  //变化率
  Sc=sub.s;
  Mc=sub.m;
  Bc=sub.b;
  
  
  if(ec<0){                             //看到的点数在增加
  
  if(Se&&Sc)  {vs[is]=min(Se,Sc);is++;}  //建表
  if(Se&&Mc)  {vm[im]=min(Se,Mc);im++;}  //b
  if(Se&&Bc)  {vb[ib]=min(Se,Bc);ib++;}
  if(Me&&Sc)  {vm[im]=min(Me,Sc);im++;}
  if(Me&&Mc)  {vb[ib]=min(Me,Mc);ib++;}
  if(Me&&Bc)  {vb[ib]=min(Me,Bc);ib++;}
  if(Be&&Sc)  {vb[ib]=min(Be,Sc);ib++;}
  if(Be&&Mc)  {vb[ib]=min(Be,Mc);ib++;} 
  if(Be&&Bc)  {vb[ib]=min(Be,Bc);ib++;} 
    if(vs[0]){
    f=&vs[0];
    Ks=subjection_k(f,is);
    }
    if(vm[0]){
    f=&vm[0];
    Km=subjection_k(f,im);
    }
    if(vb[0]){
    f=&vb[0];
    Kb=subjection_k(f,ib);
    }
    
    exspeed=(speed_b*Kb+speed_m*Km+speed_s*Ks)/(Kb+Km+Ks);
  }else{
   
  
  if(Se&&Sc)  {vs[is]=min(Se,Sc);is++;}  //建表 4.10
  if(Se&&Mc)  {vs[is]=min(Se,Mc);is++;}
  if(Se&&Bc)  {vs[is]=min(Se,Bc);is++;}
  if(Me&&Sc)  {vm[im]=min(Me,Sc);im++;}
  if(Me&&Mc)  {vs[is]=min(Me,Mc);is++;}
  if(Me&&Bc)  {vs[is]=min(Me,Bc);is++;}
  if(Be&&Sc)  {vb[ib]=min(Be,Sc);ib++;}
  if(Be&&Mc)  {vm[im]=min(Be,Mc);im++;}
  if(Be&&Bc)  {vs[is]=min(Be,Bc);is++;}

    if(vs[0]){
    f=&vs[0];
    Ks=subjection_k(f,is);
    }
    if(vm[0]){
    f=&vm[0];
    Km=subjection_k(f,im);
    }
    if(vb[0]){
    f=&vb[0];
    Kb=subjection_k(f,ib);
    }
    

    exspeed=(speed_b*Kb+speed_m*Km+speed_s*Ks)/(Kb+Km+Ks);
  }
  
  return exspeed;
}


/*****找到莫连续几行的中心线的最大最小值******/
int16 line_min(uint8 *head,uint8 *tail){
  uint8 *p=NULL;
  point.x=*head;
  point.y=0;
  for(p=head+1;p<=tail;p++){
    if(*p<point.x){
      point.x=*p;



       point.y=p-head;
    }  
  }
  return point.x;
}                                                                       
    
  

int16 line_max(uint8 *head,uint8 *tail){
  uint8 *p=NULL;
  
  point.x=*head;
  point.y=0;
  for(p=head+1;p<=tail;p++){
    if(*p>point.x){
      point.x=*p;
      point.y=p-head;
    }
  }
  return point.x;
}

int16 line(int16 value,int16 lenth){
  static int16 line[30]={0},i=0;               //30
  int16 sum=0,j=0,k=0;
  
  if(i==30) i=0;                                   
  line[i]=value;
  k=i++;
  for(j=0;j<lenth;j++){
    if(k<0) k=29;
    sum+=line[k];
    k--;
  }
  return sum;
}



int16 stages_P(int16 value,int16 parameter)
{
  //if(!value) return unclear;

  if(value<=parameter)
  {
    if(value>=(parameter-6)){NULL;return(0);}
    else if(value>=(parameter-12)){NULL;return(-1);}  
    else if(value>=(parameter-18)){NULL;return(-2);}    
    else if(value>=(parameter-24)){NULL;return(-3);}
    else if(value>=(parameter-30)){NULL;return(-4);}
    else if(value>=(parameter-36)){NULL;return(-5);}
    else {NULL;return(-6);}
  }
  else
  {
    if(value<=(parameter+6)){NULL;return(0);}
    else if(value<=(parameter+12)){NULL;return(1);}  
    else if(value<=(parameter+18)){NULL;return(2);}    
    else if(value<=(parameter+24)){NULL;return(3);}
    else if(value<=(parameter+30)){NULL;return(4);}
    else if(value<=(parameter+36)){NULL;return(5);}
    else {NULL;return(6);}
  }
}

int16 stages_low_P(int16 value,int16 parameter)
{
  //if(!value) return unclear;
  if(value<=parameter)
  {
    if(value>=(parameter-5)){NULL;return(0);}
    else if(value>=(parameter-13)){NULL;return(-1);}  
    else if(value>=(parameter-23)){NULL;return(-2);}    
    else {NULL;return(-3);}
  }
  else
  {
    if(value<=(parameter+5)){NULL;return(0);}
    else if(value<=(parameter+13)){NULL;return(1);}  
    else if(value<=(parameter+23)){NULL;return(2);}    
    else {NULL;return(3);}
  }
}

int16 stages_D(int16 value)
{
  if(value<=0)
  {
    if(value>=-6){NULL;return(0);}
    else if(value>=-12){NULL;return(-1);}  
    else if(value>=-18){NULL;return(-2);}    
    else if(value>=-24){NULL;return(-3);}
    else if(value>=-30){NULL;return(-4);}
    else if(value>=-36){NULL;return(-5);}
    else {NULL;return(-6);}
  }
  else
  {
    if(value<=6){NULL;return(0);}
    else if(value<=12){NULL;return(1);}  
    else if(value<=18){NULL;return(2);}    
    else if(value<=24){NULL;return(3);}
    else if(value<=30){NULL;return(4);}
    else if(value<=36){NULL;return(5);}
    else {NULL;return(6);}
  }
}
/******************起始线二值化***************/
void binaryzation2(){
 uint8 i=0,j=0;
  for(i=0;i<IMG_ROWS;i++)
  {
    if(i<8)
    {
      for(j=0;j<IMG_COLS;j++)
      {
        if(j<50)
        {
          if(ImageData[i][j]<black2_d1)ImageData2[i][j]=0;
          else ImageData2[i][j]=255;
          if(ImageData[i][j]<black_d1)ImageData[i][j]=0;
          else ImageData[i][j]=255;
          
        }
        else if(j<110)
        {
          if(ImageData[i][j]<black2_d2)ImageData2[i][j]=0;
          else ImageData2[i][j]=255;
          if(ImageData[i][j]<black_d2)ImageData[i][j]=0;
          else ImageData[i][j]=255;
          
        }
        else
        {
           if(ImageData[i][j]<black2_d3)ImageData2[i][j]=0;
          else ImageData2[i][j]=255;
          if(ImageData[i][j]<black_d3)ImageData[i][j]=0;
          else ImageData[i][j]=255;
         
        }
      }
    }
    else if(i<20)
    {
      for(j=0;j<IMG_COLS;j++)
      {
        if(j<50)
        {
          if(ImageData[i][j]<black2_c1)ImageData2[i][j]=0;
          else ImageData2[i][j]=255;
          if(ImageData[i][j]<black_c1)ImageData[i][j]=0;
          else ImageData[i][j]=255; 
        }
        else if(j<110)
        {
          if(ImageData[i][j]<black2_c2)ImageData2[i][j]=0;
          else ImageData2[i][j]=255;
          if(ImageData[i][j]<black_c2)ImageData[i][j]=0;
          else ImageData[i][j]=255;
        }
        else
        {
           if(ImageData[i][j]<black2_c3)ImageData2[i][j]=0;
          else ImageData2[i][j]=255;
          if(ImageData[i][j]<black_c3)ImageData[i][j]=0;
          else ImageData[i][j]=255;
         
        }
      }
    }
    else if(i<34)
    {
      for(j=0;j<IMG_COLS;j++)
      {
        if(j<50)
        {
           if(ImageData[i][j]<black2_b1)ImageData2[i][j]=0;
          else ImageData2[i][j]=255;
          if(ImageData[i][j]<black_b1)ImageData[i][j]=0;
          else ImageData[i][j]=255;
         
        }
        else if(j<110)
        {
           if(ImageData[i][j]<black2_b2)ImageData2[i][j]=0;
          else ImageData2[i][j]=255;
          if(ImageData[i][j]<black_b2)ImageData[i][j]=0;
          else ImageData[i][j]=255;
         
        }
        else
        {
          if(ImageData[i][j]<black2_b3)ImageData2[i][j]=0;
          else ImageData2[i][j]=255;
          if(ImageData[i][j]<black_b3)ImageData[i][j]=0;
          else ImageData[i][j]=255;
        }
      }
    }
    else
    {
      for(j=0;j<IMG_COLS;j++)
      {
        if(j<50)
        {
          if(ImageData[i][j]<black2_a1)ImageData2[i][j]=0;
          else ImageData2[i][j]=255;
          if(ImageData[i][j]<black_a1)ImageData[i][j]=0;
          else ImageData[i][j]=255;
        }
        else if(j<110)
        {
          if(ImageData[i][j]<black2_a2)ImageData2[i][j]=0;
          else ImageData2[i][j]=255;
          if(ImageData[i][j]<black_a2)ImageData[i][j]=0;
          else ImageData[i][j]=255;
        }
        else
        {
          if(ImageData[i][j]<black2_a3)ImageData2[i][j]=0;
          else ImageData2[i][j]=255;
          if(ImageData[i][j]<black_a3)ImageData[i][j]=0;
          else ImageData[i][j]=255;
        }
      }
    }
  }
}
/****************二值化*****************/
void binaryzation()
{
 uint8 i=0,j=0;
  for(i=0;i<IMG_ROWS;i++)
  {
    if(i<8)
    {
      for(j=0;j<IMG_COLS;j++)
      {
        if(j<50)
        {
          if(ImageData[i][j]<black_d1)ImageData[i][j]=0;
          else ImageData[i][j]=255;
        }
        else if(j<110)
        {
          if(ImageData[i][j]<black_d2)ImageData[i][j]=0;
          else ImageData[i][j]=255;
        }
        else
        {
          if(ImageData[i][j]<black_d3)ImageData[i][j]=0;
          else ImageData[i][j]=255;
        }
      }
    }
    else if(i<20)
    {
      for(j=0;j<IMG_COLS;j++)
      {
        if(j<50)
        {
          if(ImageData[i][j]<black_c1)ImageData[i][j]=0;
          else ImageData[i][j]=255;
        }
        else if(j<110)
        {
          if(ImageData[i][j]<black_c2)ImageData[i][j]=0;
          else ImageData[i][j]=255;
        }
        else
        {
          if(ImageData[i][j]<black_c3)ImageData[i][j]=0;
          else ImageData[i][j]=255;
        }
      }
    }
    else if(i<34)
    {
      for(j=0;j<IMG_COLS;j++)
      {
        if(j<50)
        {
          if(ImageData[i][j]<black_b1)ImageData[i][j]=0;
          else ImageData[i][j]=255;
        }
        else if(j<110)
        {
          if(ImageData[i][j]<black_b2)ImageData[i][j]=0;
          else ImageData[i][j]=255;
        }
        else
        {
          if(ImageData[i][j]<black_b3)ImageData[i][j]=0;
          else ImageData[i][j]=255;
        }
      }
    }
    else
    {
      for(j=0;j<IMG_COLS;j++)
      {
        if(j<50)
        {
          if(ImageData[i][j]<black_a1)ImageData[i][j]=0;
          else ImageData[i][j]=255;
        }
        else if(j<110)
        {
          if(ImageData[i][j]<black_a2)ImageData[i][j]=0;
          else ImageData[i][j]=255;
        }
        else
        {
          if(ImageData[i][j]<black_a3)ImageData[i][j]=0;
          else ImageData[i][j]=255;
        }
      }
    }
  }
    for(i=0;i<56;i++)
    position[i]=0;  
}

int Judge_startline2(void)
{
  int i,j,m=0;
  int flag1=0,find_flag=0,num_l=0,num_r=0,startline=0,num_w1=0,num_w2=0,num_w=0;
  int position_num=0,position_old=0,num=0,find_old_num=0,a;
  uint16 median=0,b=0,c=0,d=0,e=0,f=0;
  uint16 start_p;
  j=15;
  i=0;
  while(j<135)
  {
    find_flag=0;
    ption[j]=0;
    for(i=13;i<53;i++)
    {
      median=0;
      num=0;
      if(!ImageData2[i+2][j]&&ImageData2[i][j]&&ImageData2[i+1][j])
      {
        for(i=i+2;i<55;i++)
        {
          if(i<23)
          {
            a=2;            
          }
          if(i>=23&&i<33)
          {
            a=3;
          }
          if(i>=33&&i<45)
          {
            a=4;
          }
          if(i>=45&&i<55)
          {
            a=4;
          }
          if(!ImageData2[i][j]&&i<54)
          {
            median+=i;
            num++;
          }
          else
          {
            if(num<=a&&num>0)
            {              
              median=median/num;
            }
            else 
              median=0;
            if(median)
            {
              ption[j]=median;
              find_flag=1;				
            }
            break;
          }
        }			
        if(find_flag)
          break;
      }	
    }
    j++;
  }
  num=0;
  start_p=0;
  for(j=15;j<135;j++)
  {
    if(ption[j])
    {
      start_p+=ption[j];
      num++;
    }
  }
  if(num>15)
  {
    start_p=start_p/num;
    num=0;
    for(j=15;j<135;j++)
    {
      if(ption[j]&&abs(ption[j]-start_p)>4&&abs(j-position[start_p])<15)
      {
        ption[j]=0;
      }
      if(ption[j]&&abs(ption[j]-start_p)>25)
        ption[j]=0;
    }
    start_p=0;
    for(j=15;j<135;j++)
    {
      if(ption[j])
      {
        start_p+=ption[j];
        num++;
        b+=j;
      }
    }
    if(num)
      start_p=start_p/num;
    b=b/num;
    if(b<52&&b>106)  return 0;	
    if(start_p)
    {
      if(start_p<25)  f=13;
      else if(start_p<38) f=16;
      else if(start_p<55) f=17;}
  }
  else
    return 0;
  num=0;
  num_l=0;
  for(j=15;j<=position[start_p];j++)
  {
    if(ption[j])
    {
      c+=ption[j];
      num_l++;
    }
  }
  c=c/num_l;
  num_r=0;
  for(j=position[start_p];j<135;j++)
  {
    if(ption[j])
    {
      d+=ption[j];
      num_r++;
    }
  }
  d=d/num_r;
  if(start_p<=20)
  {
    if(num_l<20&&num_l>=10&&num_r<20&&num_r>=10)
    {
      if(abs(num_l-num_r)<5)
      {
        startline=1;
      }
    }
  }
  else
    if(start_p<=30)
    {
      if(num_l<24&&num_l>13&&num_r<24&&num_r>13)
      {
        if(abs(num_l-num_r)<6)
        {
          startline=1;
        }
      }
    }
    else 
      if(start_p<=38)
      {
	if(num_l<33&&num_l>18&&num_r<33&&num_r>18)
	{
          if(abs(num_l-num_r)<8)
          {
            startline=1;
          }
	}
      }
  else 
    {
      if(num_l<36&&num_l>22&&num_r<36&&num_r>22)
      {
        if(abs(num_l-num_r)<8)
        {
          startline=1;
        }
      }
    }
  if(startline==1&&start_p<40)
  {
    for(i=0;i<160;i++)
    {
      if(!ImageData[start_p][i])
        num_w1++;
    }
    for(i=15;i<135;i++)
    {
      if(ption[i]==start_p)
        num_w2++;
    }
    num_w=num_w1-num_w2;
    if(start_p<22)
    {
      if(num_w<50)
        startline=0;
    }
    else 
      if(start_p<34)
      {
        if(num_w<33)
          startline=0;
      }
      else
      {
        if(num_w<14)
          startline=0;
      }
  }
           flag1=startline;
           return flag1;
}

int get_root()
{
  uint8 i=0,j=0,position_old=0,start=0,windage=28,endR=0,endL=0,cuo_root=0,cuo_root1=0;
  uint8 find_num=0,find_flag=0,hang_num=0,num=0;
  for(i=54;i>=45;i--)
    {
      if(start==1)
	{
          find_flag=0;
          endL=limit(position_old-windage,158,1);
          endR= limit(position_old+windage,158,1);	
	  for(j=endL;j<=endR;j++)
	   {
	      if(!find_flag&&ImageData[i][j]&&ImageData[i][j+1]&&!ImageData[i][j+2])
		{
                  for(j+=2;j<endR;j++)
                  {
                    if(!ImageData[i][j]&&j<endR-1)
                    {
                      position[i]+=j;
                      find_num++;
                      find_flag=1;	
                    }
                    else
                    {
                      if(ImageData[i][j]||(j==endR-1))
                        {
	                  if(j==endR-1)
                            {
                              if(!ImageData[i][j])
                                {
                                    for(j=endR;j<=limit(endR+20,158,1);j++)
                                      {
                                        if(!ImageData[i][j])
                                          {
                                            position[i]+=j;
                                            find_num++;
                                          } 
                                        else
                                          break;
                                      }
                                }
                            }   
                          if(!ImageData[i][j])
                            {
                               position[i]=0;
                               find_num=0;
                               break;
                            }
                          if(find_num>2&&find_num<16) 
                          {
                            cuo_root1=0;
                            position[i]=position[i]/find_num;
                            find_num=0;
                          }
                        else
                          {
                            if(find_num>=16)//十字交叉处理
			      {
                                position[i]=0;
                                find_num=0;
                                cuo_root1=1;
                              }	 
                            position[i]=0;
                            find_num=0;
                          }
                        }
                      if(position[i]&&abs(position[i]-position_old)>15) 
                        {
                          position[i]=0;										   
                        }
                      if(cuo_root1==0)
                      break;
                    }
                   }
	        }
	      }
          }
      else
	{	find_flag=0;
	  for(j=4;j<155;j++)
	    {
	      if(!find_flag&&ImageData[i][j-1]&&ImageData[i][j]&&ImageData[i][j+1]&&!ImageData[i][j+2])
	        {
	          for(j+=2;j<153;j++)
	            {
		      if(!ImageData[i][j]&&j<152)
			{
                          position[i]+=j;
                          find_num++;
                          find_flag=1;
		        }
		      else
			{
			  if(find_num>2&&find_num<15) //中线黑点数大概9个左右
			    {
              
                                cuo_root=0;
                                position[i]=position[i]/find_num; 
                                find_num=0;
                           
			    }
			  else
			    {
			      if(find_num>=15)
				{
				  position[i]=0;
				  find_num=0;
				   cuo_root=1;
			        }	 
				position[i]=0;
			        find_num=0;
			    }
                           if(cuo_root==0)
                           break; 
		          } 
		    }
	         }
            }
	}
	if(position[i])
          {
            position_old=position[i]; 
            start=1;
          }
	else
	  position[i]=0;
  }

	
	for(i=54;i>=45;i--)
          {
             if(position[i])
             {
              start_position=position[i];
              num++;
             }
          } 
	if(num>1)
	return ok;
	else
   return fail;
  
       
}

int Judge_end(unsigned char end_hang)
{
	unsigned char black_num=0,i=0,j=0,black_flag1=0,black_flag2=0;
	if(end_hang<=19)
          {
            for(i=end_hang-1;i>=end_hang-1;i--)
              {
                if(!position[i])
                black_num++;  
              }	
          }
	else 
          if(end_hang<=28)
          {
            for(i=end_hang-1;i>=end_hang-2;i--)
              {
                if(!position[i])
                black_num++;  
              }
            if(black_num<2)
            black_num=0;
          }
        else
          {
            for(i=end_hang-1;i>=end_hang-4;i--)
              {
                if(!position[i])
                black_num++;  
              }
            if(black_num<4)
            black_num=0;
          }
          
	if((black_num==1)||(black_num==2)||(black_num==4))
	  return ok;
	else
          return fail;
}



void  position_youhua()
{
    int m=0,end_flag=0,num=0,i=0;
	for(m=44;m>2;m--)
        {
	   if(!position[m])
           {
		end_flag=Judge_end(m);
		if(end_flag)
                {
		  for(;m>=3;m--)
                  {
		    position[m]=0;
		  }
		}
		//break;
	   }
        } 	
	for(i=44;i>=0;i--)
        {
	  if(!position[i])
            {
              if(i>=24)
              {
		num++;
	        if(num>4)
                  {
		    for(;i>=0;i--)
                     {
		        position[i]=0;
		     }
			break;
		  }
              }
              else
                {
		num++;
	        if(num>2)
                  {
		    for(;i>=0;i--)
                     {
		        position[i]=0;
		     }
			break;
		  }
              }
	   }                	
           else
                num=0;
	     
	}
        //center_filter();
}
/**********************二值化END*********************/
//////////////找黑线中心//////////////////
int find_position()
 {
  int i=0,j=0,windage=0,root_flag=0;
  int position_width=0;
  int position_num=0,position_old=0;
  int endL=0,endR=0,num=0,ave=0,num1=0,cha=0,find_num=0;
  int median=0;
  binaryzation2();
  root_flag=get_root(); //root_flag=0k;找根成功
  if(!root_flag)
  return fail;
  i=44;
  while(i>=0)
     {
        position[i]=0;
        if(i>40)
        {
          windage=40;
          position_width=4;//22
        }
       else
       if(i>30)
        {
          windage=40;
          position_width=4;//20
        } 
        else
        if(i>20)
        {
          windage=38;
          position_width=3;//20
        } 
        else
        if(i>10)
        {
          //windage=23;
          windage=17;
          position_width=3;//10
        } 
        else
        {
          //windage=21;
          windage=17;
          position_width=3;//9
        }
   if(i==44)
   {
     position_old=start_position; 
     start_position=0;
   }
 
     endL=limit(position_old-windage,158,1);
     endR= limit(position_old+windage,158,1);
     for(j=endL;j<=endR;j++)
     {
        if(!ImageData[i][j+2]&&ImageData[i][j]&&ImageData[i][j+1])
        {
                  for(j=j+2;j<=endR;j++)
                  {
                      if(!ImageData[i][j]&&j<endR)
                      {
                        median+=j;
                         position_num++;
                      }
                      else
                      {
                          if(ImageData[i][j]||(j==endR))
                          {
	                      if(j==endR)
                                {
                                   if(!ImageData[i][j])
                                     {
                                        for(j=endR;j<=limit(endR+20,158,1);j++)
	                                    {
	                                        if(!ImageData[i][j])
		                                  {
		                                     median+=j;
		                                     position_num++;
		                                  } 
		                                else
	                                          break;
	                                    }
	                             }
	                        }
                              if(!ImageData[i][j])
                                {
                                   median=0;
                                   position_num=0;
                                   break;
                                }
                            if(position_num>position_width-3&&position_num<position_width+14) 
                            {
                               median=median/position_num;
                               position_num=0;
                             }
                            else
                            {
                             median=0;
                             position_num=0;	    			
                            }
                          }                 
                        if(median&&abs(median-position_old)>28) 
                            median=0;
                        else
                        {
                            if(median)
                            {			
                              if(!position[i])
                              {
                                      position[i]=median;
                                      median=0; 
                                      position_num=0;
                                       //break;
                              }
                              else
                              {
                                   if(abs(position[i]-position_old)>abs(median-position_old))
                                     {
                                        position[i]=median;
                                        median=0;
                                        break;
                                     }
                                   else
                                     {
                                       median=0;
                                       break;
                                     }
                              }
                            }
                         }
                     }                     
             } 
        }
     }
     if(position[i])
     position_old=position[i];
     if(position[i]&&(position[i]<3||position[i]>157))
    {
           for(;i>=0;i--)
           {
               position[i]=0;
           }
          break;
   }
    i--;
  }
  position_youhua();
  for(i=54;i>=4;i--)//补线
    {
      if(!position[i]&&(i==54))
      {
        while(!position[i])
          {
            i--;
            num1++;
          }
        if(position[i]&&num1<5)
          {
            ave=abs(position[i]-position[i-1]);
            if(position[i]>position[i-1])
              {
                for(;i<54;i++)
                  {
                    position[i+1]=position[i]+ave;
                  }
              }
            else
              {
                for(;i<54;i++)
                  {
                    position[i+1]=position[i]-ave;
                  }	
              }
          }
        i=i-num1+1;
        num1=0;
      }
      if(position[i]&&!position[i-1])
        {				
          while(!position[i-1])
            {
              i--;
              num++;
            }
          if(position[i-1])
            {
              if(num<5)
                {
                  ave=abs((position[i+num]-position[i-1])/num);
                  if(position[i+num]<position[i-1])  
                    {	
                      for(;num>0;num--)
                        {
                          position[i+num-1]=position[i+num]+ave;
                        }
                      num=0;
                    }
                  else
                    {
                      for(;num>0;num--)
                        {
                          position[i+num-1]=position[i+num]-ave;
                        }
                      num=0;	
                    }
                }
            }
         }
    }
for(i=6;i>0;i--)
  {
     cha=abs(position[i]-position[i-1]);
     if(cha>6)
     for(;i>0;i--)
     position[i-1]=0;
  }
  center_filter();
  return ok;
}
    



int8 choose_PD()
{
  int16 i=0,num=0;
  uint8 track_form=0;//j=0,
  int8 position_now_low_P=0,position_now_P=0,position_now_D=0;
  int16 line_head=0,line_end=54;//,line_mid=0;//line_head=30
  int16 line_mid_d=0,line_end_d=0;
  int16 head=0,middle=0,end=0;
  while(!position[i]&&i<IMG_ROWS)i++;
  dijihang=i;

  if(i<7)
  {
    track_form=3;
    Servo.P=P_High;
    Servo.D=D_High;
    servo.head=H_High;
    servo.tail=T_High;
  }
  if(i>=7&&i<22)
  {
    track_form=2;
    Servo.P=P_Mid;
    Servo.D=D_Mid;
    servo.head=H_Mid;
    servo.tail=T_Mid;
  }
  if(i>=22&&i<33)
  {
    track_form=2;
    Servo.P=P_Mid1;
    Servo.D=D_Mid1;
    servo.head=H_Mid1;
    servo.tail=T_Mid1;
  }
  if(i>=33)
  {
    track_form=1;
    Servo.P=P_Low;
    Servo.D=D_Low;
    servo.head=H_Low;
    servo.tail=T_Low;
  }
    for(i=servo.head;i<servo.tail;i++)
    {
      if(position[i])
      {
        num++;
        position_now+=position[i];
      }  
    }
    if(num!=0)
    {
      position_now=position_now/num;   //算出当场的中心位置
      num=0;
    }
  // for(i=45;i<55;i++)
      for(i=50;i<55;i++)
    {
      if(position[i])
      {
        num++;
        position_now_low+=position[i];
      }  
    }
    if(num!=0)
    {
      position_now_low=position_now_low/num;
      num=0;
    }
    if(!position_now)
    {
      if(!position_now_low)
        position_now=position_now_low;
      else
        return fail;
    }
    while(position[line_head]==0)  line_head++;
    while(position[line_end]==0)  line_end--;
    line_end_d=(line_head+2*line_end)/3;
    line_mid_d=(line_end+2*line_head)/3;
    for(i=line_head;i<line_mid_d;i++)head+=position[i];
    head=head/(line_mid_d-line_head);
    for(i=line_mid_d;i<=line_end_d;i++)middle+=position[i];
    middle=middle/(line_end_d-line_mid_d+1); 
   if(dijihang<=50)
   {
     if(dijihang<=21)
      position_diff=(middle-head); 
     else
       position_diff=middle-head;
   }
    else
     position_diff=position[line_end]-position[line_head];
      
  
    //position_now_low_P=stages_low_P(position_now_low,video_Middle);
    position_now_P=stages_P(position_now,video_Middle);
    position_now_D=stages_D(position_diff);
  switch(position_now_P)
  {
  case -6:
    {
      switch(position_now_D)
      {
     case -6:
        {
          switch(track_form)
          {
        //case 1:Servo.P+=5;Servo.D+=1;break;//-6-6
          case 1:Servo.P+=4;Servo.D+=2;break;
          case 2:Servo.P+=3;Servo.D+=1;break;
          case 3:Servo.P+=2;Servo.D+=0;break;
          }
        }break;
      case -5:
        {
          switch(track_form)
          {
            //case 1:Servo.P+=5;Servo.D+=1;break;
          case 1:Servo.P+=4;Servo.D+=1;break;
          case 2:Servo.P+=3;Servo.D+=0;break;
          case 3:Servo.P+=2;Servo.D-=1;break;
          }
        }break;
      case -4:
        {
          switch(track_form)
          {
            case 1:Servo.P+=4;Servo.D+=0;break;
            case 2:Servo.P+=3;Servo.D-=1;break;
            case 3:Servo.P+=2;Servo.D-=2;break;
          }
        }break;
      case -3:
        {
          switch(track_form)
          {
          case 1:Servo.P+=4;Servo.D-=1;break;
          case 2:Servo.P+=3;Servo.D-=2;break;
          case 3:Servo.P+=2;Servo.D-=3;break;
          }
        }break;
      case -2:
        {
          switch(track_form)
          {
          case 1:Servo.P+=4;Servo.D-=2;break;
          case 2:Servo.P+=3;Servo.D-=3;break;
          case 3:Servo.P+=2;Servo.D-=4;break;
          }
        }break;
      case -1:
        {
          switch(track_form)
          {
          case 1:Servo.P+=4;Servo.D-=3;break;
          case 2:Servo.P+=3;Servo.D-=4;break;
          case 3:Servo.P+=2;Servo.D-=5;break;
          }
        }break;
      case 0:
         {
          switch(track_form)
          {
          case 1:Servo.P+=4;break;
          case 2:Servo.P+=3;break;
          case 3:Servo.P+=2;break;
          }
        }break;
      case 6:
        {
          switch(track_form)
          {
        //case 1:Servo.P+=5;Servo.D+=1;break;//-6-6
          case 1:Servo.P+=4;Servo.D+=2;break;
          case 2:Servo.P+=3;Servo.D+=1;break;
          case 3:Servo.P+=2;Servo.D+=0;break;
          }
        }break;
      case 5:
        {
          switch(track_form)
          {
            //case 1:Servo.P+=5;Servo.D+=1;break;
          case 1:Servo.P+=4;Servo.D+=1;break;
          case 2:Servo.P+=3;Servo.D+=0;break;
          case 3:Servo.P+=2;Servo.D-=1;break;
          }
        }break;
      case 4:
        {
          switch(track_form)
          {
            case 1:Servo.P+=4;Servo.D+=0;break;
            case 2:Servo.P+=3;Servo.D-=1;break;
            case 3:Servo.P+=2;Servo.D-=2;break;
          }
        }break;
      case 3:
        {
          switch(track_form)
          {
          case 1:Servo.P+=4;Servo.D-=1;break;
          case 2:Servo.P+=3;Servo.D-=2;break;
          case 3:Servo.P+=2;Servo.D-=3;break;
          }
        }break;
      case 2:
        {
          switch(track_form)
          {
          case 1:Servo.P+=4;Servo.D-=2;break;
          case 2:Servo.P+=3;Servo.D-=3;break;
          case 3:Servo.P+=2;Servo.D-=4;break;
          }
        }break;
      case 1:
        {
          switch(track_form)
          {
          case 1:Servo.P+=4;Servo.D-=3;break;
          case 2:Servo.P+=3;Servo.D-=4;break;
          case 3:Servo.P+=2;Servo.D-=5;break;
          }
        }break;
      }
    }break;
  case -5:
    {
      switch(position_now_D)
      {
      case -6:
        {
          switch(track_form)
          {
        //case 1:Servo.P+=5;Servo.D+=1;break;//-6-6
          case 1:Servo.P+=3;Servo.D+=2;break;
          case 2:Servo.P+=2;Servo.D+=1;break;
          case 3:Servo.P+=1;Servo.D+=0;break;
          }
        }break;
      case -5:
        {
          switch(track_form)
          {
            //case 1:Servo.P+=5;Servo.D+=1;break;
          case 1:Servo.P+=3;Servo.D+=1;break;
          case 2:Servo.P+=2;Servo.D+=0;break;
          case 3:Servo.P+=1;Servo.D-=1;break;
          }
        }break;
      case -4:
        {
          switch(track_form)
          {
            case 1:Servo.P+=3;Servo.D+=0;break;
            case 2:Servo.P+=2;Servo.D-=1;break;
            case 3:Servo.P+=1;Servo.D-=2;break;
          }
        }break;
      case -3:
        {
          switch(track_form)
          {
          case 1:Servo.P+=3;Servo.D-=1;break;
          case 2:Servo.P+=2;Servo.D-=2;break;
          case 3:Servo.P+=1;Servo.D-=3;break;
          }
        }break;
      case -2:
        {
          switch(track_form)
          {
          case 1:Servo.P+=3;Servo.D-=2;break;
          case 2:Servo.P+=2;Servo.D-=3;break;
          case 3:Servo.P+=1;Servo.D-=4;break;
          }
        }break;
      case -1:
        {
          switch(track_form)
          {
          case 1:Servo.P+=3;Servo.D-=3;break;
          case 2:Servo.P+=2;Servo.D-=4;break;
          case 3:Servo.P+=1;Servo.D-=5;break;
          }
        }break;
      case 0:
         {
          switch(track_form)
          {
          case 1:Servo.P+=3;break;
          case 2:Servo.P+=2;break;
          case 3:Servo.P+=1;break;
          }
        }break;
      case 6:
        {
          switch(track_form)
          {
        //case 1:Servo.P+=5;Servo.D+=1;break;//-6-6
          case 1:Servo.P+=3;Servo.D+=2;break;
          case 2:Servo.P+=2;Servo.D+=1;break;
          case 3:Servo.P+=1;Servo.D+=0;break;
          }
        }break;
      case 5:
        {
          switch(track_form)
          {
            //case 1:Servo.P+=5;Servo.D+=1;break;
          case 1:Servo.P+=3;Servo.D+=1;break;
          case 2:Servo.P+=2;Servo.D+=0;break;
          case 3:Servo.P+=1;Servo.D-=1;break;
          }
        }break;
      case 4:
        {
          switch(track_form)
          {
            case 1:Servo.P+=3;Servo.D+=0;break;
            case 2:Servo.P+=2;Servo.D-=1;break;
            case 3:Servo.P+=1;Servo.D-=2;break;
          }
        }break;
      case 3:
        {
          switch(track_form)
          {
          case 1:Servo.P+=3;Servo.D-=1;break;
          case 2:Servo.P+=2;Servo.D-=2;break;
          case 3:Servo.P+=1;Servo.D-=3;break;
          }
        }break;
      case 2:
        {
          switch(track_form)
          {
          case 1:Servo.P+=3;Servo.D-=2;break;
          case 2:Servo.P+=2;Servo.D-=3;break;
          case 3:Servo.P+=1;Servo.D-=4;break;
          }
        }break;
      case 1:
        {
          switch(track_form)
          {
          case 1:Servo.P+=3;Servo.D-=3;break;
          case 2:Servo.P+=2;Servo.D-=4;break;
          case 3:Servo.P+=1;Servo.D-=5;break;
          }
        }break;
      }
    }break;
  case -4:
    {
      switch(position_now_D)
      {
         case -6:
        {
          switch(track_form)
          {
        //case 1:Servo.P+=5;Servo.D+=1;break;//-6-6
          case 1:Servo.P+=2;Servo.D+=2;break;
          case 2:Servo.P+=1;Servo.D+=1;break;
          case 3:Servo.P+=0;Servo.D+=0;break;
          }
        }break;
      case -5:
        {
          switch(track_form)
          {
            //case 1:Servo.P+=5;Servo.D+=1;break;
          case 1:Servo.P+=2;Servo.D+=1;break;
          case 2:Servo.P+=1;Servo.D+=0;break;
          case 3:Servo.P+=0;Servo.D-=1;break;
          }
        }break;
      case -4:
        {
          switch(track_form)
          {
            case 1:Servo.P+=2;Servo.D+=0;break;
            case 2:Servo.P+=1;Servo.D-=1;break;
            case 3:Servo.P+=0;Servo.D-=2;break;
          }
        }break;
      case -3:
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D-=1;break;
          case 2:Servo.P+=1;Servo.D-=2;break;
          case 3:Servo.P+=0;Servo.D-=3;break;
          }
        }break;
      case -2:
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D-=2;break;
          case 2:Servo.P+=1;Servo.D-=3;break;
          case 3:Servo.P+=0;Servo.D-=4;break;
          }
        }break;
      case -1:
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D-=3;break;
          case 2:Servo.P+=1;Servo.D-=4;break;
          case 3:Servo.P+=0;Servo.D-=5;break;
          }
        }break;
      case 0:
         {
          switch(track_form)
          {
          case 1:Servo.P+=2;break;
          case 2:Servo.P+=1;break;
          case 3:Servo.P+=0;break;
          }
        }break;
      case 6:
        {
          switch(track_form)
          {
        //case 1:Servo.P+=5;Servo.D+=1;break;//-6-6
          case 1:Servo.P+=2;Servo.D+=2;break;
          case 2:Servo.P+=1;Servo.D+=1;break;
          case 3:Servo.P+=0;Servo.D+=0;break;
          }
        }break;
      case 5:
        {
          switch(track_form)
          {
            //case 1:Servo.P+=5;Servo.D+=1;break;
          case 1:Servo.P+=2;Servo.D+=1;break;
          case 2:Servo.P+=1;Servo.D+=0;break;
          case 3:Servo.P+=0;Servo.D-=1;break;
          }
        }break;
      case 4:
        {
          switch(track_form)
          {
            case 1:Servo.P+=2;Servo.D+=0;break;
            case 2:Servo.P+=1;Servo.D-=1;break;
            case 3:Servo.P+=0;Servo.D-=2;break;
          }
        }break;
      case 3:
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D-=1;break;
          case 2:Servo.P+=1;Servo.D-=2;break;
          case 3:Servo.P+=0;Servo.D-=3;break;
          }
        }break;
      case 2:
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D-=2;break;
          case 2:Servo.P+=1;Servo.D-=3;break;
          case 3:Servo.P+=0;Servo.D-=4;break;
          }
        }break;
      case 1:
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D-=3;break;
          case 2:Servo.P+=1;Servo.D-=4;break;
          case 3:Servo.P+=0;Servo.D-=5;break;
          }
        }break;
      }
    }break;
 case -3:
    {
      switch(position_now_D)
      {
   case -6:
        {
          switch(track_form)
          {
        //case 1:Servo.P+=5;Servo.D+=1;break;//-6-6
          case 1:Servo.P+=1;Servo.D+=2;break;
          case 2:Servo.P+=0;Servo.D+=1;break;
          case 3:Servo.P-=1;Servo.D+=0;break;
          }
        }break;
      case -5:
        {
          switch(track_form)
          {
            //case 1:Servo.P+=5;Servo.D+=1;break;
          case 1:Servo.P+=1;Servo.D+=1;break;
          case 2:Servo.P+=0;Servo.D+=0;break;
          case 3:Servo.P-=1;Servo.D-=1;break;
          }
        }break;
      case -4:
        {
          switch(track_form)
          {
            case 1:Servo.P+=1;Servo.D+=0;break;
            case 2:Servo.P+=0;Servo.D-=1;break;
            case 3:Servo.P-=1;Servo.D-=2;break;
          }
        }break;
      case -3:
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D-=1;break;
          case 2:Servo.P+=0;Servo.D-=2;break;
          case 3:Servo.P-=1;Servo.D-=3;break;
          }
        }break;
      case -2:
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D-=2;break;
          case 2:Servo.P+=0;Servo.D-=3;break;
          case 3:Servo.P-=1;Servo.D-=4;break;
          }
        }break;
      case -1:
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D-=3;break;
          case 2:Servo.P+=0;Servo.D-=4;break;
          case 3:Servo.P-=1;Servo.D-=5;break;
          }
        }break;
      case 0:
         {
          switch(track_form)
          {
          case 1:Servo.P+=1;break;
          case 2:Servo.P+=0;break;
          case 3:Servo.P-=1;break;
          }
        }break;
      case 6:
        {
          switch(track_form)
          {
        //case 1:Servo.P+=5;Servo.D+=1;break;//-6-6
          case 1:Servo.P+=1;Servo.D+=2;break;
          case 2:Servo.P+=0;Servo.D+=1;break;
          case 3:Servo.P-=1;Servo.D+=0;break;
          }
        }break;
      case 5:
        {
          switch(track_form)
          {
            //case 1:Servo.P+=5;Servo.D+=1;break;
          case 1:Servo.P+=1;Servo.D+=1;break;
          case 2:Servo.P+=0;Servo.D+=0;break;
          case 3:Servo.P-=1;Servo.D-=1;break;
          }
        }break;
      case 4:
        {
          switch(track_form)
          {
            case 1:Servo.P+=1;Servo.D+=0;break;
            case 2:Servo.P+=0;Servo.D-=1;break;
            case 3:Servo.P-=1;Servo.D-=2;break;
          }
        }break;
      case 3:
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D-=1;break;
          case 2:Servo.P+=0;Servo.D-=2;break;
          case 3:Servo.P-=1;Servo.D-=3;break;
          }
        }break;
      case 2:
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D-=2;break;
          case 2:Servo.P+=0;Servo.D-=3;break;
          case 3:Servo.P-=1;Servo.D-=4;break;
          }
        }break;
      case 1:
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D-=3;break;
          case 2:Servo.P+=0;Servo.D-=4;break;
          case 3:Servo.P-=1;Servo.D-=5;break;
          }
        }break;
      }
    }break;
 case -2:
    {
      switch(position_now_D)
      {
     case -6:
        {
          switch(track_form)
          {
        //case 1:Servo.P+=5;Servo.D+=1;break;//-6-6
          case 1:Servo.P+=0;Servo.D+=2;break;
          case 2:Servo.P-=1;Servo.D+=1;break;
          case 3:Servo.P-=2;Servo.D+=0;break;
          }
        }break;
      case -5:
        {
          switch(track_form)
          {
            //case 1:Servo.P+=5;Servo.D+=1;break;
          case 1:Servo.P+=0;Servo.D+=1;break;
          case 2:Servo.P-=1;Servo.D+=0;break;
          case 3:Servo.P-=2;Servo.D-=1;break;
          }
        }break;
      case -4:
        {
          switch(track_form)
          {
            case 1:Servo.P+=0;Servo.D+=0;break;
            case 2:Servo.P-=1;Servo.D-=1;break;
            case 3:Servo.P-=2;Servo.D-=2;break;
          }
        }break;
      case -3:
        {
          switch(track_form)
          {
          case 1:Servo.P+=0;Servo.D-=1;break;
          case 2:Servo.P-=1;Servo.D-=2;break;
          case 3:Servo.P-=2;Servo.D-=3;break;
          }
        }break;
      case -2:
        {
          switch(track_form)
          {
          case 1:Servo.P+=0;Servo.D-=2;break;
          case 2:Servo.P-=1;Servo.D-=3;break;
          case 3:Servo.P-=2;Servo.D-=4;break;
          }
        }break;
      case -1:
        {
          switch(track_form)
          {
          case 1:Servo.P+=0;Servo.D-=3;break;
          case 2:Servo.P-=1;Servo.D-=4;break;
          case 3:Servo.P-=2;Servo.D-=5;break;
          }
        }break;      
      case 0:
         {
          switch(track_form)
          {
          case 1:Servo.P+=0;break;
          case 2:Servo.P-=1;break;
          case 3:Servo.P-=2;break;
          }
        }break;
      case 6:
        {
          switch(track_form)
          {
        //case 1:Servo.P+=5;Servo.D+=1;break;//-6-6
          case 1:Servo.P+=0;Servo.D+=2;break;
          case 2:Servo.P-=1;Servo.D+=1;break;
          case 3:Servo.P-=2;Servo.D+=0;break;
          }
        }break;
      case 5:
        {
          switch(track_form)
          {
            //case 1:Servo.P+=5;Servo.D+=1;break;
          case 1:Servo.P+=0;Servo.D+=1;break;
          case 2:Servo.P-=1;Servo.D+=0;break;
          case 3:Servo.P-=2;Servo.D-=1;break;
          }
        }break;
      case 4:
        {
          switch(track_form)
          {
            case 1:Servo.P+=0;Servo.D+=0;break;
            case 2:Servo.P-=1;Servo.D-=1;break;
            case 3:Servo.P-=2;Servo.D-=2;break;
          }
        }break;
      case 3:
        {
          switch(track_form)
          {
          case 1:Servo.P+=0;Servo.D-=1;break;
          case 2:Servo.P-=1;Servo.D-=2;break;
          case 3:Servo.P-=2;Servo.D-=3;break;
          }
        }break;
      case 2:
        {
          switch(track_form)
          {
          case 1:Servo.P+=0;Servo.D-=2;break;
          case 2:Servo.P-=1;Servo.D-=3;break;
          case 3:Servo.P-=2;Servo.D-=4;break;
          }
        }break;
      case 1:
        {
          switch(track_form)
          {
          case 1:Servo.P+=0;Servo.D-=3;break;
          case 2:Servo.P-=1;Servo.D-=4;break;
          case 3:Servo.P-=2;Servo.D-=5;break;
          }
        }break;
      }
    }break;
case -1:
    {
      switch(position_now_D)
      {
       case -6:
        {
          switch(track_form)
          {
        //case 1:Servo.P+=5;Servo.D+=1;break;//-6-6
          case 1:Servo.P-=1;Servo.D+=2;break;
          case 2:Servo.P-=2;Servo.D+=1;break;
          case 3:Servo.P-=3;Servo.D+=0;break;
          }
        }break;
      case -5:
        {
          switch(track_form)
          {
            //case 1:Servo.P+=5;Servo.D+=1;break;
          case 1:Servo.P-=1;Servo.D+=1;break;
          case 2:Servo.P-=2;Servo.D+=0;break;
          case 3:Servo.P-=3;Servo.D-=1;break;
          }
        }break;
      case -4:
        {
          switch(track_form)
          {
            case 1:Servo.P-=1;Servo.D+=0;break;
            case 2:Servo.P-=2;Servo.D-=1;break;
            case 3:Servo.P-=3;Servo.D-=2;break;
          }
        }break;
      case -3:
        {
          switch(track_form)
          {
          case 1:Servo.P-=1;Servo.D-=1;break;
          case 2:Servo.P-=2;Servo.D-=2;break;
          case 3:Servo.P-=3;Servo.D-=3;break;
          }
        }break;
      case -2:
        {
          switch(track_form)
          {
          case 1:Servo.P-=1;Servo.D-=2;break;
          case 2:Servo.P-=2;Servo.D-=3;break;
          case 3:Servo.P-=3;Servo.D-=4;break;
          }
        }break;
      case -1:
        {
          switch(track_form)
          {
          case 1:Servo.P-=1;Servo.D-=3;break;
          case 2:Servo.P-=2;Servo.D-=4;break;
          case 3:Servo.P-=3;Servo.D-=5;break;
          }
        }break;
      case 0:
         {
          switch(track_form)
          {
          case 1:Servo.P-=1;break;
          case 2:Servo.P-=2;break;
          case 3:Servo.P-=3;break;
          }
        }break;
      case 6:
        {
          switch(track_form)
          {
        //case 1:Servo.P+=5;Servo.D+=1;break;//-6-6
          case 1:Servo.P-=1;Servo.D+=2;break;
          case 2:Servo.P-=2;Servo.D+=1;break;
          case 3:Servo.P-=3;Servo.D+=0;break;
          }
        }break;
      case 5:
        {
          switch(track_form)
          {
            //case 1:Servo.P+=5;Servo.D+=1;break;
          case 1:Servo.P-=1;Servo.D+=1;break;
          case 2:Servo.P-=2;Servo.D+=0;break;
          case 3:Servo.P-=3;Servo.D-=1;break;
          }
        }break;
      case 4:
        {
          switch(track_form)
          {
            case 1:Servo.P-=1;Servo.D+=0;break;
            case 2:Servo.P-=2;Servo.D-=1;break;
            case 3:Servo.P-=3;Servo.D-=2;break;
          }
        }break;
      case 3:
        {
          switch(track_form)
          {
          case 1:Servo.P-=1;Servo.D-=1;break;
          case 2:Servo.P-=2;Servo.D-=2;break;
          case 3:Servo.P-=3;Servo.D-=3;break;
          }
        }break;
      case 2:
        {
          switch(track_form)
          {
          case 1:Servo.P-=1;Servo.D-=2;break;
          case 2:Servo.P-=2;Servo.D-=3;break;
          case 3:Servo.P-=3;Servo.D-=4;break;
          }
        }break;
      case 1:
        {
          switch(track_form)
          {
          case 1:Servo.P-=1;Servo.D-=3;break;
          case 2:Servo.P-=2;Servo.D-=4;break;
          case 3:Servo.P-=3;Servo.D-=5;break;
          }
        }break;
      }
    }break;
  case 0:
    {
      switch(position_now_D)
      {
      case -6:
        {
          switch(track_form)
          {
          case 1:Servo.P+=0;Servo.D+=2;break;//p=0,d=6
          case 2:Servo.P+=0;Servo.D+=1;break;
          case 3:Servo.P+=0;Servo.D+=0;break;
          }
        }break;
      case -5:
        {
          switch(track_form)
          {
          case 1:Servo.P+=0;Servo.D+=1;break;//0,5
          case 2:Servo.P+=0;Servo.D+=0;break;
          case 3:Servo.P+=0;Servo.D-=1;break; 
          }
        }break;
      case -4:
        {
          switch(track_form)
          {
          case 1:Servo.P+=0;Servo.D+=0;break;//4
          case 2:Servo.P+=0;Servo.D-=1;break;
          case 3:Servo.P+=0;Servo.D-=2;break;
          }
        }break;
      case -3:
        {
          switch(track_form)
          {
          case 1:Servo.P+=0;Servo.D-=1;break;//3
          case 2:Servo.P+=0;Servo.D-=2;break;
          case 3:Servo.P+=0;Servo.D-=3;break;
          }
        }break;
      case -2:
        {
          switch(track_form)
          {
          case 1:Servo.P+=0;Servo.D-=2;break;//2
          case 2:Servo.P+=0;Servo.D-=3;break;
          case 3:Servo.P+=0;Servo.D-=4;break;
          }
        }break;
      case -1:
        {
          switch(track_form)
          {
          case 1:Servo.P+=0;Servo.D-=3;break;//1
          case 2:Servo.P+=0;Servo.D-=4;break;
          case 3:Servo.P+=0;Servo.D-=5;break;
          }
        }break;
      case 0:
         {
          switch(track_form)
          {
          case 1:Servo.P+=0;break;
          case 2:Servo.P+=0;break;
          case 3:Servo.P+=0;break;
          }
        }break;
      case 6:
        {
          switch(track_form)
          {
          case 1:Servo.P+=0;Servo.D+=2;break;//p=0,d=6
          case 2:Servo.P+=0;Servo.D+=1;break;
          case 3:Servo.P+=0;Servo.D+=0;break;
          }
        }break;
      case 5:
        {
          switch(track_form)
          {
          case 1:Servo.P+=0;Servo.D+=1;break;//0,5
          case 2:Servo.P+=0;Servo.D+=0;break;
          case 3:Servo.P+=0;Servo.D-=1;break; 
          }
        }break;
      case 4:
        {
          switch(track_form)
          {
          case 1:Servo.P+=0;Servo.D+=0;break;//4
          case 2:Servo.P+=0;Servo.D-=1;break;
          case 3:Servo.P+=0;Servo.D-=2;break;
          }
        }break;
      case 3:
        {
          switch(track_form)
          {
          case 1:Servo.P+=0;Servo.D-=1;break;//3
          case 2:Servo.P+=0;Servo.D-=2;break;
          case 3:Servo.P+=0;Servo.D-=3;break;
          }
        }break;
      case 2:
        {
          switch(track_form)
          {
          case 1:Servo.P+=0;Servo.D-=2;break;//2
          case 2:Servo.P+=0;Servo.D-=3;break;
          case 3:Servo.P+=0;Servo.D-=4;break;
          }
        }break;
      case 1:
        {
          switch(track_form)
          {
          case 1:Servo.P+=0;Servo.D-=3;break;//1
          case 2:Servo.P+=0;Servo.D-=4;break;
          case 3:Servo.P+=0;Servo.D-=5;break;
          }
        }break;
      }
    }break;
 case 6:
    {
      switch(position_now_D)
      {
     case -6:
        {
          switch(track_form)
          {
        //case 1:Servo.P+=5;Servo.D+=1;break;//-6-6
          case 1:Servo.P+=4;Servo.D+=2;break;
          case 2:Servo.P+=3;Servo.D+=1;break;
          case 3:Servo.P+=2;Servo.D+=0;break;
          }
        }break;
      case -5:
        {
          switch(track_form)
          {
            //case 1:Servo.P+=5;Servo.D+=1;break;
          case 1:Servo.P+=4;Servo.D+=1;break;
          case 2:Servo.P+=3;Servo.D+=0;break;
          case 3:Servo.P+=2;Servo.D-=1;break;
          }
        }break;
      case -4:
        {
          switch(track_form)
          {
            case 1:Servo.P+=4;Servo.D+=0;break;
            case 2:Servo.P+=3;Servo.D-=1;break;
            case 3:Servo.P+=2;Servo.D-=2;break;
          }
        }break;
      case -3:
        {
          switch(track_form)
          {
          case 1:Servo.P+=4;Servo.D-=1;break;
          case 2:Servo.P+=3;Servo.D-=2;break;
          case 3:Servo.P+=2;Servo.D-=3;break;
          }
        }break;
      case -2:
        {
          switch(track_form)
          {
          case 1:Servo.P+=4;Servo.D-=2;break;
          case 2:Servo.P+=3;Servo.D-=3;break;
          case 3:Servo.P+=2;Servo.D-=4;break;
          }
        }break;
      case -1:
        {
          switch(track_form)
          {
          case 1:Servo.P+=4;Servo.D-=3;break;
          case 2:Servo.P+=3;Servo.D-=4;break;
          case 3:Servo.P+=2;Servo.D-=5;break;
          }
        }break;
      case 0:
         {
          switch(track_form)
          {
          case 1:Servo.P+=4;break;
          case 2:Servo.P+=3;break;
          case 3:Servo.P+=2;break;
          }
        }break;
      case 6:
        {
          switch(track_form)
          {
        //case 1:Servo.P+=5;Servo.D+=1;break;//-6-6
          case 1:Servo.P+=4;Servo.D+=2;break;
          case 2:Servo.P+=3;Servo.D+=1;break;
          case 3:Servo.P+=2;Servo.D+=0;break;
          }
        }break;
      case 5:
        {
          switch(track_form)
          {
            //case 1:Servo.P+=5;Servo.D+=1;break;
          case 1:Servo.P+=4;Servo.D+=1;break;
          case 2:Servo.P+=3;Servo.D+=0;break;
          case 3:Servo.P+=2;Servo.D-=1;break;
          }
        }break;
      case 4:
        {
          switch(track_form)
          {
            case 1:Servo.P+=4;Servo.D+=0;break;
            case 2:Servo.P+=3;Servo.D-=1;break;
            case 3:Servo.P+=2;Servo.D-=2;break;
          }
        }break;
      case 3:
        {
          switch(track_form)
          {
          case 1:Servo.P+=4;Servo.D-=1;break;
          case 2:Servo.P+=3;Servo.D-=2;break;
          case 3:Servo.P+=2;Servo.D-=3;break;
          }
        }break;
      case 2:
        {
          switch(track_form)
          {
          case 1:Servo.P+=4;Servo.D-=2;break;
          case 2:Servo.P+=3;Servo.D-=3;break;
          case 3:Servo.P+=2;Servo.D-=4;break;
          }
        }break;
      case 1:
        {
          switch(track_form)
          {
          case 1:Servo.P+=4;Servo.D-=3;break;
          case 2:Servo.P+=3;Servo.D-=4;break;
          case 3:Servo.P+=2;Servo.D-=5;break;
          }
        }break;
      }
    }break;
  case 5:
    {
      switch(position_now_D)
      {
      case -6:
        {
          switch(track_form)
          {
        //case 1:Servo.P+=5;Servo.D+=1;break;//-6-6
          case 1:Servo.P+=3;Servo.D+=2;break;
          case 2:Servo.P+=2;Servo.D+=1;break;
          case 3:Servo.P+=1;Servo.D+=0;break;
          }
        }break;
      case -5:
        {
          switch(track_form)
          {
            //case 1:Servo.P+=5;Servo.D+=1;break;
          case 1:Servo.P+=3;Servo.D+=1;break;
          case 2:Servo.P+=2;Servo.D+=0;break;
          case 3:Servo.P+=1;Servo.D-=1;break;
          }
        }break;
      case -4:
        {
          switch(track_form)
          {
            case 1:Servo.P+=3;Servo.D+=0;break;
            case 2:Servo.P+=2;Servo.D-=1;break;
            case 3:Servo.P+=1;Servo.D-=2;break;
          }
        }break;
      case -3:
        {
          switch(track_form)
          {
          case 1:Servo.P+=3;Servo.D-=1;break;
          case 2:Servo.P+=2;Servo.D-=2;break;
          case 3:Servo.P+=1;Servo.D-=3;break;
          }
        }break;
      case -2:
        {
          switch(track_form)
          {
          case 1:Servo.P+=3;Servo.D-=2;break;
          case 2:Servo.P+=2;Servo.D-=3;break;
          case 3:Servo.P+=1;Servo.D-=4;break;
          }
        }break;
      case -1:
        {
          switch(track_form)
          {
          case 1:Servo.P+=3;Servo.D-=3;break;
          case 2:Servo.P+=2;Servo.D-=4;break;
          case 3:Servo.P+=1;Servo.D-=5;break;
          }
        }break;
      case 0:
         {
          switch(track_form)
          {
          case 1:Servo.P+=3;break;
          case 2:Servo.P+=2;break;
          case 3:Servo.P+=1;break;
          }
        }break;
      case 6:
        {
          switch(track_form)
          {
        //case 1:Servo.P+=5;Servo.D+=1;break;//-6-6
          case 1:Servo.P+=3;Servo.D+=2;break;
          case 2:Servo.P+=2;Servo.D+=1;break;
          case 3:Servo.P+=1;Servo.D+=0;break;
          }
        }break;
      case 5:
        {
          switch(track_form)
          {
            //case 1:Servo.P+=5;Servo.D+=1;break;
          case 1:Servo.P+=3;Servo.D+=1;break;
          case 2:Servo.P+=2;Servo.D+=0;break;
          case 3:Servo.P+=1;Servo.D-=1;break;
          }
        }break;
      case 4:
        {
          switch(track_form)
          {
            case 1:Servo.P+=3;Servo.D+=0;break;
            case 2:Servo.P+=2;Servo.D-=1;break;
            case 3:Servo.P+=1;Servo.D-=2;break;
          }
        }break;
      case 3:
        {
          switch(track_form)
          {
          case 1:Servo.P+=3;Servo.D-=1;break;
          case 2:Servo.P+=2;Servo.D-=2;break;
          case 3:Servo.P+=1;Servo.D-=3;break;
          }
        }break;
      case 2:
        {
          switch(track_form)
          {
          case 1:Servo.P+=3;Servo.D-=2;break;
          case 2:Servo.P+=2;Servo.D-=3;break;
          case 3:Servo.P+=1;Servo.D-=4;break;
          }
        }break;
      case 1:
        {
          switch(track_form)
          {
          case 1:Servo.P+=3;Servo.D-=3;break;
          case 2:Servo.P+=2;Servo.D-=4;break;
          case 3:Servo.P+=1;Servo.D-=5;break;
          }
        }break;
      }
    }break;
  case 4:
    {
      switch(position_now_D)
      {
         case -6:
        {
          switch(track_form)
          {
        //case 1:Servo.P+=5;Servo.D+=1;break;//-6-6
          case 1:Servo.P+=2;Servo.D+=2;break;
          case 2:Servo.P+=1;Servo.D+=1;break;
          case 3:Servo.P+=0;Servo.D+=0;break;
          }
        }break;
      case -5:
        {
          switch(track_form)
          {
            //case 1:Servo.P+=5;Servo.D+=1;break;
          case 1:Servo.P+=2;Servo.D+=1;break;
          case 2:Servo.P+=1;Servo.D+=0;break;
          case 3:Servo.P+=0;Servo.D-=1;break;
          }
        }break;
      case -4:
        {
          switch(track_form)
          {
            case 1:Servo.P+=2;Servo.D+=0;break;
            case 2:Servo.P+=1;Servo.D-=1;break;
            case 3:Servo.P+=0;Servo.D-=2;break;
          }
        }break;
      case -3:
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D-=1;break;
          case 2:Servo.P+=1;Servo.D-=2;break;
          case 3:Servo.P+=0;Servo.D-=3;break;
          }
        }break;
      case -2:
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D-=2;break;
          case 2:Servo.P+=1;Servo.D-=3;break;
          case 3:Servo.P+=0;Servo.D-=4;break;
          }
        }break;
      case -1:
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D-=3;break;
          case 2:Servo.P+=1;Servo.D-=4;break;
          case 3:Servo.P+=0;Servo.D-=5;break;
          }
        }break;
      case 0:
         {
          switch(track_form)
          {
          case 1:Servo.P+=2;break;
          case 2:Servo.P+=1;break;
          case 3:Servo.P+=0;break;
          }
        }break;
      case 6:
        {
          switch(track_form)
          {
        //case 1:Servo.P+=5;Servo.D+=1;break;//-6-6
          case 1:Servo.P+=2;Servo.D+=2;break;
          case 2:Servo.P+=1;Servo.D+=1;break;
          case 3:Servo.P+=0;Servo.D+=0;break;
          }
        }break;
      case 5:
        {
          switch(track_form)
          {
            //case 1:Servo.P+=5;Servo.D+=1;break;
          case 1:Servo.P+=2;Servo.D+=1;break;
          case 2:Servo.P+=1;Servo.D+=0;break;
          case 3:Servo.P+=0;Servo.D-=1;break;
          }
        }break;
      case 4:
        {
          switch(track_form)
          {
            case 1:Servo.P+=2;Servo.D+=0;break;
            case 2:Servo.P+=1;Servo.D-=1;break;
            case 3:Servo.P+=0;Servo.D-=2;break;
          }
        }break;
      case 3:
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D-=1;break;
          case 2:Servo.P+=1;Servo.D-=2;break;
          case 3:Servo.P+=0;Servo.D-=3;break;
          }
        }break;
      case 2:
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D-=2;break;
          case 2:Servo.P+=1;Servo.D-=3;break;
          case 3:Servo.P+=0;Servo.D-=4;break;
          }
        }break;
      case 1:
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D-=3;break;
          case 2:Servo.P+=1;Servo.D-=4;break;
          case 3:Servo.P+=0;Servo.D-=5;break;
          }
        }break;
      }
    }break;
 case 3:
    {
      switch(position_now_D)
      {
   case -6:
        {
          switch(track_form)
          {
        //case 1:Servo.P+=5;Servo.D+=1;break;//-6-6
          case 1:Servo.P+=1;Servo.D+=2;break;
          case 2:Servo.P+=0;Servo.D+=1;break;
          case 3:Servo.P-=1;Servo.D+=0;break;
          }
        }break;
      case -5:
        {
          switch(track_form)
          {
            //case 1:Servo.P+=5;Servo.D+=1;break;
          case 1:Servo.P+=1;Servo.D+=1;break;
          case 2:Servo.P+=0;Servo.D+=0;break;
          case 3:Servo.P-=1;Servo.D-=1;break;
          }
        }break;
      case -4:
        {
          switch(track_form)
          {
            case 1:Servo.P+=1;Servo.D+=0;break;
            case 2:Servo.P+=0;Servo.D-=1;break;
            case 3:Servo.P-=1;Servo.D-=2;break;
          }
        }break;
      case -3:
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D-=1;break;
          case 2:Servo.P+=0;Servo.D-=2;break;
          case 3:Servo.P-=1;Servo.D-=3;break;
          }
        }break;
      case -2:
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D-=2;break;
          case 2:Servo.P+=0;Servo.D-=3;break;
          case 3:Servo.P-=1;Servo.D-=4;break;
          }
        }break;
      case -1:
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D-=3;break;
          case 2:Servo.P+=0;Servo.D-=4;break;
          case 3:Servo.P-=1;Servo.D-=5;break;
          }
        }break;
      case 0:
         {
          switch(track_form)
          {
          case 1:Servo.P+=1;break;
          case 2:Servo.P+=0;break;
          case 3:Servo.P-=1;break;
          }
        }break;
      case 6:
        {
          switch(track_form)
          {
        //case 1:Servo.P+=5;Servo.D+=1;break;//-6-6
          case 1:Servo.P+=1;Servo.D+=2;break;
          case 2:Servo.P+=0;Servo.D+=1;break;
          case 3:Servo.P-=1;Servo.D+=0;break;
          }
        }break;
      case 5:
        {
          switch(track_form)
          {
            //case 1:Servo.P+=5;Servo.D+=1;break;
          case 1:Servo.P+=1;Servo.D+=1;break;
          case 2:Servo.P+=0;Servo.D+=0;break;
          case 3:Servo.P-=1;Servo.D-=1;break;
          }
        }break;
      case 4:
        {
          switch(track_form)
          {
            case 1:Servo.P+=1;Servo.D+=0;break;
            case 2:Servo.P+=0;Servo.D-=1;break;
            case 3:Servo.P-=1;Servo.D-=2;break;
          }
        }break;
      case 3:
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D-=1;break;
          case 2:Servo.P+=0;Servo.D-=2;break;
          case 3:Servo.P-=1;Servo.D-=3;break;
          }
        }break;
      case 2:
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D-=2;break;
          case 2:Servo.P+=0;Servo.D-=3;break;
          case 3:Servo.P-=1;Servo.D-=4;break;
          }
        }break;
      case 1:
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D-=3;break;
          case 2:Servo.P+=0;Servo.D-=4;break;
          case 3:Servo.P-=1;Servo.D-=5;break;
          }
        }break;
      }
    }break;
 case 2:
    {
      switch(position_now_D)
      {
     case -6:
        {
          switch(track_form)
          {
        //case 1:Servo.P+=5;Servo.D+=1;break;//-6-6
          case 1:Servo.P+=0;Servo.D+=2;break;
          case 2:Servo.P-=1;Servo.D+=1;break;
          case 3:Servo.P-=2;Servo.D+=0;break;
          }
        }break;
      case -5:
        {
          switch(track_form)
          {
            //case 1:Servo.P+=5;Servo.D+=1;break;
          case 1:Servo.P+=0;Servo.D+=1;break;
          case 2:Servo.P-=1;Servo.D+=0;break;
          case 3:Servo.P-=2;Servo.D-=1;break;
          }
        }break;
      case -4:
        {
          switch(track_form)
          {
            case 1:Servo.P+=0;Servo.D+=0;break;
            case 2:Servo.P-=1;Servo.D-=1;break;
            case 3:Servo.P-=2;Servo.D-=2;break;
          }
        }break;
      case -3:
        {
          switch(track_form)
          {
          case 1:Servo.P+=0;Servo.D-=1;break;
          case 2:Servo.P-=1;Servo.D-=2;break;
          case 3:Servo.P-=2;Servo.D-=3;break;
          }
        }break;
      case -2:
        {
          switch(track_form)
          {
          case 1:Servo.P+=0;Servo.D-=2;break;
          case 2:Servo.P-=1;Servo.D-=3;break;
          case 3:Servo.P-=2;Servo.D-=4;break;
          }
        }break;
      case -1:
        {
          switch(track_form)
          {
          case 1:Servo.P+=0;Servo.D-=3;break;
          case 2:Servo.P-=1;Servo.D-=4;break;
          case 3:Servo.P-=2;Servo.D-=5;break;
          }
        }break;      
      case 0:
         {
          switch(track_form)
          {
          case 1:Servo.P+=0;break;
          case 2:Servo.P-=1;break;
          case 3:Servo.P-=2;break;
          }
        }break;
      case 6:
        {
          switch(track_form)
          {
        //case 1:Servo.P+=5;Servo.D+=1;break;//-6-6
          case 1:Servo.P+=0;Servo.D+=2;break;
          case 2:Servo.P-=1;Servo.D+=1;break;
          case 3:Servo.P-=2;Servo.D+=0;break;
          }
        }break;
      case 5:
        {
          switch(track_form)
          {
            //case 1:Servo.P+=5;Servo.D+=1;break;
          case 1:Servo.P+=0;Servo.D+=1;break;
          case 2:Servo.P-=1;Servo.D+=0;break;
          case 3:Servo.P-=2;Servo.D-=1;break;
          }
        }break;
      case 4:
        {
          switch(track_form)
          {
            case 1:Servo.P+=0;Servo.D+=0;break;
            case 2:Servo.P-=1;Servo.D-=1;break;
            case 3:Servo.P-=2;Servo.D-=2;break;
          }
        }break;
      case 3:
        {
          switch(track_form)
          {
          case 1:Servo.P+=0;Servo.D-=1;break;
          case 2:Servo.P-=1;Servo.D-=2;break;
          case 3:Servo.P-=2;Servo.D-=3;break;
          }
        }break;
      case 2:
        {
          switch(track_form)
          {
          case 1:Servo.P+=0;Servo.D-=2;break;
          case 2:Servo.P-=1;Servo.D-=3;break;
          case 3:Servo.P-=2;Servo.D-=4;break;
          }
        }break;
      case 1:
        {
          switch(track_form)
          {
          case 1:Servo.P+=0;Servo.D-=3;break;
          case 2:Servo.P-=1;Servo.D-=4;break;
          case 3:Servo.P-=2;Servo.D-=5;break;
          }
        }break;
      }
    }break;
case 1:
    {
      switch(position_now_D)
      {
       case -6:
        {
          switch(track_form)
          {
        //case 1:Servo.P+=5;Servo.D+=1;break;//-6-6
          case 1:Servo.P-=1;Servo.D+=2;break;
          case 2:Servo.P-=2;Servo.D+=1;break;
          case 3:Servo.P-=3;Servo.D+=0;break;
          }
        }break;
      case -5:
        {
          switch(track_form)
          {
            //case 1:Servo.P+=5;Servo.D+=1;break;
          case 1:Servo.P-=1;Servo.D+=1;break;
          case 2:Servo.P-=2;Servo.D+=0;break;
          case 3:Servo.P-=3;Servo.D-=1;break;
          }
        }break;
      case -4:
        {
          switch(track_form)
          {
            case 1:Servo.P-=1;Servo.D+=0;break;
            case 2:Servo.P-=2;Servo.D-=1;break;
            case 3:Servo.P-=3;Servo.D-=2;break;
          }
        }break;
      case -3:
        {
          switch(track_form)
          {
          case 1:Servo.P-=1;Servo.D-=1;break;
          case 2:Servo.P-=2;Servo.D-=2;break;
          case 3:Servo.P-=3;Servo.D-=3;break;
          }
        }break;
      case -2:
        {
          switch(track_form)
          {
          case 1:Servo.P-=1;Servo.D-=2;break;
          case 2:Servo.P-=2;Servo.D-=3;break;
          case 3:Servo.P-=3;Servo.D-=4;break;
          }
        }break;
      case -1:
        {
          switch(track_form)
          {
          case 1:Servo.P-=1;Servo.D-=3;break;
          case 2:Servo.P-=2;Servo.D-=4;break;
          case 3:Servo.P-=3;Servo.D-=5;break;
          }
        }break;
      case 0:
         {
          switch(track_form)
          {
          case 1:Servo.P-=1;break;
          case 2:Servo.P-=2;break;
          case 3:Servo.P-=3;break;
          }
        }break;
      case 6:
        {
          switch(track_form)
          {
        //case 1:Servo.P+=5;Servo.D+=1;break;//-6-6
          case 1:Servo.P-=1;Servo.D+=2;break;
          case 2:Servo.P-=2;Servo.D+=1;break;
          case 3:Servo.P-=3;Servo.D+=0;break;
          }
        }break;
      case 5:
        {
          switch(track_form)
          {
            //case 1:Servo.P+=5;Servo.D+=1;break;
          case 1:Servo.P-=1;Servo.D+=1;break;
          case 2:Servo.P-=2;Servo.D+=0;break;
          case 3:Servo.P-=3;Servo.D-=1;break;
          }
        }break;
      case 4:
        {
          switch(track_form)
          {
            case 1:Servo.P-=1;Servo.D+=0;break;
            case 2:Servo.P-=2;Servo.D-=1;break;
            case 3:Servo.P-=3;Servo.D-=2;break;
          }
        }break;
      case 3:
        {
          switch(track_form)
          {
          case 1:Servo.P-=1;Servo.D-=1;break;
          case 2:Servo.P-=2;Servo.D-=2;break;
          case 3:Servo.P-=3;Servo.D-=3;break;
          }
        }break;
      case 2:
        {
          switch(track_form)
          {
          case 1:Servo.P-=1;Servo.D-=2;break;
          case 2:Servo.P-=2;Servo.D-=3;break;
          case 3:Servo.P-=3;Servo.D-=4;break;
          }
        }break;
      case 1:
        {
          switch(track_form)
          {
          case 1:Servo.P-=1;Servo.D-=3;break;
          case 2:Servo.P-=2;Servo.D-=4;break;
          case 3:Servo.P-=3;Servo.D-=5;break;
          }
        }break;
      }
    }break;
  }
  //if(sai_dao_lei_xing==12&&abs(position_now_low_P)<2&&abs(position_now_P)<3)Servo.D+=20;
  //if(track_form==3&&abs(position_now_low_P)<1&&abs(position_now)<2)Servo.D+=25;
  //if(track_form==3&&abs(position_now_low_P)==1&&abs(position_now)==2)Servo.D+=18;
 // Servo_value=Servo_Middle+Servo.P*(79-position_now)+Servo.D*position_diff;//PD控制舵机
 
  Servo_value=Servo_Middle+Servo.P*(position_mid-position_now)+Servo.D*position_diff;//PD控制舵机
  Servo_value=limit(Servo_value,Servo_Left,Servo_Right);
  //if(abs(Servo_value-Servo_value_old)>2000)Servo_value=Servo_value_old;
  //Servo_value=0.1*Servo_value_old+0.9*Servo_value;
  Servo_value_old=Servo_value;
  FTM1_C0V=Servo_value;  
  return ok;
}

int16 stages3(int16 value,int16 parameter,int16 windage_k)
{
 
    if(!value) return unclear;
    if(value<(parameter-windage_k)){
      NULL;
      return(-1);
    }
    else if(value>=(parameter+windage_k)){
      NULL;
      return(1); 
    }
         else return(0);
}




/*************判断赛道类型0**************/
uint8 judge_locus_0(void)
{
  int16 i=0,num=0,buff1=0;
  uint8 sai_dao_lei_xing_0=0;
 // for(i=54;i>21;i--) 
  for(i=54;i>25;i--)//前段   59~41  33
    if(position[i])
    {
      low_0+=position[i];
      num++;
    }  
  if(low_0)
    low_0=low_0/num;
  num=0;
    
 // for(i=21;i>7;i--) //中段  40~26    20  15 
  for(i=25;i>8;i--)
    if(position[i]){
      mid_0+=position[i];
      num++;

    }
  if(mid_0)
    mid_0=mid_0/num;
  num=0;    
  
    for(i=8;i>0;i--)//远段    25~0
    if(position[i])
    {
      high_0+=position[i];
      num++;
    }
   if(high_0)
     high_0=high_0/num;
   num=0;
   
   low_now_0=low_0;                      //远近中端的坐标位置
   mid_now_0=mid_0;
   high_now_0=high_0; 
  
   low_0=stages3(low_0,video_Middle,24);        // 22          low,mid,high就是1，0，-1
   mid_0=stages3(mid_0,video_Middle,22);        // 19
   high_0=stages3(high_0,video_Middle,18);     //16
   if(high_0==unclear)
   {    
     buff1=low_0*mid_0;
     if(mid_0==unclear)
     { //超急弯
       if(low_0==0)
       {              
         sai_dao_lei_xing_0=0;        
       }
       else
       {          
         sai_dao_lei_xing_0=1;         
       }       
     }
     else
     {                                                  
       if(buff1==1)
       {   //急弯        
         sai_dao_lei_xing_0=2;                  
       }
       else
       if(buff1==0)
       {  //急弯
         if(low_0!=0) 
         {           
           sai_dao_lei_xing_0=4;           
         }
         else
         {
           if(mid_0==0&&low_0==0)
           {
             sai_dao_lei_xing_0=5;                         
           }            
           sai_dao_lei_xing_0=6;           
         }           
       }
       else if(buff1==-1)
       { // 超急弯         
         sai_dao_lei_xing_0=8;                    
       }
     }                                         
   }
   else
   {           //high!=unclear时
     if(mid_0==high_0)
     {
       if(high_0==0)
       {        //直道         
         if(high_now_0>70&&high_now_0<90&&mid_now_0>70&&mid_now_0<90&&low_now_0>70&&low_now_0<90)
         {
           sai_dao_lei_xing_0=20;
           Very_straight=1; 
         }
         else sai_dao_lei_xing_0=9;            
       }
       else
       {           /////////////////////////         
         sai_dao_lei_xing_0=10;                
       }
     }
     else
     {     //mid!=high时
       if(low_0==0&&mid_0==0)
       {         //长大弯         
         sai_dao_lei_xing_0=12;               
       }//else
       if(mid_0==0&&(low_0*high_0==-1))
       {  //大弯         
         sai_dao_lei_xing_0=13;                  
       }//else
       if(high_0==low_0&&low_0!=mid_0)
       {    //S弯         
         sai_dao_lei_xing_0=14;           
       }
       /////////////////////////////////////////////////////
       if(high_0==0&&(mid_0*low_0==-1))
       {        
         sai_dao_lei_xing_0=15;               
       }
       if(high_0==0&&(mid_0*low_0==1))
       {         //长大弯        
         sai_dao_lei_xing_0=16;                
       }
       if((high==-1&&mid==1&&low!=-1)||(high==1&&mid==-1&&low!=1))
       {         //长急弯         
         sai_dao_lei_xing_0=17;         
       }
       /////////////////////////////////////////////////
     }
   }
   return sai_dao_lei_xing_0;  
}
/*************判断赛道类型0END**************/


int16 speed_control(void)
{
  volatile int16 i=0,E=0,Ec=0;
  volatile int32 exspeed=0;
  volatile struct membership_grade S; 
  static int16 i_old=0;
  
  
  while(!position[i]&&i<55) i++;   //50
  
      
  E=54-i;   //i无效点个数    //49    E为有效的个数
  Ec=i-i_old; //有效点变化率
  i_old=i;
  
  if(i<=5) return speed_top;    //直道全速冲刺  3
    
  if(i>5)
  {
    if(!Ec)//变化率为0的时候
    {
      //sub_e(E,25,20,50,45);//20，15，45，40通过采集到的点数来算出，speed_s,speed_m,speed_b所占的份额
  //  sub_e(E,24,0,54,24);       //主要决定十字弯道和直到的过弯速度
      sub_e(E,35,30,50,45);
      S.s=sub.s;
      S.m=sub.m;
      S.b=sub.b;
      exspeed=(sub.s*speed_s+sub.m*speed_m+sub.b*speed_b)/100;
    }
    else //变化率不等于0的时候
    {
      //exspeed=speed_fuzzy(E,Ec,35,25,50,45,3,1,7,5); 4.8—10：00
      exspeed=speed_fuzzy(E,Ec,35,30,50,45,3,1,7,5);
    }
    return exspeed;
  }
}
    


int16 Motor_control(int16 shift)
{
  volatile int32 speed=0;
  volatile int16 motor_e=0,motor_ec=0;
  static int16 motor_e_old=0,motor_sum=0;
   
  motor_e=shift-pulse; //speed_vari为编码器返回的脉冲数
  speed_now=shift;
  motor_ec=motor_e-motor_e_old;
  
  motor_fuzzy(motor_e,motor_ec,16,8,32,24,8,4,16,12);//30  20  50  40  15  10  25  20
  motor_sum=line(motor_e,Motor.I);
  speed=(long)s_old*motor_p+Motor.P*(long)motor_e+motor_p*(long)motor_sum/2;
  speed=speed/100;
  s_old=pulse;
  motor_e_old=motor_e;
  //if(pulse<50&&speed<0) return 50;
  return speed;            
}   


/************************主函数****************************/


void main(void)
{  
  uint8 m=0,line_head=0,line_end=0;
  uint8 P_stop=0;//停车关速度控制标志位
  uint8 i,direct=0,LCD_begin=0;
  int8 judge=0,set=0;       
  int16  exspeed=0,pwm_speed=0;
  int16 start_flag=0,stf=0,stop_flag2=1,stop_flag=0,stop_num=0;
  
  DisableInterrupts;
  pllinit180M();
  IO_Init();
  LCD_Init();  
  hw_FTM1_init();
  hw_FTM0_init();
  FTM2_QUAD_Init();
  lptmr_pulse_counter(LPTMR_ALT2);
  //gpio_init(PORTA,14,1,1);
  //gpio_init(PORTA,15,1,1);
  //gpio_init(PORTA,16,1,1);
  //gpio_init(PORTA,17,1,1);
  JIANPAN_ini();
  CH451_ini();
  CH451_WriteCommand(CH451_BCD);
  LCD_P6x8Str(0,0,"speed"); 
  LCD_P6x8Str(0,7,"J_line");//英文字符串显示
  LCD_P6x8Str_3(50,10,Judge_startline);
  Car_Speed();
  switch(car_speed)
  {
    case 0:speed_Very_straight=170;speed_top=165;speed_b=160;speed_m=155;speed_s=150; speed_ms=145;break;
    case 1:speed_Very_straight=180;speed_top=175;speed_b=170;speed_m=165;speed_s=160; speed_ms=155;break;
    case 2:speed_Very_straight=200;speed_top=190;speed_b=175;speed_m=170;speed_s=165; speed_ms=160;break;
    case 3:speed_Very_straight=210;speed_top=200;speed_b=190;speed_m=180;speed_s=175; speed_ms=175;break;
    case 4:speed_Very_straight=220;speed_top=210;speed_b=190;speed_m=180;speed_s=175; speed_ms=170;break;
    case 5:speed_Very_straight=230;speed_top=220;speed_b=190;speed_m=180;speed_s=180; speed_ms=175;break;
    case 6:speed_Very_straight=240;speed_top=230;speed_b=195;speed_m=180;speed_s=175; speed_ms=175;break; 
    case 7:speed_Very_straight=250;speed_top=240;speed_b=200;speed_m=185;speed_s=180; speed_ms=180;break;
    case 8:speed_Very_straight=260;speed_top=250;speed_b=210;speed_m=195;speed_s=190; speed_ms=190;break;
    case 9:speed_Very_straight=270;speed_top=260;speed_b=220;speed_m=200;speed_s=200; speed_ms=190;break;
     case 10:speed_Very_straight=290;speed_top=270;speed_b=230;speed_m=220;speed_s=210; speed_ms=190;break;
      case 11:speed_Very_straight=290;speed_top=270;speed_b=240;speed_m=230;speed_s=220; speed_ms=190;break;
  }
  while(KeyValue!=50)
  {
    KeyValue=50;
    CH451_GetKeyValue();
  }
  //1直接发车，不调节PD，，，2调节PD后发车/
  while(!direct)
  {
    LCD_P6x8Str(0,1,"TJorFC");     //英文字符串显示      
    CH451_GetKeyValue();
    switch(KeyValue)
    {
    case 1:
      LCD_P6x8Str(40,1,"FC");     //英文字符串显示      
      while(KeyValue==1)
      {
        KeyValue=50;
        CH451_GetKeyValue();
      }
      direct=1;
      break;
    case 2:
      LCD_P6x8Str(40,1,"TJ");     //英文字符串显示      
      while(KeyValue==2)
      {
        KeyValue=50;
        CH451_GetKeyValue();
      }
      LCD_P6x8Str(0,2,"sai_dao_number");     //英文字符串显示  
      Duo_Ji_PD();
      LCD_P6x8Str(58,1,"FC");     //英文字符串显示  
      direct=1;
      break;
    case 5:
      LCD_P6x8Str(76,0,"LCDgo_on");     //英文字符串显示      
      while(KeyValue==5)
      {
        KeyValue=50;
        CH451_GetKeyValue();
      }
      LCD_begin=1;
      break;       
    }
  }
  /****************1直接发车，不调节PD，，，2调节PD后发车end*******************/           
  enable_irq (87);//使能A口中断 ，A24场中断
  UART0_Init();
  //uart_init(UART5,45,115200);
  DMA0_Init();
  //hw_pit_init(1,16000);         //pit1中断初始化
  //enable_irq(89);             //开c9口红外遥控停车中断
  //enable_pit_interrupt(1);    //使能PIT1中断，，，用于红外计数
  delays(3);   //延时发车
  lptmr_pulse_counter(LPTMR_ALT2);
  LCD_CLS();
  hw_pit_init(PIT2,4500000);//100ms
  enable_pit_interrupt(PIT2);
  EnableInterrupts;
  for(;;) 
  {	  
    if(finish==1)
    {   
      tu_flag=find_position();
      if(stop_flag==1)
      {
        stop_num++;
      }
      m=judge_locus_0();
      sai_dao_lei_xing=m;
       if(P_timer>Judge_startline&&dijihang<5&&high_now_0>60&&high_now_0<100&&mid_now_0>60&&mid_now_0<100&&low_now_0>60&&low_now_0<100)
    {
     if(stop_flag2)
     {
    stop_flag=Judge_startline2();
    
     }
     if(stop_flag==1)
     {
       stop_flag2=0;
     }
    } 
if(stop_flag==1)
{
  if(stop_num>12)
  {
    if(pulse>150)
  {
      FTM0_C0V=0;
      FTM0_C3V=350;
      FTM0_C4V=350;
      FTM0_C6V=0;
  }
  if(pulse>100)
  {
      FTM0_C0V=0;
      FTM0_C3V=250;
      FTM0_C4V=250;
      FTM0_C6V=0;
  }
  else if(pulse>50)
  {
      FTM0_C0V=0;
      FTM0_C3V=200;
      FTM0_C4V=200;
      FTM0_C6V=0;
  }
  else if(pulse>30)
  {
      FTM0_C0V=0;
      FTM0_C3V=10;
      FTM0_C1V=0;
      FTM0_C5V=0;
      FTM0_C4V=10;
      FTM0_C6V=0;
  }
else
{     
     FTM0_C0V=0;
      FTM0_C3V=0;
      FTM0_C4V=0;
      FTM0_C6V=0;
      FTM0_C1V=0;
      FTM0_C5V=0;

}
  }
  else
  {
  FTM0_C0V=375;
      FTM0_C3V=0;
      FTM0_C4V=0;
      FTM0_C6V=375;
  }
      P_stop=1;
      stf=1;

}
        
          
      
      if(tu_flag==1)  
      {
        set=choose_PD();
        tu_flag=0;
      }
      else
      {
        if(Servo_value>(Servo_Middle+200))
        {
        Servo_value=limit(Servo_value+250,Servo_Left,Servo_Right);
        FTM1_C0V=Servo_value;
         // Servo_value=9320;
        }
        if(Servo_value<(Servo_Middle-200))
        {
        Servo_value=limit(Servo_value-200,Servo_Left,Servo_Right);
        FTM1_C0V=Servo_value;
        //Servo_value=7035;
        }
      }
      if(P_stop==0)
      {
        if(P_timer<10) 
          pwm_speed=375;//发车前1.3s给满占空比,使其快速发车
        else
        {
          if(Very_straight==0)           
            exspeed=speed_control();
          if(Very_straight==1)
          {
            Very_straight=0;
            exspeed=speed_Very_straight;
          }
          pwm_speed=Motor_control(exspeed);
        }
        set_speed(pwm_speed);
      }
      
    hw_pit_init(PIT0,745000);//16ms  使用定时器，这样才是在固定的时间里面得到的脉冲数
    enable_pit_interrupt(PIT0);
    /********************LCD数据显示模块*********************/
    CH451_GetKeyValue();
    if(KeyValue==10||LCD_begin==1)           //LCD_begin==1
    {
      if(LCD_P1==250)LCD_P1=0;
                    //秒加1
                  
      if(++LCD_P1%10)
      {
        LCD_CLS();                     
        LCD_P6x8Str(1,0,"low_0");     //英文字符串显示
        LCD_P6x8Str_3(45,0,low_now_0);
        if(low_now)LCD_P6x8Str_1(70,0,low_0);    //如果存在的话就显示，不存在的话就不显示
        //LCD_P6x8Str(10,0,"Servo.P");
        LCD_P6x8Str_3(70,0,Servo.P);//显示赛道类型.
         LCD_P6x8Str_3(90,0,Servo.D);
        LCD_P6x8Str(1,1,"mid_0");     //英文字符串显示
        LCD_P6x8Str_3(45,1,mid_now_0);
        if(mid_now)LCD_P6x8Str_1(70,1,mid_0);
       LCD_P6x8Str_3(70,1,pulse);
       
        LCD_P6x8Str_3(90,1,pulse2);
        LCD_P6x8Str(1,2,"high_0");     //英文字符串显示
        LCD_P6x8Str_3(45,2,high_now_0);
        if(high_now)LCD_P6x8Str_1(70,2,high_0);
        LCD_P6x8Str(1,3,"position_now");     //英文字符串显示
        LCD_P6x8Str_3(75,3,position_now);
        LCD_P6x8Str(1,4,"position_diff");     //英文字符串显示
        LCD_P6x8Str_FuHao_3(88,4,position_diff);
        LCD_P6x8Str(1,5,"position_now_low");     //英文字符串显示
        LCD_P6x8Str_FuHao_3(100,5,position_now_low);
        LCD_P6x8Str(1,6,"Servo_value"); 

        LCD_P6x8Str_5(88,6,Servo_value);
        LCD_P6x8Str(1,7,"dijihang"); 
        LCD_P6x8Str_5(88,7,dijihang);
      }
    }
    if(KeyValue==6)
    {
      if(LCD_P3==250)LCD_P3=0;
      if(++LCD_P3%10)
      {
        LCD_CLS();       
        for(i=0;i<IMG_ROWS-1;i++)
        {
           LCD_PutPixel(position[i],i);
        }
       }
     }             
      /***************对一些变量进行清零****************/
      position_now_low=0;
      position_now=0;
      position_diff=0;
      dijihang=0;
      for(i=0;i<IMG_ROWS;i++)
      {  
        position[i]=0;
      }      
      ramp_flag=0;
      finish=0;
      /***************对一些变量进行清零END****************/
      /********************LCD数据显示模块 end*********************/
    }
  }	 
}
//************************主函数END***********************