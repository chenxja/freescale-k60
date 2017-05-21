

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
#define  H 55
#define  V 160
#define  video_Middle  80
#define windage 40
//#define black_max 150   //150
#define shi_zi_jiao_cha_number 9      //用来判断十字交叉的行数的，大于这个值就认为是十字交叉
//int16 Servo_Middle= 12385;    //中间12305 11625  250Hz  37.27%
//int16 Servo_Left=   10475;   //最左10475  250Hz  44%
//int16 Servo_Right=  14375;   //最右15575   250Hz  30.53%
int16 Servo_Middle= 11385;//中间12305;    //11385中间12305 11625  250Hz  37.27%
int16 Servo_Left=   9205;   //最左10475  250Hz  44%
int16 Servo_Right=  13675;   //最右15575   250Hz  30.53%

#define  unclear  -100
#define  ok        1
#define  fail      0
#define  position_mid   79           //中心位置
/*****************************阈值设定*******************************/
unsigned char black_max_a1 = 115;
unsigned char black_max_a2 = 130;
unsigned char black_max_a3 = 115;
unsigned char black_max_b1 = 118;
unsigned char black_max_b2 = 135;
unsigned char black_max_b3 = 118;
unsigned char black_max_c1 = 120;
unsigned char black_max_c2 = 140;
unsigned char black_max_c3 = 120;
unsigned char black_max_d1 = 130;
unsigned char black_max_d2 = 140;
unsigned char black_max_d3 = 130;

unsigned char black_max_1 = 0;
unsigned char black_max_2 = 0;
unsigned char black_max_3 = 0;
/*****************************END********************************/
volatile int P_timer=0;

unsigned int row=0;//摄像头行计数，最大240
volatile uint8 video[H][V];//存放数据数组
volatile uint8 bianyan[2][H];//存放两边黑线
volatile uint8 bianyan2[2][H];
volatile uint8 center[H];//存放中心线
volatile uint8 BW[101];//这个需要清零，用于起始线检测25-35
volatile uint8 BW2[121];//这个需要清零，用于起始线检测35-45
volatile uint8 BW3[81];//18-25起始线检测
volatile uint8 BW4[141];//45-55起始线检测
volatile uint8 start_xian=0;//这个需要清零
volatile uint8 Very_straight=0;//这个需要清零
volatile uint8 ramp_flag=0;//这个需要清零

volatile uint8 END_Line_2=0;

int16 position_Save[20];
uint8 video2[V];
int16 position_ceshi=0,position_old_ceshi=0;
uint16 left_ceshi=0,right_ceshi=0;
uint16 see_distance=0;
 int16 position_now_P=0,position_now_low_P=0,position_now_D=0;
 uint16 slope_flag=0;
volatile uint8 m_zhi=0;
uint8 zhidao_flag=0;
 uint16 speed_cut_timer=100,start_startline=20;
 int16 i_old=0;
 int16 xiaoS_flag=0;
 uint8 m=0;

unsigned int imagerow=0;//采集行计数，最大H
unsigned int const data_table[H]={ 1,   3,   5,   7,   9,    //2 
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
                                       
                                       
                                       
                                       
                                       
   
                                   };//需采集数据的行，2cm一行实际测得
unsigned char mo_hu[31]={70,71,72,74,75,
                         77,81,83,85,87,
                         90,93,95,95,95,
                         95,95,95,95,95,
                         95,95,95,95,95,
                         95,95,95,95,95,95};

uint8 row_F[H];//该行采集完成标志
char startline;//起始行
char endline;//结束行
char startline_F;//发现起始行
char endline_F;//发现结束行


uint8 T_P4=0,T_P1=0,T_P2=0,T_P28=0;//用来控制小灯变化的P4闪烁为脉冲计数运行标志，P1为边沿函数的执行标志
uint8 LCD_P1=0,LCD_P2=0,LCD_P3=0;
volatile uint16 pulse=0,pulse_old=0,pulse_old_old=0;
volatile uint8 finish=0;


volatile uint8 xu_xian2=0;//这个需要清零
volatile uint8 xu1=0,xu2=0,xu3=0,xu4=0;//这个需要清零


volatile uint8 sai_dao_lei_xing=0;

volatile int16 low_0=0,mid_0=0,high_0=0;

volatile int16 low=0,mid=0,high=0;
volatile int16 topline=0;
volatile int16 low_old=0,mid_old=0,high_old=0;
volatile int16 position_now=0,position_diff=0,position_now_low=0;
volatile int16 Servo_value=0,Servo_value_old=12305;//这里Servo_value_old的初始值要设置的和Servo_Middle一样，是用来限定的
volatile int16 low_now=0,mid_now=0,high_now=0;

volatile int16 low_now_0=0,mid_now_0=0,high_now_0=0;
volatile int16 END_0=0;

volatile uint8 car_speed=4;
volatile int16 speed_Very_straight,speed_top,speed_b,speed_m,speed_s,speed_ms;

volatile int16 time_m=0,speed_now=0;
volatile int16 s_old=0;

volatile int16 END_Line=0;
volatile int16 xie_shizi_flag_1=0,xie_shizi_flag_2=0,xie_shizi_flag_3=0;

volatile int16 Judage_right_hang=0,Judage_right_lie=0;
volatile int16 Judage_left_hang=0,Judage_left_lie=0;
int16 position_old=76,position_old_old=76,position_old_old_old=76;
int16 weizhi=0,weizhi_old=0,weizhi_old_old=0;


uint8 start_line2=0;//注意这个需要清零
uint8 start_18_25_2=0,start_25_35_2=0,start_35_45_2=0,start_45_55_2=0;
////////////模糊控制///////////////

#define MP_S      510//110  260  360
#define MP_M      540//180  320  400
#define MP_B      580//240  360  440

#define MI_S      6//6 4
#define MI_M      9//10 8 
#define MI_B      12//15  10

#define  motor_p      70//   45 50 55 60 65




//////////////////////舵机参数////////////////////////////
uint8 Servo_P_zhidao=25;
uint8 Servo_D_zhidao=110;
uint8 P_High=27;//24 6 27  29
uint8 D_High=8;//10 8 18  15
uint8  D_piancha_high=10;
uint8 H_High=15;//20
uint8 T_High=30;//25

uint8 P_Mid=40;//41 42
uint8 D_Mid=22;//25 26
uint8 D_piancha_mid=20;
uint8 H_Mid=15;
uint8 T_Mid=45;


uint8 P_Low=43;//43
uint8 D_Low=30;//32 30
uint8 D_piancha_low=26;
uint8 H_Low=24;
uint8 T_Low=45;

volatile int16 E=0,Ec=0,Ec_old=0,Ec_old_old=0;
uint8 servo_d=0;
///////////////////////////////////////////////////////////
/******************在这里加入键盘控制舵机PD参数头文件*******************/
#include "keyboard_PD.h"
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
    SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(2) 
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
      FTM1_SC = 0xB; //not enable the interrupt mask向上计数模式
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
      FTM1_MOD=30000;       //Modulo value,The EPWM period is determined by (MOD - CNTIN + 0x0001) 
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
      
      SIM_SCGC6|=SIM_SCGC6_FTM0_MASK;         //使能FTM0时钟      
      
      //C0口 
      FTM0_C0SC |= FTM_CnSC_ELSB_MASK;
      FTM0_C0SC &= ~FTM_CnSC_ELSA_MASK;
      FTM0_C0SC |= FTM_CnSC_MSB_MASK;    
      //C3口
      FTM0_C3SC |= FTM_CnSC_ELSB_MASK;
      FTM0_C3SC &= ~FTM_CnSC_ELSA_MASK;
      FTM0_C3SC |= FTM_CnSC_MSB_MASK;
      //C1口 电机控制长期高电压
      FTM0_C1SC |= FTM_CnSC_ELSB_MASK;
      FTM0_C1SC &= ~FTM_CnSC_ELSA_MASK;
      FTM0_C1SC |= FTM_CnSC_MSB_MASK;    
      
      
      FTM0_SC = 0xC; //not enable the interrupt mask向上计数模式
      FTM0_MODE |= FTM_MODE_WPDIS_MASK;      
       //BIT1   Initialize the Channels Output
      //FTMEN is bit 0, need to set to zero so DECAPEN can be set to 0
      FTM0_MODE &= ~1;
       //BIT0   FTM Enable
       //0 Only the TPM-compatible registers (first set of registers) can be used without any restriction. Do not use the FTM-specific registers.
       //1 All registers including the FTM-specific registers (second set of registers) are available for use with no restrictions.
      
      FTM0_OUTMASK=0xf4;   //0 Channel output is not masked. It continues to operate normally.
                           //1 Channel output is masked. It is forced to its inactive state.
      
      FTM0_COMBINE=0;      //Function for Linked Channels (FTMx_COMBINE)
      FTM0_OUTINIT=0;
      FTM0_EXTTRIG=0;      //FTM External Trigger (FTMx_EXTTRIG)
      FTM0_POL=0;          //Channels Polarity (FTMx_POL)
                           //0 The channel polarity is active high.
                           //1 The channel polarity is active low.     
      //Set Edge Aligned PWM
      FTM0_QDCTRL &=~FTM_QDCTRL_QUADEN_MASK;
      //QUADEN is Bit 1, Set Quadrature Decoder Mode (QUADEN) Enable to 0,   (disabled)
      //FTM0_SC = 0x16; //Center Aligned PWM Select = 0, sets FTM Counter to operate in up counting mode,
      //it is field 5 of FTMx_SC (status control) - also setting the pre-scale bits here
      
      FTM0_INVCTRL=0;     //反转控制
      FTM0_SWOCTRL=0;     //软件输出控制F TM Software Output Control (FTMx_SWOCTRL)
      FTM0_PWMLOAD=0;     //FTM PWM Load
                          //BIT9: 0 Loading updated values is disabled.
                          //1 Loading updated values is enabled.
      FTM0_CNTIN=0;        //Counter Initial Value      
      FTM0_MOD=375;       //375为10K 
                           //采用龙丘时钟初始化函数，可以得到4分频的频率，系统60M频率时，PWM频率是15M,以此类推
                           //PMW频率=X系统频率/4/(2^FTM1_SC_PS)/FTM1_MOD
      FTM0_C0V=0;        //反转
      FTM0_C1V=375;        //持续高电平
      FTM0_C3V=0;           //正转
      
      FTM0_CNT=0;          //只有低16位可用
}
/*********************电机正反转  C1口C3口输出PWM波END*************/

/****************FTM正交分解，用于测脉冲数的   A10口*************/
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


/********************中心线发送**********************/
void SCI_center(void)
{
  uint8 a=0;
  //uart_send1(UART0,0xff);
  //uart_send1(UART0,0xff);
  for(a=0;a<H;a++)
  {
    uart_send1(UART0,center[a]);
  }
}
/********************中心线发送**********************/
/******************与上位机相连，用于看图像的*****************/
void SCI(void)
{
  volatile uint8 *uc_FramePoint;
  uint8 a=0;
  uint16 b=0;   
  //uart_send1(UART0,0x00);
  //uart_send1(UART0,0xFF);
  uart_send1(UART0,0x01);
  //uart_send1(UART0,0x00);
  for(a=0;a<H;a++)
  {
    for(b=0;b<V;b++)
    {
      uc_FramePoint=video[a]+b;
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
	PORTD_PCR6|=PORT_PCR_MUX(3);//将D6引脚设置为模式3，即UART0_RX
	PORTD_PCR7|=PORT_PCR_MUX(3);//将D7引脚设置为模式3，即UART0_TX
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
	
	DMA_TCD0_CITER_ELINKNO=DMA_CITER_ELINKNO_CITER(V);//当前主循环次数,采集点数
	DMA_TCD0_BITER_ELINKNO=DMA_BITER_ELINKNO_BITER(V);//起始主循环次数，采集点数
	DMA_TCD0_SADDR=(uint32)&GPIOB_PDIR;//设置源地址GPIO口，PORTB
	DMA_TCD0_SOFF=0;//每次传送源地址不变
	//DMA_TCD1_NBYTES_MLOFFYES=DMA_NBYTES_MLOFFYES_NBYTES(1)+DMA_NBYTES_MLOFFNO_SMLOE_MASK+DMA_NBYTES_MLOFFYES_MLOFF(-4);//传送4字节
	DMA_TCD0_NBYTES_MLNO=DMA_NBYTES_MLNO_NBYTES(1);//每次读取一字节
	DMA_TCD0_SLAST=0;//主循环结束后源地址0回写tcd
	DMA_TCD0_DLASTSGA=0;//主循环结束后目的地址0回写tcd
	DMA_TCD0_DADDR=(uint32)video;//设置目的地址，video数组第一个元素
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
    //
    DMA_ERQ=0x00;
    disable_irq(0);
    disable_irq(88);
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
  pulse_old_old=pulse_old;
  pulse_old=pulse;
  pulse=FTM2_CNT;
  FTM2_CNT=0;
  pulse=(pulse_old_old+pulse_old+pulse)/3;
  //if(pulse==0)FTM0_C1V=0;     //如果电机没转那么电机就关掉
  
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


/************************取绝对值************************/
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

int16 max(int16 value1,int16 value2){
  if(value1>value2)
    return value1;
  else 
    return value2;
}

int16 min(int16 value1,int16 value2){
  if(value1<value2)
    return value1;
  else 
    return value2;
}

void set_speed(int16 speed_low)
{
  //int16 i=0; 
  //uart_send1(UART0,0x23);
  //speed_low=65;
  FTM0_C0V=limit(188+speed_low/2,375,0);
  FTM0_C3V=limit(188-speed_low/2,375,0);
}
/************************取绝对值END************************/

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

/*************************结构体区***************************/

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

struct range{
  int16 head;
  int16 tail;
}servo;

struct membership_grade{
  int16 s;
  int16 m;
  int16 b;
}sub;
/*************************结构体区END***************************/

/************************读取速度************************/
void get_speed(void)
{
  pulse=FTM2_CNT;
  FTM2_CNT=0;
  //if(pulse==0)FTM0_C1V=0;     //如果电机没转那么电机就关掉
  
}
/************************读取速度END************************/


/***************************找出中心线的最大值的坐标*************************/
void find_center_max(void)
{
	uint8 Judage_flag_1=V-1;
	uint8 i=0;
    for(Judage_flag_1=159;Judage_flag_1>10;Judage_flag_1--)
	{
		for(i=53;i>0;i--)
		{
			if(center[i]==Judage_flag_1)
			{
				if(center[i-1]>Judage_flag_1-10||center[i+1]>Judage_flag_1-10)
				{
					Judage_right_hang=i;
					Judage_right_lie=Judage_flag_1;
					//return ok;
				   
				}
			}
		}
	}
	//return fail;
}
	/***************************找出中心线的最大值的坐标END*************************/
    /*************************找出中心线最小值的坐标*************************/
void find_center_min(void)
{
	uint8 Judage_flag_0=1;
	uint8 i=0;
    for(Judage_flag_0=1;Judage_flag_0<V-11;Judage_flag_0++)
	{
		for(i=53;i>0;i--)
		{
			if(center[i]==Judage_flag_0)
			{
				if(center[i-1]<Judage_flag_0+10||center[i+1]<Judage_flag_0+10)
				{
					Judage_left_hang=i;
					Judage_left_lie=Judage_flag_0;
					//return ok;
				   
				}
			}
		}
	}
	//return fail;
}



    /*************************找出中心线最小值的坐标END*************************/



/******************两断点之间的补线连接算法******************/
void LianJie(char i,int16 m,int16 n)
{
  if(m-n>1)
  {
    bianyan[i][(m+n)/2]=(bianyan[i][m]+bianyan[i][n])/2;
    LianJie(i,(m+n)/2,n);
    LianJie(i,m,(m+n)/2);
  }
}
/******************两断点之间的补线连接算法end******************/
/****************起始线的检测函数****************/
void Start_Detection(unsigned char a,unsigned char h1,unsigned char h2,unsigned char r1,unsigned char r2)
{
	unsigned char i=0,m=0,n=0;
	for(i=h1;i>h2;i--)
	{
		if(video[i][r1]==0)m++;
		if(video[i][r1]==1)n++;
	}
	if(m>0)BW[a]=0;
	if(m==0) BW[a]=1;
    if(r1<r2)Start_Detection(a+1,h1,h2,r1+1,r2);
}


void Start_Detection_2(unsigned char a,unsigned char h1,unsigned char h2,unsigned char r1,unsigned char r2)
{
	unsigned char i=0,m=0,n=0;
	
    for(i=h1;i>h2;i--)
	{
		if(video[i][r1]==0)m++;
		if(video[i][r1]==1)n++;
	}
	if(m>0)BW2[a]=0;
	if(m==0) BW2[a]=1;
	if(r1<r2)Start_Detection_2(a+1,h1,h2,r1+1,r2);

}

void Start_Detection_3(unsigned char a,unsigned char h1,unsigned char h2,unsigned char r1,unsigned char r2)
{
	unsigned char i=0,m=0,n=0;
	
    for(i=h1;i>h2;i--)
	{
		if(video[i][r1]==0)m++;
		if(video[i][r1]==1)n++;
	}
	if(m>0)BW3[a]=0;
	if(m==0) BW3[a]=1;
	if(r1<r2)Start_Detection_3(a+1,h1,h2,r1+1,r2);

}

void Start_Detection_4(unsigned char a,unsigned char h1,unsigned char h2,unsigned char r1,unsigned char r2)
{
	unsigned char i=0,m=0,n=0;
	
    for(i=h1;i>h2;i--)
	{
		if(video[i][r1]==0)m++;
		if(video[i][r1]==1)n++;
	}
	if(m>0)BW4[a]=0;
	if(m==0) BW4[a]=1;
	if(r1<r2)Start_Detection_4(a+1,h1,h2,r1+1,r2);

}

/*********************************
*name:BW_filter(char n)
*function:起始线检测后的滤波
*method:1，对单独出现的一个点滤波去除
*       2，对单独出现的两个点要看其左右的情况，如果出现左右连续黑色或者白色
           点数大于1的话可以把其滤去，否则不滤。
*explain:n表示要滤波的BW数组号
*note:2号滤波可能不用。
*date:2013.5.3
**********************************/
void BW_filter(char n)
{
  volatile uint8 *p;
  switch(n)
  {
  case 0:
    {
      p=BW;
      if(*p==0)
      {
        p+=2;
        while(p<&BW[99])
        {
          if(*(p-1)!=*p&&*(p+1)!=*p)*p=*(p-1);
          p++;
        }
        /*p=&BW[3];
        while(p<&BW[99])
        {
                if(*(p-1)==*p&&*p!=*(p-2)&&*p!=*(p-3)&&*p!=*(p+1)&&*p!=*(p+2)){*p=*(p-2);*(p-1)=*p;}
                p++;
        }*/
      }
    }
  case 2:
    {
      p=BW2;
      if(*p==0)
      {
        p+=2;
        while(p<&BW2[119])
        {
          if(*(p-1)!=*p&&*(p+1)!=*p)*p=*(p-1);
          p++;
        }
        /*p=&BW2[3];
        while(p<&BW2[119])
        {
                if(*(p-1)==*p&&*p!=*(p-2)&&*p!=*(p-3)&&*p!=*(p+1)&&*p!=*(p+2)){*p=*(p-2);*(p-1)=*p;}
                p++;
        }*/
      }
    }
  case 3:
    {
      p=BW3;
      if(*p==0)
      {
        p+=2;
        while(p<&BW3[79])
        {
          if(*(p-1)!=*p&&*(p+1)!=*p)*p=*(p-1);
          p++;
        }
        /*p=&BW3[3];
        while(p<&BW3[79])
        {
                if(*(p-1)==*p&&*p!=*(p-2)&&*p!=*(p-3)&&*p!=*(p+1)&&*p!=*(p+2)){*p=*(p-2);*(p-1)=*p;}
                p++;
        }*/
      }
    }
  case 4:
    {
      p=BW4;
      if(*p==0)
      {
        p+=2;
        while(p<&BW4[139])
        {
                if(*(p-1)!=*p&&*(p+1)!=*p)*p=*(p-1);
                p++;
        }
        /*p=&BW4[3];
        while(p<&BW4[139])
        {
                if(*(p-1)==*p&&*p!=*(p-2)&&*p!=*(p-3)&&*p!=*(p+1)&&*p!=*(p+2)){*p=*(p-2);*(p-1)=*p;}
                p++;
        }*/
        }
     }
  }
}
/****************起始线的检测函数END****************/
/*******************中心线两点的连接******************/
void LianJie_center(int16 m,int16 n)
{
	if(m-n>1)
	{
		center[(m+n)/2]=(center[m]+center[n])/2;
		LianJie_center((m+n)/2,n);
		LianJie_center(m,(m+n)/2);
	}
}
/*******************中心线两点的连接END******************/

/*************加权平均法对中心线的滤波平滑处理*************/
void center_filter(void)
{
  char code_coe[5]={1,2,3,3,4};
  char code_coe_sum=13;
  int16 i=0,j=0,center_sum=0;
  for(i=52;i>1;i--)
  {
    if(center[i-2]>0)
    {
      center_sum=0;
      for(j=0;j<5;j++)center_sum+=center[j+i-2]*code_coe[j];
      center[i]=(int)(center_sum/code_coe_sum);
    }
  }  
}
/*************加权平均法对中心线的滤波平滑处理*END************/
/*************起始线防抖判断与处理*************/
void Start_Line(void)
{
  int16 i=0,j=0,k=0;
  uint8 num_b1=0,num_b2=0;
  uint8 bianyan3[2][55]={0};//注意这个需要清零
  uint8 center3[55]={0};//注意这个需要清零
  if(xu_xian2==0)
  {
    if(END_Line_2>5&&END_Line_2<35)
    {
      for(i=54;i>35;i--)
      {
        if(bianyan[0][i]!=bianyan2[0][i])num_b1++;
        if(bianyan[0][i]!=bianyan2[0][i])num_b2++;
      }
                  
      if(num_b1==0&&num_b2==0)
      {
              
        if(abs(center[35]-center[54])<15)
        {
          for(k=END_Line_2;k<30;k++)
          {
                  if(center[k])break;
          }
          for(j=center[k];j>30;j--)
          {
            if(video[END_Line_2][j]==0&&video[END_Line_2][j-1]==0)
            {
                    bianyan3[0][END_Line_2]=j;
                    break;
            }
          }
          for(j=center[k];j<130;j++)
          {
            if(video[END_Line_2][j]==0&&video[END_Line_2][j+1]==0)
            {
                    bianyan3[1][END_Line_2]=j;
                    break;
            }
          }
          if(bianyan3[1][END_Line_2]-bianyan3[0][END_Line_2]>20)
          {
                  center3[END_Line_2]=(bianyan3[1][END_Line_2]+bianyan3[0][END_Line_2])/2;
          }
          if(center3[END_Line_2]>30&&center[END_Line_2]<130)
          {
            for(i=END_Line_2-1;i>=0;i--)
            {
              for(j=center3[i+1];j>10;j--)
              {
                if(video[i][j]==0&&video[i][j-1]==0)
                {
                        bianyan3[0][i]=j;
                        break;
                }
              }
              for(j=center3[i+1];j<150;j++)
              {
                if(video[i][j]==0&&video[i][j+1]==0)
                {
                        bianyan3[1][i]=j;
                        break;
                }
              }
              center3[i]=(bianyan3[1][i]+bianyan3[0][i])/2;
              //if((bianyan3[1][i]-bianyan3[0][i])<5)break;
            }
            if(i==-1)start_line2=1;
            /*if(i==-1)
            {
                    for(i=END_Line_2;i>=0;i--)center3[i]=(bianyan3[1][i]+bianyan3[0][i])/2;
  
            }*/
    
          }
        }
      }
    }
  }
	
  if(start_line2==1)
  {
    for(i=END_Line_2;i>=0;i--)center[i]=center3[i];
    LianJie_center(k,END_Line_2);
  }
}
/*************起始线防抖判断与处理*END************/
/******************虚线找断点算法*****************/
//虚线找断点算法只能用于先找进出的断点
void xu_xian_find(char a,int i,int h)
{
	int16 f1=0,f2=0;
	int16 m=0;
	if(a==0)
	{
		for(m=i;m>h;m--)
		{
			if((!(bianyan2[a][m]==bianyan[a][m]))||bianyan2[a][m]<2)
			{
				f1=m+1;
				break;
			}
		}
		for(;m>h;m--)
		{
			if(bianyan[a][m]>2)
			{
				if(bianyan2[a][m]==bianyan[a][m])
				{
					f2=m;
					break;
				}
			}
		}
		
		if(!(m==h))
		{
			LianJie(a,f1,f2);
			xu_xian_find(a,m,h);
		}
	}
	if(a==1)
	{
		for(m=i;m>h;m--)
		{
			if((!(bianyan2[a][m]==bianyan[a][m]))||bianyan2[a][m]>=V-3)
			{
				f1=m+1;
				break;
			}
		}
	
		for(;m>h;m--)
		{
			if(bianyan[a][m]<V-3)
			{
				if(bianyan2[a][m]==bianyan[a][m])
				{
					f2=m;
					break;
				}
			}
		}
		
		if(!(m==h))
		{
			LianJie(a,f1,f2);
			xu_xian_find(a,m,h);
		}
	}

}
/******************虚线找断点算法END*****************/
void sub_e(uint8 value,uint8 S_end,uint8 M_start,uint8 M_end,uint8 B_start){
  if(value<=M_start){
    sub.s=100;
    sub.m=0;
    sub.b=0;
  }
  if(value<S_end&&value>M_start){
    sub.s=100*(S_end-value)/(S_end-M_start);
    sub.m=100*(value-M_start)/(S_end-M_start);
    sub.b=0;
  }
  if(value>=S_end&&value<=B_start){
    sub.s=0;
    sub.m=100;
    sub.b=0;
  }
  if(value<M_end&&value>B_start){
    sub.s=0;
    sub.m=100*(M_end-value)/(M_end-B_start);
    sub.b=100*(value-B_start)/(M_end-B_start);
  }
  if(value>=M_end){
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
  //if(Be&&Sc)  {vb[ib]=min(Be,Sc);ib++;}
  if(Be&&Sc)  {vs[is]=min(Be,Sc);is++;}
  
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
//  if(Be&&Bc)  {vb[ib]=min(Be,Bc);ib++;}
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

/*******找到莫连续几行的中心线的最大最小值END********/

/******************************************************
*function name:int16 stages(int16 value,int16 parameter)
*Description:  求出位置值
*Imput:        value:算出的当场中心
               parameter：视场中心
*Output:       位置值
*Date:         2013.5.6
*Name:         潘杰
*******************************************************/
int16 stages_P(int16 value,int16 parameter)
{
  //if(!value) return unclear;
  if(value<=parameter)
  {
    if(value>=(parameter-5)){NULL;return(0);}
    else if(value>=(parameter-10)){NULL;return(-1);}  
    else if(value>=(parameter-15)){NULL;return(-2);}    
    else if(value>=(parameter-20)){NULL;return(-3);}
    else if(value>=(parameter-25)){NULL;return(-4);}
    else if(value>=(parameter-30)){NULL;return(-5);}
    else {NULL;return(-6);}
  }
  else
  {
    if(value<=(parameter+5)){NULL;return(0);}
    else if(value<=(parameter+10)){NULL;return(1);}  
    else if(value<=(parameter+15)){NULL;return(2);}    
    else if(value<=(parameter+20)){NULL;return(3);}
    else if(value<=(parameter+25)){NULL;return(4);}
    else if(value<=(parameter+30)){NULL;return(5);}
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
    if(value>=-5){NULL;return(0);}
    else if(value>=-10){NULL;return(-1);}  
    else if(value>=-15){NULL;return(-2);}    
    else if(value>=-20){NULL;return(-3);}
    else if(value>=-25){NULL;return(-4);}
    else if(value>=-30){NULL;return(-5);}
    else {NULL;return(-6);}
  }
  else
  {
    if(value<=5){NULL;return(0);}
    else if(value<=10){NULL;return(1);}  
    else if(value<=15){NULL;return(2);}    
    else if(value<=20){NULL;return(3);}
    else if(value<=25){NULL;return(4);}
    else if(value<=30){NULL;return(5);}
    else {NULL;return(6);}
  }
}



/***********************************
*name:judge_small_S
*fuction:区别直道与小S，确定小S参数，使小S能够直走
*time:2013.6.22
*way:1，首先小S被认定为很直的直道20
     2，通过54~25行的中心值的最大与最小进行比较，如果最大值减去最小值大于某个数则认为是小S
***********************************/
uint16 judge_small_S(void)
{
  uint8 i=0,center_max=0,center_min=0,center_chazhi=0;
  center_max=center[54];
  center_min=center[54];
  for(i=53;i>=20;i--)
  {
  if(center_max<center[i])center_max=center[i];
          if(center_min>center[i])center_min=center[i];
  }
  center_chazhi=center_max-center_min;
  if(center_chazhi>9)return 1;
  else return 0;
}


int16 choose_PD(m)
{
  int16 i=0,num=0;
  uint8 track_form=0;//j=0,
  int16 buff1=0;//,buff2=0,buff3=0
  //int8 position_now_P=0,position_now_low_P=0,position_now_D=0;
  int16 line_head=0,line_end=54,line_mid=0;//line_head=30
  int16 line_mid_d=0,line_end_d=0;
  int16 head=0,end=0;
  int16 position_now_sum=0;
  //int16 xiaoS_flag=0;
  int16 piancha_weizhi=0,piancha_num=0;
 int16 position_diff_zhi=0;
  
  
  while(!center[i]&&i<H)i++;
  topline=i;

  if(i<8)
  {
    track_form=3;
    Servo.P=P_High;
    Servo.D=D_High;
    servo_d=D_piancha_high;
    servo.head=H_High;
    servo.tail=T_High;
    see_distance=3;
  }
  if(i>=8&&i<22)
  {
    track_form=2;
    Servo.P=P_Mid;
    Servo.D=D_Mid;
    servo_d=D_piancha_mid;
    servo.head=H_Mid;
    servo.tail=T_Mid;
    see_distance=2;
  }
  if(i>=22)
  {
    track_form=1;
    Servo.P=P_Low;
    Servo.D=D_Low;
    servo_d=D_piancha_low;
    servo.head=H_Low;
    servo.tail=T_Low;
    see_distance=1;
  }
  
  for(i=servo.head;i<servo.tail;i++)
  {
    if(center[i])
    {
      num++;
      position_now_sum+=center[i];
    }  
  }
 position_old_old_old=position_old_old;
 position_old_old=position_old;
 position_old=position_now;
 if(num)
  position_now=position_now_sum/num;   //算出当场的中心位置
  num=0;
  
  weizhi_old=weizhi;
  piancha_weizhi=0;
  for(i=35;i<=45;i++){
    if(center[i]){
      piancha_weizhi+=center[i];
      piancha_num++;
    }
  }
  
  
  if(piancha_num)
    piancha_weizhi=piancha_weizhi/piancha_num;
  weizhi=piancha_weizhi;
  for(i=50;i<55;i++)
  {
    if(center[i])
    {
      num++;
      position_now_low+=center[i];
    }  
  }
  position_now_low=position_now_low/num;
  
  while(center[line_head]==0) line_head++;
  while(center[line_end]==0)  line_end--;
  line_end_d=(line_head+2*line_end)/3;
  line_mid_d=(line_end+2*line_head)/3;
  for(i=line_head;i<line_mid_d;i++)head+=center[i];
  head=head/(line_mid_d-line_head);
  for(i=line_mid_d;i<=line_end_d;i++)end+=center[i];
  end=end/(line_end_d-line_mid_d+1);
  position_diff=end-head;
  
  position_now_low_P=stages_low_P(position_now_low,video_Middle);
  position_now_P=stages_P(position_now,video_Middle);
  position_now_D=stages_D(position_diff);
  switch(position_now_P)                       //p=-6              
  {
  case -6:                                      //p=-6  D=-6
    {
      switch(position_now_D)
      {
      case -6:
        {
          switch(track_form)
          {
          case 1:Servo.P+=6;Servo.D+=0;break;
          case 2:Servo.P+=6;Servo.D+=0;break;
          case 3:Servo.P+=6;Servo.D+=0;break;
          }
        }break;
      case -5:                               //p=-6  D=-5
        {
          switch(track_form)
          {
          case 1:Servo.P+=6;Servo.D+=0;break;
          case 2:Servo.P+=6;Servo.D+=0;break;
          case 3:Servo.P+=6;Servo.D+=0;break;
          }
        }break;
      case -4:                             //p=-6  D=-4
        {
          switch(track_form)
          {
          case 1:Servo.P+=5;Servo.D-=1;break;
          case 2:Servo.P+=5;Servo.D-=1;break;
          case 3:Servo.P+=5;Servo.D-=1;break;
          }
        }break;
      case -3:                               //p=-6  D=-3
        {
          switch(track_form)
          {
          case 1:Servo.P+=5;Servo.D-=2;break;
          case 2:Servo.P+=5;Servo.D-=2;break;
          case 3:Servo.P+=5;Servo.D-=2;break;
          }
        }break;
      case -2:                                  //p=-6  D=-2
        {
          switch(track_form)
          {
          case 1:Servo.P+=5;Servo.D-=3;break;
          case 2:Servo.P+=5;Servo.D-=3;break;
          case 3:Servo.P+=5;Servo.D-=3;break;
          }
        }break;
      case -1:                                //p=-6  D=-1
        {
          switch(track_form)
          {
          case 1:Servo.P+=5;Servo.D-=4;break;
          case 2:Servo.P+=5;Servo.D-=4;break;
          case 3:Servo.P+=5;Servo.D-=4;break;
          }
        }break;
      case 0:Servo.P+=5;break;                 //p=-6  D=0
      case 1:                                  //p=-6  D=1
        {
          switch(track_form)
          {
          case 1:Servo.P+=5;Servo.D+=1;break;
          case 2:Servo.P+=5;Servo.D+=1;break;
          case 3:Servo.P+=5;Servo.D+=1;break;
          }
        }break;
      case 2:                                 //p=-6  D=2
        {
          switch(track_form)
          {
          case 1:Servo.P+=5;Servo.D+=1;break;
          case 2:Servo.P+=5;Servo.D+=1;break;
          case 3:Servo.P+=5;Servo.D+=1;break;
          }
        }break;
      case 3:                                  //p=-6  D=3
        {
          switch(track_form)
          {
          case 1:Servo.P+=5;Servo.D+=2;break;
          case 2:Servo.P+=5;Servo.D+=2;break;
          case 3:Servo.P+=5;Servo.D+=2;break;
          }
        }break;
      case 4:                                //p=-6  D=4
        {
          switch(track_form)
          {
          case 1:Servo.P+=5;Servo.D+=2;break;
          case 2:Servo.P+=5;Servo.D+=2;break;
          case 3:Servo.P+=5;Servo.D+=2;break;
          }
        }break;
      case 5:                                 //p=-6  D=5
        {
          switch(track_form)
          {
          case 1:Servo.P+=5;Servo.D+=3;break;
          case 2:Servo.P+=4;Servo.D+=2;break;
          case 3:Servo.P+=4;Servo.D+=2;break;
          }
        }break;
      case 6:                                 //p=-6  D=6
        {
          switch(track_form)
          {
          case 1:Servo.P+=5;Servo.D+=3;break;
          case 2:Servo.P+=4;Servo.D+=2;break;
          case 3:Servo.P+=4;Servo.D+=2;break;
          }
        }break;
      }
    }break;
  case -5:                                   //P=-5  D=-6
    {
      switch(position_now_D)
      {
      case -6:
        {
          switch(track_form)
          {
          case 1:Servo.P+=4;Servo.D+=1;break;
          case 2:Servo.P+=4;Servo.D+=1;break;
          case 3:Servo.P+=4;Servo.D+=1;break;
          }
        }break;
      case -5:                                //P=-5  D=-5
        {
          switch(track_form)
          {
          case 1:Servo.P+=4;Servo.D+=0;break;
          case 2:Servo.P+=4;Servo.D+=0;break;
          case 3:Servo.P+=4;Servo.D+=0;break;
          }
        }break;
      case -4:                                    //P=-5  D=-4
        {
          switch(track_form)
          {
          case 1:Servo.P+=4;Servo.D-=1;break;
          case 2:Servo.P+=4;Servo.D-=1;break;
          case 3:Servo.P+=4;Servo.D-=1;break;
          }
        }break;
      case -3:                                  //P=-5  D=-3
        {
          switch(track_form)
          {
          case 1:Servo.P+=4;Servo.D-=2;break;
          case 2:Servo.P+=4;Servo.D-=2;break;
          case 3:Servo.P+=4;Servo.D-=2;break;
          }
        }break;
      case -2:                                 //P=-5  D=-2
        {
          switch(track_form)
          {
          case 1:Servo.P+=4;Servo.D-=3;break;
          case 2:Servo.P+=4;Servo.D-=3;break;
          case 3:Servo.P+=4;Servo.D-=3;break;
          }
        }break;
      case -1:                                 //P=-5  D=-1
        {
          switch(track_form)
          {
          case 1:Servo.P+=4;Servo.D-=4;break;
          case 2:Servo.P+=4;Servo.D-=4;break;
          case 3:Servo.P+=4;Servo.D-=4;break;
          }
        }break;
      case 0:Servo.P+=4;break;                //P=-5  D=-0
      case 1:                                 //P=-5  D=1
        {
          switch(track_form)
          {
          case 1:Servo.P+=4;Servo.D+=1;break;
          case 2:Servo.P+=4;Servo.D+=1;break;
          case 3:Servo.P+=4;Servo.D+=1;break;
          }
        }break;
      case 2:                               //P=-5  D=2
        {
          switch(track_form)
          {
          case 1:Servo.P+=4;Servo.D+=1;break;
          case 2:Servo.P+=4;Servo.D+=1;break;
          case 3:Servo.P+=4;Servo.D+=1;break;
          }
        }break;
      case 3:                                //P=-5  D=3
        {
          switch(track_form)
          {
          case 1:Servo.P+=4;Servo.D+=2;break;
          case 2:Servo.P+=4;Servo.D+=2;break;
          case 3:Servo.P+=4;Servo.D+=2;break;
          }
        }break;
      case 4:                                 //P=-5  D=4
        {
          switch(track_form)
          {
          case 1:Servo.P+=4;Servo.D+=2;break;
          case 2:Servo.P+=4;Servo.D+=2;break;
          case 3:Servo.P+=4;Servo.D+=2;break;
          }
        }break;
      case 5:                                 //P=-5  D=5
        {
          switch(track_form)
          {
          case 1:Servo.P+=4;Servo.D+=2;break;
          case 2:Servo.P+=3;Servo.D+=1;break;
          case 3:Servo.P+=3;Servo.D+=1;break;
          }
        }break;
      case 6:                                  //P=-5  D=6
        {
          switch(track_form)
          {
          case 1:Servo.P+=4;Servo.D+=2;break;
          case 2:Servo.P+=3;Servo.D+=1;break;
          case 3:Servo.P+=3;Servo.D+=1;break;
          }
        }break;
      }
    }break;
  case -4:                                     //P=-4  D=-6
    {
      switch(position_now_D)
      {
      case -6:
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D+=3;break;
          case 2:Servo.P+=2;Servo.D+=2;break;
          case 3:Servo.P+=2;Servo.D+=2;break;
          }
        }break;
      case -5:                                   //P=-4  D=-5
        { 
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D+=2;break;
          case 2:Servo.P+=2;Servo.D+=2;break;
          case 3:Servo.P+=2;Servo.D+=2;break;
          }
        }break;
      case -4:                                //P=-4  D=-4
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D+=1;break;
          case 2:Servo.P+=2;Servo.D+=1;break;
          case 3:Servo.P+=2;Servo.D+=1;break;
          }
        }break;
      case -3:                                 //P=-4  D=-3
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D-=0;break;
          case 2:Servo.P+=2;Servo.D-=0;break;
          case 3:Servo.P+=2;Servo.D-=0;break;
          }
        }break;
      case -2:                                //P=-4  D=-2
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D-=1;break;
          case 2:Servo.P+=2;Servo.D-=1;break;
          case 3:Servo.P+=2;Servo.D-=1;break;
          }
        }break;
      case -1:                                    //P=-4  D=-1
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D-=2;break;
          case 2:Servo.P+=2;Servo.D-=2;break;
          case 3:Servo.P+=2;Servo.D-=2;break;
          }
        }break;
      case 0:Servo.P+=2;break;                      //P=-4  D=0
      case 1:                                        //P=-4  D=1
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D+=1;break;
          case 2:Servo.P+=2;Servo.D+=1;break;
          case 3:Servo.P+=2;Servo.D+=1;break;
          }
        }break;
      case 2:                                        //P=-4  D=2
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D+=1;break;
          case 2:Servo.P+=2;Servo.D+=1;break;
          case 3:Servo.P+=2;Servo.D+=1;break;
          }
        }break;
      case 3:                                     //P=-4  D=3
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D+=2;break;
          case 2:Servo.P+=2;Servo.D+=2;break;
          case 3:Servo.P+=2;Servo.D+=2;break;
          }
        }break;
      case 4:                                   //P=-4  D=4
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D+=2;break;
          case 2:Servo.P+=2;Servo.D+=2;break;
          case 3:Servo.P+=2;Servo.D+=2;break;
          }
        }break;
      case 5:                                   //P=-4  D=5
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D+=3;break;
          case 2:Servo.P+=2;Servo.D+=3;break;
          case 3:Servo.P+=2;Servo.D+=3;break;
          }
        }break;
      case 6:                                   //P=-4  D=6
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D+=4;break;
          case 2:Servo.P+=2;Servo.D+=4;break;
          case 3:Servo.P+=2;Servo.D+=4;break;
          }
        }break;
      }
    }break;
  case -3:                                      //P=-3  D=-6
    {
      switch(position_now_D)
      {
      case -6:
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D+=1;break;
          case 2:Servo.P+=2;Servo.D+=1;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=2;Servo.D+=1;break;
              case 2:Servo.P+=2;Servo.D+=1;break;
              case 1:Servo.P+=2;Servo.D+=1;break;
              case 0:Servo.P+=2;Servo.D+=0;break;
              case -1:Servo.P+=2;Servo.D+=0;break;
              case -2:Servo.P+=2;Servo.D+=0;break;
              case -3:Servo.P+=2;Servo.D+=0;break;
              }
            }break;
          }
        }break;
      case -5:                                      //P=-3  D=-5
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D+=0;break;
          case 2:Servo.P+=2;Servo.D+=0;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=2;Servo.D+=1;break;
              case 2:Servo.P+=2;Servo.D+=1;break;
              case 1:Servo.P+=2;Servo.D+=1;break;
              case 0:Servo.P+=2;Servo.D+=0;break;
              case -1:Servo.P+=2;Servo.D+=0;break;
              case -2:Servo.P+=2;Servo.D+=0;break;
              case -3:Servo.P+=2;Servo.D+=0;break;
              }
            }break;
          }
        }break;
      case -4:                                         //P=-3  D=-4
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D-=1;break;
          case 2:Servo.P+=2;Servo.D-=1;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=2;Servo.D-=1;break;
              case 2:Servo.P+=2;Servo.D-=1;break;
              case 1:Servo.P+=2;Servo.D-=1;break;
              case 0:Servo.P+=2;Servo.D-=0;break;
              case -1:Servo.P+=2;Servo.D-=2;break;
              case -2:Servo.P+=2;Servo.D-=2;break;
              case -3:Servo.P+=2;Servo.D-=2;break;
              }
            }break;
          }
        }break;
      case -3:                                       //P=-3  D=-3
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D-=2;break;
          case 2:Servo.P+=2;Servo.D-=2;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=2;Servo.D-=2;break;
              case 2:Servo.P+=2;Servo.D-=2;break;
              case 1:Servo.P+=2;Servo.D-=2;break;
              case 0:Servo.P+=2;Servo.D-=2;break;
              case -1:Servo.P+=2;Servo.D-=3;break;
              case -2:Servo.P+=2;Servo.D-=3;break;
              case -3:Servo.P+=2;Servo.D-=3;break;
              }
            }break;
          }
        }break;
      case -2:                                        //P=-3  D=-2
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D-=3;break;
          case 2:Servo.P+=2;Servo.D-=3;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=2;Servo.D-=3;break;
              case 2:Servo.P+=2;Servo.D-=3;break;
              case 1:Servo.P+=2;Servo.D-=3;break;
              case 0:Servo.P+=2;Servo.D-=3;break;
              case -1:Servo.P+=2;Servo.D-=4;break;
              case -2:Servo.P+=2;Servo.D-=4;break;
              case -3:Servo.P+=2;Servo.D-=4;break;
              }
            }break;
          }
        }break;
      case -1:                                        //P=-3  D=-1
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D-=4;break;
          case 2:Servo.P+=2;Servo.D-=4;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=2;Servo.D-=3;break;
              case 2:Servo.P+=2;Servo.D-=3;break;
              case 1:Servo.P+=2;Servo.D-=3;break;
              case 0:Servo.P+=2;Servo.D-=3;break;
              case -1:Servo.P+=2;Servo.D-=4;break;
              case -2:Servo.P+=2;Servo.D-=4;break;
              case -3:Servo.P+=2;Servo.D-=4;break;
              }
            }break;
          }
        }break;
      case 0:Servo.P+=2;break;                         //P=-3  D=0
      case 1:                                            //P=-3  D=1
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D+=1;break;
          case 2:Servo.P+=2;Servo.D+=1;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=2;Servo.D+=1;break;
              case 2:Servo.P+=2;Servo.D+=1;break;
              case 1:Servo.P+=2;Servo.D+=1;break;
              case 0:Servo.P+=2;Servo.D+=1;break;
              case -1:Servo.P+=2;Servo.D+=1;break;
              case -2:Servo.P+=2;Servo.D+=1;break;
              case -3:Servo.P+=2;Servo.D+=1;break;
              }
            }break;
          }
        }break;
      case 2:                                                //P=-3  D=2
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D+=1;break;
          case 2:Servo.P+=2;Servo.D+=1;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=2;Servo.D+=1;break;
              case 2:Servo.P+=2;Servo.D+=1;break;
              case 1:Servo.P+=2;Servo.D+=1;break;
              case 0:Servo.P+=2;Servo.D+=1;break;
              case -1:Servo.P+=2;Servo.D+=1;break;
              case -2:Servo.P+=2;Servo.D+=1;break;
              case -3:Servo.P+=2;Servo.D+=1;break;
              }
            }break;
          }
        }break;
      case 3:                                              //P=-3  D=3
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D+=2;break;
          case 2:Servo.P+=2;Servo.D+=2;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=2;Servo.D+=2;break;
              case 2:Servo.P+=2;Servo.D+=2;break;
              case 1:Servo.P+=2;Servo.D+=2;break;
              case 0:Servo.P+=2;Servo.D+=2;break;
              case -1:Servo.P+=2;Servo.D+=2;break;
              case -2:Servo.P+=2;Servo.D+=2;break;
              case -3:Servo.P+=2;Servo.D+=2;break;
              }
            }break;
          }
        }break;
      case 4:                                            //P=-3  D=4
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D+=2;break;
          case 2:Servo.P+=2;Servo.D+=2;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=2;Servo.D+=1;break;
              case 2:Servo.P+=2;Servo.D+=1;break;
              case 1:Servo.P+=2;Servo.D+=1;break;
              case 0:Servo.P+=2;Servo.D+=1;break;
              case -1:Servo.P+=2;Servo.D+=2;break;
              case -2:Servo.P+=2;Servo.D+=2;break;
              case -3:Servo.P+=2;Servo.D+=2;break;
              }
            }break;
          }
        }break;
      case 5:                                               //P=-3  D=5
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D+=3;break;
          case 2:Servo.P+=2;Servo.D+=3;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=2;Servo.D+=1;break;
              case 2:Servo.P+=2;Servo.D+=1;break;
              case 1:Servo.P+=2;Servo.D+=1;break;
              case 0:Servo.P+=2;Servo.D+=1;break;
              case -1:Servo.P+=2;Servo.D+=2;break;
              case -2:Servo.P+=2;Servo.D+=2;break;
              case -3:Servo.P+=2;Servo.D+=2;break;
              }
            }break;
          }
        }break;
      case 6:                                            //P=-3  D=6
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D+=4;break;
          case 2:Servo.P+=2;Servo.D+=4;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=2;Servo.D+=1;break;
              case 2:Servo.P+=2;Servo.D+=2;break;
              case 1:Servo.P+=2;Servo.D+=3;break;
              case 0:Servo.P+=2;Servo.D+=3;break;
              case -1:Servo.P+=2;Servo.D+=3;break;
              case -2:Servo.P+=2;Servo.D+=3;break;
              case -3:Servo.P+=2;Servo.D+=3;break;
              }
            }break;
          }
        }break;
      }
    }break;
  case -2:                                             //P=-2  D=-6                                          
    {
      switch(position_now_D)
      {
      case -6:
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D+=4;break;
          case 2:Servo.P+=1;Servo.D+=4;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=1;Servo.D+=4;break;
              case 2:Servo.P+=1;Servo.D+=4;break;
              case 1:Servo.P+=1;Servo.D+=4;break;
              case 0:Servo.P+=1;Servo.D+=4;break;
              case -1:Servo.P+=1;Servo.D+=3;break;
              case -2:Servo.P+=1;Servo.D+=3;break;
              case -3:Servo.P+=1;Servo.D+=3;break;
              }
            }break;
          }
        }break;
      case -5:                                //P=-2  D=-5
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D+=3;break;
          case 2:Servo.P+=1;Servo.D+=3;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=1;Servo.D+=3;break;
              case 2:Servo.P+=1;Servo.D+=3;break;
              case 1:Servo.P+=1;Servo.D+=3;break;
              case 0:Servo.P+=1;Servo.D+=2;break;
              case -1:Servo.P+=1;Servo.D+=2;break;
              case -2:Servo.P+=1;Servo.D+=2;break;
              case -3:Servo.P+=1;Servo.D+=2;break;
              }
            }break;
          }
        }break;
      case -4:                                   //P=-2  D=-4    
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D+=2;break;
          case 2:Servo.P+=1;Servo.D+=2;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=1;Servo.D+=2;break;
              case 2:Servo.P+=1;Servo.D+=2;break;
              case 1:Servo.P+=1;Servo.D+=2;break;
              case 0:Servo.P+=1;Servo.D+=2;break;
              case -1:Servo.P+=1;Servo.D+=1;break;
              case -2:Servo.P+=1;Servo.D+=1;break;
              case -3:Servo.P+=1;Servo.D+=1;break;
              }
            }break;
          }
        }break;
      case -3:                                 //P=-2  D=-3
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D+=1;break;
          case 2:Servo.P+=1;Servo.D+=1;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=1;Servo.D+=1;break;
              case 2:Servo.P+=1;Servo.D+=1;break;
              case 1:Servo.P+=1;Servo.D+=1;break;
              case 0:Servo.P+=1;Servo.D+=0;break;
              case -1:Servo.P+=1;Servo.D+=0;break;
              case -2:Servo.P+=1;Servo.D+=0;break;
              case -3:Servo.P+=1;Servo.D+=0;break;
              }
            }break;
          }
        }break;
      case -2:                                    //P=-2  D=-2
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D-=0;break;
          case 2:Servo.P+=1;Servo.D-=0;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=1;Servo.D+=1;break;
              case 2:Servo.P+=1;Servo.D+=1;break;
              case 1:Servo.P+=1;Servo.D+=1;break;
              case 0:Servo.P+=1;Servo.D+=0;break;
              case -1:Servo.P+=1;Servo.D+=0;break;
              case -2:Servo.P+=1;Servo.D+=0;break;
              case -3:Servo.P+=1;Servo.D+=0;break;
              }
            }break;
          }
        }break;
      case -1:                                    //P=-2  D=-1
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D-=1;break;
          case 2:Servo.P+=1;Servo.D-=1;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=1;Servo.D-=1;break;
              case 2:Servo.P+=1;Servo.D-=1;break;
              case 1:Servo.P+=1;Servo.D-=1;break;
              case 0:Servo.P+=1;Servo.D-=1;break;
              case -1:Servo.P+=1;Servo.D-=1;break;
              case -2:Servo.P+=1;Servo.D-=1;break;
              case -3:Servo.P+=1;Servo.D-=1;break;
              }
            }break;
          }
        }break;
      case 0:Servo.P+=1;break;                      //P=-2  D=0
      case 1:                                       //P=-2  D=1
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D+=1;break;
          case 2:Servo.P+=1;Servo.D+=1;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=1;Servo.D+=1;break;
              case 2:Servo.P+=1;Servo.D+=1;break;
              case 1:Servo.P+=1;Servo.D+=1;break;
              case 0:Servo.P+=1;Servo.D+=1;break;
              case -1:Servo.P+=1;Servo.D+=1;break;
              case -2:Servo.P+=1;Servo.D+=1;break;
              case -3:Servo.P+=1;Servo.D+=1;break;
              }
            }break;
          }
        }break;
      case 2:                                      //P=-2  D=2
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D+=1;break;
          case 2:Servo.P+=1;Servo.D+=1;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=1;Servo.D+=1;break;
              case 2:Servo.P+=1;Servo.D+=1;break;
              case 1:Servo.P+=1;Servo.D+=1;break;
              case 0:Servo.P+=1;Servo.D+=1;break;
              case -1:Servo.P+=1;Servo.D+=1;break;
              case -2:Servo.P+=1;Servo.D+=1;break;
              case -3:Servo.P+=1;Servo.D+=1;break;
              }
            }break;
          }
        }break;
      case 3:                                        //P=-2  D=3
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D+=2;break;
          case 2:Servo.P+=1;Servo.D+=2;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=1;Servo.D+=2;break;
              case 2:Servo.P+=1;Servo.D+=2;break;
              case 1:Servo.P+=1;Servo.D+=2;break;
              case 0:Servo.P+=1;Servo.D+=2;break;
              case -1:Servo.P+=1;Servo.D+=2;break;
              case -2:Servo.P+=1;Servo.D+=2;break;
              case -3:Servo.P+=1;Servo.D+=2;break;
              }
            }break;
          }
        }break;
      case 4:                                       //P=-2  D=4
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D+=2;break;
          case 2:Servo.P+=1;Servo.D+=2;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=1;Servo.D+=1;break;
              case 2:Servo.P+=1;Servo.D+=1;break;
              case 1:Servo.P+=1;Servo.D+=1;break;
              case 0:Servo.P+=1;Servo.D+=1;break;
              case -1:Servo.P+=1;Servo.D+=2;break;
              case -2:Servo.P+=1;Servo.D+=2;break;
              case -3:Servo.P+=1;Servo.D+=2;break;
              }
            }break;
          }
        }break;
      case 5:                                       //P=-2  D=5
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D+=3;break;
          case 2:Servo.P+=1;Servo.D+=3;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=1;Servo.D+=2;break;
              case 2:Servo.P+=1;Servo.D+=2;break;
              case 1:Servo.P+=1;Servo.D+=2;break;
              case 0:Servo.P+=1;Servo.D+=2;break;
              case -1:Servo.P+=1;Servo.D+=3;break;
              case -2:Servo.P+=1;Servo.D+=3;break;
              case -3:Servo.P+=1;Servo.D+=3;break;
              }
            }break;
          }
        }break;
      case 6:                                        //P=-2  D=6
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D+=4;break;
          case 2:Servo.P+=1;Servo.D+=4;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=1;Servo.D+=3;break;
              case 2:Servo.P+=1;Servo.D+=3;break;
              case 1:Servo.P+=1;Servo.D+=3;break;
              case 0:Servo.P+=1;Servo.D+=3;break;
              case -1:Servo.P+=1;Servo.D+=4;break;
              case -2:Servo.P+=1;Servo.D+=4;break;
              case -3:Servo.P+=1;Servo.D+=4;break;
              }
            }break;
          }
        }break;
      }
    }break;
   case -1:                                           //P=-1  D=-6
    {
      switch(position_now_D)
      {
      case -6:
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D+=5;break;
          case 2:Servo.P+=1;Servo.D+=5;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=1;Servo.D+=5;break;
              case 2:Servo.P+=1;Servo.D+=5;break;
              case 1:Servo.P+=1;Servo.D+=5;break;
              case 0:Servo.P+=1;Servo.D+=6;break;
              case -1:Servo.P+=1;Servo.D+=6;break;
              case -2:Servo.P+=1;Servo.D+=6;break;
              case -3:Servo.P+=1;Servo.D+=6;break;
              }
            }break;
          }
        }break;
      case -5:                                     //P=-1  D=-5
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D+=4;break;
          case 2:Servo.P+=1;Servo.D+=4;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=1;Servo.D+=4;break;
              case 2:Servo.P+=1;Servo.D+=4;break;
              case 1:Servo.P+=1;Servo.D+=4;break;
              case 0:Servo.P+=1;Servo.D+=3;break;
              case -1:Servo.P+=1;Servo.D+=3;break;
              case -2:Servo.P+=1;Servo.D+=3;break;
              case -3:Servo.P+=1;Servo.D+=3;break;
              }
            }break;
          }
        }break;
      case -4:                                    //P=-1  D=-4
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D+=3;break;
          case 2:Servo.P+=1;Servo.D+=3;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=1;Servo.D+=3;break;
              case 2:Servo.P+=1;Servo.D+=3;break;
              case 1:Servo.P+=1;Servo.D+=3;break;
              case 0:Servo.P+=1;Servo.D+=2;break;
              case -1:Servo.P+=1;Servo.D+=2;break;
              case -2:Servo.P+=1;Servo.D+=2;break;
              case -3:Servo.P+=1;Servo.D+=2;break;
              }
            }break;
          }
        }break;
      case -3:                                    //P=-1  D=-3
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D+=2;break;
          case 2:Servo.P+=1;Servo.D+=2;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=1;Servo.D+=2;break;
              case 2:Servo.P+=1;Servo.D+=2;break;
              case 1:Servo.P+=1;Servo.D+=2;break;
              case 0:Servo.P+=1;Servo.D+=1;break;
              case -1:Servo.P+=1;Servo.D+=1;break;
              case -2:Servo.P+=1;Servo.D+=1;break;
              case -3:Servo.P+=1;Servo.D+=1;break;
              }
            }break;
          }
        }break;
      case -2:                                    //P=-1  D=-2
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D+=1;break;
          case 2:Servo.P+=1;Servo.D+=1;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=1;Servo.D+=1;break;
              case 2:Servo.P+=1;Servo.D+=1;break;
              case 1:Servo.P+=1;Servo.D+=1;break;
              case 0:Servo.P+=1;Servo.D+=1;break;
              case -1:Servo.P+=1;Servo.D+=1;break;
              case -2:Servo.P+=1;Servo.D+=1;break;
              case -3:Servo.P+=1;Servo.D+=1;break;
              }
            }break;
          }
        }break;
      case -1:                                       //P=-1  D=-1
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D+=0;break;
          case 2:Servo.P+=1;Servo.D+=0;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=1;Servo.D+=0;break;
              case 2:Servo.P+=1;Servo.D+=0;break;
              case 1:Servo.P+=1;Servo.D+=0;break;
              case 0:Servo.P+=1;Servo.D+=0;break;
              case -1:Servo.P+=1;Servo.D+=0;break;
              case -2:Servo.P+=1;Servo.D+=0;break;
              case -3:Servo.P+=1;Servo.D+=0;break;
              }
            }break;
          }
        }break;
      case 0:Servo.P+=6;break;                      //P=-1  D=0
      case 1:                                      //P=-1  D=1
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D+=1;break;
          case 2:Servo.P+=1;Servo.D+=1;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=1;Servo.D+=1;break;
              case 2:Servo.P+=1;Servo.D+=1;break;
              case 1:Servo.P+=1;Servo.D+=1;break;
              case 0:Servo.P+=1;Servo.D+=1;break;
              case -1:Servo.P+=1;Servo.D+=1;break;
              case -2:Servo.P+=1;Servo.D+=1;break;
              case -3:Servo.P+=1;Servo.D+=1;break;
              }
            }break;
          }
        }break;
      case 2:                                         //P=-1  D=2
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D+=1;break;
          case 2:Servo.P+=1;Servo.D+=1;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=1;Servo.D+=1;break;
              case 2:Servo.P+=1;Servo.D+=1;break;
              case 1:Servo.P+=1;Servo.D+=1;break;
              case 0:Servo.P+=1;Servo.D+=1;break;
              case -1:Servo.P+=1;Servo.D+=1;break;
              case -2:Servo.P+=1;Servo.D+=1;break;
              case -3:Servo.P+=1;Servo.D+=1;break;
              }
            }break;
          }
        }break;
      case 3:                                       //P=-1  D=3
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D+=2;break;
          case 2:Servo.P+=1;Servo.D+=2;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=1;Servo.D+=1;break;
              case 2:Servo.P+=1;Servo.D+=1;break;
              case 1:Servo.P+=1;Servo.D+=1;break;
              case 0:Servo.P+=1;Servo.D+=1;break;
              case -1:Servo.P+=1;Servo.D+=2;break;
              case -2:Servo.P+=1;Servo.D+=2;break;
              case -3:Servo.P+=1;Servo.D+=2;break;
              }
            }break;
          }
        }break;
      case 4:                                    //P=-1  D=4
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D+=2;break;
          case 2:Servo.P+=1;Servo.D+=2;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=1;Servo.D+=1;break;
              case 2:Servo.P+=1;Servo.D+=1;break;
              case 1:Servo.P+=1;Servo.D+=1;break;
              case 0:Servo.P+=1;Servo.D+=1;break;
              case -1:Servo.P+=1;Servo.D+=2;break;
              case -2:Servo.P+=1;Servo.D+=2;break;
              case -3:Servo.P+=1;Servo.D+=2;break;
              }
            }break;
          }
        }break;
      case 5:                                       //P=-1  D=5
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D+=3;break;
          case 2:Servo.P+=1;Servo.D+=3;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=1;Servo.D+=2;break;
              case 2:Servo.P+=1;Servo.D+=2;break;
              case 1:Servo.P+=1;Servo.D+=2;break;
              case 0:Servo.P+=1;Servo.D+=2;break;
              case -1:Servo.P+=1;Servo.D+=3;break;
              case -2:Servo.P+=1;Servo.D+=3;break;
              case -3:Servo.P+=1;Servo.D+=3;break;
              }
            }break;
          }
        }break;
      case 6:                                     //P=-1  D=6
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D+=2;break;
          case 2:Servo.P+=1;Servo.D+=3;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=1;Servo.D+=2;break;
              case 2:Servo.P+=1;Servo.D+=2;break;
              case 1:Servo.P+=1;Servo.D+=2;break;
              case 0:Servo.P+=1;Servo.D+=3;break;
              case -1:Servo.P+=1;Servo.D+=4;break;
              case -2:Servo.P+=1;Servo.D+=4;break;
              case -3:Servo.P+=1;Servo.D+=4;break;
              }
            }break;
          }
        }break;
      }
    }break;
  case 0:                                        //P=0  D=-6
    {
      switch(position_now_D)
      {
      case -6:
        {
          switch(track_form)
          {
          case 1:Servo.P+=0;Servo.D+=6;break;
          case 2:Servo.P+=0;Servo.D+=6;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=0;Servo.D+=6;break;
              case 2:Servo.P+=0;Servo.D+=6;break;
              case 1:Servo.P+=0;Servo.D+=6;break;
              case 0:Servo.P+=0;Servo.D+=5;break;
              case -1:Servo.P+=0;Servo.D+=5;break;
              case -2:Servo.P+=0;Servo.D+=5;break;
              case -3:Servo.P+=0;Servo.D+=5;break;
              }
            }break;
          }
        }break;
      case -5:                                   //P=0  D=-5
        {
          switch(track_form)
          {
          case 1:Servo.P+=0;Servo.D+=5;break;
          case 2:Servo.P+=0;Servo.D+=5;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=0;Servo.D+=5;break;
              case 2:Servo.P+=0;Servo.D+=5;break;
              case 1:Servo.P+=0;Servo.D+=5;break;
              case 0:Servo.P+=0;Servo.D+=4;break;
              case -1:Servo.P+=0;Servo.D+=4;break;
              case -2:Servo.P+=0;Servo.D+=4;break;
              case -3:Servo.P+=0;Servo.D+=4;break;
              }
            }break;
          }
        }break;
      case -4:                                       //P=0  D=-4
        {
          switch(track_form)
          {
          case 1:Servo.P+=0;Servo.D+=4;break;
          case 2:Servo.P+=0;Servo.D+=4;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=0;Servo.D+=4;break;
              case 2:Servo.P+=0;Servo.D+=4;break;
              case 1:Servo.P+=0;Servo.D+=4;break;
              case 0:Servo.P+=0;Servo.D+=3;break;
              case -1:Servo.P+=0;Servo.D+=3;break;
              case -2:Servo.P+=0;Servo.D+=3;break;
              case -3:Servo.P+=0;Servo.D+=3;break;
              }
            }break;
          }
        }break;
      case -3:                                     //P=0  D=-3
        {
          switch(track_form)
          {
          case 1:Servo.P+=0;Servo.D+=3;break;
          case 2:Servo.P+=0;Servo.D+=3;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=0;Servo.D+=3;break;
              case 2:Servo.P+=0;Servo.D+=3;break;
              case 1:Servo.P+=0;Servo.D+=3;break;
              case 0:Servo.P+=0;Servo.D+=2;break;
              case -1:Servo.P+=0;Servo.D+=2;break;
              case -2:Servo.P+=0;Servo.D+=2;break;
              case -3:Servo.P+=0;Servo.D+=2;break;
              }
            }break;
          }
        }break;
      case -2:                                    //P=0  D=-2
        {
          switch(track_form)
          {
          case 1:Servo.P+=0;Servo.D+=2;break;
          case 2:Servo.P+=0;Servo.D+=2;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=0;Servo.D+=2;break;
              case 2:Servo.P+=0;Servo.D+=2;break;
              case 1:Servo.P+=0;Servo.D+=2;break;
              case 0:Servo.P+=0;Servo.D+=2;break;
              case -1:Servo.P+=0;Servo.D+=2;break;
              case -2:Servo.P+=0;Servo.D+=2;break;
              case -3:Servo.P+=0;Servo.D+=2;break;
              }
            }break;
          }
        }break;
      case -1:                                //P=0  D=-1
        {
          switch(track_form)
          {
          case 1:Servo.P+=0;Servo.D+=1;break;
          case 2:Servo.P+=0;Servo.D+=1;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=0;Servo.D+=3;break;
              case 2:Servo.P+=0;Servo.D+=3;break;
              case 1:Servo.P+=0;Servo.D+=3;break;
              case 0:Servo.P+=0;Servo.D+=3;break;
              case -1:Servo.P+=0;Servo.D+=3;break;
              case -2:Servo.P+=0;Servo.D+=3;break;
              case -3:Servo.P+=0;Servo.D+=3;break;
              }
            }break;
          }
        }break;
      case 0:Servo.P+=6;break;                    //P=0  D=0
      case 1:                                      //P=0  D=1
        {
          switch(track_form)
          {
          case 1:Servo.P+=0;Servo.D+=1;break;
          case 2:Servo.P+=0;Servo.D+=1;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=0;Servo.D+=3;break;
              case 2:Servo.P+=0;Servo.D+=3;break;
              case 1:Servo.P+=0;Servo.D+=3;break;
              case 0:Servo.P+=0;Servo.D+=3;break;
              case -1:Servo.P+=0;Servo.D+=3;break;
              case -2:Servo.P+=0;Servo.D+=3;break;
              case -3:Servo.P+=0;Servo.D+=3;break;
              }
            }break;
          }
        }break;
      case 2:                                      //P=0  D=2
        {
          switch(track_form)
          {
          case 1:Servo.P+=0;Servo.D+=2;break;
          case 2:Servo.P+=0;Servo.D+=2;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=0;Servo.D+=2;break;
              case 2:Servo.P+=0;Servo.D+=2;break;
              case 1:Servo.P+=0;Servo.D+=2;break;
              case 0:Servo.P+=0;Servo.D+=2;break;
              case -1:Servo.P+=0;Servo.D+=2;break;
              case -2:Servo.P+=0;Servo.D+=2;break;
              case -3:Servo.P+=0;Servo.D+=2;break;
              }
            }break;
          }
        }break;
      case 3:                                  //P=0  D=3
        {
          switch(track_form)
          {
          case 1:Servo.P+=0;Servo.D+=3;break;
          case 2:Servo.P+=0;Servo.D+=3;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=0;Servo.D+=2;break;
              case 2:Servo.P+=0;Servo.D+=2;break;
              case 1:Servo.P+=0;Servo.D+=2;break;
              case 0:Servo.P+=0;Servo.D+=2;break;
              case -1:Servo.P+=0;Servo.D+=3;break;
              case -2:Servo.P+=0;Servo.D+=3;break;
              case -3:Servo.P+=0;Servo.D+=3;break;
              }
            }break;
          }
        }break;
      case 4:                                    //P=0  D=4
        {
          switch(track_form)
          {
          case 1:Servo.P+=0;Servo.D+=4;break;
          case 2:Servo.P+=0;Servo.D+=4;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=0;Servo.D+=3;break;
              case 2:Servo.P+=0;Servo.D+=3;break;
              case 1:Servo.P+=0;Servo.D+=3;break;
              case 0:Servo.P+=0;Servo.D+=3;break;
              case -1:Servo.P+=0;Servo.D+=4;break;
              case -2:Servo.P+=0;Servo.D+=4;break;
              case -3:Servo.P+=0;Servo.D+=4;break;
              }
            }break;
          }
        }break;
      case 5:                                      //P=0  D=5
        {
          switch(track_form)
          {
          case 1:Servo.P+=0;Servo.D+=5;break;
          case 2:Servo.P+=0;Servo.D+=5;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=0;Servo.D+=4;break;
              case 2:Servo.P+=0;Servo.D+=4;break;
              case 1:Servo.P+=0;Servo.D+=4;break;
              case 0:Servo.P+=0;Servo.D+=4;break;
              case -1:Servo.P+=0;Servo.D+=5;break;
              case -2:Servo.P+=0;Servo.D+=5;break;
              case -3:Servo.P+=0;Servo.D+=5;break;
              }
            }break;
          }
        }break;
      case 6:                                     //P=0  D=6
        {
          switch(track_form)
          {
          case 1:Servo.P+=0;Servo.D+=6;break;
          case 2:Servo.P+=0;Servo.D+=4;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=0;Servo.D+=5;break;
              case 2:Servo.P+=0;Servo.D+=5;break;
              case 1:Servo.P+=0;Servo.D+=5;break;
              case 0:Servo.P+=0;Servo.D+=5;break;
              case -1:Servo.P+=0;Servo.D+=6;break;
              case -2:Servo.P+=0;Servo.D+=6;break;
              case -3:Servo.P+=0;Servo.D+=6;break;
              }
            }break;
          }
        }break;
      }
    }break;
  case 1:                                       //P=1  D=6
    {
      switch(position_now_D)
      {
      case 6:
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D+=5;break;
          case 2:Servo.P+=1;Servo.D+=5;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=1;Servo.D+=1;break;
              case 2:Servo.P+=1;Servo.D+=2;break;
              case 1:Servo.P+=1;Servo.D+=2;break;
              case 0:Servo.P+=1;Servo.D+=3;break;
              case -1:Servo.P+=1;Servo.D+=3;break;
              case -2:Servo.P+=1;Servo.D+=3;break;
              case -3:Servo.P+=1;Servo.D+=3;break;
              }
            }break;
          }
        }break;
      case 5:                                     //P=1  D=5
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D+=4;break;
          case 2:Servo.P+=1;Servo.D+=4;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=1;Servo.D+=1;break;
              case 2:Servo.P+=1;Servo.D+=2;break;
              case 1:Servo.P+=1;Servo.D+=3;break;
              case 0:Servo.P+=1;Servo.D+=3;break;
              case -1:Servo.P+=1;Servo.D+=4;break;
              case -2:Servo.P+=1;Servo.D+=4;break;
              case -3:Servo.P+=1;Servo.D+=4;break;
              }
            }break;
          }
        }break;
      case 4:                                     //P=1  D=4                                  
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D+=3;break;
          case 2:Servo.P+=1;Servo.D+=3;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=1;Servo.D+=2;break;
              case 2:Servo.P+=1;Servo.D+=2;break;
              case 1:Servo.P+=1;Servo.D+=2;break;
              case 0:Servo.P+=1;Servo.D+=2;break;
              case -1:Servo.P+=1;Servo.D+=3;break;
              case -2:Servo.P+=1;Servo.D+=3;break;
              case -3:Servo.P+=1;Servo.D+=3;break;
              }
            }break;
          }
        }break;
      case 3:                                    //P=1  D=3
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D+=2;break;
          case 2:Servo.P+=1;Servo.D+=2;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=1;Servo.D+=1;break;
              case 2:Servo.P+=1;Servo.D+=1;break;
              case 1:Servo.P+=1;Servo.D+=1;break;
              case 0:Servo.P+=1;Servo.D+=1;break;
              case -1:Servo.P+=1;Servo.D+=2;break;
              case -2:Servo.P+=1;Servo.D+=2;break;
              case -3:Servo.P+=1;Servo.D+=2;break;
              }
            }break;
          }
        }break;
      case 2:                                    //P=1  D=2
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D+=1;break;
          case 2:Servo.P+=1;Servo.D+=1;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=1;Servo.D+=1;break;
              case 2:Servo.P+=1;Servo.D+=1;break;
              case 1:Servo.P+=1;Servo.D+=1;break;
              case 0:Servo.P+=1;Servo.D+=1;break;
              case -1:Servo.P+=1;Servo.D+=1;break;
              case -2:Servo.P+=1;Servo.D+=1;break;
              case -3:Servo.P+=1;Servo.D+=1;break;
              }
            }break;
          }
        }break;
      case 1:                                    //P=1  D=1
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D+=0;break;
          case 2:Servo.P+=1;Servo.D+=0;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=1;Servo.D+=0;break;
              case 2:Servo.P+=1;Servo.D+=0;break;
              case 1:Servo.P+=1;Servo.D+=0;break;
              case 0:Servo.P+=1;Servo.D+=0;break;
              case -1:Servo.P+=1;Servo.D+=0;break;
              case -2:Servo.P+=1;Servo.D+=0;break;
              case -3:Servo.P+=1;Servo.D+=0;break;
              }
            }break;
          }
        }break;
      case 0:Servo.P+=6;break;                                    //P=1  D=0
      case -1:                                    //P=1  D=-1
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D+=1;break;
          case 2:Servo.P+=1;Servo.D+=1;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=1;Servo.D+=1;break;
              case 2:Servo.P+=1;Servo.D+=1;break;
              case 1:Servo.P+=1;Servo.D+=1;break;
              case 0:Servo.P+=1;Servo.D+=1;break;
              case -1:Servo.P+=1;Servo.D+=1;break;
              case -2:Servo.P+=1;Servo.D+=1;break;
              case -3:Servo.P+=1;Servo.D+=1;break;
              }
            }break;
          }
        }break;
      case -2:                                    //P=1  D=-2
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D+=1;break;
          case 2:Servo.P+=1;Servo.D+=1;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=1;Servo.D+=1;break;
              case 2:Servo.P+=1;Servo.D+=1;break;
              case 1:Servo.P+=1;Servo.D+=1;break;
              case 0:Servo.P+=1;Servo.D+=1;break;
              case -1:Servo.P+=1;Servo.D+=1;break;
              case -2:Servo.P+=1;Servo.D+=1;break;
              case -3:Servo.P+=1;Servo.D+=1;break;
              }
            }break;
          }
        }break;
      case -3:                                    //P=1  D=-3
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D+=2;break;
          case 2:Servo.P+=1;Servo.D+=2;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=1;Servo.D+=2;break;
              case 2:Servo.P+=1;Servo.D+=2;break;
              case 1:Servo.P+=1;Servo.D+=2;break;
              case 0:Servo.P+=1;Servo.D+=1;break;
              case -1:Servo.P+=1;Servo.D+=1;break;
              case -2:Servo.P+=1;Servo.D+=1;break;
              case -3:Servo.P+=1;Servo.D+=1;break;
              }
            }break;
          }
        }break;
      case -4:                                    //P=1  D=-4
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D+=2;break;
          case 2:Servo.P+=1;Servo.D+=2;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=1;Servo.D+=2;break;
              case 2:Servo.P+=1;Servo.D+=2;break;
              case 1:Servo.P+=1;Servo.D+=2;break;
              case 0:Servo.P+=1;Servo.D+=1;break;
              case -1:Servo.P+=1;Servo.D+=1;break;
              case -2:Servo.P+=1;Servo.D+=1;break;
              case -3:Servo.P+=1;Servo.D+=1;break;
              }
            }break;
          }
        }break;
      case -5:                                    //P=1  D=-5
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D+=3;break;
          case 2:Servo.P+=1;Servo.D+=3;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=1;Servo.D+=3;break;
              case 2:Servo.P+=1;Servo.D+=3;break;
              case 1:Servo.P+=1;Servo.D+=3;break;
              case 0:Servo.P+=1;Servo.D+=2;break;
              case -1:Servo.P+=1;Servo.D+=2;break;
              case -2:Servo.P+=1;Servo.D+=2;break;
              case -3:Servo.P+=1;Servo.D+=2;break;
              }
            }break;
          }
        }break;
      case -6:                                    //P=1  D=-6
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D+=4;break;
          case 2:Servo.P+=1;Servo.D+=4;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=1;Servo.D+=4;break;
              case 2:Servo.P+=1;Servo.D+=4;break;
              case 1:Servo.P+=1;Servo.D+=4;break;
              case 0:Servo.P+=1;Servo.D+=3;break;
              case -1:Servo.P+=1;Servo.D+=3;break;
              case -2:Servo.P+=1;Servo.D+=3;break;
              case -3:Servo.P+=1;Servo.D+=3;break;
              }
            }break;
          }
        }break;
      }
    }break;
  case 2:                                    //P=2  
    {
      switch(position_now_D)
      {
      case 6:                                    //P=2  D=6
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D+=4;break;
          case 2:Servo.P+=1;Servo.D+=4;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=1;Servo.D+=3;break;
              case 2:Servo.P+=1;Servo.D+=3;break;
              case 1:Servo.P+=1;Servo.D+=3;break;
              case 0:Servo.P+=1;Servo.D+=3;break;
              case -1:Servo.P+=1;Servo.D+=4;break;
              case -2:Servo.P+=1;Servo.D+=4;break;
              case -3:Servo.P+=1;Servo.D+=4;break;
              }
            }break;
          }
        }break;
      case 5:                                    //P=2  D=5
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D+=3;break;
          case 2:Servo.P+=1;Servo.D+=3;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=1;Servo.D+=2;break;
              case 2:Servo.P+=1;Servo.D+=2;break;
              case 1:Servo.P+=1;Servo.D+=2;break;
              case 0:Servo.P+=1;Servo.D+=2;break;
              case -1:Servo.P+=1;Servo.D+=3;break;
              case -2:Servo.P+=1;Servo.D+=3;break;
              case -3:Servo.P+=1;Servo.D+=3;break;
              }
            }break;
          }
        }break;
      case 4:                                    //P=2  D=4
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D+=2;break;
          case 2:Servo.P+=1;Servo.D+=2;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=1;Servo.D+=1;break;
              case 2:Servo.P+=1;Servo.D+=1;break;
              case 1:Servo.P+=1;Servo.D+=1;break;
              case 0:Servo.P+=1;Servo.D+=1;break;
              case -1:Servo.P+=1;Servo.D+=2;break;
              case -2:Servo.P+=1;Servo.D+=2;break;
              case -3:Servo.P+=1;Servo.D+=2;break;
              }
            }break;
          }
        }break;
      case 3:                                    //P=2  D=3
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D+=1;break;
          case 2:Servo.P+=1;Servo.D+=1;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=1;Servo.D+=1;break;
              case 2:Servo.P+=1;Servo.D+=1;break;
              case 1:Servo.P+=1;Servo.D+=1;break;
              case 0:Servo.P+=1;Servo.D+=1;break;
              case -1:Servo.P+=1;Servo.D+=2;break;
              case -2:Servo.P+=1;Servo.D+=2;break;
              case -3:Servo.P+=1;Servo.D+=2;break;
              }
            }break;
          }
        }break;
      case 2:                                    //P=2  D=2
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D-=0;break;
          case 2:Servo.P+=1;Servo.D-=0;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=1;Servo.D+=0;break;
              case 2:Servo.P+=1;Servo.D+=0;break;
              case 1:Servo.P+=1;Servo.D+=0;break;
              case 0:Servo.P+=1;Servo.D+=0;break;
              case -1:Servo.P+=1;Servo.D+=0;break;
              case -2:Servo.P+=1;Servo.D+=0;break;
              case -3:Servo.P+=1;Servo.D+=0;break;
              }
            }break;
          }
        }break;
      case 1:                                    //P=2  D=1
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D-=1;break;
          case 2:Servo.P+=1;Servo.D-=1;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=1;Servo.D-=1;break;
              case 2:Servo.P+=1;Servo.D-=1;break;
              case 1:Servo.P+=1;Servo.D-=1;break;
              case 0:Servo.P+=1;Servo.D-=1;break;
              case -1:Servo.P+=1;Servo.D-=1;break;
              case -2:Servo.P+=1;Servo.D-=1;break;
              case -3:Servo.P+=1;Servo.D-=1;break;
              }
            }break;
          }
        }break;
      case 0:Servo.P+=1;break;                                    //P=2  D=0
      case -1:                                    //P=2  D=-1
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D+=1;break;
          case 2:Servo.P+=1;Servo.D+=1;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=1;Servo.D+=1;break;
              case 2:Servo.P+=1;Servo.D+=1;break;
              case 1:Servo.P+=1;Servo.D+=1;break;
              case 0:Servo.P+=1;Servo.D+=1;break;
              case -1:Servo.P+=1;Servo.D+=1;break;
              case -2:Servo.P+=1;Servo.D+=1;break;
              case -3:Servo.P+=1;Servo.D+=1;break;
              }
            }break;
          }
        }break;
      case -2:                                    //P=2  D=-2
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D+=1;break;
          case 2:Servo.P+=1;Servo.D+=1;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=1;Servo.D+=1;break;
              case 2:Servo.P+=1;Servo.D+=1;break;
              case 1:Servo.P+=1;Servo.D+=1;break;
              case 0:Servo.P+=1;Servo.D+=1;break;
              case -1:Servo.P+=1;Servo.D+=1;break;
              case -2:Servo.P+=1;Servo.D+=1;break;
              case -3:Servo.P+=1;Servo.D+=1;break;
              }
            }break;
          }
        }break;
      case -3:                                    //P=2  D=-3
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D+=2;break;
          case 2:Servo.P+=1;Servo.D+=2;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=1;Servo.D+=2;break;
              case 2:Servo.P+=1;Servo.D+=2;break;
              case 1:Servo.P+=1;Servo.D+=2;break;
              case 0:Servo.P+=1;Servo.D+=1;break;
              case -1:Servo.P+=1;Servo.D+=1;break;
              case -2:Servo.P+=1;Servo.D+=1;break;
              case -3:Servo.P+=1;Servo.D+=1;break;
              }
            }break;
          }
        }break;
      case -4:                                    //P=2  D=-4
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D+=2;break;
          case 2:Servo.P+=1;Servo.D+=2;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=1;Servo.D+=2;break;
              case 2:Servo.P+=1;Servo.D+=2;break;
              case 1:Servo.P+=1;Servo.D+=2;break;
              case 0:Servo.P+=1;Servo.D+=1;break;
              case -1:Servo.P+=1;Servo.D+=1;break;
              case -2:Servo.P+=1;Servo.D+=1;break;
              case -3:Servo.P+=1;Servo.D+=1;break;
              }
            }break;
          }
        }break;
      case -5:                                    //P=2  D=-5
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D+=3;break;
          case 2:Servo.P+=1;Servo.D+=3;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=1;Servo.D+=3;break;
              case 2:Servo.P+=1;Servo.D+=3;break;
              case 1:Servo.P+=1;Servo.D+=3;break;
              case 0:Servo.P+=1;Servo.D+=2;break;
              case -1:Servo.P+=1;Servo.D+=2;break;
              case -2:Servo.P+=1;Servo.D+=2;break;
              case -3:Servo.P+=1;Servo.D+=2;break;
              }
            }break;
          }
        }break;
      case -6:                                    //P=2  D=-6
        {
          switch(track_form)
          {
          case 1:Servo.P+=1;Servo.D+=4;break;
          case 2:Servo.P+=1;Servo.D+=4;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=1;Servo.D+=4;break;
              case 2:Servo.P+=1;Servo.D+=4;break;
              case 1:Servo.P+=1;Servo.D+=4;break;
              case 0:Servo.P+=1;Servo.D+=4;break;
              case -1:Servo.P+=1;Servo.D+=3;break;
              case -2:Servo.P+=1;Servo.D+=3;break;
              case -3:Servo.P+=1;Servo.D+=3;break;
              }
            }break;
          }
        }break;
      }
    }break;
  case 3:                                    //P=3  D=6
    {
      switch(position_now_D)
      {
      case 6:
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D+=1;break;
          case 2:Servo.P+=2;Servo.D+=1;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=2;Servo.D+=0;break;
              case 2:Servo.P+=2;Servo.D+=0;break;
              case 1:Servo.P+=2;Servo.D+=0;break;
              case 0:Servo.P+=2;Servo.D+=0;break;
              case -1:Servo.P+=2;Servo.D+=1;break;
              case -2:Servo.P+=2;Servo.D+=1;break;
              case -3:Servo.P+=2;Servo.D+=1;break;
              }
            }break;
          }
        }break;
      case 5:                                   //P=3  D=5
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D+=0;break;
          case 2:Servo.P+=2;Servo.D+=0;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=2;Servo.D+=0;break;
              case 2:Servo.P+=2;Servo.D+=0;break;
              case 1:Servo.P+=2;Servo.D+=0;break;
              case 0:Servo.P+=2;Servo.D+=0;break;
              case -1:Servo.P+=2;Servo.D+=1;break;
              case -2:Servo.P+=2;Servo.D+=1;break;
              case -3:Servo.P+=2;Servo.D+=1;break;
              }
            }break;
          }
        }break;
      case 4:                                   //P=3  D=4
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D-=1;break;
          case 2:Servo.P+=2;Servo.D-=1;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=2;Servo.D-=1;break;
              case 2:Servo.P+=2;Servo.D-=1;break;
              case 1:Servo.P+=2;Servo.D-=1;break;
              case 0:Servo.P+=2;Servo.D-=1;break;
              case -1:Servo.P+=2;Servo.D-=0;break;
              case -2:Servo.P+=2;Servo.D-=0;break;
              case -3:Servo.P+=2;Servo.D-=0;break;
              }
            }break;
          }
        }break;
      case 3:                                   //P=3  D=3
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D-=2;break;
          case 2:Servo.P+=2;Servo.D-=2;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=2;Servo.D-=2;break;
              case 2:Servo.P+=2;Servo.D-=2;break;
              case 1:Servo.P+=2;Servo.D-=2;break;
              case 0:Servo.P+=2;Servo.D-=2;break;
              case -1:Servo.P+=2;Servo.D-=1;break;
              case -2:Servo.P+=2;Servo.D-=1;break;
              case -3:Servo.P+=2;Servo.D-=1;break;
              }
            }break;
          }
        }break;
      case 2:                                   //P=3  D=2
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D-=3;break;
          case 2:Servo.P+=2;Servo.D-=3;break;
          case 3:Servo.P+=2;Servo.D-=3;break;
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=2;Servo.D-=3;break;
              case 2:Servo.P+=2;Servo.D-=3;break;
              case 1:Servo.P+=2;Servo.D-=3;break;
              case 0:Servo.P+=2;Servo.D-=3;break;
              case -1:Servo.P+=2;Servo.D-=3;break;
              case -2:Servo.P+=2;Servo.D-=3;break;
              case -3:Servo.P+=2;Servo.D-=3;break;
              }
            }break;
          }
        }break;
      case 1:                                   //P=3  D=1
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D-=4;break;
          case 2:Servo.P+=2;Servo.D-=4;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=2;Servo.D-=4;break;
              case 2:Servo.P+=2;Servo.D-=4;break;
              case 1:Servo.P+=2;Servo.D-=4;break;
              case 0:Servo.P+=2;Servo.D-=4;break;
              case -1:Servo.P+=2;Servo.D-=4;break;
              case -2:Servo.P+=2;Servo.D-=4;break;
              case -3:Servo.P+=2;Servo.D-=4;break;
              }
            }break;
          }
        }break;
      case 0:Servo.P+=2;break;                                   //P=3  D=0
      case -1:                                   //P=3  D=-1
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D+=1;break;
          case 2:Servo.P+=2;Servo.D+=1;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=2;Servo.D+=1;break;
              case 2:Servo.P+=2;Servo.D+=1;break;
              case 1:Servo.P+=2;Servo.D+=1;break;
              case 0:Servo.P+=2;Servo.D+=1;break;
              case -1:Servo.P+=2;Servo.D+=1;break;
              case -2:Servo.P+=2;Servo.D+=1;break;
              case -3:Servo.P+=2;Servo.D+=1;break;
              }
            }break;
          }
        }break;
      case -2:                                   //P=3  D=-2
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D+=1;break;
          case 2:Servo.P+=2;Servo.D+=1;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=2;Servo.D+=1;break;
              case 2:Servo.P+=2;Servo.D+=1;break;
              case 1:Servo.P+=2;Servo.D+=1;break;
              case 0:Servo.P+=2;Servo.D+=1;break;
              case -1:Servo.P+=2;Servo.D+=1;break;
              case -2:Servo.P+=2;Servo.D+=1;break;
              case -3:Servo.P+=2;Servo.D+=1;break;
              }
            }break;
          }
        }break;
      case -3:                                   //P=3  D=-3
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D+=2;break;
          case 2:Servo.P+=2;Servo.D+=2;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=2;Servo.D+=2;break;
              case 2:Servo.P+=2;Servo.D+=2;break;
              case 1:Servo.P+=2;Servo.D+=2;break;
              case 0:Servo.P+=2;Servo.D+=1;break;
              case -1:Servo.P+=2;Servo.D+=1;break;
              case -2:Servo.P+=2;Servo.D+=1;break;
              case -3:Servo.P+=2;Servo.D+=1;break;
              }
            }break;
          }
        }break;
      case -4:                                   //P=3  D=-4
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D+=2;break;
          case 2:Servo.P+=2;Servo.D+=2;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=2;Servo.D+=2;break;
              case 2:Servo.P+=2;Servo.D+=2;break;
              case 1:Servo.P+=2;Servo.D+=2;break;
              case 0:Servo.P+=2;Servo.D+=1;break;
              case -1:Servo.P+=2;Servo.D+=1;break;
              case -2:Servo.P+=2;Servo.D+=1;break;
              case -3:Servo.P+=2;Servo.D+=1;break;
              }
            }break;
          }
        }break;
      case -5:                                   //P=3  D=-5
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D+=3;break;
          case 2:Servo.P+=2;Servo.D+=3;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=2;Servo.D+=3;break;
              case 2:Servo.P+=2;Servo.D+=3;break;
              case 1:Servo.P+=2;Servo.D+=3;break;
              case 0:Servo.P+=2;Servo.D+=2;break;
              case -1:Servo.P+=2;Servo.D+=2;break;
              case -2:Servo.P+=2;Servo.D+=2;break;
              case -3:Servo.P+=2;Servo.D+=2;break;
              }
            }break;
          }
        }break;
      case -6:                                   //P=3  D=-6
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D+=4;break;
          case 2:Servo.P+=2;Servo.D+=4;break;
          case 3:
            {
              switch(position_now_low_P)
              {
              case 3:Servo.P+=2;Servo.D+=4;break;
              case 2:Servo.P+=2;Servo.D+=4;break;
              case 1:Servo.P+=2;Servo.D+=4;break;
              case 0:Servo.P+=2;Servo.D+=3;break;
              case -1:Servo.P+=2;Servo.D+=3;break;
              case -2:Servo.P+=2;Servo.D+=3;break;
              case -3:Servo.P+=2;Servo.D+=3;break;
              }
            }break;
          }
        }break;
      }
    }break;
  case 4:                                   //P=4  D=6
    {
      switch(position_now_D)
      {
      case 6:
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D+=3;break;
          case 2:Servo.P+=2;Servo.D+=3;break;
          case 3:Servo.P+=2;Servo.D+=3;break;
          }
        }break;
      case 5:                                   //P=4  D=5
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D+=2;break;
          case 2:Servo.P+=2;Servo.D+=2;break;
          case 3:Servo.P+=2;Servo.D+=2;break;
          }
        }break;
      case 4:                                   //P=4  D=4
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D+=1;break;
          case 2:Servo.P+=2;Servo.D+=1;break;
          case 3:Servo.P+=2;Servo.D+=1;break;
          }
        }break;
      case 3:                                   //P=4  D=3
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D-=0;break;
          case 2:Servo.P+=2;Servo.D-=0;break;
          case 3:Servo.P+=2;Servo.D+=0;break;
          }
        }break;
      case 2:                                   //P=4  D=2
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D-=1;break;
          case 2:Servo.P+=2;Servo.D-=1;break;
          case 3:Servo.P+=2;Servo.D-=1;break;
          }
        }break;
      case 1:                                   //P=4  D=1
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D-=2;break;
          case 2:Servo.P+=2;Servo.D-=2;break;
          case 3:Servo.P+=2;Servo.D-=2;break;
          }
        }break;
      case 0:Servo.P+=2;break;                                   //P=4  D=0
      case -1:                                   //P=4  D=-1
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D+=1;break;
          case 2:Servo.P+=2;Servo.D+=1;break;
          case 3:Servo.P+=2;Servo.D+=1;break;
          }
        }break;
      case -2:                                   //P=4  D=-2
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D+=1;break;
          case 2:Servo.P+=2;Servo.D+=1;break;
          case 3:Servo.P+=2;Servo.D+=1;break;
          }
        }break;
      case -3:                                   //P=4  D=-3
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D+=2;break;
          case 2:Servo.P+=2;Servo.D+=2;break;
          case 3:Servo.P+=2;Servo.D+=2;break;
          }
        }break;
      case -4:                                   //P=4  D=-4
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D+=2;break;
          case 2:Servo.P+=2;Servo.D+=2;break;
          case 3:Servo.P+=2;Servo.D+=2;break;
          }
        }break;
      case -5:                                   //P=4  D=-5
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D+=3;break;
          case 2:Servo.P+=2;Servo.D+=3;break;
          case 3:Servo.P+=2;Servo.D+=3;break;
          }
        }break;
      case -6:                                   //P=4  D=-6
        {
          switch(track_form)
          {
          case 1:Servo.P+=2;Servo.D+=4;break;
          case 2:Servo.P+=2;Servo.D+=4;break;
          case 3:Servo.P+=2;Servo.D+=4;break;
          }
        }break;
      }
    }break;
  case 5:                                   //P=5  D=6
    {
      switch(position_now_D)
      {
      case 6:
        {
          switch(track_form)
          {
          case 1:Servo.P+=4;Servo.D+=1;break;
          case 2:Servo.P+=4;Servo.D+=1;break;
          case 3:Servo.P+=4;Servo.D+=1;break;
          }
        }break;
      case 5:                                   //P=5  D=5
        {
          switch(track_form)
          {
          case 1:Servo.P+=4;Servo.D+=0;break;
          case 2:Servo.P+=4;Servo.D+=0;break;
          case 3:Servo.P+=4;Servo.D+=0;break;
          }
        }break;
      case 4:                                   //P=5  D=4
        {
          switch(track_form)
          {
          case 1:Servo.P+=4;Servo.D-=1;break;
          case 2:Servo.P+=4;Servo.D-=1;break;
          case 3:Servo.P+=4;Servo.D-=1;break;
          }
        }break;
      case 3:                                   //P=5  D=3
        {
          switch(track_form)
          {
          case 1:Servo.P+=4;Servo.D-=2;break;
          case 2:Servo.P+=4;Servo.D-=2;break;
          case 3:Servo.P+=4;Servo.D-=2;break;
          }
        }break;
      case 2:                                   //P=5  D=2
        {
          switch(track_form)
          {
          case 1:Servo.P+=4;Servo.D-=3;break;
          case 2:Servo.P+=4;Servo.D-=3;break;
          case 3:Servo.P+=4;Servo.D-=3;break;
          }
        }break;
      case 1:                                   //P=5  D=1
        {
          switch(track_form)
          {
          case 1:Servo.P+=4;Servo.D-=4;break;
          case 2:Servo.P+=4;Servo.D-=4;break;
          case 3:Servo.P+=4;Servo.D-=4;break;
          }
        }break;
      case 0:Servo.P+=4;break;                                   //P=5  D=0
      case -1:                                   //P=5  D=-1
        {
          switch(track_form)
          {
          case 1:Servo.P+=4;Servo.D+=1;break;
          case 2:Servo.P+=4;Servo.D+=1;break;
          case 3:Servo.P+=4;Servo.D+=1;break;
          }
        }break;
      case -2:                                   //P=5  D=-2
        {
          switch(track_form)
          {
          case 1:Servo.P+=4;Servo.D+=1;break;
          case 2:Servo.P+=4;Servo.D+=1;break;
          case 3:Servo.P+=4;Servo.D+=1;break;
          }
        }break;
      case -3:                                   //P=5  D=-3
        {
          switch(track_form)
          {
          case 1:Servo.P+=4;Servo.D+=2;break;
          case 2:Servo.P+=4;Servo.D+=2;break;
          case 3:Servo.P+=4;Servo.D+=2;break;
          }
        }break;
      case -4:                                   //P=5  D=-4
        {
          switch(track_form)
          {
          case 1:Servo.P+=4;Servo.D+=2;break;
          case 2:Servo.P+=4;Servo.D+=2;break;
          case 3:Servo.P+=4;Servo.D+=2;break;
          }
        }break;
      case -5:                                   //P=5  D=-5
        {
          switch(track_form)
          {
          case 1:Servo.P+=4;Servo.D+=3;break;
          case 2:Servo.P+=4;Servo.D+=3;break;
          case 3:Servo.P+=4;Servo.D+=3;break;
          }
        }break;
      case -6:                                   //P=5  D=-6
        {
          switch(track_form)
          {
          case 1:Servo.P+=4;Servo.D+=4;break;
          case 2:Servo.P+=4;Servo.D+=4;break;
          case 3:Servo.P+=4;Servo.D+=4;break;
          }
        }break;
      }
    }break;
  case 6:                                   //P=6  D=6
    {
      switch(position_now_D)
      {
      case 6:
        {
          switch(track_form)
          {
          case 1:Servo.P+=5;Servo.D+=0;break;
          case 2:Servo.P+=5;Servo.D+=0;break;
          case 3:Servo.P+=5;Servo.D+=0;break;
          }
        }break;
      case 5:                                  //P=6  D=5
        {
          switch(track_form)
          {
          case 1:Servo.P+=5;Servo.D+=0;break;
          case 2:Servo.P+=5;Servo.D+=0;break;
          case 3:Servo.P+=5;Servo.D+=0;break;
          }
        }break;
      case 4:                                  //P=6  D=4
        {
          switch(track_form)
          {
          case 1:Servo.P+=5;Servo.D-=1;break;
          case 2:Servo.P+=5;Servo.D-=1;break;
          case 3:Servo.P+=5;Servo.D-=1;break;
          }
        }break;
      case 3:                                  //P=6  D=3
        {
          switch(track_form)
          {
          case 1:Servo.P+=5;Servo.D-=2;break;
          case 2:Servo.P+=5;Servo.D-=2;break;
          case 3:Servo.P+=5;Servo.D-=2;break;
          }
        }break;
      case 2:                                  //P=6  D=2
        {
          switch(track_form)
          {
          case 1:Servo.P+=5;Servo.D-=3;break;
          case 2:Servo.P+=5;Servo.D-=3;break;
          case 3:Servo.P+=5;Servo.D-=3;break;
          }
        }break;
      case 1:                                  //P=6  D=1
        {
          switch(track_form)
          {
          case 1:Servo.P+=5;Servo.D-=4;break;
          case 2:Servo.P+=5;Servo.D-=4;break;
          case 3:Servo.P+=5;Servo.D-=4;break;
          }
        }break;
      case 0:Servo.P+=5;break;                                  //P=6  D=0
      case -1:                                  //P=6  D=-1
        {
          switch(track_form)
          {
          case 1:Servo.P+=5;Servo.D+=1;break;
          case 2:Servo.P+=5;Servo.D+=1;break;
          case 3:Servo.P+=5;Servo.D+=1;break;
          }
        }break;
      case -2:                                  //P=6  D=-2
        {
          switch(track_form)
          {
          case 1:Servo.P+=5;Servo.D+=1;break;
          case 2:Servo.P+=5;Servo.D+=1;break;
          case 3:Servo.P+=5;Servo.D+=1;break;
          }
        }break;
      case -3:                                  //P=6  D=-3
        {
          switch(track_form)
          {
          case 1:Servo.P+=5;Servo.D+=2;break;
          case 2:Servo.P+=5;Servo.D+=2;break;
          case 3:Servo.P+=5;Servo.D+=2;break;
          }
        }break;
      case -4:                                  //P=6  D=-4
        {
          switch(track_form)
          {
          case 1:Servo.P+=5;Servo.D+=2;break;
          case 2:Servo.P+=5;Servo.D+=2;break;
          case 3:Servo.P+=5;Servo.D+=2;break;
          }
        }break;
      case -5:                                  //P=6  D=-5
        {
          switch(track_form)
          {
          case 1:Servo.P+=5;Servo.D+=3;break;
          case 2:Servo.P+=5;Servo.D+=3;break;
          case 3:Servo.P+=5;Servo.D+=3;break;
          }
        }break;
      case -6:                                  //P=6  D=-6
        {
          switch(track_form)
          {
          case 1:Servo.P+=5;Servo.D+=4;break;
          case 2:Servo.P+=5;Servo.D+=4;break;
          case 3:Servo.P+=5;Servo.D+=4;break;
          }
        }break;
      }
    }break;
  }
 // if(zhidao_flag){
  //  Servo.P-=2;
  //  Servo.D-=2;
  //}
  xiaoS_flag=judge_small_S();
  
  if(sai_dao_lei_xing==20&&!xiaoS_flag)
  {
//    num=0;
//  position_now=0;
//   for(i=20;i<25;i++)
//  {
//    if(center[i])
//    {
//      num++;
//      position_now+=center[i];
//    }  
//  }
//  position_old_old_old=position_old_old;
//  position_old_old=position_old;
//  position_old=position_now;
//  if(num)
//  position_now=position_now/num;   
//  num=0;
  
    position_diff_zhi=position_old-position_now;
    Servo_value=Servo_Middle-Servo_P_zhidao*(76-position_now)-Servo_D_zhidao*position_diff_zhi;//PD控制舵机
         gpio_init(PORTA,14,1,0);
        gpio_init(PORTA,15,1,0);
        gpio_init(PORTA,16,1,0);
        gpio_init(PORTA,17,1,0);
  }else{
         gpio_init(PORTA,14,1,1);
        gpio_init(PORTA,15,1,1);
        gpio_init(PORTA,16,1,1);
        gpio_init(PORTA,17,1,1);
  Servo_value=Servo_Middle-Servo.P*(77-position_now)-Servo.D*position_diff;//PD控制舵机
    //Servo_value=Servo_Middle-Servo.P*(77-position_now)-(Servo.D*position_diff*7+servo_d*(weizhi_old-weizhi)*3)/10;//PD控制舵机
  }
  Servo_value=limit(Servo_value,Servo_Right,Servo_Left);
  
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
  for(i=54;i>21;i--)      //前段   59~41  33
    if(center[i]){
      low_0+=center[i];
        num++;
    }
  if(low_0&&num>2)
    low_0=low_0/num;
  num=0;
  
    
  for(;i>9;i--)    //中段  40~26    20  15
    if(center[i]){
      mid_0+=center[i];
      num++;
      //j=i;
    }
  if(mid_0&&num>2)
    mid_0=mid_0/num;
  num=0;    
   
  if(abs(low_0-79)<8&&abs(mid_0-79)<8&&abs(high_0-79)<8){
    zhidao_flag=1;
  }else{
   zhidao_flag=0; 
  }
      
  for(;i>=0;i--)         //远段    25~0
    if(center[i]){
      high_0+=center[i];
      num++;
    }
  if(high_0&&num>2)
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
         if(high_now_0>62&&high_now_0<92&&mid_now_0>61&&mid_now_0<93&&low_now_0>60&&low_now_0<94)
         {
           sai_dao_lei_xing_0=20;
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



/////////////////坡道识别/////////////////
int16 Judge_slope(void){
    unsigned char  i,j;
	unsigned char slope_num=0;
	int16 track_width=0,slope_flag=0;
    for(i=3;i<=23;i++){
		if(bianyan[0][i]&&bianyan[1][i]!=159){
		track_width+=bianyan[1][i]-bianyan[0][i];
			slope_num++;
		}
	}
	if(slope_num>17){
      track_width=track_width/slope_num;
	}else{
		track_width=0;
	}
	//str.Format("track_width：%d\r\n",track_width);
	   //PrintDebug(str);
	if(track_width>67&&!high_0&&!mid_0&&!low_0){
		slope_flag=1;
//                gpio_init(PORTA,14,1,0);
//        gpio_init(PORTA,15,1,0);
//        gpio_init(PORTA,16,1,0);
//        gpio_init(PORTA,17,1,0);
	}else{
		slope_flag=0;
	}
    return slope_flag;
}

/***************很歪的十字交叉连接判断***************/
uint8 judage_shizi(void)
{
  uint8 i=54,j_k=0,m=0;
  
  if(mid_0==1)
  {
    for(i=54;i>10;i--)
    {
      if(center[i]>center[i-1]&&center[i]>center[i-2]&&center[i]>center[i-3])
      {
        j_k=i;
        break;
      }
    }
    
  }
  else if(mid_0==-1)
  {
    for(i=54;i>10;i--)
    {
      if(center[i]<center[i-1]&&center[i]<center[i-2]&&center[i]<center[i-3])
      {
        j_k=i;
        break;
      }
    }
    
  }
 
  return j_k;
}
/***************很歪的十字交叉连接判断END***************/



int16 speed_control(void)
{
  volatile int16 i=0;//,E=0,Ec=0;
  volatile int32 exspeed=0;
  volatile struct membership_grade S; 
 // static 
    //int16 i_old=0;
  
  
  while(!center[i]&&i<55) i++;   //50
  
      
  E=54-i;   //i无效点个数    //49    E为有效的个数
  Ec_old_old=Ec_old;
  Ec_old=Ec;
  Ec=i-i_old; //有效点变化率
  i_old=i;
  
  if(i<=5&&Ec>=0&&Ec_old>=0) return speed_top;    //直道全速冲刺  3
  if(i>=45)  return speed_ms;
    
  if(i>=0)
  {
    if(!Ec)//变化率为0的时候
    {
      //sub_e(E,25,20,50,45);//20，15，45，40通过采集到的点数来算出，speed_s,speed_m,speed_b所占的份额
  //  sub_e(E,24,0,54,24);       //主要决定十字弯道和直到的过弯速度
     // sub_e(E,35,30,50,45);
      sub_e(E,30,25,45,40);
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
    
     
      
int16 Motor_control(int16 shift){
  volatile int32 speed=0;
  volatile int16 motor_e=0,motor_ec=0,absa=0;
  static int16 motor_e_old=0,motor_sum=0;
   
  motor_e=shift-pulse;             //speed_vari为编码器返回的脉冲数
  speed_now=shift;
  motor_ec=motor_e-motor_e_old;
  absa=abs(motor_e);
  //uart_send1(UART0,shift);
   //uart_send1(UART0,pulse);
  // uart_send1(UART0,absa);
  motor_fuzzy(motor_e,motor_ec,16,8,24,20,8,4,16,12);//30  20  50  40  15  10  25  20
  motor_sum=line(motor_e,Motor.I);
  speed=(long)s_old*motor_p+Motor.P*(long)motor_e+motor_p*(long)motor_sum/2;
  speed=speed/100;
  s_old=pulse;
  motor_e_old=motor_e;
  //if(pulse<50&&speed<0) return 50;
  return speed;            
}   

/*****************非动态阈值二值化*****************/
void threshold(void)
{
  uint8 i=0,j=0;
//  uint8 black_a1 = 140;
//  uint8 black_a2 = 145;
//  uint8 black_a3 = 140;
//  uint8 black_b1 = 135;
//  uint8 black_b2 = 140;
//  uint8 black_b3 = 135;
//  uint8 black_c1 = 132;
//  uint8 black_c2 = 138;
//  uint8 black_c3 = 132;
//  uint8 black_d1 = 130;
//  uint8 black_d2 = 142;
//  uint8 black_d3 = 130;
  
  uint8 black_a1 = 135;//140
  uint8 black_a2 = 145;//145
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
  for(i=0;i<H;i++)
  {
    if(i<8)
    {
      for(j=0;j<V;j++)
      {
        if(j<50)
        {
          if(video[i][j]<black_d1)video[i][j]=0;
          else video[i][j]=1;
        }
        else if(j<110)
        {
          if(video[i][j]<black_d2)video[i][j]=0;
          else video[i][j]=1;
        }
        else
        {
          if(video[i][j]<black_d3)video[i][j]=0;
          else video[i][j]=1;
        }
      }
    }
    else if(i<20)
    {
      for(j=0;j<V;j++)
      {
        if(j<50)
        {
          if(video[i][j]<black_c1)video[i][j]=0;
          else video[i][j]=1;
        }
        else if(j<110)
        {
          if(video[i][j]<black_c2)video[i][j]=0;
          else video[i][j]=1;
        }
        else
        {
          if(video[i][j]<black_c3)video[i][j]=0;
          else video[i][j]=1;
        }
      }
    }
    else if(i<34)
    {
      for(j=0;j<V;j++)
      {
        if(j<50)
        {
          if(video[i][j]<black_b1)video[i][j]=0;
          else video[i][j]=1;
        }
        else if(j<110)
        {
          if(video[i][j]<black_b2)video[i][j]=0;
          else video[i][j]=1;
        }
        else
        {
          if(video[i][j]<black_b3)video[i][j]=0;
          else video[i][j]=1;
        }
      }
    }
    else
    {
      for(j=0;j<V;j++)
      {
        if(j<50)
        {
          if(video[i][j]<black_a1)video[i][j]=0;
          else video[i][j]=1;
        }
        else if(j<110)
        {
          if(video[i][j]<black_a2)video[i][j]=0;
          else video[i][j]=1;
        }
        else
        {
          if(video[i][j]<black_a3)video[i][j]=0;
          else video[i][j]=1;
        }
      }
    }
  }
}
/**********************非动态阈值END*********************/
/**********************动态阈值***********************/
void Yu_Zhi(unsigned char x1, unsigned char x2,unsigned char y)
{
	unsigned char i=0,black_number=0,white_number=0;
	int16 black_numbers=0,white_numbers=0;
	if(y<10)
	{
		if(x1==0)
		{
		    for(i=x1;i<x2;i++)
			{
			    if(video[y][i]<=16);
			    else if(video[y][i]<black_max_a1){black_numbers=black_numbers+video[y][i];black_number++;}
                else {white_numbers=white_numbers+video[y][i];white_number++;}
			}
		    if(black_number)black_numbers=black_numbers/black_number;
		    if(white_number)white_numbers=white_numbers/white_number;
		    if(white_numbers&&black_numbers)black_max_1=(black_numbers+white_numbers)/2;
		    if(!(white_numbers&&black_numbers))black_max_1=black_max_a1;
		}
		else if(x1==40)
		{
			for(i=x1;i<x2;i++)
			{
			    if(video[y][i]<=16);
			    else if(video[y][i]<black_max_a2){black_numbers=black_numbers+video[y][i];black_number++;}
                else {white_numbers=white_numbers+video[y][i];white_number++;}
			}
		    if(black_number)black_numbers=black_numbers/black_number;
		    if(white_number)white_numbers=white_numbers/white_number;
		    if(white_numbers&&black_numbers)black_max_2=(black_numbers+white_numbers)/2;
            if(!(white_numbers&&black_numbers))black_max_2=black_max_a2;
		}
		else
		{
			for(i=x1;i<x2;i++)
			{
			    if(video[y][i]<=16);
			    else if(video[y][i]<black_max_a3){black_numbers=black_numbers+video[y][i];black_number++;}
                else {white_numbers=white_numbers+video[y][i];white_number++;}
			}
		    if(black_number)black_numbers=black_numbers/black_number;
		    if(white_number)white_numbers=white_numbers/white_number;
		    if(white_numbers&&black_numbers)black_max_3=(black_numbers+white_numbers)/2;
		    if(!(white_numbers&&black_numbers))black_max_3=black_max_a3;
		}
	}
	else if(y<20)
	{
		if(x1==0)
		{
		    for(i=x1;i<x2;i++)
			{
			    if(video[y][i]<=16);
			    else if(video[y][i]<black_max_b1){black_numbers=black_numbers+video[y][i];black_number++;}
                else {white_numbers=white_numbers+video[y][i];white_number++;}
			}
		    if(black_number)black_numbers=black_numbers/black_number;
		    if(white_number)white_numbers=white_numbers/white_number;
		    if(white_numbers&&black_numbers)black_max_1=(black_numbers+white_numbers)/2;
		    if(!(white_numbers&&black_numbers))black_max_1=black_max_b1;
		}
		else if(x1==40)
		{
			for(i=x1;i<x2;i++)
			{
			    if(video[y][i]<=16);
			    else if(video[y][i]<black_max_b2){black_numbers=black_numbers+video[y][i];black_number++;}
                else {white_numbers=white_numbers+video[y][i];white_number++;}
			}
		    if(black_number)black_numbers=black_numbers/black_number;
		    if(white_number)white_numbers=white_numbers/white_number;
		    if(white_numbers&&black_numbers)black_max_2=(black_numbers+white_numbers)/2;
            if(!(white_numbers&&black_numbers))black_max_2=black_max_b2;
		}
		else
		{
			for(i=x1;i<x2;i++)
			{
			    if(video[y][i]<=16);
			    else if(video[y][i]<black_max_b3){black_numbers=black_numbers+video[y][i];black_number++;}
                else {white_numbers=white_numbers+video[y][i];white_number++;}
			}
		    if(black_number)black_numbers=black_numbers/black_number;
		    if(white_number)white_numbers=white_numbers/white_number;
		    if(white_numbers&&black_numbers)black_max_3=(black_numbers+white_numbers)/2;
		    if(!(white_numbers&&black_numbers))black_max_3=black_max_b3;
		}
	}
	else if(y<35)
	{
		if(x1==0)
		{
		    for(i=x1;i<x2;i++)
			{
			    if(video[y][i]<=16);
			    else if(video[y][i]<black_max_c1){black_numbers=black_numbers+video[y][i];black_number++;}
                else {white_numbers=white_numbers+video[y][i];white_number++;}
			}
		    if(black_number)black_numbers=black_numbers/black_number;
		    if(white_number)white_numbers=white_numbers/white_number;
		    if(white_numbers&&black_numbers)black_max_1=(black_numbers+white_numbers)/2;
		    if(!(white_numbers&&black_numbers))black_max_1=black_max_c1;
		}
		else if(x1==40)
		{
			for(i=x1;i<x2;i++)
			{
			    if(video[y][i]<=16);
			    else if(video[y][i]<black_max_c2){black_numbers=black_numbers+video[y][i];black_number++;}
                else {white_numbers=white_numbers+video[y][i];white_number++;}
			}
		    if(black_number)black_numbers=black_numbers/black_number;
		    if(white_number)white_numbers=white_numbers/white_number;
		    if(white_numbers&&black_numbers)black_max_2=(black_numbers+white_numbers)/2;
            if(!(white_numbers&&black_numbers))black_max_2=black_max_c2;
		}
		else
		{
			for(i=x1;i<x2;i++)
			{
			    if(video[y][i]<=16);
			    else if(video[y][i]<black_max_c3){black_numbers=black_numbers+video[y][i];black_number++;}
                else {white_numbers=white_numbers+video[y][i];white_number++;}
			}
		    if(black_number)black_numbers=black_numbers/black_number;
		    if(white_number)white_numbers=white_numbers/white_number;
		    if(white_numbers&&black_numbers)black_max_3=(black_numbers+white_numbers)/2;
		    if(!(white_numbers&&black_numbers))black_max_3=black_max_c3;
		}
	}
	else
	{
		if(x1==0)
		{
		    for(i=x1;i<x2;i++)
			{
			    if(video[y][i]<=16);
			    else if(video[y][i]<black_max_d1){black_numbers=black_numbers+video[y][i];black_number++;}
                else {white_numbers=white_numbers+video[y][i];white_number++;}
			}
		    if(black_number)black_numbers=black_numbers/black_number;
		    if(white_number)white_numbers=white_numbers/white_number;
		    if(white_numbers&&black_numbers)black_max_1=(black_numbers+white_numbers)/2;
		    if(!(white_numbers&&black_numbers))black_max_1=black_max_d1;
		}
		else if(x1==40)
		{
			for(i=x1;i<x2;i++)
			{
			    if(video[y][i]<=16);
			    else if(video[y][i]<black_max_d2){black_numbers=black_numbers+video[y][i];black_number++;}
                else {white_numbers=white_numbers+video[y][i];white_number++;}
			}
		    if(black_number)black_numbers=black_numbers/black_number;
		    if(white_number)white_numbers=white_numbers/white_number;
		    if(white_numbers&&black_numbers)black_max_2=(black_numbers+white_numbers)/2;
            if(!(white_numbers&&black_numbers))black_max_2=black_max_d2;
		}
		else
		{
			for(i=x1;i<x2;i++)
			{
			    if(video[y][i]<=16);
			    else if(video[y][i]<black_max_d3){black_numbers=black_numbers+video[y][i];black_number++;}
                else {white_numbers=white_numbers+video[y][i];white_number++;}
			}
		    if(black_number)black_numbers=black_numbers/black_number;
		    if(white_number)white_numbers=white_numbers/white_number;
		    if(white_numbers&&black_numbers)black_max_3=(black_numbers+white_numbers)/2;
		    if(!(white_numbers&&black_numbers))black_max_3=black_max_d3;
		}
	}
}





/**********************虚线的判断***********************/
void judage_xuxian(void)
{ 
  int16 i=0,b2chazhi1=0,b2chazhi3=0;
  int16 d=0,f=0,m=0,n=0;
  for(i=53;i>=0;i--)
  {
    b2chazhi1=bianyan2[0][i]-bianyan2[0][i+1];
    b2chazhi3=bianyan2[1][i]-bianyan2[1][i+1];
		
    if(b2chazhi1<-8)
    {
      d++;
    }
    if(b2chazhi1>8)
    {
      m++;
    }
    if(b2chazhi3<-8)
    {
      n++;
    }
	
    if(b2chazhi3>8)
    {
      f++;
    }
  }
  if(d>2&&m>2&&n>2&&f>2)
  {
    xu_xian2=1;
    xu1=d;
    xu2=m;
    xu3=n;
    xu4=f;
  }


}
/**********************虚线的判断END***********************/
/*********************二次虚线的补线********************/
void xu_xian_bu(void)
{
  int16 i=H-1,m=0,h=0;
  for(i=H-1;i>h;i--)
  {
    if(bianyan2[0][54]==0&&i==54)
    {
      for(i=54;i>h;i--)
      {
              if(bianyan2[0][i]>0)	break;
              
              
      }
      for(m=i;m<H-1;m++)
      {
              bianyan[0][m+1]=bianyan[0][m];
              
      }
      xu_xian_find(0,i,h);

            
    }
    if(bianyan2[0][54]>0) xu_xian_find(0,i,h);
  }
  for(i=H-1;i>h;i--)
  {
    if(bianyan2[1][54]>=V-3&&i==54)
    {
            for(i=54;i>h;i--)
            {
                    if(bianyan2[1][i]<V-3)	break;
                    
                    
            }
            for(m=i;m<H-1;m++)
            {
                    bianyan[1][m+1]=bianyan[1][m];
                    
            }
            xu_xian_find(1,i,h);

            
    }
    if(bianyan2[1][54]<V-3) xu_xian_find(1,i,h);
  }
  for(i=H-1;i>=0;i--)center[i]=(bianyan[0][i]+bianyan[1][i])/2;

  
}
/*********************二次虚线的补线END********************/

void xu_xian_cai()
{
  int16 i=H-1,j=0,X=0;
  int16 chazhi1=0,chazhi2=0,chazhi3=0,chazhi4=0;
  for(i=H-1;i>=0;i--)
  {
    if(i>=53)
    {
      for(j=video_Middle;j>=2;j--)
      {
         if(video[i][j]==0)
         {
           if(video[i][j-1]==0)
           {
             bianyan[0][i]=j;
             break;
           }
           else if(video[i][j-2]==0)
           {
             bianyan[0][i]=j;
             break;
           }
         }
      }
      if(j==1)
      {
        if(video[i][j]==0&&video[i][j-1]==0)
        {
             bianyan[0][i]=1;
        }
        else
        {
             bianyan[0][i]=0;
        }
      }
      for(j=video_Middle;j<=V-2;j++)
      {
        if(video[i][j]==0)
        {
          if(video[i][j+1]==0)
          {
           bianyan[1][i]=j;
           break;
          }
          else if(video[i][j+2]==0)
          {
           bianyan[1][i]=j;
           break;
          }
        }
      }
      if(j==V-1)
      {
        if(video[i][j]==0&&video[i][j+1]==0)
        {
             bianyan[1][i]=V-2;
        }
        else
        {
             bianyan[1][i]=V-1;
        }
      }
  
    }
        
    if(i>=12&&i<H-2)
    {
      if(X>=2*windage)
      {  /********************左边******************************/
        for(j=bianyan[0][i+1]+windage;j>=2;j--)
        {
          if(video[i][j]==0)
          {
            if(video[i][j-1]==0)
            {
              bianyan[0][i]=j;
              break;
            }
            else if(video[i][j-2]==0)
            {
              bianyan[0][i]=j;
              break;
            }
          }
        }
        if(j==1)
        {
          if(video[i][j]==0&&video[i][j-1]==0)
          {
                bianyan[0][i]=1;
          }
          else
          {
                bianyan[0][i]=0;
          }
        }
  
  /*********************右边*******************************/
        for(j=bianyan[1][i+1]-windage;j<=V-2;j++)
        {
          if(video[i][j]==0)
          {
            if(video[i][j+1]==0)
            {
                    bianyan[1][i]=j;
                    break;
            }
            else if(video[i][j+2]==0)
            {
                    bianyan[1][i]=j;
                    break;
            }
          }
        }
        if(j==V-1)
        {
          if(video[i][j]==0&&video[i][j+1]==0)
          {
                bianyan[1][i]=V-2;
          }
          else
          {
                bianyan[1][i]=V-1;
          }
        }
  
      }
      else
      {
        for(j=center[i+1];j>=2;j--)
        {
          if(video[i][j]==0)
          {
            if(video[i][j-1]==0)
            {
                    bianyan[0][i]=j;
                    break;
            }
            else if(video[i][j-2]==0)
            {
                    bianyan[0][i]=j;
                    break;
            }
          }
        }
        if(j==1)
        {
          if(video[i][j]==0&&video[i][j-1]==0)
          {
            bianyan[0][i]=1;
          }
          else
          {
                bianyan[0][i]=0;
          }
        }
  
  
  /*********************右边*******************************/
        for(j=center[i+1];j<=V-2;j++)
        {
          if(video[i][j]==0)
          {
            if(video[i][j+1]==0)
            {
                    bianyan[1][i]=j;
                    break;
            }
            else if(video[i][j+2]==0)
            {
                    bianyan[1][i]=j;
                    break;
            }
          }
        }
        if(j==V-1)
        {
          if(video[i][j]==0&&video[i][j+1]==0)
          {
                bianyan[1][i]=V-2;
          }
          else
          {
                bianyan[1][i]=V-1;
          }
        }
      }
    }
    if(i<12)
    {
      for(j=center[i+1];j>=0;j--)
      {
        if(video[i][j]==0)
        {
                bianyan[0][i]=j;
                break;
        }
                        
      }
      for(j=center[i+1];j<=V-1;j++)
      {
        if(video[i][j]==0)
        {
                bianyan[1][i]=j;
                break;
        }
              
      }
    }
    bianyan2[0][i]=bianyan[0][i];
    bianyan2[1][i]=bianyan[1][i];
    if(i<52)
    {
      chazhi1=bianyan[0][i]-bianyan[0][i+1];
      chazhi2=bianyan[0][i+1]-bianyan[0][i+2];
      chazhi3=bianyan[1][i]-bianyan[1][i+1];
      chazhi4=bianyan[1][i+1]-bianyan[1][i+2];
    }
    if(i>=39&&i<52)
    {
      if(chazhi1<=-8)
      {
      //bianyan[0][i+1]=bianyan[0][i+2]+chazhi3;
              bianyan[0][i]=bianyan[0][i+1]+chazhi2;
      }
      if(chazhi3>=8)
      {
              bianyan[1][i]=bianyan[1][i+1]+chazhi4;
      }
    }
        
    if(i<39)
    {
      if(chazhi1<=-10)
      {
              bianyan[0][i]=bianyan[0][i+1];
      }
      if(chazhi3>=10)
      {
          bianyan[1][i]=bianyan[1][i+1];
      }
    }
    X=bianyan[1][i]-bianyan[0][i];
    center[i]=(bianyan[1][i]+bianyan[0][i])/2;
  
  
  }
}


/*********************十字交叉系统识别与补线********************/
void xie_shizi(void)
{
  int16 break_point_1=0,break_point_2=0,break_point_3=0;
  int16 i=0,j=0;
  //int16 xie_shizi_flag_1=0,xie_shizi_flag_2=0;
  int16 xie_shizi[H]={0};
  if((bianyan2[1][45]-bianyan2[1][54])<-10&&(bianyan2[0][45]-bianyan2[0][54])<5)
  {
    for(i=40;i>10;i--)
    {
      if((bianyan2[1][i]-bianyan2[1][i-1])<0&&(bianyan2[1][i-1]-bianyan2[1][i-2])<0&&(bianyan2[1][i-2]-bianyan2[1][i-3])<0)
      {
              break_point_1=i;
              break;
      }
    }
    if(i!=10)
    {
      if((bianyan2[1][i]-bianyan2[1][i-3])<-2&&(bianyan2[1][i]-bianyan2[1][i+3])<-2)
      {
        for(;i>1;i--)
        {
                if(video[i-1][bianyan2[1][break_point_1]-5]==0&&video[i-2][bianyan2[1][break_point_1]-5]==0)
                {
                        break_point_3=i;
                        break;
                }
        }
              
        if(i>1)
        {
          for(i=break_point_3;i<break_point_1;i++)
          {
                  for(j=bianyan2[1][break_point_1]-5;j>0;j--)
                  {
                          if(video[i][j]==0&&video[i][j-1]==0){xie_shizi[i]=j;break;}
                  }
                  if(j==0)xie_shizi[i]=0;
          }
          for(i=break_point_3;i<break_point_1;i++)
          {
                  if((xie_shizi[i]-xie_shizi[i+1])>10&&(xie_shizi[i]-xie_shizi[i+2])>10)
                  {
                          break_point_2=i;
                          break;
                  }
          }
          if(i!=break_point_1)//证明存在斜十字
          {
                  xie_shizi_flag_1=1;
                  for(i=break_point_1;i>=0;i--)center[i]=0;
          }
        }
      }
    }
    if(xie_shizi_flag_1==0)
    {
      if(abs(bianyan2[1][break_point_1]-bianyan2[1][break_point_1-1])+abs(bianyan2[1][break_point_1]-bianyan2[1][break_point_1+1])>4)xie_shizi_flag_3=1;
    }
          
  }
  if((bianyan2[1][45]-bianyan2[1][54])>-5&&(bianyan2[0][45]-bianyan2[0][54])>10)
  {
    for(i=40;i>10;i--)
    {
      if((bianyan2[0][i]-bianyan2[0][i-1])>0&&(bianyan2[0][i-1]-bianyan2[0][i-2])>0&&(bianyan2[0][i-2]-bianyan2[0][i-3])>0)
      {
              break_point_1=i;
              break;
      }
    }
    if(i!=10)
    {
      if((bianyan2[0][i]-bianyan2[0][i-3])>2&&(bianyan2[0][i]-bianyan2[0][i+3])>2)
      {
              
        for(;i>1;i--)
        {
          if(video[i-1][bianyan2[0][break_point_1]+5]==0&&video[i-2][bianyan2[0][break_point_1]+5]==0)
          {
                  break_point_3=i;
                  break;
          }
        }
                          
        if(i>1)
        {
          for(i=break_point_3;i<break_point_1;i++)
          {
            for(j=bianyan2[0][break_point_1]+5;j<159;j++)
            {
                    if(video[i][j]==0&&video[i][j+1]==0){xie_shizi[i]=j;break;}//

            }
            if(j==159)xie_shizi[i]=159;
          }
          for(i=break_point_3;i<break_point_1;i++)
          {
            if((xie_shizi[i]-xie_shizi[i+1])<-10&&(xie_shizi[i]-xie_shizi[i+2])<-10)
            {
                    break_point_2=i;
                    break;
            }
          }
          if(i!=break_point_1)//证明存在斜十字
          {
                  xie_shizi_flag_2=1;
                  for(i=break_point_1;i>=0;i--)center[i]=0;
          }
        }
      }
    }
    if(xie_shizi_flag_2==0)
    {
      if(abs(bianyan2[0][break_point_1]-bianyan2[0][break_point_1-1])+abs(bianyan2[0][break_point_1]-bianyan2[0][break_point_1+1])>4)xie_shizi_flag_3=1;
    }
  }
  if(xie_shizi_flag_1==1)
	{
		//bianyan[1][break_point_2]=xie_shizi[break_point_2];
		
		for(i=break_point_2;i>0;i--)
		{
			
			for(j=xie_shizi[break_point_2]/2;j<xie_shizi[break_point_2];j++)
			{
				if(video[i][j]==0&&video[i][j+1]==0){bianyan[1][i]=j;break;}
			}
			if((bianyan[1][i]-bianyan[1][i+1])>0)bianyan[1][i]=bianyan[1][i+1];

			if(j==xie_shizi[break_point_2]/2)break;
		
		}
        LianJie(1,break_point_1,break_point_2);
        END_Line=i;
		if(i==0)
		{
			for(j=bianyan[1][2];j>0;j--)
			{
				if(video[2][j]==0&&video[2][j-1]==0)break;
			}
			center[2]=(j+bianyan[1][2])/2;
            LianJie_center(break_point_1+1,2);
		}
		if(i)
		{
			for(j=xie_shizi[break_point_2]/2;j>0;j++)
			{
				if(video[i+1][j]==0&&video[i+1][j-1]==0)break;
			}
			center[i+1]=(bianyan[1][i+1]+j)/2;
            if(center[break_point_1+1]&&(center[i+1]-center[break_point_1+1])<-9)//
			{
				LianJie_center(break_point_1+1,i+1);
			}
			if(center[break_point_1+1]==0||(center[i+1]-center[break_point_1+1])>=-9)center[i+1]=0;
        
		}
                for(i=END_Line;i>=0;i--){bianyan[0][i]=0;bianyan[1][i]=0;}
		for(i=break_point_2+1;i>=END_Line;i--)
		{
            for(j=bianyan[1][i];j>0;j--)
			{
				if(video[i][j]==0&&video[i][j-1]==0)
				{
					bianyan[0][i]=j;
					break;
				}
			}
			if(j==0)bianyan[1][i]=0;
			if(bianyan[0][i]-bianyan[0][i+1]>3)bianyan[0][i]=bianyan[0][i+1];
		}

	}
	if(xie_shizi_flag_2==1)
	{
		//bianyan[0][break_point_2]=xie_shizi[break_point_2];
		
		for(i=break_point_2;i>0;i--)
		{
			for(j=(159+xie_shizi[break_point_2])/2;j>xie_shizi[break_point_2];j--)
			{
                if(video[i][j]==0&&video[i][j-1]==0){bianyan[0][i]=j;break;}
			}
			if((bianyan[0][i]-bianyan[0][i+1])<0)bianyan[0][i]=bianyan[0][i+1];
			if(j==(159+xie_shizi[break_point_2])/2)break;
		}
		LianJie(0,break_point_1,break_point_2);
                END_Line=i;
		if(i==0)
		{
			for(j=bianyan[0][2];j<V-1;j++)
			{
				if(video[2][j]==0&&video[2][j+1]==0)break;
			}
			center[2]=(j+bianyan[0][2])/2;
            LianJie_center(break_point_1+1,2);
		}
		if(i)
		{
			for(j=(159+xie_shizi[break_point_2])/2;j<V-1;j++)
			{
				if(video[i+1][j]==0&&video[i+1][j+1]==0)break;
			}
			center[i+1]=(bianyan[0][i+1]+j)/2;
            if(center[break_point_1+1]&&(center[i+1]-center[break_point_1+1])>9)//
			{
				LianJie_center(break_point_1+1,i+1);
			}
			if(center[break_point_1+1]==0||(center[i+1]-center[break_point_1+1])<=9)center[i+1]=0;
        
		}
                for(i=END_Line;i>=0;i--){bianyan[0][i]=0;bianyan[1][i]=0;}
		for(i=break_point_2;i>=END_Line;i--)
		{
            for(j=bianyan[0][i];j<V-1;j++)
			{
				if(video[i][j]==0&&video[i][j+1]==0)
				{
					bianyan[1][i]=j;
					break;
				}
			}
			if(j==V-1)bianyan[1][i]=V-1;
			if(bianyan[1][i]-bianyan[1][i+1]>3)bianyan[1][i]=bianyan[1][i+1];
		}
		
	}
  if(xie_shizi_flag_1==0&&xie_shizi_flag_2==0&&break_point_1<35&&break_point_1>18)
	{
		if((bianyan[0][break_point_1]-bianyan[0][54])>100)
		{
			if((bianyan[0][break_point_1]-bianyan[0][break_point_1+2])>10)
			{
				for(i=53;i>=break_point_1;i--)
				{
					if(bianyan[1][i]>156)
					{
						if(center[i+1]+bianyan[0][i]-bianyan[0][i+1]<160)center[i]=center[i+1]+bianyan[0][i]-bianyan[0][i+1];
						else
						{
							for(;i>=0;i--)center[i]=0;
						}
					}
				}
				for(i=break_point_1;i>=0;i--)center[i]=0;
			}
		}
		if((bianyan[1][break_point_1]-bianyan[1][54])<-100)
		{
			if((bianyan[1][break_point_1]-bianyan[1][break_point_1+2])<-10)
			{
				for(i=53;i>=break_point_1;i--)
				{
					if(bianyan[0][i]<3)
					{
						if(center[i+1]+bianyan[1][i]-bianyan[1][i+1]>=0)center[i]=center[i+1]+bianyan[0][i]-bianyan[0][i+1];
						else
						{
							for(;i>=0;i--)center[i]=0;
						}
					}
				}
				for(i=break_point_1;i>=0;i--)center[i]=0;
			}
		}
	}
}
/*********************十字交叉系统识别与补线END********************/
/***********************************
*function name:X_LineBend()
*function:优化图像处理，使急弯更加平滑的过渡到长大弯
*date:2013.5.4
*潘杰
************************************/
void X_LineBend(void)
{
  int16 k=54,i=0,j=0;
  while(center[k]&&k>10)k--;
  if(k>15)
  {
    if((k-END_Line_2)>15)
    {
      if(xie_shizi_flag_3==0)
      {
        for(i=k;i>=END_Line_2;i--)
        {
          if(bianyan[0][i]>0&&bianyan[1][i]<159){center[i]=(bianyan[1][i]+bianyan[0][i])/2;break;}
        }
        for(j=k;j>i;j--)center[j]=center[i];
        if((k-i)<3){center[k+1]=center[i];center[k+2]=center[i];}
        for(i=i-1;i>=END_Line_2;i--)center[i]=(bianyan[1][i]+bianyan[0][i])/2;
      }
    }
  }
}

void find_bianyan(void)
{
  uint8 start_18_25=0,start_25_35=0,start_35_45=0,start_45_55=0;
  uint8 s1=0,s2=0,s3=0,s4=0,s5=0,s6=0,s7=0;
  int16 quan_bai_num_k=0,shizi2=0,shizi3=0,kk=0;
  int16 n_1=0,num_3=0,num_4=0,num_5=0,num_6=0;
  int16 chazhi_center_1=0,chazhi_center_2=0;
  int16 num_1=0,num_2=0;
  int16 n=0,numx=0;
  int16 bu1_number=0,bu2_number=0;
  int16 chazhi_5=0,chazhi_6=0;//用于十字交叉的断点与补线
  int16 i=0,j=0;
  int16 L0=0,L1=0,iii=H-1,iii_end=0,iiii=0,quan_bai_begin=0,quan_bai_end_1=0,quan_bai_end=0,shizi=0;
  int16 Small_S=0,b1=0;
  int16 X=0,chazhi1=0,chazhi3=0,chazhi1_buxian=0,chazhi3_buxian=0;
  int16 END=0,END1=0;//END2=0,END3=0;
  int16 he=0,m=0,cha=0;
  int16 d1_1=0,d1_2=0,d2_1=0,d2_2=0;//十字交叉断点寻找与补线
  unsigned char quan_bai[H]={0};
  threshold();            //非动态阈值二值化
  
  
  /************起始线检测************/
  if(P_timer<30||P_timer>170)//P_timer<30||P_timer>200  pit2定时器，设置的是0.1s一次，每次P_timer++,即前3s与后20s进行起始线检测，但只有后20s检测到的起始线才停车
  {
    Start_Detection_3(0,25,18,40,120);
    BW_filter(3);
    if(BW3[0]==0)
    {
      for(i=0;i<82;i++)
      {
        if(BW3[i]==1)
        {
                s1=i;
                break;
        }
      }
      if(i<82)
      {
        for(;i<82;i++)
        {
          if(BW3[i]==0)
          {
                  s2=i;
                  break;
          }
        }
        if(i<82)
        {
          for(;i<82;i++)
          {
            if(BW3[i]==1)
            {
                    s3=i;
                    break;
            }
          }
          if(i<82)
          {
            for(;i<82;i++)
            {
              if(BW3[i]==0)
              {
                      s4=i;
                      break;
              }
            }
            if(i<82)
            {
              for(;i<82;i++)
              {
                if(BW3[i]==1)
                {
                  s5=i;
                  break;
                }
              }
              if(i<82)
              {
                for(;i<82;i++)
                {
                  if(BW3[i]==0)
                  {
                          s6=i;
                          break;
                  }
                }
                if(i<82)
                {
                  for(;i<82;i++)
                  {
                    if(BW3[i]==1)
                    {
                            s7=i;
                            break;
                    }
                  }
                  if(i==82)start_xian=1;
                }
	      }
	    }
	  }
	}

      }
      if(start_xian==1)
      {
         if((s4-s3)>25||(s2-s1)>7||(s6-s5)>7||(s3-s2)<6||(s5-s4)<6)
         {
                 start_18_25_2=1;
                 start_xian=0;
         }
         else start_18_25=1;
      }
    }
	
    if(start_xian==0)
    {
      Start_Detection(0,35,25,30,130);
      BW_filter(0);
      if(BW[0]==0)
      {
        for(i=0;i<102;i++)
        {
          if(BW[i]==1)
          {
                  s1=i;
                  break;
          }
        }
        if(i<102)
        {
          for(;i<102;i++)
          {
            if(BW[i]==0)
            {
                    s2=i;
                    break;
            }
          }
          if(i<102)
          {
            for(;i<102;i++)
            {
              if(BW[i]==1)
              {
                      s3=i;
                      break;
              }
            }
            if(i<102)
            {
              for(;i<102;i++)
              {
                if(BW[i]==0)
                {
                        s4=i;
                        break;
                }
              }
              if(i<102)
              {
                for(;i<102;i++)
                {
                  if(BW[i]==1)
                  {
                          s5=i;
                          break;
                  }
                }
                if(i<102)
                {
                  for(;i<102;i++)
                  {
                    if(BW[i]==0)
                    {
                            s6=i;
                            break;
                    }
                  }
                  if(i<102)
                  {
                    for(;i<102;i++)
                    {
                      if(BW[i]==1)
                      {
                              s7=i;
                              break;
                      }
                    }
                    if(i==102)start_xian=1;
		  }
		}
	      }
	    }
	  }
        }
      }
      if(start_xian==1)
      {
         if((s4-s3)>30||(s2-s1)>12||(s6-s5)>12)//||(s3-s2)<4||(s5-s4)<4
         {
                 start_25_35_2=1;
                 start_xian=0;
         }
         else start_25_35=1;
      }
      if(start_xian==0)
      {
        Start_Detection_2(0,45,35,20,140);
        BW_filter(2);
        if(BW2[0]==0)
        {
          for(i=0;i<122;i++)
          {
            if(BW2[i]==1)
            {
                    s1=i;
                    break;
            }
          }
          if(i<122)
          {
            for(;i<122;i++)
            {
              if(BW2[i]==0)
              {
                      s2=i;
                      break;
              }
            }
            if(i<122)
            {
              for(;i<122;i++)
              {
                if(BW2[i]==1)
                {
                        s3=i;
                        break;
                }
              }
              if(i<122)
              {
                for(;i<122;i++)
                {
                  if(BW2[i]==0)
                  {
                          s4=i;
                          break;
                  }
                }
                if(i<122)
                {
                  for(;i<122;i++)
                  {
                    if(BW2[i]==1)
                    {
                            s5=i;
                            break;
                    }
                  }
                  if(i<122)
                  {
                    for(;i<122;i++)
                    {
                      if(BW2[i]==0)
                      {
                              s6=i;
                              break;
                      }
                    }
                    if(i<122)
                    {
                      for(;i<122;i++)
                      {
                        if(BW2[i]==1)
                        {
                                s7=i;
                                break;
                        }
                      }
                      if(i==122)start_xian=1;
                    }
                  }
                }
              }
            }
          }
        }
        if(start_xian==1)
        {
          if((s4-s3)>35||(s2-s1)>13||(s6-s5)>13||(s3-s2)<15||(s5-s4)<15)
          {
                  start_35_45_2=1;
                  start_xian=0;
          }
          else start_35_45=1;
        }
        if(start_xian==0)
        {
          Start_Detection_4(0,54,45,10,150);
          BW_filter(4);
          if(BW4[0]==0)
          {
            for(i=0;i<142;i++)
            {
              if(BW4[i]==1)
              {
                      s1=i;
                      break;
              }
            }
            if(i<142)
            {
              for(;i<142;i++)
              {
                if(BW4[i]==0)
                {
                        s2=i;
                        break;
                }
              }
              if(i<142)
              {
                for(;i<142;i++)
                {
                  if(BW4[i]==1)
                  {
                          s3=i;
                          break;
                  }
                }
                if(i<142)
                {
                  for(;i<142;i++)
                  {
                    if(BW4[i]==0)
                    {
                            s4=i;
                            break;
                    }
                  }
                  if(i<142)
                  {
                    for(;i<142;i++)
                    {
                      if(BW4[i]==1)
                      {
                              s5=i;
                              break;
                      }
                    }
                    if(i<142)
                    {
                      for(;i<142;i++)
                      {
                        if(BW4[i]==0)
                        {
                                s6=i;
                                break;
                        }
                      }
                      if(i<142)
                      {
                        for(;i<142;i++)
                        {
                          if(BW4[i]==1)
                          {
                                  s7=i;
                                  break;
                          }
                        }
                        if(i==142)start_xian=1;
                      }
                    }
		  }
		}
	      }
            }
          }
          if(start_xian==1)
          {
            if((s4-s3)>35||(s2-s1)>15||(s6-s5)>15)
            {
                    start_45_55_2=1;
                    start_xian=0;
            }
            else start_45_55=1;
          }
        }
      }
    }
    //for(i=0;i<102;i++)BW[i]=0;
	
	/************起始线检测END************/
  /***************如果检测出起始线就把起始线黑线去掉**************/
    if(start_xian==1)
	{
		if(start_45_55==1)
		{
             for(i=(s1+s2)/2+10;i<=(s3+s4)/2+10;i++)
			 {
				 for(j=54;j>45;j--)
				 {
					 if(video[j][i]==0)break;
				 }
				 for(;j>40;j--)
				 {
					 if(video[j][i]==0)video[j][i]=1;
				 }
			 }
			 for(i=(s3+s4)/2+10;i<=(s5+s6)/2+10;i++)
			 {
				 for(j=54;j>45;j--)
				 {
					 if(video[j][i]==0)break;
				 }
				 for(;j>40;j--)
				 {
					 if(video[j][i]==0)video[j][i]=1;
				 }
			 }
		}
		if(start_35_45==1)
		{
             for(i=(s1+s2)/2+20;i<=(s3+s4)/2+20;i++)
			 {
				 for(j=47;j>35;j--)
				 {
					 if(video[j][i]==0)
					 {
						 m=j;
						 break;
					 }
				 }
				 if(j==47)
				 {
					 m=j+1;
					 for(;j>40;j--)
					 {
						 if(video[j][i]==0)video[j][i]=1;
					 }
					 for(;m<50;m++)
					 {
						 if(video[m][i]==0)video[m][i]=1;
					 }
				 }
				 else if(j==35)
				 {
					 for(;j>31;j--)video[j][i]=1;
				 }
				 else
				 {
					 for(;j>31;j--)
					 {
						 if(video[j][i]==1)break;
					 }
					 for(;m>j;m--)video[m][i]=1;
				 }
				 
			 }
			 for(i=(s3+s4)/2+20;i<=(s5+s6)/2+20;i++)
			 {
				 for(j=47;j>35;j--)
				 {
					 if(video[j][i]==0)
					 {
						 m=j;
						 break;
					 }
				 }
				 if(j==47)
				 {
					 m=j+1;
					 for(;j>40;j--)
					 {
						 if(video[j][i]==0)video[j][i]=1;
					 }
					 for(;m<50;m++)
					 {
						 if(video[m][i]==0)video[m][i]=1;
					 }
				 }
				 else if(j==35)
				 {
					 for(;j>31;j--)video[j][i]=1;
				 }
				 else
				 {
					 for(;j>31;j--)
					 {
						 if(video[j][i]==1)break;
					 }
					 for(;m>j;m--)video[m][i]=1;
				 }
				 
			 }
		}
		if(start_25_35==1)
		{
             for(i=(s2+s1)/2+30;i<=(s3+s4)/2+30;i++)
			 {
				 for(j=37;j>25;j--)
				 {
					 if(video[j][i]==0)
					 {
						 m=j;
						 break;
					 }
				 }
				 if(j==37)
				 {
					 m=j+1;
					 for(;j>30;j--)
					 {
						 if(video[j][i]==0)video[j][i]=1;
					 }
					 for(;m<40;m++)
					 {
						 if(video[m][i]==0)video[m][i]=1;
					 }
				 }
				 else
				 {
					 for(;j>22;j--)
					 {
						 if(video[j][i]==1)break;
					 }
					 for(;m>j;m--)video[m][i]=1;
				 }
				 
			 }
			 for(i=(s3+s4)/2+30;i<=(s5+s6)/2+30;i++)
			 {
				 for(j=37;j>25;j--)
				 {
					 if(video[j][i]==0)
					 {
						 m=j;
						 break;
					 }
				 }
				 if(j==37)
				 {
					 m=j+1;
					 for(;j>30;j--)
					 {
						 if(video[j][i]==0)video[j][i]=1;
					 }
					 for(;m<40;m++)
					 {
						 if(video[m][i]==0)video[m][i]=1;
					 }
				 }
				 else
				 {
					 for(;j>22;j--)
					 {
						 if(video[j][i]==1)break;
					 }
					 for(;m>j;m--)video[m][i]=1;
				 }
				 
			 }
		}
		if(start_18_25==1)
		{
             for(i=(s1+s2)/2+40;i<=(s3+s4)/2+40;i++)
			 {
				 for(j=27;j>18;j--)
				 {
					 if(video[j][i]==0)
					 {
						 m=j;
						 break;
					 }
				 }
				 if(j==27)
				 {
					 m=j+1;
					 for(;j>20;j--)
					 {
						 if(video[j][i]==0)video[j][i]=1;
					 }
					 for(;m<30;m++)
					 {
						 if(video[m][i]==0)video[m][i]=1;
					 }
				 }
				 else
				 {
					 for(;j>16;j--)
					 {
						 if(video[j][i]==1)break;
					 }
					 for(;m>j;m--)video[m][i]=1;
				 }
				 
			 }
			 for(i=(s3+s4)/2+40;i<=(s5+s6)/2+40;i++)
			 {
				 for(j=27;j>18;j--)
				 {
					 if(video[j][i]==0)
					 {
						 m=j;
						 break;
					 }
				 }
				 if(j==27)
				 {
					 m=j+1;
					 for(;j>20;j--)
					 {
						 if(video[j][i]==0)video[j][i]=1;
					 }
					 for(;m<30;m++)
					 {
						 if(video[m][i]==0)video[m][i]=1;
					 }
				 }
				 else
				 {
					 for(;j>16;j--)
					 {
						 if(video[j][i]==1)break;
					 }
					 for(;m>j;m--)video[m][i]=1;
				 }
				 
			 }
		}
	}
  }
	/***************如果检测出起始线就把起始线黑线去掉END**************/
 
  
    /******************全图扫描******************
	/开始对全图扫描一次用来判断赛道的情况
	/b1,b2的数目是用来判断全部是白点的行数
    ********************************************/
  for(i=H-1;i>10;i--)
  {
    for(j=V/2;j>=1;j--)
    {
      if(video[i][j]==0&&video[i][j-1]==0)break;
    }
    L0=j;
    for(j=V/2;j<=V-2;j++)
    {
      if(video[i][j]==0&&video[i][j+1]==0)break;
    }
    L1=j;
    if(L1-L0>V-20)
    {
      quan_bai[i]=i;
      b1++;
    }
  }
  if(b1>shi_zi_jiao_cha_number)shizi=1;
  if(shizi==1)
  {
    while(quan_bai[iii]==0)iii--;
    quan_bai_begin=iii;
    iii_end=iii;
    while(quan_bai[iii_end]>0)iii_end--;
    quan_bai_end_1=iii_end;
    while(quan_bai[iiii]==0)iiii++;
    quan_bai_end=iiii;
    if(quan_bai_begin-quan_bai_end>b1)Small_S=1; 
  }
  /******************全图扫描END******************/
  if(Small_S==1)
  {
    xu_xian_cai();
  }
  if(Small_S==0)
  {
    i=H-1;
    while(i>=0)
    {
		
    
    /***********************寻点范围设定***************************/
      if(i>=53)
      {
        for(j=video_Middle;j>=2;j--)
        {
          if(video[i][j]==0)
          {
            if(video[i][j-1]==0)
            {
                    bianyan[0][i]=j;
                    break;
            }
            else if(video[i][j-2]==0)
            {
                    bianyan[0][i]=j;
                    break;
            }
          }
        }
        if(j==1)
        {
          if(video[i][j]==0&&video[i][j-1]==0)
          {
                  bianyan[0][i]=1;
          }
          else
          {
                  bianyan[0][i]=0;
          }
        }
        for(j=video_Middle;j<=V-4;j++)
        {
          if(video[i][j]==0)
          {
            if(video[i][j+1]==0)
            {
                    bianyan[1][i]=j;
                    break;
            }
            else if(video[i][j+2]==0)
            {
                    bianyan[1][i]=j;
                    break;
            }
          }
        }
        if(j==V-3)
        {
          if(video[i][j]==0&&video[i][j+1]==0)
          {
                  bianyan[1][i]=V-4;
          }
          else
          {
                  bianyan[1][i]=V-3;
          }
        }
        bianyan2[0][i]=bianyan[0][i];
        bianyan2[1][i]=bianyan[1][i];
      } 
      else
      {
        if(X>=2*windage)
        {  /********************左边******************************/
          for(j=bianyan[0][i+1]+windage;j>=2;j--)
          {
            if(video[i][j]==0)
            {
              if(video[i][j-1]==0)
              {
                     bianyan[0][i]=j;
                     break;
              }
              else if(video[i][j-2]==0)
              {
                     bianyan[0][i]=j;
                     break;
              }
            }
          }
          if(j==1)
          {
            if(video[i][j]==0&&video[i][j-1]==0)
            {
               bianyan[0][i]=1;
            }
            else
            {
               bianyan[0][i]=0;
            }
          }

/*********************右边*******************************/
          for(j=bianyan[1][i+1]-windage;j<=V-4;j++)
          {
            if(video[i][j]==0)
            {
              if(video[i][j+1]==0)
              {
                      bianyan[1][i]=j;
                      break;
              }
              else if(video[i][j+2]==0)
              {
                      bianyan[1][i]=j;
                      break;
              }
            }
          }
          if(j==V-3)
          {
            if(video[i][j]==0&&video[i][j+1]==0)
            {
                   bianyan[1][i]=V-4;
            }
            else
            {
                   bianyan[1][i]=V-3;
            }
          }

        }
        else
        {
          for(j=center[i+1];j>=2;j--)
          {
            if(video[i][j]==0)
            {
              if(video[i][j-1]==0)
              {
                      bianyan[0][i]=j;
                      break;
              }
              else if(video[i][j-2]==0)
              {
                      bianyan[0][i]=j;
                      break;
              }
            }
          }
          if(j==1)
          {
            if(video[i][j]==0&&video[i][j-1]==0)
            {
                    bianyan[0][i]=1;
            }
            else
            {
                    bianyan[0][i]=0;
            }
          }
     
      
		  /*********************右边*******************************/
          for(j=center[i+1];j<=V-4;j++)
          {
            if(video[i][j]==0)
            {
              if(video[i][j+1]==0)
              {
                      bianyan[1][i]=j;
                      break;
              }
              else if(video[i][j+2]==0)
              {
                      bianyan[1][i]=j;
                      break;
              }
            }
          }
          if(j==V-3)
          {
            if(video[i][j]==0&&video[i][j+1]==0)
            {
                    bianyan[1][i]=V-4;
            }
            else
            {
                    bianyan[1][i]=V-3;
            }
          }
    

	}
		
		  
        bianyan2[0][i]=bianyan[0][i];
        bianyan2[1][i]=bianyan[1][i];


        chazhi1=bianyan[0][i]-bianyan[0][i+1];
        chazhi3=bianyan[1][i]-bianyan[1][i+1];
        if(i>=50)
        {
                chazhi1_buxian=bianyan[0][i+1]-bianyan[0][i+2];
                chazhi3_buxian=bianyan[1][i+1]-bianyan[1][i+2];
        }
				
        if(i<50)
        {
                chazhi1_buxian=bianyan[0][i+2]-bianyan[0][i+5];
                chazhi1_buxian=chazhi1_buxian/3;
                chazhi3_buxian=bianyan[1][i+2]-bianyan[1][i+5];
                chazhi3_buxian=chazhi3_buxian/3;
        }
      }
		

			 /*********************对bianyan[][]的值进行调整***********************/
   
    
      if(chazhi1<=-8)
      {

        if(i>30&&chazhi1_buxian>1)
        {
                bianyan[0][i]=bianyan[0][i+1]+1;
        }
        else
        {
                bianyan[0][i]=bianyan[0][i+1]+chazhi1_buxian;
        }
              
              
              
      }

      if(chazhi3>=8)
      {
        if(i>30&&chazhi3_buxian<-1)
        {
                bianyan[1][i]=bianyan[1][i+1]-1;
        }
        else
        {
                bianyan[1][i]=bianyan[1][i+1]+chazhi3_buxian;
        }

      }
    

			
			

			
      if(shizi==1&&i>=quan_bai_end)
      {
        if(chazhi1<=-6||(bianyan[0][i]<3&&i<H-1))
        {
                if(i%2==0)bianyan[0][i]=bianyan[0][i+1]+1;
                else bianyan[0][i]=bianyan[0][i+1];
                
        }
        if(chazhi3>=6||(bianyan[1][i]>=V-4&&i<H-1))
        {
                if(i%2==0)bianyan[1][i]=bianyan[1][i+1]-1;
                else bianyan[1][i]=bianyan[1][i+1];
        }
      }




			/******************十字交叉END********************/
			/***********************************************/
      X=bianyan[1][i]-bianyan[0][i];
    
    
      if(X<=5)
      {
              END=i;	
              break;
      }//弯道时判断处理结束线的


      center[i]=(bianyan[1][i]+bianyan[0][i])/2;
      i--;
    }
  }
  END_Line_2=END;
  /**************中心优化1***************/
  if(END>0)
  {
    he=center[END+1]+center[END+2]+center[END+3];
    he=he/3;
    for(m=END+1;m<H-3;m++)
    {
      cha=center[m]-he;
      cha=abs(cha);
      if(cha>2)
      {
        END1=m;
        break;
      }
    }
  }
  for(m=END1-1;m>=0;m--)
  {
          center[m]=0;
  }
  judage_xuxian();
  /*********************十字交叉断点的寻找与补线***********************/
  if(shizi==1&&xu_xian2==0)
  {
    int16 d1_chazhi1=0,d1_chazhi2=0,d2_chazhi1=0,d2_chazhi2=0;
    if(quan_bai_begin<H-1)
    {
      for(i=quan_bai_begin;i<H-1;i++)
      {
        if(bianyan2[0][i]>3)
        {
          d1_chazhi1=bianyan2[0][i]-bianyan2[0][i+1];
          d1_chazhi1=abs(d1_chazhi1);
          if(d1_chazhi1<3)break;
        }
      }
      d1_1=i;
      for(i=quan_bai_begin;i<H-1;i++)
      {
        if(bianyan2[1][i]<V-4)
        {
          d2_chazhi1=bianyan2[1][i]-bianyan2[1][i+1];
          d2_chazhi1=abs(d2_chazhi1);
          if(d2_chazhi1<3)break;
        }
      }
      d2_1=i;
    }
    if(quan_bai_begin==H-1){d1_1=H;d2_1=H;}
	
    for(i=quan_bai_end_1;i>0;i--)
    {   
      if(bianyan2[0][i]>3)
      {
         d1_chazhi2=bianyan2[0][i]-bianyan2[0][i-1];
         d1_chazhi2=abs(d1_chazhi2);
         if(d1_chazhi2<3)break;
      }
  
    }
    d1_2=i;
    for(i=quan_bai_end_1;i>0;i--)
    {
      if(bianyan2[1][i]<V-4)
      {
         d2_chazhi2=bianyan2[1][i]-bianyan2[1][i-1];
         d2_chazhi2=abs(d2_chazhi2);
         if(d2_chazhi2<3)break;
      }
  
    }
    d2_2=i;
    if(d1_1==quan_bai_begin)
    {
      for(i=quan_bai_begin-1;i>quan_bai_end_1;i--)
      {
            
          d1_chazhi1=bianyan2[0][i]-bianyan2[0][i-1];
          d1_chazhi1=abs(d1_chazhi1);
          if(d1_chazhi1>3)break;
    
      }
      d1_1=i;
    }
    if(d2_1==quan_bai_begin)
    {
      for(i=quan_bai_begin-1;i>quan_bai_end_1;i--)
      {
            
          d2_chazhi1=bianyan2[1][i]-bianyan2[1][i-1];
          d2_chazhi1=abs(d2_chazhi1);
          if(d2_chazhi1>3)break;
    
  
      }
      d2_1=i;
    }
    if(d1_1<H){bianyan[0][d1_1]=bianyan2[0][d1_1];bianyan[0][d1_2]=bianyan2[0][d1_2];LianJie(0,d1_1,d1_2);}
    if(d2_1<H){bianyan[1][d1_1]=bianyan2[1][d1_1];bianyan[1][d1_2]=bianyan2[1][d1_2];LianJie(1,d2_1,d2_2);}
    if(d1_1==H)
    {
      for(i=d1_2-9;i<d1_2;i++)
      {
             chazhi_5=chazhi_5+bianyan[0][i]-bianyan[0][i-1];
             
      }
      chazhi_5=chazhi_5/9;
      for(i=d1_2;i<H-1;i++)bianyan[0][i+1]=bianyan[0][i]+chazhi_5;
    }
    if(d2_1==H)
    {
      for(i=d2_2-9;i<d2_2;i++)
      {
             chazhi_6=chazhi_6+bianyan[1][i]-bianyan[1][i-1];
              
      }
      chazhi_6=chazhi_6/9;
      for(i=d2_2;i<H-1;i++)bianyan[1][i+1]=bianyan[1][i]+chazhi_6;
    }

	   
    if(d1_1>d2_1)
    {
      if(d1_2>d2_2)
      {
             for(i=d1_1;i>=d2_2;i--)center[i]=(bianyan[0][i]+bianyan[1][i])/2;
      }
      else
      {
         for(i=d1_1;i>=d1_2;i--)center[i]=(bianyan[0][i]+bianyan[1][i])/2;
      }
  
    }
    if(d1_1<=d2_1)
    {
      if(d1_2>d2_2)
      {
         for(i=d2_1;i>=d2_2;i--)center[i]=(bianyan[0][i]+bianyan[1][i])/2;
      }
      else
      {
         for(i=d2_1;i>=d1_2;i--)center[i]=(bianyan[0][i]+bianyan[1][i])/2;
      }
    }
  }
  /*********************十字交叉断点的寻找与补线END***********************/
  if(xu_xian2==1)
  {  
    for(i=quan_bai_begin;i>quan_bai_end_1;i--)
		{
			
			if((bianyan2[1][i]-bianyan2[0][i])>V-5)
			{
				m=i;
				break;
			}
		}
		for(;i>quan_bai_end_1;i--)
		{
			if((bianyan2[1][i]-bianyan2[0][i])<=V-5)break;
		}
		quan_bai_num_k=m-i;
		if(quan_bai_num_k>((quan_bai_begin-quan_bai_end_1)/2))shizi2=1;
		if(shizi2==1)
		{
			if(quan_bai_num_k>16)shizi3=1;
			else
			{
				for(;i>=quan_bai_end;i--)
				{
					if(bianyan[0][i]==bianyan2[0][i]&&bianyan[1][i]==bianyan2[1][i])
					{
						kk=i;
						break;
					}
					
				}
				for(;i>=quan_bai_end;i--)
				{
					if(!(bianyan[0][i]==bianyan2[0][i]&&bianyan[1][i]==bianyan2[1][i]))break;
				}
				if((kk-i)>=10)shizi3=1;
			}
			
		}
		if(shizi3==0)	xu_xian_bu();
		if(shizi3==1)
		{
			int d1_chazhi1=0,d1_chazhi2=0,d2_chazhi1=0,d2_chazhi2=0;
   	   if(quan_bai_begin<H-1)
	   {
	   	  for(i=quan_bai_begin;i<H-1;i++)
		  {
		     if(bianyan2[0][i]>3)
			 {
		        d1_chazhi1=bianyan2[0][i]-bianyan2[0][i+1];
		        d1_chazhi1=abs(d1_chazhi1);
		        if(d1_chazhi1<3)break;
			 }
		  }
          d1_1=i;
	      for(i=quan_bai_begin;i<H-1;i++)
		  {
	      	 if(bianyan2[1][i]<V-4)
			 {
		        d2_chazhi1=bianyan2[1][i]-bianyan2[1][i+1];
		        d2_chazhi1=abs(d2_chazhi1);
		        if(d2_chazhi1<3)break;
			 }
		  }
    	  d2_1=i;
	   }
	   if(quan_bai_begin==H-1){d1_1=H;d2_1=H;}
	
	   for(i=quan_bai_end_1;i>0;i--)
	   {   
		  if(bianyan2[0][i]>3)
		  {
		     d1_chazhi2=bianyan2[0][i]-bianyan2[0][i-1];
		     d1_chazhi2=abs(d1_chazhi2);
		     if(d1_chazhi2<3)break;
		  }

	   }
       d1_2=i;
	   for(i=quan_bai_end_1;i>0;i--)
	   {
		  if(bianyan2[1][i]<V-4)
		  {
		     d2_chazhi2=bianyan2[1][i]-bianyan2[1][i-1];
		     d2_chazhi2=abs(d2_chazhi2);
		     if(d2_chazhi2<3)break;
		  }

	   }
	   d2_2=i;
       if(d1_1==quan_bai_begin)
	   {
		  for(i=quan_bai_begin-1;i>quan_bai_end_1;i--)
		  {
			
		      d1_chazhi1=bianyan2[0][i]-bianyan2[0][i-1];
		      d1_chazhi1=abs(d1_chazhi1);
		      if(d1_chazhi1>3)break;
		
		  }
		  d1_1=i;
	   }
	   if(d2_1==quan_bai_begin)
	   {
		  for(i=quan_bai_begin-1;i>quan_bai_end_1;i--)
		  {
			
		      d2_chazhi1=bianyan2[1][i]-bianyan2[1][i-1];
		      d2_chazhi1=abs(d2_chazhi1);
		      if(d2_chazhi1>3)break;
		
            
		  }
		  d2_1=i;
	   }
	   if(d1_1<H){bianyan[0][d1_1]=bianyan2[0][d1_1];bianyan[0][d1_2]=bianyan2[0][d1_2];LianJie(0,d1_1,d1_2);}
	   if(d2_1<H){bianyan[1][d1_1]=bianyan2[1][d1_1];bianyan[1][d1_2]=bianyan2[1][d1_2];LianJie(1,d2_1,d2_2);}
	   if(d1_1==H)
	   {
		   for(i=d1_2-9;i<d1_2;i++)
		   {
			   chazhi_5=chazhi_5+bianyan[0][i]-bianyan[0][i-1];
			   
		   }
           chazhi_5=chazhi_5/9;
		   for(i=d1_2;i<H-1;i++)bianyan[0][i+1]=bianyan[0][i]+chazhi_5;
	   }
	   if(d2_1==H)
	   {
		   for(i=d2_2-9;i<d2_2;i++)
		   {
			   chazhi_6=chazhi_6+bianyan[1][i]-bianyan[1][i-1];
			   
		   }
           chazhi_6=chazhi_6/9;
		   for(i=d2_2;i<H-1;i++)bianyan[1][i+1]=bianyan[1][i]+chazhi_6;
	   }

	   
	   if(d1_1>d2_1)
	   {
		  if(d1_2>d2_2)
		  {
			 for(i=d1_1;i>=d2_2;i--)center[i]=(bianyan[0][i]+bianyan[1][i])/2;
		  }
		  else
		  {
		     for(i=d1_1;i>=d1_2;i--)center[i]=(bianyan[0][i]+bianyan[1][i])/2;
		  }

	   }
	   if(d1_1<=d2_1)
	   {
          if(d1_2>d2_2)
		  {
		     for(i=d2_1;i>=d2_2;i--)center[i]=(bianyan[0][i]+bianyan[1][i])/2;
		  }
		  else
		  {
		     for(i=d2_1;i>=d1_2;i--)center[i]=(bianyan[0][i]+bianyan[1][i])/2;
		  }
	   }
		}

  }
  /*****************弯道的处理*****************/
  if(xu_xian2==0&&END>0)//shizi==0
  {
    if(END<6)
    {
      for(i=END+15;i>END;i--)
      {
         if(!(bianyan2[0][i]==bianyan[0][i]))bu1_number++;
         if(!(bianyan2[1][i]==bianyan[1][i]))bu2_number++;
      }
            
    }
    else if(END<15)
    {
      for(i=END+20;i>END;i--)
      {
        if((!(bianyan2[0][i]==bianyan[0][i]))||bianyan2[0][i]<2)bu1_number++;
        if((!(bianyan2[1][i]==bianyan[1][i]))||bianyan2[1][i]>V-5)bu2_number++;
      }
    }
    else if(END<20)
    {
      for(i=END+25;i>END;i--)
      {
         if((!(bianyan2[0][i]==bianyan[0][i]))||bianyan2[0][i]<2)bu1_number++;
         if((!(bianyan2[1][i]==bianyan[1][i]))||bianyan2[1][i]>V-5)bu2_number++;
      }
    }
    else
    {
      for(i=H-1;i>END;i--)
      {
              if(bianyan2[0][i]<2)num_1++;
              if(bianyan2[1][i]>V-5)num_2++;
      }
      if(num_1>num_2)
      {
        
        if(bianyan2[0][54]<2&&bianyan2[0][53]<2&&bianyan2[0][52]<2)
        {
                if(END>23&&END<55)
                {
                        if((bianyan2[1][54]-mo_hu[END-24])>=0)center[54]=bianyan2[1][54]-mo_hu[END-24];
                }
        }
        for(i=H-2;i>END;i--)
         {
                if(bianyan2[0][i]<1)break;
                 
         }
        for(;i>END;i--)
        {
                if((center[i+1]+bianyan[1][i]-bianyan[1][i+1])>=0)center[i]=center[i+1]+bianyan[1][i]-bianyan[1][i+1];
            else 
                {
                   for(;i>END;i--)center[i]=0;
                }

        }
      }
      if(num_1<num_2)
      {
         if(bianyan2[1][54]>V-3&&bianyan2[1][53]>V-3&&bianyan2[1][52]>V-3)
        {
                if(END>23&&END<55)
                {
                        if((bianyan2[0][54]+mo_hu[END-24])<V-1)center[54]=bianyan2[0][54]+mo_hu[END-24];
                }
        }
         for(i=H-2;i>END;i--)
         {
                 if(bianyan2[1][i]>=V-4)break;
                 
         }
         for(;i>END;i--)
         {
                 if((center[i+1]+bianyan[0][i]-bianyan[0][i+1])<V)center[i]=center[i+1]+bianyan[0][i]-bianyan[0][i+1];
                 else 
                 {
                         for(;i>END;i--)center[i]=0;
                 }
         }
      }
      
    }
  }
  if(bu1_number>0&&(bu2_number==0||(bu1_number-bu2_number)>15))
  {
    if(END<6)
    {
      for(i=END+15;i>END;i--)
      {
        if(!(bianyan2[0][i]==bianyan[0][i]))break;
      }
      for(;i>END;i--)
      {
        if((center[i+1]+bianyan[1][i]-bianyan[1][i+1])>=0)center[i]=center[i+1]+bianyan[1][i]-bianyan[1][i+1];
        else 
        {
          for(;i>END;i--)center[i]=0;
        }
      }
    }
    else if(END<15)
    {
      for(i=END+20;i>END;i--)
      {
        if((!(bianyan2[0][i]==bianyan[0][i]))||bianyan2[0][i]<2)break;
      }
      for(;i>END;i--)
      {
        if((center[i+1]+bianyan[1][i]-bianyan[1][i+1])>=0)center[i]=center[i+1]+bianyan[1][i]-bianyan[1][i+1];
        else 
        {
                for(;i>END;i--)center[i]=0;
        }
      }
    }
    else if(END<20)
    {
      for(i=END+25;i>END;i--)
      {
        if((!(bianyan2[0][i]==bianyan[0][i]))||bianyan2[0][i]<2)break;
      }
      for(;i>END;i--)
      {
        if((center[i+1]+bianyan[1][i]-bianyan[1][i+1])>=0)center[i]=center[i+1]+bianyan[1][i]-bianyan[1][i+1];
        else 
        {
                for(;i>END;i--)center[i]=0;
        }
      }
    }
    else 
    {
      for(i=H-2;i>END;i--)
      {
        if((!(bianyan2[0][i]==bianyan[0][i]))||bianyan2[0][i]<2)break;
      }
      for(;i>END;i--)
      {
        if((center[i+1]+bianyan[1][i]-bianyan[1][i+1])>=0)center[i]=center[i+1]+bianyan[1][i]-bianyan[1][i+1];
        else 
        {
                for(;i>END;i--)center[i]=0;
        }
      }
    }
  }
	
  if((bu1_number==0||(bu2_number-bu1_number)>15)&&bu2_number>0)
  {
    if(END<6)
    {
      for(i=END+15;i>END;i--)
      {
        if(!(bianyan2[1][i]==bianyan[1][i]))break;
      }
      for(;i>END;i--)
      {
        if((center[i+1]+bianyan[0][i]-bianyan[0][i+1])<V)center[i]=center[i+1]+bianyan[0][i]-bianyan[0][i+1];
        else 
        {
                for(;i>END;i--)center[i]=0;
        }
      }
    }
    else if(END<15)
    {
      for(i=END+20;i>END;i--)
      {
        if((!(bianyan2[1][i]==bianyan[1][i]))||bianyan2[1][i]>V-5)break;
      }
      for(;i>END;i--)
      {
        if((center[i+1]+bianyan[0][i]-bianyan[0][i+1])<V)center[i]=center[i+1]+bianyan[0][i]-bianyan[0][i+1];
        else 
        {
                for(;i>END;i--)center[i]=0;
        }
      }
    }
    else if(END<20)
    {
      for(i=END+25;i>END;i--)
      {
        if((!(bianyan2[1][i]==bianyan[1][i]))||bianyan2[1][i]>V-5)break;
      }
      for(;i>END;i--)
      {
        if((center[i+1]+bianyan[0][i]-bianyan[0][i+1])<V)center[i]=center[i+1]+bianyan[0][i]-bianyan[0][i+1];
        else 
        {
                for(;i>END;i--)center[i]=0;
        }
      }
    }
    else 
    {
      for(i=H-2;i>END;i--)
      {
        if((!(bianyan2[1][i]==bianyan[1][i]))||bianyan2[1][i]>V-5)break;
      }
      for(;i>END;i--)
      {
        if((center[i+1]+bianyan[0][i]-bianyan[0][i+1])<V)center[i]=center[i+1]+bianyan[0][i]-bianyan[0][i+1];
        else 
        {
                for(;i>END;i--)center[i]=0;
        }
      }
    }
  }
  /*****************弯道的处理END*****************/
  /*****************长直弯与十字交叉快要出弯的识别与补线****************/
  m=0;
  if(shizi==0&&xu_xian2==0&&END<6)
  {
    if(bianyan[0][54]<2)
    {
      for(i=H-1;i>=0;i--)
      {
              if(bianyan[0][i]>=2)break;
      }
      for(;i>=0;i--)
      {
              if(bianyan[0][i]<2)
              {
                      m=i;
                      break;
              }
      }
      for(;i>=0;i--)
      {
              if(bianyan[0][i]>=2)break;
      }
      if(m-i>15)
      {
        for(n=m;n>i;n--)
        {
                if(!(bianyan2[1][n]==bianyan[1][n]))numx++;
        }
        if(numx<5)
        {
          for(n=m;n>i;n--)
          {
                  if(bianyan[1][n]-bianyan[1][n+1]<0)
                  {
                     if(center[n+1]+bianyan[1][n]-bianyan[1][n+1]>=0)center[n]=center[n+1]+bianyan[1][n]-bianyan[1][n+1];
                     else
                     {
                             for(;n>=0;n--)center[n]=0;
                             break;
                     }
                  }
                  else
                  {
                    for(;n>=0;n--)
                    {
                      if(center[n+1]+center[n+1]-center[n+2]>=0)center[n]=center[n+1]+center[n+1]-center[n+2];
                      else
                      {
                              for(;n>=0;n--)center[n]=0;
                              
                      }
                    }
               }
          }
        }
      }
    }
    if(bianyan[0][54]>=2)
    {
      for(i=H-1;i>=0;i--)
      {
              if(bianyan[0][i]<2)
              {
                      m=i;
                      break;
              }
      }
      for(;i>=0;i--)
      {
              if(bianyan[0][i]>=2)break;
      }
      if(m-i>15)
      {
        for(n=m;n>i;n--)
        {
                if(!(bianyan2[1][n]==bianyan[1][n]))numx++;
        }
        if(numx<5)
        {
          for(n=m;n>i;n--)
          {
            if(bianyan[1][n]-bianyan[1][n+1]<0)
            {
               if(center[n+1]+bianyan[1][n]-bianyan[1][n+1]>=0)center[n]=center[n+1]+bianyan[1][n]-bianyan[1][n+1];
           else
               {
                       for(;n>=0;n--)center[n]=0;
                       break;
               }
            }
            else
            {
                    for(;n>=0;n--)
                    {
                            if(center[n+1]+center[n+1]-center[n+2]>=0)center[n]=center[n+1]+center[n+1]-center[n+2];
                            else
                            {
                                    for(;n>=0;n--)center[n]=0;
                                    
                            }
                    }
            }
          }
        }
      }
    }
    m=0;
    numx=0;
    if(bianyan[1][54]>V-4)
    {
      for(i=H-1;i>=0;i--)
      {
              if(bianyan[1][i]<=V-4)break;
      }
      for(;i>=0;i--)
      {
              if(bianyan[1][i]>V-4)
              {
                      m=i;
                      break;
              }
      }
      for(;i>=0;i--)
      {
              if(bianyan[1][i]<=V-4)break;
      }
      if(m-i>15)
      {
        for(n=m;n>i;n--)
        {
                if(!(bianyan2[0][n]==bianyan[0][n]))numx++;
        }
        if(numx<5)
        {
                for(n=m;n>i;n--)
                {
                        if(bianyan[0][n]-bianyan[0][n+1]>0)
                        {
                           if(center[n+1]+bianyan[0][n]-bianyan[0][n+1]<V)center[n]=center[n+1]+bianyan[0][n]-bianyan[0][n+1];
                           else
                           {
                                   for(;n>=0;n--)center[n]=0;
                                   break;
                           }
                        }
                        else
                        {
                                for(;n>=0;n--)
                                {
                                        if(center[n+1]+center[n+1]-center[n+2]<V)center[n]=center[n+1]+center[n+1]-center[n+2];
                                        else
                                        {
                                                for(;n>=0;n--)center[n]=0;
                                                
                                        }
                                }
                        }
                }
        }
      }
    }
    if(bianyan[1][54]<=V-4)
    {
      for(i=H-1;i>=0;i--)
      {
              if(bianyan[1][i]>V-4)
              {
                      m=i;
                      break;
              }
      }
      for(;i>=0;i--)
      {
              if(bianyan[0][i]<=V-4)break;
      }
        if(m-i>15)
        {
          for(n=m;n>i;n--)
          {
                  if(!(bianyan2[0][n]==bianyan[0][n]))numx++;
          }
            if(numx<5)
            {
              for(n=m;n>i;n--)
              {
                      if(bianyan[0][n]-bianyan[0][n+1]>0)
                      {
                         if(center[n+1]+bianyan[0][n]-bianyan[0][n+1]<V)center[n]=center[n+1]+bianyan[0][n]-bianyan[0][n+1];
                         else
                         {
                                 for(;n>=0;n--)center[n]=0;
                                 break;
                         }
                      }
                      else
                      {
                              for(;n>=0;n--)
                              {
                                      if(center[n+1]+center[n+1]-center[n+2]<V)center[n]=center[n+1]+center[n+1]-center[n+2];
                                      else
                                      {
                                              for(;n>=0;n--)center[n]=0;
                                              
                                      }
                              }
                      }
              }
            }
        }
      }
  }
    /*****************长直弯与十字交叉快要出弯的识别与补线END****************/
  if(END<8)
	{
		for(i=0;i<H;i++)
		{
			if(center[i]>0)break;
		}
		for(m=H-3;m>i;m--)
		{
			chazhi_center_1=center[m]-center[m+1];
			chazhi_center_2=center[m+1]-center[m+2];
			chazhi_center_1=abs(chazhi_center_1);
			chazhi_center_2=abs(chazhi_center_2);
			if((chazhi_center_1-chazhi_center_2)>8)
			{
				n_1=m;
				n=m;
				for(;n>i;n--)
				{
					if((bianyan[0][n]==bianyan2[0][n])&&(bianyan[1][n]==bianyan2[1][n]))break;
				}
				for(m=m+1;m<H;m++)
				{
					if((bianyan[0][m]==bianyan2[0][m])&&(bianyan[1][m]==bianyan2[1][m]))break;
				}
				if(n>i&&(m-n)>1){LianJie_center(m,n);break;}
				else break;
			}
		}
		if(n_1&&(m-n)==1)
		{
			for(;m<H;m++)
			{
				if(!(bianyan2[0][m]==bianyan[0][m]))num_3++;
                if(!(bianyan2[1][m]==bianyan[1][m]))num_4++;
				
			}
			if(num_3<5&&num_4<5)
			{
				for(i=n_1;i<H;i++)
				{
                    if(bianyan[0][i]==0)num_5++;
					if(bianyan[1][i]>V-4)num_6++;
				}
				if(num_5<num_6)
				{
					if(num_6>6)
					{
						LianJie_center((n_1+num_6/3),n_1);
					}
				}
				if(num_5>num_6)
				{
					if(num_5>6)
					{
                        LianJie_center((n_1+num_5/3),n_1);
					}
				}
			}
		}
		
	}
  END_0=END;
  /*************十字交叉系统识别*************/
  if(shizi==0&&END<15)
  {
    xie_shizi();
    if(xie_shizi_flag_1==1||xie_shizi_flag_2==1)
    {
      END=END_Line;
      END_0=END;
      xie_shizi_flag_1=0;
      xie_shizi_flag_2=0;
      END_Line=0;
    }
  }
  /*************十字交叉系统识别END*************/
  /***************边沿倾斜过于严重的情况***************/
	
  if(END<18)
  {
    if(END>12)
    {
      if(bianyan[0][END+1]>130)
      {
        if((bianyan[0][END+1]-bianyan[0][54])>100)
        {
          for(i=53;i>=0;i--)
          {
            if(bianyan[1][i]>V-4)
            {
              if(center[i+1]+bianyan[0][i]-bianyan[0][i+1]<V)center[i]=center[i+1]+bianyan[0][i]-bianyan[0][i+1];
              else
              {
                for(;i>=0;i--)center[i]=0;
              }
            }
            else
            {
                    center[i]=(bianyan[0][i]+bianyan[1][i])/2;
            }
          }
        }
      }
      if(bianyan[1][END+1]<30)
      {
        if((bianyan[1][END+1]-bianyan[1][54])<-100)
        {
          for(i=53;i>=0;i--)
          {
            if(bianyan[0][i]<2)
            {
              if(center[i+1]+bianyan[1][i]-bianyan[1][i+1]>=0)center[i]=center[i+1]+bianyan[1][i]-bianyan[1][i+1];
              else
              {
                for(;i>=0;i--)center[i]=0;
              }
            }
            else
            {
                    center[i]=(bianyan[1][i]+bianyan[0][i])/2;
            }
          }
        }
      }
    }
    else
    {
      if((bianyan[0][END+1]-bianyan[0][54])>100)
      {
        for(i=53;i>=0;i--)
        {
          if(bianyan[1][i]>V-4)
          {
            if(center[i+1]+bianyan[0][i]-bianyan[0][i+1]<V)center[i]=center[i+1]+bianyan[0][i]-bianyan[0][i+1];
            else
            {
                    for(;i>=0;i--)center[i]=0;
            }
          }
          else
          {
                  center[i]=(bianyan[0][i]+bianyan[1][i])/2;
          }
        }
      }
      if((bianyan[1][END+1]-bianyan[1][54])<-100)
      {
        for(i=53;i>=0;i--)
        {
          if(bianyan[0][i]<2)
          {
            if(center[i+1]+bianyan[1][i]-bianyan[1][i+1]>=0)center[i]=center[i+1]+bianyan[1][i]-bianyan[1][i+1];
            else
            {
                    for(;i>=0;i--)center[i]=0;
            }
          }
          else
          {
                  center[i]=(bianyan[1][i]+bianyan[0][i])/2;
          }
        }
      }
    }
  }
  /***************边沿倾斜过于严重的情况END***************/
  /********************************************
  *踩线补线有漏洞导致最后一个点center[0]被漏掉了
  *故在这里补上,为避免麻烦这里我们就使用上一次的
  *边沿位置值
  
  *********************************************/
  if(center[1])
  {
    if(bianyan[0][0]&&bianyan[1][0])
    {
      bianyan[1][0]=bianyan[1][1];
      bianyan[0][0]=bianyan[0][1];
      bianyan2[1][0]=bianyan[1][1];
      bianyan2[0][0]=bianyan[0][1];
      center[0]=(bianyan[1][0]+bianyan[0][0])/2;
    }
  }
}




/************************主函数****************************/


  uint16 Judge_startline2(){
  int16 i=54,j=0,black_max=0,k=0;
  int16 white_position=0,white_num=0,start_place=0,start_black_num=0;
  int16 bianyan_startline[2][55]={0};
  uint8 endl=0,startl=0,endr=0,startr=0;
  uint8  bianyan1_old=0,bianyan2_old=0;
  int last_hang=0;

  //首先以一个很小框框的范围找到两边边沿，这样避免因为起始影响边沿的寻找
  i=54;
  while(i>=0){
  if(i>35){
  black_max=160;
  }else
  if(i>20){
  black_max=158;
  }else
  if(i>10){
   black_max=157;
  }else{
  black_max=156;
  }
 
 
 
    if(i==54){
    for(j=0;j<160;j++){
       if(video[i][j]>black_max){
          white_position+=j;
          white_num++;
		 
       } 
	   //str.Format("white_position:%d\r\n\r\n",white_position);
	 // PrintDebug(str);
    }
    if(white_num){ 
    start_place=white_position/white_num;  
    }else{
        start_place=80;
    }
	//str.Format("start_place:%d\r\n\r\n",start_place);
	 // PrintDebug(str);
    }
    

     if(i==54){
       startl=start_place;
       endl=2;
       startr=start_place;
       endr=157;
     }else{
       startl=limit(bianyan1_old+5,157,1);
       endl=limit(bianyan1_old-5,157,1);
       startr=limit(bianyan2_old-5,157,1);
       endr=limit(bianyan2_old+5,157,1);
     }

    
     
     
    for(j=startl;j>0;j--){
       if(video[i][j+2]>=black_max&&video[i][j+1]>=black_max&&video[i][j]<black_max&&video[i][j-1]){
           bianyan_startline[0][i]=j;
           bianyan1_old=j;
		   
           break;
       }
       
       if(j==endl){
		   
          bianyan_startline[0][i]=0;
          
        break;  
       }
      
    }
   
   
    for(j=startr;j<158;j++){
       if(video[i][j-2]>=black_max&&video[i][j-1]>=black_max&&video[i][j]<black_max&&video[i][j+1]){
           bianyan_startline[1][i]=j;
           bianyan2_old=j;
		   
           break;
       }
       
       if(j==endr){
		  
          bianyan_startline[1][i]=0;
         
        break;  
       }
      
    }
    
  i--;
  }
/////边沿寻找完毕

///在边沿两边之间寻找黑色的点，确定寻找起始线行的范围
for(i=54;i>=0;i--){
	if(!(bianyan_startline[0][i]&&bianyan_startline[1][i])){
		last_hang=i+1;
		 
	
		break;
	}
}
if(last_hang<20)
  last_hang=20;
  
  black_max=155;
  if(last_hang<=46){
for(j=bianyan_startline[0][54]+1;j<bianyan_startline[1][54];j++){
for(i=54;i>last_hang;i--){
	if(j>bianyan_startline[0][i]&&j<bianyan_startline[1][i]){
		if(video[i][j]<black_max){
            
            start_black_num++;
		}
	}else{
		break;
	}
}
}
  }


return start_black_num;
}



void main(void)
{  
  uint8 start_xian_3=0;
  int16 track[30]={-1},track_2[30]={0},track_number=0;
  uint8 j=0,j_num=0;
  //uint8 m=0;
  uint8 P_stop=0;//停车关速度控制标志位
  int park=-100,xu_xian2_P=-100;
  uint8 i,Hong_Wai=0,direct=0,LCD_begin=0;
  int8 judge=0,set=0;       
  int16  exspeed=0,pwm_speed=0;
  uint16 slope_flag1=0,slope_flag2=0,slope_flag3=0,slope_flag4=0,slope_flag5=0;
  uint16 startline_black_num=0,startline_flag=0;
  uint16 changshu=0;
  int startline_time=0;
  DisableInterrupts;
  pllinit180M();
  IO_Init();
  LCD_Init();  
  hw_FTM1_init();
  hw_FTM0_init();
  FTM2_QUAD_Init();
  JIANPAN_ini();
      gpio_init(PORTA,14,1,1);
      gpio_init(PORTA,15,1,1);
      gpio_init(PORTA,16,1,1);
      gpio_init(PORTA,17,1,1);
  CH451_ini();
  CH451_WriteCommand(CH451_BCD);
  speed_Very_straight=70;speed_top=60;speed_b=50;speed_m=40;speed_s=30 ;speed_ms=80;
  LCD_P6x8Str(0,0,"speed");     //英文字符串显示
   LCD_P6x8Str(0,1,"speed_cut_timer"); 
   LCD_P6x8Str(0,7,"start_startline");  
  Car_Speed();
  
  
  while(KeyValue!=50)
  {
    KeyValue=50;
    CH451_GetKeyValue();
  }
  /****************1直接发车，不调节PD，，，2调节PD后发车*******************/
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
  DMA0_Init();
  //hw_pit_init(1,16000);         //pit1中断初始化
  //enable_irq(89);             //开c9口红外遥控停车中断
  //enable_pit_interrupt(1);    //使能PIT1中断，，，用于红外计数
  delays(2);              //延时发车
  LCD_CLS();
  
  hw_pit_init(PIT2,6000000);//100ms
  enable_pit_interrupt(PIT2);
  
  EnableInterrupts;
  for(;;) 
  {	
    
    if(finish==1)
    { 
      
      if(P_timer>=speed_cut_timer&&P_timer<speed_cut_timer+50&&!(speed_cut_timer==100)){
    speed_Very_straight=118;speed_top=108;speed_b=98;speed_m=88;speed_s=82; speed_ms=70 ;
     //speed_Very_straight=70;speed_top=70;speed_b=70;speed_m=70;speed_s=70; speed_ms=70 ;
    }else{
   switch(car_speed)
  {
     case 0:speed_Very_straight=80;speed_top=80;speed_b=80;speed_m=80;speed_s=80; speed_ms=70;break;
  case 1:speed_Very_straight=90;speed_top=90;speed_b=90;speed_m=90;speed_s=90; speed_ms=90;break;
  //case 1:speed_Very_straight=110;speed_top=100;speed_b=90;speed_m=80;speed_s=75; speed_ms=70 ;break;
  case 2:speed_Very_straight=113;speed_top=103;speed_b=93;speed_m=83;speed_s=78; speed_ms=70 ;break;
  case 3:speed_Very_straight=115;speed_top=105;speed_b=95;speed_m=85;speed_s=80; speed_ms=70 ;break;
  case 4:speed_Very_straight=118;speed_top=108;speed_b=98;speed_m=88;speed_s=82; speed_ms=70 ;break;
  case 5:speed_Very_straight=122;speed_top=108;speed_b=98;speed_m=88;speed_s=82; speed_ms=70 ;break;
  case 6:speed_Very_straight=124;speed_top=110;speed_b=100;speed_m=90;speed_s=84; speed_ms=70 ;break;
  case 7:speed_Very_straight=124;speed_top=110;speed_b=104;speed_m=94;speed_s=88; speed_ms=70 ;break;
  case 8:speed_Very_straight=125;speed_top=112;speed_b=104;speed_m=94;speed_s=88; speed_ms=70 ;break;
  case 9:speed_Very_straight=125;speed_top=114;speed_b=106;speed_m=92;speed_s=82; speed_ms=70 ;break;
  case 10:speed_Very_straight=125;speed_top=116;speed_b=108;speed_m=94;speed_s=84; speed_ms=73 ;break;
  case 11:speed_Very_straight=130;speed_top=116;speed_b=110;speed_m=95;speed_s=88; speed_ms=75 ;break;
  case 12:speed_Very_straight=140;speed_top=116;speed_b=112;speed_m=97;speed_s=90; speed_ms=70 ;break;
  case 13:speed_Very_straight=140;speed_top=118;speed_b=114;speed_m=99;speed_s=93; speed_ms=70;break;
  case 14:speed_Very_straight=145;speed_top=117;speed_b=115;speed_m=95;speed_s=93; speed_ms=70;break;
  case 15:speed_Very_straight=103;speed_top=100;speed_b=98;speed_m=96;speed_s=94; speed_ms=70;break;
  case 16:speed_Very_straight=106;speed_top=103;speed_b=101;speed_m=99;speed_s=97; speed_ms=70;break;
  case 17:speed_Very_straight=110;speed_top=106;speed_b=103;speed_m=100;speed_s=97; speed_ms=70;break;
  case 18:speed_Very_straight=95;speed_top=95;speed_b=95;speed_m=95;speed_s=95; speed_ms=70;break;
  case 19:speed_Very_straight=100;speed_top=100;speed_b=100;speed_m=100;speed_s=100; speed_ms=70;break;
  }
    } 
    
    
      if(P_timer>start_startline){
      startline_black_num=Judge_startline2();
      }
      find_bianyan();  //找两边黑线
      if(P_timer<30||P_timer>170)Start_Line();
      X_LineBend();
      //m=judge_locus_0();
      //if(m!=20)center_filter();//加权平均法使中心线变得平滑
      center_filter();
      m=judge_locus_0();
      sai_dao_lei_xing=m;
    slope_flag5=slope_flag4;  
    slope_flag4=slope_flag3;
    slope_flag3=slope_flag2;
    slope_flag2=slope_flag1;
    slope_flag1=Judge_slope();
    if(slope_flag1||slope_flag2||slope_flag3||slope_flag4||slope_flag5){
      slope_flag=1;
    }
      /************再次检测起始线以免是起始线误判成不是*************/
      if(start_18_25_2==1||start_25_35_2==1||start_35_45_2==1||start_45_55_2==1)
      {
        if(m==20)
        {
          start_xian_3=1;
        }
      }
      if(start_xian_3==0)
      {
        if(start_line2==1&&sai_dao_lei_xing==20)start_xian_3=1;
      }
      if(start_xian==1&&m!=20)start_xian=0;
	/************再次检测起始线以免是起始线误判成不是*END************/
      /****************停车控制***************/
      if(xu_xian2_P==-100)          //如果检测到虚线，那么在检测到虚线后的1s内不让车停车
      {
        if(xu_xian2==1)
        {
          xu_xian2_P=P_timer;
        }
      }
      if(P_timer==xu_xian2_P+10)xu_xian2_P=-100;
      if(P_timer<30||xu_xian2_P!=-100){start_xian=0;start_xian_3=0;}//发车，只检测，不停车||m!=9||m!=20
      /*************记录最近30场的赛道类型*************/
      if(track_number==30)track_number=0;
      if(ramp_flag==1)track_2[track_number]=40;                    //40在这里代表坡道
      if(start_xian==1) track_2[track_number]=100;             //100在这里表示采集到了起始线
      if(start_xian!=1&&start_xian_3==1) track_2[track_number]=80;//80代表有可能是起始线
      if(xu_xian2==1)track_2[track_number]=60;                     //60代表是虚线
      
      if(start_xian!=1&&start_xian_3!=1&&xu_xian2!=1&&ramp_flag!=1)track_2[track_number]=0;
      track[track_number]=m;
      track_number++;
      
      /*************记录最近30场的赛道类型END*************/
      /*************是否停车的判断**************
      *我的停车判断方法：看最近5场的图像类型  肯定是起始线100，可能是起始线80
      *1,一场肯定是起始线的，就判定为起始线
      *2，两场以上可能是起始线的，就判定为起始线
      *3，
      *4，
      *****************************************/
      if(park==-100)
      {
        if(track_2[track_number-1]==100)
        {
          start_xian==0;
          park=P_timer;
        }
        if(track_2[track_number-1]==80)
        {
          uint8 start_num_3=0;
          if(track_number==1)
          {
            if(track_2[29]==80)start_num_3++;
            if(track_2[28]==80)start_num_3++;
            if(track_2[27]==80)start_num_3++;
            if(track_2[26]==80)start_num_3++;
          }
          if(track_number==2)
          {
            if(track_2[0]==80)start_num_3++;
            if(track_2[29]==80)start_num_3++;
            if(track_2[28]==80)start_num_3++;
            if(track_2[27]==80)start_num_3++;
          }
          if(track_number==3)
          {
            if(track_2[1]==80)start_num_3++;
            if(track_2[0]==80)start_num_3++;
            if(track_2[29]==80)start_num_3++;
            if(track_2[28]==80)start_num_3++;
          }
          if(track_number==4)
          {
            if(track_2[2]==80)start_num_3++;
            if(track_2[1]==80)start_num_3++;
            if(track_2[0]==80)start_num_3++;
            if(track_2[29]==80)start_num_3++;
          }
          if(track_number>4)
          {
            if(track_2[track_number-2]==80)start_num_3++;
            if(track_2[track_number-3]==80)start_num_3++;
            if(track_2[track_number-4]==80)start_num_3++;
            if(track_2[track_number-5]==80)start_num_3++;
          }
          if(start_num_3>=1)
          {
            park=P_timer;
          }
        }
      }
    
      if(P_timer==park+3)
      {
        P_stop=1;
        //park=-100;
      }
      if(xu_xian2_P==-100){
      if(startline_black_num>32&&P_timer>20&&!(slope_flag1||slope_flag2||slope_flag3||slope_flag4||slope_flag5))
      {
        startline_flag=1;
      startline_time=P_timer;
      }
      }
      if(startline_flag){
        changshu++;
      }
      if(startline_flag&&changshu>8)//P_timer>startline_time+1)
      //if(P_stop==1&&!(slope_flag1||slope_flag2||slope_flag3||slope_flag4||slope_flag5))
      { 
        //FTM0_C3V=0;
        if(pulse>20)
        {
          FTM0_C3V=375;          //300   停车倒转
          FTM0_C0V=0;
        }
        else
        {
          FTM0_C3V=0;
          FTM0_C0V=0;
          FTM0_C1V=0;
        }
        
        
      }
                    
    // if(P_timer>startline_time+15&&!startline_time)    
     // {
       // FTM0_C3V=0;
        //FTM0_C0V=0;
       // FTM0_C1V=0;
        
     // }
      
      /*************是否停车的判断*END*************/
      /****************停车控制END***************/
      /**************稳定在很直的长直道判断***************/
      if(track[track_number-1]==20)
      {
        if(track_number==1)
        {
          if(track[29]==20&&track[28]==20)Very_straight=1;
          
        }
        else if(track_number==2)
        {
          if(track[0]==20&&track[29]==20)Very_straight=1;
        }
        else 
        {
          if(track[track_number-2]==20&&track[track_number-3]==20&&track[track_number-4]==20)Very_straight=1;;
        }
      }
      /**************稳定在很直的长直道判断***************/            
    set=choose_PD();  
    
    
    
    
    
    //if(P_stop==0)
    if(!(startline_flag&&changshu>8))
    {
      if(P_timer<10)pwm_speed=150;
      
      //发车前1.3s给满占空比，使其快速发车
      else
      {
        if(Very_straight==0)
        {
          exspeed=speed_control();
        }
        if(Very_straight==1)
        {
          Very_straight=0;
          exspeed=speed_Very_straight;
        }
        pwm_speed=Motor_control(exspeed);
      }
      
      
      set_speed(pwm_speed);
    }
    
    hw_pit_init(PIT0,800000);//20ms,1200000,30ms,1800000  注意这个初始化一定要放在大的for循环里面，使用定时器，这样才是在固定的时间里面得到的脉冲数
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
        LCD_P6x8Str_3(70,0,sai_dao_lei_xing);//显示赛道类型
         LCD_P6x8Str_3(100,0,i_old);
        LCD_P6x8Str(1,1,"mid_0");     //英文字符串显示
        LCD_P6x8Str_3(45,1,mid_now_0);
        if(mid_now)LCD_P6x8Str_1(70,1,mid_0);
        LCD_P6x8Str_1(100,1,xiaoS_flag);
        LCD_P6x8Str(1,2,"high_0");     //英文字符串显示
        LCD_P6x8Str_3(45,2,high_now_0);
        LCD_P6x8Str_3(80,2,startline_black_num);
        if(high_now)LCD_P6x8Str_1(70,2,high_0);
        LCD_P6x8Str(1,3,"position_now");     //英文字符串显示
        LCD_P6x8Str_3(75,3,position_now);
        LCD_P6x8Str(1,4,"position_diff");     //英文字符串显示
        LCD_P6x8Str_FuHao_3(88,4,position_diff);
        LCD_P6x8Str(1,5,"p_low");     //英文字符串显示
        LCD_P6x8Str_FuHao_3(30,5,position_now_low);
        LCD_P6x8Str(1,6,"see_distance");     //英文字符串显示
        LCD_P6x8Str_FuHao_3(100,6,see_distance);
        LCD_P6x8Str(1,7,"P_D");     //英文字符串显示
        LCD_P6x8Str_FuHao_3(30,7,position_now_P);
         LCD_P6x8Str_FuHao_3(60,7,position_now_D);
        //LCD_P6x8Str_FuHao_3(30,7,Servo.P);
         //LCD_P6x8Str_FuHao_3(60,7,Servo.D);
         // LCD_P6x8Str_FuHao_3(100,7,position_now_low_P);
        //LCD_P6x8Str(1,5,"Servo_value");     //英文字符串显示Servo_value
        LCD_P6x8Str_5(80,5,Servo_value);
        //LCD_P6x8Str_3(1,6,start_xian);
        //LCD_P6x8Str_3(20,6,start_xian_3);
        //LCD_P6x8Str_3(40,6,ramp_flag);
        
        //LCD_P6x8Str_3(60,6,topline);
        //LCD_P6x8Str(1,7,"pwm_speed");     //英文字符串显示
        //LCD_P6x8Str_3(58,7,pwm_speed);
        //LCD_P6x8Str_3(80,7,pulse);
      }
    }
    if(KeyValue==6)
    {
      if(LCD_P3==250)LCD_P3=0;
      if(++LCD_P3%10)
      {
        LCD_CLS();
        
        for(i=0;i<H-1;i++)
        {
           LCD_PutPixel(center[i],i);
        }
       }
     }
                  
      /***************对一些变量进行清零****************/
      position_now_low=0;
     // position_now=0;
      position_diff=0;
      xie_shizi_flag_1=0;
      xie_shizi_flag_2=0;
      xie_shizi_flag_3=0;   //用来表示只找到一个拐点的
      xu_xian2=0;
      xu1=0;
      xu2=0;
      xu3=0;
      xu4=0;   
      for(i=0;i<H;i++)
      {
        bianyan2[0][i]=0;
        bianyan[0][i]=0;
        bianyan2[1][i]=0;
        bianyan[1][i]=0;
        center[i]=0;
      }
      for(i=0;i<142;i++)BW4[i]=0;
      for(i=0;i<82;i++)BW3[i]=0;
      for(i=0;i<122;i++)BW2[i]=0;
      for(i=0;i<102;i++)BW[i]=0;
      start_18_25_2=0;
      start_25_35_2=0;
      start_35_45_2=0;
      start_45_55_2=0;
      start_xian_3=0;
      start_line2=0;
      ramp_flag=0;
      finish=0;
      xiaoS_flag=0;
      /***************对一些变量进行清零END****************/
      /********************LCD数据显示模块end*********************/
      
    }
  }	
	//return 0;
}
/************************主函数END****************************/

