//keyboard.h

#define CH451_RESET     0x0201             //复位
#define CH451_LEFTMOV   0x0300             //设置移动方式-左移
#define CH451_LEFTCYC   0x0301             //设置移动方式-左循
#define CH451_RIGHTMOV  0x0302             //设置移动方式-右移
#define CH451_RIGHTCYC  0x0303             //设置移动方式-右循 
#define CH451_SYSOFF    0x0400             //关显示、键盘、看门狗
#define CH451_SYSON1    0x0401             //开显示
#define CH451_KEYEN     0x0402             //开键盘
#define CH451_SYSON2    0x0403             //开显示、键盘
#define CH451_SYSON3    0x0407             //开显示、键盘、看门狗功能
#define CH451_DSP       0x0500             //设置默认显示方式
#define CH451_BCD       0x0580             //设置BCD译码方式
#define CH451_TWINKLE   0x0600             //设置闪烁控制
#define CH451_KEY       0x0700             //读键盘代码
#define CH451_DIG0      0x0800             //数码管位0显示
#define CH451_DIG1      0x0900             //数码管位1显示 
#define CH451_DIG2      0x0A00             //数码管位2显示
#define CH451_DIG3      0x0B00             //数码管位3显示
#define CH451_DIG4      0x0C00             //数码管位4显示
#define CH451_DIG5      0x0D00             //数码管位5显示 
#define CH451_DIG6      0x0E00             //数码管位6显示 
#define CH451_DIG7      0x0F00             //数码管位7显示

//#define CH451_Load GPIOA[2]          // 3    1
//#define CH451_IN   GPIOA[1]          // 2    0
//#define CH451_CLK  GPIOA[0]         // 1    2
//#define CH451_Out     GPIOA_PDIR//GPIOA[3]          // 4    3
/*void gpio_ctrl (GPIO_MemMapPtr port, int index, int data)
{
    if(data == 1)//output
         GPIO_PDOR_REG(port) |= (1<<index);
    else
         GPIO_PDOR_REG(port) &= ~(1<<index);
}*/

uint8 KeyCode[16]={0x40,0x41,0x42,0x43,//0,1,2,3
                   0x48,0x49,0x4A,0x4B,//4,5,6,7
                   0x50,0x51,0x52,0x53,//8,9,A,B
                   0x58,0x59,0x5A,0x5B};//C,D,E,F
uint8 KeyValue=50;
uint8 GetKeyValue=0;


void Delay(unsigned int num){
  unsigned int n;
  for (n=0;n<num*2;n++);
}

void CH451_WriteCommand(uint32 Command){        //向CH451写命令
 uint8 k=12;
 //CH451_Load=0;
gpio_ctrl(PORTA,2,0); 
 for(k=0;k<12;k++){
  //CH451_IN=Command&1;
   gpio_ctrl(PORTA,1,Command&1);
 // CH451_CLK=0;
   gpio_ctrl(PORTA,0,0);
  Command>>=1;                   //数据右移一位
  //CH451_CLK=1;//上升沿有效
  gpio_ctrl(PORTA,0,1);
 }
 //CH451_Load=1;
 gpio_ctrl(PORTA,2,1);
// Delay(2);
// CH451_Load=1;
}

void CH451_GetKeyValue(void){
    uint8 k;
    uint8 Command=0x07;               //读取键值命令
    GetKeyValue=0x00;           //键盘代码
    //CH451_Load=0;
    gpio_ctrl(PORTA,2,0);
                        //命令开始       
    for(k=0;k<4;k++){                 //送入4位数据，低位在前
      //CH451_IN=Command&0x0001; 
       gpio_ctrl(PORTA,1,Command&0X0001);
      //CH451_CLK=0;
        gpio_ctrl(PORTA,0,0);
      Command>>=1;
      //CH451_CLK=1;                   //上升沿有效
       gpio_ctrl(PORTA,0,1);
    }
   // CH451_Load=1;                     //上升沿加载数据
    gpio_ctrl(PORTA,2,1);    
    for(k=0;k<7;k++){
        GetKeyValue<<=1;
        GetKeyValue|=gpio_Get(PORTA,3);//CH451_Out;
        //CH451_CLK=1;               //下降沿有效
        gpio_ctrl(PORTA,0,1);
        //CH451_CLK=0;
        gpio_ctrl(PORTA,0,0);
    }
    
    for(k=0;k<16;k++){                //查找键盘代码对应的键值
     if(GetKeyValue==KeyCode[k]){
      KeyValue=k;
      break;
      }
    }
}

void CH451_ini(void){
   // CH451_IN=1;
     gpio_ctrl(PORTA,1,1);
   // CH451_IN=0;                        //低电平使能串行接口
     gpio_ctrl(PORTA,1,0);
    //CH451_IN=1;
     gpio_ctrl(PORTA,1,1);
    
    CH451_WriteCommand(CH451_RESET);    //复位CH451
    CH451_WriteCommand(CH451_SYSON2); 
}
//键盘控制端口初始化
void JIANPAN_ini(void){
        PORTA_PCR1=PORT_PCR_MUX(1);
        PORTA_PCR2=PORT_PCR_MUX(1);
        PORTA_PCR3=PORT_PCR_MUX(1);
        PORTA_PCR4=PORT_PCR_MUX(1);//A1,A2,A3,A0设置为GPIO模式
        gpio_init(PORTA,0,1,1);
        gpio_init(PORTA,1,1,1);
        gpio_init(PORTA,2,1,1);
        gpio_init(PORTA,3,0,1);
        //GPIOA_PDDR&=0XFFFFFFF7;   //A3输入
        //GPIOA_PDDR|=0X00000007;  //A0，A1，A2输出
        //GPIOA_PCOR|=0X0000000F;   //A1,A2,A3,A0设置为高电平
        //GPIOA_PDIR|=0X00000008;

  //DDRB=0xf7; 
  //PORTB=0xff;
}

void print_d(int num){  //下显示
int b1[4]={0};
b1[0]=num%10;
b1[1]=num%100/10;
//b1[2]=num/100%10;
//b1[3]=num/1000;
CH451_WriteCommand(CH451_DIG0|b1[0]);    //0位显示1
CH451_WriteCommand(CH451_DIG1|b1[1]);
//CH451_WriteCommand(CH451_DIG2|b1[2]);
//CH451_WriteCommand(CH451_DIG3|b1[3]);
}                  

void print_d1(int num){          
int b1[4]={0};
b1[0]=num%10;
b1[1]=num%100/10;
//b1[2]=num/100%10;
//b1[3]=num/1000;
//CH451_WriteCommand(CH451_DIG0|b1[0]);    //0位显示1
//CH451_WriteCommand(CH451_DIG1|b1[1]);
CH451_WriteCommand(CH451_DIG2|b1[0]);
CH451_WriteCommand(CH451_DIG3|b1[1]);
}

void print_dall(int num){
int b1[4]={0};
b1[0]=num%10;
b1[1]=num%100/10;
b1[2]=num/100%10;
b1[3]=num/1000;
CH451_WriteCommand(CH451_DIG0|b1[0]);    //0位显示1
CH451_WriteCommand(CH451_DIG1|b1[1]);
CH451_WriteCommand(CH451_DIG2|b1[2]);
CH451_WriteCommand(CH451_DIG3|b1[3]);
}


void print_t(int num){  //上显示
int b1[4]={0};
b1[0]=num%10;
b1[1]=num%100/10;
//b1[2]=num/100%10;
//b1[3]=num/1000;
CH451_WriteCommand(CH451_DIG4|b1[0]);    //0位显示1
CH451_WriteCommand(CH451_DIG5|b1[1]);
//CH451_WriteCommand(CH451_DIG6|b1[2]);
//CH451_WriteCommand(CH451_DIG7|b1[3]);
}

void print_tall(int num){  //上显示
int b1[4]={0};
b1[0]=num%10;
b1[1]=num%100/10;
b1[2]=num/100%10;
b1[3]=num/1000;
CH451_WriteCommand(CH451_DIG4|b1[0]);    //0位显示1
CH451_WriteCommand(CH451_DIG5|b1[1]);
CH451_WriteCommand(CH451_DIG6|b1[2]);
CH451_WriteCommand(CH451_DIG7|b1[3]);
}

void print_t1(int num){
int b1[4]={0};
b1[0]=num%10;
b1[1]=num%100/10;
//b1[2]=num/100%10;
//b1[3]=num/1000; 
CH451_WriteCommand(CH451_DIG6|b1[0]);
CH451_WriteCommand(CH451_DIG7|b1[1]);
}


void print_hell(){
CH451_WriteCommand(CH451_DIG4|0x38);
CH451_WriteCommand(CH451_DIG5|0x38);
CH451_WriteCommand(CH451_DIG6|0x0e);
CH451_WriteCommand(CH451_DIG7|0x77);
}

void print_fall(int num){
int b1[4]={0};
b1[0]=num%10;
b1[1]=num%100/10;
b1[2]=num/100%10;
CH451_WriteCommand(CH451_DIG0|b1[0]);    //0位显示1
CH451_WriteCommand(CH451_DIG1|b1[1]);
CH451_WriteCommand(CH451_DIG2|b1[2]);
CH451_WriteCommand(CH451_DIG3|0x12);
}
