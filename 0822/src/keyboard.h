//keyboard.h

#define CH451_RESET     0x0201             //��λ
#define CH451_LEFTMOV   0x0300             //�����ƶ���ʽ-����
#define CH451_LEFTCYC   0x0301             //�����ƶ���ʽ-��ѭ
#define CH451_RIGHTMOV  0x0302             //�����ƶ���ʽ-����
#define CH451_RIGHTCYC  0x0303             //�����ƶ���ʽ-��ѭ 
#define CH451_SYSOFF    0x0400             //����ʾ�����̡����Ź�
#define CH451_SYSON1    0x0401             //����ʾ
#define CH451_KEYEN     0x0402             //������
#define CH451_SYSON2    0x0403             //����ʾ������
#define CH451_SYSON3    0x0407             //����ʾ�����̡����Ź�����
#define CH451_DSP       0x0500             //����Ĭ����ʾ��ʽ
#define CH451_BCD       0x0580             //����BCD���뷽ʽ
#define CH451_TWINKLE   0x0600             //������˸����
#define CH451_KEY       0x0700             //�����̴���
#define CH451_DIG0      0x0800             //�����λ0��ʾ
#define CH451_DIG1      0x0900             //�����λ1��ʾ 
#define CH451_DIG2      0x0A00             //�����λ2��ʾ
#define CH451_DIG3      0x0B00             //�����λ3��ʾ
#define CH451_DIG4      0x0C00             //�����λ4��ʾ
#define CH451_DIG5      0x0D00             //�����λ5��ʾ 
#define CH451_DIG6      0x0E00             //�����λ6��ʾ 
#define CH451_DIG7      0x0F00             //�����λ7��ʾ

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

void CH451_WriteCommand(uint32 Command){        //��CH451д����
 uint8 k=12;
 //CH451_Load=0;
gpio_ctrl(PORTA,2,0); 
 for(k=0;k<12;k++){
  //CH451_IN=Command&1;
   gpio_ctrl(PORTA,1,Command&1);
 // CH451_CLK=0;
   gpio_ctrl(PORTA,0,0);
  Command>>=1;                   //��������һλ
  //CH451_CLK=1;//��������Ч
  gpio_ctrl(PORTA,0,1);
 }
 //CH451_Load=1;
 gpio_ctrl(PORTA,2,1);
// Delay(2);
// CH451_Load=1;
}

void CH451_GetKeyValue(void){
    uint8 k;
    uint8 Command=0x07;               //��ȡ��ֵ����
    GetKeyValue=0x00;           //���̴���
    //CH451_Load=0;
    gpio_ctrl(PORTA,2,0);
                        //���ʼ       
    for(k=0;k<4;k++){                 //����4λ���ݣ���λ��ǰ
      //CH451_IN=Command&0x0001; 
       gpio_ctrl(PORTA,1,Command&0X0001);
      //CH451_CLK=0;
        gpio_ctrl(PORTA,0,0);
      Command>>=1;
      //CH451_CLK=1;                   //��������Ч
       gpio_ctrl(PORTA,0,1);
    }
   // CH451_Load=1;                     //�����ؼ�������
    gpio_ctrl(PORTA,2,1);    
    for(k=0;k<7;k++){
        GetKeyValue<<=1;
        GetKeyValue|=gpio_Get(PORTA,3);//CH451_Out;
        //CH451_CLK=1;               //�½�����Ч
        gpio_ctrl(PORTA,0,1);
        //CH451_CLK=0;
        gpio_ctrl(PORTA,0,0);
    }
    
    for(k=0;k<16;k++){                //���Ҽ��̴����Ӧ�ļ�ֵ
     if(GetKeyValue==KeyCode[k]){
      KeyValue=k;
      break;
      }
    }
}

void CH451_ini(void){
   // CH451_IN=1;
     gpio_ctrl(PORTA,1,1);
   // CH451_IN=0;                        //�͵�ƽʹ�ܴ��нӿ�
     gpio_ctrl(PORTA,1,0);
    //CH451_IN=1;
     gpio_ctrl(PORTA,1,1);
    
    CH451_WriteCommand(CH451_RESET);    //��λCH451
    CH451_WriteCommand(CH451_SYSON2); 
}
//���̿��ƶ˿ڳ�ʼ��
void JIANPAN_ini(void){
        PORTA_PCR1=PORT_PCR_MUX(1);
        PORTA_PCR2=PORT_PCR_MUX(1);
        PORTA_PCR3=PORT_PCR_MUX(1);
        PORTA_PCR4=PORT_PCR_MUX(1);//A1,A2,A3,A0����ΪGPIOģʽ
        gpio_init(PORTA,0,1,1);
        gpio_init(PORTA,1,1,1);
        gpio_init(PORTA,2,1,1);
        gpio_init(PORTA,3,0,1);
        //GPIOA_PDDR&=0XFFFFFFF7;   //A3����
        //GPIOA_PDDR|=0X00000007;  //A0��A1��A2���
        //GPIOA_PCOR|=0X0000000F;   //A1,A2,A3,A0����Ϊ�ߵ�ƽ
        //GPIOA_PDIR|=0X00000008;

  //DDRB=0xf7; 
  //PORTB=0xff;
}

void print_d(int num){  //����ʾ
int b1[4]={0};
b1[0]=num%10;
b1[1]=num%100/10;
//b1[2]=num/100%10;
//b1[3]=num/1000;
CH451_WriteCommand(CH451_DIG0|b1[0]);    //0λ��ʾ1
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
//CH451_WriteCommand(CH451_DIG0|b1[0]);    //0λ��ʾ1
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
CH451_WriteCommand(CH451_DIG0|b1[0]);    //0λ��ʾ1
CH451_WriteCommand(CH451_DIG1|b1[1]);
CH451_WriteCommand(CH451_DIG2|b1[2]);
CH451_WriteCommand(CH451_DIG3|b1[3]);
}


void print_t(int num){  //����ʾ
int b1[4]={0};
b1[0]=num%10;
b1[1]=num%100/10;
//b1[2]=num/100%10;
//b1[3]=num/1000;
CH451_WriteCommand(CH451_DIG4|b1[0]);    //0λ��ʾ1
CH451_WriteCommand(CH451_DIG5|b1[1]);
//CH451_WriteCommand(CH451_DIG6|b1[2]);
//CH451_WriteCommand(CH451_DIG7|b1[3]);
}

void print_tall(int num){  //����ʾ
int b1[4]={0};
b1[0]=num%10;
b1[1]=num%100/10;
b1[2]=num/100%10;
b1[3]=num/1000;
CH451_WriteCommand(CH451_DIG4|b1[0]);    //0λ��ʾ1
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
CH451_WriteCommand(CH451_DIG0|b1[0]);    //0λ��ʾ1
CH451_WriteCommand(CH451_DIG1|b1[1]);
CH451_WriteCommand(CH451_DIG2|b1[2]);
CH451_WriteCommand(CH451_DIG3|0x12);
}
