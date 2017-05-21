//IR.h
int16 irq_flag=0,irq_count=0;
int16 timer=0,startflag=0;
int16 bitnum=0,bitdata[33],receive_flag;
int16 data[4],deal_flag=0,irkey;;
int16 bianma[]={22,12,24,94,8,28,90,66,82,74}; 


void ir_deal(void);
void get_irkey(void);
void data_deal(void);   //��������


void get_irkey()
 {      uint8 x;
        if(receive_flag)
             {
               receive_flag=0;
               data_deal();
             }
             if(deal_flag)
             {
                for(x=0;x<=9;x++) 
                {
                  if(data[2]==bianma[x])
                     irkey=x;
                }
             }
}
void data_deal(void)
{   int16 i=0,j=0,k;
    uint8 value=0;
    k=1;
  for(j=0;j<4;j++)
  {
      
     for(i=0;i<8;i++)
     {
      
       value=value>>1;
       if(bitdata[k]>5)
       value=value|0x80;      //0�������������1.125ms   1�������������2.25ms
       k++;
     }                    //һλ������+������λ�û��𣨹̶���+һ����λ������+һ����λ��������
     data[j]=value; 
  }
  deal_flag=1;
}

void ir_deal(void) {
    if(startflag)
    {
       if(timer>25)
       {
         bitnum=0;
       }
       bitdata[bitnum]=timer;
       timer=0;
       bitnum++;
       if(bitnum==33)
       {
         bitnum=0;
         receive_flag=1;
       }
 
    }
   else
     {
      startflag=1;
      timer=0;
     }
}