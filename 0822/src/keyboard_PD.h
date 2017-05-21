//keyboard_PD.h


//修改！！！只有1，2，5，6，9，10可用



/**********************键盘改舵机PD参数**********************/


/*
void sai_dao_yu_zhi(void)
{
  uint8 yu_zhi=0;
  while(!yu_zhi)
  {
    CH451_GetKeyValue();
    switch(KeyValue)
    {
    case 1:
      {
        while(KeyValue==1)
       {
        KeyValue=50;
        CH451_GetKeyValue();
       }
       black_max_a++;
       black_max_b++;
       black_max_c++;
       black_max_d++;
       black_max_e++;
       //print_tall(black_max_a);
       LCD_P6x8Str_3(72,3,black_max_a)  ; 
      }break;
    case 5:
      {
        while(KeyValue==5)
       {
        KeyValue=50;
        CH451_GetKeyValue();
       }
       black_max_a--;
       black_max_b--;
       black_max_c--;
       black_max_d--;
       black_max_e--;
       //print_tall(black_max_a);
       LCD_P6x8Str_3(72,3,black_max_a)  ; 
      }break;
    case 6:
      {
        while(KeyValue==6)
       {
        KeyValue=50;
        CH451_GetKeyValue();
       }
       yu_zhi=1;
       //print_tall(black_max_a);
      }
      
    }
  }
}
*/


void High_You(void)
{
  uint8 High_You_END=0;
  while(!High_You_END)
        {
          
          CH451_GetKeyValue();
          switch(KeyValue)
          {
          case 1:
            {
              while(KeyValue==1)
              {
                KeyValue=50;
                CH451_GetKeyValue();

              }
              P_High+=1;
              
              LCD_P6x8Str_3(72,3,P_High); 
            } break;    
          case 9:
            {
              while(KeyValue==9)
              {
                KeyValue=50;
                CH451_GetKeyValue();

              }
              D_High+=1;
              
              LCD_P6x8Str_3(92,3,D_High)  ; 
              break; 
            }
          case 5:
            {
              while(KeyValue==5)
              {
                KeyValue=50;
                CH451_GetKeyValue();

              }
              P_High-=1;
              
              LCD_P6x8Str_3(72,3,P_High)  ; 
            } break;   
          case 10:
            {
              while(KeyValue==10)
              {
                KeyValue=50;
                CH451_GetKeyValue();

              }
              D_High-=1;
              
              LCD_P6x8Str_3(92,3,D_High)  ; 
              break; 
            }   
          case 2:
            while(KeyValue==2)
              {
                KeyValue=50;
                CH451_GetKeyValue();

              }
               
          
            break;
          case 6:
            while(KeyValue==6)
            {
              KeyValue=50;
              CH451_GetKeyValue();

            }
            High_You_END=1;
           
            break;
          }
          
        }
}

void High_Wu(void)
{
  uint8 High_Wu_END=0;
  while(!High_Wu_END)
        {
          
          CH451_GetKeyValue();
          switch(KeyValue)
          {
          case 1:
            {
              while(KeyValue==1)
              {
                KeyValue=50;
                CH451_GetKeyValue();

              }
              P_Mid+=1;
              
              LCD_P6x8Str_3(72,3,P_Mid); 
            } break;    
          case 9:
            {
              while(KeyValue==9)
              {
                KeyValue=50;
                CH451_GetKeyValue();

              }
              D_Mid+=1;
              
              LCD_P6x8Str_3(92,3,D_Mid)  ; 
              break; 
            }
          case 5:
            {
              while(KeyValue==5)
              {
                KeyValue=50;
                CH451_GetKeyValue();

              }
              P_Mid-=1;
              
              LCD_P6x8Str_3(72,3,P_Mid)  ; 
            } break;   
          case 10:
            {
              while(KeyValue==10)
              {
                KeyValue=50;
                CH451_GetKeyValue();

              }
              D_Mid-=1;
              
              LCD_P6x8Str_3(92,3,D_Mid)  ; 
              break; 
            }   
          case 2:
            while(KeyValue==2)
              {
                KeyValue=50;
                CH451_GetKeyValue();

              }
               
            break;
          case 6:
            while(KeyValue==6)
            {
              KeyValue=50;
              CH451_GetKeyValue();

            }
            High_Wu_END=1;
           
            break;
          }
          
        }
}
void High_Mid(void)
{
  uint8 High_Mid_END=0;
  while(!High_Mid_END)
        {
          
          CH451_GetKeyValue();
          switch(KeyValue)
          {
          case 1:
            {
              while(KeyValue==1)
              {
                KeyValue=50;
                CH451_GetKeyValue();

              }
              P_Mid1+=1;
              
              LCD_P6x8Str_3(72,3,P_Mid1); 
            } break;    
          case 9:
            {
              while(KeyValue==9)
              {
                KeyValue=50;
                CH451_GetKeyValue();

              }
              D_Mid1+=1;
              
              LCD_P6x8Str_3(92,3,D_Mid1)  ; 
              break; 
            }
          case 5:
            {
              while(KeyValue==5)
              {
                KeyValue=50;
                CH451_GetKeyValue();

              }
              P_Mid1-=1;
              
              LCD_P6x8Str_3(72,3,P_Mid1)  ; 
            } break;   
          case 10:
            {
              while(KeyValue==10)
              {
                KeyValue=50;
                CH451_GetKeyValue();

              }
              D_Mid1-=1;
              
              LCD_P6x8Str_3(92,3,D_Mid1)  ; 
              break; 
            }   
          case 2:
            while(KeyValue==2)
              {
                KeyValue=50;
                CH451_GetKeyValue();

              }               
            break;
          case 6:
            while(KeyValue==6)
            {
              KeyValue=50;
              CH451_GetKeyValue();

            }
            High_Mid_END=1;
           
            break;
          }
          
        }
}


void Mid_Wu(void)
{
  uint8 Mid_Wu_END=0;
  while(!Mid_Wu_END)
        {
          
          CH451_GetKeyValue();
          switch(KeyValue)
          {
          case 1:
            {
              while(KeyValue==1)
              {
                KeyValue=50;
                CH451_GetKeyValue();

              }
              P_Low+=1;
              LCD_P6x8Str_3(72,3,P_Low); 
            } break;    
          case 9:
            {
              while(KeyValue==9)
              {
                KeyValue=50;
                CH451_GetKeyValue();

              }
              D_Low+=1;
              LCD_P6x8Str_3(92,3,D_Low)  ; 
              break; 
            }
          case 5:
            {
              while(KeyValue==5)
              {
                KeyValue=50;
                CH451_GetKeyValue();

              }
              P_Low-=1;
              LCD_P6x8Str_3(72,3,P_Low)  ; 
            } break;   
          case 10:
            {
              while(KeyValue==10)
              {
                KeyValue=50;
                CH451_GetKeyValue();

              }
              D_Low-=1;
              LCD_P6x8Str_3(92,3,D_Low)  ; 
              break; 
            }   
          case 2:
            while(KeyValue==2)
              {
                KeyValue=50;
                CH451_GetKeyValue();

              }
               
            //print_t1(P_Line_9);print_t(D_Line_9);
            break;
          case 6:
            while(KeyValue==6)
            {
              KeyValue=50;
              CH451_GetKeyValue();

            }
            Mid_Wu_END=1;
            //print_t1(P_Line_9);print_t(D_Line_9);Line_PD_9_END=1;
            break;
          }
          
        }
}

void Duo_Ji_PD(void)
{
  uint8 Duo_Ji_PD_END=0;
  uint8 sai_dao_number=0;
  LCD_P6x8Str_3(90,2,sai_dao_number);//显示赛道类型
  while(!Duo_Ji_PD_END)
  {
    CH451_GetKeyValue();
    switch(KeyValue)
    {
    case 1:
      {
        while(KeyValue==1)
        {
          KeyValue=50;
          CH451_GetKeyValue();

        }
        sai_dao_number++;
        LCD_P6x8Str_3(90,2,sai_dao_number);//显示赛道类型
        //print_d1(sai_dao_number);
      }break;
    case 9:
      {
        while(KeyValue==9)
        {
          KeyValue=50;
          CH451_GetKeyValue();

        }
        sai_dao_number+=2;
        LCD_P6x8Str_3(90,2,sai_dao_number);//显示赛道类型
        //print_d1(sai_dao_number);
      }break;
      
    case 5:
      {
        while(KeyValue==5)
        {
          KeyValue=50;
          CH451_GetKeyValue();

        }
        sai_dao_number--;
        LCD_P6x8Str_3(90,2,sai_dao_number);//显示赛道类型
        //print_d1(sai_dao_number);
      }break;
    case 10:
      {
        while(KeyValue==10)
        {
          KeyValue=50;
          CH451_GetKeyValue();

        }
        sai_dao_number-=2;
        LCD_P6x8Str_3(90,2,sai_dao_number);//显示赛道类型
        //print_d1(sai_dao_number);
      }break;
    case 2:
      {
        
        while(KeyValue==2)
        {
          KeyValue=50;
          CH451_GetKeyValue();

        }
        if(sai_dao_number==4){LCD_CLS2(3);LCD_P6x8Str(0,3,"Mid_Wu");LCD_P6x8Str_3(72,3,P_Low); LCD_P6x8Str_3(92,3,D_Low);Mid_Wu();}        //14
        if(sai_dao_number==2){LCD_CLS2(3);LCD_P6x8Str(0,3,"High_Wu");LCD_P6x8Str_3(72,3,P_Mid); LCD_P6x8Str_3(92,3,D_Mid);High_Wu();} 
        if(sai_dao_number==3){LCD_CLS2(3);LCD_P6x8Str(0,3,"High_Mid");LCD_P6x8Str_3(72,3,P_Mid1); LCD_P6x8Str_3(92,3,D_Mid1);High_Mid();} 
        if(sai_dao_number==1){LCD_CLS2(3);LCD_P6x8Str(0,3,"High_You");LCD_P6x8Str_3(72,3,P_High); LCD_P6x8Str_3(92,3,D_High);High_You();} 
        //if(sai_dao_number==15){LCD_CLS2(3);LCD_P6x8Str(0,3,"yu_zhi");LCD_P6x8Str_3(72,3,black_max_a);sai_dao_yu_zhi();}     //赛道阈值的设置调整
      }break;
    case 6:
      {
        LCD_P6x8Str(110,2,"OK");     //英文字符串显示 
        //print_d(KeyValue);
        while(KeyValue==6)
        {
          KeyValue=50;
          CH451_GetKeyValue();

        }
        Duo_Ji_PD_END=1;
        
      }break;
    }
    
  }
}


void Car_Speed(void)
{
  uint8 car_speed_flag=0;  
  while(!car_speed_flag)
  {
    CH451_GetKeyValue();
    switch(KeyValue)
    {
    case 1:
      {
        while(KeyValue==1)
        {
          KeyValue=50;
          CH451_GetKeyValue();

        }
        car_speed++;
        LCD_P6x8Str_3(36,0,car_speed);
        //print_d1(sai_dao_number);
      }break;
    
      
    case 5:
      {
        while(KeyValue==5)
        {
          KeyValue=50;
          CH451_GetKeyValue();

        }
        car_speed--;
        LCD_P6x8Str_3(36,0,car_speed);
        //print_d1(sai_dao_number);
      }break;
      
      case 9:
      {
        while(KeyValue==9)
        {
          KeyValue=50;
          CH451_GetKeyValue();

        }
        Judge_startline+=10;
        LCD_P6x8Str_3(50,7,Judge_startline);
        
        //print_d1(sai_dao_number);
      }break;
    case 10:
      {
        while(KeyValue==10)
        {
          KeyValue=50;
          CH451_GetKeyValue();

        }
        Judge_startline-=10;
        LCD_P6x8Str_3(50,7,Judge_startline);
        //print_d1(sai_dao_number);
      }break;
    case 2:
      {
        while(KeyValue==2)
        {
          KeyValue=50;
          CH451_GetKeyValue();

        }
        
      }
      LCD_P6x8Str(60,0,"OK");     //英文字符串显示
      car_speed_flag=1;
      break;
    
    }
    
  }
}



