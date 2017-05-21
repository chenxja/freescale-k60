//-------------------------------------------------------------------------*
// 文件名: light.h                                                         *
// 说  明: 指示灯驱动程序头文件                                            *
//-------------------------------------------------------------------------*

#ifndef LIGHT_H_
#define LIGHT_H_

    //1 头文件
    #include "common.h"
    #include "gpio.h"
    
    //2 宏定义
    //2.1 灯控制引脚定义
    #define Light_Run_PORT PORTA     //运行指示灯使用的端口
    #define Light_Run1      17       //运行指示灯使用的引脚  
    #define Light_Run2      14       //运行指示灯使用的引脚  
    #define Light_Run3      15       //运行指示灯使用的引脚 
    //2.2 灯状态宏定义
    #define Light_ON       0         //灯亮(对应低电平)
    #define Light_OFF      1         //灯暗(对应高电平)
    
    
    //3 函数声明
    //-----------------------------------------------------------------------*
    //函数名: light_init                                                     *
    //功  能: 初始化指示灯状态                                               *
    //参  数: port:端口名                                                    *
    //        name:指定端口引脚号                                            *
    //        state:初始状态,1=高电平,0=低电平                               *
    //返  回: 无                                                             *
    //说  明: 调用GPIO_Init函数                                              * 
    //-----------------------------------------------------------------------*
    void light_init(GPIO_MemMapPtr port,int name,int state);   
    
    //-----------------------------------------------------------------------* 
    //函数名: Light_control                                                  *
    //功  能: 控制灯的亮和暗                                                 *
    //参  数: port:端口名                                                    *
    //        name:指定端口引脚号                                            *
    //        state:状态,1=高电平,0=低电平                                   *
    //返  回: 无                                                             *
    //说  明: 调用GPIO_Set函数                                               * 
    //-----------------------------------------------------------------------* 	
    void light_control(GPIO_MemMapPtr port,int name,int state);
    
    //-----------------------------------------------------------------------* 
    //函数名: Light_change                                                   *
    //功  能: 状态切换:原来"暗",则变"亮";原来"亮",则变"暗"                   *
    //参  数: port:端口名                                                    *
    //        name:指定端口引脚号                                            *
    //返  回: 无                                                             *
    //说  明: 调用GPIO_Get、GPIO_Set函数                                     *
    //-----------------------------------------------------------------------*   
    void light_change(GPIO_MemMapPtr port,int name);
    
#endif 
