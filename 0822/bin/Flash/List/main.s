///////////////////////////////////////////////////////////////////////////////
//                                                                            /
// IAR ANSI C/C++ Compiler V6.30.1.53127/W32 for ARM    22/Aug/2013  15:19:17 /
// Copyright 1999-2011 IAR Systems AB.                                        /
//                                                                            /
//    Cpu mode     =  thumb                                                   /
//    Endian       =  little                                                  /
//    Source file  =  C:\Users\Administrator\Desktop\哈尔滨比赛程序\0822\src\ /
//                    main.c                                                  /
//    Command line =  C:\Users\Administrator\Desktop\哈尔滨比赛程序\0822\src\ /
//                    main.c -D IAR -D TWR_K60N512 -lCN                       /
//                    C:\Users\Administrator\Desktop\哈尔滨比赛程序\0822\bin\ /
//                    Flash\List\ -lB C:\Users\Administrator\Desktop\哈尔滨比 /
//                    赛程序\0822\bin\Flash\List\ -o                          /
//                    C:\Users\Administrator\Desktop\哈尔滨比赛程序\0822\bin\ /
//                    Flash\Obj\ --no_cse --no_unroll --no_inline             /
//                    --no_code_motion --no_tbaa --no_clustering              /
//                    --no_scheduling --debug --endian=little                 /
//                    --cpu=Cortex-M4 -e --fpu=None --dlib_config             /
//                    E:\学习程序\arm\INC\c\DLib_Config_Normal.h -I           /
//                    C:\Users\Administrator\Desktop\哈尔滨比赛程序\0822\src\ /
//                    Sources\H\ -I C:\Users\Administrator\Desktop\哈尔滨比赛 /
//                    程序\0822\src\Sources\H\Component_H\ -I                 /
//                    C:\Users\Administrator\Desktop\哈尔滨比赛程序\0822\src\ /
//                    Sources\H\Frame_H\ -Ol --use_c++_inline                 /
//    List file    =  C:\Users\Administrator\Desktop\哈尔滨比赛程序\0822\bin\ /
//                    Flash\List\main.s                                       /
//                                                                            /
//                                                                            /
///////////////////////////////////////////////////////////////////////////////

        NAME main

        #define SHT_PROGBITS 0x1

        EXTERN LCD_CLS
        EXTERN LCD_CLS2
        EXTERN LCD_Init
        EXTERN LCD_P6x8Str
        EXTERN LCD_P6x8Str_1
        EXTERN LCD_P6x8Str_3
        EXTERN LCD_P6x8Str_5
        EXTERN LCD_P6x8Str_FuHao_3
        EXTERN LCD_PutPixel
        EXTERN __aeabi_memclr4
        EXTERN disable_irq
        EXTERN enable_irq
        EXTERN gpio_Get
        EXTERN gpio_ctrl
        EXTERN gpio_init
        EXTERN uart_send1

        PUBLIC CH451_GetKeyValue
        PUBLIC CH451_WriteCommand
        PUBLIC CH451_ini
        PUBLIC Car_Speed
        PUBLIC DMA0_Init
        PUBLIC DMA_CHO_ISR
        PUBLIC D_High
        PUBLIC D_Low
        PUBLIC D_Mid
        PUBLIC D_Mid1
        PUBLIC Delay
        PUBLIC Duo_Ji_PD
        PUBLIC END_0
        PUBLIC FTM2_QUAD_Init
        PUBLIC GetKeyValue
        PUBLIC H_High
        PUBLIC H_Low
        PUBLIC H_Mid
        PUBLIC H_Mid1
        PUBLIC High_Mid
        PUBLIC High_Wu
        PUBLIC High_You
        PUBLIC IO_Init
        PUBLIC ImageData
        PUBLIC ImageData2
        PUBLIC JIANPAN_ini
        PUBLIC Judge_end
        PUBLIC Judge_startline
        PUBLIC Judge_startline2
        PUBLIC KeyCode
        PUBLIC KeyValue
        PUBLIC LCD_P1
        PUBLIC LCD_P2
        PUBLIC LCD_P3
        PUBLIC LPTMR_INTERRUPT
        PUBLIC Mid_Wu
        PUBLIC Motor
        PUBLIC Motor_control
        PUBLIC P_High
        PUBLIC P_Low
        PUBLIC P_Mid
        PUBLIC P_Mid1
        PUBLIC P_timer
        PUBLIC SCI
        PUBLIC Servo
        PUBLIC Servo_Left
        PUBLIC Servo_Middle
        PUBLIC Servo_Right
        PUBLIC Servo_value
        PUBLIC Servo_value_old
        PUBLIC T_High
        PUBLIC T_Low
        PUBLIC T_Mid
        PUBLIC T_Mid1
        PUBLIC UART0_Init
        PUBLIC Very_straight
        PUBLIC Wspeed_diff
        PUBLIC abs
        PUBLIC backup
        PUBLIC bianma
        PUBLIC binaryzation
        PUBLIC binaryzation2
        PUBLIC bitdata
        PUBLIC bitnum
        PUBLIC black2_a1
        PUBLIC black2_a2
        PUBLIC black2_a3
        PUBLIC black2_b1
        PUBLIC black2_b2
        PUBLIC black2_b3
        PUBLIC black2_c1
        PUBLIC black2_c2
        PUBLIC black2_c3
        PUBLIC black2_d1
        PUBLIC black2_d2
        PUBLIC black2_d3
        PUBLIC black_a1
        PUBLIC black_a2
        PUBLIC black_a3
        PUBLIC black_b1
        PUBLIC black_b2
        PUBLIC black_b3
        PUBLIC black_c1
        PUBLIC black_c2
        PUBLIC black_c3
        PUBLIC black_d1
        PUBLIC black_d2
        PUBLIC black_d3
        PUBLIC car_speed
        PUBLIC center_filter
        PUBLIC choose_PD
        PUBLIC `data`
        PUBLIC data_deal
        PUBLIC data_table
        PUBLIC deal_flag
        PUBLIC delayms
        PUBLIC delays
        PUBLIC dijihang
        PUBLIC disable_pit_interrupt
        PUBLIC enable_pit_interrupt
        PUBLIC endline
        PUBLIC endline_F
        PUBLIC find_position
        PUBLIC finish
        PUBLIC get_counter_value
        PUBLIC get_irkey
        PUBLIC get_root
        PUBLIC get_speed
        PUBLIC `high`
        PUBLIC high_0
        PUBLIC high_now
        PUBLIC high_now_0
        PUBLIC high_old
        PUBLIC hw_FTM0_init
        PUBLIC hw_FTM1_init
        PUBLIC hw_pit_init
        PUBLIC imagerow
        PUBLIC ir_deal
        PUBLIC irkey
        PUBLIC irq_count
        PUBLIC irq_flag
        PUBLIC judge_locus_0
        PUBLIC limit
        PUBLIC line
        PUBLIC line_max
        PUBLIC line_min
        PUBLIC `low`
        PUBLIC low_0
        PUBLIC low_now
        PUBLIC low_now_0
        PUBLIC low_old
        PUBLIC lptmr_32khz_input
        PUBLIC lptmr_clear_registers
        PUBLIC lptmr_external_clk_input
        PUBLIC lptmr_internal_ref_input
        PUBLIC lptmr_interrupt
        PUBLIC lptmr_isr_example
        PUBLIC lptmr_lpo_input
        PUBLIC lptmr_prescale
        PUBLIC lptmr_pulse_counter
        PUBLIC lptmr_time_counter
        PUBLIC main
        PUBLIC max
        PUBLIC mid
        PUBLIC mid_0
        PUBLIC mid_now
        PUBLIC mid_now_0
        PUBLIC mid_old
        PUBLIC min
        PUBLIC motor_fuzzy
        PUBLIC pit0_isr
        PUBLIC pit1_isr
        PUBLIC pit2_isr
        PUBLIC pllinit180M
        PUBLIC point
        PUBLIC porta_isr
        PUBLIC portb_isr
        PUBLIC portc_isr
        PUBLIC position
        PUBLIC position_diff
        PUBLIC position_now
        PUBLIC position_now_low
        PUBLIC position_youhua
        PUBLIC print_d
        PUBLIC print_d1
        PUBLIC print_dall
        PUBLIC print_fall
        PUBLIC print_hell
        PUBLIC print_t
        PUBLIC print_t1
        PUBLIC print_tall
        PUBLIC ption
        PUBLIC pulse
        PUBLIC pulse2
        PUBLIC ramp_flag
        PUBLIC receive_flag
        PUBLIC row
        PUBLIC row_F
        PUBLIC s_old
        PUBLIC sai_dao_lei_xing
        PUBLIC servo
        PUBLIC set_speed
        PUBLIC speed_Very_straight
        PUBLIC speed_b
        PUBLIC speed_control
        PUBLIC speed_fuzzy
        PUBLIC speed_m
        PUBLIC speed_ms
        PUBLIC speed_now
        PUBLIC speed_s
        PUBLIC speed_top
        PUBLIC stages3
        PUBLIC stages_D
        PUBLIC stages_P
        PUBLIC stages_low_P
        PUBLIC start_position
        PUBLIC startflag
        PUBLIC startline
        PUBLIC startline_F
        PUBLIC `sub`
        PUBLIC sub_e
        PUBLIC subjection_k
        PUBLIC time_m
        PUBLIC timer
        PUBLIC topline
        PUBLIC tu_flag

// C:\Users\Administrator\Desktop\哈尔滨比赛程序\0822\src\main.c
//    1 //使用B0~B7采集数字摄像头OV7620 8位灰度输入
//    2 //A24引脚设置为GPIO模式，下降沿中断,场中断
//    3 //B10引脚设置为GPIO模式，上升沿中断,行中断
//    4 //B11引脚设置为GPIO模式，像素同步脉冲pclk，4分频后，上升沿触发DMA请求
//    5 //共采集H行，每行V个点，dma传送到video数组
//    6 //本代码仅供个人学习使用，请勿用于其他用途
//    7 //舵机5V电压250Hz    电机10KHz  3.3V   摄像头5V供电 
//    8 
//    9 /********************************************************/
//   10         //给某个引脚设置高低电平的方法
//   11         //PORTD_PCR10=PORT_PCR_MUX(1);                 选择GPIO模式
//   12         //GPIOD_PDDR=GPIO_PDDR_PDD(GPIO_PIN(10));      选择输出模式
//   13         //GPIOD_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(10));  高电平
//   14         //GPIOA_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(14));  低电平
//   15         /********************************************************/
//   16 #include <stdio.h>
//   17 #include "includes.h"
//   18 #include "MK60N512VMD100.h"
//   19 #include "uart.h"
//   20 #include "keyboard.h"

        SECTION `.data`:DATA:REORDER:NOROOT(2)
// __absolute uint8 KeyCode[16]
KeyCode:
        DATA
        DC8 64, 65, 66, 67, 72, 73, 74, 75, 80, 81, 82, 83, 88, 89, 90, 91

        SECTION `.data`:DATA:REORDER:NOROOT(0)
// __absolute uint8 KeyValue
KeyValue:
        DATA
        DC8 50

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
// __absolute uint8 GetKeyValue
GetKeyValue:
        DS8 1

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// __interwork __softfp void Delay(unsigned int)
Delay:
        MOVS     R1,#+0
        B.N      ??Delay_0
??Delay_1:
        ADDS     R1,R1,#+1
??Delay_0:
        CMP      R1,R0, LSL #+1
        BCC.N    ??Delay_1
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// __interwork __softfp void CH451_WriteCommand(uint32)
CH451_WriteCommand:
        PUSH     {R3-R5,LR}
        MOVS     R4,R0
        MOVS     R5,#+12
        MOVS     R2,#+0
        MOVS     R1,#+2
        LDR.W    R0,??DataTable21  ;; 0x400ff000
        BL       gpio_ctrl
        MOVS     R0,#+0
        MOVS     R5,R0
        B.N      ??CH451_WriteCommand_0
??CH451_WriteCommand_1:
        ANDS     R2,R4,#0x1
        MOVS     R1,#+1
        LDR.W    R0,??DataTable21  ;; 0x400ff000
        BL       gpio_ctrl
        MOVS     R2,#+0
        MOVS     R1,#+0
        LDR.W    R0,??DataTable21  ;; 0x400ff000
        BL       gpio_ctrl
        LSRS     R4,R4,#+1
        MOVS     R2,#+1
        MOVS     R1,#+0
        LDR.W    R0,??DataTable21  ;; 0x400ff000
        BL       gpio_ctrl
        ADDS     R5,R5,#+1
??CH451_WriteCommand_0:
        UXTB     R5,R5            ;; ZeroExt  R5,R5,#+24,#+24
        CMP      R5,#+12
        BCC.N    ??CH451_WriteCommand_1
        MOVS     R2,#+1
        MOVS     R1,#+2
        LDR.W    R0,??DataTable21  ;; 0x400ff000
        BL       gpio_ctrl
        POP      {R0,R4,R5,PC}    ;; return

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// __interwork __softfp void CH451_GetKeyValue(void)
CH451_GetKeyValue:
        PUSH     {R3-R5,LR}
        MOVS     R5,#+7
        LDR.W    R0,??DataTable21_1
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
        MOVS     R2,#+0
        MOVS     R1,#+2
        LDR.W    R0,??DataTable21  ;; 0x400ff000
        BL       gpio_ctrl
        MOVS     R4,#+0
        B.N      ??CH451_GetKeyValue_0
??CH451_GetKeyValue_1:
        UXTB     R5,R5            ;; ZeroExt  R5,R5,#+24,#+24
        ANDS     R2,R5,#0x1
        MOVS     R1,#+1
        LDR.W    R0,??DataTable21  ;; 0x400ff000
        BL       gpio_ctrl
        MOVS     R2,#+0
        MOVS     R1,#+0
        LDR.W    R0,??DataTable21  ;; 0x400ff000
        BL       gpio_ctrl
        UXTB     R5,R5            ;; ZeroExt  R5,R5,#+24,#+24
        LSRS     R5,R5,#+1
        MOVS     R2,#+1
        MOVS     R1,#+0
        LDR.W    R0,??DataTable21  ;; 0x400ff000
        BL       gpio_ctrl
        ADDS     R4,R4,#+1
??CH451_GetKeyValue_0:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+4
        BCC.N    ??CH451_GetKeyValue_1
        MOVS     R2,#+1
        MOVS     R1,#+2
        LDR.W    R0,??DataTable21  ;; 0x400ff000
        BL       gpio_ctrl
        MOVS     R4,#+0
        B.N      ??CH451_GetKeyValue_2
??CH451_GetKeyValue_3:
        LDR.W    R0,??DataTable21_1
        LDRB     R0,[R0, #+0]
        LSLS     R0,R0,#+1
        LDR.W    R1,??DataTable21_1
        STRB     R0,[R1, #+0]
        MOVS     R1,#+3
        LDR.W    R0,??DataTable21  ;; 0x400ff000
        BL       gpio_Get
        LDR.W    R1,??DataTable21_1
        LDRB     R1,[R1, #+0]
        ORRS     R0,R0,R1
        LDR.W    R1,??DataTable21_1
        STRB     R0,[R1, #+0]
        MOVS     R2,#+1
        MOVS     R1,#+0
        LDR.W    R0,??DataTable21  ;; 0x400ff000
        BL       gpio_ctrl
        MOVS     R2,#+0
        MOVS     R1,#+0
        LDR.W    R0,??DataTable21  ;; 0x400ff000
        BL       gpio_ctrl
        ADDS     R4,R4,#+1
??CH451_GetKeyValue_2:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+7
        BCC.N    ??CH451_GetKeyValue_3
        MOVS     R4,#+0
        B.N      ??CH451_GetKeyValue_4
??CH451_GetKeyValue_5:
        ADDS     R4,R4,#+1
??CH451_GetKeyValue_4:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+16
        BCS.N    ??CH451_GetKeyValue_6
        LDR.W    R0,??DataTable21_1
        LDRB     R0,[R0, #+0]
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.W    R1,??DataTable21_2
        LDRB     R1,[R4, R1]
        CMP      R0,R1
        BNE.N    ??CH451_GetKeyValue_5
        LDR.W    R0,??DataTable21_3
        STRB     R4,[R0, #+0]
??CH451_GetKeyValue_6:
        POP      {R0,R4,R5,PC}    ;; return

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// __interwork __softfp void CH451_ini(void)
CH451_ini:
        PUSH     {R7,LR}
        MOVS     R2,#+1
        MOVS     R1,#+1
        LDR.W    R0,??DataTable21  ;; 0x400ff000
        BL       gpio_ctrl
        MOVS     R2,#+0
        MOVS     R1,#+1
        LDR.W    R0,??DataTable21  ;; 0x400ff000
        BL       gpio_ctrl
        MOVS     R2,#+1
        MOVS     R1,#+1
        LDR.W    R0,??DataTable21  ;; 0x400ff000
        BL       gpio_ctrl
        MOVW     R0,#+513
        BL       CH451_WriteCommand
        MOVW     R0,#+1027
        BL       CH451_WriteCommand
        POP      {R0,PC}          ;; return

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// __interwork __softfp void JIANPAN_ini(void)
JIANPAN_ini:
        PUSH     {R7,LR}
        LDR.W    R0,??DataTable21_4  ;; 0x40049004
        MOV      R1,#+256
        STR      R1,[R0, #+0]
        LDR.W    R0,??DataTable21_5  ;; 0x40049008
        MOV      R1,#+256
        STR      R1,[R0, #+0]
        LDR.W    R0,??DataTable21_6  ;; 0x4004900c
        MOV      R1,#+256
        STR      R1,[R0, #+0]
        LDR.W    R0,??DataTable21_7  ;; 0x40049010
        MOV      R1,#+256
        STR      R1,[R0, #+0]
        MOVS     R3,#+1
        MOVS     R2,#+1
        MOVS     R1,#+0
        LDR.W    R0,??DataTable21  ;; 0x400ff000
        BL       gpio_init
        MOVS     R3,#+1
        MOVS     R2,#+1
        MOVS     R1,#+1
        LDR.W    R0,??DataTable21  ;; 0x400ff000
        BL       gpio_init
        MOVS     R3,#+1
        MOVS     R2,#+1
        MOVS     R1,#+2
        LDR.W    R0,??DataTable21  ;; 0x400ff000
        BL       gpio_init
        MOVS     R3,#+1
        MOVS     R2,#+0
        MOVS     R1,#+3
        LDR.W    R0,??DataTable21  ;; 0x400ff000
        BL       gpio_init
        POP      {R0,PC}          ;; return

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// __interwork __softfp void print_d(int)
print_d:
        PUSH     {R0-R4,LR}
        MOVS     R4,R0
        ADD      R0,SP,#+0
        MOVS     R1,#+16
        BL       __aeabi_memclr4
        MOVS     R0,#+10
        SDIV     R1,R4,R0
        MLS      R0,R0,R1,R4
        STR      R0,[SP, #+0]
        MOVS     R0,#+100
        SDIV     R1,R4,R0
        MLS      R0,R0,R1,R4
        MOVS     R1,#+10
        SDIV     R0,R0,R1
        STR      R0,[SP, #+4]
        LDR      R0,[SP, #+0]
        ORRS     R0,R0,#0x800
        BL       CH451_WriteCommand
        LDR      R0,[SP, #+4]
        ORRS     R0,R0,#0x900
        BL       CH451_WriteCommand
        POP      {R0-R4,PC}       ;; return

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// __interwork __softfp void print_d1(int)
print_d1:
        PUSH     {R0-R4,LR}
        MOVS     R4,R0
        ADD      R0,SP,#+0
        MOVS     R1,#+16
        BL       __aeabi_memclr4
        MOVS     R0,#+10
        SDIV     R1,R4,R0
        MLS      R0,R0,R1,R4
        STR      R0,[SP, #+0]
        MOVS     R0,#+100
        SDIV     R1,R4,R0
        MLS      R0,R0,R1,R4
        MOVS     R1,#+10
        SDIV     R0,R0,R1
        STR      R0,[SP, #+4]
        LDR      R0,[SP, #+0]
        ORRS     R0,R0,#0xA00
        BL       CH451_WriteCommand
        LDR      R0,[SP, #+4]
        ORRS     R0,R0,#0xB00
        BL       CH451_WriteCommand
        POP      {R0-R4,PC}       ;; return

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// __interwork __softfp void print_dall(int)
print_dall:
        PUSH     {R0-R4,LR}
        MOVS     R4,R0
        ADD      R0,SP,#+0
        MOVS     R1,#+16
        BL       __aeabi_memclr4
        MOVS     R0,#+10
        SDIV     R1,R4,R0
        MLS      R0,R0,R1,R4
        STR      R0,[SP, #+0]
        MOVS     R0,#+100
        SDIV     R1,R4,R0
        MLS      R0,R0,R1,R4
        MOVS     R1,#+10
        SDIV     R0,R0,R1
        STR      R0,[SP, #+4]
        MOVS     R0,#+100
        SDIV     R0,R4,R0
        MOVS     R1,#+10
        SDIV     R2,R0,R1
        MLS      R0,R1,R2,R0
        STR      R0,[SP, #+8]
        MOV      R0,#+1000
        SDIV     R0,R4,R0
        STR      R0,[SP, #+12]
        LDR      R0,[SP, #+0]
        ORRS     R0,R0,#0x800
        BL       CH451_WriteCommand
        LDR      R0,[SP, #+4]
        ORRS     R0,R0,#0x900
        BL       CH451_WriteCommand
        LDR      R0,[SP, #+8]
        ORRS     R0,R0,#0xA00
        BL       CH451_WriteCommand
        LDR      R0,[SP, #+12]
        ORRS     R0,R0,#0xB00
        BL       CH451_WriteCommand
        POP      {R0-R4,PC}       ;; return

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// __interwork __softfp void print_t(int)
print_t:
        PUSH     {R0-R4,LR}
        MOVS     R4,R0
        ADD      R0,SP,#+0
        MOVS     R1,#+16
        BL       __aeabi_memclr4
        MOVS     R0,#+10
        SDIV     R1,R4,R0
        MLS      R0,R0,R1,R4
        STR      R0,[SP, #+0]
        MOVS     R0,#+100
        SDIV     R1,R4,R0
        MLS      R0,R0,R1,R4
        MOVS     R1,#+10
        SDIV     R0,R0,R1
        STR      R0,[SP, #+4]
        LDR      R0,[SP, #+0]
        ORRS     R0,R0,#0xC00
        BL       CH451_WriteCommand
        LDR      R0,[SP, #+4]
        ORRS     R0,R0,#0xD00
        BL       CH451_WriteCommand
        POP      {R0-R4,PC}       ;; return

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// __interwork __softfp void print_tall(int)
print_tall:
        PUSH     {R0-R4,LR}
        MOVS     R4,R0
        ADD      R0,SP,#+0
        MOVS     R1,#+16
        BL       __aeabi_memclr4
        MOVS     R0,#+10
        SDIV     R1,R4,R0
        MLS      R0,R0,R1,R4
        STR      R0,[SP, #+0]
        MOVS     R0,#+100
        SDIV     R1,R4,R0
        MLS      R0,R0,R1,R4
        MOVS     R1,#+10
        SDIV     R0,R0,R1
        STR      R0,[SP, #+4]
        MOVS     R0,#+100
        SDIV     R0,R4,R0
        MOVS     R1,#+10
        SDIV     R2,R0,R1
        MLS      R0,R1,R2,R0
        STR      R0,[SP, #+8]
        MOV      R0,#+1000
        SDIV     R0,R4,R0
        STR      R0,[SP, #+12]
        LDR      R0,[SP, #+0]
        ORRS     R0,R0,#0xC00
        BL       CH451_WriteCommand
        LDR      R0,[SP, #+4]
        ORRS     R0,R0,#0xD00
        BL       CH451_WriteCommand
        LDR      R0,[SP, #+8]
        ORRS     R0,R0,#0xE00
        BL       CH451_WriteCommand
        LDR      R0,[SP, #+12]
        ORRS     R0,R0,#0xF00
        BL       CH451_WriteCommand
        POP      {R0-R4,PC}       ;; return

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// __interwork __softfp void print_t1(int)
print_t1:
        PUSH     {R0-R4,LR}
        MOVS     R4,R0
        ADD      R0,SP,#+0
        MOVS     R1,#+16
        BL       __aeabi_memclr4
        MOVS     R0,#+10
        SDIV     R1,R4,R0
        MLS      R0,R0,R1,R4
        STR      R0,[SP, #+0]
        MOVS     R0,#+100
        SDIV     R1,R4,R0
        MLS      R0,R0,R1,R4
        MOVS     R1,#+10
        SDIV     R0,R0,R1
        STR      R0,[SP, #+4]
        LDR      R0,[SP, #+0]
        ORRS     R0,R0,#0xE00
        BL       CH451_WriteCommand
        LDR      R0,[SP, #+4]
        ORRS     R0,R0,#0xF00
        BL       CH451_WriteCommand
        POP      {R0-R4,PC}       ;; return

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// __interwork __softfp void print_hell()
print_hell:
        PUSH     {R7,LR}
        MOVW     R0,#+3128
        BL       CH451_WriteCommand
        MOVW     R0,#+3384
        BL       CH451_WriteCommand
        MOVW     R0,#+3598
        BL       CH451_WriteCommand
        MOVW     R0,#+3959
        BL       CH451_WriteCommand
        POP      {R0,PC}          ;; return

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// __interwork __softfp void print_fall(int)
print_fall:
        PUSH     {R0-R4,LR}
        MOVS     R4,R0
        ADD      R0,SP,#+0
        MOVS     R1,#+16
        BL       __aeabi_memclr4
        MOVS     R0,#+10
        SDIV     R1,R4,R0
        MLS      R0,R0,R1,R4
        STR      R0,[SP, #+0]
        MOVS     R0,#+100
        SDIV     R1,R4,R0
        MLS      R0,R0,R1,R4
        MOVS     R1,#+10
        SDIV     R0,R0,R1
        STR      R0,[SP, #+4]
        MOVS     R0,#+100
        SDIV     R0,R4,R0
        MOVS     R1,#+10
        SDIV     R2,R0,R1
        MLS      R0,R1,R2,R0
        STR      R0,[SP, #+8]
        LDR      R0,[SP, #+0]
        ORRS     R0,R0,#0x800
        BL       CH451_WriteCommand
        LDR      R0,[SP, #+4]
        ORRS     R0,R0,#0x900
        BL       CH451_WriteCommand
        LDR      R0,[SP, #+8]
        ORRS     R0,R0,#0xA00
        BL       CH451_WriteCommand
        MOVW     R0,#+2834
        BL       CH451_WriteCommand
        POP      {R0-R4,PC}       ;; return
//   21 #include "IR.h"

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
// __absolute int16 irq_flag
irq_flag:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
// __absolute int16 irq_count
irq_count:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
// __absolute int16 timer
timer:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
// __absolute int16 startflag
startflag:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
// __absolute int16 bitnum
bitnum:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(2)
// __absolute int16 bitdata[33]
bitdata:
        DS8 68

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
// __absolute int16 receive_flag
receive_flag:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(2)
// __absolute int16 data[4]
`data`:
        DS8 8

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
// __absolute int16 deal_flag
deal_flag:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
// __absolute int16 irkey
irkey:
        DS8 2

        SECTION `.data`:DATA:REORDER:NOROOT(2)
// __absolute int16 bianma[10]
bianma:
        DATA
        DC16 22, 12, 24, 94, 8, 28, 90, 66, 82, 74

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// __interwork __softfp void get_irkey()
get_irkey:
        PUSH     {R7,LR}
        LDR.W    R0,??DataTable23_1
        LDRSH    R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.N    ??get_irkey_0
        LDR.W    R0,??DataTable23_1
        MOVS     R1,#+0
        STRH     R1,[R0, #+0]
        BL       data_deal
??get_irkey_0:
        LDR.W    R0,??DataTable24
        LDRSH    R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.N    ??get_irkey_1
        MOVS     R0,#+0
        B.N      ??get_irkey_2
??get_irkey_3:
        LDR.W    R1,??DataTable23_2
        LDRSH    R1,[R1, #+4]
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        LDR.W    R2,??DataTable23_3
        LDRSH    R2,[R2, R0, LSL #+1]
        CMP      R1,R2
        BNE.N    ??get_irkey_4
        LDR.W    R1,??DataTable24_1
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        STRH     R0,[R1, #+0]
??get_irkey_4:
        ADDS     R0,R0,#+1
??get_irkey_2:
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        CMP      R0,#+10
        BCC.N    ??get_irkey_3
??get_irkey_1:
        POP      {R0,PC}          ;; return

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// __interwork __softfp void data_deal(void)
data_deal:
        PUSH     {R4}
        MOVS     R0,#+0
        MOVS     R1,#+0
        MOVS     R3,#+0
        MOVS     R2,#+1
        MOVS     R4,#+0
        MOVS     R1,R4
        B.N      ??data_deal_0
??data_deal_1:
        UXTB     R3,R3            ;; ZeroExt  R3,R3,#+24,#+24
        LSRS     R3,R3,#+1
        SXTH     R2,R2            ;; SignExt  R2,R2,#+16,#+16
        LDR.W    R4,??DataTable23_4
        LDRSH    R4,[R4, R2, LSL #+1]
        CMP      R4,#+6
        BLT.N    ??data_deal_2
        ORRS     R3,R3,#0x80
??data_deal_2:
        ADDS     R2,R2,#+1
        ADDS     R0,R0,#+1
??data_deal_3:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        CMP      R0,#+8
        BLT.N    ??data_deal_1
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        LDR.W    R0,??DataTable23_2
        UXTB     R3,R3            ;; ZeroExt  R3,R3,#+24,#+24
        STRH     R3,[R0, R1, LSL #+1]
        ADDS     R1,R1,#+1
??data_deal_0:
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        CMP      R1,#+4
        BGE.N    ??data_deal_4
        MOVS     R0,#+0
        B.N      ??data_deal_3
??data_deal_4:
        LDR.W    R0,??DataTable24
        MOVS     R1,#+1
        STRH     R1,[R0, #+0]
        POP      {R4}
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// __interwork __softfp void ir_deal(void)
ir_deal:
        LDR.W    R0,??DataTable23_5
        LDRSH    R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.N    ??ir_deal_0
        LDR.W    R0,??DataTable25
        LDRSH    R0,[R0, #+0]
        CMP      R0,#+26
        BLT.N    ??ir_deal_1
        LDR.W    R0,??DataTable23_6
        MOVS     R1,#+0
        STRH     R1,[R0, #+0]
??ir_deal_1:
        LDR.W    R0,??DataTable23_6
        LDRSH    R0,[R0, #+0]
        LDR.W    R1,??DataTable23_4
        LDR.W    R2,??DataTable25
        LDRH     R2,[R2, #+0]
        STRH     R2,[R1, R0, LSL #+1]
        LDR.W    R0,??DataTable25
        MOVS     R1,#+0
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable23_6
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.W    R1,??DataTable23_6
        STRH     R0,[R1, #+0]
        LDR.W    R0,??DataTable23_6
        LDRSH    R0,[R0, #+0]
        CMP      R0,#+33
        BNE.N    ??ir_deal_2
        LDR.W    R0,??DataTable23_6
        MOVS     R1,#+0
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable23_1
        MOVS     R1,#+1
        STRH     R1,[R0, #+0]
        B.N      ??ir_deal_2
??ir_deal_0:
        LDR.W    R0,??DataTable23_5
        MOVS     R1,#+1
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable25
        MOVS     R1,#+0
        STRH     R1,[R0, #+0]
??ir_deal_2:
        BX       LR               ;; return
//   22 #include "LQ12864.h" 
//   23 #include "lptmr.h"

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
// __absolute char LPTMR_INTERRUPT
LPTMR_INTERRUPT:
        DS8 1

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// __interwork __softfp void lptmr_interrupt(void)
lptmr_interrupt:
        PUSH     {R4,LR}
        MOVW     R4,#+5000
        LDR.W    R0,??DataTable24_2
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
        BL       lptmr_clear_registers
        MOVS     R0,#+85
        BL       enable_irq
        UXTH     R0,R4            ;; ZeroExt  R0,R4,#+16,#+16
        LDR.W    R1,??DataTable25_1  ;; 0x40040008
        STR      R0,[R1, #+0]
        LDR.W    R0,??DataTable24_3  ;; 0x40040004
        MOVS     R1,#+5
        STR      R1,[R0, #+0]
        LDR.W    R0,??DataTable24_4  ;; 0x40040000
        MOVS     R1,#+64
        STR      R1,[R0, #+0]
        LDR.W    R0,??DataTable24_4  ;; 0x40040000
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x1
        LDR.W    R1,??DataTable24_4  ;; 0x40040000
        STR      R0,[R1, #+0]
??lptmr_interrupt_0:
        LDR.W    R0,??DataTable24_2
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.N    ??lptmr_interrupt_0
        LDR.W    R0,??DataTable24_4  ;; 0x40040000
        LDR      R0,[R0, #+0]
        LSRS     R0,R0,#+1
        LSLS     R0,R0,#+1
        LDR.W    R1,??DataTable24_4  ;; 0x40040000
        STR      R0,[R1, #+0]
        BL       lptmr_clear_registers
        POP      {R4,PC}          ;; return

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// __interwork __softfp void lptmr_isr_example(void)
lptmr_isr_example:
        LDR.W    R0,??DataTable24_4  ;; 0x40040000
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x80
        LDR.W    R1,??DataTable24_4  ;; 0x40040000
        STR      R0,[R1, #+0]
        LDR.W    R0,??DataTable24_2
        MOVS     R1,#+1
        STRB     R1,[R0, #+0]
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// __interwork __softfp void lptmr_time_counter()
lptmr_time_counter:
        PUSH     {R4,LR}
        MOVW     R4,#+5000
        BL       lptmr_clear_registers
        UXTH     R0,R4            ;; ZeroExt  R0,R4,#+16,#+16
        LDR.W    R1,??DataTable25_1  ;; 0x40040008
        STR      R0,[R1, #+0]
        LDR.W    R0,??DataTable24_3  ;; 0x40040004
        MOVS     R1,#+5
        STR      R1,[R0, #+0]
        LDR.W    R0,??DataTable24_4  ;; 0x40040000
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x1
        LDR.W    R1,??DataTable24_4  ;; 0x40040000
        STR      R0,[R1, #+0]
??lptmr_time_counter_0:
        LDR.W    R0,??DataTable24_4  ;; 0x40040000
        LDR      R0,[R0, #+0]
        LSLS     R0,R0,#+24
        BPL.N    ??lptmr_time_counter_0
        POP      {R4,PC}          ;; return

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// __interwork __softfp void lptmr_prescale()
lptmr_prescale:
        PUSH     {R4,LR}
        MOVS     R4,#+250
        BL       lptmr_clear_registers
        UXTH     R0,R4            ;; ZeroExt  R0,R4,#+16,#+16
        LDR.W    R1,??DataTable25_1  ;; 0x40040008
        STR      R0,[R1, #+0]
        LDR.W    R0,??DataTable24_3  ;; 0x40040004
        MOVS     R1,#+33
        STR      R1,[R0, #+0]
        LDR.W    R0,??DataTable24_4  ;; 0x40040000
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x1
        LDR.W    R1,??DataTable24_4  ;; 0x40040000
        STR      R0,[R1, #+0]
??lptmr_prescale_0:
        LDR.W    R0,??DataTable24_4  ;; 0x40040000
        LDR      R0,[R0, #+0]
        LSLS     R0,R0,#+24
        BPL.N    ??lptmr_prescale_0
        POP      {R4,PC}          ;; return

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// __interwork __softfp void lptmr_internal_ref_input()
lptmr_internal_ref_input:
        PUSH     {R4,LR}
        MOVW     R4,#+15625
        BL       lptmr_clear_registers
        LDR.W    R0,??DataTable24_5  ;; 0x40064000
        LDRB     R0,[R0, #+0]
        ORRS     R0,R0,#0x2
        LDR.W    R1,??DataTable24_5  ;; 0x40064000
        STRB     R0,[R1, #+0]
        LDR.W    R0,??DataTable24_6  ;; 0x40064001
        LDRB     R0,[R0, #+0]
        ORRS     R0,R0,#0x1
        LDR.W    R1,??DataTable24_6  ;; 0x40064001
        STRB     R0,[R1, #+0]
        UXTH     R0,R4            ;; ZeroExt  R0,R4,#+16,#+16
        LDR.W    R1,??DataTable25_1  ;; 0x40040008
        STR      R0,[R1, #+0]
        LDR.W    R0,??DataTable24_3  ;; 0x40040004
        MOVS     R1,#+64
        STR      R1,[R0, #+0]
        LDR.W    R0,??DataTable24_4  ;; 0x40040000
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x1
        LDR.W    R1,??DataTable24_4  ;; 0x40040000
        STR      R0,[R1, #+0]
??lptmr_internal_ref_input_0:
        LDR.W    R0,??DataTable24_4  ;; 0x40040000
        LDR      R0,[R0, #+0]
        LSLS     R0,R0,#+24
        BPL.N    ??lptmr_internal_ref_input_0
        POP      {R4,PC}          ;; return

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// __interwork __softfp void lptmr_external_clk_input()
lptmr_external_clk_input:
        PUSH     {R7,LR}
        BL       lptmr_clear_registers
        MOVW     R0,#+7630
        UXTH     R0,R0            ;; ZeroExt  R0,R0,#+16,#+16
        LDR.W    R1,??DataTable25_1  ;; 0x40040008
        STR      R0,[R1, #+0]
        LDR.W    R0,??DataTable24_3  ;; 0x40040004
        MOVS     R1,#+123
        STR      R1,[R0, #+0]
        LDR.W    R0,??DataTable24_4  ;; 0x40040000
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x1
        LDR.W    R1,??DataTable24_4  ;; 0x40040000
        STR      R0,[R1, #+0]
??lptmr_external_clk_input_0:
        LDR.W    R0,??DataTable24_4  ;; 0x40040000
        LDR      R0,[R0, #+0]
        LSLS     R0,R0,#+24
        BPL.N    ??lptmr_external_clk_input_0
        POP      {R0,PC}          ;; return

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// __interwork __softfp void lptmr_lpo_input()
lptmr_lpo_input:
        PUSH     {R4,LR}
        MOV      R4,#+4000
        BL       lptmr_clear_registers
        UXTH     R0,R4            ;; ZeroExt  R0,R4,#+16,#+16
        LDR.W    R1,??DataTable25_1  ;; 0x40040008
        STR      R0,[R1, #+0]
        LDR.W    R0,??DataTable24_3  ;; 0x40040004
        MOVS     R1,#+5
        STR      R1,[R0, #+0]
        LDR.W    R0,??DataTable24_4  ;; 0x40040000
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x1
        LDR.W    R1,??DataTable24_4  ;; 0x40040000
        STR      R0,[R1, #+0]
??lptmr_lpo_input_0:
        LDR.W    R0,??DataTable24_4  ;; 0x40040000
        LDR      R0,[R0, #+0]
        LSLS     R0,R0,#+24
        BPL.N    ??lptmr_lpo_input_0
        POP      {R4,PC}          ;; return

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// __interwork __softfp void lptmr_32khz_input()
lptmr_32khz_input:
        PUSH     {R4,LR}
        MOV      R4,#+32768
        BL       lptmr_clear_registers
        LDR.W    R0,??DataTable25_2  ;; 0x4004803c
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x20000000
        LDR.W    R1,??DataTable25_2  ;; 0x4004803c
        STR      R0,[R1, #+0]
        LDR.W    R0,??DataTable25_3  ;; 0x4003d010
        LDR      R0,[R0, #+0]
        MOV      R1,#+256
        ORRS     R0,R1,R0
        LDR.W    R1,??DataTable25_3  ;; 0x4003d010
        STR      R0,[R1, #+0]
        LDR.W    R0,??DataTable25_4  ;; 0x40047000
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x80000
        LDR.W    R1,??DataTable25_4  ;; 0x40047000
        STR      R0,[R1, #+0]
        UXTH     R0,R4            ;; ZeroExt  R0,R4,#+16,#+16
        LDR.W    R1,??DataTable25_1  ;; 0x40040008
        STR      R0,[R1, #+0]
        LDR.W    R0,??DataTable24_3  ;; 0x40040004
        MOVS     R1,#+10
        STR      R1,[R0, #+0]
        LDR.W    R0,??DataTable24_4  ;; 0x40040000
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x1
        LDR.W    R1,??DataTable24_4  ;; 0x40040000
        STR      R0,[R1, #+0]
??lptmr_32khz_input_0:
        LDR.W    R0,??DataTable24_4  ;; 0x40040000
        LDR      R0,[R0, #+0]
        LSLS     R0,R0,#+24
        BPL.N    ??lptmr_32khz_input_0
        POP      {R4,PC}          ;; return

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// __interwork __softfp void lptmr_pulse_counter(char)
lptmr_pulse_counter:
        PUSH     {R3-R5,LR}
        MOVS     R4,R0
        MOVW     R5,#+10000
        LDR.W    R0,??DataTable25_5  ;; 0x40048038
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x1
        LDR.W    R1,??DataTable25_5  ;; 0x40048038
        STR      R0,[R1, #+0]
        BL       lptmr_clear_registers
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BNE.N    ??lptmr_pulse_counter_0
        LDR.W    R0,??DataTable25_5  ;; 0x40048038
        LDR      R0,[R0, #+0]
        MOV      R1,#+512
        ORRS     R0,R1,R0
        LDR.W    R1,??DataTable25_5  ;; 0x40048038
        STR      R0,[R1, #+0]
        LDR.W    R0,??DataTable25_6  ;; 0x4004904c
        MOV      R1,#+1536
        STR      R1,[R0, #+0]
??lptmr_pulse_counter_1:
        LDR.W    R0,??DataTable24_3  ;; 0x40040004
        MOVS     R1,#+5
        STR      R1,[R0, #+0]
        UXTH     R0,R5            ;; ZeroExt  R0,R5,#+16,#+16
        LDR.W    R1,??DataTable25_1  ;; 0x40040008
        STR      R0,[R1, #+0]
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LSLS     R0,R4,#+4
        ANDS     R0,R0,#0x30
        ORRS     R0,R0,#0x2
        LDR.W    R1,??DataTable24_4  ;; 0x40040000
        STR      R0,[R1, #+0]
        LDR.W    R0,??DataTable24_4  ;; 0x40040000
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x1
        LDR.W    R1,??DataTable24_4  ;; 0x40040000
        STR      R0,[R1, #+0]
??lptmr_pulse_counter_2:
        POP      {R0,R4,R5,PC}    ;; return
??lptmr_pulse_counter_0:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+2
        BNE.N    ??lptmr_pulse_counter_3
        LDR.W    R0,??DataTable25_5  ;; 0x40048038
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x800
        LDR.W    R1,??DataTable25_5  ;; 0x40048038
        STR      R0,[R1, #+0]
        LDR.W    R0,??DataTable27  ;; 0x4004b014
        MOV      R1,#+1024
        STR      R1,[R0, #+0]
        B.N      ??lptmr_pulse_counter_1
??lptmr_pulse_counter_3:
        B.N      ??lptmr_pulse_counter_2

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// __interwork __softfp uint16 get_counter_value()
get_counter_value:
        LDR.W    R0,??DataTable27_1  ;; 0x4004000c
        MOVS     R1,#+1
        STR      R1,[R0, #+0]
        LDR.W    R0,??DataTable27_1  ;; 0x4004000c
        LDR      R0,[R0, #+0]
        UXTH     R0,R0            ;; ZeroExt  R0,R0,#+16,#+16
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// __interwork __softfp void lptmr_clear_registers()
lptmr_clear_registers:
        LDR.W    R0,??DataTable24_4  ;; 0x40040000
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
        LDR.W    R0,??DataTable24_3  ;; 0x40040004
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
        LDR.W    R0,??DataTable25_1  ;; 0x40040008
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
        BX       LR               ;; return
//   24 #define READword(address)     ((unsigned int)(*(volatile unsigned int *)(address)))
//   25 #define FLEXNVM_startline_ADDR 0X10000000
//   26 #define FLEXRAM_startline_ADDR 0X14000000
//   27 #define PIT0 0
//   28 #define PIT1 1
//   29 #define PIT2 2
//   30 #define PIT3 3
//   31 #define GPIO_PIN_MASK      0x1Fu    //0x1f=31,限制位数为0--31有效
//   32 #define GPIO_PIN(x)        (((1)<<(x & GPIO_PIN_MASK)))  //把当前位置1                
//   33 #define endlineROW  169        //OV7620，每场240行
//   34 #define  IMG_ROWS 55
//   35 #define  IMG_COLS 160
//   36 #define  video_Middle  77/76//91偏左//85
//   37 

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//   38 int16 Servo_Middle=8151;//8151;   //中间  8155
Servo_Middle:
        DATA
        DC16 8151

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//   39 int16 Servo_Left=9320;    //最左10475   250Hz  44%
Servo_Left:
        DATA
        DC16 9320

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//   40 int16 Servo_Right=7035;    //最右15575   250Hz  30.53%7035
Servo_Right:
        DATA
        DC16 7035
//   41 
//   42 #define  unclear  -100
//   43 #define  ok        1
//   44 #define  fail      0
//   45 #define  position_mid   77      //中心位置
//   46 

        SECTION `.bss`:DATA:REORDER:NOROOT(2)
//   47 int tu_flag=0;
tu_flag:
        DS8 4

        SECTION `.bss`:DATA:REORDER:NOROOT(2)
//   48 volatile int P_timer=0;
P_timer:
        DS8 4
//   49 

        SECTION `.bss`:DATA:REORDER:NOROOT(2)
//   50 unsigned int row=0;//摄像头行计数，最大240
row:
        DS8 4

        SECTION `.bss`:DATA:REORDER:NOROOT(2)
//   51 volatile uint8 ImageData[IMG_ROWS][IMG_COLS];//存放数据数组
ImageData:
        DS8 8800

        SECTION `.bss`:DATA:REORDER:NOROOT(2)
//   52 volatile uint8 ImageData2[IMG_ROWS][IMG_COLS];
ImageData2:
        DS8 8800

        SECTION `.bss`:DATA:REORDER:NOROOT(2)
//   53 volatile uint16 ption[160];
ption:
        DS8 320

        SECTION `.bss`:DATA:REORDER:NOROOT(2)
//   54 volatile int position[IMG_ROWS];//存放中心线
position:
        DS8 220

        SECTION `.bss`:DATA:REORDER:NOROOT(2)
//   55 volatile int backup[IMG_ROWS];//存放中心线
backup:
        DS8 220

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   56 volatile uint8 Very_straight=0;//这个需要清零
Very_straight:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   57 volatile uint8 ramp_flag=0;//这个需要清零
ramp_flag:
        DS8 1

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//   58 volatile int16 Judge_startline=150;
Judge_startline:
        DATA
        DC16 150

        SECTION `.bss`:DATA:REORDER:NOROOT(2)
//   59 unsigned int imagerow=0;//采集行计数，最大H
imagerow:
        DS8 4

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   60 int16 dijihang=0;
dijihang:
        DS8 2

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
        DATA
        DC32 0
        DC8 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
        DATA
        DC32 0
        DC8 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
        DATA
        DC32 0
        DC8 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
        DATA
        DC32 0
        DC8 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
        DATA
        DC32 0
        DC8 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
        DATA
        DC32 0
        DC8 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
        DATA
        DC32 0
        DC8 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "Mid_Wu">`:
        DATA
        DC8 "Mid_Wu"
        DC8 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "High_Wu">`:
        DATA
        DC8 "High_Wu"

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "High_Mid">`:
        DATA
        DC8 "High_Mid"
        DC8 0, 0, 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "High_You">`:
        DATA
        DC8 "High_You"
        DC8 0, 0, 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
        DATA
        DC8 "OK"
        DC8 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant {1, 2, 3, 3, 4}>`:
        DATA
        DC8 1, 2, 3, 3, 4, 0, 0, 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
        DATA
        DC16 0
        DC8 0, 0, 0, 0, 0, 0, 0, 0, 0, 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
        DATA
        DC16 0
        DC8 0, 0, 0, 0, 0, 0, 0, 0, 0, 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
        DATA
        DC16 0
        DC8 0, 0, 0, 0, 0, 0, 0, 0, 0, 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
        DATA
        DC16 0
        DC8 0, 0, 0, 0, 0, 0, 0, 0, 0, 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
        DATA
        DC16 0
        DC8 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
        DATA
        DC16 0
        DC8 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
        DATA
        DC16 0
        DC8 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "speed">`:
        DATA
        DC8 "speed"
        DC8 0, 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "J_line">`:
        DATA
        DC8 "J_line"
        DC8 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "TJorFC">`:
        DATA
        DC8 "TJorFC"
        DC8 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
        DATA
        DC8 "FC"
        DC8 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
        DATA
        DC8 "TJ"
        DC8 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "sai_dao_number">`:
        DATA
        DC8 "sai_dao_number"
        DC8 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "LCDgo_on">`:
        DATA
        DC8 "LCDgo_on"
        DC8 0, 0, 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "low_0">`:
        DATA
        DC8 "low_0"
        DC8 0, 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "mid_0">`:
        DATA
        DC8 "mid_0"
        DC8 0, 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "high_0">`:
        DATA
        DC8 "high_0"
        DC8 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "position_now">`:
        DATA
        DC8 "position_now"
        DC8 0, 0, 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "position_diff">`:
        DATA
        DC8 "position_diff"
        DC8 0, 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "position_now_low">`:
        DATA
        DC8 "position_now_low"
        DC8 0, 0, 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "Servo_value">`:
        DATA
        DC8 "Servo_value"

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
`?<Constant "dijihang">`:
        DATA
        DC8 "dijihang"
        DC8 0, 0, 0

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
//   61 unsigned int const data_table[IMG_ROWS]={ 1,   3,   5,   7, 9,//2 
data_table:
        DATA
        DC32 1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31, 34, 37
        DC32 40, 43, 46, 49, 52, 55, 58, 61, 64, 67, 70, 73, 76, 79, 82, 85, 88
        DC32 92, 96, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144
        DC32 148, 152, 156, 160, 164, 168
//   62                                    11,  13,  15,  17,  19,    //2
//   63                                    21,  23,  25,  27,  29,    //2
//   64                                    31,  34,  37,  40,  43,    //3
//   65                                    46,  49,  52,  55,  58,    //3
//   66                                    61,  64,  67,  70,  73,    //3
//   67                                    76,  79,  82,  85,  88,    //3
//   68                                    92,  96, 100, 104, 108,    //4
//   69                                    112, 116, 120, 124, 128,    //4
//   70                                    132, 136, 140, 144, 148,    //4
//   71                                    152, 156, 160, 164, 168     //4         
//   72                                    };
//   73 
//   74 

        SECTION `.rodata`:CONST:REORDER:NOROOT(2)
//   75 const unsigned char Wspeed_diff[2][60]={{100,100,100,100,100,100,100,100,100,100,  //差速表
Wspeed_diff:
        DATA
        DC8 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 99, 99, 99, 99
        DC8 98, 98, 98, 97, 97, 97, 97, 96, 96, 96, 95, 94, 93, 92, 91, 90, 89
        DC8 88, 87, 86, 85, 84, 83, 82, 81, 80, 79, 78, 77, 76, 75, 74, 73, 72
        DC8 71, 70, 69, 68, 67, 66, 65, 64, 63, 62, 61, 60, 100, 100, 100, 100
        DC8 100, 100, 100, 100, 100, 100, 101, 102, 103, 104, 105, 106, 107
        DC8 108, 109, 110, 111, 112, 113, 114, 115, 117, 119, 121, 122, 124
        DC8 126, 128, 130, 132, 134, 136, 138, 140, 142, 144, 146, 148, 150
        DC8 152, 153, 154, 155, 157, 159, 161, 163, 165, 167, 168, 169, 170
        DC8 172, 174, 176, 178
//   76                                           99,99,99,99, 98, 98, 98, 97, 97, 97,     /////////自己注释
//   77                                           //96, 96, 96, 96, 95, 95, 94, 94, 94, 93,
//   78                                           97,96,96, 96, 95, 94, 93, 92,91,90,
//   79                                          //93, 93, 91, 91, 89, 89, 87, 87, 85, 85, 
//   80                                          89,88,87,86,85,84,83,82,81,80,
//   81                                           //, 83, 81, 81, 79, 79, 77, 77, 75, 75,
//   82                                           79,78,77,76,75,74,73,72,71,70, 
//   83                                         // 73, 73, 71, 71, 69, 67, 65, 63, 61, 59
//   84                                          69,68,67,66,65,64,63,62,61,60
//   85                                         },                                                                     
//   86                                        { 
//   87                                        //100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
//   88                                        100, 100, 100, 100, 100,100,100,100,100, 100,
//   89                                        //101, 101, 101, 101, 101, 101, 101, 101, 101, 103,
//   90                                         //104, 105, 106, 107, 108, 109, 110, 111, 112, 113,
//   91                                         101, 102, 103, 104, 105, 106, 107, 108, 109, 110,
//   92                                         //114, 115, 116, 117, 118, 119, 120, 121, 122, 123,
//   93                                          111, 112, 113, 114, 115, 117,119, 121, 122, 124,
//   94                                         //125, 127, 129, 131, 133, 135, 137, 139, 141, 143,
//   95                                         126,128,130,132,134,136,138,140,142,144,
//   96                                         // 128, 132, 137, 142, 148, 154, 160, 166, 170, 175,
//   97                                          146,148,150, 152,153,154,155,157,159,161,
//   98                                          163,165,167,168,169,170,172,174,176,178
//   99                                        }
//  100                                       }; 
//  101 
//  102   //找根参数b
//  103 //int b=48;

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  104    uint8 black2_a1 = 145;//135 140
black2_a1:
        DATA
        DC8 145

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  105   uint8 black2_a2 = 155; //145 145
black2_a2:
        DATA
        DC8 155

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  106   uint8 black2_a3 = 145;// 135 140
black2_a3:
        DATA
        DC8 145

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  107   uint8 black2_b1 = 165;//150 135
black2_b1:
        DATA
        DC8 165

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  108   uint8 black2_b2 = 165;//150 140
black2_b2:
        DATA
        DC8 165

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  109   uint8 black2_b3 = 165;//150 135
black2_b3:
        DATA
        DC8 165

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  110   uint8 black2_c1 = 168;//145 154   132
black2_c1:
        DATA
        DC8 168

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  111   uint8 black2_c2 = 169;//158 158     138
black2_c2:
        DATA
        DC8 169

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  112   uint8 black2_c3 = 168;// 145 154     132
black2_c3:
        DATA
        DC8 168

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  113   uint8 black2_d1 = 162;//152 157  130
black2_d1:
        DATA
        DC8 162

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  114   uint8 black2_d2 = 163;//153 158  142
black2_d2:
        DATA
        DC8 163

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  115   uint8 black2_d3 = 162; //152 157  130
black2_d3:
        DATA
        DC8 162
//  116   
//  117   
//  118   

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  119   uint8 black_a1 = 135;//140
black_a1:
        DATA
        DC8 135

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  120   uint8 black_a2 =135;//145
black_a2:
        DATA
        DC8 135

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  121   uint8 black_a3 = 135;//140
black_a3:
        DATA
        DC8 135

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  122   uint8 black_b1 = 150;//135
black_b1:
        DATA
        DC8 150

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  123   uint8 black_b2 = 150;//140
black_b2:
        DATA
        DC8 150

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  124   uint8 black_b3 = 150;//135
black_b3:
        DATA
        DC8 150

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  125   uint8 black_c1 = 145;//154   132
black_c1:
        DATA
        DC8 145

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  126   uint8 black_c2 = 158;//158     138
black_c2:
        DATA
        DC8 158

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  127   uint8 black_c3 = 145;//154     132
black_c3:
        DATA
        DC8 145

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  128   uint8 black_d1 = 152;//157  130
black_d1:
        DATA
        DC8 152

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  129   uint8 black_d2 = 153;//158  142
black_d2:
        DATA
        DC8 153

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  130   uint8 black_d3 = 152;//157  130
black_d3:
        DATA
        DC8 152

        SECTION `.bss`:DATA:REORDER:NOROOT(2)
//  131 uint8 row_F[IMG_ROWS];//该行采集完成标志
row_F:
        DS8 56

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  132 char startline;//起始行
startline:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  133 char endline;//结束行
endline:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  134 char startline_F;//发现起始行
startline_F:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  135 char endline_F;//发现结束行
endline_F:
        DS8 1
//  136 

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  137 uint8 LCD_P1=0,LCD_P2=0,LCD_P3=0;
LCD_P1:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
LCD_P2:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
LCD_P3:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  138 volatile uint16 pulse=0,pulse2=0;
pulse:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
pulse2:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  139 volatile uint8 finish=0;
finish:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(2)
//  140  int start_position=0;
start_position:
        DS8 4
//  141 
//  142 

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  143 volatile uint8 sai_dao_lei_xing=0;
sai_dao_lei_xing:
        DS8 1
//  144 

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  145 volatile int16 low_0=0,mid_0=0,high_0=0;
low_0:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
mid_0:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
high_0:
        DS8 2
//  146 

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  147 volatile int16 low=0,mid=0,high=0;
`low`:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
mid:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
`high`:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  148 volatile int16 topline=0;
topline:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  149 volatile int16 low_old=0,mid_old=0,high_old=0;
low_old:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
mid_old:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
high_old:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  150 volatile int16 position_now=0,position_diff=0,position_now_low=0;
position_now:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
position_diff:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
position_now_low:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  151 volatile int16 Servo_value=0,Servo_value_old=13015;//这里Servo_value_old的初始值要设置的和Servo_Middle一样，是用来限定的
Servo_value:
        DS8 2

        SECTION `.data`:DATA:REORDER:NOROOT(1)
Servo_value_old:
        DATA
        DC16 13015

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  152 volatile int16 low_now=0,mid_now=0,high_now=0;
low_now:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
mid_now:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
high_now:
        DS8 2
//  153 

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  154 volatile int16 low_now_0=0,mid_now_0=0,high_now_0=0;
low_now_0:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
mid_now_0:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
high_now_0:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  155 volatile int16 END_0=0;
END_0:
        DS8 2
//  156 

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  157 volatile uint8 car_speed=0;
car_speed:
        DS8 1

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  158 volatile int16 speed_Very_straight,speed_top,speed_b,speed_m,speed_s,speed_ms;
speed_Very_straight:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
speed_top:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
speed_b:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
speed_m:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
speed_s:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
speed_ms:
        DS8 2
//  159 

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  160 volatile int16 time_m=0,speed_now=0;
time_m:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
speed_now:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  161 volatile int16 s_old=0;
s_old:
        DS8 2
//  162 //volatile int16 position_now_P=0,position_now_D=0;
//  163 
//  164 ////////////模糊控制///////////////
//  165 #define MP_S      400//110  260  360
//  166 #define MP_M      440//180  320  400
//  167 #define MP_B      480//240  360  440
//  168 
//  169 #define MI_S      6//6 4
//  170 #define MI_M      9//10 8 
//  171 #define MI_B      12//15  10
//  172 
//  173 #define  motor_p      70//   45 50 55 60 65
//  174 
//  175 
//  176 //////////////////////舵机参数////////////////////////////
//  177 

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  178 uint8 P_High=12;//uint8 P_High=11;//27  29
P_High:
        DATA
        DC8 12

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  179 uint8 D_High=6;//uint8 D_High=0;//18  15
D_High:
        DATA
        DC8 6

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  180 uint8 H_High=17;//uint8 H_High=20;
H_High:
        DATA
        DC8 17

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  181 uint8 T_High=29;//41//uint8 T_High=25;
T_High:
        DATA
        DC8 29
//  182 
//  183 

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  184 uint8 P_Mid=10;//18//15//12//13
P_Mid:
        DATA
        DC8 10

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  185 uint8 D_Mid=16;//18//13//11//9//10
D_Mid:
        DATA
        DC8 16

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  186 uint8 H_Mid=15;//uint8 H_Mid=15;
H_Mid:
        DATA
        DC8 15

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  187 uint8 T_Mid=45;//uint8 T_Mid=50;
T_Mid:
        DATA
        DC8 45
//  188 
//  189 

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  190 uint8 P_Mid1=10;
P_Mid1:
        DATA
        DC8 10

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  191 uint8 D_Mid1=21;
D_Mid1:
        DATA
        DC8 21

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  192 uint8 H_Mid1=30;
H_Mid1:
        DATA
        DC8 30

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  193 uint8 T_Mid1=45;
T_Mid1:
        DATA
        DC8 45
//  194 
//  195 

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  196 uint8 P_Low=25;//21//18//15//14//
P_Low:
        DATA
        DC8 25

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  197 uint8 D_Low=41;//21//15//11//13
D_Low:
        DATA
        DC8 41

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  198 uint8 H_Low=29;//H_Low=26;
H_Low:
        DATA
        DC8 29

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  199 uint8 T_Low=50;//T_Low=50;
T_Low:
        DATA
        DC8 50
//  200 
//  201 #include "keyboard_PD.h"

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// __interwork __softfp void High_You(void)
High_You:
        PUSH     {R4,LR}
        MOVS     R4,#+0
        B.N      ??High_You_0
??High_You_1:
        LDR.W    R0,??DataTable21_3
        MOVS     R1,#+50
        STRB     R1,[R0, #+0]
        BL       CH451_GetKeyValue
??High_You_2:
        LDR.W    R0,??DataTable21_3
        LDRB     R0,[R0, #+0]
        CMP      R0,#+1
        BEQ.N    ??High_You_1
        LDR.W    R0,??DataTable25_7
        LDRB     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.W    R1,??DataTable25_7
        STRB     R0,[R1, #+0]
        LDR.W    R0,??DataTable25_7
        LDRB     R2,[R0, #+0]
        SXTH     R2,R2            ;; SignExt  R2,R2,#+16,#+16
        MOVS     R1,#+3
        MOVS     R0,#+72
        BL       LCD_P6x8Str_3
??High_You_0:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+0
        BNE.N    ??High_You_3
        BL       CH451_GetKeyValue
        LDR.W    R0,??DataTable21_3
        LDRB     R0,[R0, #+0]
        CMP      R0,#+1
        BEQ.N    ??High_You_2
        CMP      R0,#+2
        BEQ.N    ??High_You_4
        CMP      R0,#+5
        BEQ.N    ??High_You_5
        CMP      R0,#+6
        BEQ.N    ??High_You_6
        CMP      R0,#+9
        BEQ.N    ??High_You_7
        CMP      R0,#+10
        BEQ.N    ??High_You_8
        B.N      ??High_You_0
??High_You_9:
        LDR.W    R0,??DataTable21_3
        MOVS     R1,#+50
        STRB     R1,[R0, #+0]
        BL       CH451_GetKeyValue
??High_You_7:
        LDR.W    R0,??DataTable21_3
        LDRB     R0,[R0, #+0]
        CMP      R0,#+9
        BEQ.N    ??High_You_9
        LDR.W    R0,??DataTable25_8
        LDRB     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.W    R1,??DataTable25_8
        STRB     R0,[R1, #+0]
        LDR.W    R0,??DataTable25_8
        LDRB     R2,[R0, #+0]
        SXTH     R2,R2            ;; SignExt  R2,R2,#+16,#+16
        MOVS     R1,#+3
        MOVS     R0,#+92
        BL       LCD_P6x8Str_3
        B.N      ??High_You_0
??High_You_10:
        LDR.W    R0,??DataTable21_3
        MOVS     R1,#+50
        STRB     R1,[R0, #+0]
        BL       CH451_GetKeyValue
??High_You_5:
        LDR.W    R0,??DataTable21_3
        LDRB     R0,[R0, #+0]
        CMP      R0,#+5
        BEQ.N    ??High_You_10
        LDR.W    R0,??DataTable25_7
        LDRB     R0,[R0, #+0]
        SUBS     R0,R0,#+1
        LDR.W    R1,??DataTable25_7
        STRB     R0,[R1, #+0]
        LDR.W    R0,??DataTable25_7
        LDRB     R2,[R0, #+0]
        SXTH     R2,R2            ;; SignExt  R2,R2,#+16,#+16
        MOVS     R1,#+3
        MOVS     R0,#+72
        BL       LCD_P6x8Str_3
        B.N      ??High_You_0
??High_You_11:
        LDR.W    R0,??DataTable21_3
        MOVS     R1,#+50
        STRB     R1,[R0, #+0]
        BL       CH451_GetKeyValue
??High_You_8:
        LDR.W    R0,??DataTable21_3
        LDRB     R0,[R0, #+0]
        CMP      R0,#+10
        BEQ.N    ??High_You_11
        LDR.W    R0,??DataTable25_8
        LDRB     R0,[R0, #+0]
        SUBS     R0,R0,#+1
        LDR.W    R1,??DataTable25_8
        STRB     R0,[R1, #+0]
        LDR.W    R0,??DataTable25_8
        LDRB     R2,[R0, #+0]
        SXTH     R2,R2            ;; SignExt  R2,R2,#+16,#+16
        MOVS     R1,#+3
        MOVS     R0,#+92
        BL       LCD_P6x8Str_3
        B.N      ??High_You_0
??High_You_12:
        LDR.W    R0,??DataTable21_3
        MOVS     R1,#+50
        STRB     R1,[R0, #+0]
        BL       CH451_GetKeyValue
??High_You_4:
        LDR.W    R0,??DataTable21_3
        LDRB     R0,[R0, #+0]
        CMP      R0,#+2
        BEQ.N    ??High_You_12
        B.N      ??High_You_0
??High_You_13:
        LDR.W    R0,??DataTable21_3
        MOVS     R1,#+50
        STRB     R1,[R0, #+0]
        BL       CH451_GetKeyValue
??High_You_6:
        LDR.W    R0,??DataTable21_3
        LDRB     R0,[R0, #+0]
        CMP      R0,#+6
        BEQ.N    ??High_You_13
        MOVS     R4,#+1
        B.N      ??High_You_0
??High_You_3:
        POP      {R4,PC}          ;; return

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// __interwork __softfp void High_Wu(void)
High_Wu:
        PUSH     {R4,LR}
        MOVS     R4,#+0
        B.N      ??High_Wu_0
??High_Wu_1:
        LDR.W    R0,??DataTable21_3
        MOVS     R1,#+50
        STRB     R1,[R0, #+0]
        BL       CH451_GetKeyValue
??High_Wu_2:
        LDR.W    R0,??DataTable21_3
        LDRB     R0,[R0, #+0]
        CMP      R0,#+1
        BEQ.N    ??High_Wu_1
        LDR.W    R0,??DataTable26
        LDRB     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.W    R1,??DataTable26
        STRB     R0,[R1, #+0]
        LDR.W    R0,??DataTable26
        LDRB     R2,[R0, #+0]
        SXTH     R2,R2            ;; SignExt  R2,R2,#+16,#+16
        MOVS     R1,#+3
        MOVS     R0,#+72
        BL       LCD_P6x8Str_3
??High_Wu_0:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+0
        BNE.N    ??High_Wu_3
        BL       CH451_GetKeyValue
        LDR.W    R0,??DataTable21_3
        LDRB     R0,[R0, #+0]
        CMP      R0,#+1
        BEQ.N    ??High_Wu_2
        CMP      R0,#+2
        BEQ.N    ??High_Wu_4
        CMP      R0,#+5
        BEQ.N    ??High_Wu_5
        CMP      R0,#+6
        BEQ.N    ??High_Wu_6
        CMP      R0,#+9
        BEQ.N    ??High_Wu_7
        CMP      R0,#+10
        BEQ.N    ??High_Wu_8
        B.N      ??High_Wu_0
??High_Wu_9:
        LDR.N    R0,??DataTable21_3
        MOVS     R1,#+50
        STRB     R1,[R0, #+0]
        BL       CH451_GetKeyValue
??High_Wu_7:
        LDR.N    R0,??DataTable21_3
        LDRB     R0,[R0, #+0]
        CMP      R0,#+9
        BEQ.N    ??High_Wu_9
        LDR.W    R0,??DataTable26_1
        LDRB     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.W    R1,??DataTable26_1
        STRB     R0,[R1, #+0]
        LDR.W    R0,??DataTable26_1
        LDRB     R2,[R0, #+0]
        SXTH     R2,R2            ;; SignExt  R2,R2,#+16,#+16
        MOVS     R1,#+3
        MOVS     R0,#+92
        BL       LCD_P6x8Str_3
        B.N      ??High_Wu_0
??High_Wu_10:
        LDR.N    R0,??DataTable21_3
        MOVS     R1,#+50
        STRB     R1,[R0, #+0]
        BL       CH451_GetKeyValue
??High_Wu_5:
        LDR.N    R0,??DataTable21_3
        LDRB     R0,[R0, #+0]
        CMP      R0,#+5
        BEQ.N    ??High_Wu_10
        LDR.W    R0,??DataTable26
        LDRB     R0,[R0, #+0]
        SUBS     R0,R0,#+1
        LDR.W    R1,??DataTable26
        STRB     R0,[R1, #+0]
        LDR.W    R0,??DataTable26
        LDRB     R2,[R0, #+0]
        SXTH     R2,R2            ;; SignExt  R2,R2,#+16,#+16
        MOVS     R1,#+3
        MOVS     R0,#+72
        BL       LCD_P6x8Str_3
        B.N      ??High_Wu_0
??High_Wu_11:
        LDR.N    R0,??DataTable21_3
        MOVS     R1,#+50
        STRB     R1,[R0, #+0]
        BL       CH451_GetKeyValue
??High_Wu_8:
        LDR.N    R0,??DataTable21_3
        LDRB     R0,[R0, #+0]
        CMP      R0,#+10
        BEQ.N    ??High_Wu_11
        LDR.W    R0,??DataTable26_1
        LDRB     R0,[R0, #+0]
        SUBS     R0,R0,#+1
        LDR.W    R1,??DataTable26_1
        STRB     R0,[R1, #+0]
        LDR.W    R0,??DataTable26_1
        LDRB     R2,[R0, #+0]
        SXTH     R2,R2            ;; SignExt  R2,R2,#+16,#+16
        MOVS     R1,#+3
        MOVS     R0,#+92
        BL       LCD_P6x8Str_3
        B.N      ??High_Wu_0
??High_Wu_12:
        LDR.N    R0,??DataTable21_3
        MOVS     R1,#+50
        STRB     R1,[R0, #+0]
        BL       CH451_GetKeyValue
??High_Wu_4:
        LDR.N    R0,??DataTable21_3
        LDRB     R0,[R0, #+0]
        CMP      R0,#+2
        BEQ.N    ??High_Wu_12
        B.N      ??High_Wu_0
??High_Wu_13:
        LDR.N    R0,??DataTable21_3
        MOVS     R1,#+50
        STRB     R1,[R0, #+0]
        BL       CH451_GetKeyValue
??High_Wu_6:
        LDR.N    R0,??DataTable21_3
        LDRB     R0,[R0, #+0]
        CMP      R0,#+6
        BEQ.N    ??High_Wu_13
        MOVS     R4,#+1
        B.N      ??High_Wu_0
??High_Wu_3:
        POP      {R4,PC}          ;; return

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// __interwork __softfp void High_Mid(void)
High_Mid:
        PUSH     {R4,LR}
        MOVS     R4,#+0
        B.N      ??High_Mid_0
??High_Mid_1:
        LDR.N    R0,??DataTable21_3
        MOVS     R1,#+50
        STRB     R1,[R0, #+0]
        BL       CH451_GetKeyValue
??High_Mid_2:
        LDR.N    R0,??DataTable21_3
        LDRB     R0,[R0, #+0]
        CMP      R0,#+1
        BEQ.N    ??High_Mid_1
        LDR.W    R0,??DataTable27_2
        LDRB     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.W    R1,??DataTable27_2
        STRB     R0,[R1, #+0]
        LDR.W    R0,??DataTable27_2
        LDRB     R2,[R0, #+0]
        SXTH     R2,R2            ;; SignExt  R2,R2,#+16,#+16
        MOVS     R1,#+3
        MOVS     R0,#+72
        BL       LCD_P6x8Str_3
??High_Mid_0:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+0
        BNE.N    ??High_Mid_3
        BL       CH451_GetKeyValue
        LDR.N    R0,??DataTable21_3
        LDRB     R0,[R0, #+0]
        CMP      R0,#+1
        BEQ.N    ??High_Mid_2
        CMP      R0,#+2
        BEQ.N    ??High_Mid_4
        CMP      R0,#+5
        BEQ.N    ??High_Mid_5
        CMP      R0,#+6
        BEQ.N    ??High_Mid_6
        CMP      R0,#+9
        BEQ.N    ??High_Mid_7
        CMP      R0,#+10
        BEQ.N    ??High_Mid_8
        B.N      ??High_Mid_0
??High_Mid_9:
        LDR.N    R0,??DataTable21_3
        MOVS     R1,#+50
        STRB     R1,[R0, #+0]
        BL       CH451_GetKeyValue
??High_Mid_7:
        LDR.N    R0,??DataTable21_3
        LDRB     R0,[R0, #+0]
        CMP      R0,#+9
        BEQ.N    ??High_Mid_9
        LDR.W    R0,??DataTable27_3
        LDRB     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.W    R1,??DataTable27_3
        STRB     R0,[R1, #+0]
        LDR.W    R0,??DataTable27_3
        LDRB     R2,[R0, #+0]
        SXTH     R2,R2            ;; SignExt  R2,R2,#+16,#+16
        MOVS     R1,#+3
        MOVS     R0,#+92
        BL       LCD_P6x8Str_3
        B.N      ??High_Mid_0
??High_Mid_10:
        LDR.N    R0,??DataTable21_3
        MOVS     R1,#+50
        STRB     R1,[R0, #+0]
        BL       CH451_GetKeyValue
??High_Mid_5:
        LDR.N    R0,??DataTable21_3
        LDRB     R0,[R0, #+0]
        CMP      R0,#+5
        BEQ.N    ??High_Mid_10
        LDR.W    R0,??DataTable27_2
        LDRB     R0,[R0, #+0]
        SUBS     R0,R0,#+1
        LDR.W    R1,??DataTable27_2
        STRB     R0,[R1, #+0]
        LDR.W    R0,??DataTable27_2
        LDRB     R2,[R0, #+0]
        SXTH     R2,R2            ;; SignExt  R2,R2,#+16,#+16
        MOVS     R1,#+3
        MOVS     R0,#+72
        BL       LCD_P6x8Str_3
        B.N      ??High_Mid_0
??High_Mid_11:
        LDR.N    R0,??DataTable21_3
        MOVS     R1,#+50
        STRB     R1,[R0, #+0]
        BL       CH451_GetKeyValue
??High_Mid_8:
        LDR.N    R0,??DataTable21_3
        LDRB     R0,[R0, #+0]
        CMP      R0,#+10
        BEQ.N    ??High_Mid_11
        LDR.W    R0,??DataTable27_3
        LDRB     R0,[R0, #+0]
        SUBS     R0,R0,#+1
        LDR.W    R1,??DataTable27_3
        STRB     R0,[R1, #+0]
        LDR.W    R0,??DataTable27_3
        LDRB     R2,[R0, #+0]
        SXTH     R2,R2            ;; SignExt  R2,R2,#+16,#+16
        MOVS     R1,#+3
        MOVS     R0,#+92
        BL       LCD_P6x8Str_3
        B.N      ??High_Mid_0
??High_Mid_12:
        LDR.N    R0,??DataTable21_3
        MOVS     R1,#+50
        STRB     R1,[R0, #+0]
        BL       CH451_GetKeyValue
??High_Mid_4:
        LDR.N    R0,??DataTable21_3
        LDRB     R0,[R0, #+0]
        CMP      R0,#+2
        BEQ.N    ??High_Mid_12
        B.N      ??High_Mid_0
??High_Mid_13:
        LDR.N    R0,??DataTable21_3
        MOVS     R1,#+50
        STRB     R1,[R0, #+0]
        BL       CH451_GetKeyValue
??High_Mid_6:
        LDR.N    R0,??DataTable21_3
        LDRB     R0,[R0, #+0]
        CMP      R0,#+6
        BEQ.N    ??High_Mid_13
        MOVS     R4,#+1
        B.N      ??High_Mid_0
??High_Mid_3:
        POP      {R4,PC}          ;; return

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// __interwork __softfp void Mid_Wu(void)
Mid_Wu:
        PUSH     {R4,LR}
        MOVS     R4,#+0
        B.N      ??Mid_Wu_0
??Mid_Wu_1:
        LDR.W    R0,??DataTable34
        MOVS     R1,#+50
        STRB     R1,[R0, #+0]
        BL       CH451_GetKeyValue
??Mid_Wu_2:
        LDR.W    R0,??DataTable34
        LDRB     R0,[R0, #+0]
        CMP      R0,#+1
        BEQ.N    ??Mid_Wu_1
        LDR.W    R0,??DataTable28
        LDRB     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.W    R1,??DataTable28
        STRB     R0,[R1, #+0]
        LDR.W    R0,??DataTable28
        LDRB     R2,[R0, #+0]
        SXTH     R2,R2            ;; SignExt  R2,R2,#+16,#+16
        MOVS     R1,#+3
        MOVS     R0,#+72
        BL       LCD_P6x8Str_3
??Mid_Wu_0:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+0
        BNE.N    ??Mid_Wu_3
        BL       CH451_GetKeyValue
        LDR.W    R0,??DataTable34
        LDRB     R0,[R0, #+0]
        CMP      R0,#+1
        BEQ.N    ??Mid_Wu_2
        CMP      R0,#+2
        BEQ.N    ??Mid_Wu_4
        CMP      R0,#+5
        BEQ.N    ??Mid_Wu_5
        CMP      R0,#+6
        BEQ.N    ??Mid_Wu_6
        CMP      R0,#+9
        BEQ.N    ??Mid_Wu_7
        CMP      R0,#+10
        BEQ.N    ??Mid_Wu_8
        B.N      ??Mid_Wu_0
??Mid_Wu_9:
        LDR.W    R0,??DataTable34
        MOVS     R1,#+50
        STRB     R1,[R0, #+0]
        BL       CH451_GetKeyValue
??Mid_Wu_7:
        LDR.W    R0,??DataTable34
        LDRB     R0,[R0, #+0]
        CMP      R0,#+9
        BEQ.N    ??Mid_Wu_9
        LDR.W    R0,??DataTable29
        LDRB     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.W    R1,??DataTable29
        STRB     R0,[R1, #+0]
        LDR.W    R0,??DataTable29
        LDRB     R2,[R0, #+0]
        SXTH     R2,R2            ;; SignExt  R2,R2,#+16,#+16
        MOVS     R1,#+3
        MOVS     R0,#+92
        BL       LCD_P6x8Str_3
        B.N      ??Mid_Wu_0
??Mid_Wu_10:
        LDR.W    R0,??DataTable34
        MOVS     R1,#+50
        STRB     R1,[R0, #+0]
        BL       CH451_GetKeyValue
??Mid_Wu_5:
        LDR.W    R0,??DataTable34
        LDRB     R0,[R0, #+0]
        CMP      R0,#+5
        BEQ.N    ??Mid_Wu_10
        LDR.W    R0,??DataTable28
        LDRB     R0,[R0, #+0]
        SUBS     R0,R0,#+1
        LDR.W    R1,??DataTable28
        STRB     R0,[R1, #+0]
        LDR.W    R0,??DataTable28
        LDRB     R2,[R0, #+0]
        SXTH     R2,R2            ;; SignExt  R2,R2,#+16,#+16
        MOVS     R1,#+3
        MOVS     R0,#+72
        BL       LCD_P6x8Str_3
        B.N      ??Mid_Wu_0
??Mid_Wu_11:
        LDR.W    R0,??DataTable34
        MOVS     R1,#+50
        STRB     R1,[R0, #+0]
        BL       CH451_GetKeyValue
??Mid_Wu_8:
        LDR.W    R0,??DataTable34
        LDRB     R0,[R0, #+0]
        CMP      R0,#+10
        BEQ.N    ??Mid_Wu_11
        LDR.W    R0,??DataTable29
        LDRB     R0,[R0, #+0]
        SUBS     R0,R0,#+1
        LDR.W    R1,??DataTable29
        STRB     R0,[R1, #+0]
        LDR.W    R0,??DataTable29
        LDRB     R2,[R0, #+0]
        SXTH     R2,R2            ;; SignExt  R2,R2,#+16,#+16
        MOVS     R1,#+3
        MOVS     R0,#+92
        BL       LCD_P6x8Str_3
        B.N      ??Mid_Wu_0
??Mid_Wu_12:
        LDR.W    R0,??DataTable34
        MOVS     R1,#+50
        STRB     R1,[R0, #+0]
        BL       CH451_GetKeyValue
??Mid_Wu_4:
        LDR.W    R0,??DataTable34
        LDRB     R0,[R0, #+0]
        CMP      R0,#+2
        BEQ.N    ??Mid_Wu_12
        B.N      ??Mid_Wu_0
??Mid_Wu_13:
        LDR.W    R0,??DataTable34
        MOVS     R1,#+50
        STRB     R1,[R0, #+0]
        BL       CH451_GetKeyValue
??Mid_Wu_6:
        LDR.W    R0,??DataTable34
        LDRB     R0,[R0, #+0]
        CMP      R0,#+6
        BEQ.N    ??Mid_Wu_13
        MOVS     R4,#+1
        B.N      ??Mid_Wu_0
??Mid_Wu_3:
        POP      {R4,PC}          ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21:
        DC32     0x400ff000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_1:
        DC32     GetKeyValue

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_2:
        DC32     KeyCode

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_3:
        DC32     KeyValue

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_4:
        DC32     0x40049004

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_5:
        DC32     0x40049008

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_6:
        DC32     0x4004900c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable21_7:
        DC32     0x40049010

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// __interwork __softfp void Duo_Ji_PD(void)
Duo_Ji_PD:
        PUSH     {R3-R5,LR}
        MOVS     R5,#+0
        MOVS     R4,#+0
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        MOVS     R2,R4
        SXTH     R2,R2            ;; SignExt  R2,R2,#+16,#+16
        MOVS     R1,#+2
        MOVS     R0,#+90
        BL       LCD_P6x8Str_3
        B.N      ??Duo_Ji_PD_0
??Duo_Ji_PD_1:
        LDR.W    R0,??DataTable34
        MOVS     R1,#+50
        STRB     R1,[R0, #+0]
        BL       CH451_GetKeyValue
??Duo_Ji_PD_2:
        LDR.W    R0,??DataTable34
        LDRB     R0,[R0, #+0]
        CMP      R0,#+1
        BEQ.N    ??Duo_Ji_PD_1
        ADDS     R4,R4,#+1
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        MOVS     R2,R4
        SXTH     R2,R2            ;; SignExt  R2,R2,#+16,#+16
        MOVS     R1,#+2
        MOVS     R0,#+90
        BL       LCD_P6x8Str_3
??Duo_Ji_PD_0:
        UXTB     R5,R5            ;; ZeroExt  R5,R5,#+24,#+24
        CMP      R5,#+0
        BNE.W    ??Duo_Ji_PD_3
        BL       CH451_GetKeyValue
        LDR.W    R0,??DataTable34
        LDRB     R0,[R0, #+0]
        CMP      R0,#+1
        BEQ.N    ??Duo_Ji_PD_2
        CMP      R0,#+2
        BEQ.N    ??Duo_Ji_PD_4
        CMP      R0,#+5
        BEQ.N    ??Duo_Ji_PD_5
        CMP      R0,#+6
        BEQ.N    ??Duo_Ji_PD_6
        CMP      R0,#+9
        BEQ.N    ??Duo_Ji_PD_7
        CMP      R0,#+10
        BEQ.N    ??Duo_Ji_PD_8
        B.N      ??Duo_Ji_PD_0
??Duo_Ji_PD_6:
        ADR.N    R2,??DataTable23  ;; 0x4F, 0x4B, 0x00, 0x00
        MOVS     R1,#+2
        MOVS     R0,#+110
        BL       LCD_P6x8Str
        B.N      ??Duo_Ji_PD_9
??Duo_Ji_PD_10:
        LDR.W    R0,??DataTable34
        MOVS     R1,#+50
        STRB     R1,[R0, #+0]
        BL       CH451_GetKeyValue
??Duo_Ji_PD_7:
        LDR.W    R0,??DataTable34
        LDRB     R0,[R0, #+0]
        CMP      R0,#+9
        BEQ.N    ??Duo_Ji_PD_10
        ADDS     R4,R4,#+2
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        MOVS     R2,R4
        SXTH     R2,R2            ;; SignExt  R2,R2,#+16,#+16
        MOVS     R1,#+2
        MOVS     R0,#+90
        BL       LCD_P6x8Str_3
        B.N      ??Duo_Ji_PD_0
??Duo_Ji_PD_11:
        LDR.W    R0,??DataTable34
        MOVS     R1,#+50
        STRB     R1,[R0, #+0]
        BL       CH451_GetKeyValue
??Duo_Ji_PD_5:
        LDR.W    R0,??DataTable34
        LDRB     R0,[R0, #+0]
        CMP      R0,#+5
        BEQ.N    ??Duo_Ji_PD_11
        SUBS     R4,R4,#+1
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        MOVS     R2,R4
        SXTH     R2,R2            ;; SignExt  R2,R2,#+16,#+16
        MOVS     R1,#+2
        MOVS     R0,#+90
        BL       LCD_P6x8Str_3
        B.N      ??Duo_Ji_PD_0
??Duo_Ji_PD_12:
        LDR.W    R0,??DataTable34
        MOVS     R1,#+50
        STRB     R1,[R0, #+0]
        BL       CH451_GetKeyValue
??Duo_Ji_PD_8:
        LDR.W    R0,??DataTable34
        LDRB     R0,[R0, #+0]
        CMP      R0,#+10
        BEQ.N    ??Duo_Ji_PD_12
        SUBS     R4,R4,#+2
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        MOVS     R2,R4
        SXTH     R2,R2            ;; SignExt  R2,R2,#+16,#+16
        MOVS     R1,#+2
        MOVS     R0,#+90
        BL       LCD_P6x8Str_3
        B.N      ??Duo_Ji_PD_0
??Duo_Ji_PD_13:
        LDR.W    R0,??DataTable34
        MOVS     R1,#+50
        STRB     R1,[R0, #+0]
        BL       CH451_GetKeyValue
??Duo_Ji_PD_4:
        LDR.W    R0,??DataTable34
        LDRB     R0,[R0, #+0]
        CMP      R0,#+2
        BEQ.N    ??Duo_Ji_PD_13
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+4
        BNE.N    ??Duo_Ji_PD_14
        MOVS     R0,#+3
        BL       LCD_CLS2
        LDR.W    R2,??DataTable31
        MOVS     R1,#+3
        MOVS     R0,#+0
        BL       LCD_P6x8Str
        LDR.W    R0,??DataTable28
        LDRB     R2,[R0, #+0]
        SXTH     R2,R2            ;; SignExt  R2,R2,#+16,#+16
        MOVS     R1,#+3
        MOVS     R0,#+72
        BL       LCD_P6x8Str_3
        LDR.W    R0,??DataTable29
        LDRB     R2,[R0, #+0]
        SXTH     R2,R2            ;; SignExt  R2,R2,#+16,#+16
        MOVS     R1,#+3
        MOVS     R0,#+92
        BL       LCD_P6x8Str_3
        BL       Mid_Wu
??Duo_Ji_PD_14:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+2
        BNE.N    ??Duo_Ji_PD_15
        MOVS     R0,#+3
        BL       LCD_CLS2
        LDR.W    R2,??DataTable31_1
        MOVS     R1,#+3
        MOVS     R0,#+0
        BL       LCD_P6x8Str
        LDR.W    R0,??DataTable26
        LDRB     R2,[R0, #+0]
        SXTH     R2,R2            ;; SignExt  R2,R2,#+16,#+16
        MOVS     R1,#+3
        MOVS     R0,#+72
        BL       LCD_P6x8Str_3
        LDR.W    R0,??DataTable26_1
        LDRB     R2,[R0, #+0]
        SXTH     R2,R2            ;; SignExt  R2,R2,#+16,#+16
        MOVS     R1,#+3
        MOVS     R0,#+92
        BL       LCD_P6x8Str_3
        BL       High_Wu
??Duo_Ji_PD_15:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+3
        BNE.N    ??Duo_Ji_PD_16
        MOVS     R0,#+3
        BL       LCD_CLS2
        LDR.W    R2,??DataTable32
        MOVS     R1,#+3
        MOVS     R0,#+0
        BL       LCD_P6x8Str
        LDR.W    R0,??DataTable27_2
        LDRB     R2,[R0, #+0]
        SXTH     R2,R2            ;; SignExt  R2,R2,#+16,#+16
        MOVS     R1,#+3
        MOVS     R0,#+72
        BL       LCD_P6x8Str_3
        LDR.W    R0,??DataTable27_3
        LDRB     R2,[R0, #+0]
        SXTH     R2,R2            ;; SignExt  R2,R2,#+16,#+16
        MOVS     R1,#+3
        MOVS     R0,#+92
        BL       LCD_P6x8Str_3
        BL       High_Mid
??Duo_Ji_PD_16:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BNE.N    ??Duo_Ji_PD_17
        MOVS     R0,#+3
        BL       LCD_CLS2
        LDR.W    R2,??DataTable33
        MOVS     R1,#+3
        MOVS     R0,#+0
        BL       LCD_P6x8Str
        LDR.N    R0,??DataTable25_7
        LDRB     R2,[R0, #+0]
        SXTH     R2,R2            ;; SignExt  R2,R2,#+16,#+16
        MOVS     R1,#+3
        MOVS     R0,#+72
        BL       LCD_P6x8Str_3
        LDR.N    R0,??DataTable25_8
        LDRB     R2,[R0, #+0]
        SXTH     R2,R2            ;; SignExt  R2,R2,#+16,#+16
        MOVS     R1,#+3
        MOVS     R0,#+92
        BL       LCD_P6x8Str_3
        BL       High_You
??Duo_Ji_PD_17:
        B.N      ??Duo_Ji_PD_0
??Duo_Ji_PD_18:
        LDR.W    R0,??DataTable34
        MOVS     R1,#+50
        STRB     R1,[R0, #+0]
        BL       CH451_GetKeyValue
??Duo_Ji_PD_9:
        LDR.W    R0,??DataTable34
        LDRB     R0,[R0, #+0]
        CMP      R0,#+6
        BEQ.N    ??Duo_Ji_PD_18
        MOVS     R5,#+1
        B.N      ??Duo_Ji_PD_0
??Duo_Ji_PD_3:
        POP      {R0,R4,R5,PC}    ;; return

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// __interwork __softfp void Car_Speed(void)
Car_Speed:
        PUSH     {R4,LR}
        MOVS     R4,#+0
        B.N      ??Car_Speed_0
??Car_Speed_1:
        LDR.W    R0,??DataTable34
        MOVS     R1,#+50
        STRB     R1,[R0, #+0]
        BL       CH451_GetKeyValue
??Car_Speed_2:
        LDR.W    R0,??DataTable34
        LDRB     R0,[R0, #+0]
        CMP      R0,#+1
        BEQ.N    ??Car_Speed_1
        LDR.W    R0,??DataTable35
        LDRB     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.W    R1,??DataTable35
        STRB     R0,[R1, #+0]
        LDR.W    R0,??DataTable35
        LDRB     R2,[R0, #+0]
        SXTH     R2,R2            ;; SignExt  R2,R2,#+16,#+16
        MOVS     R1,#+0
        MOVS     R0,#+36
        BL       LCD_P6x8Str_3
??Car_Speed_0:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+0
        BNE.N    ??Car_Speed_3
        BL       CH451_GetKeyValue
        LDR.W    R0,??DataTable34
        LDRB     R0,[R0, #+0]
        CMP      R0,#+1
        BEQ.N    ??Car_Speed_2
        CMP      R0,#+2
        BEQ.N    ??Car_Speed_4
        CMP      R0,#+5
        BEQ.N    ??Car_Speed_5
        CMP      R0,#+9
        BEQ.N    ??Car_Speed_6
        CMP      R0,#+10
        BEQ.N    ??Car_Speed_7
        B.N      ??Car_Speed_0
??Car_Speed_8:
        LDR.W    R0,??DataTable34
        MOVS     R1,#+50
        STRB     R1,[R0, #+0]
        BL       CH451_GetKeyValue
??Car_Speed_5:
        LDR.W    R0,??DataTable34
        LDRB     R0,[R0, #+0]
        CMP      R0,#+5
        BEQ.N    ??Car_Speed_8
        LDR.W    R0,??DataTable35
        LDRB     R0,[R0, #+0]
        SUBS     R0,R0,#+1
        LDR.W    R1,??DataTable35
        STRB     R0,[R1, #+0]
        LDR.W    R0,??DataTable35
        LDRB     R2,[R0, #+0]
        SXTH     R2,R2            ;; SignExt  R2,R2,#+16,#+16
        MOVS     R1,#+0
        MOVS     R0,#+36
        BL       LCD_P6x8Str_3
        B.N      ??Car_Speed_0
??Car_Speed_9:
        LDR.W    R0,??DataTable34
        MOVS     R1,#+50
        STRB     R1,[R0, #+0]
        BL       CH451_GetKeyValue
??Car_Speed_6:
        LDR.W    R0,??DataTable34
        LDRB     R0,[R0, #+0]
        CMP      R0,#+9
        BEQ.N    ??Car_Speed_9
        LDR.W    R0,??DataTable36
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+10
        LDR.W    R1,??DataTable36
        STRH     R0,[R1, #+0]
        LDR.W    R0,??DataTable36
        LDRSH    R2,[R0, #+0]
        MOVS     R1,#+7
        MOVS     R0,#+50
        BL       LCD_P6x8Str_3
        B.N      ??Car_Speed_0
??Car_Speed_10:
        LDR.W    R0,??DataTable34
        MOVS     R1,#+50
        STRB     R1,[R0, #+0]
        BL       CH451_GetKeyValue
??Car_Speed_7:
        LDR.W    R0,??DataTable34
        LDRB     R0,[R0, #+0]
        CMP      R0,#+10
        BEQ.N    ??Car_Speed_10
        LDR.W    R0,??DataTable36
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+10
        LDR.W    R1,??DataTable36
        STRH     R0,[R1, #+0]
        LDR.W    R0,??DataTable36
        LDRSH    R2,[R0, #+0]
        MOVS     R1,#+7
        MOVS     R0,#+50
        BL       LCD_P6x8Str_3
        B.N      ??Car_Speed_0
??Car_Speed_11:
        LDR.W    R0,??DataTable34
        MOVS     R1,#+50
        STRB     R1,[R0, #+0]
        BL       CH451_GetKeyValue
??Car_Speed_4:
        LDR.W    R0,??DataTable34
        LDRB     R0,[R0, #+0]
        CMP      R0,#+2
        BEQ.N    ??Car_Speed_11
        ADR.N    R2,??DataTable23  ;; 0x4F, 0x4B, 0x00, 0x00
        MOVS     R1,#+0
        MOVS     R0,#+60
        BL       LCD_P6x8Str
        MOVS     R4,#+1
        B.N      ??Car_Speed_0
??Car_Speed_3:
        POP      {R4,PC}          ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23:
        DC8      0x4F, 0x4B, 0x00, 0x00

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_1:
        DC32     receive_flag

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_2:
        DC32     `data`

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_3:
        DC32     bianma

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_4:
        DC32     bitdata

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_5:
        DC32     startflag

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable23_6:
        DC32     bitnum
//  202 
//  203 /*******************point结构体*********************/
//  204 struct point_position{
//  205   volatile byte x;
//  206   volatile byte y;
//  207   volatile byte i;//
//  208   volatile byte j;//

        SECTION `.bss`:DATA:REORDER:NOROOT(2)
//  209 }point;
point:
        DS8 4
//  210 /*******************point结构体END*********************/
//  211 struct PID_proportion{
//  212   volatile int16 P;
//  213   volatile int16 I;
//  214   volatile int16 D;

        SECTION `.bss`:DATA:REORDER:NOROOT(2)
//  215 }Servo,Motor;
Servo:
        DS8 8

        SECTION `.bss`:DATA:REORDER:NOROOT(2)
Motor:
        DS8 8
//  216 
//  217 struct range
//  218 {
//  219   int16 head;
//  220   int16 tail;

        SECTION `.bss`:DATA:REORDER:NOROOT(2)
//  221 }servo;
servo:
        DS8 4
//  222 
//  223 struct membership_grade
//  224 {
//  225   int16 s;
//  226   int16 m;
//  227   int16 b;

        SECTION `.bss`:DATA:REORDER:NOROOT(2)
//  228 }sub;
`sub`:
        DS8 8
//  229 
//  230 
//  231 /***********************系统、总线频率设置**********************/

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  232 void pllinit180M(void)
//  233 {
//  234 	uint32_t temp_reg;
//  235         //使能IO端口时钟    
//  236     SIM_SCGC5 |= (SIM_SCGC5_PORTA_MASK
//  237                               | SIM_SCGC5_PORTB_MASK
//  238                               | SIM_SCGC5_PORTC_MASK
//  239                               | SIM_SCGC5_PORTD_MASK
//  240                               | SIM_SCGC5_PORTE_MASK );
pllinit180M:
        LDR.N    R0,??DataTable25_5  ;; 0x40048038
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x3E00
        LDR.N    R1,??DataTable25_5  ;; 0x40048038
        STR      R0,[R1, #+0]
//  241     //这里处在默认的FEI模式
//  242     //首先移动到FBE模式
//  243     MCG_C2 = 0;  
        LDR.N    R0,??DataTable24_6  ;; 0x40064001
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
//  244     //MCG_C2 = MCG_C2_RANGE(2) | MCG_C2_HGO_MASK | MCG_C2_EREFS_MASK;
//  245     //初始化晶振后释放锁定状态的振荡器和GPIO
//  246     SIM_SCGC4 |= SIM_SCGC4_LLWU_MASK;
        LDR.W    R0,??DataTable39  ;; 0x40048034
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x10000000
        LDR.W    R1,??DataTable39  ;; 0x40048034
        STR      R0,[R1, #+0]
//  247     LLWU_CS |= LLWU_CS_ACKISO_MASK;
        LDR.W    R0,??DataTable39_1  ;; 0x4007c008
        LDRB     R0,[R0, #+0]
        ORRS     R0,R0,#0x80
        LDR.W    R1,??DataTable39_1  ;; 0x4007c008
        STRB     R0,[R1, #+0]
//  248     
//  249     //选择外部晶振，参考分频器，清IREFS来启动外部晶振
//  250     //011 If RANGE = 0, Divide Factor is 8; for all other RANGE values, Divide Factor is 256.
//  251     MCG_C1 = MCG_C1_CLKS(2) | MCG_C1_FRDIV(3);
        LDR.N    R0,??DataTable24_5  ;; 0x40064000
        MOVS     R1,#+152
        STRB     R1,[R0, #+0]
//  252     
//  253     //等待晶振稳定	    
//  254     //while (!(MCG_S & MCG_S_OSCINIT_MASK)){}              //等待锁相环初始化结束
//  255     while (MCG_S & MCG_S_IREFST_MASK){}                  //等待时钟切换到外部参考时钟
??pllinit180M_0:
        LDR.W    R0,??DataTable44  ;; 0x40064006
        LDRB     R0,[R0, #+0]
        LSLS     R0,R0,#+27
        BMI.N    ??pllinit180M_0
//  256     while (((MCG_S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) != 0x2){}
??pllinit180M_1:
        LDR.W    R0,??DataTable44  ;; 0x40064006
        LDRB     R0,[R0, #+0]
        UBFX     R0,R0,#+2,#+2
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        CMP      R0,#+2
        BNE.N    ??pllinit180M_1
//  257     //进入FBE模式,
//  258     MCG_C5 = MCG_C5_PRDIV(0x0e);//分频在2~4MHz之间，分频后频率=晶振频率/(PRDIV+1) 3.3MHz                
        LDR.W    R0,??DataTable39_2  ;; 0x40064004
        MOVS     R1,#+14
        STRB     R1,[R0, #+0]
//  259     MCG_C6 = 0x0;//确保MCG_C6处于复位状态，禁止LOLIE、PLL、和时钟控制器，清PLL VCO分频器
        LDR.W    R0,??DataTable39_3  ;; 0x40064005
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
//  260     temp_reg = FMC_PFAPR;//保存FMC_PFAPR当前的值
        LDR.W    R0,??DataTable39_4  ;; 0x4001f000
        LDR      R0,[R0, #+0]
//  261     FMC_PFAPR |= FMC_PFAPR_M7PFD_MASK | FMC_PFAPR_M6PFD_MASK | FMC_PFAPR_M5PFD_MASK
//  262                      | FMC_PFAPR_M4PFD_MASK | FMC_PFAPR_M3PFD_MASK | FMC_PFAPR_M2PFD_MASK
//  263                      | FMC_PFAPR_M1PFD_MASK | FMC_PFAPR_M0PFD_MASK; //通过M&PFD置位M0PFD来禁止预取功能   
        LDR.W    R1,??DataTable39_4  ;; 0x4001f000
        LDR      R1,[R1, #+0]
        ORRS     R1,R1,#0xFF0000
        LDR.W    R2,??DataTable39_4  ;; 0x4001f000
        STR      R1,[R2, #+0]
//  264     ///设置系统分频器
//  265     //MCG=PLL, core = MCG, bus = MCG/3, FlexBus = MCG/3, Flash clock= MCG/8
//  266     SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV2(3) 
//  267                  | SIM_CLKDIV1_OUTDIV3(2) | SIM_CLKDIV1_OUTDIV4(7);       
        LDR.W    R1,??DataTable39_5  ;; 0x40048044
        LDR.W    R2,??DataTable39_6  ;; 0x3270000
        STR      R2,[R1, #+0]
//  268     FMC_PFAPR = temp_reg;//从新存FMC_PFAPR的原始值 
        LDR.W    R1,??DataTable39_4  ;; 0x4001f000
        STR      R0,[R1, #+0]
//  269     //设置VCO分频器，使能PLL为100MHz, LOLIE=0, PLLS=1, CME=0, VDIV=26
//  270     MCG_C6 = MCG_C6_PLLS_MASK | MCG_C6_VDIV(30);  //VDIV = 31 (x55)
        LDR.W    R0,??DataTable39_3  ;; 0x40064005
        MOVS     R1,#+94
        STRB     R1,[R0, #+0]
//  271                                                   //VDIV = 26 (x50)
//  272     while (!(MCG_S & MCG_S_PLLST_MASK)){}; // wait for PLL status bit to set    
??pllinit180M_2:
        LDR.W    R0,??DataTable44  ;; 0x40064006
        LDRB     R0,[R0, #+0]
        LSLS     R0,R0,#+26
        BPL.N    ??pllinit180M_2
//  273     while (!(MCG_S & MCG_S_LOCK_MASK)){}; // Wait for LOCK bit to set    
??pllinit180M_3:
        LDR.W    R0,??DataTable44  ;; 0x40064006
        LDRB     R0,[R0, #+0]
        LSLS     R0,R0,#+25
        BPL.N    ??pllinit180M_3
//  274     //进入PBE模式    
//  275     //通过清零CLKS位来进入PEE模式
//  276     // CLKS=0, FRDIV=3, IREFS=0, IRCLKEN=0, IREFSTEN=0
//  277     MCG_C1 &= ~MCG_C1_CLKS_MASK;
        LDR.N    R0,??DataTable24_5  ;; 0x40064000
        LDRB     R0,[R0, #+0]
        ANDS     R0,R0,#0x3F
        LDR.N    R1,??DataTable24_5  ;; 0x40064000
        STRB     R0,[R1, #+0]
//  278     //等待时钟状态位更新
//  279     while (((MCG_S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) != 0x3){};
??pllinit180M_4:
        LDR.W    R0,??DataTable44  ;; 0x40064006
        LDRB     R0,[R0, #+0]
        UBFX     R0,R0,#+2,#+2
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        CMP      R0,#+3
        BNE.N    ??pllinit180M_4
//  280 }
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable24:
        DC32     deal_flag

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable24_1:
        DC32     irkey

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable24_2:
        DC32     LPTMR_INTERRUPT

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable24_3:
        DC32     0x40040004

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable24_4:
        DC32     0x40040000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable24_5:
        DC32     0x40064000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable24_6:
        DC32     0x40064001
//  281 /***********************系统、总线频率设置END**********************/
//  282 
//  283 /************************舵机控制输出PWM模块***********************/

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  284 void hw_FTM1_init(void)
//  285     {      	
//  286       //SIM_SOPT4|=SIM_SOPT4_FTM1FLT0_MASK;        
//  287       /* Turn on all port clocks */
//  288       SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
hw_FTM1_init:
        LDR.N    R0,??DataTable25_5  ;; 0x40048038
        LDR      R0,[R0, #+0]
        MOV      R1,#+512
        ORRS     R0,R1,R0
        LDR.N    R1,??DataTable25_5  ;; 0x40048038
        STR      R0,[R1, #+0]
//  289         
//  290       /* Enable the function on PTA8 */
//  291       PORTA_PCR8 = PORT_PCR_MUX(0x3)| PORT_PCR_DSE_MASK;; // FTM is alt3 function for this pin         
        LDR.W    R0,??DataTable41  ;; 0x40049020
        MOV      R1,#+832
        STR      R1,[R0, #+0]
//  292     
//  293       SIM_SCGC6|=SIM_SCGC6_FTM1_MASK;         //使能FTM1时钟      
        LDR.N    R0,??DataTable25_2  ;; 0x4004803c
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x2000000
        LDR.N    R1,??DataTable25_2  ;; 0x4004803c
        STR      R0,[R1, #+0]
//  294       
//  295       //change MSnB = 1  
//  296       FTM1_C0SC |= FTM_CnSC_ELSB_MASK;
        LDR.W    R0,??DataTable41_1  ;; 0x4003900c
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x8
        LDR.W    R1,??DataTable41_1  ;; 0x4003900c
        STR      R0,[R1, #+0]
//  297       FTM1_C0SC &= ~FTM_CnSC_ELSA_MASK;
        LDR.W    R0,??DataTable41_1  ;; 0x4003900c
        LDR      R0,[R0, #+0]
        BICS     R0,R0,#0x4
        LDR.W    R1,??DataTable41_1  ;; 0x4003900c
        STR      R0,[R1, #+0]
//  298       FTM1_C0SC |= FTM_CnSC_MSB_MASK;     
        LDR.W    R0,??DataTable41_1  ;; 0x4003900c
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x20
        LDR.W    R1,??DataTable41_1  ;; 0x4003900c
        STR      R0,[R1, #+0]
//  299       
//  300       //FTM1_SC = FTM_SC_PS(0) | FTM_SC_CLKS(1);
//  301       //FTM1_SC=0X0F;     
//  302       FTM1_SC = 0xb; //not enable the interrupt mask向上计数模式
        LDR.W    R0,??DataTable41_2  ;; 0x40039000
        MOVS     R1,#+11
        STR      R1,[R0, #+0]
//  303       //FTM1_SC=0X1F;       //BIT5  0 FTM counter operates in up counting mode.
//  304                             //1 FTM counter operates in up-down counting mode.      
//  305       //BIT43 FTM1_SC|=FTM1_SC_CLKS_MASK;
//  306                             //00 No clock selected (This in effect disables the FTM counter.)
//  307                             //01 System clock
//  308                             //10 Fixed frequency clock
//  309                             //11 External clock
//  310       //BIT210 FTM1_SC|=FTM1_SC_PS_MASK; 
//  311                             //100M          MOD=2000;     MOD=4000;   MOD=1000; 
//  312                             //000 Divide by 1---12KHZ     6K          24k
//  313                             //001 Divide by 2--- 6KHZ
//  314                             //010 Divide by 4--- 3K
//  315                             //011 Divide by 8--- 1.5K
//  316                             //100 Divide by 16---750
//  317                             //101 Divide by 32---375
//  318                             //110 Divide by 64---187.5HZ
//  319                             //111 Divide by 128--93.75hz             
//  320       
//  321       FTM1_MODE |= FTM_MODE_WPDIS_MASK;      
        LDR.W    R0,??DataTable41_3  ;; 0x40039054
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x4
        LDR.W    R1,??DataTable41_3  ;; 0x40039054
        STR      R0,[R1, #+0]
//  322        //BIT1   Initialize the Channels Output
//  323       //FTMEN is bit 0, need to set to zero so DECAPEN can be set to 0
//  324       FTM1_MODE &= ~1;
        LDR.W    R0,??DataTable41_3  ;; 0x40039054
        LDR      R0,[R0, #+0]
        LSRS     R0,R0,#+1
        LSLS     R0,R0,#+1
        LDR.W    R1,??DataTable41_3  ;; 0x40039054
        STR      R0,[R1, #+0]
//  325        //BIT0   FTM Enable
//  326        //0 Only the TPM-compatible registers (first set of registers) can be used without any restriction. Do not use the FTM-specific registers.
//  327        //1 All registers including the FTM-specific registers (second set of registers) are available for use with no restrictions.
//  328       
//  329       FTM1_OUTMASK=0xFE;   //0 Channel output is not masked. It continues to operate normally.
        LDR.W    R0,??DataTable41_4  ;; 0x40039060
        MOVS     R1,#+254
        STR      R1,[R0, #+0]
//  330                            //1 Channel output is masked. It is forced to its inactive state.
//  331       
//  332       FTM1_COMBINE=0;      //Function for Linked Channels (FTMx_COMBINE)
        LDR.W    R0,??DataTable41_5  ;; 0x40039064
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  333       FTM1_OUTINIT=0;
        LDR.W    R0,??DataTable41_6  ;; 0x4003905c
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  334       FTM1_EXTTRIG=0;      //FTM External Trigger (FTMx_EXTTRIG)
        LDR.W    R0,??DataTable41_7  ;; 0x4003906c
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  335       FTM1_POL=0;          //Channels Polarity (FTMx_POL)
        LDR.W    R0,??DataTable41_8  ;; 0x40039070
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  336                            //0 The channel polarity is active high.
//  337                            //1 The channel polarity is active low.     
//  338       //Set Edge Aligned PWM
//  339       FTM1_QDCTRL &=~FTM_QDCTRL_QUADEN_MASK;
        LDR.W    R0,??DataTable41_9  ;; 0x40039080
        LDR      R0,[R0, #+0]
        LSRS     R0,R0,#+1
        LSLS     R0,R0,#+1
        LDR.W    R1,??DataTable41_9  ;; 0x40039080
        STR      R0,[R1, #+0]
//  340       //QUADEN is Bit 1, Set Quadrature Decoder Mode (QUADEN) Enable to 0,   (disabled)
//  341       //FTM0_SC = 0x16; //Center Aligned PWM Select = 0, sets FTM Counter to operate in up counting mode,
//  342       //it is field 5 of FTMx_SC (status control) - also setting the pre-scale bits here
//  343       
//  344       FTM1_INVCTRL=0;     //反转控制
        LDR.W    R0,??DataTable41_10  ;; 0x40039090
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  345       FTM1_SWOCTRL=0;     //软件输出控制F TM Software Output Control (FTMx_SWOCTRL)
        LDR.W    R0,??DataTable41_11  ;; 0x40039094
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  346       FTM1_PWMLOAD=0;     //FTM PWM Load
        LDR.W    R0,??DataTable41_12  ;; 0x40039098
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  347                           //BIT9: 0 Loading updated values is disabled.
//  348                           //1 Loading updated values is enabled.
//  349       FTM1_CNTIN=0;        //Counter Initial Value      
        LDR.W    R0,??DataTable41_13  ;; 0x4003904c
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  350       FTM1_MOD=62500;//56250       //Modulo value,The EPWM period is determined by (MOD - CNTIN + 0x0001) 
        LDR.W    R0,??DataTable41_14  ;; 0x40039008
        MOVW     R1,#+62500
        STR      R1,[R0, #+0]
//  351                            //采用龙丘时钟初始化函数，可以得到4分频的频率，系统60M频率时，PWM频率是15M,以此类推
//  352                            //PMW频率=X系统频率/4/(2^FTM1_SC_PS)/FTM1_MOD
//  353       FTM1_C0V=Servo_Middle;        //设置 the pulse width(duty cycle) is determined by (CnV - CNTIN).
        LDR.W    R0,??DataTable41_15  ;; 0x40039010
        LDR.W    R1,??DataTable41_16
        LDRSH    R1,[R1, #+0]
        STR      R1,[R0, #+0]
//  354       FTM1_CNT=0;          //只有低16位可用
        LDR.W    R0,??DataTable45  ;; 0x40039004
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  355 }
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable25:
        DC32     timer

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable25_1:
        DC32     0x40040008

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable25_2:
        DC32     0x4004803c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable25_3:
        DC32     0x4003d010

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable25_4:
        DC32     0x40047000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable25_5:
        DC32     0x40048038

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable25_6:
        DC32     0x4004904c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable25_7:
        DC32     P_High

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable25_8:
        DC32     D_High
//  356 /************************舵机控制输出PWM模块END***********************/
//  357 
//  358 /*********************电机正反转  C1口C3口输出PWM波*************/

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  359 void hw_FTM0_init(void)
//  360     {      	
//  361       
//  362       SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK;
hw_FTM0_init:
        LDR.W    R0,??DataTable41_17  ;; 0x40048038
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x1800
        LDR.W    R1,??DataTable41_17  ;; 0x40048038
        STR      R0,[R1, #+0]
//  363         
//  364       
//  365       PORTC_PCR1 = PORT_PCR_MUX(0x4)| PORT_PCR_DSE_MASK; // FTM is alt4 function for this pin         
        LDR.W    R0,??DataTable41_18  ;; 0x4004b004
        MOV      R1,#+1088
        STR      R1,[R0, #+0]
//  366       PORTC_PCR4 = PORT_PCR_MUX(0x4)| PORT_PCR_DSE_MASK;
        LDR.W    R0,??DataTable41_19  ;; 0x4004b010
        MOV      R1,#+1088
        STR      R1,[R0, #+0]
//  367       PORTC_PCR2 = PORT_PCR_MUX(0x4)| PORT_PCR_DSE_MASK;
        LDR.W    R0,??DataTable41_20  ;; 0x4004b008
        MOV      R1,#+1088
        STR      R1,[R0, #+0]
//  368       
//  369       PORTD_PCR4 = PORT_PCR_MUX(0x4)| PORT_PCR_DSE_MASK; // FTM is alt4 function for this pin         
        LDR.W    R0,??DataTable41_21  ;; 0x4004c010
        MOV      R1,#+1088
        STR      R1,[R0, #+0]
//  370       PORTD_PCR5 = PORT_PCR_MUX(0x4)| PORT_PCR_DSE_MASK;
        LDR.W    R0,??DataTable41_22  ;; 0x4004c014
        MOV      R1,#+1088
        STR      R1,[R0, #+0]
//  371       PORTD_PCR6 = PORT_PCR_MUX(0x4)| PORT_PCR_DSE_MASK;
        LDR.W    R0,??DataTable41_23  ;; 0x4004c018
        MOV      R1,#+1088
        STR      R1,[R0, #+0]
//  372       
//  373       SIM_SCGC6|=SIM_SCGC6_FTM0_MASK;         //使能FTM0时钟      
        LDR.W    R0,??DataTable41_24  ;; 0x4004803c
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x1000000
        LDR.W    R1,??DataTable41_24  ;; 0x4004803c
        STR      R0,[R1, #+0]
//  374       
//  375       
//  376       //电机1
//  377       //C0口 
//  378       FTM0_C0SC |= FTM_CnSC_ELSB_MASK;//ELSB=1，ELSA=0，左对齐，先高后低
        LDR.W    R0,??DataTable41_25  ;; 0x4003800c
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x8
        LDR.W    R1,??DataTable41_25  ;; 0x4003800c
        STR      R0,[R1, #+0]
//  379       FTM0_C0SC &= ~FTM_CnSC_ELSA_MASK;
        LDR.W    R0,??DataTable41_25  ;; 0x4003800c
        LDR      R0,[R0, #+0]
        BICS     R0,R0,#0x4
        LDR.W    R1,??DataTable41_25  ;; 0x4003800c
        STR      R0,[R1, #+0]
//  380       FTM0_C0SC |= FTM_CnSC_MSB_MASK;    //MSB=1，模式选择边沿对齐
        LDR.W    R0,??DataTable41_25  ;; 0x4003800c
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x20
        LDR.W    R1,??DataTable41_25  ;; 0x4003800c
        STR      R0,[R1, #+0]
//  381       //C3口
//  382       FTM0_C3SC |= FTM_CnSC_ELSB_MASK;
        LDR.W    R0,??DataTable42  ;; 0x40038024
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x8
        LDR.W    R1,??DataTable42  ;; 0x40038024
        STR      R0,[R1, #+0]
//  383       FTM0_C3SC &= ~FTM_CnSC_ELSA_MASK;
        LDR.W    R0,??DataTable42  ;; 0x40038024
        LDR      R0,[R0, #+0]
        BICS     R0,R0,#0x4
        LDR.W    R1,??DataTable42  ;; 0x40038024
        STR      R0,[R1, #+0]
//  384       FTM0_C3SC |= FTM_CnSC_MSB_MASK;
        LDR.W    R0,??DataTable42  ;; 0x40038024
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x20
        LDR.W    R1,??DataTable42  ;; 0x40038024
        STR      R0,[R1, #+0]
//  385       //C1口 电机控制长期高电压
//  386       FTM0_C1SC |= FTM_CnSC_ELSB_MASK;
        LDR.W    R0,??DataTable42_1  ;; 0x40038014
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x8
        LDR.W    R1,??DataTable42_1  ;; 0x40038014
        STR      R0,[R1, #+0]
//  387       FTM0_C1SC &= ~FTM_CnSC_ELSA_MASK;
        LDR.W    R0,??DataTable42_1  ;; 0x40038014
        LDR      R0,[R0, #+0]
        BICS     R0,R0,#0x4
        LDR.W    R1,??DataTable42_1  ;; 0x40038014
        STR      R0,[R1, #+0]
//  388       FTM0_C1SC |= FTM_CnSC_MSB_MASK;    
        LDR.W    R0,??DataTable42_1  ;; 0x40038014
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x20
        LDR.W    R1,??DataTable42_1  ;; 0x40038014
        STR      R0,[R1, #+0]
//  389       
//  390       //电机2
//  391        FTM0_C4SC |= FTM_CnSC_ELSB_MASK;//ELSB=1，ELSA=0，左对齐，先高后低
        LDR.W    R0,??DataTable43  ;; 0x4003802c
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x8
        LDR.W    R1,??DataTable43  ;; 0x4003802c
        STR      R0,[R1, #+0]
//  392       FTM0_C4SC &= ~FTM_CnSC_ELSA_MASK;
        LDR.W    R0,??DataTable43  ;; 0x4003802c
        LDR      R0,[R0, #+0]
        BICS     R0,R0,#0x4
        LDR.W    R1,??DataTable43  ;; 0x4003802c
        STR      R0,[R1, #+0]
//  393       FTM0_C4SC |= FTM_CnSC_MSB_MASK;    //MSB=1，模式选择边沿对齐
        LDR.W    R0,??DataTable43  ;; 0x4003802c
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x20
        LDR.W    R1,??DataTable43  ;; 0x4003802c
        STR      R0,[R1, #+0]
//  394       //C3口
//  395       FTM0_C5SC |= FTM_CnSC_ELSB_MASK;
        LDR.W    R0,??DataTable43_1  ;; 0x40038034
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x8
        LDR.W    R1,??DataTable43_1  ;; 0x40038034
        STR      R0,[R1, #+0]
//  396       FTM0_C5SC &= ~FTM_CnSC_ELSA_MASK;
        LDR.W    R0,??DataTable43_1  ;; 0x40038034
        LDR      R0,[R0, #+0]
        BICS     R0,R0,#0x4
        LDR.W    R1,??DataTable43_1  ;; 0x40038034
        STR      R0,[R1, #+0]
//  397       FTM0_C5SC |= FTM_CnSC_MSB_MASK;
        LDR.W    R0,??DataTable43_1  ;; 0x40038034
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x20
        LDR.W    R1,??DataTable43_1  ;; 0x40038034
        STR      R0,[R1, #+0]
//  398       //C1口 电机控制长期高电压
//  399       FTM0_C6SC |= FTM_CnSC_ELSB_MASK;
        LDR.W    R0,??DataTable44_1  ;; 0x4003803c
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x8
        LDR.W    R1,??DataTable44_1  ;; 0x4003803c
        STR      R0,[R1, #+0]
//  400       FTM0_C6SC &= ~FTM_CnSC_ELSA_MASK;
        LDR.W    R0,??DataTable44_1  ;; 0x4003803c
        LDR      R0,[R0, #+0]
        BICS     R0,R0,#0x4
        LDR.W    R1,??DataTable44_1  ;; 0x4003803c
        STR      R0,[R1, #+0]
//  401       FTM0_C6SC |= FTM_CnSC_MSB_MASK;    
        LDR.W    R0,??DataTable44_1  ;; 0x4003803c
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x20
        LDR.W    R1,??DataTable44_1  ;; 0x4003803c
        STR      R0,[R1, #+0]
//  402       FTM0_SC = 0xC; //not enable the interrupt mask向上计数模式
        LDR.W    R0,??DataTable44_2  ;; 0x40038000
        MOVS     R1,#+12
        STR      R1,[R0, #+0]
//  403                      //中断禁止，时钟源来自总线时钟60MHz，16分频后得到3.75MHz，CLKS=01，PS=100,CPWMS=0，递增计数
//  404       FTM0_MODE |= FTM_MODE_WPDIS_MASK;  //写保护禁止    
        LDR.W    R0,??DataTable44_3  ;; 0x40038054
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x4
        LDR.W    R1,??DataTable44_3  ;; 0x40038054
        STR      R0,[R1, #+0]
//  405        //BIT1   Initialize the Channels Output
//  406       //FTMEN is bit 0, need to set to zero so DECAPEN can be set to 0
//  407       FTM0_MODE &= ~1;  //FTM0使能
        LDR.W    R0,??DataTable44_3  ;; 0x40038054
        LDR      R0,[R0, #+0]
        LSRS     R0,R0,#+1
        LSLS     R0,R0,#+1
        LDR.W    R1,??DataTable44_3  ;; 0x40038054
        STR      R0,[R1, #+0]
//  408        //BIT0   FTM Enable
//  409        //0 Only the TPM-compatible registers (first set of registers) can be used without any restriction. Do not use the FTM-specific registers.
//  410        //1 All registers including the FTM-specific registers (second set of registers) are available for use with no restrictions.
//  411       
//  412       FTM0_OUTMASK=0x84;   //0 Channel output is not masked. It continues to operate normally.
        LDR.W    R0,??DataTable44_4  ;; 0x40038060
        MOVS     R1,#+132
        STR      R1,[R0, #+0]
//  413                            //1 Channel output is masked. It is forced to its inactive state.
//  414                            //通道3 1 0输出，其他通道屏蔽   
//  415       FTM0_COMBINE=0;      //Function for Linked Channels (FTMx_COMBINE)//DECAPEN=0，双边沿捕捉禁止，COMBINE=0，不级联
        LDR.W    R0,??DataTable44_5  ;; 0x40038064
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  416       FTM0_OUTINIT=0;
        LDR.W    R0,??DataTable44_6  ;; 0x4003805c
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  417       FTM0_EXTTRIG=0;      //FTM External Trigger (FTMx_EXTTRIG)
        LDR.W    R0,??DataTable44_7  ;; 0x4003806c
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  418       FTM0_POL=0;          //Channels Polarity (FTMx_POL)
        LDR.W    R0,??DataTable44_8  ;; 0x40038070
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  419                            //0 The channel polarity is active high.
//  420                            //1 The channel polarity is active low.     
//  421       //Set Edge Aligned PWM
//  422       FTM0_QDCTRL &=~FTM_QDCTRL_QUADEN_MASK;//禁止正交解码模式
        LDR.W    R0,??DataTable44_9  ;; 0x40038080
        LDR      R0,[R0, #+0]
        LSRS     R0,R0,#+1
        LSLS     R0,R0,#+1
        LDR.W    R1,??DataTable44_9  ;; 0x40038080
        STR      R0,[R1, #+0]
//  423       //QUADEN is Bit 1, Set Quadrature Decoder Mode (QUADEN) Enable to 0,   (disabled)
//  424       //FTM0_SC = 0x16; //Center Aligned PWM Select = 0, sets FTM Counter to operate in up counting mode,
//  425       //it is field 5 of FTMx_SC (status control) - also setting the pre-scale bits here
//  426       
//  427       FTM0_INVCTRL=0;     //反转控制
        LDR.W    R0,??DataTable44_10  ;; 0x40038090
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  428       FTM0_SWOCTRL=0;     //软件输出控制F TM Software Output Control (FTMx_SWOCTRL)
        LDR.W    R0,??DataTable44_11  ;; 0x40038094
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  429       FTM0_PWMLOAD=0;     //FTM PWM Load
        LDR.W    R0,??DataTable44_12  ;; 0x40038098
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  430                           //BIT9: 0 Loading updated values is disabled.
//  431                           //1 Loading updated values is enabled.
//  432       FTM0_CNTIN=0;        //Counter Initial Value  初始技术值为0    
        LDR.W    R0,??DataTable44_13  ;; 0x4003804c
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  433       FTM0_MOD=375;       //375为10K 
        LDR.W    R0,??DataTable44_14  ;; 0x40038008
        MOVW     R1,#+375
        STR      R1,[R0, #+0]
//  434                            //采用龙丘时钟初始化函数，可以得到4分频的频率，系统60M频率时，PWM频率是15M,以此类推
//  435                            //PMW频率=X系统频率/4/(2^FTM1_SC_PS)/FTM1_MOD
//  436       FTM0_C0V=0;//20;        //反转
        LDR.W    R0,??DataTable44_15  ;; 0x40038010
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  437       FTM0_C1V=375;        //持续高电平
        LDR.W    R0,??DataTable45_1  ;; 0x40038018
        MOVW     R1,#+375
        STR      R1,[R0, #+0]
//  438       FTM0_C3V=0;//80;           //正转
        LDR.W    R0,??DataTable45_2  ;; 0x40038028
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  439       
//  440       FTM0_C4V=0;//100;        //正转
        LDR.W    R0,??DataTable45_3  ;; 0x40038030
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  441       FTM0_C5V=375;        //持续高电平
        LDR.W    R0,??DataTable45_4  ;; 0x40038038
        MOVW     R1,#+375
        STR      R1,[R0, #+0]
//  442       FTM0_C6V=0;//20;           //反转
        LDR.W    R0,??DataTable45_5  ;; 0x40038040
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  443       
//  444       FTM0_CNT=0;          //只有低16位可用
        LDR.W    R0,??DataTable45_6  ;; 0x40038004
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  445 }
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable26:
        DC32     P_Mid

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable26_1:
        DC32     D_Mid
//  446 /*********************电机正反转  C1口C3口输出PWM波END*************/
//  447 
//  448 /****************FTM正交分解，用于电机1测脉冲数的   A10口*************/

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  449 void FTM2_QUAD_Init(void)  
//  450 {  
//  451   PORTA_PCR10    =  PORT_PCR_MUX(6);             // 设置引脚A10引脚为FTM2_PHA功能  
FTM2_QUAD_Init:
        LDR.W    R0,??DataTable45_7  ;; 0x40049028
        MOV      R1,#+1536
        STR      R1,[R0, #+0]
//  452   PORTA_PCR11    =  PORT_PCR_MUX(6);             // 设置引脚A11引脚为FTM2_PHB功能  
        LDR.W    R0,??DataTable45_8  ;; 0x4004902c
        MOV      R1,#+1536
        STR      R1,[R0, #+0]
//  453   PORT_PCR_REG(PORTA_BASE_PTR, 10) |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK ; //开弱上拉
        LDR.W    R0,??DataTable45_7  ;; 0x40049028
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x3
        LDR.W    R1,??DataTable45_7  ;; 0x40049028
        STR      R0,[R1, #+0]
//  454   PORT_PCR_REG(PORTA_BASE_PTR, 11) |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK ; //开弱上拉
        LDR.W    R0,??DataTable45_8  ;; 0x4004902c
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x3
        LDR.W    R1,??DataTable45_8  ;; 0x4004902c
        STR      R0,[R1, #+0]
//  455   SIM_SCGC3     |=  SIM_SCGC3_FTM2_MASK;                // 使能FTM2时钟  
        LDR.W    R0,??DataTable45_9  ;; 0x40048030
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x1000000
        LDR.W    R1,??DataTable45_9  ;; 0x40048030
        STR      R0,[R1, #+0]
//  456   FTM2_MODE     |=  FTM_MODE_WPDIS_MASK;                // 写保护禁止  
        LDR.W    R0,??DataTable45_10  ;; 0x400b8054
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x4
        LDR.W    R1,??DataTable45_10  ;; 0x400b8054
        STR      R0,[R1, #+0]
//  457   FTM2_QDCTRL   |=  FTM_QDCTRL_QUADMODE_MASK;          // AB相同时确定方向和计数值  
        LDR.W    R0,??DataTable45_11  ;; 0x400b8080
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x8
        LDR.W    R1,??DataTable45_11  ;; 0x400b8080
        STR      R0,[R1, #+0]
//  458   FTM2_CNTIN     =  0;                                 // FTM0计数器初始值为0  
        LDR.W    R0,??DataTable45_12  ;; 0x400b804c
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  459   FTM2_MOD       =  65535;                                 // 结束值  
        LDR.W    R0,??DataTable45_13  ;; 0x400b8008
        MOVW     R1,#+65535
        STR      R1,[R0, #+0]
//  460   FTM2_QDCTRL   |=  FTM_QDCTRL_QUADEN_MASK;             // 启用FTM2正交解码模式  
        LDR.W    R0,??DataTable45_11  ;; 0x400b8080
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x1
        LDR.W    R1,??DataTable45_11  ;; 0x400b8080
        STR      R0,[R1, #+0]
//  461   FTM2_MODE     |=  FTM_MODE_FTMEN_MASK;                // FTM2EN=1    
        LDR.W    R0,??DataTable45_10  ;; 0x400b8054
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x1
        LDR.W    R1,??DataTable45_10  ;; 0x400b8054
        STR      R0,[R1, #+0]
//  462   FTM2_CNT       =  0;  
        LDR.W    R0,??DataTable45_14  ;; 0x400b8004
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  463 }  
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable27:
        DC32     0x4004b014

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable27_1:
        DC32     0x4004000c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable27_2:
        DC32     P_Mid1

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable27_3:
        DC32     D_Mid1
//  464 /****************FTM正交分解，用于测脉冲数的   A10口END*************/
//  465 //电机2 DMA测脉冲数B18
//  466 
//  467 /**************************IO口初始化***********************/

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  468 void IO_Init()
//  469 {
//  470 	/* 打开各个端口的时钟源 */
//  471 	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | 
//  472 	SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;
IO_Init:
        LDR.W    R0,??DataTable41_17  ;; 0x40048038
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x3E00
        LDR.W    R1,??DataTable41_17  ;; 0x40048038
        STR      R0,[R1, #+0]
//  473 	PORTA_PCR14=PORT_PCR_MUX(1);//A14引脚设置为GPIO模式
        LDR.W    R0,??DataTable45_15  ;; 0x40049038
        MOV      R1,#+256
        STR      R1,[R0, #+0]
//  474 	PORTA_PCR15=PORT_PCR_MUX(1);//A15引脚设置为GPIO模式
        LDR.W    R0,??DataTable45_16  ;; 0x4004903c
        MOV      R1,#+256
        STR      R1,[R0, #+0]
//  475 	PORTA_PCR16=PORT_PCR_MUX(1);//A16引脚设置为GPIO模式
        LDR.W    R0,??DataTable45_17  ;; 0x40049040
        MOV      R1,#+256
        STR      R1,[R0, #+0]
//  476 	PORTA_PCR17=PORT_PCR_MUX(1);//A17引脚设置为GPIO模式
        LDR.W    R0,??DataTable45_18  ;; 0x40049044
        MOV      R1,#+256
        STR      R1,[R0, #+0]
//  477 	
//  478 	//B0~B7设为GPIO输入模式，连接ov7260的8位灰度输入
//  479 	PORTB_PCR0=PORT_PCR_MUX(1);//B0引脚设置为GPIO模式
        LDR.W    R0,??DataTable45_19  ;; 0x4004a000
        MOV      R1,#+256
        STR      R1,[R0, #+0]
//  480 	PORTB_PCR1=PORT_PCR_MUX(1);//B1引脚设置为GPIO模式
        LDR.W    R0,??DataTable45_20  ;; 0x4004a004
        MOV      R1,#+256
        STR      R1,[R0, #+0]
//  481 	PORTB_PCR2=PORT_PCR_MUX(1);//B2引脚设置为GPIO模式
        LDR.W    R0,??DataTable45_21  ;; 0x4004a008
        MOV      R1,#+256
        STR      R1,[R0, #+0]
//  482 	PORTB_PCR3=PORT_PCR_MUX(1);//B3引脚设置为GPIO模式
        LDR.W    R0,??DataTable45_22  ;; 0x4004a00c
        MOV      R1,#+256
        STR      R1,[R0, #+0]
//  483 	PORTB_PCR4=PORT_PCR_MUX(1);//B4引脚设置为GPIO模式
        LDR.W    R0,??DataTable45_23  ;; 0x4004a010
        MOV      R1,#+256
        STR      R1,[R0, #+0]
//  484 	PORTB_PCR5=PORT_PCR_MUX(1);//B5引脚设置为GPIO模式
        LDR.W    R0,??DataTable45_24  ;; 0x4004a014
        MOV      R1,#+256
        STR      R1,[R0, #+0]
//  485 	PORTB_PCR6=PORT_PCR_MUX(1);//B6引脚设置为GPIO模式
        LDR.W    R0,??DataTable45_25  ;; 0x4004a018
        MOV      R1,#+256
        STR      R1,[R0, #+0]
//  486 	PORTB_PCR7=PORT_PCR_MUX(1);//B7引脚设置为GPIO模式
        LDR.W    R0,??DataTable45_26  ;; 0x4004a01c
        MOV      R1,#+256
        STR      R1,[R0, #+0]
//  487 	GPIOB_PDDR&=0XFFFFFF00;//B0~B7设置为输入，数字摄像头8位灰度输入
        LDR.W    R0,??DataTable45_27  ;; 0x400ff054
        LDR      R0,[R0, #+0]
        LSRS     R0,R0,#+8
        LSLS     R0,R0,#+8
        LDR.W    R1,??DataTable45_27  ;; 0x400ff054
        STR      R0,[R1, #+0]
//  488 	
//  489 	PORTA_PCR24=PORT_PCR_MUX(1)|PORT_PCR_IRQC(10);//A24引脚设置为GPIO模式，下降沿中断,场中断
        LDR.W    R0,??DataTable45_28  ;; 0x40049060
        LDR.W    R1,??DataTable45_29  ;; 0xa0100
        STR      R1,[R0, #+0]
//  490 	
//  491 	PORTB_PCR10=PORT_PCR_MUX(1)|PORT_PCR_IRQC(9);//B10引脚设置为GPIO模式，上升沿中断,行中断
        LDR.W    R0,??DataTable45_30  ;; 0x4004a028
        LDR.W    R1,??DataTable45_31  ;; 0x90100
        STR      R1,[R0, #+0]
//  492 	
//  493 	PORTB_PCR11=PORT_PCR_MUX(1)|PORT_PCR_IRQC(1);//B11引脚设置为GPIO模式，上升沿触发DMA请求
        LDR.W    R0,??DataTable45_32  ;; 0x4004a02c
        LDR.W    R1,??DataTable45_33  ;; 0x10100
        STR      R1,[R0, #+0]
//  494 	
//  495         PORTC_PCR9=PORT_PCR_MUX(1)|PORT_PCR_IRQC(10);//C9引脚设置为GPIO模式，下降沿中断,红外遥控停车中断
        LDR.W    R0,??DataTable45_34  ;; 0x4004b024
        LDR.W    R1,??DataTable45_29  ;; 0xa0100
        STR      R1,[R0, #+0]
//  496         
//  497         
//  498         
//  499 	GPIOA_PDDR=GPIO_PDDR_PDD(GPIO_PIN(14)|GPIO_PIN(16)|GPIO_PIN(17)|GPIO_PIN(26)|GPIO_PIN(28)); //A14为输出模式  P4灯闪烁证明进入PIT中断给脉冲计时
        LDR.W    R0,??DataTable45_35  ;; 0x400ff014
        LDR.W    R1,??DataTable45_36  ;; 0x14034000
        STR      R1,[R0, #+0]
//  500         
//  501         PORTA_PCR28=PORT_PCR_MUX(1);//A17引脚设置为GPIO模式
        LDR.W    R0,??DataTable45_37  ;; 0x40049070
        MOV      R1,#+256
        STR      R1,[R0, #+0]
//  502         PORTA_PCR26=PORT_PCR_MUX(1);//A17引脚设置为GPIO模式
        LDR.W    R0,??DataTable45_38  ;; 0x40049068
        MOV      R1,#+256
        STR      R1,[R0, #+0]
//  503 	//PORTC_PCR8=PORT_PCR_MUX(1);//C8引脚设置为GPIO模式，液晶命令/数据选择引脚
//  504 	//PORTC_PCR10=PORT_PCR_MUX(1);//C10引脚设置为GPIO模式，液晶复位引脚
//  505 	//GPIOC_PDDR|=0X00000500;//C8,C10设置为输出
//  506 	//GPIOA_PDDR|=0X0003E000;//A14~A17设置为输出
//  507 	//GPIOA_PCOR|=0X0003E000;//初始低电平输出
//  508 	//PORTD_PCR2 = PORT_PCR_MUX(1)|PORT_PCR_IRQC(0x9);//IRQ|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK;
//  509 	//PORTD_PCR2 = PORT_PCR_MUX(1)|PORT_PCR_IRQC(0x1);//DMA|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK;
//  510 	//D2口GPIO功能，上升沿中断，PE启用拉电阻，PS上拉电阻
//  511 }
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable28:
        DC32     P_Low
//  512 /**************************IO口初始化END***********************/
//  513 
//  514 
//  515 /******************与上位机相连，用于看图像的*****************/

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  516 void SCI(void)
//  517 {
SCI:
        PUSH     {R3-R5,LR}
//  518   volatile uint8 *uc_FramePoint;
//  519   uint8 a=0;
        MOVS     R4,#+0
//  520   uint16 b=0;   
        MOVS     R5,#+0
//  521   
//  522   uart_send1(UART0,0x01);
        MOVS     R1,#+1
        LDR.W    R0,??DataTable45_39  ;; 0x4006a000
        BL       uart_send1
//  523   
//  524   for(a=0;a<IMG_ROWS;a++)
        MOVS     R0,#+0
        MOVS     R4,R0
        B.N      ??SCI_0
//  525   {
//  526     for(b=0;b<IMG_COLS;b++)
//  527     {
//  528       uc_FramePoint=ImageData[a]+b;
??SCI_1:
        UXTH     R5,R5            ;; ZeroExt  R5,R5,#+16,#+16
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        MOVS     R0,#+160
        LDR.W    R1,??DataTable45_40
        MLA      R0,R0,R4,R1
        ADDS     R0,R5,R0
//  529       uart_send1(UART0,*uc_FramePoint);
        LDRB     R1,[R0, #+0]
        LDR.W    R0,??DataTable45_39  ;; 0x4006a000
        BL       uart_send1
//  530     }
        ADDS     R5,R5,#+1
??SCI_2:
        UXTH     R5,R5            ;; ZeroExt  R5,R5,#+16,#+16
        CMP      R5,#+160
        BCC.N    ??SCI_1
        ADDS     R4,R4,#+1
??SCI_0:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+55
        BCS.N    ??SCI_3
        MOVS     R5,#+0
        B.N      ??SCI_2
//  531   }
//  532  
//  533   uart_send1(UART0,0x01);
??SCI_3:
        MOVS     R1,#+1
        LDR.W    R0,??DataTable45_39  ;; 0x4006a000
        BL       uart_send1
//  534   
//  535   PORTA_PCR24|=PORT_PCR_ISF_MASK;//清除中断标志
        LDR.W    R0,??DataTable45_28  ;; 0x40049060
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x1000000
        LDR.W    R1,??DataTable45_28  ;; 0x40049060
        STR      R0,[R1, #+0]
//  536 }
        POP      {R0,R4,R5,PC}    ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable29:
        DC32     D_Low
//  537 /******************与上位机相连，用于看图像的END*****************/
//  538 
//  539 
//  540 
//  541 /********************串口模块波特率的设置*********************/
//  542 //UART0初始化，使用PTD6为UART0_RX,PTD7为UART0_TX   D6接RXD  D7接TXD
//  543 //波特率：115200

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  544 void UART0_Init(void)
//  545 {
UART0_Init:
        PUSH     {R4}
//  546 	unsigned long uartclk_khz=180000;//时钟180MHz
        LDR.W    R0,??DataTable45_41  ;; 0x2bf20
//  547 	unsigned long baud=115200;//波特率256000
        MOVS     R1,#+115200
//  548 	unsigned short sbr,brfa;
//  549 	PORTB_PCR16|=PORT_PCR_MUX(3);//将D6引脚设置为模式3，即UART0_RX
        LDR.W    R2,??DataTable45_42  ;; 0x4004a040
        LDR      R2,[R2, #+0]
        MOV      R3,#+768
        ORRS     R2,R3,R2
        LDR.W    R3,??DataTable45_42  ;; 0x4004a040
        STR      R2,[R3, #+0]
//  550 	PORTB_PCR17|=PORT_PCR_MUX(3);//将D7引脚设置为模式3，即UART0_TX
        LDR.W    R2,??DataTable45_43  ;; 0x4004a044
        LDR      R2,[R2, #+0]
        MOV      R3,#+768
        ORRS     R2,R3,R2
        LDR.W    R3,??DataTable45_43  ;; 0x4004a044
        STR      R2,[R3, #+0]
//  551 	SIM_SCGC4|=SIM_SCGC4_UART0_MASK;//开启UART0时钟
        LDR.N    R2,??DataTable39  ;; 0x40048034
        LDR      R2,[R2, #+0]
        ORRS     R2,R2,#0x400
        LDR.N    R3,??DataTable39  ;; 0x40048034
        STR      R2,[R3, #+0]
//  552 	sbr=(unsigned short)((uartclk_khz*1000)/(baud*16));//计算并设置波特率
        MOV      R2,#+1000
        MUL      R2,R2,R0
        LSLS     R3,R1,#+4
        UDIV     R2,R2,R3
//  553 	
//  554 	UART0_BDH=(unsigned char)((sbr&0x1F00)>>8);
        UXTH     R2,R2            ;; ZeroExt  R2,R2,#+16,#+16
        ASRS     R3,R2,#+8
        ANDS     R3,R3,#0x1F
        LDR.W    R4,??DataTable45_39  ;; 0x4006a000
        STRB     R3,[R4, #+0]
//  555 	UART0_BDL=(unsigned char)(sbr&0x00FF);
        LDR.W    R3,??DataTable45_44  ;; 0x4006a001
        STRB     R2,[R3, #+0]
//  556 	brfa = (((uartclk_khz*32000)/(baud*16))-(sbr*32));
        MOV      R3,#+32000
        MULS     R0,R3,R0
        LSLS     R1,R1,#+4
        UDIV     R0,R0,R1
        UXTH     R2,R2            ;; ZeroExt  R2,R2,#+16,#+16
        SUBS     R0,R0,R2, LSL #+5
//  557 	UART0_C4 = (unsigned char)(brfa & 0x001F);
        ANDS     R0,R0,#0x1F
        LDR.W    R1,??DataTable45_45  ;; 0x4006a00a
        STRB     R0,[R1, #+0]
//  558 	UART0_C2|=(UART_C2_TE_MASK|UART_C2_RE_MASK);
        LDR.W    R0,??DataTable45_46  ;; 0x4006a003
        LDRB     R0,[R0, #+0]
        ORRS     R0,R0,#0xC
        LDR.W    R1,??DataTable45_46  ;; 0x4006a003
        STRB     R0,[R1, #+0]
//  559 	UART0_C1=0;	
        LDR.W    R0,??DataTable45_47  ;; 0x4006a002
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
//  560 }
        POP      {R4}
        BX       LR               ;; return
//  561 /********************串口模块波特率的设置END*********************/
//  562 
//  563 
//  564 
//  565 /****************************DMA初始化*****************************/
//  566 //DMA初始化 使用PLCK经过4分频后与B11相连用于DMA外部中断

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  567 void DMA0_Init(void)
//  568 {
//  569 	SIM_SCGC6|=SIM_SCGC6_DMAMUX_MASK;//打开DMA多路复用器时钟
DMA0_Init:
        LDR.W    R0,??DataTable41_24  ;; 0x4004803c
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x2
        LDR.W    R1,??DataTable41_24  ;; 0x4004803c
        STR      R0,[R1, #+0]
//  570 	SIM_SCGC7|=SIM_SCGC7_DMA_MASK;//打开DMA模块时钟
        LDR.W    R0,??DataTable45_48  ;; 0x40048040
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x2
        LDR.W    R1,??DataTable45_48  ;; 0x40048040
        STR      R0,[R1, #+0]
//  571 	DMAMUX_CHCFG0=DMAMUX_CHCFG_SOURCE(50);//DMA通道0对应50号DMA请求，即PORTB	
        LDR.W    R0,??DataTable45_49  ;; 0x40021000
        MOVS     R1,#+50
        STRB     R1,[R0, #+0]
//  572 	
//  573 	DMA_TCD0_CITER_ELINKNO=DMA_CITER_ELINKNO_CITER(IMG_COLS);//当前主循环次数,采集点数
        LDR.W    R0,??DataTable45_50  ;; 0x40009016
        MOVS     R1,#+160
        STRH     R1,[R0, #+0]
//  574 	DMA_TCD0_BITER_ELINKNO=DMA_BITER_ELINKNO_BITER(IMG_COLS);//起始主循环次数，采集点数
        LDR.W    R0,??DataTable45_51  ;; 0x4000901e
        MOVS     R1,#+160
        STRH     R1,[R0, #+0]
//  575 	DMA_TCD0_SADDR=(uint32)&GPIOB_PDIR;//设置源地址GPIO口，PORTB
        LDR.W    R0,??DataTable45_52  ;; 0x40009000
        LDR.W    R1,??DataTable45_53  ;; 0x400ff050
        STR      R1,[R0, #+0]
//  576 	DMA_TCD0_SOFF=0;//每次传送源地址不变
        LDR.W    R0,??DataTable45_54  ;; 0x40009004
        MOVS     R1,#+0
        STRH     R1,[R0, #+0]
//  577 	//DMA_TCD1_NBYTES_MLOFFYES=DMA_NBYTES_MLOFFYES_NBYTES(1)+DMA_NBYTES_MLOFFNO_SMLOE_MASK+DMA_NBYTES_MLOFFYES_MLOFF(-4);//传送4字节
//  578 	DMA_TCD0_NBYTES_MLNO=DMA_NBYTES_MLNO_NBYTES(1);//每次读取一字节
        LDR.W    R0,??DataTable45_55  ;; 0x40009008
        MOVS     R1,#+1
        STR      R1,[R0, #+0]
//  579 	DMA_TCD0_SLAST=0;//主循环结束后源地址0回写tcd
        LDR.W    R0,??DataTable45_56  ;; 0x4000900c
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  580 	DMA_TCD0_DLASTSGA=0;//主循环结束后目的地址0回写tcd
        LDR.W    R0,??DataTable45_57  ;; 0x40009018
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  581 	DMA_TCD0_DADDR=(uint32)ImageData;//设置目的地址，video数组第一个元素
        LDR.W    R0,??DataTable45_58  ;; 0x40009010
        LDR.W    R1,??DataTable45_40
        STR      R1,[R0, #+0]
//  582 	DMA_TCD0_DOFF=1;//每次写目的地址加1
        LDR.W    R0,??DataTable45_59  ;; 0x40009014
        MOVS     R1,#+1
        STRH     R1,[R0, #+0]
//  583 	DMA_TCD0_ATTR=DMA_ATTR_SSIZE(0)+DMA_ATTR_DSIZE(0);//源数据宽度8bit，目的数据宽度8bit
        LDR.W    R0,??DataTable45_60  ;; 0x40009006
        MOVS     R1,#+0
        STRH     R1,[R0, #+0]
//  584 	DMA_TCD0_CSR=DMA_CSR_DREQ_MASK;//DMA通道0主循环结束后停止硬件请求
        LDR.W    R0,??DataTable45_61  ;; 0x4000901c
        MOVS     R1,#+8
        STRH     R1,[R0, #+0]
//  585 	DMA_TCD0_CSR|=DMA_CSR_INTMAJOR_MASK;//使能DMA0中断
        LDR.W    R0,??DataTable45_61  ;; 0x4000901c
        LDRH     R0,[R0, #+0]
        ORRS     R0,R0,#0x2
        LDR.W    R1,??DataTable45_61  ;; 0x4000901c
        STRH     R0,[R1, #+0]
//  586 	DMAMUX_CHCFG0|=DMAMUX_CHCFG_ENBL_MASK;//DMA通道0使能
        LDR.W    R0,??DataTable45_49  ;; 0x40021000
        LDRB     R0,[R0, #+0]
        ORRS     R0,R0,#0x80
        LDR.W    R1,??DataTable45_49  ;; 0x40021000
        STRB     R0,[R1, #+0]
//  587 	
//  588 	//DMA_TCD1_CSR|=DMA_CSR_INTMAJOR_MASK;//使能DMA中断
//  589 	//DMA_TCD0_CSR|=DMA_CSR_startline_MASK;
//  590 }
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable31:
        DC32     `?<Constant "Mid_Wu">`

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable31_1:
        DC32     `?<Constant "High_Wu">`
//  591 /****************************DMA初始化END*****************************/
//  592 
//  593 
//  594 
//  595 //==========================================================================
//  596 //函数名称: hw_pit_init                                                         
//  597 //函数返回: 无                                          
//  598 //参数说明: pitno:表示pit通道号。  
//  599 //			timeout:表示定时溢出开始递减的值          
//  600 //功能概要: 设置相关寄存器的值        
//  601 //==========================================================================

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  602     void hw_pit_init(uint8 pitno,uint32 timeout)
//  603     {
//  604       SIM_SCGC6|=SIM_SCGC6_PIT_MASK;              //使能PIT时钟
hw_pit_init:
        LDR.N    R2,??DataTable41_24  ;; 0x4004803c
        LDR      R2,[R2, #+0]
        ORRS     R2,R2,#0x800000
        LDR.N    R3,??DataTable41_24  ;; 0x4004803c
        STR      R2,[R3, #+0]
//  605       PIT_MCR&=~(PIT_MCR_MDIS_MASK);              //调试模式下禁止
        LDR.W    R2,??DataTable45_62  ;; 0x40037000
        LDR      R2,[R2, #+0]
        BICS     R2,R2,#0x2
        LDR.W    R3,??DataTable45_62  ;; 0x40037000
        STR      R2,[R3, #+0]
//  606       PIT_MCR|=PIT_MCR_FRZ_MASK;                  //使能PIT模块时钟
        LDR.W    R2,??DataTable45_62  ;; 0x40037000
        LDR      R2,[R2, #+0]
        ORRS     R2,R2,#0x1
        LDR.W    R3,??DataTable45_62  ;; 0x40037000
        STR      R2,[R3, #+0]
//  607       PIT_LDVAL(pitno)=timeout;                   //设置周期
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        LSLS     R2,R0,#+4
        LDR.W    R3,??DataTable45_63  ;; 0x40037100
        STR      R1,[R2, R3]
//  608       PIT_TCTRL(pitno)|=PIT_TCTRL_TEN_MASK;       //使能pit模块运行
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        LDR.W    R1,??DataTable45_63  ;; 0x40037100
        ADDS     R1,R1,R0, LSL #+4
        LDR      R1,[R1, #+8]
        ORRS     R1,R1,#0x1
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        LDR.W    R2,??DataTable45_63  ;; 0x40037100
        ADDS     R2,R2,R0, LSL #+4
        STR      R1,[R2, #+8]
//  609       PIT_TCTRL(pitno)&=~(PIT_TCTRL_TIE_MASK);    //关pit中断
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        LDR.W    R1,??DataTable45_63  ;; 0x40037100
        ADDS     R1,R1,R0, LSL #+4
        LDR      R1,[R1, #+8]
        BICS     R1,R1,#0x2
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        LDR.W    R2,??DataTable45_63  ;; 0x40037100
        ADDS     R0,R2,R0, LSL #+4
        STR      R1,[R0, #+8]
//  610     }
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable32:
        DC32     `?<Constant "High_Mid">`
//  611    
//  612 
//  613 /*void binaryzation(){
//  614  uint8 i=0,j=0;
//  615   for(i=0;i<IMG_ROWS;i++)
//  616   {
//  617     if(i<8)
//  618     {
//  619       for(j=0;j<IMG_COLS;j++)
//  620       {
//  621         if(j<50)
//  622         {
//  623           if(ImageData[i][j]<black_d1)ImageData[i][j]=0;
//  624           else ImageData[i][j]=255;
//  625         }
//  626         else if(j<110)
//  627         {
//  628           if(ImageData[i][j]<black_d2)ImageData[i][j]=0;
//  629           else ImageData[i][j]=255;
//  630         }
//  631         else
//  632         {
//  633           if(ImageData[i][j]<black_d3)ImageData[i][j]=0;
//  634           else ImageData[i][j]=255;
//  635         }
//  636       }
//  637     }
//  638     else if(i<20)
//  639     {
//  640       for(j=0;j<IMG_COLS;j++)
//  641       {
//  642         if(j<50)
//  643         {
//  644           if(ImageData[i][j]<black_c1)ImageData[i][j]=0;
//  645           else ImageData[i][j]=255;
//  646         }
//  647         else if(j<110)
//  648         {
//  649           if(ImageData[i][j]<black_c2)ImageData[i][j]=0;
//  650           else ImageData[i][j]=255;
//  651         }
//  652         else
//  653         {
//  654           if(ImageData[i][j]<black_c3)ImageData[i][j]=0;
//  655           else ImageData[i][j]=255;
//  656         }
//  657       }
//  658     }
//  659     else if(i<34)
//  660     {
//  661       for(j=0;j<IMG_COLS;j++)
//  662       {
//  663         if(j<50)
//  664         {
//  665           if(ImageData[i][j]<black_b1)ImageData[i][j]=0;
//  666           else ImageData[i][j]=255;
//  667         }
//  668         else if(j<110)
//  669         {
//  670           if(ImageData[i][j]<black_b2)ImageData[i][j]=0;
//  671           else ImageData[i][j]=255;
//  672         }
//  673         else
//  674         {
//  675           if(ImageData[i][j]<black_b3)ImageData[i][j]=0;
//  676           else ImageData[i][j]=255;
//  677         }
//  678       }
//  679     }
//  680     else
//  681     {
//  682       for(j=0;j<IMG_COLS;j++)
//  683       {
//  684         if(j<50)
//  685         {
//  686           if(ImageData[i][j]<black_a1)ImageData[i][j]=0;
//  687           else ImageData[i][j]=255;
//  688         }
//  689         else if(j<110)
//  690         {
//  691           if(ImageData[i][j]<black_a2)ImageData[i][j]=0;
//  692           else ImageData[i][j]=255;
//  693         }
//  694         else
//  695         {
//  696           if(ImageData[i][j]<black_a3)ImageData[i][j]=0;
//  697           else ImageData[i][j]=255;
//  698         }
//  699       }
//  700     }
//  701   }
//  702 }*/
//  703 //==========================================================================
//  704 //函数名: enable_pit_interrupt                                                     
//  705 //函数返回: 无                                              
//  706 //参数说明: pitno: 表示pit通道号      
//  707 //功能概要: 开接收引脚的IRQ中断                                                                                                     
//  708 //==========================================================================

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  709     void enable_pit_interrupt(uint8 pitno)
//  710     {
enable_pit_interrupt:
        PUSH     {R7,LR}
//  711       PIT_TCTRL(pitno)|=(PIT_TCTRL_TIE_MASK); //开pit中断
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        LDR.W    R1,??DataTable45_63  ;; 0x40037100
        ADDS     R1,R1,R0, LSL #+4
        LDR      R1,[R1, #+8]
        ORRS     R1,R1,#0x2
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        LDR.W    R2,??DataTable45_63  ;; 0x40037100
        ADDS     R2,R2,R0, LSL #+4
        STR      R1,[R2, #+8]
//  712       switch(pitno)
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        CMP      R0,#+0
        BEQ.N    ??enable_pit_interrupt_0
        CMP      R0,#+2
        BEQ.N    ??enable_pit_interrupt_1
        BCC.N    ??enable_pit_interrupt_2
        CMP      R0,#+3
        BEQ.N    ??enable_pit_interrupt_3
        B.N      ??enable_pit_interrupt_4
//  713       {
//  714       case 0:
//  715         enable_irq(68);			      //开接收引脚的IRQ中断
??enable_pit_interrupt_0:
        MOVS     R0,#+68
        BL       enable_irq
//  716         break;
        B.N      ??enable_pit_interrupt_4
//  717       case 1:
//  718         enable_irq(69);			      //开接收引脚的IRQ中断
??enable_pit_interrupt_2:
        MOVS     R0,#+69
        BL       enable_irq
//  719         break;
        B.N      ??enable_pit_interrupt_4
//  720       case 2:
//  721         enable_irq(70);			      //开接收引脚的IRQ中断
??enable_pit_interrupt_1:
        MOVS     R0,#+70
        BL       enable_irq
//  722         break;
        B.N      ??enable_pit_interrupt_4
//  723       case 3:
//  724         enable_irq(71);			      //开接收引脚的IRQ中断
??enable_pit_interrupt_3:
        MOVS     R0,#+71
        BL       enable_irq
//  725         break;
//  726       }
//  727     }
??enable_pit_interrupt_4:
        POP      {R0,PC}          ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable33:
        DC32     `?<Constant "High_You">`
//  728 
//  729 
//  730 
//  731 //==========================================================================
//  732 //函数名: disable_pit_interrupt                                                     
//  733 //函数返回: 无                                              
//  734 //参数说明: pitno: 表示pit通道号      
//  735 //功能概要: 开接收引脚的IRQ中断                                                                                                     
//  736 //==========================================================================

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  737     void disable_pit_interrupt(uint8 pitno)
//  738     {
disable_pit_interrupt:
        PUSH     {R7,LR}
//  739       PIT_TCTRL(pitno)&=~(PIT_TCTRL_TIE_MASK);//关pit中断
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        LDR.W    R1,??DataTable45_63  ;; 0x40037100
        ADDS     R1,R1,R0, LSL #+4
        LDR      R1,[R1, #+8]
        BICS     R1,R1,#0x2
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        LDR.W    R2,??DataTable45_63  ;; 0x40037100
        ADDS     R2,R2,R0, LSL #+4
        STR      R1,[R2, #+8]
//  740       switch(pitno)
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        CMP      R0,#+0
        BEQ.N    ??disable_pit_interrupt_0
        CMP      R0,#+2
        BEQ.N    ??disable_pit_interrupt_1
        BCC.N    ??disable_pit_interrupt_2
        CMP      R0,#+3
        BEQ.N    ??disable_pit_interrupt_3
        B.N      ??disable_pit_interrupt_4
//  741       {
//  742       case 0:
//  743         disable_irq(68);	              //关接收引脚的IRQ中断
??disable_pit_interrupt_0:
        MOVS     R0,#+68
        BL       disable_irq
//  744         break;
        B.N      ??disable_pit_interrupt_4
//  745       case 1:
//  746         disable_irq(69);		      //关接收引脚的IRQ中断
??disable_pit_interrupt_2:
        MOVS     R0,#+69
        BL       disable_irq
//  747         break;
        B.N      ??disable_pit_interrupt_4
//  748       case 2:
//  749         disable_irq(70);		      //关接收引脚的IRQ中断
??disable_pit_interrupt_1:
        MOVS     R0,#+70
        BL       disable_irq
//  750         break;
        B.N      ??disable_pit_interrupt_4
//  751       case 3:
//  752         disable_irq(71);		      //关接收引脚的IRQ中断
??disable_pit_interrupt_3:
        MOVS     R0,#+71
        BL       disable_irq
//  753         break;
//  754       }
//  755     }
??disable_pit_interrupt_4:
        POP      {R0,PC}          ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable34:
        DC32     KeyValue
//  756 
//  757 
//  758 
//  759 //*************************中断区*****************************//
//  760 //DMA外部中断   B11口

        SECTION `.text`:CODE:NOROOT(2)
        THUMB
//  761 void DMA_CHO_ISR(void)
//  762 {
//  763         DisableInterrupts;
DMA_CHO_ISR:
        CPSID i         
//  764         //DMA_INT&=~(DMA_INT_INT0_MASK);
//  765 	DMA_INT|=DMA_INT_INT0_MASK;//清除通道0中断
        LDR.W    R0,??DataTable46  ;; 0x40008024
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x1
        LDR.W    R1,??DataTable46  ;; 0x40008024
        STR      R0,[R1, #+0]
//  766 	//putstr("DMA complete！");
//  767 	//GPIOA_PTOR|=0X0003E000;//A14~A17设置为输出
//  768 	row_F[imagerow]=1;//采集完成标志
        LDR.W    R0,??DataTable46_1
        LDR      R0,[R0, #+0]
        LDR.W    R1,??DataTable46_2
        MOVS     R2,#+1
        STRB     R2,[R0, R1]
//  769 	imagerow++;	
        LDR.W    R0,??DataTable46_1
        LDR      R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.W    R1,??DataTable46_1
        STR      R0,[R1, #+0]
//  770         EnableInterrupts;   
        CPSIE i         
//  771 }
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable35:
        DC32     car_speed
//  772 
//  773 

        SECTION `.text`:CODE:NOROOT(2)
        THUMB
//  774 void porta_isr(void)//场中断，A24，下降沿中断
//  775 { 
porta_isr:
        PUSH     {R7,LR}
//  776   DisableInterrupts;
        CPSID i         
//  777   //PORTA_ISFR = 0xFFFFFFFF;  //Clear Port A ISR flags
//  778   PORTA_PCR24|=PORT_PCR_ISF_MASK;//清除中断标志
        LDR.W    R0,??DataTable45_28  ;; 0x40049060
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x1000000
        LDR.W    R1,??DataTable45_28  ;; 0x40049060
        STR      R0,[R1, #+0]
//  779   DMA0_Init();
        BL       DMA0_Init
//  780   enable_irq(0);//使能DMA通道0完成中断
        MOVS     R0,#+0
        BL       enable_irq
//  781   row=0;//初始化行
        LDR.W    R0,??DataTable46_3
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  782   imagerow=0;//初始化采集行
        LDR.W    R0,??DataTable46_1
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  783   enable_irq (88);//使能B口中断 ，B10行中断
        MOVS     R0,#+88
        BL       enable_irq
//  784   EnableInterrupts;
        CPSIE i         
//  785   
//  786   
//  787 }
        POP      {R0,PC}          ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable36:
        DC32     Judge_startline
//  788 
//  789 

        SECTION `.text`:CODE:NOROOT(2)
        THUMB
//  790 void portb_isr(void)//行中断，B10，上升沿中断
//  791 {
portb_isr:
        PUSH     {R7,LR}
//  792   DisableInterrupts;
        CPSID i         
//  793   
//  794   PORTB_PCR10|=PORT_PCR_ISF_MASK;//清除中断标志位
        LDR.W    R0,??DataTable45_30  ;; 0x4004a028
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x1000000
        LDR.W    R1,??DataTable45_30  ;; 0x4004a028
        STR      R0,[R1, #+0]
//  795   row++; //行计数
        LDR.W    R0,??DataTable46_3
        LDR      R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.W    R1,??DataTable46_3
        STR      R0,[R1, #+0]
//  796   
//  797   //DMA_ERQ=0x1;//使能通道0硬件DMA请求    查看整场图像的时候用的
//  798   if(row==data_table[imagerow])//如果当前行数据应该采集
        LDR.W    R0,??DataTable46_3
        LDR      R0,[R0, #+0]
        LDR.W    R1,??DataTable46_1
        LDR      R1,[R1, #+0]
        LDR.W    R2,??DataTable46_4
        LDR      R1,[R2, R1, LSL #+2]
        CMP      R0,R1
        BNE.N    ??portb_isr_0
//  799   {
//  800     
//  801     DMA_ERQ|=DMA_ERQ_ERQ0_MASK;//使能通道0硬件DMA请求  
        LDR.W    R0,??DataTable46_5  ;; 0x4000800c
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x1
        LDR.W    R1,??DataTable46_5  ;; 0x4000800c
        STR      R0,[R1, #+0]
        B.N      ??portb_isr_1
//  802   } 
//  803  
//  804    else if(row>=endlineROW) //一场完成，关闭行中断
??portb_isr_0:
        LDR.W    R0,??DataTable46_3
        LDR      R0,[R0, #+0]
        CMP      R0,#+169
        BCC.N    ??portb_isr_1
//  805   {  
//  806     DMA_ERQ=0x00;
        LDR.W    R0,??DataTable46_5  ;; 0x4000800c
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  807     disable_irq(0);
        MOVS     R0,#+0
        BL       disable_irq
//  808     disable_irq(88);
        MOVS     R0,#+88
        BL       disable_irq
//  809     //binaryzation();
//  810   // SCI();
//  811     finish=1; 
        LDR.W    R0,??DataTable46_6
        MOVS     R1,#+1
        STRB     R1,[R0, #+0]
//  812     GPIOA_PDOR=GPIOA_PDOR^(GPIO_PDOR_PDO(GPIO_PIN(28)));//对引脚某位进行取反操作
        LDR.W    R0,??DataTable46_7  ;; 0x400ff000
        LDR      R0,[R0, #+0]
        EORS     R0,R0,#0x10000000
        LDR.W    R1,??DataTable46_7  ;; 0x400ff000
        STR      R0,[R1, #+0]
//  813   }
//  814   EnableInterrupts;
??portb_isr_1:
        CPSIE i         
//  815   
//  816 }
        POP      {R0,PC}          ;; return
//  817 

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  818 void pit1_isr(void)
//  819 {
pit1_isr:
        PUSH     {R7,LR}
//  820   PIT_TFLG(1)|=PIT_TFLG_TIF_MASK;
        LDR.W    R0,??DataTable46_8  ;; 0x4003711c
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x1
        LDR.W    R1,??DataTable46_8  ;; 0x4003711c
        STR      R0,[R1, #+0]
//  821   enable_pit_interrupt(1);
        MOVS     R0,#+1
        BL       enable_pit_interrupt
//  822   timer++;
        LDR.W    R0,??DataTable46_9
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.W    R1,??DataTable46_9
        STRH     R0,[R1, #+0]
//  823 }
        POP      {R0,PC}          ;; return
//  824   
//  825 

        SECTION `.text`:CODE:NOROOT(2)
        THUMB
//  826 void portc_isr(void)
//  827 {
portc_isr:
        PUSH     {R7,LR}
//  828   DisableInterrupts;
        CPSID i         
//  829   
//  830   PORTC_PCR9|=PORT_PCR_ISF_MASK;//清除中断标志位
        LDR.W    R0,??DataTable45_34  ;; 0x4004b024
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x1000000
        LDR.W    R1,??DataTable45_34  ;; 0x4004b024
        STR      R0,[R1, #+0]
//  831   //light_change(Light_Run_PORT,Light_Run3);
//  832   ir_deal();
        BL       ir_deal
//  833   EnableInterrupts;
        CPSIE i         
//  834 }
        POP      {R0,PC}          ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable39:
        DC32     0x40048034

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable39_1:
        DC32     0x4007c008

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable39_2:
        DC32     0x40064004

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable39_3:
        DC32     0x40064005

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable39_4:
        DC32     0x4001f000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable39_5:
        DC32     0x40048044

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable39_6:
        DC32     0x3270000
//  835 
//  836 

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  837 void pit0_isr(void)
//  838 {  
pit0_isr:
        PUSH     {R7,LR}
//  839   pulse=FTM2_CNT;
        LDR.W    R0,??DataTable46_10
        LDR.W    R1,??DataTable45_14  ;; 0x400b8004
        LDR      R1,[R1, #+0]
        STRH     R1,[R0, #+0]
//  840   FTM2_CNT=0;
        LDR.W    R0,??DataTable45_14  ;; 0x400b8004
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  841   pulse+=20;
        LDR.W    R0,??DataTable46_10
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+20
        LDR.W    R1,??DataTable46_10
        STRH     R0,[R1, #+0]
//  842   pulse2=get_counter_value();
        BL       get_counter_value
        LDR.W    R1,??DataTable46_11
        STRH     R0,[R1, #+0]
//  843   lptmr_counter_clean();
        LDR.W    R0,??DataTable46_12  ;; 0x40040000
        LDR      R0,[R0, #+0]
        LSRS     R0,R0,#+1
        LSLS     R0,R0,#+1
        LDR.W    R1,??DataTable46_12  ;; 0x40040000
        STR      R0,[R1, #+0]
        LDR.W    R0,??DataTable46_12  ;; 0x40040000
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x1
        LDR.W    R1,??DataTable46_12  ;; 0x40040000
        STR      R0,[R1, #+0]
//  844   pulse=(pulse+pulse2)/2;
        LDR.W    R0,??DataTable46_10
        LDRH     R0,[R0, #+0]
        LDR.W    R1,??DataTable46_11
        LDRH     R1,[R1, #+0]
        UXTAH    R0,R1,R0
        MOVS     R1,#+2
        SDIV     R0,R0,R1
        LDR.W    R1,??DataTable46_10
        STRH     R0,[R1, #+0]
//  845   PIT_TFLG(0)|=PIT_TFLG_TIF_MASK;       //清标志
        LDR.W    R0,??DataTable46_13  ;; 0x4003710c
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x1
        LDR.W    R1,??DataTable46_13  ;; 0x4003710c
        STR      R0,[R1, #+0]
//  846   enable_pit_interrupt(0);
        MOVS     R0,#+0
        BL       enable_pit_interrupt
//  847 }
        POP      {R0,PC}          ;; return
//  848 

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  849 void pit2_isr(void)
//  850 {
pit2_isr:
        PUSH     {R7,LR}
//  851   P_timer++;
        LDR.W    R0,??DataTable46_14
        LDR      R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.W    R1,??DataTable46_14
        STR      R0,[R1, #+0]
//  852   PIT_TFLG(2)|=PIT_TFLG_TIF_MASK;
        LDR.W    R0,??DataTable46_15  ;; 0x4003712c
        LDR      R0,[R0, #+0]
        ORRS     R0,R0,#0x1
        LDR.W    R1,??DataTable46_15  ;; 0x4003712c
        STR      R0,[R1, #+0]
//  853   enable_pit_interrupt(2);
        MOVS     R0,#+2
        BL       enable_pit_interrupt
//  854 }
        POP      {R0,PC}          ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable41:
        DC32     0x40049020

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable41_1:
        DC32     0x4003900c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable41_2:
        DC32     0x40039000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable41_3:
        DC32     0x40039054

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable41_4:
        DC32     0x40039060

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable41_5:
        DC32     0x40039064

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable41_6:
        DC32     0x4003905c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable41_7:
        DC32     0x4003906c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable41_8:
        DC32     0x40039070

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable41_9:
        DC32     0x40039080

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable41_10:
        DC32     0x40039090

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable41_11:
        DC32     0x40039094

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable41_12:
        DC32     0x40039098

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable41_13:
        DC32     0x4003904c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable41_14:
        DC32     0x40039008

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable41_15:
        DC32     0x40039010

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable41_16:
        DC32     Servo_Middle

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable41_17:
        DC32     0x40048038

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable41_18:
        DC32     0x4004b004

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable41_19:
        DC32     0x4004b010

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable41_20:
        DC32     0x4004b008

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable41_21:
        DC32     0x4004c010

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable41_22:
        DC32     0x4004c014

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable41_23:
        DC32     0x4004c018

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable41_24:
        DC32     0x4004803c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable41_25:
        DC32     0x4003800c
//  855 
//  856 /******************************************************************/
//  857 //***********************中断区结束*********************************
//  858 
//  859 
//  860 
//  861 /*****************延时函数1s*****************/

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  862 void delays(int s)
//  863 {
delays:
        PUSH     {R4}
//  864   int m=0,i=0,j=0;
        MOVS     R1,#+0
        MOVS     R2,#+0
        MOVS     R3,#+0
//  865   for(m=0;m<s;m++)
        MOVS     R4,#+0
        MOVS     R1,R4
        B.N      ??delays_0
??delays_1:
        ADDS     R1,R1,#+1
??delays_0:
        CMP      R1,R0
        BGE.N    ??delays_2
//  866   {
//  867     for(i=0;i<20000;i++)
        MOVS     R2,#+0
        B.N      ??delays_3
//  868     {
//  869       for(j=0;j<1000;j++);
??delays_4:
        ADDS     R3,R3,#+1
??delays_5:
        MOV      R4,#+1000
        CMP      R3,R4
        BLT.N    ??delays_4
        ADDS     R2,R2,#+1
??delays_3:
        MOVW     R3,#+20000
        CMP      R2,R3
        BGE.N    ??delays_1
        MOVS     R3,#+0
        B.N      ??delays_5
//  870     }
//  871   }
//  872 }
??delays_2:
        POP      {R4}
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  873 void delayms(int z)
//  874 {
//  875   int x,y;
//  876   for(x=z;x>0;x--)
delayms:
        B.N      ??delayms_0
//  877     for(y=110;y>0;y--);
??delayms_1:
        SUBS     R1,R1,#+1
??delayms_2:
        CMP      R1,#+1
        BGE.N    ??delayms_1
        SUBS     R0,R0,#+1
??delayms_0:
        CMP      R0,#+1
        BLT.N    ??delayms_3
        MOVS     R1,#+110
        B.N      ??delayms_2
//  878 }
??delayms_3:
        BX       LR               ;; return
//  879 

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  880 int16 abs(int16 a)
//  881 {
//  882   if(a>=0)return a;
abs:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        CMP      R0,#+0
        BMI.N    ??abs_0
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        B.N      ??abs_1
//  883   
//  884   else return -a;
??abs_0:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        RSBS     R0,R0,#+0
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
??abs_1:
        BX       LR               ;; return
//  885   
//  886 }
//  887 
//  888 
//  889 /******************************************************************/

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  890 int16 limit(int16 value,int16 top,int16 bottom)
//  891 {
//  892   if(value>top) value=top;
limit:
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        CMP      R1,R0
        BGE.N    ??limit_0
        MOVS     R0,R1
//  893   if(value<bottom) value=bottom;
??limit_0:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        SXTH     R2,R2            ;; SignExt  R2,R2,#+16,#+16
        CMP      R0,R2
        BGE.N    ??limit_1
        MOVS     R0,R2
//  894   return value;
??limit_1:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BX       LR               ;; return
//  895 }
//  896 

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  897 int16 max(int16 value1,int16 value2)
//  898 {
//  899   if(value1>value2)
max:
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        CMP      R1,R0
        BGE.N    ??max_0
//  900     return value1;
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        B.N      ??max_1
//  901   else 
//  902     return value2;
??max_0:
        MOVS     R0,R1
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
??max_1:
        BX       LR               ;; return
//  903 }
//  904 

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  905 int16 min(int16 value1,int16 value2)
//  906 {
//  907   if(value1<value2)
min:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        CMP      R0,R1
        BGE.N    ??min_0
//  908     return value1;
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        B.N      ??min_1
//  909   else 
//  910     return value2;
??min_0:
        MOVS     R0,R1
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
??min_1:
        BX       LR               ;; return
//  911 }
//  912 
//  913 
//  914 //void set_speed(int16 speed_low)
//  915 //{
//  916 //
//  917 //  FTM0_C0V=limit(188+speed_low/2,375,0);
//  918 //  FTM0_C3V=limit(188-speed_low/2,375,0);
//  919 //}
//  920 

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  921 int set_speed(int speed_low)
//  922 { 
set_speed:
        PUSH     {R3-R5,LR}
        MOVS     R4,R0
//  923   //int back_1,back_2;
//  924   long int speed_1,speed_2;
//  925   int speed_diff=0;
        MOVS     R0,#+0
//  926 
//  927   speed_diff=Servo_value-Servo_Middle;
        LDR.W    R1,??DataTable46_16
        LDRSH    R1,[R1, #+0]
        LDR.W    R2,??DataTable46_17
        LDRSH    R2,[R2, #+0]
        SUBS     R1,R1,R2
        MOVS     R0,R1
//  928   if(speed_diff>0)
        CMP      R0,#+1
        BLT.N    ??set_speed_0
//  929   {      //左转
//  930     speed_diff=speed_diff/20;//25
        MOVS     R1,#+20
        SDIV     R0,R0,R1
//  931     if(speed_diff>=60)
        CMP      R0,#+60
        BLT.N    ??set_speed_1
//  932      speed_diff=59;
        MOVS     R0,#+59
//  933     
//  934     speed_1=(long)speed_low*Wspeed_diff[1][speed_diff]/100;
??set_speed_1:
        LDR.W    R1,??DataTable46_18
        ADDS     R1,R0,R1
        LDRB     R1,[R1, #+60]
        MUL      R1,R1,R4
        MOVS     R2,#+100
        SDIV     R5,R1,R2
//  935     speed_2=(long)speed_low*Wspeed_diff[0][speed_diff]/100;  //+
        LDR.W    R1,??DataTable46_18
        LDRB     R0,[R0, R1]
        MUL      R0,R0,R4
        MOVS     R1,#+100
        SDIV     R4,R0,R1
//  936     
//  937 
//  938     FTM0_C0V=limit(188+speed_2/2,375,0);
        MOVS     R2,#+0
        MOVW     R1,#+375
        MOVS     R0,#+2
        SDIV     R0,R4,R0
        ADDS     R0,R0,#+188
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       limit
        LDR.N    R1,??DataTable44_15  ;; 0x40038010
        STR      R0,[R1, #+0]
//  939     FTM0_C3V=limit(188-speed_2/2,375,0);
        MOVS     R2,#+0
        MOVW     R1,#+375
        MOVS     R0,#+2
        SDIV     R0,R4,R0
        RSBS     R0,R0,#+188
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       limit
        LDR.N    R1,??DataTable45_2  ;; 0x40038028
        STR      R0,[R1, #+0]
//  940     
//  941     FTM0_C4V=limit(188-speed_1/2,375,0);
        MOVS     R2,#+0
        MOVW     R1,#+375
        MOVS     R0,#+2
        SDIV     R0,R5,R0
        RSBS     R0,R0,#+188
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       limit
        LDR.N    R1,??DataTable45_3  ;; 0x40038030
        STR      R0,[R1, #+0]
//  942     FTM0_C5V=375;
        LDR.N    R0,??DataTable45_4  ;; 0x40038038
        MOVW     R1,#+375
        STR      R1,[R0, #+0]
//  943     FTM0_C6V=limit(188+speed_1/2,375,0);
        MOVS     R2,#+0
        MOVW     R1,#+375
        MOVS     R0,#+2
        SDIV     R0,R5,R0
        ADDS     R0,R0,#+188
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       limit
        LDR.N    R1,??DataTable45_5  ;; 0x40038040
        STR      R0,[R1, #+0]
        B.N      ??set_speed_2
//  944     
//  945  }else { 
//  946                 //右转
//  947      speed_diff=abs(speed_diff)/19;
??set_speed_0:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       abs
        MOVS     R1,#+19
        SDIV     R0,R0,R1
//  948     if(speed_diff>=60)
        CMP      R0,#+60
        BLT.N    ??set_speed_3
//  949       speed_diff=59;
        MOVS     R0,#+59
//  950     speed_1=(long)speed_low*Wspeed_diff[1][speed_diff]/100;
??set_speed_3:
        LDR.W    R1,??DataTable46_18
        ADDS     R1,R0,R1
        LDRB     R1,[R1, #+60]
        MUL      R1,R1,R4
        MOVS     R2,#+100
        SDIV     R5,R1,R2
//  951     speed_2=(long)speed_low*Wspeed_diff[0][speed_diff]/100; 
        LDR.W    R1,??DataTable46_18
        LDRB     R0,[R0, R1]
        MUL      R0,R0,R4
        MOVS     R1,#+100
        SDIV     R4,R0,R1
//  952 
//  953      FTM0_C0V=limit(188+speed_1/2,375,0);//198;
        MOVS     R2,#+0
        MOVW     R1,#+375
        MOVS     R0,#+2
        SDIV     R0,R5,R0
        ADDS     R0,R0,#+188
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       limit
        LDR.N    R1,??DataTable44_15  ;; 0x40038010
        STR      R0,[R1, #+0]
//  954     FTM0_C3V=limit(188-speed_1/2,375,0);//0;
        MOVS     R2,#+0
        MOVW     R1,#+375
        MOVS     R0,#+2
        SDIV     R0,R5,R0
        RSBS     R0,R0,#+188
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       limit
        LDR.N    R1,??DataTable45_2  ;; 0x40038028
        STR      R0,[R1, #+0]
//  955 
//  956     FTM0_C4V=limit(188-speed_2/2,375,0);//0;
        MOVS     R2,#+0
        MOVW     R1,#+375
        MOVS     R0,#+2
        SDIV     R0,R4,R0
        RSBS     R0,R0,#+188
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       limit
        LDR.N    R1,??DataTable45_3  ;; 0x40038030
        STR      R0,[R1, #+0]
//  957      FTM0_C5V=375;
        LDR.N    R0,??DataTable45_4  ;; 0x40038038
        MOVW     R1,#+375
        STR      R1,[R0, #+0]
//  958     FTM0_C6V=limit(188+speed_2/2,375,0);//190;
        MOVS     R2,#+0
        MOVW     R1,#+375
        MOVS     R0,#+2
        SDIV     R0,R4,R0
        ADDS     R0,R0,#+188
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       limit
        LDR.N    R1,??DataTable45_5  ;; 0x40038040
        STR      R0,[R1, #+0]
//  959  } 
//  960   return ok;
??set_speed_2:
        MOVS     R0,#+1
        POP      {R1,R4,R5,PC}    ;; return
//  961 }

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable42:
        DC32     0x40038024

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable42_1:
        DC32     0x40038014
//  962   
//  963 

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  964 int16 subjection_k(int16 *f,int16 i)
//  965 {
subjection_k:
        PUSH     {R4,LR}
        MOVS     R4,R0
//  966   int16 value;
//  967   
//  968   if(i>1)
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        CMP      R1,#+2
        BLT.N    ??subjection_k_0
//  969    value=subjection_k(f+1,i-1);
        SUBS     R1,R1,#+1
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        ADDS     R0,R4,#+2
        BL       subjection_k
        B.N      ??subjection_k_1
//  970   else
//  971    value=*f; 
??subjection_k_0:
        LDRSH    R0,[R4, #+0]
//  972    value=max(value,*(f+1));
??subjection_k_1:
        LDRSH    R1,[R4, #+2]
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       max
//  973   return value;
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        POP      {R4,PC}          ;; return
//  974 }
//  975 
//  976 
//  977 /************************读取速度************************/

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  978 void get_speed(void)
//  979 {
//  980   pulse=FTM2_CNT;
get_speed:
        LDR.W    R0,??DataTable46_10
        LDR.W    R1,??DataTable46_19  ;; 0x400b8004
        LDR      R1,[R1, #+0]
        STRH     R1,[R0, #+0]
//  981   FTM2_CNT=0;
        LDR.W    R0,??DataTable46_19  ;; 0x400b8004
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
//  982   //if(pulse==0)FTM0_C1V=0;     //如果电机没转那么电机就关掉
//  983  //pulse2=get_counter_value();
//  984 //lptmr_counter_clean(); 
//  985 }
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable43:
        DC32     0x4003802c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable43_1:
        DC32     0x40038034
//  986 /************************读取速度END************************/
//  987 
//  988     
//  989 /*************加权平均法对中心线的滤波平滑处理*************/

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  990 void center_filter(void)
//  991 {
center_filter:
        PUSH     {R2-R5}
//  992   char code_coe[5]={1,2,3,3,4};
        ADD      R0,SP,#+0
        LDR.W    R1,??DataTable47
        LDM      R1!,{R2,R3}
        STM      R0!,{R2,R3}
        SUBS     R1,R1,#+8
        SUBS     R0,R0,#+8
//  993   char code_coe_sum=13;
        MOVS     R0,#+13
//  994   int16 i=0,j=0,center_sum=0;
        MOVS     R1,#+0
        MOVS     R2,#+0
        MOVS     R3,#+0
//  995   for(i=52;i>1;i--)
        MOVS     R4,#+52
        MOVS     R1,R4
        B.N      ??center_filter_0
//  996   {
//  997     if(position[i-2]>0)
//  998     {
//  999       center_sum=0;
// 1000       for(j=0;j<5;j++)center_sum+=position[j+i-2]*code_coe[j];
??center_filter_1:
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        SXTAH    R4,R1,R2
        LDR.W    R5,??DataTable47_1
        ADDS     R4,R5,R4, LSL #+2
        LDR      R4,[R4, #-8]
        SXTH     R2,R2            ;; SignExt  R2,R2,#+16,#+16
        ADD      R5,SP,#+0
        LDRB     R5,[R2, R5]
        MULS     R4,R5,R4
        SXTAH    R3,R4,R3
        ADDS     R2,R2,#+1
??center_filter_2:
        SXTH     R2,R2            ;; SignExt  R2,R2,#+16,#+16
        CMP      R2,#+5
        BLT.N    ??center_filter_1
// 1001       position[i]=(int)(center_sum/code_coe_sum);
        SXTH     R3,R3            ;; SignExt  R3,R3,#+16,#+16
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        SDIV     R2,R3,R0
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        LDR.W    R3,??DataTable47_1
        STR      R2,[R3, R1, LSL #+2]
??center_filter_3:
        SUBS     R1,R1,#+1
??center_filter_0:
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        CMP      R1,#+2
        BLT.N    ??center_filter_4
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        LDR.W    R2,??DataTable47_1
        ADDS     R2,R2,R1, LSL #+2
        LDR      R2,[R2, #-8]
        CMP      R2,#+1
        BLT.N    ??center_filter_3
        MOVS     R3,#+0
        MOVS     R2,#+0
        B.N      ??center_filter_2
// 1002     }
// 1003   }  
// 1004 }
??center_filter_4:
        POP      {R0,R1,R4,R5}
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable44:
        DC32     0x40064006

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable44_1:
        DC32     0x4003803c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable44_2:
        DC32     0x40038000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable44_3:
        DC32     0x40038054

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable44_4:
        DC32     0x40038060

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable44_5:
        DC32     0x40038064

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable44_6:
        DC32     0x4003805c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable44_7:
        DC32     0x4003806c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable44_8:
        DC32     0x40038070

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable44_9:
        DC32     0x40038080

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable44_10:
        DC32     0x40038090

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable44_11:
        DC32     0x40038094

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable44_12:
        DC32     0x40038098

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable44_13:
        DC32     0x4003804c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable44_14:
        DC32     0x40038008

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable44_15:
        DC32     0x40038010
// 1005 /*************加权平均法对中心线的滤波平滑处理*END************/
// 1006 
// 1007 

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1008 void sub_e(uint8 value,uint8 S_end,uint8 M_start,uint8 M_end,uint8 B_start)//sub_e(E,35,30,50,45);
// 1009 {
sub_e:
        PUSH     {R4-R6}
        LDR      R4,[SP, #+12]
// 1010   if(value<=M_start)
        UXTB     R2,R2            ;; ZeroExt  R2,R2,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        CMP      R2,R0
        BCC.N    ??sub_e_0
// 1011   {
// 1012     sub.s=100;
        LDR.W    R5,??DataTable49
        MOVS     R6,#+100
        STRH     R6,[R5, #+0]
// 1013     sub.m=0;
        LDR.W    R5,??DataTable49
        MOVS     R6,#+0
        STRH     R6,[R5, #+2]
// 1014     sub.b=0;
        LDR.W    R5,??DataTable49
        MOVS     R6,#+0
        STRH     R6,[R5, #+4]
// 1015   }
// 1016   if(value<S_end&&value>M_start)
??sub_e_0:
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        CMP      R0,R1
        BCS.N    ??sub_e_1
        UXTB     R2,R2            ;; ZeroExt  R2,R2,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        CMP      R2,R0
        BCS.N    ??sub_e_1
// 1017   {
// 1018     sub.s=100*(S_end-value)/(S_end-M_start);
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        SUBS     R5,R1,R0
        MOVS     R6,#+100
        MULS     R5,R6,R5
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R2,R2            ;; ZeroExt  R2,R2,#+24,#+24
        SUBS     R6,R1,R2
        SDIV     R5,R5,R6
        LDR.W    R6,??DataTable49
        STRH     R5,[R6, #+0]
// 1019     sub.m=100*(value-M_start)/(S_end-M_start);
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        UXTB     R2,R2            ;; ZeroExt  R2,R2,#+24,#+24
        SUBS     R5,R0,R2
        MOVS     R6,#+100
        MULS     R5,R6,R5
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R2,R2            ;; ZeroExt  R2,R2,#+24,#+24
        SUBS     R2,R1,R2
        SDIV     R2,R5,R2
        LDR.W    R5,??DataTable49
        STRH     R2,[R5, #+2]
// 1020     sub.b=0;
        LDR.W    R2,??DataTable49
        MOVS     R5,#+0
        STRH     R5,[R2, #+4]
// 1021   }
// 1022   if(value>=S_end&&value<=B_start)
??sub_e_1:
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        CMP      R0,R1
        BCC.N    ??sub_e_2
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        CMP      R4,R0
        BCC.N    ??sub_e_2
// 1023   {
// 1024     sub.s=0;
        LDR.W    R1,??DataTable49
        MOVS     R2,#+0
        STRH     R2,[R1, #+0]
// 1025     sub.m=100;
        LDR.W    R1,??DataTable49
        MOVS     R2,#+100
        STRH     R2,[R1, #+2]
// 1026     sub.b=0;
        LDR.W    R1,??DataTable49
        MOVS     R2,#+0
        STRH     R2,[R1, #+4]
// 1027   }
// 1028   if(value<M_end&&value>B_start)
??sub_e_2:
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        UXTB     R3,R3            ;; ZeroExt  R3,R3,#+24,#+24
        CMP      R0,R3
        BCS.N    ??sub_e_3
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        CMP      R4,R0
        BCS.N    ??sub_e_3
// 1029   {
// 1030     sub.s=0;
        LDR.W    R1,??DataTable49
        MOVS     R2,#+0
        STRH     R2,[R1, #+0]
// 1031     sub.m=100*(M_end-value)/(M_end-B_start);
        UXTB     R3,R3            ;; ZeroExt  R3,R3,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        SUBS     R1,R3,R0
        MOVS     R2,#+100
        MULS     R1,R2,R1
        UXTB     R3,R3            ;; ZeroExt  R3,R3,#+24,#+24
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        SUBS     R2,R3,R4
        SDIV     R1,R1,R2
        LDR.W    R2,??DataTable49
        STRH     R1,[R2, #+2]
// 1032     sub.b=100*(value-B_start)/(M_end-B_start);
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        SUBS     R1,R0,R4
        MOVS     R2,#+100
        MULS     R1,R2,R1
        UXTB     R3,R3            ;; ZeroExt  R3,R3,#+24,#+24
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        SUBS     R2,R3,R4
        SDIV     R1,R1,R2
        LDR.W    R2,??DataTable49
        STRH     R1,[R2, #+4]
// 1033   }
// 1034   if(value>=M_end)
??sub_e_3:
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        UXTB     R3,R3            ;; ZeroExt  R3,R3,#+24,#+24
        CMP      R0,R3
        BCC.N    ??sub_e_4
// 1035   {
// 1036     sub.s=0;
        LDR.W    R0,??DataTable49
        MOVS     R1,#+0
        STRH     R1,[R0, #+0]
// 1037     sub.m=0;
        LDR.W    R0,??DataTable49
        MOVS     R1,#+0
        STRH     R1,[R0, #+2]
// 1038     sub.b=100;
        LDR.W    R0,??DataTable49
        MOVS     R1,#+100
        STRH     R1,[R0, #+4]
// 1039   }
// 1040 }
??sub_e_4:
        POP      {R4-R6}
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45:
        DC32     0x40039004

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_1:
        DC32     0x40038018

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_2:
        DC32     0x40038028

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_3:
        DC32     0x40038030

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_4:
        DC32     0x40038038

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_5:
        DC32     0x40038040

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_6:
        DC32     0x40038004

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_7:
        DC32     0x40049028

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_8:
        DC32     0x4004902c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_9:
        DC32     0x40048030

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_10:
        DC32     0x400b8054

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_11:
        DC32     0x400b8080

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_12:
        DC32     0x400b804c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_13:
        DC32     0x400b8008

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_14:
        DC32     0x400b8004

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_15:
        DC32     0x40049038

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_16:
        DC32     0x4004903c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_17:
        DC32     0x40049040

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_18:
        DC32     0x40049044

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_19:
        DC32     0x4004a000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_20:
        DC32     0x4004a004

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_21:
        DC32     0x4004a008

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_22:
        DC32     0x4004a00c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_23:
        DC32     0x4004a010

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_24:
        DC32     0x4004a014

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_25:
        DC32     0x4004a018

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_26:
        DC32     0x4004a01c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_27:
        DC32     0x400ff054

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_28:
        DC32     0x40049060

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_29:
        DC32     0xa0100

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_30:
        DC32     0x4004a028

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_31:
        DC32     0x90100

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_32:
        DC32     0x4004a02c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_33:
        DC32     0x10100

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_34:
        DC32     0x4004b024

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_35:
        DC32     0x400ff014

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_36:
        DC32     0x14034000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_37:
        DC32     0x40049070

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_38:
        DC32     0x40049068

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_39:
        DC32     0x4006a000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_40:
        DC32     ImageData

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_41:
        DC32     0x2bf20

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_42:
        DC32     0x4004a040

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_43:
        DC32     0x4004a044

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_44:
        DC32     0x4006a001

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_45:
        DC32     0x4006a00a

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_46:
        DC32     0x4006a003

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_47:
        DC32     0x4006a002

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_48:
        DC32     0x40048040

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_49:
        DC32     0x40021000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_50:
        DC32     0x40009016

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_51:
        DC32     0x4000901e

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_52:
        DC32     0x40009000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_53:
        DC32     0x400ff050

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_54:
        DC32     0x40009004

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_55:
        DC32     0x40009008

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_56:
        DC32     0x4000900c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_57:
        DC32     0x40009018

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_58:
        DC32     0x40009010

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_59:
        DC32     0x40009014

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_60:
        DC32     0x40009006

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_61:
        DC32     0x4000901c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_62:
        DC32     0x40037000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable45_63:
        DC32     0x40037100
// 1041 
// 1042 

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1043 int16 motor_fuzzy(int16 e,int16 ec,int16 S_end,int16 M_start,int16 M_end,int16 B_start,int16 Sc_end,int16 Mc_start,int16 Mc_end,int16 Bc_start)
// 1044 {
motor_fuzzy:
        PUSH     {R2-R11,LR}
        SUB      SP,SP,#+84
        MOVS     R4,R0
        MOVS     R5,R1
        LDRSH    R6,[SP, #+128]
        LDRSH    R7,[SP, #+132]
        LDRSH    R8,[SP, #+136]
        LDRSH    R9,[SP, #+140]
        LDRSH    R10,[SP, #+144]
        LDRSH    R11,[SP, #+148]
// 1045   volatile int16 ib=0,im=0,is=0,iz=0;
        MOVS     R0,#+0
        STRH     R0,[SP, #+8]
        MOVS     R0,#+0
        STRH     R0,[SP, #+6]
        MOVS     R0,#+0
        STRH     R0,[SP, #+4]
        MOVS     R0,#+0
        STRH     R0,[SP, #+18]
// 1046   volatile int16 Se=0,Me=0,Be=0;
        MOVS     R0,#+0
        STRH     R0,[SP, #+16]
        MOVS     R0,#+0
        STRH     R0,[SP, #+14]
        MOVS     R0,#+0
        STRH     R0,[SP, #+12]
// 1047   volatile int16 Sc=0,Mc=0,Bc=0;
        MOVS     R0,#+0
        STRH     R0,[SP, #+10]
        MOVS     R0,#+0
        STRH     R0,[SP, #+2]
        MOVS     R0,#+0
        STRH     R0,[SP, #+0]
// 1048   int16 vb[6]={0};
        ADD      R0,SP,#+60
        MOVS     R1,#+0
        MOVS     R2,#+0
        MOVS     R3,#+0
        STM      R0!,{R1-R3}
        SUBS     R0,R0,#+12
// 1049   int16 vm[6]={0};
        ADD      R0,SP,#+48
        MOVS     R1,#+0
        MOVS     R2,#+0
        MOVS     R3,#+0
        STM      R0!,{R1-R3}
        SUBS     R0,R0,#+12
// 1050   int16 vs[6]={0};
        ADD      R0,SP,#+36
        MOVS     R1,#+0
        MOVS     R2,#+0
        MOVS     R3,#+0
        STM      R0!,{R1-R3}
        SUBS     R0,R0,#+12
// 1051   int16 vz[6]={0};
        ADD      R0,SP,#+72
        MOVS     R1,#+0
        MOVS     R2,#+0
        MOVS     R3,#+0
        STM      R0!,{R1-R3}
        SUBS     R0,R0,#+12
// 1052   volatile long Ks=0,Km=0,Kb=0,Kz=0;
        MOVS     R0,#+0
        STR      R0,[SP, #+28]
        MOVS     R0,#+0
        STR      R0,[SP, #+24]
        MOVS     R0,#+0
        STR      R0,[SP, #+20]
        MOVS     R0,#+0
        STR      R0,[SP, #+32]
// 1053   int16 *f;
// 1054   Motor.P=0;
        LDR.W    R0,??DataTable50
        MOVS     R1,#+0
        STRH     R1,[R0, #+0]
// 1055   Motor.I=0;
        LDR.W    R0,??DataTable50
        MOVS     R1,#+0
        STRH     R1,[R0, #+2]
// 1056 
// 1057   e=abs(e);
        MOVS     R0,R4
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       abs
        MOVS     R4,R0
// 1058   ec=abs(ec);
        MOVS     R0,R5
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       abs
        MOVS     R5,R0
// 1059   
// 1060   sub_e(e,S_end,M_start,M_end,B_start);
        UXTB     R7,R7            ;; ZeroExt  R7,R7,#+24,#+24
        STR      R7,[SP, #+0]
        MOVS     R3,R6
        UXTB     R3,R3            ;; ZeroExt  R3,R3,#+24,#+24
        LDRH     R2,[SP, #+88]
        UXTB     R2,R2            ;; ZeroExt  R2,R2,#+24,#+24
        LDRH     R1,[SP, #+84]
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        MOVS     R0,R4
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        BL       sub_e
// 1061  
// 1062   Se=sub.s;
        LDR.W    R0,??DataTable49
        LDRH     R0,[R0, #+0]
        STRH     R0,[SP, #+16]
// 1063   Me=sub.m;
        LDR.W    R0,??DataTable49
        LDRH     R0,[R0, #+2]
        STRH     R0,[SP, #+14]
// 1064   Be=sub.b;
        LDR.W    R0,??DataTable49
        LDRH     R0,[R0, #+4]
        STRH     R0,[SP, #+12]
// 1065 
// 1066   sub_e(ec,Sc_end,Mc_start,Mc_end,Bc_start);
        UXTB     R11,R11          ;; ZeroExt  R11,R11,#+24,#+24
        STR      R11,[SP, #+0]
        MOV      R3,R10
        UXTB     R3,R3            ;; ZeroExt  R3,R3,#+24,#+24
        MOV      R2,R9
        UXTB     R2,R2            ;; ZeroExt  R2,R2,#+24,#+24
        MOV      R1,R8
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        MOVS     R0,R5
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        BL       sub_e
// 1067  
// 1068   Sc=sub.s;
        LDR.W    R0,??DataTable49
        LDRH     R0,[R0, #+0]
        STRH     R0,[SP, #+10]
// 1069   Mc=sub.m;
        LDR.W    R0,??DataTable49
        LDRH     R0,[R0, #+2]
        STRH     R0,[SP, #+2]
// 1070   Bc=sub.b;
        LDR.W    R0,??DataTable49
        LDRH     R0,[R0, #+4]
        STRH     R0,[SP, #+0]
// 1071 
// 1072 
// 1073   if(Se&&Sc)  {vm[im]=min(Se,Sc);im++;}  //P建表
        LDRSH    R0,[SP, #+16]
        CMP      R0,#+0
        BEQ.N    ??motor_fuzzy_0
        LDRSH    R0,[SP, #+10]
        CMP      R0,#+0
        BEQ.N    ??motor_fuzzy_0
        LDRSH    R4,[SP, #+6]
        LDRSH    R1,[SP, #+10]
        LDRSH    R0,[SP, #+16]
        BL       min
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        ADD      R1,SP,#+48
        STRH     R0,[R1, R4, LSL #+1]
        LDRH     R0,[SP, #+6]
        ADDS     R0,R0,#+1
        STRH     R0,[SP, #+6]
// 1074   if(Se&&Mc)  {vs[is]=min(Se,Mc);is++;}  
??motor_fuzzy_0:
        LDRSH    R0,[SP, #+16]
        CMP      R0,#+0
        BEQ.N    ??motor_fuzzy_1
        LDRSH    R0,[SP, #+2]
        CMP      R0,#+0
        BEQ.N    ??motor_fuzzy_1
        LDRSH    R4,[SP, #+4]
        LDRSH    R1,[SP, #+2]
        LDRSH    R0,[SP, #+16]
        BL       min
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        ADD      R1,SP,#+36
        STRH     R0,[R1, R4, LSL #+1]
        LDRH     R0,[SP, #+4]
        ADDS     R0,R0,#+1
        STRH     R0,[SP, #+4]
// 1075   if(Se&&Bc)  {vs[is]=min(Se,Bc);is++;}
??motor_fuzzy_1:
        LDRSH    R0,[SP, #+16]
        CMP      R0,#+0
        BEQ.N    ??motor_fuzzy_2
        LDRSH    R0,[SP, #+0]
        CMP      R0,#+0
        BEQ.N    ??motor_fuzzy_2
        LDRSH    R4,[SP, #+4]
        LDRSH    R1,[SP, #+0]
        LDRSH    R0,[SP, #+16]
        BL       min
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        ADD      R1,SP,#+36
        STRH     R0,[R1, R4, LSL #+1]
        LDRH     R0,[SP, #+4]
        ADDS     R0,R0,#+1
        STRH     R0,[SP, #+4]
// 1076   if(Me&&Sc)  {vm[im]=min(Me,Sc);im++;}
??motor_fuzzy_2:
        LDRSH    R0,[SP, #+14]
        CMP      R0,#+0
        BEQ.N    ??motor_fuzzy_3
        LDRSH    R0,[SP, #+10]
        CMP      R0,#+0
        BEQ.N    ??motor_fuzzy_3
        LDRSH    R4,[SP, #+6]
        LDRSH    R1,[SP, #+10]
        LDRSH    R0,[SP, #+14]
        BL       min
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        ADD      R1,SP,#+48
        STRH     R0,[R1, R4, LSL #+1]
        LDRH     R0,[SP, #+6]
        ADDS     R0,R0,#+1
        STRH     R0,[SP, #+6]
// 1077   if(Me&&Mc)  {vm[im]=min(Me,Mc);im++;}
??motor_fuzzy_3:
        LDRSH    R0,[SP, #+14]
        CMP      R0,#+0
        BEQ.N    ??motor_fuzzy_4
        LDRSH    R0,[SP, #+2]
        CMP      R0,#+0
        BEQ.N    ??motor_fuzzy_4
        LDRSH    R4,[SP, #+6]
        LDRSH    R1,[SP, #+2]
        LDRSH    R0,[SP, #+14]
        BL       min
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        ADD      R1,SP,#+48
        STRH     R0,[R1, R4, LSL #+1]
        LDRH     R0,[SP, #+6]
        ADDS     R0,R0,#+1
        STRH     R0,[SP, #+6]
// 1078   if(Me&&Bc)  {vs[is]=min(Me,Bc);is++;}
??motor_fuzzy_4:
        LDRSH    R0,[SP, #+14]
        CMP      R0,#+0
        BEQ.N    ??motor_fuzzy_5
        LDRSH    R0,[SP, #+0]
        CMP      R0,#+0
        BEQ.N    ??motor_fuzzy_5
        LDRSH    R4,[SP, #+4]
        LDRSH    R1,[SP, #+0]
        LDRSH    R0,[SP, #+14]
        BL       min
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        ADD      R1,SP,#+36
        STRH     R0,[R1, R4, LSL #+1]
        LDRH     R0,[SP, #+4]
        ADDS     R0,R0,#+1
        STRH     R0,[SP, #+4]
// 1079   if(Be&&Sc)  {vb[ib]=min(Be,Sc);ib++;}
??motor_fuzzy_5:
        LDRSH    R0,[SP, #+12]
        CMP      R0,#+0
        BEQ.N    ??motor_fuzzy_6
        LDRSH    R0,[SP, #+10]
        CMP      R0,#+0
        BEQ.N    ??motor_fuzzy_6
        LDRSH    R4,[SP, #+8]
        LDRSH    R1,[SP, #+10]
        LDRSH    R0,[SP, #+12]
        BL       min
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        ADD      R1,SP,#+60
        STRH     R0,[R1, R4, LSL #+1]
        LDRH     R0,[SP, #+8]
        ADDS     R0,R0,#+1
        STRH     R0,[SP, #+8]
// 1080   if(Be&&Mc)  {vb[ib]=min(Be,Mc);ib++;} 
??motor_fuzzy_6:
        LDRSH    R0,[SP, #+12]
        CMP      R0,#+0
        BEQ.N    ??motor_fuzzy_7
        LDRSH    R0,[SP, #+2]
        CMP      R0,#+0
        BEQ.N    ??motor_fuzzy_7
        LDRSH    R4,[SP, #+8]
        LDRSH    R1,[SP, #+2]
        LDRSH    R0,[SP, #+12]
        BL       min
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        ADD      R1,SP,#+60
        STRH     R0,[R1, R4, LSL #+1]
        LDRH     R0,[SP, #+8]
        ADDS     R0,R0,#+1
        STRH     R0,[SP, #+8]
// 1081   if(Be&&Bc)  {vm[im]=min(Be,Bc);im++;}
??motor_fuzzy_7:
        LDRSH    R0,[SP, #+12]
        CMP      R0,#+0
        BEQ.N    ??motor_fuzzy_8
        LDRSH    R0,[SP, #+0]
        CMP      R0,#+0
        BEQ.N    ??motor_fuzzy_8
        LDRSH    R4,[SP, #+6]
        LDRSH    R1,[SP, #+0]
        LDRSH    R0,[SP, #+12]
        BL       min
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        ADD      R1,SP,#+48
        STRH     R0,[R1, R4, LSL #+1]
        LDRH     R0,[SP, #+6]
        ADDS     R0,R0,#+1
        STRH     R0,[SP, #+6]
// 1082 
// 1083 
// 1084   if(vs[0]){
??motor_fuzzy_8:
        LDRSH    R0,[SP, #+36]
        CMP      R0,#+0
        BEQ.N    ??motor_fuzzy_9
// 1085   f=&vs[0];
        ADD      R0,SP,#+36
// 1086   Ks=subjection_k(f,is);
        LDRSH    R1,[SP, #+4]
        BL       subjection_k
        STR      R0,[SP, #+28]
// 1087   }
// 1088   if(vm[0]){
??motor_fuzzy_9:
        LDRSH    R0,[SP, #+48]
        CMP      R0,#+0
        BEQ.N    ??motor_fuzzy_10
// 1089   f=&vm[0];
        ADD      R0,SP,#+48
// 1090   Km=subjection_k(f,im); 
        LDRSH    R1,[SP, #+6]
        BL       subjection_k
        STR      R0,[SP, #+24]
// 1091   }
// 1092   if(vb[0]){
??motor_fuzzy_10:
        LDRSH    R0,[SP, #+60]
        CMP      R0,#+0
        BEQ.N    ??motor_fuzzy_11
// 1093   f=&vb[0];
        ADD      R0,SP,#+60
// 1094   Kb=subjection_k(f,ib);
        LDRSH    R1,[SP, #+8]
        BL       subjection_k
        STR      R0,[SP, #+20]
// 1095   }
// 1096 
// 1097   Motor.P=(MP_B*Kb+MP_M*Km+MP_S*Ks)/(Kb+Km+Ks);
??motor_fuzzy_11:
        LDR      R0,[SP, #+20]
        MOV      R1,#+480
        LDR      R2,[SP, #+24]
        MOV      R3,#+440
        MULS     R2,R3,R2
        MLA      R0,R1,R0,R2
        LDR      R1,[SP, #+28]
        MOV      R2,#+400
        MLA      R0,R2,R1,R0
        LDR      R1,[SP, #+20]
        LDR      R2,[SP, #+24]
        ADDS     R1,R2,R1
        LDR      R2,[SP, #+28]
        ADDS     R1,R2,R1
        SDIV     R0,R0,R1
        LDR.W    R1,??DataTable50
        STRH     R0,[R1, #+0]
// 1098 
// 1099   
// 1100   for(is=0;is<6;is++){
        MOVS     R0,#+0
        STRH     R0,[SP, #+4]
        B.N      ??motor_fuzzy_12
// 1101     vs[is]=0;
??motor_fuzzy_13:
        LDRSH    R0,[SP, #+4]
        ADD      R1,SP,#+36
        MOVS     R2,#+0
        STRH     R2,[R1, R0, LSL #+1]
// 1102     vm[is]=0;
        LDRSH    R0,[SP, #+4]
        ADD      R1,SP,#+48
        MOVS     R2,#+0
        STRH     R2,[R1, R0, LSL #+1]
// 1103     vb[is]=0;
        LDRSH    R0,[SP, #+4]
        ADD      R1,SP,#+60
        MOVS     R2,#+0
        STRH     R2,[R1, R0, LSL #+1]
// 1104   }
        LDRH     R0,[SP, #+4]
        ADDS     R0,R0,#+1
        STRH     R0,[SP, #+4]
??motor_fuzzy_12:
        LDRSH    R0,[SP, #+4]
        CMP      R0,#+6
        BLT.N    ??motor_fuzzy_13
// 1105   is=0;
        MOVS     R0,#+0
        STRH     R0,[SP, #+4]
// 1106   im=0;
        MOVS     R0,#+0
        STRH     R0,[SP, #+6]
// 1107   ib=0;
        MOVS     R0,#+0
        STRH     R0,[SP, #+8]
// 1108   Ks=0;Km=0;Kb=0;Kz=0;
        MOVS     R0,#+0
        STR      R0,[SP, #+28]
        MOVS     R0,#+0
        STR      R0,[SP, #+24]
        MOVS     R0,#+0
        STR      R0,[SP, #+20]
        MOVS     R0,#+0
        STR      R0,[SP, #+32]
// 1109   ///////////////////////////////////////I参数
// 1110   if(Se&&Sc)  {vb[ib]=min(Se,Sc);ib++;}  //建表
        LDRSH    R0,[SP, #+16]
        CMP      R0,#+0
        BEQ.N    ??motor_fuzzy_14
        LDRSH    R0,[SP, #+10]
        CMP      R0,#+0
        BEQ.N    ??motor_fuzzy_14
        LDRSH    R4,[SP, #+8]
        LDRSH    R1,[SP, #+10]
        LDRSH    R0,[SP, #+16]
        BL       min
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        ADD      R1,SP,#+60
        STRH     R0,[R1, R4, LSL #+1]
        LDRH     R0,[SP, #+8]
        ADDS     R0,R0,#+1
        STRH     R0,[SP, #+8]
// 1111   if(Se&&Mc)  {vb[ib]=min(Se,Mc);ib++;}
??motor_fuzzy_14:
        LDRSH    R0,[SP, #+16]
        CMP      R0,#+0
        BEQ.N    ??motor_fuzzy_15
        LDRSH    R0,[SP, #+2]
        CMP      R0,#+0
        BEQ.N    ??motor_fuzzy_15
        LDRSH    R4,[SP, #+8]
        LDRSH    R1,[SP, #+2]
        LDRSH    R0,[SP, #+16]
        BL       min
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        ADD      R1,SP,#+60
        STRH     R0,[R1, R4, LSL #+1]
        LDRH     R0,[SP, #+8]
        ADDS     R0,R0,#+1
        STRH     R0,[SP, #+8]
// 1112   if(Se&&Bc)  {vm[im]=min(Se,Bc);im++;}
??motor_fuzzy_15:
        LDRSH    R0,[SP, #+16]
        CMP      R0,#+0
        BEQ.N    ??motor_fuzzy_16
        LDRSH    R0,[SP, #+0]
        CMP      R0,#+0
        BEQ.N    ??motor_fuzzy_16
        LDRSH    R4,[SP, #+6]
        LDRSH    R1,[SP, #+0]
        LDRSH    R0,[SP, #+16]
        BL       min
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        ADD      R1,SP,#+48
        STRH     R0,[R1, R4, LSL #+1]
        LDRH     R0,[SP, #+6]
        ADDS     R0,R0,#+1
        STRH     R0,[SP, #+6]
// 1113   if(Me&&Sc)  {vz[iz]=min(Me,Sc);iz++;}
??motor_fuzzy_16:
        LDRSH    R0,[SP, #+14]
        CMP      R0,#+0
        BEQ.N    ??motor_fuzzy_17
        LDRSH    R0,[SP, #+10]
        CMP      R0,#+0
        BEQ.N    ??motor_fuzzy_17
        LDRSH    R4,[SP, #+18]
        LDRSH    R1,[SP, #+10]
        LDRSH    R0,[SP, #+14]
        BL       min
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        ADD      R1,SP,#+72
        STRH     R0,[R1, R4, LSL #+1]
        LDRH     R0,[SP, #+18]
        ADDS     R0,R0,#+1
        STRH     R0,[SP, #+18]
// 1114   if(Me&&Mc)  {vs[is]=min(Me,Mc);is++;}
??motor_fuzzy_17:
        LDRSH    R0,[SP, #+14]
        CMP      R0,#+0
        BEQ.N    ??motor_fuzzy_18
        LDRSH    R0,[SP, #+2]
        CMP      R0,#+0
        BEQ.N    ??motor_fuzzy_18
        LDRSH    R4,[SP, #+4]
        LDRSH    R1,[SP, #+2]
        LDRSH    R0,[SP, #+14]
        BL       min
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        ADD      R1,SP,#+36
        STRH     R0,[R1, R4, LSL #+1]
        LDRH     R0,[SP, #+4]
        ADDS     R0,R0,#+1
        STRH     R0,[SP, #+4]
// 1115   if(Me&&Bc)  {vs[is]=min(Me,Bc);is++;}
??motor_fuzzy_18:
        LDRSH    R0,[SP, #+14]
        CMP      R0,#+0
        BEQ.N    ??motor_fuzzy_19
        LDRSH    R0,[SP, #+0]
        CMP      R0,#+0
        BEQ.N    ??motor_fuzzy_19
        LDRSH    R4,[SP, #+4]
        LDRSH    R1,[SP, #+0]
        LDRSH    R0,[SP, #+14]
        BL       min
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        ADD      R1,SP,#+36
        STRH     R0,[R1, R4, LSL #+1]
        LDRH     R0,[SP, #+4]
        ADDS     R0,R0,#+1
        STRH     R0,[SP, #+4]
// 1116   if(Be&&Sc)  {vz[iz]=min(Be,Sc);iz++;}
??motor_fuzzy_19:
        LDRSH    R0,[SP, #+12]
        CMP      R0,#+0
        BEQ.N    ??motor_fuzzy_20
        LDRSH    R0,[SP, #+10]
        CMP      R0,#+0
        BEQ.N    ??motor_fuzzy_20
        LDRSH    R4,[SP, #+18]
        LDRSH    R1,[SP, #+10]
        LDRSH    R0,[SP, #+12]
        BL       min
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        ADD      R1,SP,#+72
        STRH     R0,[R1, R4, LSL #+1]
        LDRH     R0,[SP, #+18]
        ADDS     R0,R0,#+1
        STRH     R0,[SP, #+18]
// 1117   if(Be&&Mc)  {vz[iz]=min(Be,Mc);iz++;} 
??motor_fuzzy_20:
        LDRSH    R0,[SP, #+12]
        CMP      R0,#+0
        BEQ.N    ??motor_fuzzy_21
        LDRSH    R0,[SP, #+2]
        CMP      R0,#+0
        BEQ.N    ??motor_fuzzy_21
        LDRSH    R4,[SP, #+18]
        LDRSH    R1,[SP, #+2]
        LDRSH    R0,[SP, #+12]
        BL       min
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        ADD      R1,SP,#+72
        STRH     R0,[R1, R4, LSL #+1]
        LDRH     R0,[SP, #+18]
        ADDS     R0,R0,#+1
        STRH     R0,[SP, #+18]
// 1118   if(Be&&Bc)  {vz[iz]=min(Be,Bc);iz++;}
??motor_fuzzy_21:
        LDRSH    R0,[SP, #+12]
        CMP      R0,#+0
        BEQ.N    ??motor_fuzzy_22
        LDRSH    R0,[SP, #+0]
        CMP      R0,#+0
        BEQ.N    ??motor_fuzzy_22
        LDRSH    R4,[SP, #+18]
        LDRSH    R1,[SP, #+0]
        LDRSH    R0,[SP, #+12]
        BL       min
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        ADD      R1,SP,#+72
        STRH     R0,[R1, R4, LSL #+1]
        LDRH     R0,[SP, #+18]
        ADDS     R0,R0,#+1
        STRH     R0,[SP, #+18]
// 1119 
// 1120   if(vz[0]){
??motor_fuzzy_22:
        LDRSH    R0,[SP, #+72]
        CMP      R0,#+0
        BEQ.N    ??motor_fuzzy_23
// 1121   f=&vz[0];
        ADD      R0,SP,#+72
// 1122   Kz=subjection_k(f,iz);
        LDRSH    R1,[SP, #+18]
        BL       subjection_k
        STR      R0,[SP, #+32]
// 1123   }
// 1124   if(vs[0]){
??motor_fuzzy_23:
        LDRSH    R0,[SP, #+36]
        CMP      R0,#+0
        BEQ.N    ??motor_fuzzy_24
// 1125   f=&vs[0];
        ADD      R0,SP,#+36
// 1126   Ks=subjection_k(f,is);
        LDRSH    R1,[SP, #+4]
        BL       subjection_k
        STR      R0,[SP, #+28]
// 1127   }
// 1128   if(vm[0]){
??motor_fuzzy_24:
        LDRSH    R0,[SP, #+48]
        CMP      R0,#+0
        BEQ.N    ??motor_fuzzy_25
// 1129   f=&vm[0];
        ADD      R0,SP,#+48
// 1130   Km=subjection_k(f,im);
        LDRSH    R1,[SP, #+6]
        BL       subjection_k
        STR      R0,[SP, #+24]
// 1131   }
// 1132   if(vb[0]){
??motor_fuzzy_25:
        LDRSH    R0,[SP, #+60]
        CMP      R0,#+0
        BEQ.N    ??motor_fuzzy_26
// 1133   f=&vb[0];
        ADD      R0,SP,#+60
// 1134   Kb=subjection_k(f,ib);
        LDRSH    R1,[SP, #+8]
        BL       subjection_k
        STR      R0,[SP, #+20]
// 1135   }
// 1136   
// 1137   Motor.I=(MI_B*Kb+MI_M*Km+MI_S*Ks)/(Kb+Km+Ks+Kz);
??motor_fuzzy_26:
        LDR      R0,[SP, #+20]
        MOVS     R1,#+12
        LDR      R2,[SP, #+24]
        MOVS     R3,#+9
        MULS     R2,R3,R2
        MLA      R0,R1,R0,R2
        LDR      R1,[SP, #+28]
        MOVS     R2,#+6
        MLA      R0,R2,R1,R0
        LDR      R1,[SP, #+20]
        LDR      R2,[SP, #+24]
        ADDS     R1,R2,R1
        LDR      R2,[SP, #+28]
        ADDS     R1,R2,R1
        LDR      R2,[SP, #+32]
        ADDS     R1,R2,R1
        SDIV     R0,R0,R1
        LDR.W    R1,??DataTable50
        STRH     R0,[R1, #+2]
// 1138 
// 1139 }
        ADD      SP,SP,#+92
        POP      {R4-R11,PC}      ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable46:
        DC32     0x40008024

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable46_1:
        DC32     imagerow

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable46_2:
        DC32     row_F

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable46_3:
        DC32     row

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable46_4:
        DC32     data_table

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable46_5:
        DC32     0x4000800c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable46_6:
        DC32     finish

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable46_7:
        DC32     0x400ff000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable46_8:
        DC32     0x4003711c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable46_9:
        DC32     timer

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable46_10:
        DC32     pulse

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable46_11:
        DC32     pulse2

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable46_12:
        DC32     0x40040000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable46_13:
        DC32     0x4003710c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable46_14:
        DC32     P_timer

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable46_15:
        DC32     0x4003712c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable46_16:
        DC32     Servo_value

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable46_17:
        DC32     Servo_Middle

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable46_18:
        DC32     Wspeed_diff

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable46_19:
        DC32     0x400b8004
// 1140 //speed_fuzzy(E,Ec,35,30,50,45,3,1,7,5)

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1141 int16 speed_fuzzy(int16 e,int16 ec,int16 s_end,int16 m_start,int16 b_start,int16 m_end,int16 sc_end,int16 mc_start,int16 bc_start,int16 mc_end)
// 1142 {
speed_fuzzy:
        PUSH     {R0,R2,R4-R11,LR}
        SUB      SP,SP,#+84
        MOVS     R5,R1
        MOVS     R4,R3
        LDRSH    R10,[SP, #+128]
        LDRSH    R11,[SP, #+132]
        LDRSH    R6,[SP, #+136]
        LDRSH    R7,[SP, #+140]
        LDRSH    R8,[SP, #+144]
        LDRSH    R9,[SP, #+148]
// 1143   volatile int16 ib=0,im=0,is=0,exspeed=0;
        MOVS     R0,#+0
        STRH     R0,[SP, #+6]
        MOVS     R0,#+0
        STRH     R0,[SP, #+8]
        MOVS     R0,#+0
        STRH     R0,[SP, #+4]
        MOVS     R0,#+0
        STRH     R0,[SP, #+0]
// 1144   volatile int16 Se=0,Me=0,Be=0;
        MOVS     R0,#+0
        STRH     R0,[SP, #+16]
        MOVS     R0,#+0
        STRH     R0,[SP, #+14]
        MOVS     R0,#+0
        STRH     R0,[SP, #+12]
// 1145   volatile int16 Sc=0,Mc=0,Bc=0;
        MOVS     R0,#+0
        STRH     R0,[SP, #+10]
        MOVS     R0,#+0
        STRH     R0,[SP, #+2]
        MOVS     R0,#+0
        STRH     R0,[SP, #+0]
// 1146   int16 vb[8]={0};
        ADD      R0,SP,#+48
        MOVS     R1,#+16
        BL       __aeabi_memclr4
// 1147   int16 vm[8]={0};
        ADD      R0,SP,#+64
        MOVS     R1,#+16
        BL       __aeabi_memclr4
// 1148   int16 vs[8]={0};
        ADD      R0,SP,#+32
        MOVS     R1,#+16
        BL       __aeabi_memclr4
// 1149   volatile long Ks=0,Km=0,Kb=0;
        MOVS     R0,#+0
        STR      R0,[SP, #+28]
        MOVS     R0,#+0
        STR      R0,[SP, #+24]
        MOVS     R0,#+0
        STR      R0,[SP, #+20]
// 1150   int16 *f;
// 1151   
// 1152   sub_e(e,s_end,m_start,b_start,m_end); //变化差
        UXTB     R11,R11          ;; ZeroExt  R11,R11,#+24,#+24
        STR      R11,[SP, #+0]
        MOV      R3,R10
        UXTB     R3,R3            ;; ZeroExt  R3,R3,#+24,#+24
        MOVS     R2,R4
        UXTB     R2,R2            ;; ZeroExt  R2,R2,#+24,#+24
        LDRH     R1,[SP, #+88]
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        LDRH     R0,[SP, #+84]
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        BL       sub_e
// 1153   Se=sub.s;
        LDR.W    R0,??DataTable49
        LDRH     R0,[R0, #+0]
        STRH     R0,[SP, #+16]
// 1154   Me=sub.m;
        LDR.W    R0,??DataTable49
        LDRH     R0,[R0, #+2]
        STRH     R0,[SP, #+14]
// 1155   Be=sub.b;
        LDR.W    R0,??DataTable49
        LDRH     R0,[R0, #+4]
        STRH     R0,[SP, #+12]
// 1156   
// 1157   sub_e(abs(ec),sc_end,mc_start,bc_start,mc_end);  //变化率
        MOVS     R0,R5
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       abs
        UXTB     R9,R9            ;; ZeroExt  R9,R9,#+24,#+24
        STR      R9,[SP, #+0]
        MOV      R3,R8
        UXTB     R3,R3            ;; ZeroExt  R3,R3,#+24,#+24
        MOVS     R2,R7
        UXTB     R2,R2            ;; ZeroExt  R2,R2,#+24,#+24
        MOVS     R1,R6
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        BL       sub_e
// 1158   Sc=sub.s;
        LDR.W    R0,??DataTable49
        LDRH     R0,[R0, #+0]
        STRH     R0,[SP, #+10]
// 1159   Mc=sub.m;
        LDR.W    R0,??DataTable49
        LDRH     R0,[R0, #+2]
        STRH     R0,[SP, #+2]
// 1160   Bc=sub.b;
        LDR.W    R0,??DataTable49
        LDRH     R0,[R0, #+4]
        STRH     R0,[SP, #+0]
// 1161   
// 1162   
// 1163   if(ec<0){                             //看到的点数在增加
        SXTH     R5,R5            ;; SignExt  R5,R5,#+16,#+16
        CMP      R5,#+0
        BPL.W    ??speed_fuzzy_0
// 1164   
// 1165   if(Se&&Sc)  {vs[is]=min(Se,Sc);is++;}  //建表
        LDRSH    R0,[SP, #+16]
        CMP      R0,#+0
        BEQ.N    ??speed_fuzzy_1
        LDRSH    R0,[SP, #+10]
        CMP      R0,#+0
        BEQ.N    ??speed_fuzzy_1
        LDRSH    R4,[SP, #+4]
        LDRSH    R1,[SP, #+10]
        LDRSH    R0,[SP, #+16]
        BL       min
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        ADD      R1,SP,#+32
        STRH     R0,[R1, R4, LSL #+1]
        LDRH     R0,[SP, #+4]
        ADDS     R0,R0,#+1
        STRH     R0,[SP, #+4]
// 1166   if(Se&&Mc)  {vm[im]=min(Se,Mc);im++;}  //b
??speed_fuzzy_1:
        LDRSH    R0,[SP, #+16]
        CMP      R0,#+0
        BEQ.N    ??speed_fuzzy_2
        LDRSH    R0,[SP, #+2]
        CMP      R0,#+0
        BEQ.N    ??speed_fuzzy_2
        LDRSH    R4,[SP, #+8]
        LDRSH    R1,[SP, #+2]
        LDRSH    R0,[SP, #+16]
        BL       min
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        ADD      R1,SP,#+64
        STRH     R0,[R1, R4, LSL #+1]
        LDRH     R0,[SP, #+8]
        ADDS     R0,R0,#+1
        STRH     R0,[SP, #+8]
// 1167   if(Se&&Bc)  {vb[ib]=min(Se,Bc);ib++;}
??speed_fuzzy_2:
        LDRSH    R0,[SP, #+16]
        CMP      R0,#+0
        BEQ.N    ??speed_fuzzy_3
        LDRSH    R0,[SP, #+0]
        CMP      R0,#+0
        BEQ.N    ??speed_fuzzy_3
        LDRSH    R4,[SP, #+6]
        LDRSH    R1,[SP, #+0]
        LDRSH    R0,[SP, #+16]
        BL       min
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        ADD      R1,SP,#+48
        STRH     R0,[R1, R4, LSL #+1]
        LDRH     R0,[SP, #+6]
        ADDS     R0,R0,#+1
        STRH     R0,[SP, #+6]
// 1168   if(Me&&Sc)  {vm[im]=min(Me,Sc);im++;}
??speed_fuzzy_3:
        LDRSH    R0,[SP, #+14]
        CMP      R0,#+0
        BEQ.N    ??speed_fuzzy_4
        LDRSH    R0,[SP, #+10]
        CMP      R0,#+0
        BEQ.N    ??speed_fuzzy_4
        LDRSH    R4,[SP, #+8]
        LDRSH    R1,[SP, #+10]
        LDRSH    R0,[SP, #+14]
        BL       min
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        ADD      R1,SP,#+64
        STRH     R0,[R1, R4, LSL #+1]
        LDRH     R0,[SP, #+8]
        ADDS     R0,R0,#+1
        STRH     R0,[SP, #+8]
// 1169   if(Me&&Mc)  {vb[ib]=min(Me,Mc);ib++;}
??speed_fuzzy_4:
        LDRSH    R0,[SP, #+14]
        CMP      R0,#+0
        BEQ.N    ??speed_fuzzy_5
        LDRSH    R0,[SP, #+2]
        CMP      R0,#+0
        BEQ.N    ??speed_fuzzy_5
        LDRSH    R4,[SP, #+6]
        LDRSH    R1,[SP, #+2]
        LDRSH    R0,[SP, #+14]
        BL       min
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        ADD      R1,SP,#+48
        STRH     R0,[R1, R4, LSL #+1]
        LDRH     R0,[SP, #+6]
        ADDS     R0,R0,#+1
        STRH     R0,[SP, #+6]
// 1170   if(Me&&Bc)  {vb[ib]=min(Me,Bc);ib++;}
??speed_fuzzy_5:
        LDRSH    R0,[SP, #+14]
        CMP      R0,#+0
        BEQ.N    ??speed_fuzzy_6
        LDRSH    R0,[SP, #+0]
        CMP      R0,#+0
        BEQ.N    ??speed_fuzzy_6
        LDRSH    R4,[SP, #+6]
        LDRSH    R1,[SP, #+0]
        LDRSH    R0,[SP, #+14]
        BL       min
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        ADD      R1,SP,#+48
        STRH     R0,[R1, R4, LSL #+1]
        LDRH     R0,[SP, #+6]
        ADDS     R0,R0,#+1
        STRH     R0,[SP, #+6]
// 1171   if(Be&&Sc)  {vb[ib]=min(Be,Sc);ib++;}
??speed_fuzzy_6:
        LDRSH    R0,[SP, #+12]
        CMP      R0,#+0
        BEQ.N    ??speed_fuzzy_7
        LDRSH    R0,[SP, #+10]
        CMP      R0,#+0
        BEQ.N    ??speed_fuzzy_7
        LDRSH    R4,[SP, #+6]
        LDRSH    R1,[SP, #+10]
        LDRSH    R0,[SP, #+12]
        BL       min
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        ADD      R1,SP,#+48
        STRH     R0,[R1, R4, LSL #+1]
        LDRH     R0,[SP, #+6]
        ADDS     R0,R0,#+1
        STRH     R0,[SP, #+6]
// 1172   if(Be&&Mc)  {vb[ib]=min(Be,Mc);ib++;} 
??speed_fuzzy_7:
        LDRSH    R0,[SP, #+12]
        CMP      R0,#+0
        BEQ.N    ??speed_fuzzy_8
        LDRSH    R0,[SP, #+2]
        CMP      R0,#+0
        BEQ.N    ??speed_fuzzy_8
        LDRSH    R4,[SP, #+6]
        LDRSH    R1,[SP, #+2]
        LDRSH    R0,[SP, #+12]
        BL       min
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        ADD      R1,SP,#+48
        STRH     R0,[R1, R4, LSL #+1]
        LDRH     R0,[SP, #+6]
        ADDS     R0,R0,#+1
        STRH     R0,[SP, #+6]
// 1173   if(Be&&Bc)  {vb[ib]=min(Be,Bc);ib++;} 
??speed_fuzzy_8:
        LDRSH    R0,[SP, #+12]
        CMP      R0,#+0
        BEQ.N    ??speed_fuzzy_9
        LDRSH    R0,[SP, #+0]
        CMP      R0,#+0
        BEQ.N    ??speed_fuzzy_9
        LDRSH    R4,[SP, #+6]
        LDRSH    R1,[SP, #+0]
        LDRSH    R0,[SP, #+12]
        BL       min
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        ADD      R1,SP,#+48
        STRH     R0,[R1, R4, LSL #+1]
        LDRH     R0,[SP, #+6]
        ADDS     R0,R0,#+1
        STRH     R0,[SP, #+6]
// 1174     if(vs[0]){
??speed_fuzzy_9:
        LDRSH    R0,[SP, #+32]
        CMP      R0,#+0
        BEQ.N    ??speed_fuzzy_10
// 1175     f=&vs[0];
        ADD      R0,SP,#+32
// 1176     Ks=subjection_k(f,is);
        LDRSH    R1,[SP, #+4]
        BL       subjection_k
        STR      R0,[SP, #+28]
// 1177     }
// 1178     if(vm[0]){
??speed_fuzzy_10:
        LDRSH    R0,[SP, #+64]
        CMP      R0,#+0
        BEQ.N    ??speed_fuzzy_11
// 1179     f=&vm[0];
        ADD      R0,SP,#+64
// 1180     Km=subjection_k(f,im);
        LDRSH    R1,[SP, #+8]
        BL       subjection_k
        STR      R0,[SP, #+24]
// 1181     }
// 1182     if(vb[0]){
??speed_fuzzy_11:
        LDRSH    R0,[SP, #+48]
        CMP      R0,#+0
        BEQ.N    ??speed_fuzzy_12
// 1183     f=&vb[0];
        ADD      R0,SP,#+48
// 1184     Kb=subjection_k(f,ib);
        LDRSH    R1,[SP, #+6]
        BL       subjection_k
        STR      R0,[SP, #+20]
// 1185     }
// 1186     
// 1187     exspeed=(speed_b*Kb+speed_m*Km+speed_s*Ks)/(Kb+Km+Ks);
??speed_fuzzy_12:
        LDR.W    R0,??DataTable51
        LDRSH    R0,[R0, #+0]
        LDR      R1,[SP, #+20]
        LDR.W    R2,??DataTable51_1
        LDRSH    R2,[R2, #+0]
        LDR      R3,[SP, #+24]
        MULS     R2,R3,R2
        MLA      R0,R1,R0,R2
        LDR.W    R1,??DataTable51_2
        LDRSH    R1,[R1, #+0]
        LDR      R2,[SP, #+28]
        MLA      R0,R2,R1,R0
        LDR      R1,[SP, #+20]
        LDR      R2,[SP, #+24]
        ADDS     R1,R2,R1
        LDR      R2,[SP, #+28]
        ADDS     R1,R2,R1
        SDIV     R0,R0,R1
        STRH     R0,[SP, #+0]
        B.N      ??speed_fuzzy_13
// 1188   }else{
// 1189    
// 1190   
// 1191   if(Se&&Sc)  {vs[is]=min(Se,Sc);is++;}  //建表 4.10
??speed_fuzzy_0:
        LDRSH    R0,[SP, #+16]
        CMP      R0,#+0
        BEQ.N    ??speed_fuzzy_14
        LDRSH    R0,[SP, #+10]
        CMP      R0,#+0
        BEQ.N    ??speed_fuzzy_14
        LDRSH    R4,[SP, #+4]
        LDRSH    R1,[SP, #+10]
        LDRSH    R0,[SP, #+16]
        BL       min
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        ADD      R1,SP,#+32
        STRH     R0,[R1, R4, LSL #+1]
        LDRH     R0,[SP, #+4]
        ADDS     R0,R0,#+1
        STRH     R0,[SP, #+4]
// 1192   if(Se&&Mc)  {vs[is]=min(Se,Mc);is++;}
??speed_fuzzy_14:
        LDRSH    R0,[SP, #+16]
        CMP      R0,#+0
        BEQ.N    ??speed_fuzzy_15
        LDRSH    R0,[SP, #+2]
        CMP      R0,#+0
        BEQ.N    ??speed_fuzzy_15
        LDRSH    R4,[SP, #+4]
        LDRSH    R1,[SP, #+2]
        LDRSH    R0,[SP, #+16]
        BL       min
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        ADD      R1,SP,#+32
        STRH     R0,[R1, R4, LSL #+1]
        LDRH     R0,[SP, #+4]
        ADDS     R0,R0,#+1
        STRH     R0,[SP, #+4]
// 1193   if(Se&&Bc)  {vs[is]=min(Se,Bc);is++;}
??speed_fuzzy_15:
        LDRSH    R0,[SP, #+16]
        CMP      R0,#+0
        BEQ.N    ??speed_fuzzy_16
        LDRSH    R0,[SP, #+0]
        CMP      R0,#+0
        BEQ.N    ??speed_fuzzy_16
        LDRSH    R4,[SP, #+4]
        LDRSH    R1,[SP, #+0]
        LDRSH    R0,[SP, #+16]
        BL       min
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        ADD      R1,SP,#+32
        STRH     R0,[R1, R4, LSL #+1]
        LDRH     R0,[SP, #+4]
        ADDS     R0,R0,#+1
        STRH     R0,[SP, #+4]
// 1194   if(Me&&Sc)  {vm[im]=min(Me,Sc);im++;}
??speed_fuzzy_16:
        LDRSH    R0,[SP, #+14]
        CMP      R0,#+0
        BEQ.N    ??speed_fuzzy_17
        LDRSH    R0,[SP, #+10]
        CMP      R0,#+0
        BEQ.N    ??speed_fuzzy_17
        LDRSH    R4,[SP, #+8]
        LDRSH    R1,[SP, #+10]
        LDRSH    R0,[SP, #+14]
        BL       min
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        ADD      R1,SP,#+64
        STRH     R0,[R1, R4, LSL #+1]
        LDRH     R0,[SP, #+8]
        ADDS     R0,R0,#+1
        STRH     R0,[SP, #+8]
// 1195   if(Me&&Mc)  {vs[is]=min(Me,Mc);is++;}
??speed_fuzzy_17:
        LDRSH    R0,[SP, #+14]
        CMP      R0,#+0
        BEQ.N    ??speed_fuzzy_18
        LDRSH    R0,[SP, #+2]
        CMP      R0,#+0
        BEQ.N    ??speed_fuzzy_18
        LDRSH    R4,[SP, #+4]
        LDRSH    R1,[SP, #+2]
        LDRSH    R0,[SP, #+14]
        BL       min
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        ADD      R1,SP,#+32
        STRH     R0,[R1, R4, LSL #+1]
        LDRH     R0,[SP, #+4]
        ADDS     R0,R0,#+1
        STRH     R0,[SP, #+4]
// 1196   if(Me&&Bc)  {vs[is]=min(Me,Bc);is++;}
??speed_fuzzy_18:
        LDRSH    R0,[SP, #+14]
        CMP      R0,#+0
        BEQ.N    ??speed_fuzzy_19
        LDRSH    R0,[SP, #+0]
        CMP      R0,#+0
        BEQ.N    ??speed_fuzzy_19
        LDRSH    R4,[SP, #+4]
        LDRSH    R1,[SP, #+0]
        LDRSH    R0,[SP, #+14]
        BL       min
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        ADD      R1,SP,#+32
        STRH     R0,[R1, R4, LSL #+1]
        LDRH     R0,[SP, #+4]
        ADDS     R0,R0,#+1
        STRH     R0,[SP, #+4]
// 1197   if(Be&&Sc)  {vb[ib]=min(Be,Sc);ib++;}
??speed_fuzzy_19:
        LDRSH    R0,[SP, #+12]
        CMP      R0,#+0
        BEQ.N    ??speed_fuzzy_20
        LDRSH    R0,[SP, #+10]
        CMP      R0,#+0
        BEQ.N    ??speed_fuzzy_20
        LDRSH    R4,[SP, #+6]
        LDRSH    R1,[SP, #+10]
        LDRSH    R0,[SP, #+12]
        BL       min
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        ADD      R1,SP,#+48
        STRH     R0,[R1, R4, LSL #+1]
        LDRH     R0,[SP, #+6]
        ADDS     R0,R0,#+1
        STRH     R0,[SP, #+6]
// 1198   if(Be&&Mc)  {vm[im]=min(Be,Mc);im++;}
??speed_fuzzy_20:
        LDRSH    R0,[SP, #+12]
        CMP      R0,#+0
        BEQ.N    ??speed_fuzzy_21
        LDRSH    R0,[SP, #+2]
        CMP      R0,#+0
        BEQ.N    ??speed_fuzzy_21
        LDRSH    R4,[SP, #+8]
        LDRSH    R1,[SP, #+2]
        LDRSH    R0,[SP, #+12]
        BL       min
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        ADD      R1,SP,#+64
        STRH     R0,[R1, R4, LSL #+1]
        LDRH     R0,[SP, #+8]
        ADDS     R0,R0,#+1
        STRH     R0,[SP, #+8]
// 1199   if(Be&&Bc)  {vs[is]=min(Be,Bc);is++;}
??speed_fuzzy_21:
        LDRSH    R0,[SP, #+12]
        CMP      R0,#+0
        BEQ.N    ??speed_fuzzy_22
        LDRSH    R0,[SP, #+0]
        CMP      R0,#+0
        BEQ.N    ??speed_fuzzy_22
        LDRSH    R4,[SP, #+4]
        LDRSH    R1,[SP, #+0]
        LDRSH    R0,[SP, #+12]
        BL       min
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        ADD      R1,SP,#+32
        STRH     R0,[R1, R4, LSL #+1]
        LDRH     R0,[SP, #+4]
        ADDS     R0,R0,#+1
        STRH     R0,[SP, #+4]
// 1200 
// 1201     if(vs[0]){
??speed_fuzzy_22:
        LDRSH    R0,[SP, #+32]
        CMP      R0,#+0
        BEQ.N    ??speed_fuzzy_23
// 1202     f=&vs[0];
        ADD      R0,SP,#+32
// 1203     Ks=subjection_k(f,is);
        LDRSH    R1,[SP, #+4]
        BL       subjection_k
        STR      R0,[SP, #+28]
// 1204     }
// 1205     if(vm[0]){
??speed_fuzzy_23:
        LDRSH    R0,[SP, #+64]
        CMP      R0,#+0
        BEQ.N    ??speed_fuzzy_24
// 1206     f=&vm[0];
        ADD      R0,SP,#+64
// 1207     Km=subjection_k(f,im);
        LDRSH    R1,[SP, #+8]
        BL       subjection_k
        STR      R0,[SP, #+24]
// 1208     }
// 1209     if(vb[0]){
??speed_fuzzy_24:
        LDRSH    R0,[SP, #+48]
        CMP      R0,#+0
        BEQ.N    ??speed_fuzzy_25
// 1210     f=&vb[0];
        ADD      R0,SP,#+48
// 1211     Kb=subjection_k(f,ib);
        LDRSH    R1,[SP, #+6]
        BL       subjection_k
        STR      R0,[SP, #+20]
// 1212     }
// 1213     
// 1214 
// 1215     exspeed=(speed_b*Kb+speed_m*Km+speed_s*Ks)/(Kb+Km+Ks);
??speed_fuzzy_25:
        LDR.W    R0,??DataTable51
        LDRSH    R0,[R0, #+0]
        LDR      R1,[SP, #+20]
        LDR.W    R2,??DataTable51_1
        LDRSH    R2,[R2, #+0]
        LDR      R3,[SP, #+24]
        MULS     R2,R3,R2
        MLA      R0,R1,R0,R2
        LDR.W    R1,??DataTable51_2
        LDRSH    R1,[R1, #+0]
        LDR      R2,[SP, #+28]
        MLA      R0,R2,R1,R0
        LDR      R1,[SP, #+20]
        LDR      R2,[SP, #+24]
        ADDS     R1,R2,R1
        LDR      R2,[SP, #+28]
        ADDS     R1,R2,R1
        SDIV     R0,R0,R1
        STRH     R0,[SP, #+0]
// 1216   }
// 1217   
// 1218   return exspeed;
??speed_fuzzy_13:
        LDRSH    R0,[SP, #+0]
        ADD      SP,SP,#+92
        POP      {R4-R11,PC}      ;; return
// 1219 }

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable47:
        DC32     `?<Constant {1, 2, 3, 3, 4}>`

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable47_1:
        DC32     position
// 1220 
// 1221 
// 1222 /*****找到莫连续几行的中心线的最大最小值******/

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1223 int16 line_min(uint8 *head,uint8 *tail){
line_min:
        PUSH     {R4}
// 1224   uint8 *p=NULL;
        MOVS     R2,#+0
// 1225   point.x=*head;
        LDR.W    R3,??DataTable52
        LDRB     R4,[R0, #+0]
        STRB     R4,[R3, #+0]
// 1226   point.y=0;
        LDR.W    R3,??DataTable52
        MOVS     R4,#+0
        STRB     R4,[R3, #+1]
// 1227   for(p=head+1;p<=tail;p++){
        ADDS     R3,R0,#+1
        MOVS     R2,R3
        B.N      ??line_min_0
// 1228     if(*p<point.x){
??line_min_1:
        LDRB     R3,[R2, #+0]
        LDR.W    R4,??DataTable52
        LDRB     R4,[R4, #+0]
        CMP      R3,R4
        BCS.N    ??line_min_2
// 1229       point.x=*p;
        LDR.W    R3,??DataTable52
        LDRB     R4,[R2, #+0]
        STRB     R4,[R3, #+0]
// 1230 
// 1231 
// 1232 
// 1233        point.y=p-head;
        SUBS     R3,R2,R0
        LDR.W    R4,??DataTable52
        STRB     R3,[R4, #+1]
// 1234     }  
// 1235   }
??line_min_2:
        ADDS     R2,R2,#+1
??line_min_0:
        CMP      R1,R2
        BCS.N    ??line_min_1
// 1236   return point.x;
        LDR.W    R0,??DataTable52
        LDRB     R0,[R0, #+0]
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        POP      {R4}
        BX       LR               ;; return
// 1237 }                                                                       
// 1238     
// 1239   
// 1240 

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1241 int16 line_max(uint8 *head,uint8 *tail){
line_max:
        PUSH     {R4}
// 1242   uint8 *p=NULL;
        MOVS     R2,#+0
// 1243   
// 1244   point.x=*head;
        LDR.W    R3,??DataTable52
        LDRB     R4,[R0, #+0]
        STRB     R4,[R3, #+0]
// 1245   point.y=0;
        LDR.W    R3,??DataTable52
        MOVS     R4,#+0
        STRB     R4,[R3, #+1]
// 1246   for(p=head+1;p<=tail;p++){
        ADDS     R3,R0,#+1
        MOVS     R2,R3
        B.N      ??line_max_0
// 1247     if(*p>point.x){
??line_max_1:
        LDR.W    R3,??DataTable52
        LDRB     R3,[R3, #+0]
        LDRB     R4,[R2, #+0]
        CMP      R3,R4
        BCS.N    ??line_max_2
// 1248       point.x=*p;
        LDR.W    R3,??DataTable52
        LDRB     R4,[R2, #+0]
        STRB     R4,[R3, #+0]
// 1249       point.y=p-head;
        SUBS     R3,R2,R0
        LDR.W    R4,??DataTable52
        STRB     R3,[R4, #+1]
// 1250     }
// 1251   }
??line_max_2:
        ADDS     R2,R2,#+1
??line_max_0:
        CMP      R1,R2
        BCS.N    ??line_max_1
// 1252   return point.x;
        LDR.W    R0,??DataTable52
        LDRB     R0,[R0, #+0]
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        POP      {R4}
        BX       LR               ;; return
// 1253 }

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable49:
        DC32     `sub`
// 1254 

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1255 int16 line(int16 value,int16 lenth){
line:
        PUSH     {R4,R5}
// 1256   static int16 line[30]={0},i=0;               //30
// 1257   int16 sum=0,j=0,k=0;
        MOVS     R2,#+0
        MOVS     R3,#+0
        MOVS     R4,#+0
// 1258   
// 1259   if(i==30) i=0;                                   
        LDR.W    R5,??DataTable51_3
        LDRSH    R5,[R5, #+0]
        CMP      R5,#+30
        BNE.N    ??line_1
        LDR.W    R3,??DataTable51_3
        MOVS     R4,#+0
        STRH     R4,[R3, #+0]
// 1260   line[i]=value;
??line_1:
        LDR.W    R3,??DataTable51_3
        LDRSH    R3,[R3, #+0]
        LDR.W    R4,??DataTable52_1
        STRH     R0,[R4, R3, LSL #+1]
// 1261   k=i++;
        LDR.W    R0,??DataTable51_3
        LDRSH    R4,[R0, #+0]
        LDR.W    R0,??DataTable51_3
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.W    R3,??DataTable51_3
        STRH     R0,[R3, #+0]
// 1262   for(j=0;j<lenth;j++){
        MOVS     R3,#+0
        B.N      ??line_2
// 1263     if(k<0) k=29;
??line_3:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        CMP      R4,#+0
        BPL.N    ??line_4
        MOVS     R4,#+29
// 1264     sum+=line[k];
??line_4:
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        LDR.W    R0,??DataTable52_1
        LDRH     R0,[R0, R4, LSL #+1]
        ADDS     R2,R0,R2
// 1265     k--;
        SUBS     R4,R4,#+1
// 1266   }
        ADDS     R3,R3,#+1
??line_2:
        SXTH     R3,R3            ;; SignExt  R3,R3,#+16,#+16
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        CMP      R3,R1
        BLT.N    ??line_3
// 1267   return sum;
        MOVS     R0,R2
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        POP      {R4,R5}
        BX       LR               ;; return
// 1268 }

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable50:
        DC32     Motor

        SECTION `.bss`:DATA:REORDER:NOROOT(2)
??line:
        DS8 60

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
??i:
        DS8 2
// 1269 
// 1270 
// 1271 

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1272 int16 stages_P(int16 value,int16 parameter)
// 1273 {
// 1274   //if(!value) return unclear;
// 1275 
// 1276   if(value<=parameter)
stages_P:
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        CMP      R1,R0
        BLT.N    ??stages_P_0
// 1277   {
// 1278     if(value>=(parameter-6)){NULL;return(0);}
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        SUBS     R2,R1,#+6
        CMP      R0,R2
        BLT.N    ??stages_P_1
        MOVS     R0,#+0
        B.N      ??stages_P_2
// 1279     else if(value>=(parameter-12)){NULL;return(-1);}  
??stages_P_1:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        SUBS     R2,R1,#+12
        CMP      R0,R2
        BLT.N    ??stages_P_3
        MOVS     R0,#-1
        B.N      ??stages_P_2
// 1280     else if(value>=(parameter-18)){NULL;return(-2);}    
??stages_P_3:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        SUBS     R2,R1,#+18
        CMP      R0,R2
        BLT.N    ??stages_P_4
        MVNS     R0,#+1
        B.N      ??stages_P_2
// 1281     else if(value>=(parameter-24)){NULL;return(-3);}
??stages_P_4:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        SUBS     R2,R1,#+24
        CMP      R0,R2
        BLT.N    ??stages_P_5
        MVNS     R0,#+2
        B.N      ??stages_P_2
// 1282     else if(value>=(parameter-30)){NULL;return(-4);}
??stages_P_5:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        SUBS     R2,R1,#+30
        CMP      R0,R2
        BLT.N    ??stages_P_6
        MVNS     R0,#+3
        B.N      ??stages_P_2
// 1283     else if(value>=(parameter-36)){NULL;return(-5);}
??stages_P_6:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        SUBS     R1,R1,#+36
        CMP      R0,R1
        BLT.N    ??stages_P_7
        MVNS     R0,#+4
        B.N      ??stages_P_2
// 1284     else {NULL;return(-6);}
??stages_P_7:
        MVNS     R0,#+5
        B.N      ??stages_P_2
// 1285   }
// 1286   else
// 1287   {
// 1288     if(value<=(parameter+6)){NULL;return(0);}
??stages_P_0:
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        ADDS     R2,R1,#+6
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        CMP      R2,R0
        BLT.N    ??stages_P_8
        MOVS     R0,#+0
        B.N      ??stages_P_2
// 1289     else if(value<=(parameter+12)){NULL;return(1);}  
??stages_P_8:
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        ADDS     R2,R1,#+12
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        CMP      R2,R0
        BLT.N    ??stages_P_9
        MOVS     R0,#+1
        B.N      ??stages_P_2
// 1290     else if(value<=(parameter+18)){NULL;return(2);}    
??stages_P_9:
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        ADDS     R2,R1,#+18
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        CMP      R2,R0
        BLT.N    ??stages_P_10
        MOVS     R0,#+2
        B.N      ??stages_P_2
// 1291     else if(value<=(parameter+24)){NULL;return(3);}
??stages_P_10:
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        ADDS     R2,R1,#+24
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        CMP      R2,R0
        BLT.N    ??stages_P_11
        MOVS     R0,#+3
        B.N      ??stages_P_2
// 1292     else if(value<=(parameter+30)){NULL;return(4);}
??stages_P_11:
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        ADDS     R2,R1,#+30
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        CMP      R2,R0
        BLT.N    ??stages_P_12
        MOVS     R0,#+4
        B.N      ??stages_P_2
// 1293     else if(value<=(parameter+36)){NULL;return(5);}
??stages_P_12:
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        ADDS     R1,R1,#+36
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        CMP      R1,R0
        BLT.N    ??stages_P_13
        MOVS     R0,#+5
        B.N      ??stages_P_2
// 1294     else {NULL;return(6);}
??stages_P_13:
        MOVS     R0,#+6
??stages_P_2:
        BX       LR               ;; return
// 1295   }
// 1296 }
// 1297 

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1298 int16 stages_low_P(int16 value,int16 parameter)
// 1299 {
// 1300   //if(!value) return unclear;
// 1301   if(value<=parameter)
stages_low_P:
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        CMP      R1,R0
        BLT.N    ??stages_low_P_0
// 1302   {
// 1303     if(value>=(parameter-5)){NULL;return(0);}
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        SUBS     R2,R1,#+5
        CMP      R0,R2
        BLT.N    ??stages_low_P_1
        MOVS     R0,#+0
        B.N      ??stages_low_P_2
// 1304     else if(value>=(parameter-13)){NULL;return(-1);}  
??stages_low_P_1:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        SUBS     R2,R1,#+13
        CMP      R0,R2
        BLT.N    ??stages_low_P_3
        MOVS     R0,#-1
        B.N      ??stages_low_P_2
// 1305     else if(value>=(parameter-23)){NULL;return(-2);}    
??stages_low_P_3:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        SUBS     R1,R1,#+23
        CMP      R0,R1
        BLT.N    ??stages_low_P_4
        MVNS     R0,#+1
        B.N      ??stages_low_P_2
// 1306     else {NULL;return(-3);}
??stages_low_P_4:
        MVNS     R0,#+2
        B.N      ??stages_low_P_2
// 1307   }
// 1308   else
// 1309   {
// 1310     if(value<=(parameter+5)){NULL;return(0);}
??stages_low_P_0:
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        ADDS     R2,R1,#+5
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        CMP      R2,R0
        BLT.N    ??stages_low_P_5
        MOVS     R0,#+0
        B.N      ??stages_low_P_2
// 1311     else if(value<=(parameter+13)){NULL;return(1);}  
??stages_low_P_5:
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        ADDS     R2,R1,#+13
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        CMP      R2,R0
        BLT.N    ??stages_low_P_6
        MOVS     R0,#+1
        B.N      ??stages_low_P_2
// 1312     else if(value<=(parameter+23)){NULL;return(2);}    
??stages_low_P_6:
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        ADDS     R1,R1,#+23
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        CMP      R1,R0
        BLT.N    ??stages_low_P_7
        MOVS     R0,#+2
        B.N      ??stages_low_P_2
// 1313     else {NULL;return(3);}
??stages_low_P_7:
        MOVS     R0,#+3
??stages_low_P_2:
        BX       LR               ;; return
// 1314   }
// 1315 }
// 1316 

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1317 int16 stages_D(int16 value)
// 1318 {
// 1319   if(value<=0)
stages_D:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        CMP      R0,#+1
        BGE.N    ??stages_D_0
// 1320   {
// 1321     if(value>=-6){NULL;return(0);}
        MVNS     R1,#+5
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        CMP      R0,R1
        BLT.N    ??stages_D_1
        MOVS     R0,#+0
        B.N      ??stages_D_2
// 1322     else if(value>=-12){NULL;return(-1);}  
??stages_D_1:
        MVNS     R1,#+11
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        CMP      R0,R1
        BLT.N    ??stages_D_3
        MOVS     R0,#-1
        B.N      ??stages_D_2
// 1323     else if(value>=-18){NULL;return(-2);}    
??stages_D_3:
        MVNS     R1,#+17
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        CMP      R0,R1
        BLT.N    ??stages_D_4
        MVNS     R0,#+1
        B.N      ??stages_D_2
// 1324     else if(value>=-24){NULL;return(-3);}
??stages_D_4:
        MVNS     R1,#+23
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        CMP      R0,R1
        BLT.N    ??stages_D_5
        MVNS     R0,#+2
        B.N      ??stages_D_2
// 1325     else if(value>=-30){NULL;return(-4);}
??stages_D_5:
        MVNS     R1,#+29
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        CMP      R0,R1
        BLT.N    ??stages_D_6
        MVNS     R0,#+3
        B.N      ??stages_D_2
// 1326     else if(value>=-36){NULL;return(-5);}
??stages_D_6:
        MVNS     R1,#+35
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        CMP      R0,R1
        BLT.N    ??stages_D_7
        MVNS     R0,#+4
        B.N      ??stages_D_2
// 1327     else {NULL;return(-6);}
??stages_D_7:
        MVNS     R0,#+5
        B.N      ??stages_D_2
// 1328   }
// 1329   else
// 1330   {
// 1331     if(value<=6){NULL;return(0);}
??stages_D_0:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        CMP      R0,#+7
        BGE.N    ??stages_D_8
        MOVS     R0,#+0
        B.N      ??stages_D_2
// 1332     else if(value<=12){NULL;return(1);}  
??stages_D_8:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        CMP      R0,#+13
        BGE.N    ??stages_D_9
        MOVS     R0,#+1
        B.N      ??stages_D_2
// 1333     else if(value<=18){NULL;return(2);}    
??stages_D_9:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        CMP      R0,#+19
        BGE.N    ??stages_D_10
        MOVS     R0,#+2
        B.N      ??stages_D_2
// 1334     else if(value<=24){NULL;return(3);}
??stages_D_10:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        CMP      R0,#+25
        BGE.N    ??stages_D_11
        MOVS     R0,#+3
        B.N      ??stages_D_2
// 1335     else if(value<=30){NULL;return(4);}
??stages_D_11:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        CMP      R0,#+31
        BGE.N    ??stages_D_12
        MOVS     R0,#+4
        B.N      ??stages_D_2
// 1336     else if(value<=36){NULL;return(5);}
??stages_D_12:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        CMP      R0,#+37
        BGE.N    ??stages_D_13
        MOVS     R0,#+5
        B.N      ??stages_D_2
// 1337     else {NULL;return(6);}
??stages_D_13:
        MOVS     R0,#+6
??stages_D_2:
        BX       LR               ;; return
// 1338   }
// 1339 }
// 1340 /******************起始线二值化***************/

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1341 void binaryzation2(){
// 1342  uint8 i=0,j=0;
binaryzation2:
        MOVS     R0,#+0
        MOVS     R1,#+0
// 1343   for(i=0;i<IMG_ROWS;i++)
        MOVS     R2,#+0
        MOVS     R0,R2
        B.N      ??binaryzation2_0
??binaryzation2_1:
        ADDS     R0,R0,#+1
??binaryzation2_0:
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        CMP      R0,#+55
        BCS.W    ??binaryzation2_2
// 1344   {
// 1345     if(i<8)
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        CMP      R0,#+8
        BCS.W    ??binaryzation2_3
// 1346     {
// 1347       for(j=0;j<IMG_COLS;j++)
        MOVS     R1,#+0
        B.N      ??binaryzation2_4
// 1348       {
// 1349         if(j<50)
// 1350         {
// 1351           if(ImageData[i][j]<black2_d1)ImageData2[i][j]=0;
// 1352           else ImageData2[i][j]=255;
// 1353           if(ImageData[i][j]<black_d1)ImageData[i][j]=0;
// 1354           else ImageData[i][j]=255;
// 1355           
// 1356         }
// 1357         else if(j<110)
// 1358         {
// 1359           if(ImageData[i][j]<black2_d2)ImageData2[i][j]=0;
// 1360           else ImageData2[i][j]=255;
// 1361           if(ImageData[i][j]<black_d2)ImageData[i][j]=0;
// 1362           else ImageData[i][j]=255;
// 1363           
// 1364         }
// 1365         else
// 1366         {
// 1367            if(ImageData[i][j]<black2_d3)ImageData2[i][j]=0;
// 1368           else ImageData2[i][j]=255;
// 1369           if(ImageData[i][j]<black_d3)ImageData[i][j]=0;
// 1370           else ImageData[i][j]=255;
??binaryzation2_5:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        MOVS     R3,#+255
        STRB     R3,[R1, R2]
??binaryzation2_6:
        ADDS     R1,R1,#+1
??binaryzation2_4:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        CMP      R1,#+160
        BCS.N    ??binaryzation2_1
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        CMP      R1,#+50
        BCS.N    ??binaryzation2_7
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        LDRB     R2,[R1, R2]
        LDR.W    R3,??DataTable52_3
        LDRB     R3,[R3, #+0]
        CMP      R2,R3
        BCS.N    ??binaryzation2_8
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_4
        MLA      R2,R2,R0,R3
        MOVS     R3,#+0
        STRB     R3,[R1, R2]
        B.N      ??binaryzation2_9
??binaryzation2_8:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_4
        MLA      R2,R2,R0,R3
        MOVS     R3,#+255
        STRB     R3,[R1, R2]
??binaryzation2_9:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        LDRB     R2,[R1, R2]
        LDR.W    R3,??DataTable52_5
        LDRB     R3,[R3, #+0]
        CMP      R2,R3
        BCS.N    ??binaryzation2_10
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        MOVS     R3,#+0
        STRB     R3,[R1, R2]
        B.N      ??binaryzation2_6
??binaryzation2_10:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        MOVS     R3,#+255
        STRB     R3,[R1, R2]
        B.N      ??binaryzation2_6
??binaryzation2_7:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        CMP      R1,#+110
        BCS.N    ??binaryzation2_11
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        LDRB     R2,[R1, R2]
        LDR.W    R3,??DataTable52_6
        LDRB     R3,[R3, #+0]
        CMP      R2,R3
        BCS.N    ??binaryzation2_12
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_4
        MLA      R2,R2,R0,R3
        MOVS     R3,#+0
        STRB     R3,[R1, R2]
        B.N      ??binaryzation2_13
??binaryzation2_12:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_4
        MLA      R2,R2,R0,R3
        MOVS     R3,#+255
        STRB     R3,[R1, R2]
??binaryzation2_13:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        LDRB     R2,[R1, R2]
        LDR.W    R3,??DataTable52_7
        LDRB     R3,[R3, #+0]
        CMP      R2,R3
        BCS.N    ??binaryzation2_14
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        MOVS     R3,#+0
        STRB     R3,[R1, R2]
        B.N      ??binaryzation2_6
??binaryzation2_14:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        MOVS     R3,#+255
        STRB     R3,[R1, R2]
        B.N      ??binaryzation2_6
??binaryzation2_11:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        LDRB     R2,[R1, R2]
        LDR.W    R3,??DataTable52_8
        LDRB     R3,[R3, #+0]
        CMP      R2,R3
        BCS.N    ??binaryzation2_15
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_4
        MLA      R2,R2,R0,R3
        MOVS     R3,#+0
        STRB     R3,[R1, R2]
        B.N      ??binaryzation2_16
??binaryzation2_15:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_4
        MLA      R2,R2,R0,R3
        MOVS     R3,#+255
        STRB     R3,[R1, R2]
??binaryzation2_16:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        LDRB     R2,[R1, R2]
        LDR.W    R3,??DataTable52_9
        LDRB     R3,[R3, #+0]
        CMP      R2,R3
        BCS.W    ??binaryzation2_5
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        MOVS     R3,#+0
        STRB     R3,[R1, R2]
        B.N      ??binaryzation2_6
// 1371          
// 1372         }
// 1373       }
// 1374     }
// 1375     else if(i<20)
??binaryzation2_3:
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        CMP      R0,#+20
        BCS.W    ??binaryzation2_17
// 1376     {
// 1377       for(j=0;j<IMG_COLS;j++)
        MOVS     R1,#+0
        B.N      ??binaryzation2_18
// 1378       {
// 1379         if(j<50)
// 1380         {
// 1381           if(ImageData[i][j]<black2_c1)ImageData2[i][j]=0;
// 1382           else ImageData2[i][j]=255;
// 1383           if(ImageData[i][j]<black_c1)ImageData[i][j]=0;
// 1384           else ImageData[i][j]=255; 
// 1385         }
// 1386         else if(j<110)
// 1387         {
// 1388           if(ImageData[i][j]<black2_c2)ImageData2[i][j]=0;
// 1389           else ImageData2[i][j]=255;
// 1390           if(ImageData[i][j]<black_c2)ImageData[i][j]=0;
// 1391           else ImageData[i][j]=255;
// 1392         }
// 1393         else
// 1394         {
// 1395            if(ImageData[i][j]<black2_c3)ImageData2[i][j]=0;
// 1396           else ImageData2[i][j]=255;
// 1397           if(ImageData[i][j]<black_c3)ImageData[i][j]=0;
// 1398           else ImageData[i][j]=255;
??binaryzation2_19:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        MOVS     R3,#+255
        STRB     R3,[R1, R2]
??binaryzation2_20:
        ADDS     R1,R1,#+1
??binaryzation2_18:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        CMP      R1,#+160
        BCS.W    ??binaryzation2_1
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        CMP      R1,#+50
        BCS.N    ??binaryzation2_21
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        LDRB     R2,[R1, R2]
        LDR.W    R3,??DataTable52_10
        LDRB     R3,[R3, #+0]
        CMP      R2,R3
        BCS.N    ??binaryzation2_22
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_4
        MLA      R2,R2,R0,R3
        MOVS     R3,#+0
        STRB     R3,[R1, R2]
        B.N      ??binaryzation2_23
??binaryzation2_22:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_4
        MLA      R2,R2,R0,R3
        MOVS     R3,#+255
        STRB     R3,[R1, R2]
??binaryzation2_23:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        LDRB     R2,[R1, R2]
        LDR.W    R3,??DataTable53
        LDRB     R3,[R3, #+0]
        CMP      R2,R3
        BCS.N    ??binaryzation2_24
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        MOVS     R3,#+0
        STRB     R3,[R1, R2]
        B.N      ??binaryzation2_20
??binaryzation2_24:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        MOVS     R3,#+255
        STRB     R3,[R1, R2]
        B.N      ??binaryzation2_20
??binaryzation2_21:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        CMP      R1,#+110
        BCS.N    ??binaryzation2_25
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        LDRB     R2,[R1, R2]
        LDR.W    R3,??DataTable53_1
        LDRB     R3,[R3, #+0]
        CMP      R2,R3
        BCS.N    ??binaryzation2_26
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_4
        MLA      R2,R2,R0,R3
        MOVS     R3,#+0
        STRB     R3,[R1, R2]
        B.N      ??binaryzation2_27
??binaryzation2_26:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_4
        MLA      R2,R2,R0,R3
        MOVS     R3,#+255
        STRB     R3,[R1, R2]
??binaryzation2_27:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        LDRB     R2,[R1, R2]
        LDR.W    R3,??DataTable53_2
        LDRB     R3,[R3, #+0]
        CMP      R2,R3
        BCS.N    ??binaryzation2_28
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        MOVS     R3,#+0
        STRB     R3,[R1, R2]
        B.N      ??binaryzation2_20
??binaryzation2_28:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        MOVS     R3,#+255
        STRB     R3,[R1, R2]
        B.N      ??binaryzation2_20
??binaryzation2_25:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        LDRB     R2,[R1, R2]
        LDR.W    R3,??DataTable53_3
        LDRB     R3,[R3, #+0]
        CMP      R2,R3
        BCS.N    ??binaryzation2_29
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_4
        MLA      R2,R2,R0,R3
        MOVS     R3,#+0
        STRB     R3,[R1, R2]
        B.N      ??binaryzation2_30
??binaryzation2_29:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_4
        MLA      R2,R2,R0,R3
        MOVS     R3,#+255
        STRB     R3,[R1, R2]
??binaryzation2_30:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        LDRB     R2,[R1, R2]
        LDR.W    R3,??DataTable53_4
        LDRB     R3,[R3, #+0]
        CMP      R2,R3
        BCS.W    ??binaryzation2_19
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        MOVS     R3,#+0
        STRB     R3,[R1, R2]
        B.N      ??binaryzation2_20
// 1399          
// 1400         }
// 1401       }
// 1402     }
// 1403     else if(i<34)
??binaryzation2_17:
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        CMP      R0,#+34
        BCS.W    ??binaryzation2_31
// 1404     {
// 1405       for(j=0;j<IMG_COLS;j++)
        MOVS     R1,#+0
        B.N      ??binaryzation2_32
// 1406       {
// 1407         if(j<50)
// 1408         {
// 1409            if(ImageData[i][j]<black2_b1)ImageData2[i][j]=0;
// 1410           else ImageData2[i][j]=255;
// 1411           if(ImageData[i][j]<black_b1)ImageData[i][j]=0;
// 1412           else ImageData[i][j]=255;
// 1413          
// 1414         }
// 1415         else if(j<110)
// 1416         {
// 1417            if(ImageData[i][j]<black2_b2)ImageData2[i][j]=0;
// 1418           else ImageData2[i][j]=255;
// 1419           if(ImageData[i][j]<black_b2)ImageData[i][j]=0;
// 1420           else ImageData[i][j]=255;
// 1421          
// 1422         }
// 1423         else
// 1424         {
// 1425           if(ImageData[i][j]<black2_b3)ImageData2[i][j]=0;
// 1426           else ImageData2[i][j]=255;
// 1427           if(ImageData[i][j]<black_b3)ImageData[i][j]=0;
// 1428           else ImageData[i][j]=255;
??binaryzation2_33:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        MOVS     R3,#+255
        STRB     R3,[R1, R2]
??binaryzation2_34:
        ADDS     R1,R1,#+1
??binaryzation2_32:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        CMP      R1,#+160
        BCS.W    ??binaryzation2_1
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        CMP      R1,#+50
        BCS.N    ??binaryzation2_35
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        LDRB     R2,[R1, R2]
        LDR.W    R3,??DataTable53_5
        LDRB     R3,[R3, #+0]
        CMP      R2,R3
        BCS.N    ??binaryzation2_36
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_4
        MLA      R2,R2,R0,R3
        MOVS     R3,#+0
        STRB     R3,[R1, R2]
        B.N      ??binaryzation2_37
??binaryzation2_36:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_4
        MLA      R2,R2,R0,R3
        MOVS     R3,#+255
        STRB     R3,[R1, R2]
??binaryzation2_37:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        LDRB     R2,[R1, R2]
        LDR.W    R3,??DataTable53_6
        LDRB     R3,[R3, #+0]
        CMP      R2,R3
        BCS.N    ??binaryzation2_38
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        MOVS     R3,#+0
        STRB     R3,[R1, R2]
        B.N      ??binaryzation2_34
??binaryzation2_38:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        MOVS     R3,#+255
        STRB     R3,[R1, R2]
        B.N      ??binaryzation2_34
??binaryzation2_35:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        CMP      R1,#+110
        BCS.N    ??binaryzation2_39
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        LDRB     R2,[R1, R2]
        LDR.W    R3,??DataTable53_7
        LDRB     R3,[R3, #+0]
        CMP      R2,R3
        BCS.N    ??binaryzation2_40
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_4
        MLA      R2,R2,R0,R3
        MOVS     R3,#+0
        STRB     R3,[R1, R2]
        B.N      ??binaryzation2_41
??binaryzation2_40:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_4
        MLA      R2,R2,R0,R3
        MOVS     R3,#+255
        STRB     R3,[R1, R2]
??binaryzation2_41:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        LDRB     R2,[R1, R2]
        LDR.W    R3,??DataTable53_8
        LDRB     R3,[R3, #+0]
        CMP      R2,R3
        BCS.N    ??binaryzation2_42
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        MOVS     R3,#+0
        STRB     R3,[R1, R2]
        B.N      ??binaryzation2_34
??binaryzation2_42:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        MOVS     R3,#+255
        STRB     R3,[R1, R2]
        B.N      ??binaryzation2_34
??binaryzation2_39:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        LDRB     R2,[R1, R2]
        LDR.W    R3,??DataTable53_9
        LDRB     R3,[R3, #+0]
        CMP      R2,R3
        BCS.N    ??binaryzation2_43
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_4
        MLA      R2,R2,R0,R3
        MOVS     R3,#+0
        STRB     R3,[R1, R2]
        B.N      ??binaryzation2_44
??binaryzation2_43:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_4
        MLA      R2,R2,R0,R3
        MOVS     R3,#+255
        STRB     R3,[R1, R2]
??binaryzation2_44:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        LDRB     R2,[R1, R2]
        LDR.W    R3,??DataTable53_10
        LDRB     R3,[R3, #+0]
        CMP      R2,R3
        BCS.W    ??binaryzation2_33
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        MOVS     R3,#+0
        STRB     R3,[R1, R2]
        B.N      ??binaryzation2_34
// 1429         }
// 1430       }
// 1431     }
// 1432     else
// 1433     {
// 1434       for(j=0;j<IMG_COLS;j++)
??binaryzation2_31:
        MOVS     R1,#+0
        B.N      ??binaryzation2_45
// 1435       {
// 1436         if(j<50)
// 1437         {
// 1438           if(ImageData[i][j]<black2_a1)ImageData2[i][j]=0;
// 1439           else ImageData2[i][j]=255;
// 1440           if(ImageData[i][j]<black_a1)ImageData[i][j]=0;
// 1441           else ImageData[i][j]=255;
// 1442         }
// 1443         else if(j<110)
// 1444         {
// 1445           if(ImageData[i][j]<black2_a2)ImageData2[i][j]=0;
// 1446           else ImageData2[i][j]=255;
// 1447           if(ImageData[i][j]<black_a2)ImageData[i][j]=0;
// 1448           else ImageData[i][j]=255;
// 1449         }
// 1450         else
// 1451         {
// 1452           if(ImageData[i][j]<black2_a3)ImageData2[i][j]=0;
// 1453           else ImageData2[i][j]=255;
// 1454           if(ImageData[i][j]<black_a3)ImageData[i][j]=0;
// 1455           else ImageData[i][j]=255;
??binaryzation2_46:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        MOVS     R3,#+255
        STRB     R3,[R1, R2]
??binaryzation2_47:
        ADDS     R1,R1,#+1
??binaryzation2_45:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        CMP      R1,#+160
        BCS.W    ??binaryzation2_1
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        CMP      R1,#+50
        BCS.N    ??binaryzation2_48
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        LDRB     R2,[R1, R2]
        LDR.W    R3,??DataTable53_11
        LDRB     R3,[R3, #+0]
        CMP      R2,R3
        BCS.N    ??binaryzation2_49
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_4
        MLA      R2,R2,R0,R3
        MOVS     R3,#+0
        STRB     R3,[R1, R2]
        B.N      ??binaryzation2_50
??binaryzation2_49:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_4
        MLA      R2,R2,R0,R3
        MOVS     R3,#+255
        STRB     R3,[R1, R2]
??binaryzation2_50:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        LDRB     R2,[R1, R2]
        LDR.W    R3,??DataTable53_12
        LDRB     R3,[R3, #+0]
        CMP      R2,R3
        BCS.N    ??binaryzation2_51
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        MOVS     R3,#+0
        STRB     R3,[R1, R2]
        B.N      ??binaryzation2_47
??binaryzation2_51:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        MOVS     R3,#+255
        STRB     R3,[R1, R2]
        B.N      ??binaryzation2_47
??binaryzation2_48:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        CMP      R1,#+110
        BCS.N    ??binaryzation2_52
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        LDRB     R2,[R1, R2]
        LDR.W    R3,??DataTable53_13
        LDRB     R3,[R3, #+0]
        CMP      R2,R3
        BCS.N    ??binaryzation2_53
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_4
        MLA      R2,R2,R0,R3
        MOVS     R3,#+0
        STRB     R3,[R1, R2]
        B.N      ??binaryzation2_54
??binaryzation2_53:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_4
        MLA      R2,R2,R0,R3
        MOVS     R3,#+255
        STRB     R3,[R1, R2]
??binaryzation2_54:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        LDRB     R2,[R1, R2]
        LDR.W    R3,??DataTable53_14
        LDRB     R3,[R3, #+0]
        CMP      R2,R3
        BCS.N    ??binaryzation2_55
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        MOVS     R3,#+0
        STRB     R3,[R1, R2]
        B.N      ??binaryzation2_47
??binaryzation2_55:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        MOVS     R3,#+255
        STRB     R3,[R1, R2]
        B.N      ??binaryzation2_47
??binaryzation2_52:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        LDRB     R2,[R1, R2]
        LDR.W    R3,??DataTable54
        LDRB     R3,[R3, #+0]
        CMP      R2,R3
        BCS.N    ??binaryzation2_56
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_4
        MLA      R2,R2,R0,R3
        MOVS     R3,#+0
        STRB     R3,[R1, R2]
        B.N      ??binaryzation2_57
??binaryzation2_56:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_4
        MLA      R2,R2,R0,R3
        MOVS     R3,#+255
        STRB     R3,[R1, R2]
??binaryzation2_57:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        LDRB     R2,[R1, R2]
        LDR.W    R3,??DataTable54_1
        LDRB     R3,[R3, #+0]
        CMP      R2,R3
        BCS.W    ??binaryzation2_46
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        MOVS     R3,#+0
        STRB     R3,[R1, R2]
        B.N      ??binaryzation2_47
// 1456         }
// 1457       }
// 1458     }
// 1459   }
// 1460 }
??binaryzation2_2:
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable51:
        DC32     speed_b

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable51_1:
        DC32     speed_m

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable51_2:
        DC32     speed_s

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable51_3:
        DC32     ??i
// 1461 /****************二值化*****************/

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1462 void binaryzation()
// 1463 {
// 1464  uint8 i=0,j=0;
binaryzation:
        MOVS     R0,#+0
        MOVS     R1,#+0
// 1465   for(i=0;i<IMG_ROWS;i++)
        MOVS     R2,#+0
        MOVS     R0,R2
        B.N      ??binaryzation_0
??binaryzation_1:
        ADDS     R0,R0,#+1
??binaryzation_0:
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        CMP      R0,#+55
        BCS.W    ??binaryzation_2
// 1466   {
// 1467     if(i<8)
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        CMP      R0,#+8
        BCS.N    ??binaryzation_3
// 1468     {
// 1469       for(j=0;j<IMG_COLS;j++)
        MOVS     R1,#+0
        B.N      ??binaryzation_4
// 1470       {
// 1471         if(j<50)
// 1472         {
// 1473           if(ImageData[i][j]<black_d1)ImageData[i][j]=0;
// 1474           else ImageData[i][j]=255;
// 1475         }
// 1476         else if(j<110)
// 1477         {
// 1478           if(ImageData[i][j]<black_d2)ImageData[i][j]=0;
// 1479           else ImageData[i][j]=255;
// 1480         }
// 1481         else
// 1482         {
// 1483           if(ImageData[i][j]<black_d3)ImageData[i][j]=0;
// 1484           else ImageData[i][j]=255;
??binaryzation_5:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.W    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        MOVS     R3,#+255
        STRB     R3,[R1, R2]
??binaryzation_6:
        ADDS     R1,R1,#+1
??binaryzation_4:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        CMP      R1,#+160
        BCS.N    ??binaryzation_1
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        CMP      R1,#+50
        BCS.N    ??binaryzation_7
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.N    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        LDRB     R2,[R1, R2]
        LDR.N    R3,??DataTable52_5
        LDRB     R3,[R3, #+0]
        CMP      R2,R3
        BCS.N    ??binaryzation_8
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.N    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        MOVS     R3,#+0
        STRB     R3,[R1, R2]
        B.N      ??binaryzation_6
??binaryzation_8:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.N    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        MOVS     R3,#+255
        STRB     R3,[R1, R2]
        B.N      ??binaryzation_6
??binaryzation_7:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        CMP      R1,#+110
        BCS.N    ??binaryzation_9
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.N    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        LDRB     R2,[R1, R2]
        LDR.N    R3,??DataTable52_7
        LDRB     R3,[R3, #+0]
        CMP      R2,R3
        BCS.N    ??binaryzation_10
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.N    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        MOVS     R3,#+0
        STRB     R3,[R1, R2]
        B.N      ??binaryzation_6
??binaryzation_10:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.N    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        MOVS     R3,#+255
        STRB     R3,[R1, R2]
        B.N      ??binaryzation_6
??binaryzation_9:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.N    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        LDRB     R2,[R1, R2]
        LDR.N    R3,??DataTable52_9
        LDRB     R3,[R3, #+0]
        CMP      R2,R3
        BCS.N    ??binaryzation_5
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.N    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        MOVS     R3,#+0
        STRB     R3,[R1, R2]
        B.N      ??binaryzation_6
// 1485         }
// 1486       }
// 1487     }
// 1488     else if(i<20)
??binaryzation_3:
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        CMP      R0,#+20
        BCS.N    ??binaryzation_11
// 1489     {
// 1490       for(j=0;j<IMG_COLS;j++)
        MOVS     R1,#+0
        B.N      ??binaryzation_12
// 1491       {
// 1492         if(j<50)
// 1493         {
// 1494           if(ImageData[i][j]<black_c1)ImageData[i][j]=0;
// 1495           else ImageData[i][j]=255;
// 1496         }
// 1497         else if(j<110)
// 1498         {
// 1499           if(ImageData[i][j]<black_c2)ImageData[i][j]=0;
// 1500           else ImageData[i][j]=255;
// 1501         }
// 1502         else
// 1503         {
// 1504           if(ImageData[i][j]<black_c3)ImageData[i][j]=0;
// 1505           else ImageData[i][j]=255;
??binaryzation_13:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.N    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        MOVS     R3,#+255
        STRB     R3,[R1, R2]
??binaryzation_14:
        ADDS     R1,R1,#+1
??binaryzation_12:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        CMP      R1,#+160
        BCS.N    ??binaryzation_1
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        CMP      R1,#+50
        BCS.N    ??binaryzation_15
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.N    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        LDRB     R2,[R1, R2]
        LDR.W    R3,??DataTable53
        LDRB     R3,[R3, #+0]
        CMP      R2,R3
        BCS.N    ??binaryzation_16
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.N    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        MOVS     R3,#+0
        STRB     R3,[R1, R2]
        B.N      ??binaryzation_14
??binaryzation_16:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.N    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        MOVS     R3,#+255
        STRB     R3,[R1, R2]
        B.N      ??binaryzation_14
??binaryzation_15:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        CMP      R1,#+110
        BCS.N    ??binaryzation_17
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.N    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        LDRB     R2,[R1, R2]
        LDR.W    R3,??DataTable53_2
        LDRB     R3,[R3, #+0]
        CMP      R2,R3
        BCS.N    ??binaryzation_18
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.N    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        MOVS     R3,#+0
        STRB     R3,[R1, R2]
        B.N      ??binaryzation_14
??binaryzation_18:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.N    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        MOVS     R3,#+255
        STRB     R3,[R1, R2]
        B.N      ??binaryzation_14
??binaryzation_17:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.N    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        LDRB     R2,[R1, R2]
        LDR.W    R3,??DataTable53_4
        LDRB     R3,[R3, #+0]
        CMP      R2,R3
        BCS.N    ??binaryzation_13
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.N    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        MOVS     R3,#+0
        STRB     R3,[R1, R2]
        B.N      ??binaryzation_14
// 1506         }
// 1507       }
// 1508     }
// 1509     else if(i<34)
??binaryzation_11:
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        CMP      R0,#+34
        BCS.N    ??binaryzation_19
// 1510     {
// 1511       for(j=0;j<IMG_COLS;j++)
        MOVS     R1,#+0
        B.N      ??binaryzation_20
// 1512       {
// 1513         if(j<50)
// 1514         {
// 1515           if(ImageData[i][j]<black_b1)ImageData[i][j]=0;
// 1516           else ImageData[i][j]=255;
// 1517         }
// 1518         else if(j<110)
// 1519         {
// 1520           if(ImageData[i][j]<black_b2)ImageData[i][j]=0;
// 1521           else ImageData[i][j]=255;
// 1522         }
// 1523         else
// 1524         {
// 1525           if(ImageData[i][j]<black_b3)ImageData[i][j]=0;
// 1526           else ImageData[i][j]=255;
??binaryzation_21:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.N    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        MOVS     R3,#+255
        STRB     R3,[R1, R2]
??binaryzation_22:
        ADDS     R1,R1,#+1
??binaryzation_20:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        CMP      R1,#+160
        BCS.W    ??binaryzation_1
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        CMP      R1,#+50
        BCS.N    ??binaryzation_23
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.N    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        LDRB     R2,[R1, R2]
        LDR.W    R3,??DataTable53_6
        LDRB     R3,[R3, #+0]
        CMP      R2,R3
        BCS.N    ??binaryzation_24
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.N    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        MOVS     R3,#+0
        STRB     R3,[R1, R2]
        B.N      ??binaryzation_22
??binaryzation_24:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.N    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        MOVS     R3,#+255
        STRB     R3,[R1, R2]
        B.N      ??binaryzation_22
??binaryzation_23:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        CMP      R1,#+110
        BCS.N    ??binaryzation_25
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.N    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        LDRB     R2,[R1, R2]
        LDR.W    R3,??DataTable53_8
        LDRB     R3,[R3, #+0]
        CMP      R2,R3
        BCS.N    ??binaryzation_26
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.N    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        MOVS     R3,#+0
        STRB     R3,[R1, R2]
        B.N      ??binaryzation_22
??binaryzation_26:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.N    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        MOVS     R3,#+255
        STRB     R3,[R1, R2]
        B.N      ??binaryzation_22
??binaryzation_25:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.N    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        LDRB     R2,[R1, R2]
        LDR.W    R3,??DataTable53_10
        LDRB     R3,[R3, #+0]
        CMP      R2,R3
        BCS.N    ??binaryzation_21
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.N    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        MOVS     R3,#+0
        STRB     R3,[R1, R2]
        B.N      ??binaryzation_22
// 1527         }
// 1528       }
// 1529     }
// 1530     else
// 1531     {
// 1532       for(j=0;j<IMG_COLS;j++)
??binaryzation_19:
        MOVS     R1,#+0
        B.N      ??binaryzation_27
// 1533       {
// 1534         if(j<50)
// 1535         {
// 1536           if(ImageData[i][j]<black_a1)ImageData[i][j]=0;
// 1537           else ImageData[i][j]=255;
// 1538         }
// 1539         else if(j<110)
// 1540         {
// 1541           if(ImageData[i][j]<black_a2)ImageData[i][j]=0;
// 1542           else ImageData[i][j]=255;
// 1543         }
// 1544         else
// 1545         {
// 1546           if(ImageData[i][j]<black_a3)ImageData[i][j]=0;
// 1547           else ImageData[i][j]=255;
??binaryzation_28:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.N    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        MOVS     R3,#+255
        STRB     R3,[R1, R2]
??binaryzation_29:
        ADDS     R1,R1,#+1
??binaryzation_27:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        CMP      R1,#+160
        BCS.W    ??binaryzation_1
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        CMP      R1,#+50
        BCS.N    ??binaryzation_30
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.N    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        LDRB     R2,[R1, R2]
        LDR.W    R3,??DataTable53_12
        LDRB     R3,[R3, #+0]
        CMP      R2,R3
        BCS.N    ??binaryzation_31
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.N    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        MOVS     R3,#+0
        STRB     R3,[R1, R2]
        B.N      ??binaryzation_29
??binaryzation_31:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.N    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        MOVS     R3,#+255
        STRB     R3,[R1, R2]
        B.N      ??binaryzation_29
??binaryzation_30:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        CMP      R1,#+110
        BCS.N    ??binaryzation_32
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.N    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        LDRB     R2,[R1, R2]
        LDR.W    R3,??DataTable53_14
        LDRB     R3,[R3, #+0]
        CMP      R2,R3
        BCS.N    ??binaryzation_33
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.N    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        MOVS     R3,#+0
        STRB     R3,[R1, R2]
        B.N      ??binaryzation_29
??binaryzation_33:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.N    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        MOVS     R3,#+255
        STRB     R3,[R1, R2]
        B.N      ??binaryzation_29
??binaryzation_32:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.N    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        LDRB     R2,[R1, R2]
        LDR.W    R3,??DataTable54_1
        LDRB     R3,[R3, #+0]
        CMP      R2,R3
        BCS.N    ??binaryzation_28
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R2,#+160
        LDR.N    R3,??DataTable52_2
        MLA      R2,R2,R0,R3
        MOVS     R3,#+0
        STRB     R3,[R1, R2]
        B.N      ??binaryzation_29
// 1548         }
// 1549       }
// 1550     }
// 1551   }
// 1552     for(i=0;i<56;i++)
??binaryzation_2:
        MOVS     R0,#+0
        B.N      ??binaryzation_34
// 1553     position[i]=0;  
??binaryzation_35:
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        LDR.W    R1,??DataTable56
        MOVS     R2,#+0
        STR      R2,[R1, R0, LSL #+2]
        ADDS     R0,R0,#+1
??binaryzation_34:
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        CMP      R0,#+56
        BCC.N    ??binaryzation_35
// 1554 }
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable52:
        DC32     point

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable52_1:
        DC32     ??line

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable52_2:
        DC32     ImageData

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable52_3:
        DC32     black2_d1

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable52_4:
        DC32     ImageData2

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable52_5:
        DC32     black_d1

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable52_6:
        DC32     black2_d2

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable52_7:
        DC32     black_d2

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable52_8:
        DC32     black2_d3

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable52_9:
        DC32     black_d3

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable52_10:
        DC32     black2_c1
// 1555 

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1556 int Judge_startline2(void)
// 1557 {
Judge_startline2:
        PUSH     {R4-R11,LR}
        SUB      SP,SP,#+20
// 1558   int i,j,m=0;
        MOVS     R0,#+0
        STR      R0,[SP, #+4]
// 1559   int flag1=0,find_flag=0,num_l=0,num_r=0,startline=0,num_w1=0,num_w2=0,num_w=0;
        MOVS     R0,#+0
        STR      R0,[SP, #+12]
        MOVS     R12,#+0
        MOVS     R0,#+0
        MOVS     R1,#+0
        MOVS     R3,#+0
        STR      R3,[SP, #+0]
        MOVS     R4,#+0
        MOVS     R5,#+0
        MOVS     R2,#+0
// 1560   int position_num=0,position_old=0,num=0,find_old_num=0,a;
        MOVS     R6,#+0
        STR      R6,[SP, #+4]
        MOVS     R6,#+0
        STR      R6,[SP, #+4]
        MOVS     R6,#+0
        MOVS     R7,#+0
        STR      R7,[SP, #+4]
// 1561   uint16 median=0,b=0,c=0,d=0,e=0,f=0;
        MOVS     R11,#+0
        MOVS     R7,#+0
        MOVS     R8,#+0
        MOVS     R9,#+0
        MOVS     R10,#+0
        STRH     R10,[SP, #+8]
        MOVS     R10,#+0
        STRH     R10,[SP, #+8]
// 1562   uint16 start_p;
// 1563   j=15;
        MOVS     R10,#+15
// 1564   i=0;
        MOVS     R3,#+0
        B.N      ??Judge_startline2_0
// 1565   while(j<135)
// 1566   {
// 1567     find_flag=0;
// 1568     ption[j]=0;
// 1569     for(i=13;i<53;i++)
// 1570     {
// 1571       median=0;
// 1572       num=0;
// 1573       if(!ImageData2[i+2][j]&&ImageData2[i][j]&&ImageData2[i+1][j])
// 1574       {
// 1575         for(i=i+2;i<55;i++)
// 1576         {
// 1577           if(i<23)
// 1578           {
// 1579             a=2;            
// 1580           }
// 1581           if(i>=23&&i<33)
// 1582           {
// 1583             a=3;
// 1584           }
// 1585           if(i>=33&&i<45)
// 1586           {
// 1587             a=4;
// 1588           }
// 1589           if(i>=45&&i<55)
// 1590           {
// 1591             a=4;
// 1592           }
// 1593           if(!ImageData2[i][j]&&i<54)
// 1594           {
// 1595             median+=i;
// 1596             num++;
// 1597           }
// 1598           else
// 1599           {
// 1600             if(num<=a&&num>0)
// 1601             {              
// 1602               median=median/num;
// 1603             }
// 1604             else 
// 1605               median=0;
??Judge_startline2_1:
        MOVS     R11,#+0
// 1606             if(median)
??Judge_startline2_2:
        UXTH     R11,R11          ;; ZeroExt  R11,R11,#+16,#+16
        CMP      R11,#+0
        BEQ.N    ??Judge_startline2_3
// 1607             {
// 1608               ption[j]=median;
        LDR.W    R0,??DataTable56_1
        STRH     R11,[R0, R10, LSL #+1]
// 1609               find_flag=1;				
        MOVS     R12,#+1
// 1610             }
// 1611             break;
// 1612           }
// 1613         }			
// 1614         if(find_flag)
??Judge_startline2_3:
??Judge_startline2_4:
        CMP      R12,#+0
        BEQ.N    ??Judge_startline2_5
// 1615           break;
// 1616       }	
// 1617     }
// 1618     j++;
??Judge_startline2_6:
        ADDS     R10,R10,#+1
??Judge_startline2_0:
        CMP      R10,#+135
        BGE.N    ??Judge_startline2_7
        MOVS     R12,#+0
        LDR.W    R0,??DataTable56_1
        MOVS     R1,#+0
        STRH     R1,[R0, R10, LSL #+1]
        MOVS     R3,#+13
        B.N      ??Judge_startline2_8
??Judge_startline2_5:
        ADDS     R3,R3,#+1
??Judge_startline2_8:
        CMP      R3,#+53
        BGE.N    ??Judge_startline2_6
        MOVS     R11,#+0
        MOVS     R6,#+0
        MOVS     R0,#+160
        LDR.W    R1,??DataTable56_2
        MLA      R0,R0,R3,R1
        ADDS     R0,R10,R0
        LDRB     R0,[R0, #+320]
        CMP      R0,#+0
        BNE.N    ??Judge_startline2_5
        MOVS     R0,#+160
        LDR.W    R1,??DataTable56_2
        MLA      R0,R0,R3,R1
        LDRB     R0,[R10, R0]
        CMP      R0,#+0
        BEQ.N    ??Judge_startline2_5
        MOVS     R0,#+160
        LDR.W    R1,??DataTable56_2
        MLA      R0,R0,R3,R1
        ADDS     R0,R10,R0
        LDRB     R0,[R0, #+160]
        CMP      R0,#+0
        BEQ.N    ??Judge_startline2_5
        ADDS     R3,R3,#+2
        B.N      ??Judge_startline2_9
??Judge_startline2_10:
        ADDS     R11,R3,R11
        ADDS     R6,R6,#+1
        ADDS     R3,R3,#+1
??Judge_startline2_9:
        CMP      R3,#+55
        BGE.N    ??Judge_startline2_4
        CMP      R3,#+23
        BGE.N    ??Judge_startline2_11
        MOVS     LR,#+2
??Judge_startline2_11:
        SUBS     R0,R3,#+23
        CMP      R0,#+10
        BCS.N    ??Judge_startline2_12
        MOVS     LR,#+3
??Judge_startline2_12:
        SUBS     R0,R3,#+33
        CMP      R0,#+12
        BCS.N    ??Judge_startline2_13
        MOVS     LR,#+4
??Judge_startline2_13:
        SUBS     R0,R3,#+45
        CMP      R0,#+10
        BCS.N    ??Judge_startline2_14
        MOVS     LR,#+4
??Judge_startline2_14:
        MOVS     R0,#+160
        LDR.W    R1,??DataTable56_2
        MLA      R0,R0,R3,R1
        LDRB     R0,[R10, R0]
        CMP      R0,#+0
        BNE.N    ??Judge_startline2_15
        CMP      R3,#+54
        BLT.N    ??Judge_startline2_10
??Judge_startline2_15:
        CMP      LR,R6
        BLT.N    ??Judge_startline2_1
        CMP      R6,#+1
        BLT.N    ??Judge_startline2_1
        UXTH     R11,R11          ;; ZeroExt  R11,R11,#+16,#+16
        SDIV     R11,R11,R6
        B.N      ??Judge_startline2_2
// 1619   }
// 1620   num=0;
??Judge_startline2_7:
        MOVS     R6,#+0
// 1621   start_p=0;
        MOVS     R11,#+0
// 1622   for(j=15;j<135;j++)
        MOVS     R10,#+15
        B.N      ??Judge_startline2_16
// 1623   {
// 1624     if(ption[j])
??Judge_startline2_17:
        LDR.W    R0,??DataTable56_1
        LDRH     R0,[R0, R10, LSL #+1]
        CMP      R0,#+0
        BEQ.N    ??Judge_startline2_18
// 1625     {
// 1626       start_p+=ption[j];
        LDR.W    R0,??DataTable56_1
        LDRH     R0,[R0, R10, LSL #+1]
        ADDS     R11,R0,R11
// 1627       num++;
        ADDS     R6,R6,#+1
// 1628     }
// 1629   }
??Judge_startline2_18:
        ADDS     R10,R10,#+1
??Judge_startline2_16:
        CMP      R10,#+135
        BLT.N    ??Judge_startline2_17
// 1630   if(num>15)
        CMP      R6,#+16
        BLT.W    ??Judge_startline2_19
// 1631   {
// 1632     start_p=start_p/num;
        UXTH     R11,R11          ;; ZeroExt  R11,R11,#+16,#+16
        SDIV     R11,R11,R6
// 1633     num=0;
        MOVS     R6,#+0
// 1634     for(j=15;j<135;j++)
        MOVS     R10,#+15
        B.N      ??Judge_startline2_20
// 1635     {
// 1636       if(ption[j]&&abs(ption[j]-start_p)>4&&abs(j-position[start_p])<15)
??Judge_startline2_21:
        LDR.W    R0,??DataTable56_1
        LDRH     R0,[R0, R10, LSL #+1]
        CMP      R0,#+0
        BEQ.N    ??Judge_startline2_22
        LDR.W    R0,??DataTable56_1
        LDRSH    R0,[R0, R10, LSL #+1]
        SUBS     R0,R0,R11
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       abs
        CMP      R0,#+5
        BLT.N    ??Judge_startline2_22
        UXTH     R11,R11          ;; ZeroExt  R11,R11,#+16,#+16
        LDR.W    R0,??DataTable56
        LDR      R0,[R0, R11, LSL #+2]
        SUBS     R0,R10,R0
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       abs
        CMP      R0,#+15
        BGE.N    ??Judge_startline2_22
// 1637       {
// 1638         ption[j]=0;
        LDR.W    R0,??DataTable56_1
        MOVS     R1,#+0
        STRH     R1,[R0, R10, LSL #+1]
// 1639       }
// 1640       if(ption[j]&&abs(ption[j]-start_p)>25)
??Judge_startline2_22:
        LDR.W    R0,??DataTable56_1
        LDRH     R0,[R0, R10, LSL #+1]
        CMP      R0,#+0
        BEQ.N    ??Judge_startline2_23
        LDR.W    R0,??DataTable56_1
        LDRSH    R0,[R0, R10, LSL #+1]
        SUBS     R0,R0,R11
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       abs
        CMP      R0,#+26
        BLT.N    ??Judge_startline2_23
// 1641         ption[j]=0;
        LDR.W    R0,??DataTable56_1
        MOVS     R1,#+0
        STRH     R1,[R0, R10, LSL #+1]
// 1642     }
??Judge_startline2_23:
        ADDS     R10,R10,#+1
??Judge_startline2_20:
        CMP      R10,#+135
        BLT.N    ??Judge_startline2_21
// 1643     start_p=0;
        MOVS     R11,#+0
// 1644     for(j=15;j<135;j++)
        MOVS     R10,#+15
        B.N      ??Judge_startline2_24
// 1645     {
// 1646       if(ption[j])
??Judge_startline2_25:
        LDR.W    R0,??DataTable56_1
        LDRH     R0,[R0, R10, LSL #+1]
        CMP      R0,#+0
        BEQ.N    ??Judge_startline2_26
// 1647       {
// 1648         start_p+=ption[j];
        LDR.W    R0,??DataTable56_1
        LDRH     R0,[R0, R10, LSL #+1]
        ADDS     R11,R0,R11
// 1649         num++;
        ADDS     R6,R6,#+1
// 1650         b+=j;
        ADDS     R7,R10,R7
// 1651       }
// 1652     }
??Judge_startline2_26:
        ADDS     R10,R10,#+1
??Judge_startline2_24:
        CMP      R10,#+135
        BLT.N    ??Judge_startline2_25
// 1653     if(num)
        CMP      R6,#+0
        BEQ.N    ??Judge_startline2_27
// 1654       start_p=start_p/num;
        UXTH     R11,R11          ;; ZeroExt  R11,R11,#+16,#+16
        SDIV     R11,R11,R6
// 1655     b=b/num;
??Judge_startline2_27:
        UXTH     R7,R7            ;; ZeroExt  R7,R7,#+16,#+16
        SDIV     R7,R7,R6
// 1656     if(b<52&&b>106)  return 0;	
        UXTH     R7,R7            ;; ZeroExt  R7,R7,#+16,#+16
        CMP      R7,#+52
        BCS.N    ??Judge_startline2_28
        UXTH     R7,R7            ;; ZeroExt  R7,R7,#+16,#+16
        CMP      R7,#+107
        BCC.N    ??Judge_startline2_28
        MOVS     R0,#+0
        B.N      ??Judge_startline2_29
// 1657     if(start_p)
??Judge_startline2_28:
        UXTH     R11,R11          ;; ZeroExt  R11,R11,#+16,#+16
        CMP      R11,#+0
        BEQ.N    ??Judge_startline2_30
// 1658     {
// 1659       if(start_p<25)  f=13;
        UXTH     R11,R11          ;; ZeroExt  R11,R11,#+16,#+16
        CMP      R11,#+25
        BCS.N    ??Judge_startline2_31
        MOVS     R0,#+13
        STRH     R0,[SP, #+8]
        B.N      ??Judge_startline2_30
// 1660       else if(start_p<38) f=16;
??Judge_startline2_31:
        UXTH     R11,R11          ;; ZeroExt  R11,R11,#+16,#+16
        CMP      R11,#+38
        BCS.N    ??Judge_startline2_32
        MOVS     R0,#+16
        STRH     R0,[SP, #+8]
        B.N      ??Judge_startline2_30
// 1661       else if(start_p<55) f=17;}
??Judge_startline2_32:
        UXTH     R11,R11          ;; ZeroExt  R11,R11,#+16,#+16
        CMP      R11,#+55
        BCS.N    ??Judge_startline2_30
        MOVS     R0,#+17
        STRH     R0,[SP, #+8]
// 1662   }
// 1663   else
// 1664     return 0;
// 1665   num=0;
??Judge_startline2_30:
        MOVS     R6,#+0
// 1666   num_l=0;
        MOVS     R0,#+0
// 1667   for(j=15;j<=position[start_p];j++)
        MOVS     R10,#+15
        B.N      ??Judge_startline2_33
??Judge_startline2_19:
        MOVS     R0,#+0
        B.N      ??Judge_startline2_29
// 1668   {
// 1669     if(ption[j])
??Judge_startline2_34:
        LDR.W    R1,??DataTable56_1
        LDRH     R1,[R1, R10, LSL #+1]
        CMP      R1,#+0
        BEQ.N    ??Judge_startline2_35
// 1670     {
// 1671       c+=ption[j];
        LDR.W    R1,??DataTable56_1
        LDRH     R1,[R1, R10, LSL #+1]
        ADDS     R8,R1,R8
// 1672       num_l++;
        ADDS     R0,R0,#+1
// 1673     }
// 1674   }
??Judge_startline2_35:
        ADDS     R10,R10,#+1
??Judge_startline2_33:
        UXTH     R11,R11          ;; ZeroExt  R11,R11,#+16,#+16
        LDR.W    R1,??DataTable56
        LDR      R1,[R1, R11, LSL #+2]
        CMP      R1,R10
        BGE.N    ??Judge_startline2_34
// 1675   c=c/num_l;
        UXTH     R8,R8            ;; ZeroExt  R8,R8,#+16,#+16
        SDIV     R8,R8,R0
// 1676   num_r=0;
        MOVS     R1,#+0
// 1677   for(j=position[start_p];j<135;j++)
        UXTH     R11,R11          ;; ZeroExt  R11,R11,#+16,#+16
        LDR.W    R2,??DataTable56
        LDR      R10,[R2, R11, LSL #+2]
        B.N      ??Judge_startline2_36
// 1678   {
// 1679     if(ption[j])
??Judge_startline2_37:
        LDR.W    R2,??DataTable56_1
        LDRH     R2,[R2, R10, LSL #+1]
        CMP      R2,#+0
        BEQ.N    ??Judge_startline2_38
// 1680     {
// 1681       d+=ption[j];
        LDR.W    R2,??DataTable56_1
        LDRH     R2,[R2, R10, LSL #+1]
        ADDS     R9,R2,R9
// 1682       num_r++;
        ADDS     R1,R1,#+1
// 1683     }
// 1684   }
??Judge_startline2_38:
        ADDS     R10,R10,#+1
??Judge_startline2_36:
        CMP      R10,#+135
        BLT.N    ??Judge_startline2_37
// 1685   d=d/num_r;
        UXTH     R9,R9            ;; ZeroExt  R9,R9,#+16,#+16
        SDIV     R9,R9,R1
// 1686   if(start_p<=20)
        UXTH     R11,R11          ;; ZeroExt  R11,R11,#+16,#+16
        CMP      R11,#+21
        BCS.N    ??Judge_startline2_39
// 1687   {
// 1688     if(num_l<20&&num_l>=10&&num_r<20&&num_r>=10)
        SUBS     R2,R0,#+10
        CMP      R2,#+10
        BCS.N    ??Judge_startline2_40
        SUBS     R2,R1,#+10
        CMP      R2,#+10
        BCS.N    ??Judge_startline2_40
// 1689     {
// 1690       if(abs(num_l-num_r)<5)
        SUBS     R0,R0,R1
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       abs
        CMP      R0,#+5
        BGE.N    ??Judge_startline2_40
// 1691       {
// 1692         startline=1;
        MOVS     R0,#+1
        STR      R0,[SP, #+0]
        B.N      ??Judge_startline2_40
// 1693       }
// 1694     }
// 1695   }
// 1696   else
// 1697     if(start_p<=30)
??Judge_startline2_39:
        UXTH     R11,R11          ;; ZeroExt  R11,R11,#+16,#+16
        CMP      R11,#+31
        BCS.N    ??Judge_startline2_41
// 1698     {
// 1699       if(num_l<24&&num_l>13&&num_r<24&&num_r>13)
        SUBS     R2,R0,#+14
        CMP      R2,#+10
        BCS.N    ??Judge_startline2_40
        SUBS     R2,R1,#+14
        CMP      R2,#+10
        BCS.N    ??Judge_startline2_40
// 1700       {
// 1701         if(abs(num_l-num_r)<6)
        SUBS     R0,R0,R1
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       abs
        CMP      R0,#+6
        BGE.N    ??Judge_startline2_40
// 1702         {
// 1703           startline=1;
        MOVS     R0,#+1
        STR      R0,[SP, #+0]
        B.N      ??Judge_startline2_40
// 1704         }
// 1705       }
// 1706     }
// 1707     else 
// 1708       if(start_p<=38)
??Judge_startline2_41:
        UXTH     R11,R11          ;; ZeroExt  R11,R11,#+16,#+16
        CMP      R11,#+39
        BCS.N    ??Judge_startline2_42
// 1709       {
// 1710 	if(num_l<33&&num_l>18&&num_r<33&&num_r>18)
        SUBS     R2,R0,#+19
        CMP      R2,#+14
        BCS.N    ??Judge_startline2_40
        SUBS     R2,R1,#+19
        CMP      R2,#+14
        BCS.N    ??Judge_startline2_40
// 1711 	{
// 1712           if(abs(num_l-num_r)<8)
        SUBS     R0,R0,R1
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       abs
        CMP      R0,#+8
        BGE.N    ??Judge_startline2_40
// 1713           {
// 1714             startline=1;
        MOVS     R0,#+1
        STR      R0,[SP, #+0]
        B.N      ??Judge_startline2_40
// 1715           }
// 1716 	}
// 1717       }
// 1718   else 
// 1719     {
// 1720       if(num_l<36&&num_l>22&&num_r<36&&num_r>22)
??Judge_startline2_42:
        SUBS     R2,R0,#+23
        CMP      R2,#+13
        BCS.N    ??Judge_startline2_40
        SUBS     R2,R1,#+23
        CMP      R2,#+13
        BCS.N    ??Judge_startline2_40
// 1721       {
// 1722         if(abs(num_l-num_r)<8)
        SUBS     R0,R0,R1
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       abs
        CMP      R0,#+8
        BGE.N    ??Judge_startline2_40
// 1723         {
// 1724           startline=1;
        MOVS     R0,#+1
        STR      R0,[SP, #+0]
// 1725         }
// 1726       }
// 1727     }
// 1728   if(startline==1&&start_p<40)
??Judge_startline2_40:
        LDR      R0,[SP, #+0]
        CMP      R0,#+1
        BNE.N    ??Judge_startline2_43
        UXTH     R11,R11          ;; ZeroExt  R11,R11,#+16,#+16
        CMP      R11,#+40
        BCS.N    ??Judge_startline2_43
// 1729   {
// 1730     for(i=0;i<160;i++)
        MOVS     R3,#+0
        B.N      ??Judge_startline2_44
// 1731     {
// 1732       if(!ImageData[start_p][i])
??Judge_startline2_45:
        UXTH     R11,R11          ;; ZeroExt  R11,R11,#+16,#+16
        MOVS     R0,#+160
        LDR.W    R1,??DataTable55
        MLA      R0,R0,R11,R1
        LDRB     R0,[R3, R0]
        CMP      R0,#+0
        BNE.N    ??Judge_startline2_46
// 1733         num_w1++;
        ADDS     R4,R4,#+1
// 1734     }
??Judge_startline2_46:
        ADDS     R3,R3,#+1
??Judge_startline2_44:
        CMP      R3,#+160
        BLT.N    ??Judge_startline2_45
// 1735     for(i=15;i<135;i++)
        MOVS     R3,#+15
        B.N      ??Judge_startline2_47
// 1736     {
// 1737       if(ption[i]==start_p)
??Judge_startline2_48:
        LDR.W    R0,??DataTable56_1
        LDRH     R0,[R0, R3, LSL #+1]
        UXTH     R11,R11          ;; ZeroExt  R11,R11,#+16,#+16
        CMP      R0,R11
        BNE.N    ??Judge_startline2_49
// 1738         num_w2++;
        ADDS     R5,R5,#+1
// 1739     }
??Judge_startline2_49:
        ADDS     R3,R3,#+1
??Judge_startline2_47:
        CMP      R3,#+135
        BLT.N    ??Judge_startline2_48
// 1740     num_w=num_w1-num_w2;
        SUBS     R2,R4,R5
// 1741     if(start_p<22)
        UXTH     R11,R11          ;; ZeroExt  R11,R11,#+16,#+16
        CMP      R11,#+22
        BCS.N    ??Judge_startline2_50
// 1742     {
// 1743       if(num_w<50)
        CMP      R2,#+50
        BGE.N    ??Judge_startline2_43
// 1744         startline=0;
        MOVS     R0,#+0
        STR      R0,[SP, #+0]
        B.N      ??Judge_startline2_43
// 1745     }
// 1746     else 
// 1747       if(start_p<34)
??Judge_startline2_50:
        UXTH     R11,R11          ;; ZeroExt  R11,R11,#+16,#+16
        CMP      R11,#+34
        BCS.N    ??Judge_startline2_51
// 1748       {
// 1749         if(num_w<33)
        CMP      R2,#+33
        BGE.N    ??Judge_startline2_43
// 1750           startline=0;
        MOVS     R0,#+0
        STR      R0,[SP, #+0]
        B.N      ??Judge_startline2_43
// 1751       }
// 1752       else
// 1753       {
// 1754         if(num_w<14)
??Judge_startline2_51:
        CMP      R2,#+14
        BGE.N    ??Judge_startline2_43
// 1755           startline=0;
        MOVS     R0,#+0
        STR      R0,[SP, #+0]
// 1756       }
// 1757   }
// 1758            flag1=startline;
??Judge_startline2_43:
        LDR      R0,[SP, #+0]
        STR      R0,[SP, #+12]
// 1759            return flag1;
        LDR      R0,[SP, #+12]
??Judge_startline2_29:
        ADD      SP,SP,#+20
        POP      {R4-R11,PC}      ;; return
// 1760 }

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable53:
        DC32     black_c1

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable53_1:
        DC32     black2_c2

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable53_2:
        DC32     black_c2

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable53_3:
        DC32     black2_c3

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable53_4:
        DC32     black_c3

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable53_5:
        DC32     black2_b1

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable53_6:
        DC32     black_b1

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable53_7:
        DC32     black2_b2

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable53_8:
        DC32     black_b2

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable53_9:
        DC32     black2_b3

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable53_10:
        DC32     black_b3

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable53_11:
        DC32     black2_a1

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable53_12:
        DC32     black_a1

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable53_13:
        DC32     black2_a2

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable53_14:
        DC32     black_a2
// 1761 

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1762 int get_root()
// 1763 {
get_root:
        PUSH     {R3-R11,LR}
// 1764   uint8 i=0,j=0,position_old=0,start=0,windage=28,endR=0,endL=0,cuo_root=0,cuo_root1=0;
        MOVS     R4,#+0
        MOVS     R10,#+0
        MOVS     R0,#+0
        STRB     R0,[SP, #+0]
        MOVS     R0,#+0
        STRB     R0,[SP, #+3]
        MOVS     R0,#+28
        STRB     R0,[SP, #+2]
        MOVS     R11,#+0
        MOVS     R0,#+0
        STRB     R0,[SP, #+1]
        MOVS     R5,#+0
        MOVS     R6,#+0
// 1765   uint8 find_num=0,find_flag=0,hang_num=0,num=0;
        MOVS     R7,#+0
        MOVS     R9,#+0
        MOVS     R0,#+0
        MOVS     R8,#+0
// 1766   for(i=54;i>=45;i--)
        MOVS     R1,#+54
        MOVS     R4,R1
        B.N      ??get_root_0
// 1767     {
// 1768       if(start==1)
// 1769 	{
// 1770           find_flag=0;
// 1771           endL=limit(position_old-windage,158,1);
// 1772           endR= limit(position_old+windage,158,1);	
// 1773 	  for(j=endL;j<=endR;j++)
// 1774 	   {
// 1775 	      if(!find_flag&&ImageData[i][j]&&ImageData[i][j+1]&&!ImageData[i][j+2])
// 1776 		{
// 1777                   for(j+=2;j<endR;j++)
// 1778                   {
// 1779                     if(!ImageData[i][j]&&j<endR-1)
// 1780                     {
// 1781                       position[i]+=j;
// 1782                       find_num++;
// 1783                       find_flag=1;	
// 1784                     }
// 1785                     else
// 1786                     {
// 1787                       if(ImageData[i][j]||(j==endR-1))
// 1788                         {
// 1789 	                  if(j==endR-1)
// 1790                             {
// 1791                               if(!ImageData[i][j])
// 1792                                 {
// 1793                                     for(j=endR;j<=limit(endR+20,158,1);j++)
// 1794                                       {
// 1795                                         if(!ImageData[i][j])
// 1796                                           {
// 1797                                             position[i]+=j;
// 1798                                             find_num++;
// 1799                                           } 
// 1800                                         else
// 1801                                           break;
// 1802                                       }
// 1803                                 }
// 1804                             }   
// 1805                           if(!ImageData[i][j])
// 1806                             {
// 1807                                position[i]=0;
// 1808                                find_num=0;
// 1809                                break;
// 1810                             }
// 1811                           if(find_num>2&&find_num<16) 
// 1812                           {
// 1813                             cuo_root1=0;
// 1814                             position[i]=position[i]/find_num;
// 1815                             find_num=0;
// 1816                           }
// 1817                         else
// 1818                           {
// 1819                             if(find_num>=16)//十字交叉处理
// 1820 			      {
// 1821                                 position[i]=0;
// 1822                                 find_num=0;
// 1823                                 cuo_root1=1;
// 1824                               }	 
// 1825                             position[i]=0;
// 1826                             find_num=0;
// 1827                           }
// 1828                         }
// 1829                       if(position[i]&&abs(position[i]-position_old)>15) 
// 1830                         {
// 1831                           position[i]=0;										   
// 1832                         }
// 1833                       if(cuo_root1==0)
// 1834                       break;
// 1835                     }
// 1836                    }
// 1837 	        }
// 1838 	      }
// 1839           }
// 1840       else
// 1841 	{	find_flag=0;
// 1842 	  for(j=4;j<155;j++)
// 1843 	    {
// 1844 	      if(!find_flag&&ImageData[i][j-1]&&ImageData[i][j]&&ImageData[i][j+1]&&!ImageData[i][j+2])
// 1845 	        {
// 1846 	          for(j+=2;j<153;j++)
// 1847 	            {
// 1848 		      if(!ImageData[i][j]&&j<152)
// 1849 			{
// 1850                           position[i]+=j;
// 1851                           find_num++;
// 1852                           find_flag=1;
// 1853 		        }
// 1854 		      else
// 1855 			{
// 1856 			  if(find_num>2&&find_num<15) //中线黑点数大概9个左右
// 1857 			    {
// 1858               
// 1859                                 cuo_root=0;
// 1860                                 position[i]=position[i]/find_num; 
// 1861                                 find_num=0;
// 1862                            
// 1863 			    }
// 1864 			  else
// 1865 			    {
// 1866 			      if(find_num>=15)
// 1867 				{
// 1868 				  position[i]=0;
// 1869 				  find_num=0;
// 1870 				   cuo_root=1;
// 1871 			        }	 
// 1872 				position[i]=0;
// 1873 			        find_num=0;
// 1874 			    }
// 1875                            if(cuo_root==0)
// 1876                            break; 
// 1877 		          } 
// 1878 		    }
// 1879 	         }
// 1880             }
// 1881 	}
// 1882 	if(position[i])
// 1883           {
// 1884             position_old=position[i]; 
// 1885             start=1;
// 1886           }
// 1887 	else
// 1888 	  position[i]=0;
??get_root_1:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.W    R0,??DataTable56
        MOVS     R1,#+0
        STR      R1,[R0, R4, LSL #+2]
??get_root_2:
        SUBS     R4,R4,#+1
??get_root_0:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+45
        BCC.W    ??get_root_3
        LDRB     R0,[SP, #+3]
        CMP      R0,#+1
        BNE.W    ??get_root_4
        MOVS     R9,#+0
        MOVS     R2,#+1
        MOVS     R1,#+158
        LDRB     R0,[SP, #+0]
        LDRB     R3,[SP, #+2]
        SUBS     R0,R0,R3
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       limit
        STRB     R0,[SP, #+1]
        MOVS     R2,#+1
        MOVS     R1,#+158
        LDRB     R0,[SP, #+0]
        LDRB     R3,[SP, #+2]
        UXTAB    R0,R3,R0
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       limit
        MOV      R11,R0
        LDRB     R10,[SP, #+1]
        B.N      ??get_root_5
??get_root_6:
        UXTB     R7,R7            ;; ZeroExt  R7,R7,#+24,#+24
        CMP      R7,#+16
        BCC.N    ??get_root_7
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.W    R0,??DataTable56
        MOVS     R1,#+0
        STR      R1,[R0, R4, LSL #+2]
        MOVS     R7,#+0
        MOVS     R6,#+1
??get_root_7:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.W    R0,??DataTable56
        MOVS     R1,#+0
        STR      R1,[R0, R4, LSL #+2]
        MOVS     R7,#+0
??get_root_8:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.W    R0,??DataTable56
        LDR      R0,[R0, R4, LSL #+2]
        CMP      R0,#+0
        BEQ.N    ??get_root_9
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.W    R0,??DataTable56
        LDR      R0,[R0, R4, LSL #+2]
        LDRB     R1,[SP, #+0]
        SUBS     R0,R0,R1
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       abs
        CMP      R0,#+16
        BLT.N    ??get_root_9
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.W    R0,??DataTable56
        MOVS     R1,#+0
        STR      R1,[R0, R4, LSL #+2]
??get_root_9:
        UXTB     R6,R6            ;; ZeroExt  R6,R6,#+24,#+24
        CMP      R6,#+0
        BNE.N    ??get_root_10
??get_root_11:
        ADDS     R10,R10,#+1
??get_root_5:
        UXTB     R11,R11          ;; ZeroExt  R11,R11,#+24,#+24
        UXTB     R10,R10          ;; ZeroExt  R10,R10,#+24,#+24
        CMP      R11,R10
        BCC.W    ??get_root_12
        UXTB     R9,R9            ;; ZeroExt  R9,R9,#+24,#+24
        CMP      R9,#+0
        BNE.N    ??get_root_11
        UXTB     R10,R10          ;; ZeroExt  R10,R10,#+24,#+24
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        MOVS     R0,#+160
        LDR.W    R1,??DataTable55
        MLA      R0,R0,R4,R1
        LDRB     R0,[R10, R0]
        CMP      R0,#+0
        BEQ.N    ??get_root_11
        UXTB     R10,R10          ;; ZeroExt  R10,R10,#+24,#+24
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        MOVS     R0,#+160
        LDR.W    R1,??DataTable55
        MLA      R0,R0,R4,R1
        ADDS     R0,R10,R0
        LDRB     R0,[R0, #+1]
        CMP      R0,#+0
        BEQ.N    ??get_root_11
        UXTB     R10,R10          ;; ZeroExt  R10,R10,#+24,#+24
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        MOVS     R0,#+160
        LDR.W    R1,??DataTable55
        MLA      R0,R0,R4,R1
        ADDS     R0,R10,R0
        LDRB     R0,[R0, #+2]
        CMP      R0,#+0
        BNE.N    ??get_root_11
        ADDS     R10,R10,#+2
        B.N      ??get_root_13
??get_root_14:
        UXTB     R10,R10          ;; ZeroExt  R10,R10,#+24,#+24
        UXTB     R11,R11          ;; ZeroExt  R11,R11,#+24,#+24
        SUBS     R0,R11,#+1
        CMP      R10,R0
        BGE.N    ??get_root_15
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.W    R0,??DataTable56
        LDR      R0,[R0, R4, LSL #+2]
        UXTAB    R0,R0,R10
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.W    R1,??DataTable56
        STR      R0,[R1, R4, LSL #+2]
        ADDS     R7,R7,#+1
        MOVS     R9,#+1
??get_root_10:
        ADDS     R10,R10,#+1
??get_root_13:
        UXTB     R10,R10          ;; ZeroExt  R10,R10,#+24,#+24
        UXTB     R11,R11          ;; ZeroExt  R11,R11,#+24,#+24
        CMP      R10,R11
        BCS.N    ??get_root_11
        UXTB     R10,R10          ;; ZeroExt  R10,R10,#+24,#+24
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        MOVS     R0,#+160
        LDR.N    R1,??DataTable55
        MLA      R0,R0,R4,R1
        LDRB     R0,[R10, R0]
        CMP      R0,#+0
        BEQ.N    ??get_root_14
??get_root_15:
        UXTB     R10,R10          ;; ZeroExt  R10,R10,#+24,#+24
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        MOVS     R0,#+160
        LDR.N    R1,??DataTable55
        MLA      R0,R0,R4,R1
        LDRB     R0,[R10, R0]
        CMP      R0,#+0
        BNE.N    ??get_root_16
        UXTB     R10,R10          ;; ZeroExt  R10,R10,#+24,#+24
        UXTB     R11,R11          ;; ZeroExt  R11,R11,#+24,#+24
        SUBS     R0,R11,#+1
        CMP      R10,R0
        BNE.W    ??get_root_8
??get_root_16:
        UXTB     R10,R10          ;; ZeroExt  R10,R10,#+24,#+24
        UXTB     R11,R11          ;; ZeroExt  R11,R11,#+24,#+24
        SUBS     R0,R11,#+1
        CMP      R10,R0
        BNE.N    ??get_root_17
        UXTB     R10,R10          ;; ZeroExt  R10,R10,#+24,#+24
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        MOVS     R0,#+160
        LDR.N    R1,??DataTable55
        MLA      R0,R0,R4,R1
        LDRB     R0,[R10, R0]
        CMP      R0,#+0
        BNE.N    ??get_root_17
        MOV      R10,R11
        B.N      ??get_root_18
??get_root_19:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.W    R0,??DataTable56
        LDR      R0,[R0, R4, LSL #+2]
        UXTAB    R0,R0,R10
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.W    R1,??DataTable56
        STR      R0,[R1, R4, LSL #+2]
        ADDS     R7,R7,#+1
        ADDS     R10,R10,#+1
??get_root_18:
        MOVS     R2,#+1
        MOVS     R1,#+158
        UXTB     R11,R11          ;; ZeroExt  R11,R11,#+24,#+24
        ADDS     R0,R11,#+20
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       limit
        MOV      R1,R10
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        CMP      R0,R1
        BLT.N    ??get_root_17
        UXTB     R10,R10          ;; ZeroExt  R10,R10,#+24,#+24
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        MOVS     R0,#+160
        LDR.N    R1,??DataTable55
        MLA      R0,R0,R4,R1
        LDRB     R0,[R10, R0]
        CMP      R0,#+0
        BEQ.N    ??get_root_19
??get_root_17:
        UXTB     R10,R10          ;; ZeroExt  R10,R10,#+24,#+24
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        MOVS     R0,#+160
        LDR.N    R1,??DataTable55
        MLA      R0,R0,R4,R1
        LDRB     R0,[R10, R0]
        CMP      R0,#+0
        BNE.N    ??get_root_20
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.N    R0,??DataTable56
        MOVS     R1,#+0
        STR      R1,[R0, R4, LSL #+2]
        MOVS     R7,#+0
        B.N      ??get_root_11
??get_root_20:
        SUBS     R0,R7,#+3
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        CMP      R0,#+13
        BCS.W    ??get_root_6
        MOVS     R6,#+0
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.N    R0,??DataTable56
        LDR      R0,[R0, R4, LSL #+2]
        UXTB     R7,R7            ;; ZeroExt  R7,R7,#+24,#+24
        SDIV     R0,R0,R7
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.N    R1,??DataTable56
        STR      R0,[R1, R4, LSL #+2]
        MOVS     R7,#+0
        B.N      ??get_root_8
??get_root_4:
        MOVS     R9,#+0
        MOVS     R10,#+4
        B.N      ??get_root_21
??get_root_22:
        UXTB     R7,R7            ;; ZeroExt  R7,R7,#+24,#+24
        CMP      R7,#+15
        BCC.N    ??get_root_23
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.N    R0,??DataTable56
        MOVS     R1,#+0
        STR      R1,[R0, R4, LSL #+2]
        MOVS     R7,#+0
        MOVS     R5,#+1
??get_root_23:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.N    R0,??DataTable56
        MOVS     R1,#+0
        STR      R1,[R0, R4, LSL #+2]
        MOVS     R7,#+0
??get_root_24:
        UXTB     R5,R5            ;; ZeroExt  R5,R5,#+24,#+24
        CMP      R5,#+0
        BNE.N    ??get_root_25
??get_root_26:
        ADDS     R10,R10,#+1
??get_root_21:
        UXTB     R10,R10          ;; ZeroExt  R10,R10,#+24,#+24
        CMP      R10,#+155
        BCS.N    ??get_root_12
        UXTB     R9,R9            ;; ZeroExt  R9,R9,#+24,#+24
        CMP      R9,#+0
        BNE.N    ??get_root_26
        UXTB     R10,R10          ;; ZeroExt  R10,R10,#+24,#+24
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        MOVS     R0,#+160
        LDR.N    R1,??DataTable55
        MLA      R0,R0,R4,R1
        ADDS     R0,R10,R0
        LDRB     R0,[R0, #-1]
        CMP      R0,#+0
        BEQ.N    ??get_root_26
        UXTB     R10,R10          ;; ZeroExt  R10,R10,#+24,#+24
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        MOVS     R0,#+160
        LDR.N    R1,??DataTable55
        MLA      R0,R0,R4,R1
        LDRB     R0,[R10, R0]
        CMP      R0,#+0
        BEQ.N    ??get_root_26
        UXTB     R10,R10          ;; ZeroExt  R10,R10,#+24,#+24
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        MOVS     R0,#+160
        LDR.N    R1,??DataTable55
        MLA      R0,R0,R4,R1
        ADDS     R0,R10,R0
        LDRB     R0,[R0, #+1]
        CMP      R0,#+0
        BEQ.N    ??get_root_26
        UXTB     R10,R10          ;; ZeroExt  R10,R10,#+24,#+24
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        MOVS     R0,#+160
        LDR.N    R1,??DataTable55
        MLA      R0,R0,R4,R1
        ADDS     R0,R10,R0
        LDRB     R0,[R0, #+2]
        CMP      R0,#+0
        BNE.N    ??get_root_26
        ADDS     R10,R10,#+2
        B.N      ??get_root_27
??get_root_28:
        UXTB     R10,R10          ;; ZeroExt  R10,R10,#+24,#+24
        CMP      R10,#+152
        BCS.N    ??get_root_29
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.N    R0,??DataTable56
        LDR      R0,[R0, R4, LSL #+2]
        UXTAB    R0,R0,R10
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.N    R1,??DataTable56
        STR      R0,[R1, R4, LSL #+2]
        ADDS     R7,R7,#+1
        MOVS     R9,#+1
??get_root_25:
        ADDS     R10,R10,#+1
??get_root_27:
        UXTB     R10,R10          ;; ZeroExt  R10,R10,#+24,#+24
        CMP      R10,#+153
        BCS.N    ??get_root_26
        UXTB     R10,R10          ;; ZeroExt  R10,R10,#+24,#+24
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        MOVS     R0,#+160
        LDR.N    R1,??DataTable55
        MLA      R0,R0,R4,R1
        LDRB     R0,[R10, R0]
        CMP      R0,#+0
        BEQ.N    ??get_root_28
??get_root_29:
        SUBS     R0,R7,#+3
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        CMP      R0,#+12
        BCS.N    ??get_root_22
        MOVS     R5,#+0
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.N    R0,??DataTable56
        LDR      R0,[R0, R4, LSL #+2]
        UXTB     R7,R7            ;; ZeroExt  R7,R7,#+24,#+24
        SDIV     R0,R0,R7
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.N    R1,??DataTable56
        STR      R0,[R1, R4, LSL #+2]
        MOVS     R7,#+0
        B.N      ??get_root_24
??get_root_12:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.N    R0,??DataTable56
        LDR      R0,[R0, R4, LSL #+2]
        CMP      R0,#+0
        BEQ.W    ??get_root_1
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.N    R0,??DataTable56
        LDR      R0,[R0, R4, LSL #+2]
        STRB     R0,[SP, #+0]
        MOVS     R0,#+1
        STRB     R0,[SP, #+3]
        B.N      ??get_root_2
// 1889   }
// 1890 
// 1891 	
// 1892 	for(i=54;i>=45;i--)
??get_root_3:
        MOVS     R4,#+54
        B.N      ??get_root_30
// 1893           {
// 1894              if(position[i])
??get_root_31:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.N    R0,??DataTable56
        LDR      R0,[R0, R4, LSL #+2]
        CMP      R0,#+0
        BEQ.N    ??get_root_32
// 1895              {
// 1896               start_position=position[i];
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        LDR.N    R0,??DataTable56
        LDR      R0,[R0, R4, LSL #+2]
        LDR.W    R1,??DataTable57
        STR      R0,[R1, #+0]
// 1897               num++;
        ADDS     R8,R8,#+1
// 1898              }
// 1899           } 
??get_root_32:
        SUBS     R4,R4,#+1
??get_root_30:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+45
        BCS.N    ??get_root_31
// 1900 	if(num>1)
        UXTB     R8,R8            ;; ZeroExt  R8,R8,#+24,#+24
        CMP      R8,#+2
        BCC.N    ??get_root_33
// 1901 	return ok;
        MOVS     R0,#+1
        B.N      ??get_root_34
// 1902 	else
// 1903    return fail;
??get_root_33:
        MOVS     R0,#+0
??get_root_34:
        POP      {R1,R4-R11,PC}   ;; return
// 1904   
// 1905        
// 1906 }

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable54:
        DC32     black2_a3

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable54_1:
        DC32     black_a3
// 1907 

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1908 int Judge_end(unsigned char end_hang)
// 1909 {
Judge_end:
        PUSH     {R4,R5}
// 1910 	unsigned char black_num=0,i=0,j=0,black_flag1=0,black_flag2=0;
        MOVS     R1,#+0
        MOVS     R2,#+0
        MOVS     R3,#+0
        MOVS     R4,#+0
        MOVS     R5,#+0
// 1911 	if(end_hang<=19)
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        CMP      R0,#+20
        BCS.N    ??Judge_end_0
// 1912           {
// 1913             for(i=end_hang-1;i>=end_hang-1;i--)
        SUBS     R2,R0,#+1
??Judge_end_1:
        UXTB     R2,R2            ;; ZeroExt  R2,R2,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        SUBS     R3,R0,#+1
        CMP      R2,R3
        BLT.N    ??Judge_end_2
// 1914               {
// 1915                 if(!position[i])
        UXTB     R2,R2            ;; ZeroExt  R2,R2,#+24,#+24
        LDR.W    R3,??DataTable57_1
        LDR      R3,[R3, R2, LSL #+2]
        CMP      R3,#+0
        BNE.N    ??Judge_end_3
// 1916                 black_num++;  
        ADDS     R1,R1,#+1
// 1917               }	
??Judge_end_3:
        SUBS     R2,R2,#+1
        B.N      ??Judge_end_1
// 1918           }
// 1919 	else 
// 1920           if(end_hang<=28)
??Judge_end_0:
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        CMP      R0,#+29
        BCS.N    ??Judge_end_4
// 1921           {
// 1922             for(i=end_hang-1;i>=end_hang-2;i--)
        SUBS     R2,R0,#+1
        B.N      ??Judge_end_5
// 1923               {
// 1924                 if(!position[i])
??Judge_end_6:
        UXTB     R2,R2            ;; ZeroExt  R2,R2,#+24,#+24
        LDR.W    R3,??DataTable57_1
        LDR      R3,[R3, R2, LSL #+2]
        CMP      R3,#+0
        BNE.N    ??Judge_end_7
// 1925                 black_num++;  
        ADDS     R1,R1,#+1
// 1926               }
??Judge_end_7:
        SUBS     R2,R2,#+1
??Judge_end_5:
        UXTB     R2,R2            ;; ZeroExt  R2,R2,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        SUBS     R3,R0,#+2
        CMP      R2,R3
        BGE.N    ??Judge_end_6
// 1927             if(black_num<2)
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        CMP      R1,#+2
        BCS.N    ??Judge_end_2
// 1928             black_num=0;
        MOVS     R1,#+0
        B.N      ??Judge_end_2
// 1929           }
// 1930         else
// 1931           {
// 1932             for(i=end_hang-1;i>=end_hang-4;i--)
??Judge_end_4:
        SUBS     R2,R0,#+1
        B.N      ??Judge_end_8
// 1933               {
// 1934                 if(!position[i])
??Judge_end_9:
        UXTB     R2,R2            ;; ZeroExt  R2,R2,#+24,#+24
        LDR.W    R3,??DataTable57_1
        LDR      R3,[R3, R2, LSL #+2]
        CMP      R3,#+0
        BNE.N    ??Judge_end_10
// 1935                 black_num++;  
        ADDS     R1,R1,#+1
// 1936               }
??Judge_end_10:
        SUBS     R2,R2,#+1
??Judge_end_8:
        UXTB     R2,R2            ;; ZeroExt  R2,R2,#+24,#+24
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        SUBS     R3,R0,#+4
        CMP      R2,R3
        BGE.N    ??Judge_end_9
// 1937             if(black_num<4)
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        CMP      R1,#+4
        BCS.N    ??Judge_end_2
// 1938             black_num=0;
        MOVS     R1,#+0
// 1939           }
// 1940           
// 1941 	if((black_num==1)||(black_num==2)||(black_num==4))
??Judge_end_2:
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        CMP      R1,#+1
        BEQ.N    ??Judge_end_11
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        CMP      R1,#+2
        BEQ.N    ??Judge_end_11
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        CMP      R1,#+4
        BNE.N    ??Judge_end_12
// 1942 	  return ok;
??Judge_end_11:
        MOVS     R0,#+1
        B.N      ??Judge_end_13
// 1943 	else
// 1944           return fail;
??Judge_end_12:
        MOVS     R0,#+0
??Judge_end_13:
        POP      {R4,R5}
        BX       LR               ;; return
// 1945 }

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable55:
        DC32     ImageData
// 1946 
// 1947 
// 1948 

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 1949 void  position_youhua()
// 1950 {
position_youhua:
        PUSH     {R3-R5,LR}
// 1951     int m=0,end_flag=0,num=0,i=0;
        MOVS     R5,#+0
        MOVS     R0,#+0
        MOVS     R4,#+0
        MOVS     R1,#+0
// 1952 	for(m=44;m>2;m--)
        MOVS     R2,#+44
        MOVS     R5,R2
        B.N      ??position_youhua_0
// 1953         {
// 1954 	   if(!position[m])
// 1955            {
// 1956 		end_flag=Judge_end(m);
// 1957 		if(end_flag)
// 1958                 {
// 1959 		  for(;m>=3;m--)
// 1960                   {
// 1961 		    position[m]=0;
??position_youhua_1:
        LDR.W    R0,??DataTable57_1
        MOVS     R1,#+0
        STR      R1,[R0, R5, LSL #+2]
// 1962 		  }
        SUBS     R5,R5,#+1
??position_youhua_2:
        CMP      R5,#+3
        BGE.N    ??position_youhua_1
// 1963 		}
??position_youhua_3:
        SUBS     R5,R5,#+1
??position_youhua_0:
        CMP      R5,#+3
        BLT.N    ??position_youhua_4
        LDR.W    R0,??DataTable57_1
        LDR      R0,[R0, R5, LSL #+2]
        CMP      R0,#+0
        BNE.N    ??position_youhua_3
        MOVS     R0,R5
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        BL       Judge_end
        CMP      R0,#+0
        BNE.N    ??position_youhua_2
        B.N      ??position_youhua_3
// 1964 		//break;
// 1965 	   }
// 1966         } 	
// 1967 	for(i=44;i>=0;i--)
??position_youhua_4:
        MOVS     R1,#+44
        B.N      ??position_youhua_5
// 1968         {
// 1969 	  if(!position[i])
// 1970             {
// 1971               if(i>=24)
// 1972               {
// 1973 		num++;
// 1974 	        if(num>4)
// 1975                   {
// 1976 		    for(;i>=0;i--)
// 1977                      {
// 1978 		        position[i]=0;
// 1979 		     }
// 1980 			break;
// 1981 		  }
// 1982               }
// 1983               else
// 1984                 {
// 1985 		num++;
// 1986 	        if(num>2)
// 1987                   {
// 1988 		    for(;i>=0;i--)
// 1989                      {
// 1990 		        position[i]=0;
// 1991 		     }
// 1992 			break;
// 1993 		  }
// 1994               }
// 1995 	   }                	
// 1996            else
// 1997                 num=0;
??position_youhua_6:
        MOVS     R4,#+0
??position_youhua_7:
        SUBS     R1,R1,#+1
??position_youhua_5:
        CMP      R1,#+0
        BMI.N    ??position_youhua_8
        LDR.W    R0,??DataTable57_1
        LDR      R0,[R0, R1, LSL #+2]
        CMP      R0,#+0
        BNE.N    ??position_youhua_6
        CMP      R1,#+24
        BLT.N    ??position_youhua_9
        ADDS     R4,R4,#+1
        CMP      R4,#+5
        BLT.N    ??position_youhua_7
??position_youhua_10:
        CMP      R1,#+0
        BMI.N    ??position_youhua_11
        LDR.W    R0,??DataTable57_1
        MOVS     R2,#+0
        STR      R2,[R0, R1, LSL #+2]
        SUBS     R1,R1,#+1
        B.N      ??position_youhua_10
??position_youhua_11:
        B.N      ??position_youhua_8
??position_youhua_9:
        ADDS     R4,R4,#+1
        CMP      R4,#+3
        BLT.N    ??position_youhua_7
??position_youhua_12:
        CMP      R1,#+0
        BMI.N    ??position_youhua_13
        LDR.W    R0,??DataTable57_1
        MOVS     R2,#+0
        STR      R2,[R0, R1, LSL #+2]
        SUBS     R1,R1,#+1
        B.N      ??position_youhua_12
// 1998 	     
// 1999 	}
// 2000         //center_filter();
// 2001 }
??position_youhua_13:
??position_youhua_8:
        POP      {R0,R4,R5,PC}    ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable56:
        DC32     position

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable56_1:
        DC32     ption

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable56_2:
        DC32     ImageData2
// 2002 /**********************二值化END*********************/
// 2003 //////////////找黑线中心//////////////////

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 2004 int find_position()
// 2005  {
find_position:
        PUSH     {R4-R11,LR}
        SUB      SP,SP,#+28
// 2006   int i=0,j=0,windage=0,root_flag=0;
        MOVS     R8,#+0
        MOVS     R9,#+0
        MOVS     R0,#+0
        STR      R0,[SP, #+8]
        MOVS     R0,#+0
        STR      R0,[SP, #+20]
// 2007   int position_width=0;
        MOVS     R0,#+0
        STR      R0,[SP, #+4]
// 2008   int position_num=0,position_old=0;
        MOVS     R4,#+0
        MOVS     R0,#+0
        STR      R0,[SP, #+0]
// 2009   int endL=0,endR=0,num=0,ave=0,num1=0,cha=0,find_num=0;
        MOVS     R0,#+0
        STR      R0,[SP, #+16]
        MOVS     R10,#+0
        MOVS     R5,#+0
        MOVS     R11,#+0
        MOVS     R6,#+0
        MOVS     R0,#+0
        STR      R0,[SP, #+12]
        MOVS     R0,#+0
        STR      R0,[SP, #+24]
// 2010   int median=0;
        MOVS     R7,#+0
// 2011   binaryzation2();
        BL       binaryzation2
// 2012   root_flag=get_root(); //root_flag=0k;找根成功
        BL       get_root
        STR      R0,[SP, #+20]
// 2013   if(!root_flag)
        LDR      R0,[SP, #+20]
        CMP      R0,#+0
        BNE.N    ??find_position_0
// 2014   return fail;
        MOVS     R0,#+0
        B.N      ??find_position_1
// 2015   i=44;
??find_position_0:
        MOVS     R8,#+44
        B.N      ??find_position_2
// 2016   while(i>=0)
// 2017      {
// 2018         position[i]=0;
// 2019         if(i>40)
// 2020         {
// 2021           windage=40;
// 2022           position_width=4;//22
// 2023         }
// 2024        else
// 2025        if(i>30)
// 2026         {
// 2027           windage=40;
// 2028           position_width=4;//20
// 2029         } 
// 2030         else
// 2031         if(i>20)
// 2032         {
// 2033           windage=38;
// 2034           position_width=3;//20
// 2035         } 
// 2036         else
// 2037         if(i>10)
// 2038         {
// 2039           //windage=23;
// 2040           windage=17;
// 2041           position_width=3;//10
// 2042         } 
// 2043         else
// 2044         {
// 2045           //windage=21;
// 2046           windage=17;
// 2047           position_width=3;//9
// 2048         }
// 2049    if(i==44)
// 2050    {
// 2051      position_old=start_position; 
// 2052      start_position=0;
// 2053    }
// 2054  
// 2055      endL=limit(position_old-windage,158,1);
// 2056      endR= limit(position_old+windage,158,1);
// 2057      for(j=endL;j<=endR;j++)
// 2058      {
// 2059         if(!ImageData[i][j+2]&&ImageData[i][j]&&ImageData[i][j+1])
// 2060         {
// 2061                   for(j=j+2;j<=endR;j++)
// 2062                   {
// 2063                       if(!ImageData[i][j]&&j<endR)
// 2064                       {
// 2065                         median+=j;
// 2066                          position_num++;
// 2067                       }
// 2068                       else
// 2069                       {
// 2070                           if(ImageData[i][j]||(j==endR))
// 2071                           {
// 2072 	                      if(j==endR)
// 2073                                 {
// 2074                                    if(!ImageData[i][j])
// 2075                                      {
// 2076                                         for(j=endR;j<=limit(endR+20,158,1);j++)
// 2077 	                                    {
// 2078 	                                        if(!ImageData[i][j])
// 2079 		                                  {
// 2080 		                                     median+=j;
// 2081 		                                     position_num++;
// 2082 		                                  } 
// 2083 		                                else
// 2084 	                                          break;
// 2085 	                                    }
// 2086 	                             }
// 2087 	                        }
// 2088                               if(!ImageData[i][j])
// 2089                                 {
// 2090                                    median=0;
// 2091                                    position_num=0;
// 2092                                    break;
// 2093                                 }
// 2094                             if(position_num>position_width-3&&position_num<position_width+14) 
// 2095                             {
// 2096                                median=median/position_num;
// 2097                                position_num=0;
// 2098                              }
// 2099                             else
// 2100                             {
// 2101                              median=0;
// 2102                              position_num=0;	    			
// 2103                             }
// 2104                           }                 
// 2105                         if(median&&abs(median-position_old)>28) 
// 2106                             median=0;
// 2107                         else
// 2108                         {
// 2109                             if(median)
// 2110                             {			
// 2111                               if(!position[i])
// 2112                               {
// 2113                                       position[i]=median;
// 2114                                       median=0; 
// 2115                                       position_num=0;
// 2116                                        //break;
// 2117                               }
// 2118                               else
// 2119                               {
// 2120                                    if(abs(position[i]-position_old)>abs(median-position_old))
// 2121                                      {
// 2122                                         position[i]=median;
// 2123                                         median=0;
// 2124                                         break;
// 2125                                      }
// 2126                                    else
// 2127                                      {
// 2128                                        median=0;
// 2129                                        break;
// 2130                                      }
// 2131                               }
// 2132                             }
// 2133                          }
// 2134                      }                     
// 2135              } 
// 2136         }
// 2137      }
// 2138      if(position[i])
// 2139      position_old=position[i];
// 2140      if(position[i]&&(position[i]<3||position[i]>157))
// 2141     {
// 2142            for(;i>=0;i--)
// 2143            {
// 2144                position[i]=0;
// 2145            }
// 2146           break;
// 2147    }
// 2148     i--;
??find_position_3:
        SUBS     R8,R8,#+1
??find_position_2:
        CMP      R8,#+0
        BMI.W    ??find_position_4
        LDR.W    R0,??DataTable57_1
        MOVS     R1,#+0
        STR      R1,[R0, R8, LSL #+2]
        CMP      R8,#+41
        BLT.N    ??find_position_5
        MOVS     R0,#+40
        STR      R0,[SP, #+8]
        MOVS     R0,#+4
        STR      R0,[SP, #+4]
        B.N      ??find_position_6
??find_position_5:
        CMP      R8,#+31
        BLT.N    ??find_position_7
        MOVS     R0,#+40
        STR      R0,[SP, #+8]
        MOVS     R0,#+4
        STR      R0,[SP, #+4]
        B.N      ??find_position_6
??find_position_7:
        CMP      R8,#+21
        BLT.N    ??find_position_8
        MOVS     R0,#+38
        STR      R0,[SP, #+8]
        MOVS     R0,#+3
        STR      R0,[SP, #+4]
        B.N      ??find_position_6
??find_position_8:
        CMP      R8,#+11
        BLT.N    ??find_position_9
        MOVS     R0,#+17
        STR      R0,[SP, #+8]
        MOVS     R0,#+3
        STR      R0,[SP, #+4]
        B.N      ??find_position_6
??find_position_9:
        MOVS     R0,#+17
        STR      R0,[SP, #+8]
        MOVS     R0,#+3
        STR      R0,[SP, #+4]
??find_position_6:
        CMP      R8,#+44
        BNE.N    ??find_position_10
        LDR.W    R0,??DataTable57
        LDR      R0,[R0, #+0]
        STR      R0,[SP, #+0]
        LDR.W    R0,??DataTable57
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
??find_position_10:
        MOVS     R2,#+1
        MOVS     R1,#+158
        LDR      R0,[SP, #+0]
        LDR      R3,[SP, #+8]
        SUBS     R0,R0,R3
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       limit
        STR      R0,[SP, #+16]
        MOVS     R2,#+1
        MOVS     R1,#+158
        LDR      R0,[SP, #+0]
        LDR      R3,[SP, #+8]
        ADDS     R0,R3,R0
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       limit
        MOV      R10,R0
        LDR      R9,[SP, #+16]
        B.N      ??find_position_11
??find_position_12:
        MOVS     R7,#+0
??find_position_13:
        ADDS     R9,R9,#+1
??find_position_11:
        CMP      R10,R9
        BLT.W    ??find_position_14
        MOVS     R0,#+160
        LDR.W    R1,??DataTable57_2
        MLA      R0,R0,R8,R1
        ADDS     R0,R9,R0
        LDRB     R0,[R0, #+2]
        CMP      R0,#+0
        BNE.N    ??find_position_13
        MOVS     R0,#+160
        LDR.W    R1,??DataTable57_2
        MLA      R0,R0,R8,R1
        LDRB     R0,[R9, R0]
        CMP      R0,#+0
        BEQ.N    ??find_position_13
        MOVS     R0,#+160
        LDR.W    R1,??DataTable57_2
        MLA      R0,R0,R8,R1
        ADDS     R0,R9,R0
        LDRB     R0,[R0, #+1]
        CMP      R0,#+0
        BEQ.N    ??find_position_13
        ADDS     R9,R9,#+2
        B.N      ??find_position_15
??find_position_16:
        CMP      R9,R10
        BGE.N    ??find_position_17
        ADDS     R7,R9,R7
        ADDS     R4,R4,#+1
??find_position_18:
        ADDS     R9,R9,#+1
??find_position_15:
        CMP      R10,R9
        BLT.N    ??find_position_13
        MOVS     R0,#+160
        LDR.N    R1,??DataTable57_2
        MLA      R0,R0,R8,R1
        LDRB     R0,[R9, R0]
        CMP      R0,#+0
        BEQ.N    ??find_position_16
??find_position_17:
        MOVS     R0,#+160
        LDR.N    R1,??DataTable57_2
        MLA      R0,R0,R8,R1
        LDRB     R0,[R9, R0]
        CMP      R0,#+0
        BNE.N    ??find_position_19
        CMP      R9,R10
        BNE.N    ??find_position_20
??find_position_19:
        CMP      R9,R10
        BNE.N    ??find_position_21
        MOVS     R0,#+160
        LDR.N    R1,??DataTable57_2
        MLA      R0,R0,R8,R1
        LDRB     R0,[R9, R0]
        CMP      R0,#+0
        BNE.N    ??find_position_21
        MOV      R9,R10
        B.N      ??find_position_22
??find_position_23:
        ADDS     R7,R9,R7
        ADDS     R4,R4,#+1
        ADDS     R9,R9,#+1
??find_position_22:
        MOVS     R2,#+1
        MOVS     R1,#+158
        ADDS     R0,R10,#+20
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       limit
        CMP      R0,R9
        BLT.N    ??find_position_21
        MOVS     R0,#+160
        LDR.N    R1,??DataTable57_2
        MLA      R0,R0,R8,R1
        LDRB     R0,[R9, R0]
        CMP      R0,#+0
        BEQ.N    ??find_position_23
??find_position_21:
        MOVS     R0,#+160
        LDR.N    R1,??DataTable57_2
        MLA      R0,R0,R8,R1
        LDRB     R0,[R9, R0]
        CMP      R0,#+0
        BNE.N    ??find_position_24
        MOVS     R7,#+0
        MOVS     R4,#+0
        B.N      ??find_position_13
??find_position_24:
        LDR      R0,[SP, #+4]
        SUBS     R0,R0,#+3
        CMP      R0,R4
        BGE.N    ??find_position_25
        LDR      R0,[SP, #+4]
        ADDS     R0,R0,#+14
        CMP      R4,R0
        BGE.N    ??find_position_25
        SDIV     R7,R7,R4
        MOVS     R4,#+0
        B.N      ??find_position_20
??find_position_25:
        MOVS     R7,#+0
        MOVS     R4,#+0
??find_position_20:
        CMP      R7,#+0
        BEQ.N    ??find_position_26
        LDR      R0,[SP, #+0]
        SUBS     R0,R7,R0
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       abs
        CMP      R0,#+29
        BLT.N    ??find_position_26
        MOVS     R7,#+0
        B.N      ??find_position_18
??find_position_26:
        CMP      R7,#+0
        BEQ.N    ??find_position_18
        LDR.N    R0,??DataTable57_1
        LDR      R0,[R0, R8, LSL #+2]
        CMP      R0,#+0
        BNE.N    ??find_position_27
        LDR.N    R0,??DataTable57_1
        STR      R7,[R0, R8, LSL #+2]
        MOVS     R7,#+0
        MOVS     R4,#+0
        B.N      ??find_position_18
??find_position_27:
        LDR      R0,[SP, #+0]
        SUBS     R0,R7,R0
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       abs
        MOV      R11,R0
        LDR.N    R0,??DataTable57_1
        LDR      R0,[R0, R8, LSL #+2]
        LDR      R1,[SP, #+0]
        SUBS     R0,R0,R1
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       abs
        SXTH     R11,R11          ;; SignExt  R11,R11,#+16,#+16
        CMP      R11,R0
        BGE.W    ??find_position_12
        LDR.N    R0,??DataTable57_1
        STR      R7,[R0, R8, LSL #+2]
        MOVS     R7,#+0
        B.N      ??find_position_13
??find_position_14:
        LDR.N    R0,??DataTable57_1
        LDR      R0,[R0, R8, LSL #+2]
        CMP      R0,#+0
        BEQ.N    ??find_position_28
        LDR.N    R0,??DataTable57_1
        LDR      R0,[R0, R8, LSL #+2]
        STR      R0,[SP, #+0]
??find_position_28:
        LDR.N    R0,??DataTable57_1
        LDR      R0,[R0, R8, LSL #+2]
        CMP      R0,#+0
        BEQ.W    ??find_position_3
        LDR.N    R0,??DataTable57_1
        LDR      R0,[R0, R8, LSL #+2]
        CMP      R0,#+3
        BLT.N    ??find_position_29
        LDR.N    R0,??DataTable57_1
        LDR      R0,[R0, R8, LSL #+2]
        CMP      R0,#+158
        BLT.W    ??find_position_3
??find_position_29:
        CMP      R8,#+0
        BMI.N    ??find_position_30
        LDR.N    R0,??DataTable57_1
        MOVS     R1,#+0
        STR      R1,[R0, R8, LSL #+2]
        SUBS     R8,R8,#+1
        B.N      ??find_position_29
// 2149   }
// 2150   position_youhua();
??find_position_30:
??find_position_4:
        BL       position_youhua
// 2151   for(i=54;i>=4;i--)//补线
        MOVS     R8,#+54
        B.N      ??find_position_31
// 2152     {
// 2153       if(!position[i]&&(i==54))
// 2154       {
// 2155         while(!position[i])
// 2156           {
// 2157             i--;
// 2158             num1++;
// 2159           }
// 2160         if(position[i]&&num1<5)
// 2161           {
// 2162             ave=abs(position[i]-position[i-1]);
// 2163             if(position[i]>position[i-1])
// 2164               {
// 2165                 for(;i<54;i++)
// 2166                   {
// 2167                     position[i+1]=position[i]+ave;
// 2168                   }
// 2169               }
// 2170             else
// 2171               {
// 2172                 for(;i<54;i++)
// 2173                   {
// 2174                     position[i+1]=position[i]-ave;
// 2175                   }	
// 2176               }
// 2177           }
// 2178         i=i-num1+1;
// 2179         num1=0;
// 2180       }
// 2181       if(position[i]&&!position[i-1])
// 2182         {				
// 2183           while(!position[i-1])
// 2184             {
// 2185               i--;
// 2186               num++;
// 2187             }
// 2188           if(position[i-1])
// 2189             {
// 2190               if(num<5)
// 2191                 {
// 2192                   ave=abs((position[i+num]-position[i-1])/num);
// 2193                   if(position[i+num]<position[i-1])  
// 2194                     {	
// 2195                       for(;num>0;num--)
// 2196                         {
// 2197                           position[i+num-1]=position[i+num]+ave;
// 2198                         }
// 2199                       num=0;
// 2200                     }
// 2201                   else
// 2202                     {
// 2203                       for(;num>0;num--)
// 2204                         {
// 2205                           position[i+num-1]=position[i+num]-ave;
??find_position_32:
        ADDS     R0,R5,R8
        LDR.N    R1,??DataTable57_1
        ADDS     R0,R1,R0, LSL #+2
        ADDS     R1,R5,R8
        LDR.N    R2,??DataTable57_1
        LDR      R1,[R2, R1, LSL #+2]
        SUBS     R1,R1,R11
        STR      R1,[R0, #-4]
// 2206                         }
        SUBS     R5,R5,#+1
??find_position_33:
        CMP      R5,#+1
        BGE.N    ??find_position_32
// 2207                       num=0;	
        MOVS     R5,#+0
// 2208                     }
??find_position_34:
        SUBS     R8,R8,#+1
??find_position_31:
        CMP      R8,#+4
        BLT.W    ??find_position_35
        LDR.N    R0,??DataTable57_1
        LDR      R0,[R0, R8, LSL #+2]
        CMP      R0,#+0
        BNE.N    ??find_position_36
        CMP      R8,#+54
        BNE.N    ??find_position_36
        B.N      ??find_position_37
??find_position_38:
        SUBS     R8,R8,#+1
        ADDS     R6,R6,#+1
??find_position_37:
        LDR.N    R0,??DataTable57_1
        LDR      R0,[R0, R8, LSL #+2]
        CMP      R0,#+0
        BEQ.N    ??find_position_38
        LDR.N    R0,??DataTable57_1
        LDR      R0,[R0, R8, LSL #+2]
        CMP      R0,#+0
        BEQ.N    ??find_position_39
        CMP      R6,#+5
        BGE.N    ??find_position_39
        LDR.N    R0,??DataTable57_1
        LDR      R0,[R0, R8, LSL #+2]
        LDR.N    R1,??DataTable57_1
        ADDS     R1,R1,R8, LSL #+2
        LDR      R1,[R1, #-4]
        SUBS     R0,R0,R1
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       abs
        MOV      R11,R0
        LDR.N    R0,??DataTable57_1
        ADDS     R0,R0,R8, LSL #+2
        LDR      R0,[R0, #-4]
        LDR.N    R1,??DataTable57_1
        LDR      R1,[R1, R8, LSL #+2]
        CMP      R0,R1
        BGE.N    ??find_position_40
??find_position_41:
        CMP      R8,#+54
        BGE.N    ??find_position_39
        LDR.N    R0,??DataTable57_1
        LDR      R0,[R0, R8, LSL #+2]
        ADDS     R0,R11,R0
        LDR.N    R1,??DataTable57_1
        ADDS     R1,R1,R8, LSL #+2
        STR      R0,[R1, #+4]
        ADDS     R8,R8,#+1
        B.N      ??find_position_41
??find_position_42:
        LDR.N    R0,??DataTable57_1
        LDR      R0,[R0, R8, LSL #+2]
        SUBS     R0,R0,R11
        LDR.N    R1,??DataTable57_1
        ADDS     R1,R1,R8, LSL #+2
        STR      R0,[R1, #+4]
        ADDS     R8,R8,#+1
??find_position_40:
        CMP      R8,#+54
        BLT.N    ??find_position_42
??find_position_39:
        SUBS     R0,R8,R6
        ADDS     R8,R0,#+1
        MOVS     R6,#+0
??find_position_36:
        LDR.N    R0,??DataTable57_1
        LDR      R0,[R0, R8, LSL #+2]
        CMP      R0,#+0
        BEQ.N    ??find_position_34
        LDR.N    R0,??DataTable57_1
        ADDS     R0,R0,R8, LSL #+2
        LDR      R0,[R0, #-4]
        CMP      R0,#+0
        BNE.N    ??find_position_34
        B.N      ??find_position_43
??find_position_44:
        SUBS     R8,R8,#+1
        ADDS     R5,R5,#+1
??find_position_43:
        LDR.N    R0,??DataTable57_1
        ADDS     R0,R0,R8, LSL #+2
        LDR      R0,[R0, #-4]
        CMP      R0,#+0
        BEQ.N    ??find_position_44
        LDR.N    R0,??DataTable57_1
        ADDS     R0,R0,R8, LSL #+2
        LDR      R0,[R0, #-4]
        CMP      R0,#+0
        BEQ.N    ??find_position_34
        CMP      R5,#+5
        BGE.N    ??find_position_34
        ADDS     R0,R5,R8
        LDR.N    R1,??DataTable57_1
        LDR      R0,[R1, R0, LSL #+2]
        LDR.N    R1,??DataTable57_1
        ADDS     R1,R1,R8, LSL #+2
        LDR      R1,[R1, #-4]
        SUBS     R0,R0,R1
        SDIV     R0,R0,R5
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       abs
        MOV      R11,R0
        ADDS     R0,R5,R8
        LDR.N    R1,??DataTable57_1
        LDR      R0,[R1, R0, LSL #+2]
        LDR.N    R1,??DataTable57_1
        ADDS     R1,R1,R8, LSL #+2
        LDR      R1,[R1, #-4]
        CMP      R0,R1
        BGE.W    ??find_position_33
        B.N      ??find_position_45
??find_position_46:
        ADDS     R0,R5,R8
        LDR.N    R1,??DataTable57_1
        ADDS     R0,R1,R0, LSL #+2
        ADDS     R1,R5,R8
        LDR.N    R2,??DataTable57_1
        LDR      R1,[R2, R1, LSL #+2]
        ADDS     R1,R11,R1
        STR      R1,[R0, #-4]
        SUBS     R5,R5,#+1
??find_position_45:
        CMP      R5,#+1
        BGE.N    ??find_position_46
        MOVS     R5,#+0
        B.N      ??find_position_34
// 2209                 }
// 2210             }
// 2211          }
// 2212     }
// 2213 for(i=6;i>0;i--)
??find_position_35:
        MOVS     R8,#+6
        B.N      ??find_position_47
// 2214   {
// 2215      cha=abs(position[i]-position[i-1]);
// 2216      if(cha>6)
// 2217      for(;i>0;i--)
// 2218      position[i-1]=0;
??find_position_48:
        LDR.N    R0,??DataTable57_1
        ADDS     R0,R0,R8, LSL #+2
        MOVS     R1,#+0
        STR      R1,[R0, #-4]
        SUBS     R8,R8,#+1
??find_position_49:
        CMP      R8,#+1
        BGE.N    ??find_position_48
??find_position_50:
        SUBS     R8,R8,#+1
??find_position_47:
        CMP      R8,#+1
        BLT.N    ??find_position_51
        LDR.N    R0,??DataTable57_1
        LDR      R0,[R0, R8, LSL #+2]
        LDR.N    R1,??DataTable57_1
        ADDS     R1,R1,R8, LSL #+2
        LDR      R1,[R1, #-4]
        SUBS     R0,R0,R1
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       abs
        STR      R0,[SP, #+12]
        LDR      R0,[SP, #+12]
        CMP      R0,#+7
        BLT.N    ??find_position_50
        B.N      ??find_position_49
// 2219   }
// 2220   center_filter();
??find_position_51:
        BL       center_filter
// 2221   return ok;
        MOVS     R0,#+1
??find_position_1:
        ADD      SP,SP,#+28
        POP      {R4-R11,PC}      ;; return
// 2222 }

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable57:
        DC32     start_position

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable57_1:
        DC32     position

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable57_2:
        DC32     ImageData
// 2223     
// 2224 
// 2225 
// 2226 

        SECTION `.text`:CODE:NOROOT(2)
        THUMB
// 2227 int8 choose_PD()
// 2228 {
choose_PD:
        PUSH     {R4-R10,LR}
// 2229   int16 i=0,num=0;
        MOVS     R12,#+0
        MOVS     R1,#+0
// 2230   uint8 track_form=0;//j=0,
        MOVS     R4,#+0
// 2231   int8 position_now_low_P=0,position_now_P=0,position_now_D=0;
        MOVS     R9,#+0
        MOVS     R5,#+0
        MOVS     R0,#+0
// 2232   int16 line_head=0,line_end=54;//,line_mid=0;//line_head=30
        MOVS     R2,#+0
        MOVS     R3,#+54
// 2233   int16 line_mid_d=0,line_end_d=0;
        MOVS     R8,#+0
        MOVS     LR,#+0
// 2234   int16 head=0,middle=0,end=0;
        MOVS     R6,#+0
        MOVS     R7,#+0
        MOVS     R10,#+0
        B.N      ??choose_PD_0
// 2235   while(!position[i]&&i<IMG_ROWS)i++;
??choose_PD_1:
        ADDS     R12,R12,#+1
??choose_PD_0:
        SXTH     R12,R12          ;; SignExt  R12,R12,#+16,#+16
        LDR.W    R0,??choose_PD_2
        LDR      R0,[R0, R12, LSL #+2]
        CMP      R0,#+0
        BNE.N    ??choose_PD_3
        SXTH     R12,R12          ;; SignExt  R12,R12,#+16,#+16
        CMP      R12,#+55
        BLT.N    ??choose_PD_1
// 2236   dijihang=i;
??choose_PD_3:
        LDR.W    R0,??choose_PD_2+0x4
        STRH     R12,[R0, #+0]
// 2237 
// 2238   if(i<7)
        SXTH     R12,R12          ;; SignExt  R12,R12,#+16,#+16
        CMP      R12,#+7
        BGE.N    ??choose_PD_4
// 2239   {
// 2240     track_form=3;
        MOVS     R4,#+3
// 2241     Servo.P=P_High;
        LDR.W    R0,??choose_PD_2+0x8
        LDR.W    R5,??choose_PD_2+0xC
        LDRB     R5,[R5, #+0]
        STRH     R5,[R0, #+0]
// 2242     Servo.D=D_High;
        LDR.W    R0,??choose_PD_2+0x8
        LDR.W    R5,??choose_PD_2+0x10
        LDRB     R5,[R5, #+0]
        STRH     R5,[R0, #+4]
// 2243     servo.head=H_High;
        LDR.W    R0,??choose_PD_2+0x14
        LDR.W    R5,??choose_PD_2+0x18
        LDRB     R5,[R5, #+0]
        STRH     R5,[R0, #+0]
// 2244     servo.tail=T_High;
        LDR.W    R0,??choose_PD_2+0x14
        LDR.W    R5,??choose_PD_2+0x1C
        LDRB     R5,[R5, #+0]
        STRH     R5,[R0, #+2]
// 2245   }
// 2246   if(i>=7&&i<22)
??choose_PD_4:
        SUBS     R0,R12,#+7
        UXTH     R0,R0            ;; ZeroExt  R0,R0,#+16,#+16
        CMP      R0,#+15
        BCS.N    ??choose_PD_5
// 2247   {
// 2248     track_form=2;
        MOVS     R4,#+2
// 2249     Servo.P=P_Mid;
        LDR.W    R0,??choose_PD_2+0x8
        LDR.W    R5,??choose_PD_2+0x20
        LDRB     R5,[R5, #+0]
        STRH     R5,[R0, #+0]
// 2250     Servo.D=D_Mid;
        LDR.W    R0,??choose_PD_2+0x8
        LDR.W    R5,??choose_PD_2+0x24
        LDRB     R5,[R5, #+0]
        STRH     R5,[R0, #+4]
// 2251     servo.head=H_Mid;
        LDR.W    R0,??choose_PD_2+0x14
        LDR.W    R5,??choose_PD_2+0x28
        LDRB     R5,[R5, #+0]
        STRH     R5,[R0, #+0]
// 2252     servo.tail=T_Mid;
        LDR.W    R0,??choose_PD_2+0x14
        LDR.W    R5,??choose_PD_2+0x2C
        LDRB     R5,[R5, #+0]
        STRH     R5,[R0, #+2]
// 2253   }
// 2254   if(i>=22&&i<33)
??choose_PD_5:
        SUBS     R0,R12,#+22
        UXTH     R0,R0            ;; ZeroExt  R0,R0,#+16,#+16
        CMP      R0,#+11
        BCS.N    ??choose_PD_6
// 2255   {
// 2256     track_form=2;
        MOVS     R4,#+2
// 2257     Servo.P=P_Mid1;
        LDR.W    R0,??choose_PD_2+0x8
        LDR.W    R5,??choose_PD_2+0x30
        LDRB     R5,[R5, #+0]
        STRH     R5,[R0, #+0]
// 2258     Servo.D=D_Mid1;
        LDR.W    R0,??choose_PD_2+0x8
        LDR.W    R5,??choose_PD_2+0x34
        LDRB     R5,[R5, #+0]
        STRH     R5,[R0, #+4]
// 2259     servo.head=H_Mid1;
        LDR.W    R0,??choose_PD_2+0x14
        LDR.W    R5,??choose_PD_2+0x38
        LDRB     R5,[R5, #+0]
        STRH     R5,[R0, #+0]
// 2260     servo.tail=T_Mid1;
        LDR.W    R0,??choose_PD_2+0x14
        LDR.W    R5,??choose_PD_2+0x3C
        LDRB     R5,[R5, #+0]
        STRH     R5,[R0, #+2]
// 2261   }
// 2262   if(i>=33)
??choose_PD_6:
        SXTH     R12,R12          ;; SignExt  R12,R12,#+16,#+16
        CMP      R12,#+33
        BLT.N    ??choose_PD_7
// 2263   {
// 2264     track_form=1;
        MOVS     R4,#+1
// 2265     Servo.P=P_Low;
        LDR.W    R0,??choose_PD_2+0x8
        LDR.W    R5,??choose_PD_2+0x40
        LDRB     R5,[R5, #+0]
        STRH     R5,[R0, #+0]
// 2266     Servo.D=D_Low;
        LDR.W    R0,??choose_PD_2+0x8
        LDR.W    R5,??choose_PD_2+0x44
        LDRB     R5,[R5, #+0]
        STRH     R5,[R0, #+4]
// 2267     servo.head=H_Low;
        LDR.W    R0,??choose_PD_2+0x14
        LDR.W    R5,??choose_PD_2+0x48
        LDRB     R5,[R5, #+0]
        STRH     R5,[R0, #+0]
// 2268     servo.tail=T_Low;
        LDR.W    R0,??choose_PD_2+0x14
        LDR.W    R5,??choose_PD_2+0x4C
        LDRB     R5,[R5, #+0]
        STRH     R5,[R0, #+2]
// 2269   }
// 2270     for(i=servo.head;i<servo.tail;i++)
??choose_PD_7:
        LDR.W    R0,??choose_PD_2+0x14
        LDRSH    R12,[R0, #+0]
        B.N      ??choose_PD_8
// 2271     {
// 2272       if(position[i])
??choose_PD_9:
        SXTH     R12,R12          ;; SignExt  R12,R12,#+16,#+16
        LDR.W    R0,??choose_PD_2
        LDR      R0,[R0, R12, LSL #+2]
        CMP      R0,#+0
        BEQ.N    ??choose_PD_10
// 2273       {
// 2274         num++;
        ADDS     R1,R1,#+1
// 2275         position_now+=position[i];
        LDR.W    R0,??choose_PD_2+0x50
        LDRH     R0,[R0, #+0]
        SXTH     R12,R12          ;; SignExt  R12,R12,#+16,#+16
        LDR.W    R5,??choose_PD_2
        LDR      R5,[R5, R12, LSL #+2]
        ADDS     R0,R5,R0
        LDR.W    R5,??choose_PD_2+0x50
        STRH     R0,[R5, #+0]
// 2276       }  
// 2277     }
??choose_PD_10:
        ADDS     R12,R12,#+1
??choose_PD_8:
        LDR.W    R0,??choose_PD_2+0x14
        LDRSH    R0,[R0, #+2]
        SXTH     R12,R12          ;; SignExt  R12,R12,#+16,#+16
        CMP      R12,R0
        BLT.N    ??choose_PD_9
// 2278     if(num!=0)
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        CMP      R1,#+0
        BEQ.N    ??choose_PD_11
// 2279     {
// 2280       position_now=position_now/num;   //算出当场的中心位置
        LDR.W    R0,??choose_PD_2+0x50
        LDRSH    R0,[R0, #+0]
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        SDIV     R0,R0,R1
        LDR.W    R1,??choose_PD_2+0x50
        STRH     R0,[R1, #+0]
// 2281       num=0;
        MOVS     R1,#+0
// 2282     }
// 2283   // for(i=45;i<55;i++)
// 2284       for(i=50;i<55;i++)
??choose_PD_11:
        MOVS     R12,#+50
        B.N      ??choose_PD_12
// 2285     {
// 2286       if(position[i])
??choose_PD_13:
        SXTH     R12,R12          ;; SignExt  R12,R12,#+16,#+16
        LDR.W    R0,??choose_PD_2
        LDR      R0,[R0, R12, LSL #+2]
        CMP      R0,#+0
        BEQ.N    ??choose_PD_14
// 2287       {
// 2288         num++;
        ADDS     R1,R1,#+1
// 2289         position_now_low+=position[i];
        LDR.W    R0,??choose_PD_2+0x54
        LDRH     R0,[R0, #+0]
        SXTH     R12,R12          ;; SignExt  R12,R12,#+16,#+16
        LDR.W    R5,??choose_PD_2
        LDR      R5,[R5, R12, LSL #+2]
        ADDS     R0,R5,R0
        LDR.W    R5,??choose_PD_2+0x54
        STRH     R0,[R5, #+0]
// 2290       }  
// 2291     }
??choose_PD_14:
        ADDS     R12,R12,#+1
??choose_PD_12:
        SXTH     R12,R12          ;; SignExt  R12,R12,#+16,#+16
        CMP      R12,#+55
        BLT.N    ??choose_PD_13
// 2292     if(num!=0)
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        CMP      R1,#+0
        BEQ.N    ??choose_PD_15
// 2293     {
// 2294       position_now_low=position_now_low/num;
        LDR.W    R0,??choose_PD_2+0x54
        LDRSH    R0,[R0, #+0]
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        SDIV     R0,R0,R1
        LDR.W    R1,??choose_PD_2+0x54
        STRH     R0,[R1, #+0]
// 2295       num=0;
        MOVS     R1,#+0
// 2296     }
// 2297     if(!position_now)
??choose_PD_15:
        LDR.W    R0,??choose_PD_2+0x50
        LDRSH    R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??choose_PD_16
// 2298     {
// 2299       if(!position_now_low)
        LDR.W    R0,??choose_PD_2+0x54
        LDRSH    R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??choose_PD_17
// 2300         position_now=position_now_low;
        LDR.W    R0,??choose_PD_2+0x50
        LDR.W    R1,??choose_PD_2+0x54
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
        B.N      ??choose_PD_16
// 2301       else
// 2302         return fail;
??choose_PD_17:
        MOVS     R0,#+0
        B.W      ??choose_PD_18
// 2303     }
// 2304     while(position[line_head]==0)  line_head++;
??choose_PD_19:
        ADDS     R2,R2,#+1
??choose_PD_16:
        SXTH     R2,R2            ;; SignExt  R2,R2,#+16,#+16
        LDR.W    R0,??choose_PD_2
        LDR      R0,[R0, R2, LSL #+2]
        CMP      R0,#+0
        BEQ.N    ??choose_PD_19
// 2305     while(position[line_end]==0)  line_end--;
??choose_PD_20:
        SXTH     R3,R3            ;; SignExt  R3,R3,#+16,#+16
        LDR.W    R0,??choose_PD_2
        LDR      R0,[R0, R3, LSL #+2]
        CMP      R0,#+0
        BNE.N    ??choose_PD_21
        SUBS     R3,R3,#+1
        B.N      ??choose_PD_20
// 2306     line_end_d=(line_head+2*line_end)/3;
??choose_PD_21:
        SXTH     R3,R3            ;; SignExt  R3,R3,#+16,#+16
        LSLS     R0,R3,#+1
        SXTAH    R0,R0,R2
        MOVS     R1,#+3
        SDIV     LR,R0,R1
// 2307     line_mid_d=(line_end+2*line_head)/3;
        SXTH     R2,R2            ;; SignExt  R2,R2,#+16,#+16
        LSLS     R0,R2,#+1
        SXTAH    R0,R0,R3
        MOVS     R1,#+3
        SDIV     R8,R0,R1
// 2308     for(i=line_head;i<line_mid_d;i++)head+=position[i];
        MOV      R12,R2
        B.N      ??choose_PD_22
??choose_PD_23:
        SXTH     R12,R12          ;; SignExt  R12,R12,#+16,#+16
        LDR.W    R0,??choose_PD_2
        LDR      R0,[R0, R12, LSL #+2]
        ADDS     R6,R0,R6
        ADDS     R12,R12,#+1
??choose_PD_22:
        SXTH     R12,R12          ;; SignExt  R12,R12,#+16,#+16
        SXTH     R8,R8            ;; SignExt  R8,R8,#+16,#+16
        CMP      R12,R8
        BLT.N    ??choose_PD_23
// 2309     head=head/(line_mid_d-line_head);
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        SXTH     R8,R8            ;; SignExt  R8,R8,#+16,#+16
        SXTH     R2,R2            ;; SignExt  R2,R2,#+16,#+16
        SUBS     R0,R8,R2
        SDIV     R6,R6,R0
// 2310     for(i=line_mid_d;i<=line_end_d;i++)middle+=position[i];
        MOV      R12,R8
        B.N      ??choose_PD_24
??choose_PD_25:
        SXTH     R12,R12          ;; SignExt  R12,R12,#+16,#+16
        LDR.W    R0,??choose_PD_2
        LDR      R0,[R0, R12, LSL #+2]
        ADDS     R7,R0,R7
        ADDS     R12,R12,#+1
??choose_PD_24:
        SXTH     LR,LR            ;; SignExt  LR,LR,#+16,#+16
        SXTH     R12,R12          ;; SignExt  R12,R12,#+16,#+16
        CMP      LR,R12
        BGE.N    ??choose_PD_25
// 2311     middle=middle/(line_end_d-line_mid_d+1); 
        SXTH     R7,R7            ;; SignExt  R7,R7,#+16,#+16
        SXTH     LR,LR            ;; SignExt  LR,LR,#+16,#+16
        SXTH     R8,R8            ;; SignExt  R8,R8,#+16,#+16
        SUBS     R0,LR,R8
        ADDS     R0,R0,#+1
        SDIV     R7,R7,R0
// 2312    if(dijihang<=50)
        LDR.W    R0,??choose_PD_2+0x4
        LDRSH    R0,[R0, #+0]
        CMP      R0,#+51
        BGE.N    ??choose_PD_26
// 2313    {
// 2314      if(dijihang<=21)
        LDR.W    R0,??choose_PD_2+0x4
        LDRSH    R0,[R0, #+0]
        CMP      R0,#+22
        BGE.N    ??choose_PD_27
// 2315       position_diff=(middle-head); 
        SUBS     R0,R7,R6
        LDR.W    R1,??choose_PD_28
        STRH     R0,[R1, #+0]
        B.N      ??choose_PD_29
// 2316      else
// 2317        position_diff=middle-head;
??choose_PD_27:
        SUBS     R0,R7,R6
        LDR.W    R1,??choose_PD_28
        STRH     R0,[R1, #+0]
        B.N      ??choose_PD_29
// 2318    }
// 2319     else
// 2320      position_diff=position[line_end]-position[line_head];
??choose_PD_26:
        SXTH     R3,R3            ;; SignExt  R3,R3,#+16,#+16
        LDR.W    R0,??choose_PD_2
        LDR      R0,[R0, R3, LSL #+2]
        SXTH     R2,R2            ;; SignExt  R2,R2,#+16,#+16
        LDR.W    R1,??choose_PD_2
        LDR      R1,[R1, R2, LSL #+2]
        SUBS     R0,R0,R1
        LDR.W    R1,??choose_PD_28
        STRH     R0,[R1, #+0]
// 2321       
// 2322   
// 2323     //position_now_low_P=stages_low_P(position_now_low,video_Middle);
// 2324     position_now_P=stages_P(position_now,video_Middle);
??choose_PD_29:
        MOVS     R1,#+1
        LDR.W    R0,??choose_PD_2+0x50
        LDRSH    R0,[R0, #+0]
        BL       stages_P
        MOVS     R5,R0
// 2325     position_now_D=stages_D(position_diff);
        LDR.W    R0,??choose_PD_28
        LDRSH    R0,[R0, #+0]
        BL       stages_D
// 2326   switch(position_now_P)
        UXTB     R5,R5            ;; ZeroExt  R5,R5,#+24,#+24
        CMP      R5,#+0
        BEQ.N    ??choose_PD_30
        CMP      R5,#+2
        BEQ.W    ??choose_PD_31
        BCC.W    ??choose_PD_32
        CMP      R5,#+4
        BEQ.W    ??choose_PD_33
        BCC.W    ??choose_PD_34
        CMP      R5,#+6
        BEQ.W    ??choose_PD_35
        BCC.W    ??choose_PD_36
        B.W      ??choose_PD_37
// 2327   {
// 2328   case -6:
// 2329     {
// 2330       switch(position_now_D)
// 2331       {
// 2332      case -6:
// 2333         {
// 2334           switch(track_form)
// 2335           {
// 2336         //case 1:Servo.P+=5;Servo.D+=1;break;//-6-6
// 2337           case 1:Servo.P+=4;Servo.D+=2;break;
// 2338           case 2:Servo.P+=3;Servo.D+=1;break;
// 2339           case 3:Servo.P+=2;Servo.D+=0;break;
// 2340           }
// 2341         }break;
// 2342       case -5:
// 2343         {
// 2344           switch(track_form)
// 2345           {
// 2346             //case 1:Servo.P+=5;Servo.D+=1;break;
// 2347           case 1:Servo.P+=4;Servo.D+=1;break;
// 2348           case 2:Servo.P+=3;Servo.D+=0;break;
// 2349           case 3:Servo.P+=2;Servo.D-=1;break;
// 2350           }
// 2351         }break;
// 2352       case -4:
// 2353         {
// 2354           switch(track_form)
// 2355           {
// 2356             case 1:Servo.P+=4;Servo.D+=0;break;
// 2357             case 2:Servo.P+=3;Servo.D-=1;break;
// 2358             case 3:Servo.P+=2;Servo.D-=2;break;
// 2359           }
// 2360         }break;
// 2361       case -3:
// 2362         {
// 2363           switch(track_form)
// 2364           {
// 2365           case 1:Servo.P+=4;Servo.D-=1;break;
// 2366           case 2:Servo.P+=3;Servo.D-=2;break;
// 2367           case 3:Servo.P+=2;Servo.D-=3;break;
// 2368           }
// 2369         }break;
// 2370       case -2:
// 2371         {
// 2372           switch(track_form)
// 2373           {
// 2374           case 1:Servo.P+=4;Servo.D-=2;break;
// 2375           case 2:Servo.P+=3;Servo.D-=3;break;
// 2376           case 3:Servo.P+=2;Servo.D-=4;break;
// 2377           }
// 2378         }break;
// 2379       case -1:
// 2380         {
// 2381           switch(track_form)
// 2382           {
// 2383           case 1:Servo.P+=4;Servo.D-=3;break;
// 2384           case 2:Servo.P+=3;Servo.D-=4;break;
// 2385           case 3:Servo.P+=2;Servo.D-=5;break;
// 2386           }
// 2387         }break;
// 2388       case 0:
// 2389          {
// 2390           switch(track_form)
// 2391           {
// 2392           case 1:Servo.P+=4;break;
// 2393           case 2:Servo.P+=3;break;
// 2394           case 3:Servo.P+=2;break;
// 2395           }
// 2396         }break;
// 2397       case 6:
// 2398         {
// 2399           switch(track_form)
// 2400           {
// 2401         //case 1:Servo.P+=5;Servo.D+=1;break;//-6-6
// 2402           case 1:Servo.P+=4;Servo.D+=2;break;
// 2403           case 2:Servo.P+=3;Servo.D+=1;break;
// 2404           case 3:Servo.P+=2;Servo.D+=0;break;
// 2405           }
// 2406         }break;
// 2407       case 5:
// 2408         {
// 2409           switch(track_form)
// 2410           {
// 2411             //case 1:Servo.P+=5;Servo.D+=1;break;
// 2412           case 1:Servo.P+=4;Servo.D+=1;break;
// 2413           case 2:Servo.P+=3;Servo.D+=0;break;
// 2414           case 3:Servo.P+=2;Servo.D-=1;break;
// 2415           }
// 2416         }break;
// 2417       case 4:
// 2418         {
// 2419           switch(track_form)
// 2420           {
// 2421             case 1:Servo.P+=4;Servo.D+=0;break;
// 2422             case 2:Servo.P+=3;Servo.D-=1;break;
// 2423             case 3:Servo.P+=2;Servo.D-=2;break;
// 2424           }
// 2425         }break;
// 2426       case 3:
// 2427         {
// 2428           switch(track_form)
// 2429           {
// 2430           case 1:Servo.P+=4;Servo.D-=1;break;
// 2431           case 2:Servo.P+=3;Servo.D-=2;break;
// 2432           case 3:Servo.P+=2;Servo.D-=3;break;
// 2433           }
// 2434         }break;
// 2435       case 2:
// 2436         {
// 2437           switch(track_form)
// 2438           {
// 2439           case 1:Servo.P+=4;Servo.D-=2;break;
// 2440           case 2:Servo.P+=3;Servo.D-=3;break;
// 2441           case 3:Servo.P+=2;Servo.D-=4;break;
// 2442           }
// 2443         }break;
// 2444       case 1:
// 2445         {
// 2446           switch(track_form)
// 2447           {
// 2448           case 1:Servo.P+=4;Servo.D-=3;break;
// 2449           case 2:Servo.P+=3;Servo.D-=4;break;
// 2450           case 3:Servo.P+=2;Servo.D-=5;break;
// 2451           }
// 2452         }break;
// 2453       }
// 2454     }break;
// 2455   case -5:
// 2456     {
// 2457       switch(position_now_D)
// 2458       {
// 2459       case -6:
// 2460         {
// 2461           switch(track_form)
// 2462           {
// 2463         //case 1:Servo.P+=5;Servo.D+=1;break;//-6-6
// 2464           case 1:Servo.P+=3;Servo.D+=2;break;
// 2465           case 2:Servo.P+=2;Servo.D+=1;break;
// 2466           case 3:Servo.P+=1;Servo.D+=0;break;
// 2467           }
// 2468         }break;
// 2469       case -5:
// 2470         {
// 2471           switch(track_form)
// 2472           {
// 2473             //case 1:Servo.P+=5;Servo.D+=1;break;
// 2474           case 1:Servo.P+=3;Servo.D+=1;break;
// 2475           case 2:Servo.P+=2;Servo.D+=0;break;
// 2476           case 3:Servo.P+=1;Servo.D-=1;break;
// 2477           }
// 2478         }break;
// 2479       case -4:
// 2480         {
// 2481           switch(track_form)
// 2482           {
// 2483             case 1:Servo.P+=3;Servo.D+=0;break;
// 2484             case 2:Servo.P+=2;Servo.D-=1;break;
// 2485             case 3:Servo.P+=1;Servo.D-=2;break;
// 2486           }
// 2487         }break;
// 2488       case -3:
// 2489         {
// 2490           switch(track_form)
// 2491           {
// 2492           case 1:Servo.P+=3;Servo.D-=1;break;
// 2493           case 2:Servo.P+=2;Servo.D-=2;break;
// 2494           case 3:Servo.P+=1;Servo.D-=3;break;
// 2495           }
// 2496         }break;
// 2497       case -2:
// 2498         {
// 2499           switch(track_form)
// 2500           {
// 2501           case 1:Servo.P+=3;Servo.D-=2;break;
// 2502           case 2:Servo.P+=2;Servo.D-=3;break;
// 2503           case 3:Servo.P+=1;Servo.D-=4;break;
// 2504           }
// 2505         }break;
// 2506       case -1:
// 2507         {
// 2508           switch(track_form)
// 2509           {
// 2510           case 1:Servo.P+=3;Servo.D-=3;break;
// 2511           case 2:Servo.P+=2;Servo.D-=4;break;
// 2512           case 3:Servo.P+=1;Servo.D-=5;break;
// 2513           }
// 2514         }break;
// 2515       case 0:
// 2516          {
// 2517           switch(track_form)
// 2518           {
// 2519           case 1:Servo.P+=3;break;
// 2520           case 2:Servo.P+=2;break;
// 2521           case 3:Servo.P+=1;break;
// 2522           }
// 2523         }break;
// 2524       case 6:
// 2525         {
// 2526           switch(track_form)
// 2527           {
// 2528         //case 1:Servo.P+=5;Servo.D+=1;break;//-6-6
// 2529           case 1:Servo.P+=3;Servo.D+=2;break;
// 2530           case 2:Servo.P+=2;Servo.D+=1;break;
// 2531           case 3:Servo.P+=1;Servo.D+=0;break;
// 2532           }
// 2533         }break;
// 2534       case 5:
// 2535         {
// 2536           switch(track_form)
// 2537           {
// 2538             //case 1:Servo.P+=5;Servo.D+=1;break;
// 2539           case 1:Servo.P+=3;Servo.D+=1;break;
// 2540           case 2:Servo.P+=2;Servo.D+=0;break;
// 2541           case 3:Servo.P+=1;Servo.D-=1;break;
// 2542           }
// 2543         }break;
// 2544       case 4:
// 2545         {
// 2546           switch(track_form)
// 2547           {
// 2548             case 1:Servo.P+=3;Servo.D+=0;break;
// 2549             case 2:Servo.P+=2;Servo.D-=1;break;
// 2550             case 3:Servo.P+=1;Servo.D-=2;break;
// 2551           }
// 2552         }break;
// 2553       case 3:
// 2554         {
// 2555           switch(track_form)
// 2556           {
// 2557           case 1:Servo.P+=3;Servo.D-=1;break;
// 2558           case 2:Servo.P+=2;Servo.D-=2;break;
// 2559           case 3:Servo.P+=1;Servo.D-=3;break;
// 2560           }
// 2561         }break;
// 2562       case 2:
// 2563         {
// 2564           switch(track_form)
// 2565           {
// 2566           case 1:Servo.P+=3;Servo.D-=2;break;
// 2567           case 2:Servo.P+=2;Servo.D-=3;break;
// 2568           case 3:Servo.P+=1;Servo.D-=4;break;
// 2569           }
// 2570         }break;
// 2571       case 1:
// 2572         {
// 2573           switch(track_form)
// 2574           {
// 2575           case 1:Servo.P+=3;Servo.D-=3;break;
// 2576           case 2:Servo.P+=2;Servo.D-=4;break;
// 2577           case 3:Servo.P+=1;Servo.D-=5;break;
// 2578           }
// 2579         }break;
// 2580       }
// 2581     }break;
// 2582   case -4:
// 2583     {
// 2584       switch(position_now_D)
// 2585       {
// 2586          case -6:
// 2587         {
// 2588           switch(track_form)
// 2589           {
// 2590         //case 1:Servo.P+=5;Servo.D+=1;break;//-6-6
// 2591           case 1:Servo.P+=2;Servo.D+=2;break;
// 2592           case 2:Servo.P+=1;Servo.D+=1;break;
// 2593           case 3:Servo.P+=0;Servo.D+=0;break;
// 2594           }
// 2595         }break;
// 2596       case -5:
// 2597         {
// 2598           switch(track_form)
// 2599           {
// 2600             //case 1:Servo.P+=5;Servo.D+=1;break;
// 2601           case 1:Servo.P+=2;Servo.D+=1;break;
// 2602           case 2:Servo.P+=1;Servo.D+=0;break;
// 2603           case 3:Servo.P+=0;Servo.D-=1;break;
// 2604           }
// 2605         }break;
// 2606       case -4:
// 2607         {
// 2608           switch(track_form)
// 2609           {
// 2610             case 1:Servo.P+=2;Servo.D+=0;break;
// 2611             case 2:Servo.P+=1;Servo.D-=1;break;
// 2612             case 3:Servo.P+=0;Servo.D-=2;break;
// 2613           }
// 2614         }break;
// 2615       case -3:
// 2616         {
// 2617           switch(track_form)
// 2618           {
// 2619           case 1:Servo.P+=2;Servo.D-=1;break;
// 2620           case 2:Servo.P+=1;Servo.D-=2;break;
// 2621           case 3:Servo.P+=0;Servo.D-=3;break;
// 2622           }
// 2623         }break;
// 2624       case -2:
// 2625         {
// 2626           switch(track_form)
// 2627           {
// 2628           case 1:Servo.P+=2;Servo.D-=2;break;
// 2629           case 2:Servo.P+=1;Servo.D-=3;break;
// 2630           case 3:Servo.P+=0;Servo.D-=4;break;
// 2631           }
// 2632         }break;
// 2633       case -1:
// 2634         {
// 2635           switch(track_form)
// 2636           {
// 2637           case 1:Servo.P+=2;Servo.D-=3;break;
// 2638           case 2:Servo.P+=1;Servo.D-=4;break;
// 2639           case 3:Servo.P+=0;Servo.D-=5;break;
// 2640           }
// 2641         }break;
// 2642       case 0:
// 2643          {
// 2644           switch(track_form)
// 2645           {
// 2646           case 1:Servo.P+=2;break;
// 2647           case 2:Servo.P+=1;break;
// 2648           case 3:Servo.P+=0;break;
// 2649           }
// 2650         }break;
// 2651       case 6:
// 2652         {
// 2653           switch(track_form)
// 2654           {
// 2655         //case 1:Servo.P+=5;Servo.D+=1;break;//-6-6
// 2656           case 1:Servo.P+=2;Servo.D+=2;break;
// 2657           case 2:Servo.P+=1;Servo.D+=1;break;
// 2658           case 3:Servo.P+=0;Servo.D+=0;break;
// 2659           }
// 2660         }break;
// 2661       case 5:
// 2662         {
// 2663           switch(track_form)
// 2664           {
// 2665             //case 1:Servo.P+=5;Servo.D+=1;break;
// 2666           case 1:Servo.P+=2;Servo.D+=1;break;
// 2667           case 2:Servo.P+=1;Servo.D+=0;break;
// 2668           case 3:Servo.P+=0;Servo.D-=1;break;
// 2669           }
// 2670         }break;
// 2671       case 4:
// 2672         {
// 2673           switch(track_form)
// 2674           {
// 2675             case 1:Servo.P+=2;Servo.D+=0;break;
// 2676             case 2:Servo.P+=1;Servo.D-=1;break;
// 2677             case 3:Servo.P+=0;Servo.D-=2;break;
// 2678           }
// 2679         }break;
// 2680       case 3:
// 2681         {
// 2682           switch(track_form)
// 2683           {
// 2684           case 1:Servo.P+=2;Servo.D-=1;break;
// 2685           case 2:Servo.P+=1;Servo.D-=2;break;
// 2686           case 3:Servo.P+=0;Servo.D-=3;break;
// 2687           }
// 2688         }break;
// 2689       case 2:
// 2690         {
// 2691           switch(track_form)
// 2692           {
// 2693           case 1:Servo.P+=2;Servo.D-=2;break;
// 2694           case 2:Servo.P+=1;Servo.D-=3;break;
// 2695           case 3:Servo.P+=0;Servo.D-=4;break;
// 2696           }
// 2697         }break;
// 2698       case 1:
// 2699         {
// 2700           switch(track_form)
// 2701           {
// 2702           case 1:Servo.P+=2;Servo.D-=3;break;
// 2703           case 2:Servo.P+=1;Servo.D-=4;break;
// 2704           case 3:Servo.P+=0;Servo.D-=5;break;
// 2705           }
// 2706         }break;
// 2707       }
// 2708     }break;
// 2709  case -3:
// 2710     {
// 2711       switch(position_now_D)
// 2712       {
// 2713    case -6:
// 2714         {
// 2715           switch(track_form)
// 2716           {
// 2717         //case 1:Servo.P+=5;Servo.D+=1;break;//-6-6
// 2718           case 1:Servo.P+=1;Servo.D+=2;break;
// 2719           case 2:Servo.P+=0;Servo.D+=1;break;
// 2720           case 3:Servo.P-=1;Servo.D+=0;break;
// 2721           }
// 2722         }break;
// 2723       case -5:
// 2724         {
// 2725           switch(track_form)
// 2726           {
// 2727             //case 1:Servo.P+=5;Servo.D+=1;break;
// 2728           case 1:Servo.P+=1;Servo.D+=1;break;
// 2729           case 2:Servo.P+=0;Servo.D+=0;break;
// 2730           case 3:Servo.P-=1;Servo.D-=1;break;
// 2731           }
// 2732         }break;
// 2733       case -4:
// 2734         {
// 2735           switch(track_form)
// 2736           {
// 2737             case 1:Servo.P+=1;Servo.D+=0;break;
// 2738             case 2:Servo.P+=0;Servo.D-=1;break;
// 2739             case 3:Servo.P-=1;Servo.D-=2;break;
// 2740           }
// 2741         }break;
// 2742       case -3:
// 2743         {
// 2744           switch(track_form)
// 2745           {
// 2746           case 1:Servo.P+=1;Servo.D-=1;break;
// 2747           case 2:Servo.P+=0;Servo.D-=2;break;
// 2748           case 3:Servo.P-=1;Servo.D-=3;break;
// 2749           }
// 2750         }break;
// 2751       case -2:
// 2752         {
// 2753           switch(track_form)
// 2754           {
// 2755           case 1:Servo.P+=1;Servo.D-=2;break;
// 2756           case 2:Servo.P+=0;Servo.D-=3;break;
// 2757           case 3:Servo.P-=1;Servo.D-=4;break;
// 2758           }
// 2759         }break;
// 2760       case -1:
// 2761         {
// 2762           switch(track_form)
// 2763           {
// 2764           case 1:Servo.P+=1;Servo.D-=3;break;
// 2765           case 2:Servo.P+=0;Servo.D-=4;break;
// 2766           case 3:Servo.P-=1;Servo.D-=5;break;
// 2767           }
// 2768         }break;
// 2769       case 0:
// 2770          {
// 2771           switch(track_form)
// 2772           {
// 2773           case 1:Servo.P+=1;break;
// 2774           case 2:Servo.P+=0;break;
// 2775           case 3:Servo.P-=1;break;
// 2776           }
// 2777         }break;
// 2778       case 6:
// 2779         {
// 2780           switch(track_form)
// 2781           {
// 2782         //case 1:Servo.P+=5;Servo.D+=1;break;//-6-6
// 2783           case 1:Servo.P+=1;Servo.D+=2;break;
// 2784           case 2:Servo.P+=0;Servo.D+=1;break;
// 2785           case 3:Servo.P-=1;Servo.D+=0;break;
// 2786           }
// 2787         }break;
// 2788       case 5:
// 2789         {
// 2790           switch(track_form)
// 2791           {
// 2792             //case 1:Servo.P+=5;Servo.D+=1;break;
// 2793           case 1:Servo.P+=1;Servo.D+=1;break;
// 2794           case 2:Servo.P+=0;Servo.D+=0;break;
// 2795           case 3:Servo.P-=1;Servo.D-=1;break;
// 2796           }
// 2797         }break;
// 2798       case 4:
// 2799         {
// 2800           switch(track_form)
// 2801           {
// 2802             case 1:Servo.P+=1;Servo.D+=0;break;
// 2803             case 2:Servo.P+=0;Servo.D-=1;break;
// 2804             case 3:Servo.P-=1;Servo.D-=2;break;
// 2805           }
// 2806         }break;
// 2807       case 3:
// 2808         {
// 2809           switch(track_form)
// 2810           {
// 2811           case 1:Servo.P+=1;Servo.D-=1;break;
// 2812           case 2:Servo.P+=0;Servo.D-=2;break;
// 2813           case 3:Servo.P-=1;Servo.D-=3;break;
// 2814           }
// 2815         }break;
// 2816       case 2:
// 2817         {
// 2818           switch(track_form)
// 2819           {
// 2820           case 1:Servo.P+=1;Servo.D-=2;break;
// 2821           case 2:Servo.P+=0;Servo.D-=3;break;
// 2822           case 3:Servo.P-=1;Servo.D-=4;break;
// 2823           }
// 2824         }break;
// 2825       case 1:
// 2826         {
// 2827           switch(track_form)
// 2828           {
// 2829           case 1:Servo.P+=1;Servo.D-=3;break;
// 2830           case 2:Servo.P+=0;Servo.D-=4;break;
// 2831           case 3:Servo.P-=1;Servo.D-=5;break;
// 2832           }
// 2833         }break;
// 2834       }
// 2835     }break;
// 2836  case -2:
// 2837     {
// 2838       switch(position_now_D)
// 2839       {
// 2840      case -6:
// 2841         {
// 2842           switch(track_form)
// 2843           {
// 2844         //case 1:Servo.P+=5;Servo.D+=1;break;//-6-6
// 2845           case 1:Servo.P+=0;Servo.D+=2;break;
// 2846           case 2:Servo.P-=1;Servo.D+=1;break;
// 2847           case 3:Servo.P-=2;Servo.D+=0;break;
// 2848           }
// 2849         }break;
// 2850       case -5:
// 2851         {
// 2852           switch(track_form)
// 2853           {
// 2854             //case 1:Servo.P+=5;Servo.D+=1;break;
// 2855           case 1:Servo.P+=0;Servo.D+=1;break;
// 2856           case 2:Servo.P-=1;Servo.D+=0;break;
// 2857           case 3:Servo.P-=2;Servo.D-=1;break;
// 2858           }
// 2859         }break;
// 2860       case -4:
// 2861         {
// 2862           switch(track_form)
// 2863           {
// 2864             case 1:Servo.P+=0;Servo.D+=0;break;
// 2865             case 2:Servo.P-=1;Servo.D-=1;break;
// 2866             case 3:Servo.P-=2;Servo.D-=2;break;
// 2867           }
// 2868         }break;
// 2869       case -3:
// 2870         {
// 2871           switch(track_form)
// 2872           {
// 2873           case 1:Servo.P+=0;Servo.D-=1;break;
// 2874           case 2:Servo.P-=1;Servo.D-=2;break;
// 2875           case 3:Servo.P-=2;Servo.D-=3;break;
// 2876           }
// 2877         }break;
// 2878       case -2:
// 2879         {
// 2880           switch(track_form)
// 2881           {
// 2882           case 1:Servo.P+=0;Servo.D-=2;break;
// 2883           case 2:Servo.P-=1;Servo.D-=3;break;
// 2884           case 3:Servo.P-=2;Servo.D-=4;break;
// 2885           }
// 2886         }break;
// 2887       case -1:
// 2888         {
// 2889           switch(track_form)
// 2890           {
// 2891           case 1:Servo.P+=0;Servo.D-=3;break;
// 2892           case 2:Servo.P-=1;Servo.D-=4;break;
// 2893           case 3:Servo.P-=2;Servo.D-=5;break;
// 2894           }
// 2895         }break;      
// 2896       case 0:
// 2897          {
// 2898           switch(track_form)
// 2899           {
// 2900           case 1:Servo.P+=0;break;
// 2901           case 2:Servo.P-=1;break;
// 2902           case 3:Servo.P-=2;break;
// 2903           }
// 2904         }break;
// 2905       case 6:
// 2906         {
// 2907           switch(track_form)
// 2908           {
// 2909         //case 1:Servo.P+=5;Servo.D+=1;break;//-6-6
// 2910           case 1:Servo.P+=0;Servo.D+=2;break;
// 2911           case 2:Servo.P-=1;Servo.D+=1;break;
// 2912           case 3:Servo.P-=2;Servo.D+=0;break;
// 2913           }
// 2914         }break;
// 2915       case 5:
// 2916         {
// 2917           switch(track_form)
// 2918           {
// 2919             //case 1:Servo.P+=5;Servo.D+=1;break;
// 2920           case 1:Servo.P+=0;Servo.D+=1;break;
// 2921           case 2:Servo.P-=1;Servo.D+=0;break;
// 2922           case 3:Servo.P-=2;Servo.D-=1;break;
// 2923           }
// 2924         }break;
// 2925       case 4:
// 2926         {
// 2927           switch(track_form)
// 2928           {
// 2929             case 1:Servo.P+=0;Servo.D+=0;break;
// 2930             case 2:Servo.P-=1;Servo.D-=1;break;
// 2931             case 3:Servo.P-=2;Servo.D-=2;break;
// 2932           }
// 2933         }break;
// 2934       case 3:
// 2935         {
// 2936           switch(track_form)
// 2937           {
// 2938           case 1:Servo.P+=0;Servo.D-=1;break;
// 2939           case 2:Servo.P-=1;Servo.D-=2;break;
// 2940           case 3:Servo.P-=2;Servo.D-=3;break;
// 2941           }
// 2942         }break;
// 2943       case 2:
// 2944         {
// 2945           switch(track_form)
// 2946           {
// 2947           case 1:Servo.P+=0;Servo.D-=2;break;
// 2948           case 2:Servo.P-=1;Servo.D-=3;break;
// 2949           case 3:Servo.P-=2;Servo.D-=4;break;
// 2950           }
// 2951         }break;
// 2952       case 1:
// 2953         {
// 2954           switch(track_form)
// 2955           {
// 2956           case 1:Servo.P+=0;Servo.D-=3;break;
// 2957           case 2:Servo.P-=1;Servo.D-=4;break;
// 2958           case 3:Servo.P-=2;Servo.D-=5;break;
// 2959           }
// 2960         }break;
// 2961       }
// 2962     }break;
// 2963 case -1:
// 2964     {
// 2965       switch(position_now_D)
// 2966       {
// 2967        case -6:
// 2968         {
// 2969           switch(track_form)
// 2970           {
// 2971         //case 1:Servo.P+=5;Servo.D+=1;break;//-6-6
// 2972           case 1:Servo.P-=1;Servo.D+=2;break;
// 2973           case 2:Servo.P-=2;Servo.D+=1;break;
// 2974           case 3:Servo.P-=3;Servo.D+=0;break;
// 2975           }
// 2976         }break;
// 2977       case -5:
// 2978         {
// 2979           switch(track_form)
// 2980           {
// 2981             //case 1:Servo.P+=5;Servo.D+=1;break;
// 2982           case 1:Servo.P-=1;Servo.D+=1;break;
// 2983           case 2:Servo.P-=2;Servo.D+=0;break;
// 2984           case 3:Servo.P-=3;Servo.D-=1;break;
// 2985           }
// 2986         }break;
// 2987       case -4:
// 2988         {
// 2989           switch(track_form)
// 2990           {
// 2991             case 1:Servo.P-=1;Servo.D+=0;break;
// 2992             case 2:Servo.P-=2;Servo.D-=1;break;
// 2993             case 3:Servo.P-=3;Servo.D-=2;break;
// 2994           }
// 2995         }break;
// 2996       case -3:
// 2997         {
// 2998           switch(track_form)
// 2999           {
// 3000           case 1:Servo.P-=1;Servo.D-=1;break;
// 3001           case 2:Servo.P-=2;Servo.D-=2;break;
// 3002           case 3:Servo.P-=3;Servo.D-=3;break;
// 3003           }
// 3004         }break;
// 3005       case -2:
// 3006         {
// 3007           switch(track_form)
// 3008           {
// 3009           case 1:Servo.P-=1;Servo.D-=2;break;
// 3010           case 2:Servo.P-=2;Servo.D-=3;break;
// 3011           case 3:Servo.P-=3;Servo.D-=4;break;
// 3012           }
// 3013         }break;
// 3014       case -1:
// 3015         {
// 3016           switch(track_form)
// 3017           {
// 3018           case 1:Servo.P-=1;Servo.D-=3;break;
// 3019           case 2:Servo.P-=2;Servo.D-=4;break;
// 3020           case 3:Servo.P-=3;Servo.D-=5;break;
// 3021           }
// 3022         }break;
// 3023       case 0:
// 3024          {
// 3025           switch(track_form)
// 3026           {
// 3027           case 1:Servo.P-=1;break;
// 3028           case 2:Servo.P-=2;break;
// 3029           case 3:Servo.P-=3;break;
// 3030           }
// 3031         }break;
// 3032       case 6:
// 3033         {
// 3034           switch(track_form)
// 3035           {
// 3036         //case 1:Servo.P+=5;Servo.D+=1;break;//-6-6
// 3037           case 1:Servo.P-=1;Servo.D+=2;break;
// 3038           case 2:Servo.P-=2;Servo.D+=1;break;
// 3039           case 3:Servo.P-=3;Servo.D+=0;break;
// 3040           }
// 3041         }break;
// 3042       case 5:
// 3043         {
// 3044           switch(track_form)
// 3045           {
// 3046             //case 1:Servo.P+=5;Servo.D+=1;break;
// 3047           case 1:Servo.P-=1;Servo.D+=1;break;
// 3048           case 2:Servo.P-=2;Servo.D+=0;break;
// 3049           case 3:Servo.P-=3;Servo.D-=1;break;
// 3050           }
// 3051         }break;
// 3052       case 4:
// 3053         {
// 3054           switch(track_form)
// 3055           {
// 3056             case 1:Servo.P-=1;Servo.D+=0;break;
// 3057             case 2:Servo.P-=2;Servo.D-=1;break;
// 3058             case 3:Servo.P-=3;Servo.D-=2;break;
// 3059           }
// 3060         }break;
// 3061       case 3:
// 3062         {
// 3063           switch(track_form)
// 3064           {
// 3065           case 1:Servo.P-=1;Servo.D-=1;break;
// 3066           case 2:Servo.P-=2;Servo.D-=2;break;
// 3067           case 3:Servo.P-=3;Servo.D-=3;break;
// 3068           }
// 3069         }break;
// 3070       case 2:
// 3071         {
// 3072           switch(track_form)
// 3073           {
// 3074           case 1:Servo.P-=1;Servo.D-=2;break;
// 3075           case 2:Servo.P-=2;Servo.D-=3;break;
// 3076           case 3:Servo.P-=3;Servo.D-=4;break;
// 3077           }
// 3078         }break;
// 3079       case 1:
// 3080         {
// 3081           switch(track_form)
// 3082           {
// 3083           case 1:Servo.P-=1;Servo.D-=3;break;
// 3084           case 2:Servo.P-=2;Servo.D-=4;break;
// 3085           case 3:Servo.P-=3;Servo.D-=5;break;
// 3086           }
// 3087         }break;
// 3088       }
// 3089     }break;
// 3090   case 0:
// 3091     {
// 3092       switch(position_now_D)
??choose_PD_30:
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        CMP      R0,#+0
        BEQ.N    ??choose_PD_38
        CMP      R0,#+2
        BEQ.W    ??choose_PD_39
        BCC.W    ??choose_PD_40
        CMP      R0,#+4
        BEQ.W    ??choose_PD_41
        BCC.W    ??choose_PD_42
        CMP      R0,#+6
        BEQ.N    ??choose_PD_43
        BCC.N    ??choose_PD_44
        B.N      ??choose_PD_45
// 3093       {
// 3094       case -6:
// 3095         {
// 3096           switch(track_form)
// 3097           {
// 3098           case 1:Servo.P+=0;Servo.D+=2;break;//p=0,d=6
// 3099           case 2:Servo.P+=0;Servo.D+=1;break;
// 3100           case 3:Servo.P+=0;Servo.D+=0;break;
// 3101           }
// 3102         }break;
// 3103       case -5:
// 3104         {
// 3105           switch(track_form)
// 3106           {
// 3107           case 1:Servo.P+=0;Servo.D+=1;break;//0,5
// 3108           case 2:Servo.P+=0;Servo.D+=0;break;
// 3109           case 3:Servo.P+=0;Servo.D-=1;break; 
// 3110           }
// 3111         }break;
// 3112       case -4:
// 3113         {
// 3114           switch(track_form)
// 3115           {
// 3116           case 1:Servo.P+=0;Servo.D+=0;break;//4
// 3117           case 2:Servo.P+=0;Servo.D-=1;break;
// 3118           case 3:Servo.P+=0;Servo.D-=2;break;
// 3119           }
// 3120         }break;
// 3121       case -3:
// 3122         {
// 3123           switch(track_form)
// 3124           {
// 3125           case 1:Servo.P+=0;Servo.D-=1;break;//3
// 3126           case 2:Servo.P+=0;Servo.D-=2;break;
// 3127           case 3:Servo.P+=0;Servo.D-=3;break;
// 3128           }
// 3129         }break;
// 3130       case -2:
// 3131         {
// 3132           switch(track_form)
// 3133           {
// 3134           case 1:Servo.P+=0;Servo.D-=2;break;//2
// 3135           case 2:Servo.P+=0;Servo.D-=3;break;
// 3136           case 3:Servo.P+=0;Servo.D-=4;break;
// 3137           }
// 3138         }break;
// 3139       case -1:
// 3140         {
// 3141           switch(track_form)
// 3142           {
// 3143           case 1:Servo.P+=0;Servo.D-=3;break;//1
// 3144           case 2:Servo.P+=0;Servo.D-=4;break;
// 3145           case 3:Servo.P+=0;Servo.D-=5;break;
// 3146           }
// 3147         }break;
// 3148       case 0:
// 3149          {
// 3150           switch(track_form)
??choose_PD_38:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_46
        BCC.N    ??choose_PD_47
        CMP      R4,#+3
        BEQ.N    ??choose_PD_48
        BCC.N    ??choose_PD_49
        B.N      ??choose_PD_47
// 3151           {
// 3152           case 1:Servo.P+=0;break;
??choose_PD_46:
        LDR.W    R0,??choose_PD_2+0x8
        LDR.W    R1,??choose_PD_2+0x8
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
        B.N      ??choose_PD_47
// 3153           case 2:Servo.P+=0;break;
??choose_PD_49:
        LDR.W    R0,??choose_PD_2+0x8
        LDR.W    R1,??choose_PD_2+0x8
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
        B.N      ??choose_PD_47
// 3154           case 3:Servo.P+=0;break;
??choose_PD_48:
        LDR.W    R0,??choose_PD_2+0x8
        LDR.W    R1,??choose_PD_2+0x8
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
// 3155           }
// 3156         }break;
??choose_PD_47:
        B.N      ??choose_PD_45
// 3157       case 6:
// 3158         {
// 3159           switch(track_form)
??choose_PD_43:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_50
        BCC.N    ??choose_PD_51
        CMP      R4,#+3
        BEQ.N    ??choose_PD_52
        BCC.N    ??choose_PD_53
        B.N      ??choose_PD_51
// 3160           {
// 3161           case 1:Servo.P+=0;Servo.D+=2;break;//p=0,d=6
??choose_PD_50:
        LDR.W    R0,??choose_PD_2+0x8
        LDR.W    R1,??choose_PD_2+0x8
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
        LDR.W    R0,??choose_PD_2+0x8
        LDRH     R0,[R0, #+4]
        ADDS     R0,R0,#+2
        LDR.W    R1,??choose_PD_2+0x8
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_51
// 3162           case 2:Servo.P+=0;Servo.D+=1;break;
??choose_PD_53:
        LDR.W    R0,??choose_PD_2+0x8
        LDR.W    R1,??choose_PD_2+0x8
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
        LDR.W    R0,??choose_PD_2+0x8
        LDRH     R0,[R0, #+4]
        ADDS     R0,R0,#+1
        LDR.W    R1,??choose_PD_2+0x8
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_51
// 3163           case 3:Servo.P+=0;Servo.D+=0;break;
??choose_PD_52:
        LDR.W    R0,??choose_PD_2+0x8
        LDR.W    R1,??choose_PD_2+0x8
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
        LDR.W    R0,??choose_PD_2+0x8
        LDR.W    R1,??choose_PD_2+0x8
        LDRH     R1,[R1, #+4]
        STRH     R1,[R0, #+4]
// 3164           }
// 3165         }break;
??choose_PD_51:
        B.N      ??choose_PD_45
// 3166       case 5:
// 3167         {
// 3168           switch(track_form)
??choose_PD_44:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_54
        BCC.N    ??choose_PD_55
        CMP      R4,#+3
        BEQ.N    ??choose_PD_56
        BCC.N    ??choose_PD_57
        B.N      ??choose_PD_55
// 3169           {
// 3170           case 1:Servo.P+=0;Servo.D+=1;break;//0,5
??choose_PD_54:
        LDR.W    R0,??choose_PD_2+0x8
        LDR.W    R1,??choose_PD_2+0x8
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
        LDR.W    R0,??choose_PD_2+0x8
        LDRH     R0,[R0, #+4]
        ADDS     R0,R0,#+1
        LDR.W    R1,??choose_PD_2+0x8
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_55
// 3171           case 2:Servo.P+=0;Servo.D+=0;break;
??choose_PD_57:
        LDR.W    R0,??choose_PD_2+0x8
        LDR.W    R1,??choose_PD_2+0x8
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
        LDR.W    R0,??choose_PD_2+0x8
        LDR.W    R1,??choose_PD_2+0x8
        LDRH     R1,[R1, #+4]
        STRH     R1,[R0, #+4]
        B.N      ??choose_PD_55
// 3172           case 3:Servo.P+=0;Servo.D-=1;break; 
??choose_PD_56:
        LDR.W    R0,??choose_PD_2+0x8
        LDR.W    R1,??choose_PD_2+0x8
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
        LDR.W    R0,??choose_PD_2+0x8
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+1
        LDR.W    R1,??choose_PD_2+0x8
        STRH     R0,[R1, #+4]
// 3173           }
// 3174         }break;
??choose_PD_55:
        B.N      ??choose_PD_45
// 3175       case 4:
// 3176         {
// 3177           switch(track_form)
??choose_PD_41:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_58
        BCC.N    ??choose_PD_59
        CMP      R4,#+3
        BEQ.N    ??choose_PD_60
        BCC.N    ??choose_PD_61
        B.N      ??choose_PD_59
// 3178           {
// 3179           case 1:Servo.P+=0;Servo.D+=0;break;//4
??choose_PD_58:
        LDR.W    R0,??choose_PD_2+0x8
        LDR.W    R1,??choose_PD_2+0x8
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
        LDR.W    R0,??choose_PD_2+0x8
        LDR.N    R1,??choose_PD_2+0x8
        LDRH     R1,[R1, #+4]
        STRH     R1,[R0, #+4]
        B.N      ??choose_PD_59
// 3180           case 2:Servo.P+=0;Servo.D-=1;break;
??choose_PD_61:
        LDR.N    R0,??choose_PD_2+0x8
        LDR.N    R1,??choose_PD_2+0x8
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
        LDR.N    R0,??choose_PD_2+0x8
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+1
        LDR.N    R1,??choose_PD_2+0x8
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_59
// 3181           case 3:Servo.P+=0;Servo.D-=2;break;
??choose_PD_60:
        LDR.N    R0,??choose_PD_2+0x8
        LDR.N    R1,??choose_PD_2+0x8
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
        LDR.N    R0,??choose_PD_2+0x8
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+2
        LDR.N    R1,??choose_PD_2+0x8
        STRH     R0,[R1, #+4]
// 3182           }
// 3183         }break;
??choose_PD_59:
        B.N      ??choose_PD_45
// 3184       case 3:
// 3185         {
// 3186           switch(track_form)
??choose_PD_42:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_62
        BCC.N    ??choose_PD_63
        CMP      R4,#+3
        BEQ.N    ??choose_PD_64
        BCC.N    ??choose_PD_65
        B.N      ??choose_PD_63
// 3187           {
// 3188           case 1:Servo.P+=0;Servo.D-=1;break;//3
??choose_PD_62:
        LDR.N    R0,??choose_PD_2+0x8
        LDR.N    R1,??choose_PD_2+0x8
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
        LDR.N    R0,??choose_PD_2+0x8
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+1
        LDR.N    R1,??choose_PD_2+0x8
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_63
// 3189           case 2:Servo.P+=0;Servo.D-=2;break;
??choose_PD_65:
        LDR.N    R0,??choose_PD_2+0x8
        LDR.N    R1,??choose_PD_2+0x8
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
        LDR.N    R0,??choose_PD_2+0x8
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+2
        LDR.N    R1,??choose_PD_2+0x8
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_63
// 3190           case 3:Servo.P+=0;Servo.D-=3;break;
??choose_PD_64:
        LDR.N    R0,??choose_PD_2+0x8
        LDR.N    R1,??choose_PD_2+0x8
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
        LDR.N    R0,??choose_PD_2+0x8
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+3
        LDR.N    R1,??choose_PD_2+0x8
        STRH     R0,[R1, #+4]
// 3191           }
// 3192         }break;
??choose_PD_63:
        B.N      ??choose_PD_45
// 3193       case 2:
// 3194         {
// 3195           switch(track_form)
??choose_PD_39:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_66
        BCC.N    ??choose_PD_67
        CMP      R4,#+3
        BEQ.N    ??choose_PD_68
        BCC.N    ??choose_PD_69
        B.N      ??choose_PD_67
// 3196           {
// 3197           case 1:Servo.P+=0;Servo.D-=2;break;//2
??choose_PD_66:
        LDR.N    R0,??choose_PD_2+0x8
        LDR.N    R1,??choose_PD_2+0x8
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
        LDR.N    R0,??choose_PD_2+0x8
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+2
        LDR.N    R1,??choose_PD_2+0x8
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_67
// 3198           case 2:Servo.P+=0;Servo.D-=3;break;
??choose_PD_69:
        LDR.N    R0,??choose_PD_2+0x8
        LDR.N    R1,??choose_PD_2+0x8
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
        LDR.N    R0,??choose_PD_2+0x8
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+3
        LDR.N    R1,??choose_PD_2+0x8
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_67
// 3199           case 3:Servo.P+=0;Servo.D-=4;break;
??choose_PD_68:
        LDR.N    R0,??choose_PD_2+0x8
        LDR.N    R1,??choose_PD_2+0x8
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
        LDR.N    R0,??choose_PD_2+0x8
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+4
        LDR.N    R1,??choose_PD_2+0x8
        STRH     R0,[R1, #+4]
// 3200           }
// 3201         }break;
??choose_PD_67:
        B.N      ??choose_PD_45
// 3202       case 1:
// 3203         {
// 3204           switch(track_form)
??choose_PD_40:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_70
        BCC.N    ??choose_PD_71
        CMP      R4,#+3
        BEQ.N    ??choose_PD_72
        BCC.N    ??choose_PD_73
        B.N      ??choose_PD_71
// 3205           {
// 3206           case 1:Servo.P+=0;Servo.D-=3;break;//1
??choose_PD_70:
        LDR.N    R0,??choose_PD_2+0x8
        LDR.N    R1,??choose_PD_2+0x8
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
        LDR.N    R0,??choose_PD_2+0x8
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+3
        LDR.N    R1,??choose_PD_2+0x8
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_71
// 3207           case 2:Servo.P+=0;Servo.D-=4;break;
??choose_PD_73:
        LDR.N    R0,??choose_PD_2+0x8
        LDR.N    R1,??choose_PD_2+0x8
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
        LDR.N    R0,??choose_PD_2+0x8
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+4
        LDR.N    R1,??choose_PD_2+0x8
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_71
// 3208           case 3:Servo.P+=0;Servo.D-=5;break;
??choose_PD_72:
        LDR.N    R0,??choose_PD_2+0x8
        LDR.N    R1,??choose_PD_2+0x8
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
        LDR.N    R0,??choose_PD_2+0x8
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+5
        LDR.N    R1,??choose_PD_2+0x8
        STRH     R0,[R1, #+4]
// 3209           }
// 3210         }break;
// 3211       }
// 3212     }break;
??choose_PD_71:
??choose_PD_45:
        B.W      ??choose_PD_37
// 3213  case 6:
// 3214     {
// 3215       switch(position_now_D)
??choose_PD_35:
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        CMP      R0,#+0
        BEQ.N    ??choose_PD_74
        CMP      R0,#+2
        BEQ.W    ??choose_PD_75
        BCC.W    ??choose_PD_76
        CMP      R0,#+4
        BEQ.N    ??choose_PD_77
        BCC.W    ??choose_PD_78
        CMP      R0,#+6
        BEQ.N    ??choose_PD_79
        BCC.N    ??choose_PD_80
        B.N      ??choose_PD_81
// 3216       {
// 3217      case -6:
// 3218         {
// 3219           switch(track_form)
// 3220           {
// 3221         //case 1:Servo.P+=5;Servo.D+=1;break;//-6-6
// 3222           case 1:Servo.P+=4;Servo.D+=2;break;
// 3223           case 2:Servo.P+=3;Servo.D+=1;break;
// 3224           case 3:Servo.P+=2;Servo.D+=0;break;
// 3225           }
// 3226         }break;
// 3227       case -5:
// 3228         {
// 3229           switch(track_form)
// 3230           {
// 3231             //case 1:Servo.P+=5;Servo.D+=1;break;
// 3232           case 1:Servo.P+=4;Servo.D+=1;break;
// 3233           case 2:Servo.P+=3;Servo.D+=0;break;
// 3234           case 3:Servo.P+=2;Servo.D-=1;break;
// 3235           }
// 3236         }break;
// 3237       case -4:
// 3238         {
// 3239           switch(track_form)
// 3240           {
// 3241             case 1:Servo.P+=4;Servo.D+=0;break;
// 3242             case 2:Servo.P+=3;Servo.D-=1;break;
// 3243             case 3:Servo.P+=2;Servo.D-=2;break;
// 3244           }
// 3245         }break;
// 3246       case -3:
// 3247         {
// 3248           switch(track_form)
// 3249           {
// 3250           case 1:Servo.P+=4;Servo.D-=1;break;
// 3251           case 2:Servo.P+=3;Servo.D-=2;break;
// 3252           case 3:Servo.P+=2;Servo.D-=3;break;
// 3253           }
// 3254         }break;
// 3255       case -2:
// 3256         {
// 3257           switch(track_form)
// 3258           {
// 3259           case 1:Servo.P+=4;Servo.D-=2;break;
// 3260           case 2:Servo.P+=3;Servo.D-=3;break;
// 3261           case 3:Servo.P+=2;Servo.D-=4;break;
// 3262           }
// 3263         }break;
// 3264       case -1:
// 3265         {
// 3266           switch(track_form)
// 3267           {
// 3268           case 1:Servo.P+=4;Servo.D-=3;break;
// 3269           case 2:Servo.P+=3;Servo.D-=4;break;
// 3270           case 3:Servo.P+=2;Servo.D-=5;break;
// 3271           }
// 3272         }break;
// 3273       case 0:
// 3274          {
// 3275           switch(track_form)
??choose_PD_74:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_82
        BCC.N    ??choose_PD_83
        CMP      R4,#+3
        BEQ.N    ??choose_PD_84
        BCC.N    ??choose_PD_85
        B.N      ??choose_PD_83
// 3276           {
// 3277           case 1:Servo.P+=4;break;
??choose_PD_82:
        LDR.N    R0,??choose_PD_2+0x8
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+4
        LDR.N    R1,??choose_PD_2+0x8
        STRH     R0,[R1, #+0]
        B.N      ??choose_PD_83
// 3278           case 2:Servo.P+=3;break;
??choose_PD_85:
        LDR.N    R0,??choose_PD_2+0x8
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+3
        LDR.N    R1,??choose_PD_2+0x8
        STRH     R0,[R1, #+0]
        B.N      ??choose_PD_83
// 3279           case 3:Servo.P+=2;break;
??choose_PD_84:
        LDR.N    R0,??choose_PD_2+0x8
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+2
        LDR.N    R1,??choose_PD_2+0x8
        STRH     R0,[R1, #+0]
// 3280           }
// 3281         }break;
??choose_PD_83:
        B.N      ??choose_PD_81
// 3282       case 6:
// 3283         {
// 3284           switch(track_form)
??choose_PD_79:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_86
        BCC.N    ??choose_PD_87
        CMP      R4,#+3
        BEQ.N    ??choose_PD_88
        BCC.N    ??choose_PD_89
        B.N      ??choose_PD_87
// 3285           {
// 3286         //case 1:Servo.P+=5;Servo.D+=1;break;//-6-6
// 3287           case 1:Servo.P+=4;Servo.D+=2;break;
??choose_PD_86:
        LDR.N    R0,??choose_PD_2+0x8
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+4
        LDR.N    R1,??choose_PD_2+0x8
        STRH     R0,[R1, #+0]
        LDR.N    R0,??choose_PD_2+0x8
        LDRH     R0,[R0, #+4]
        ADDS     R0,R0,#+2
        LDR.N    R1,??choose_PD_2+0x8
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_87
// 3288           case 2:Servo.P+=3;Servo.D+=1;break;
??choose_PD_89:
        LDR.N    R0,??choose_PD_2+0x8
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+3
        LDR.N    R1,??choose_PD_2+0x8
        STRH     R0,[R1, #+0]
        LDR.N    R0,??choose_PD_2+0x8
        LDRH     R0,[R0, #+4]
        ADDS     R0,R0,#+1
        LDR.N    R1,??choose_PD_2+0x8
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_87
// 3289           case 3:Servo.P+=2;Servo.D+=0;break;
??choose_PD_88:
        LDR.N    R0,??choose_PD_2+0x8
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+2
        LDR.N    R1,??choose_PD_2+0x8
        STRH     R0,[R1, #+0]
        LDR.N    R0,??choose_PD_2+0x8
        LDR.N    R1,??choose_PD_2+0x8
        LDRH     R1,[R1, #+4]
        STRH     R1,[R0, #+4]
// 3290           }
// 3291         }break;
??choose_PD_87:
        B.N      ??choose_PD_81
// 3292       case 5:
// 3293         {
// 3294           switch(track_form)
??choose_PD_80:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_90
        BCC.N    ??choose_PD_91
        CMP      R4,#+3
        BEQ.N    ??choose_PD_92
        BCC.N    ??choose_PD_93
        B.N      ??choose_PD_91
// 3295           {
// 3296             //case 1:Servo.P+=5;Servo.D+=1;break;
// 3297           case 1:Servo.P+=4;Servo.D+=1;break;
??choose_PD_90:
        LDR.N    R0,??choose_PD_2+0x8
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+4
        LDR.N    R1,??choose_PD_2+0x8
        STRH     R0,[R1, #+0]
        LDR.N    R0,??choose_PD_2+0x8
        LDRH     R0,[R0, #+4]
        ADDS     R0,R0,#+1
        LDR.N    R1,??choose_PD_2+0x8
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_91
// 3298           case 2:Servo.P+=3;Servo.D+=0;break;
??choose_PD_93:
        LDR.N    R0,??choose_PD_2+0x8
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+3
        LDR.N    R1,??choose_PD_2+0x8
        STRH     R0,[R1, #+0]
        LDR.N    R0,??choose_PD_2+0x8
        LDR.N    R1,??choose_PD_2+0x8
        LDRH     R1,[R1, #+4]
        STRH     R1,[R0, #+4]
        B.N      ??choose_PD_91
// 3299           case 3:Servo.P+=2;Servo.D-=1;break;
??choose_PD_92:
        LDR.N    R0,??choose_PD_2+0x8
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+2
        LDR.N    R1,??choose_PD_2+0x8
        STRH     R0,[R1, #+0]
        LDR.N    R0,??choose_PD_2+0x8
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+1
        LDR.N    R1,??choose_PD_2+0x8
        STRH     R0,[R1, #+4]
// 3300           }
// 3301         }break;
??choose_PD_91:
        B.N      ??choose_PD_81
// 3302       case 4:
// 3303         {
// 3304           switch(track_form)
??choose_PD_77:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_94
        BCC.N    ??choose_PD_95
        CMP      R4,#+3
        BEQ.N    ??choose_PD_96
        BCC.N    ??choose_PD_97
        B.N      ??choose_PD_95
// 3305           {
// 3306             case 1:Servo.P+=4;Servo.D+=0;break;
??choose_PD_94:
        LDR.N    R0,??choose_PD_2+0x8
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+4
        LDR.N    R1,??choose_PD_2+0x8
        STRH     R0,[R1, #+0]
        LDR.N    R0,??choose_PD_2+0x8
        LDR.N    R1,??choose_PD_2+0x8
        LDRH     R1,[R1, #+4]
        STRH     R1,[R0, #+4]
        B.N      ??choose_PD_95
// 3307             case 2:Servo.P+=3;Servo.D-=1;break;
??choose_PD_97:
        LDR.N    R0,??choose_PD_2+0x8
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+3
        LDR.N    R1,??choose_PD_2+0x8
        STRH     R0,[R1, #+0]
        LDR.N    R0,??choose_PD_2+0x8
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+1
        LDR.N    R1,??choose_PD_2+0x8
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_95
// 3308             case 3:Servo.P+=2;Servo.D-=2;break;
??choose_PD_96:
        LDR.N    R0,??choose_PD_2+0x8
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+2
        LDR.N    R1,??choose_PD_2+0x8
        STRH     R0,[R1, #+0]
        LDR.N    R0,??choose_PD_2+0x8
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+2
        LDR.N    R1,??choose_PD_2+0x8
        STRH     R0,[R1, #+4]
// 3309           }
// 3310         }break;
??choose_PD_95:
        B.N      ??choose_PD_81
// 3311       case 3:
// 3312         {
// 3313           switch(track_form)
??choose_PD_78:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_98
        BCC.N    ??choose_PD_99
        CMP      R4,#+3
        BEQ.N    ??choose_PD_100
        BCC.N    ??choose_PD_101
        B.N      ??choose_PD_99
// 3314           {
// 3315           case 1:Servo.P+=4;Servo.D-=1;break;
??choose_PD_98:
        LDR.N    R0,??choose_PD_2+0x8
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+4
        LDR.N    R1,??choose_PD_2+0x8
        STRH     R0,[R1, #+0]
        LDR.N    R0,??choose_PD_2+0x8
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+1
        LDR.N    R1,??choose_PD_2+0x8
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_99
// 3316           case 2:Servo.P+=3;Servo.D-=2;break;
??choose_PD_101:
        LDR.N    R0,??choose_PD_2+0x8
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+3
        LDR.N    R1,??choose_PD_2+0x8
        STRH     R0,[R1, #+0]
        LDR.N    R0,??choose_PD_2+0x8
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+2
        LDR.N    R1,??choose_PD_2+0x8
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_99
        Nop      
        DATA
??choose_PD_2:
        DC32     position
        DC32     dijihang
        DC32     Servo
        DC32     P_High
        DC32     D_High
        DC32     servo
        DC32     H_High
        DC32     T_High
        DC32     P_Mid
        DC32     D_Mid
        DC32     H_Mid
        DC32     T_Mid
        DC32     P_Mid1
        DC32     D_Mid1
        DC32     H_Mid1
        DC32     T_Mid1
        DC32     P_Low
        DC32     D_Low
        DC32     H_Low
        DC32     T_Low
        DC32     position_now
        DC32     position_now_low
        THUMB
// 3317           case 3:Servo.P+=2;Servo.D-=3;break;
??choose_PD_100:
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+2
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+0]
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+3
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+4]
// 3318           }
// 3319         }break;
??choose_PD_99:
        B.N      ??choose_PD_81
// 3320       case 2:
// 3321         {
// 3322           switch(track_form)
??choose_PD_75:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_102
        BCC.N    ??choose_PD_103
        CMP      R4,#+3
        BEQ.N    ??choose_PD_104
        BCC.N    ??choose_PD_105
        B.N      ??choose_PD_103
// 3323           {
// 3324           case 1:Servo.P+=4;Servo.D-=2;break;
??choose_PD_102:
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+4
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+0]
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+2
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_103
// 3325           case 2:Servo.P+=3;Servo.D-=3;break;
??choose_PD_105:
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+3
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+0]
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+3
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_103
// 3326           case 3:Servo.P+=2;Servo.D-=4;break;
??choose_PD_104:
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+2
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+0]
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+4
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+4]
// 3327           }
// 3328         }break;
??choose_PD_103:
        B.N      ??choose_PD_81
// 3329       case 1:
// 3330         {
// 3331           switch(track_form)
??choose_PD_76:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_106
        BCC.N    ??choose_PD_107
        CMP      R4,#+3
        BEQ.N    ??choose_PD_108
        BCC.N    ??choose_PD_109
        B.N      ??choose_PD_107
// 3332           {
// 3333           case 1:Servo.P+=4;Servo.D-=3;break;
??choose_PD_106:
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+4
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+0]
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+3
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_107
// 3334           case 2:Servo.P+=3;Servo.D-=4;break;
??choose_PD_109:
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+3
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+0]
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+4
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_107
// 3335           case 3:Servo.P+=2;Servo.D-=5;break;
??choose_PD_108:
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+2
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+0]
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+5
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+4]
// 3336           }
// 3337         }break;
// 3338       }
// 3339     }break;
??choose_PD_107:
??choose_PD_81:
        B.W      ??choose_PD_37
// 3340   case 5:
// 3341     {
// 3342       switch(position_now_D)
??choose_PD_36:
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        CMP      R0,#+0
        BEQ.N    ??choose_PD_110
        CMP      R0,#+2
        BEQ.W    ??choose_PD_111
        BCC.W    ??choose_PD_112
        CMP      R0,#+4
        BEQ.W    ??choose_PD_113
        BCC.W    ??choose_PD_114
        CMP      R0,#+6
        BEQ.N    ??choose_PD_115
        BCC.N    ??choose_PD_116
        B.N      ??choose_PD_117
// 3343       {
// 3344       case -6:
// 3345         {
// 3346           switch(track_form)
// 3347           {
// 3348         //case 1:Servo.P+=5;Servo.D+=1;break;//-6-6
// 3349           case 1:Servo.P+=3;Servo.D+=2;break;
// 3350           case 2:Servo.P+=2;Servo.D+=1;break;
// 3351           case 3:Servo.P+=1;Servo.D+=0;break;
// 3352           }
// 3353         }break;
// 3354       case -5:
// 3355         {
// 3356           switch(track_form)
// 3357           {
// 3358             //case 1:Servo.P+=5;Servo.D+=1;break;
// 3359           case 1:Servo.P+=3;Servo.D+=1;break;
// 3360           case 2:Servo.P+=2;Servo.D+=0;break;
// 3361           case 3:Servo.P+=1;Servo.D-=1;break;
// 3362           }
// 3363         }break;
// 3364       case -4:
// 3365         {
// 3366           switch(track_form)
// 3367           {
// 3368             case 1:Servo.P+=3;Servo.D+=0;break;
// 3369             case 2:Servo.P+=2;Servo.D-=1;break;
// 3370             case 3:Servo.P+=1;Servo.D-=2;break;
// 3371           }
// 3372         }break;
// 3373       case -3:
// 3374         {
// 3375           switch(track_form)
// 3376           {
// 3377           case 1:Servo.P+=3;Servo.D-=1;break;
// 3378           case 2:Servo.P+=2;Servo.D-=2;break;
// 3379           case 3:Servo.P+=1;Servo.D-=3;break;
// 3380           }
// 3381         }break;
// 3382       case -2:
// 3383         {
// 3384           switch(track_form)
// 3385           {
// 3386           case 1:Servo.P+=3;Servo.D-=2;break;
// 3387           case 2:Servo.P+=2;Servo.D-=3;break;
// 3388           case 3:Servo.P+=1;Servo.D-=4;break;
// 3389           }
// 3390         }break;
// 3391       case -1:
// 3392         {
// 3393           switch(track_form)
// 3394           {
// 3395           case 1:Servo.P+=3;Servo.D-=3;break;
// 3396           case 2:Servo.P+=2;Servo.D-=4;break;
// 3397           case 3:Servo.P+=1;Servo.D-=5;break;
// 3398           }
// 3399         }break;
// 3400       case 0:
// 3401          {
// 3402           switch(track_form)
??choose_PD_110:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_118
        BCC.N    ??choose_PD_119
        CMP      R4,#+3
        BEQ.N    ??choose_PD_120
        BCC.N    ??choose_PD_121
        B.N      ??choose_PD_119
// 3403           {
// 3404           case 1:Servo.P+=3;break;
??choose_PD_118:
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+3
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+0]
        B.N      ??choose_PD_119
// 3405           case 2:Servo.P+=2;break;
??choose_PD_121:
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+2
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+0]
        B.N      ??choose_PD_119
// 3406           case 3:Servo.P+=1;break;
??choose_PD_120:
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+0]
// 3407           }
// 3408         }break;
??choose_PD_119:
        B.N      ??choose_PD_117
// 3409       case 6:
// 3410         {
// 3411           switch(track_form)
??choose_PD_115:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_122
        BCC.N    ??choose_PD_123
        CMP      R4,#+3
        BEQ.N    ??choose_PD_124
        BCC.N    ??choose_PD_125
        B.N      ??choose_PD_123
// 3412           {
// 3413         //case 1:Servo.P+=5;Servo.D+=1;break;//-6-6
// 3414           case 1:Servo.P+=3;Servo.D+=2;break;
??choose_PD_122:
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+3
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+0]
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+4]
        ADDS     R0,R0,#+2
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_123
// 3415           case 2:Servo.P+=2;Servo.D+=1;break;
??choose_PD_125:
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+2
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+0]
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+4]
        ADDS     R0,R0,#+1
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_123
// 3416           case 3:Servo.P+=1;Servo.D+=0;break;
??choose_PD_124:
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+0]
        LDR.W    R0,??choose_PD_28+0x4
        LDR.W    R1,??choose_PD_28+0x4
        LDRH     R1,[R1, #+4]
        STRH     R1,[R0, #+4]
// 3417           }
// 3418         }break;
??choose_PD_123:
        B.N      ??choose_PD_117
// 3419       case 5:
// 3420         {
// 3421           switch(track_form)
??choose_PD_116:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_126
        BCC.N    ??choose_PD_127
        CMP      R4,#+3
        BEQ.N    ??choose_PD_128
        BCC.N    ??choose_PD_129
        B.N      ??choose_PD_127
// 3422           {
// 3423             //case 1:Servo.P+=5;Servo.D+=1;break;
// 3424           case 1:Servo.P+=3;Servo.D+=1;break;
??choose_PD_126:
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+3
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+0]
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+4]
        ADDS     R0,R0,#+1
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_127
// 3425           case 2:Servo.P+=2;Servo.D+=0;break;
??choose_PD_129:
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+2
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+0]
        LDR.W    R0,??choose_PD_28+0x4
        LDR.W    R1,??choose_PD_28+0x4
        LDRH     R1,[R1, #+4]
        STRH     R1,[R0, #+4]
        B.N      ??choose_PD_127
// 3426           case 3:Servo.P+=1;Servo.D-=1;break;
??choose_PD_128:
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+0]
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+1
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+4]
// 3427           }
// 3428         }break;
??choose_PD_127:
        B.N      ??choose_PD_117
// 3429       case 4:
// 3430         {
// 3431           switch(track_form)
??choose_PD_113:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_130
        BCC.N    ??choose_PD_131
        CMP      R4,#+3
        BEQ.N    ??choose_PD_132
        BCC.N    ??choose_PD_133
        B.N      ??choose_PD_131
// 3432           {
// 3433             case 1:Servo.P+=3;Servo.D+=0;break;
??choose_PD_130:
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+3
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+0]
        LDR.W    R0,??choose_PD_28+0x4
        LDR.W    R1,??choose_PD_28+0x4
        LDRH     R1,[R1, #+4]
        STRH     R1,[R0, #+4]
        B.N      ??choose_PD_131
// 3434             case 2:Servo.P+=2;Servo.D-=1;break;
??choose_PD_133:
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+2
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+0]
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+1
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_131
// 3435             case 3:Servo.P+=1;Servo.D-=2;break;
??choose_PD_132:
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+0]
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+2
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+4]
// 3436           }
// 3437         }break;
??choose_PD_131:
        B.N      ??choose_PD_117
// 3438       case 3:
// 3439         {
// 3440           switch(track_form)
??choose_PD_114:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_134
        BCC.N    ??choose_PD_135
        CMP      R4,#+3
        BEQ.N    ??choose_PD_136
        BCC.N    ??choose_PD_137
        B.N      ??choose_PD_135
// 3441           {
// 3442           case 1:Servo.P+=3;Servo.D-=1;break;
??choose_PD_134:
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+3
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+0]
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+1
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_135
// 3443           case 2:Servo.P+=2;Servo.D-=2;break;
??choose_PD_137:
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+2
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+0]
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+2
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_135
// 3444           case 3:Servo.P+=1;Servo.D-=3;break;
??choose_PD_136:
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+0]
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+3
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+4]
// 3445           }
// 3446         }break;
??choose_PD_135:
        B.N      ??choose_PD_117
// 3447       case 2:
// 3448         {
// 3449           switch(track_form)
??choose_PD_111:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_138
        BCC.N    ??choose_PD_139
        CMP      R4,#+3
        BEQ.N    ??choose_PD_140
        BCC.N    ??choose_PD_141
        B.N      ??choose_PD_139
// 3450           {
// 3451           case 1:Servo.P+=3;Servo.D-=2;break;
??choose_PD_138:
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+3
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+0]
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+2
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_139
// 3452           case 2:Servo.P+=2;Servo.D-=3;break;
??choose_PD_141:
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+2
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+0]
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+3
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_139
// 3453           case 3:Servo.P+=1;Servo.D-=4;break;
??choose_PD_140:
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+0]
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+4
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+4]
// 3454           }
// 3455         }break;
??choose_PD_139:
        B.N      ??choose_PD_117
// 3456       case 1:
// 3457         {
// 3458           switch(track_form)
??choose_PD_112:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_142
        BCC.N    ??choose_PD_143
        CMP      R4,#+3
        BEQ.N    ??choose_PD_144
        BCC.N    ??choose_PD_145
        B.N      ??choose_PD_143
// 3459           {
// 3460           case 1:Servo.P+=3;Servo.D-=3;break;
??choose_PD_142:
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+3
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+0]
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+3
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_143
// 3461           case 2:Servo.P+=2;Servo.D-=4;break;
??choose_PD_145:
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+2
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+0]
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+4
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_143
// 3462           case 3:Servo.P+=1;Servo.D-=5;break;
??choose_PD_144:
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+0]
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+5
        LDR.W    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+4]
// 3463           }
// 3464         }break;
// 3465       }
// 3466     }break;
??choose_PD_143:
??choose_PD_117:
        B.W      ??choose_PD_37
// 3467   case 4:
// 3468     {
// 3469       switch(position_now_D)
??choose_PD_33:
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        CMP      R0,#+0
        BEQ.N    ??choose_PD_146
        CMP      R0,#+2
        BEQ.W    ??choose_PD_147
        BCC.W    ??choose_PD_148
        CMP      R0,#+4
        BEQ.N    ??choose_PD_149
        BCC.W    ??choose_PD_150
        CMP      R0,#+6
        BEQ.N    ??choose_PD_151
        BCC.N    ??choose_PD_152
        B.N      ??choose_PD_153
// 3470       {
// 3471          case -6:
// 3472         {
// 3473           switch(track_form)
// 3474           {
// 3475         //case 1:Servo.P+=5;Servo.D+=1;break;//-6-6
// 3476           case 1:Servo.P+=2;Servo.D+=2;break;
// 3477           case 2:Servo.P+=1;Servo.D+=1;break;
// 3478           case 3:Servo.P+=0;Servo.D+=0;break;
// 3479           }
// 3480         }break;
// 3481       case -5:
// 3482         {
// 3483           switch(track_form)
// 3484           {
// 3485             //case 1:Servo.P+=5;Servo.D+=1;break;
// 3486           case 1:Servo.P+=2;Servo.D+=1;break;
// 3487           case 2:Servo.P+=1;Servo.D+=0;break;
// 3488           case 3:Servo.P+=0;Servo.D-=1;break;
// 3489           }
// 3490         }break;
// 3491       case -4:
// 3492         {
// 3493           switch(track_form)
// 3494           {
// 3495             case 1:Servo.P+=2;Servo.D+=0;break;
// 3496             case 2:Servo.P+=1;Servo.D-=1;break;
// 3497             case 3:Servo.P+=0;Servo.D-=2;break;
// 3498           }
// 3499         }break;
// 3500       case -3:
// 3501         {
// 3502           switch(track_form)
// 3503           {
// 3504           case 1:Servo.P+=2;Servo.D-=1;break;
// 3505           case 2:Servo.P+=1;Servo.D-=2;break;
// 3506           case 3:Servo.P+=0;Servo.D-=3;break;
// 3507           }
// 3508         }break;
// 3509       case -2:
// 3510         {
// 3511           switch(track_form)
// 3512           {
// 3513           case 1:Servo.P+=2;Servo.D-=2;break;
// 3514           case 2:Servo.P+=1;Servo.D-=3;break;
// 3515           case 3:Servo.P+=0;Servo.D-=4;break;
// 3516           }
// 3517         }break;
// 3518       case -1:
// 3519         {
// 3520           switch(track_form)
// 3521           {
// 3522           case 1:Servo.P+=2;Servo.D-=3;break;
// 3523           case 2:Servo.P+=1;Servo.D-=4;break;
// 3524           case 3:Servo.P+=0;Servo.D-=5;break;
// 3525           }
// 3526         }break;
// 3527       case 0:
// 3528          {
// 3529           switch(track_form)
??choose_PD_146:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_154
        BCC.N    ??choose_PD_155
        CMP      R4,#+3
        BEQ.N    ??choose_PD_156
        BCC.N    ??choose_PD_157
        B.N      ??choose_PD_155
// 3530           {
// 3531           case 1:Servo.P+=2;break;
??choose_PD_154:
        LDR.W    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+2
        LDR.N    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+0]
        B.N      ??choose_PD_155
// 3532           case 2:Servo.P+=1;break;
??choose_PD_157:
        LDR.N    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.N    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+0]
        B.N      ??choose_PD_155
// 3533           case 3:Servo.P+=0;break;
??choose_PD_156:
        LDR.N    R0,??choose_PD_28+0x4
        LDR.N    R1,??choose_PD_28+0x4
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
// 3534           }
// 3535         }break;
??choose_PD_155:
        B.N      ??choose_PD_153
// 3536       case 6:
// 3537         {
// 3538           switch(track_form)
??choose_PD_151:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_158
        BCC.N    ??choose_PD_159
        CMP      R4,#+3
        BEQ.N    ??choose_PD_160
        BCC.N    ??choose_PD_161
        B.N      ??choose_PD_159
// 3539           {
// 3540         //case 1:Servo.P+=5;Servo.D+=1;break;//-6-6
// 3541           case 1:Servo.P+=2;Servo.D+=2;break;
??choose_PD_158:
        LDR.N    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+2
        LDR.N    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+0]
        LDR.N    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+4]
        ADDS     R0,R0,#+2
        LDR.N    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_159
// 3542           case 2:Servo.P+=1;Servo.D+=1;break;
??choose_PD_161:
        LDR.N    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.N    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+0]
        LDR.N    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+4]
        ADDS     R0,R0,#+1
        LDR.N    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_159
// 3543           case 3:Servo.P+=0;Servo.D+=0;break;
??choose_PD_160:
        LDR.N    R0,??choose_PD_28+0x4
        LDR.N    R1,??choose_PD_28+0x4
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
        LDR.N    R0,??choose_PD_28+0x4
        LDR.N    R1,??choose_PD_28+0x4
        LDRH     R1,[R1, #+4]
        STRH     R1,[R0, #+4]
// 3544           }
// 3545         }break;
??choose_PD_159:
        B.N      ??choose_PD_153
// 3546       case 5:
// 3547         {
// 3548           switch(track_form)
??choose_PD_152:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_162
        BCC.N    ??choose_PD_163
        CMP      R4,#+3
        BEQ.N    ??choose_PD_164
        BCC.N    ??choose_PD_165
        B.N      ??choose_PD_163
// 3549           {
// 3550             //case 1:Servo.P+=5;Servo.D+=1;break;
// 3551           case 1:Servo.P+=2;Servo.D+=1;break;
??choose_PD_162:
        LDR.N    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+2
        LDR.N    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+0]
        LDR.N    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+4]
        ADDS     R0,R0,#+1
        LDR.N    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_163
// 3552           case 2:Servo.P+=1;Servo.D+=0;break;
??choose_PD_165:
        LDR.N    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.N    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+0]
        LDR.N    R0,??choose_PD_28+0x4
        LDR.N    R1,??choose_PD_28+0x4
        LDRH     R1,[R1, #+4]
        STRH     R1,[R0, #+4]
        B.N      ??choose_PD_163
// 3553           case 3:Servo.P+=0;Servo.D-=1;break;
??choose_PD_164:
        LDR.N    R0,??choose_PD_28+0x4
        LDR.N    R1,??choose_PD_28+0x4
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
        LDR.N    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+1
        LDR.N    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+4]
// 3554           }
// 3555         }break;
??choose_PD_163:
        B.N      ??choose_PD_153
// 3556       case 4:
// 3557         {
// 3558           switch(track_form)
??choose_PD_149:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_166
        BCC.N    ??choose_PD_167
        CMP      R4,#+3
        BEQ.N    ??choose_PD_168
        BCC.N    ??choose_PD_169
        B.N      ??choose_PD_167
// 3559           {
// 3560             case 1:Servo.P+=2;Servo.D+=0;break;
??choose_PD_166:
        LDR.N    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+2
        LDR.N    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+0]
        LDR.N    R0,??choose_PD_28+0x4
        LDR.N    R1,??choose_PD_28+0x4
        LDRH     R1,[R1, #+4]
        STRH     R1,[R0, #+4]
        B.N      ??choose_PD_167
// 3561             case 2:Servo.P+=1;Servo.D-=1;break;
??choose_PD_169:
        LDR.N    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.N    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+0]
        LDR.N    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+1
        LDR.N    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_167
// 3562             case 3:Servo.P+=0;Servo.D-=2;break;
??choose_PD_168:
        LDR.N    R0,??choose_PD_28+0x4
        LDR.N    R1,??choose_PD_28+0x4
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
        LDR.N    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+2
        LDR.N    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+4]
// 3563           }
// 3564         }break;
??choose_PD_167:
        B.N      ??choose_PD_153
// 3565       case 3:
// 3566         {
// 3567           switch(track_form)
??choose_PD_150:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_170
        BCC.N    ??choose_PD_171
        CMP      R4,#+3
        BEQ.N    ??choose_PD_172
        BCC.N    ??choose_PD_173
        B.N      ??choose_PD_171
// 3568           {
// 3569           case 1:Servo.P+=2;Servo.D-=1;break;
??choose_PD_170:
        LDR.N    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+2
        LDR.N    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+0]
        LDR.N    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+1
        LDR.N    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_171
// 3570           case 2:Servo.P+=1;Servo.D-=2;break;
??choose_PD_173:
        LDR.N    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.N    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+0]
        LDR.N    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+2
        LDR.N    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_171
// 3571           case 3:Servo.P+=0;Servo.D-=3;break;
??choose_PD_172:
        LDR.N    R0,??choose_PD_28+0x4
        LDR.N    R1,??choose_PD_28+0x4
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
        LDR.N    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+3
        LDR.N    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+4]
// 3572           }
// 3573         }break;
??choose_PD_171:
        B.N      ??choose_PD_153
// 3574       case 2:
// 3575         {
// 3576           switch(track_form)
??choose_PD_147:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_174
        BCC.N    ??choose_PD_175
        CMP      R4,#+3
        BEQ.N    ??choose_PD_176
        BCC.N    ??choose_PD_177
        B.N      ??choose_PD_175
// 3577           {
// 3578           case 1:Servo.P+=2;Servo.D-=2;break;
??choose_PD_174:
        LDR.N    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+2
        LDR.N    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+0]
        LDR.N    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+2
        LDR.N    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_175
// 3579           case 2:Servo.P+=1;Servo.D-=3;break;
??choose_PD_177:
        LDR.N    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.N    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+0]
        LDR.N    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+3
        LDR.N    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_175
// 3580           case 3:Servo.P+=0;Servo.D-=4;break;
??choose_PD_176:
        LDR.N    R0,??choose_PD_28+0x4
        LDR.N    R1,??choose_PD_28+0x4
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
        LDR.N    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+4
        LDR.N    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+4]
// 3581           }
// 3582         }break;
??choose_PD_175:
        B.N      ??choose_PD_153
// 3583       case 1:
// 3584         {
// 3585           switch(track_form)
??choose_PD_148:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_178
        BCC.N    ??choose_PD_179
        CMP      R4,#+3
        BEQ.N    ??choose_PD_180
        BCC.N    ??choose_PD_181
        B.N      ??choose_PD_179
// 3586           {
// 3587           case 1:Servo.P+=2;Servo.D-=3;break;
??choose_PD_178:
        LDR.N    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+2
        LDR.N    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+0]
        LDR.N    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+3
        LDR.N    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_179
// 3588           case 2:Servo.P+=1;Servo.D-=4;break;
??choose_PD_181:
        LDR.N    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.N    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+0]
        LDR.N    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+4
        LDR.N    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_179
// 3589           case 3:Servo.P+=0;Servo.D-=5;break;
??choose_PD_180:
        LDR.N    R0,??choose_PD_28+0x4
        LDR.N    R1,??choose_PD_28+0x4
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
        LDR.N    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+5
        LDR.N    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+4]
// 3590           }
// 3591         }break;
// 3592       }
// 3593     }break;
??choose_PD_179:
??choose_PD_153:
        B.W      ??choose_PD_37
// 3594  case 3:
// 3595     {
// 3596       switch(position_now_D)
??choose_PD_34:
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        CMP      R0,#+0
        BEQ.N    ??choose_PD_182
        CMP      R0,#+2
        BEQ.W    ??choose_PD_183
        BCC.W    ??choose_PD_184
        CMP      R0,#+4
        BEQ.W    ??choose_PD_185
        BCC.W    ??choose_PD_186
        CMP      R0,#+6
        BEQ.N    ??choose_PD_187
        BCC.N    ??choose_PD_188
        B.N      ??choose_PD_189
// 3597       {
// 3598    case -6:
// 3599         {
// 3600           switch(track_form)
// 3601           {
// 3602         //case 1:Servo.P+=5;Servo.D+=1;break;//-6-6
// 3603           case 1:Servo.P+=1;Servo.D+=2;break;
// 3604           case 2:Servo.P+=0;Servo.D+=1;break;
// 3605           case 3:Servo.P-=1;Servo.D+=0;break;
// 3606           }
// 3607         }break;
// 3608       case -5:
// 3609         {
// 3610           switch(track_form)
// 3611           {
// 3612             //case 1:Servo.P+=5;Servo.D+=1;break;
// 3613           case 1:Servo.P+=1;Servo.D+=1;break;
// 3614           case 2:Servo.P+=0;Servo.D+=0;break;
// 3615           case 3:Servo.P-=1;Servo.D-=1;break;
// 3616           }
// 3617         }break;
// 3618       case -4:
// 3619         {
// 3620           switch(track_form)
// 3621           {
// 3622             case 1:Servo.P+=1;Servo.D+=0;break;
// 3623             case 2:Servo.P+=0;Servo.D-=1;break;
// 3624             case 3:Servo.P-=1;Servo.D-=2;break;
// 3625           }
// 3626         }break;
// 3627       case -3:
// 3628         {
// 3629           switch(track_form)
// 3630           {
// 3631           case 1:Servo.P+=1;Servo.D-=1;break;
// 3632           case 2:Servo.P+=0;Servo.D-=2;break;
// 3633           case 3:Servo.P-=1;Servo.D-=3;break;
// 3634           }
// 3635         }break;
// 3636       case -2:
// 3637         {
// 3638           switch(track_form)
// 3639           {
// 3640           case 1:Servo.P+=1;Servo.D-=2;break;
// 3641           case 2:Servo.P+=0;Servo.D-=3;break;
// 3642           case 3:Servo.P-=1;Servo.D-=4;break;
// 3643           }
// 3644         }break;
// 3645       case -1:
// 3646         {
// 3647           switch(track_form)
// 3648           {
// 3649           case 1:Servo.P+=1;Servo.D-=3;break;
// 3650           case 2:Servo.P+=0;Servo.D-=4;break;
// 3651           case 3:Servo.P-=1;Servo.D-=5;break;
// 3652           }
// 3653         }break;
// 3654       case 0:
// 3655          {
// 3656           switch(track_form)
??choose_PD_182:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_190
        BCC.N    ??choose_PD_191
        CMP      R4,#+3
        BEQ.N    ??choose_PD_192
        BCC.N    ??choose_PD_193
        B.N      ??choose_PD_191
// 3657           {
// 3658           case 1:Servo.P+=1;break;
??choose_PD_190:
        LDR.N    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.N    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+0]
        B.N      ??choose_PD_191
// 3659           case 2:Servo.P+=0;break;
??choose_PD_193:
        LDR.N    R0,??choose_PD_28+0x4
        LDR.N    R1,??choose_PD_28+0x4
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
        B.N      ??choose_PD_191
// 3660           case 3:Servo.P-=1;break;
??choose_PD_192:
        LDR.N    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+1
        LDR.N    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+0]
// 3661           }
// 3662         }break;
??choose_PD_191:
        B.N      ??choose_PD_189
// 3663       case 6:
// 3664         {
// 3665           switch(track_form)
??choose_PD_187:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_194
        BCC.N    ??choose_PD_195
        CMP      R4,#+3
        BEQ.N    ??choose_PD_196
        BCC.N    ??choose_PD_197
        B.N      ??choose_PD_195
// 3666           {
// 3667         //case 1:Servo.P+=5;Servo.D+=1;break;//-6-6
// 3668           case 1:Servo.P+=1;Servo.D+=2;break;
??choose_PD_194:
        LDR.N    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.N    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+0]
        LDR.N    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+4]
        ADDS     R0,R0,#+2
        LDR.N    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_195
// 3669           case 2:Servo.P+=0;Servo.D+=1;break;
??choose_PD_197:
        LDR.N    R0,??choose_PD_28+0x4
        LDR.N    R1,??choose_PD_28+0x4
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
        LDR.N    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+4]
        ADDS     R0,R0,#+1
        LDR.N    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_195
// 3670           case 3:Servo.P-=1;Servo.D+=0;break;
??choose_PD_196:
        LDR.N    R0,??choose_PD_28+0x4
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+1
        LDR.N    R1,??choose_PD_28+0x4
        STRH     R0,[R1, #+0]
        LDR.N    R0,??choose_PD_28+0x4
        LDR.N    R1,??choose_PD_28+0x4
        LDRH     R1,[R1, #+4]
        STRH     R1,[R0, #+4]
// 3671           }
// 3672         }break;
??choose_PD_195:
        B.N      ??choose_PD_189
        DATA
??choose_PD_28:
        DC32     position_diff
        DC32     Servo
        THUMB
// 3673       case 5:
// 3674         {
// 3675           switch(track_form)
??choose_PD_188:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_198
        BCC.N    ??choose_PD_199
        CMP      R4,#+3
        BEQ.N    ??choose_PD_200
        BCC.N    ??choose_PD_201
        B.N      ??choose_PD_199
// 3676           {
// 3677             //case 1:Servo.P+=5;Servo.D+=1;break;
// 3678           case 1:Servo.P+=1;Servo.D+=1;break;
??choose_PD_198:
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+0]
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+4]
        ADDS     R0,R0,#+1
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_199
// 3679           case 2:Servo.P+=0;Servo.D+=0;break;
??choose_PD_201:
        LDR.W    R0,??DataTable59
        LDR.W    R1,??DataTable59
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable59
        LDR.W    R1,??DataTable59
        LDRH     R1,[R1, #+4]
        STRH     R1,[R0, #+4]
        B.N      ??choose_PD_199
// 3680           case 3:Servo.P-=1;Servo.D-=1;break;
??choose_PD_200:
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+1
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+0]
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+1
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+4]
// 3681           }
// 3682         }break;
??choose_PD_199:
        B.N      ??choose_PD_189
// 3683       case 4:
// 3684         {
// 3685           switch(track_form)
??choose_PD_185:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_202
        BCC.N    ??choose_PD_203
        CMP      R4,#+3
        BEQ.N    ??choose_PD_204
        BCC.N    ??choose_PD_205
        B.N      ??choose_PD_203
// 3686           {
// 3687             case 1:Servo.P+=1;Servo.D+=0;break;
??choose_PD_202:
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+0]
        LDR.W    R0,??DataTable59
        LDR.W    R1,??DataTable59
        LDRH     R1,[R1, #+4]
        STRH     R1,[R0, #+4]
        B.N      ??choose_PD_203
// 3688             case 2:Servo.P+=0;Servo.D-=1;break;
??choose_PD_205:
        LDR.W    R0,??DataTable59
        LDR.W    R1,??DataTable59
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+1
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_203
// 3689             case 3:Servo.P-=1;Servo.D-=2;break;
??choose_PD_204:
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+1
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+0]
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+2
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+4]
// 3690           }
// 3691         }break;
??choose_PD_203:
        B.N      ??choose_PD_189
// 3692       case 3:
// 3693         {
// 3694           switch(track_form)
??choose_PD_186:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_206
        BCC.N    ??choose_PD_207
        CMP      R4,#+3
        BEQ.N    ??choose_PD_208
        BCC.N    ??choose_PD_209
        B.N      ??choose_PD_207
// 3695           {
// 3696           case 1:Servo.P+=1;Servo.D-=1;break;
??choose_PD_206:
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+0]
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+1
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_207
// 3697           case 2:Servo.P+=0;Servo.D-=2;break;
??choose_PD_209:
        LDR.W    R0,??DataTable59
        LDR.W    R1,??DataTable59
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+2
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_207
// 3698           case 3:Servo.P-=1;Servo.D-=3;break;
??choose_PD_208:
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+1
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+0]
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+3
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+4]
// 3699           }
// 3700         }break;
??choose_PD_207:
        B.N      ??choose_PD_189
// 3701       case 2:
// 3702         {
// 3703           switch(track_form)
??choose_PD_183:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_210
        BCC.N    ??choose_PD_211
        CMP      R4,#+3
        BEQ.N    ??choose_PD_212
        BCC.N    ??choose_PD_213
        B.N      ??choose_PD_211
// 3704           {
// 3705           case 1:Servo.P+=1;Servo.D-=2;break;
??choose_PD_210:
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+0]
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+2
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_211
// 3706           case 2:Servo.P+=0;Servo.D-=3;break;
??choose_PD_213:
        LDR.W    R0,??DataTable59
        LDR.W    R1,??DataTable59
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+3
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_211
// 3707           case 3:Servo.P-=1;Servo.D-=4;break;
??choose_PD_212:
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+1
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+0]
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+4
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+4]
// 3708           }
// 3709         }break;
??choose_PD_211:
        B.N      ??choose_PD_189
// 3710       case 1:
// 3711         {
// 3712           switch(track_form)
??choose_PD_184:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_214
        BCC.N    ??choose_PD_215
        CMP      R4,#+3
        BEQ.N    ??choose_PD_216
        BCC.N    ??choose_PD_217
        B.N      ??choose_PD_215
// 3713           {
// 3714           case 1:Servo.P+=1;Servo.D-=3;break;
??choose_PD_214:
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+0]
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+3
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_215
// 3715           case 2:Servo.P+=0;Servo.D-=4;break;
??choose_PD_217:
        LDR.W    R0,??DataTable59
        LDR.W    R1,??DataTable59
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+4
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_215
// 3716           case 3:Servo.P-=1;Servo.D-=5;break;
??choose_PD_216:
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+1
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+0]
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+5
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+4]
// 3717           }
// 3718         }break;
// 3719       }
// 3720     }break;
??choose_PD_215:
??choose_PD_189:
        B.N      ??choose_PD_37
// 3721  case 2:
// 3722     {
// 3723       switch(position_now_D)
??choose_PD_31:
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        CMP      R0,#+0
        BEQ.N    ??choose_PD_218
        CMP      R0,#+2
        BEQ.W    ??choose_PD_219
        BCC.W    ??choose_PD_220
        CMP      R0,#+4
        BEQ.W    ??choose_PD_221
        BCC.W    ??choose_PD_222
        CMP      R0,#+6
        BEQ.N    ??choose_PD_223
        BCC.N    ??choose_PD_224
        B.N      ??choose_PD_225
// 3724       {
// 3725      case -6:
// 3726         {
// 3727           switch(track_form)
// 3728           {
// 3729         //case 1:Servo.P+=5;Servo.D+=1;break;//-6-6
// 3730           case 1:Servo.P+=0;Servo.D+=2;break;
// 3731           case 2:Servo.P-=1;Servo.D+=1;break;
// 3732           case 3:Servo.P-=2;Servo.D+=0;break;
// 3733           }
// 3734         }break;
// 3735       case -5:
// 3736         {
// 3737           switch(track_form)
// 3738           {
// 3739             //case 1:Servo.P+=5;Servo.D+=1;break;
// 3740           case 1:Servo.P+=0;Servo.D+=1;break;
// 3741           case 2:Servo.P-=1;Servo.D+=0;break;
// 3742           case 3:Servo.P-=2;Servo.D-=1;break;
// 3743           }
// 3744         }break;
// 3745       case -4:
// 3746         {
// 3747           switch(track_form)
// 3748           {
// 3749             case 1:Servo.P+=0;Servo.D+=0;break;
// 3750             case 2:Servo.P-=1;Servo.D-=1;break;
// 3751             case 3:Servo.P-=2;Servo.D-=2;break;
// 3752           }
// 3753         }break;
// 3754       case -3:
// 3755         {
// 3756           switch(track_form)
// 3757           {
// 3758           case 1:Servo.P+=0;Servo.D-=1;break;
// 3759           case 2:Servo.P-=1;Servo.D-=2;break;
// 3760           case 3:Servo.P-=2;Servo.D-=3;break;
// 3761           }
// 3762         }break;
// 3763       case -2:
// 3764         {
// 3765           switch(track_form)
// 3766           {
// 3767           case 1:Servo.P+=0;Servo.D-=2;break;
// 3768           case 2:Servo.P-=1;Servo.D-=3;break;
// 3769           case 3:Servo.P-=2;Servo.D-=4;break;
// 3770           }
// 3771         }break;
// 3772       case -1:
// 3773         {
// 3774           switch(track_form)
// 3775           {
// 3776           case 1:Servo.P+=0;Servo.D-=3;break;
// 3777           case 2:Servo.P-=1;Servo.D-=4;break;
// 3778           case 3:Servo.P-=2;Servo.D-=5;break;
// 3779           }
// 3780         }break;      
// 3781       case 0:
// 3782          {
// 3783           switch(track_form)
??choose_PD_218:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_226
        BCC.N    ??choose_PD_227
        CMP      R4,#+3
        BEQ.N    ??choose_PD_228
        BCC.N    ??choose_PD_229
        B.N      ??choose_PD_227
// 3784           {
// 3785           case 1:Servo.P+=0;break;
??choose_PD_226:
        LDR.W    R0,??DataTable59
        LDR.W    R1,??DataTable59
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
        B.N      ??choose_PD_227
// 3786           case 2:Servo.P-=1;break;
??choose_PD_229:
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+1
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+0]
        B.N      ??choose_PD_227
// 3787           case 3:Servo.P-=2;break;
??choose_PD_228:
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+2
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+0]
// 3788           }
// 3789         }break;
??choose_PD_227:
        B.N      ??choose_PD_225
// 3790       case 6:
// 3791         {
// 3792           switch(track_form)
??choose_PD_223:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_230
        BCC.N    ??choose_PD_231
        CMP      R4,#+3
        BEQ.N    ??choose_PD_232
        BCC.N    ??choose_PD_233
        B.N      ??choose_PD_231
// 3793           {
// 3794         //case 1:Servo.P+=5;Servo.D+=1;break;//-6-6
// 3795           case 1:Servo.P+=0;Servo.D+=2;break;
??choose_PD_230:
        LDR.W    R0,??DataTable59
        LDR.W    R1,??DataTable59
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+4]
        ADDS     R0,R0,#+2
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_231
// 3796           case 2:Servo.P-=1;Servo.D+=1;break;
??choose_PD_233:
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+1
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+0]
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+4]
        ADDS     R0,R0,#+1
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_231
// 3797           case 3:Servo.P-=2;Servo.D+=0;break;
??choose_PD_232:
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+2
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+0]
        LDR.W    R0,??DataTable59
        LDR.W    R1,??DataTable59
        LDRH     R1,[R1, #+4]
        STRH     R1,[R0, #+4]
// 3798           }
// 3799         }break;
??choose_PD_231:
        B.N      ??choose_PD_225
// 3800       case 5:
// 3801         {
// 3802           switch(track_form)
??choose_PD_224:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_234
        BCC.N    ??choose_PD_235
        CMP      R4,#+3
        BEQ.N    ??choose_PD_236
        BCC.N    ??choose_PD_237
        B.N      ??choose_PD_235
// 3803           {
// 3804             //case 1:Servo.P+=5;Servo.D+=1;break;
// 3805           case 1:Servo.P+=0;Servo.D+=1;break;
??choose_PD_234:
        LDR.W    R0,??DataTable59
        LDR.W    R1,??DataTable59
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+4]
        ADDS     R0,R0,#+1
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_235
// 3806           case 2:Servo.P-=1;Servo.D+=0;break;
??choose_PD_237:
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+1
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+0]
        LDR.W    R0,??DataTable59
        LDR.W    R1,??DataTable59
        LDRH     R1,[R1, #+4]
        STRH     R1,[R0, #+4]
        B.N      ??choose_PD_235
// 3807           case 3:Servo.P-=2;Servo.D-=1;break;
??choose_PD_236:
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+2
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+0]
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+1
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+4]
// 3808           }
// 3809         }break;
??choose_PD_235:
        B.N      ??choose_PD_225
// 3810       case 4:
// 3811         {
// 3812           switch(track_form)
??choose_PD_221:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_238
        BCC.N    ??choose_PD_239
        CMP      R4,#+3
        BEQ.N    ??choose_PD_240
        BCC.N    ??choose_PD_241
        B.N      ??choose_PD_239
// 3813           {
// 3814             case 1:Servo.P+=0;Servo.D+=0;break;
??choose_PD_238:
        LDR.W    R0,??DataTable59
        LDR.W    R1,??DataTable59
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable59
        LDR.W    R1,??DataTable59
        LDRH     R1,[R1, #+4]
        STRH     R1,[R0, #+4]
        B.N      ??choose_PD_239
// 3815             case 2:Servo.P-=1;Servo.D-=1;break;
??choose_PD_241:
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+1
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+0]
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+1
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_239
// 3816             case 3:Servo.P-=2;Servo.D-=2;break;
??choose_PD_240:
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+2
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+0]
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+2
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+4]
// 3817           }
// 3818         }break;
??choose_PD_239:
        B.N      ??choose_PD_225
// 3819       case 3:
// 3820         {
// 3821           switch(track_form)
??choose_PD_222:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_242
        BCC.N    ??choose_PD_243
        CMP      R4,#+3
        BEQ.N    ??choose_PD_244
        BCC.N    ??choose_PD_245
        B.N      ??choose_PD_243
// 3822           {
// 3823           case 1:Servo.P+=0;Servo.D-=1;break;
??choose_PD_242:
        LDR.W    R0,??DataTable59
        LDR.W    R1,??DataTable59
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+1
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_243
// 3824           case 2:Servo.P-=1;Servo.D-=2;break;
??choose_PD_245:
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+1
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+0]
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+2
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_243
// 3825           case 3:Servo.P-=2;Servo.D-=3;break;
??choose_PD_244:
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+2
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+0]
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+3
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+4]
// 3826           }
// 3827         }break;
??choose_PD_243:
        B.N      ??choose_PD_225
// 3828       case 2:
// 3829         {
// 3830           switch(track_form)
??choose_PD_219:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_246
        BCC.N    ??choose_PD_247
        CMP      R4,#+3
        BEQ.N    ??choose_PD_248
        BCC.N    ??choose_PD_249
        B.N      ??choose_PD_247
// 3831           {
// 3832           case 1:Servo.P+=0;Servo.D-=2;break;
??choose_PD_246:
        LDR.W    R0,??DataTable59
        LDR.W    R1,??DataTable59
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+2
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_247
// 3833           case 2:Servo.P-=1;Servo.D-=3;break;
??choose_PD_249:
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+1
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+0]
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+3
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_247
// 3834           case 3:Servo.P-=2;Servo.D-=4;break;
??choose_PD_248:
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+2
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+0]
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+4
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+4]
// 3835           }
// 3836         }break;
??choose_PD_247:
        B.N      ??choose_PD_225
// 3837       case 1:
// 3838         {
// 3839           switch(track_form)
??choose_PD_220:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_250
        BCC.N    ??choose_PD_251
        CMP      R4,#+3
        BEQ.N    ??choose_PD_252
        BCC.N    ??choose_PD_253
        B.N      ??choose_PD_251
// 3840           {
// 3841           case 1:Servo.P+=0;Servo.D-=3;break;
??choose_PD_250:
        LDR.W    R0,??DataTable59
        LDR.W    R1,??DataTable59
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+3
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_251
// 3842           case 2:Servo.P-=1;Servo.D-=4;break;
??choose_PD_253:
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+1
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+0]
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+4
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_251
// 3843           case 3:Servo.P-=2;Servo.D-=5;break;
??choose_PD_252:
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+2
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+0]
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+5
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+4]
// 3844           }
// 3845         }break;
// 3846       }
// 3847     }break;
??choose_PD_251:
??choose_PD_225:
        B.N      ??choose_PD_37
// 3848 case 1:
// 3849     {
// 3850       switch(position_now_D)
??choose_PD_32:
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        CMP      R0,#+0
        BEQ.N    ??choose_PD_254
        CMP      R0,#+2
        BEQ.W    ??choose_PD_255
        BCC.W    ??choose_PD_256
        CMP      R0,#+4
        BEQ.W    ??choose_PD_257
        BCC.W    ??choose_PD_258
        CMP      R0,#+6
        BEQ.N    ??choose_PD_259
        BCC.N    ??choose_PD_260
        B.N      ??choose_PD_261
// 3851       {
// 3852        case -6:
// 3853         {
// 3854           switch(track_form)
// 3855           {
// 3856         //case 1:Servo.P+=5;Servo.D+=1;break;//-6-6
// 3857           case 1:Servo.P-=1;Servo.D+=2;break;
// 3858           case 2:Servo.P-=2;Servo.D+=1;break;
// 3859           case 3:Servo.P-=3;Servo.D+=0;break;
// 3860           }
// 3861         }break;
// 3862       case -5:
// 3863         {
// 3864           switch(track_form)
// 3865           {
// 3866             //case 1:Servo.P+=5;Servo.D+=1;break;
// 3867           case 1:Servo.P-=1;Servo.D+=1;break;
// 3868           case 2:Servo.P-=2;Servo.D+=0;break;
// 3869           case 3:Servo.P-=3;Servo.D-=1;break;
// 3870           }
// 3871         }break;
// 3872       case -4:
// 3873         {
// 3874           switch(track_form)
// 3875           {
// 3876             case 1:Servo.P-=1;Servo.D+=0;break;
// 3877             case 2:Servo.P-=2;Servo.D-=1;break;
// 3878             case 3:Servo.P-=3;Servo.D-=2;break;
// 3879           }
// 3880         }break;
// 3881       case -3:
// 3882         {
// 3883           switch(track_form)
// 3884           {
// 3885           case 1:Servo.P-=1;Servo.D-=1;break;
// 3886           case 2:Servo.P-=2;Servo.D-=2;break;
// 3887           case 3:Servo.P-=3;Servo.D-=3;break;
// 3888           }
// 3889         }break;
// 3890       case -2:
// 3891         {
// 3892           switch(track_form)
// 3893           {
// 3894           case 1:Servo.P-=1;Servo.D-=2;break;
// 3895           case 2:Servo.P-=2;Servo.D-=3;break;
// 3896           case 3:Servo.P-=3;Servo.D-=4;break;
// 3897           }
// 3898         }break;
// 3899       case -1:
// 3900         {
// 3901           switch(track_form)
// 3902           {
// 3903           case 1:Servo.P-=1;Servo.D-=3;break;
// 3904           case 2:Servo.P-=2;Servo.D-=4;break;
// 3905           case 3:Servo.P-=3;Servo.D-=5;break;
// 3906           }
// 3907         }break;
// 3908       case 0:
// 3909          {
// 3910           switch(track_form)
??choose_PD_254:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_262
        BCC.N    ??choose_PD_263
        CMP      R4,#+3
        BEQ.N    ??choose_PD_264
        BCC.N    ??choose_PD_265
        B.N      ??choose_PD_263
// 3911           {
// 3912           case 1:Servo.P-=1;break;
??choose_PD_262:
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+1
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+0]
        B.N      ??choose_PD_263
// 3913           case 2:Servo.P-=2;break;
??choose_PD_265:
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+2
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+0]
        B.N      ??choose_PD_263
// 3914           case 3:Servo.P-=3;break;
??choose_PD_264:
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+3
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+0]
// 3915           }
// 3916         }break;
??choose_PD_263:
        B.N      ??choose_PD_261
// 3917       case 6:
// 3918         {
// 3919           switch(track_form)
??choose_PD_259:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_266
        BCC.N    ??choose_PD_267
        CMP      R4,#+3
        BEQ.N    ??choose_PD_268
        BCC.N    ??choose_PD_269
        B.N      ??choose_PD_267
// 3920           {
// 3921         //case 1:Servo.P+=5;Servo.D+=1;break;//-6-6
// 3922           case 1:Servo.P-=1;Servo.D+=2;break;
??choose_PD_266:
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+1
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+0]
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+4]
        ADDS     R0,R0,#+2
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_267
// 3923           case 2:Servo.P-=2;Servo.D+=1;break;
??choose_PD_269:
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+2
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+0]
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+4]
        ADDS     R0,R0,#+1
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_267
// 3924           case 3:Servo.P-=3;Servo.D+=0;break;
??choose_PD_268:
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+3
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+0]
        LDR.W    R0,??DataTable59
        LDR.W    R1,??DataTable59
        LDRH     R1,[R1, #+4]
        STRH     R1,[R0, #+4]
// 3925           }
// 3926         }break;
??choose_PD_267:
        B.N      ??choose_PD_261
// 3927       case 5:
// 3928         {
// 3929           switch(track_form)
??choose_PD_260:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_270
        BCC.N    ??choose_PD_271
        CMP      R4,#+3
        BEQ.N    ??choose_PD_272
        BCC.N    ??choose_PD_273
        B.N      ??choose_PD_271
// 3930           {
// 3931             //case 1:Servo.P+=5;Servo.D+=1;break;
// 3932           case 1:Servo.P-=1;Servo.D+=1;break;
??choose_PD_270:
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+1
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+0]
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+4]
        ADDS     R0,R0,#+1
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_271
// 3933           case 2:Servo.P-=2;Servo.D+=0;break;
??choose_PD_273:
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+2
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+0]
        LDR.W    R0,??DataTable59
        LDR.W    R1,??DataTable59
        LDRH     R1,[R1, #+4]
        STRH     R1,[R0, #+4]
        B.N      ??choose_PD_271
// 3934           case 3:Servo.P-=3;Servo.D-=1;break;
??choose_PD_272:
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+3
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+0]
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+1
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+4]
// 3935           }
// 3936         }break;
??choose_PD_271:
        B.N      ??choose_PD_261
// 3937       case 4:
// 3938         {
// 3939           switch(track_form)
??choose_PD_257:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_274
        BCC.N    ??choose_PD_275
        CMP      R4,#+3
        BEQ.N    ??choose_PD_276
        BCC.N    ??choose_PD_277
        B.N      ??choose_PD_275
// 3940           {
// 3941             case 1:Servo.P-=1;Servo.D+=0;break;
??choose_PD_274:
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+1
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+0]
        LDR.W    R0,??DataTable59
        LDR.W    R1,??DataTable59
        LDRH     R1,[R1, #+4]
        STRH     R1,[R0, #+4]
        B.N      ??choose_PD_275
// 3942             case 2:Servo.P-=2;Servo.D-=1;break;
??choose_PD_277:
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+2
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+0]
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+1
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_275
// 3943             case 3:Servo.P-=3;Servo.D-=2;break;
??choose_PD_276:
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+3
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+0]
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+2
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+4]
// 3944           }
// 3945         }break;
??choose_PD_275:
        B.N      ??choose_PD_261
// 3946       case 3:
// 3947         {
// 3948           switch(track_form)
??choose_PD_258:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_278
        BCC.N    ??choose_PD_279
        CMP      R4,#+3
        BEQ.N    ??choose_PD_280
        BCC.N    ??choose_PD_281
        B.N      ??choose_PD_279
// 3949           {
// 3950           case 1:Servo.P-=1;Servo.D-=1;break;
??choose_PD_278:
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+1
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+0]
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+1
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_279
// 3951           case 2:Servo.P-=2;Servo.D-=2;break;
??choose_PD_281:
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+2
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+0]
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+2
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_279
// 3952           case 3:Servo.P-=3;Servo.D-=3;break;
??choose_PD_280:
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+3
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+0]
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+3
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+4]
// 3953           }
// 3954         }break;
??choose_PD_279:
        B.N      ??choose_PD_261
// 3955       case 2:
// 3956         {
// 3957           switch(track_form)
??choose_PD_255:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_282
        BCC.N    ??choose_PD_283
        CMP      R4,#+3
        BEQ.N    ??choose_PD_284
        BCC.N    ??choose_PD_285
        B.N      ??choose_PD_283
// 3958           {
// 3959           case 1:Servo.P-=1;Servo.D-=2;break;
??choose_PD_282:
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+1
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+0]
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+2
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_283
// 3960           case 2:Servo.P-=2;Servo.D-=3;break;
??choose_PD_285:
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+2
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+0]
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+3
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_283
// 3961           case 3:Servo.P-=3;Servo.D-=4;break;
??choose_PD_284:
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+3
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+0]
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+4
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+4]
// 3962           }
// 3963         }break;
??choose_PD_283:
        B.N      ??choose_PD_261
// 3964       case 1:
// 3965         {
// 3966           switch(track_form)
??choose_PD_256:
        UXTB     R4,R4            ;; ZeroExt  R4,R4,#+24,#+24
        CMP      R4,#+1
        BEQ.N    ??choose_PD_286
        BCC.N    ??choose_PD_287
        CMP      R4,#+3
        BEQ.N    ??choose_PD_288
        BCC.N    ??choose_PD_289
        B.N      ??choose_PD_287
// 3967           {
// 3968           case 1:Servo.P-=1;Servo.D-=3;break;
??choose_PD_286:
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+1
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+0]
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+3
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_287
// 3969           case 2:Servo.P-=2;Servo.D-=4;break;
??choose_PD_289:
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+2
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+0]
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+4
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+4]
        B.N      ??choose_PD_287
// 3970           case 3:Servo.P-=3;Servo.D-=5;break;
??choose_PD_288:
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+3
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+0]
        LDR.W    R0,??DataTable59
        LDRH     R0,[R0, #+4]
        SUBS     R0,R0,#+5
        LDR.W    R1,??DataTable59
        STRH     R0,[R1, #+4]
// 3971           }
// 3972         }break;
// 3973       }
// 3974     }break;
// 3975   }
// 3976   //if(sai_dao_lei_xing==12&&abs(position_now_low_P)<2&&abs(position_now_P)<3)Servo.D+=20;
// 3977   //if(track_form==3&&abs(position_now_low_P)<1&&abs(position_now)<2)Servo.D+=25;
// 3978   //if(track_form==3&&abs(position_now_low_P)==1&&abs(position_now)==2)Servo.D+=18;
// 3979  // Servo_value=Servo_Middle+Servo.P*(79-position_now)+Servo.D*position_diff;//PD控制舵机
// 3980  
// 3981   Servo_value=Servo_Middle+Servo.P*(position_mid-position_now)+Servo.D*position_diff;//PD控制舵机
??choose_PD_287:
??choose_PD_261:
??choose_PD_37:
        LDR.W    R0,??DataTable61
        LDRSH    R0,[R0, #+0]
        LDR.W    R1,??DataTable59
        LDRSH    R1,[R1, #+0]
        LDR.W    R2,??DataTable61_1
        LDRSH    R2,[R2, #+0]
        RSBS     R2,R2,#+77
        MULS     R1,R2,R1
        ADDS     R0,R1,R0
        LDR.W    R1,??DataTable59
        LDRSH    R1,[R1, #+4]
        LDR.W    R2,??DataTable61_2
        LDRSH    R2,[R2, #+0]
        MLA      R0,R2,R1,R0
        LDR.W    R1,??DataTable61_3
        STRH     R0,[R1, #+0]
// 3982   Servo_value=limit(Servo_value,Servo_Left,Servo_Right);
        LDR.W    R0,??DataTable61_4
        LDRSH    R2,[R0, #+0]
        LDR.W    R0,??DataTable61_5
        LDRSH    R1,[R0, #+0]
        LDR.W    R0,??DataTable61_3
        LDRSH    R0,[R0, #+0]
        BL       limit
        LDR.W    R1,??DataTable61_3
        STRH     R0,[R1, #+0]
// 3983   //if(abs(Servo_value-Servo_value_old)>2000)Servo_value=Servo_value_old;
// 3984   //Servo_value=0.1*Servo_value_old+0.9*Servo_value;
// 3985   Servo_value_old=Servo_value;
        LDR.W    R0,??DataTable61_6
        LDR.W    R1,??DataTable61_3
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
// 3986   FTM1_C0V=Servo_value;  
        LDR.W    R0,??DataTable61_7  ;; 0x40039010
        LDR.W    R1,??DataTable61_3
        LDRSH    R1,[R1, #+0]
        STR      R1,[R0, #+0]
// 3987   return ok;
        MOVS     R0,#+1
??choose_PD_18:
        POP      {R4-R10,PC}      ;; return
// 3988 }
// 3989 

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 3990 int16 stages3(int16 value,int16 parameter,int16 windage_k)
// 3991 {
// 3992  
// 3993     if(!value) return unclear;
stages3:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        CMP      R0,#+0
        BNE.N    ??stages3_0
        MVNS     R0,#+99
        B.N      ??stages3_1
// 3994     if(value<(parameter-windage_k)){
??stages3_0:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        SXTH     R2,R2            ;; SignExt  R2,R2,#+16,#+16
        SUBS     R3,R1,R2
        CMP      R0,R3
        BGE.N    ??stages3_2
// 3995       NULL;
// 3996       return(-1);
        MOVS     R0,#-1
        B.N      ??stages3_1
// 3997     }
// 3998     else if(value>=(parameter+windage_k)){
??stages3_2:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        SXTH     R2,R2            ;; SignExt  R2,R2,#+16,#+16
        SXTAH    R1,R2,R1
        CMP      R0,R1
        BLT.N    ??stages3_3
// 3999       NULL;
// 4000       return(1); 
        MOVS     R0,#+1
        B.N      ??stages3_1
// 4001     }
// 4002          else return(0);
??stages3_3:
        MOVS     R0,#+0
??stages3_1:
        BX       LR               ;; return
// 4003 }
// 4004 
// 4005 
// 4006 
// 4007 
// 4008 /*************判断赛道类型0**************/

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 4009 uint8 judge_locus_0(void)
// 4010 {
judge_locus_0:
        PUSH     {R3-R5,LR}
// 4011   int16 i=0,num=0,buff1=0;
        MOVS     R0,#+0
        MOVS     R4,#+0
        MOVS     R1,#+0
// 4012   uint8 sai_dao_lei_xing_0=0;
        MOVS     R5,#+0
// 4013  // for(i=54;i>21;i--) 
// 4014   for(i=54;i>25;i--)//前段   59~41  33
        MOVS     R2,#+54
        MOVS     R0,R2
        B.N      ??judge_locus_0_0
// 4015     if(position[i])
??judge_locus_0_1:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R1,??DataTable61_8
        LDR      R1,[R1, R0, LSL #+2]
        CMP      R1,#+0
        BEQ.N    ??judge_locus_0_2
// 4016     {
// 4017       low_0+=position[i];
        LDR.W    R1,??DataTable61_9
        LDRH     R1,[R1, #+0]
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R2,??DataTable61_8
        LDR      R2,[R2, R0, LSL #+2]
        ADDS     R1,R2,R1
        LDR.W    R2,??DataTable61_9
        STRH     R1,[R2, #+0]
// 4018       num++;
        ADDS     R4,R4,#+1
// 4019     }  
??judge_locus_0_2:
        SUBS     R0,R0,#+1
??judge_locus_0_0:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        CMP      R0,#+26
        BGE.N    ??judge_locus_0_1
// 4020   if(low_0)
        LDR.W    R0,??DataTable61_9
        LDRSH    R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.N    ??judge_locus_0_3
// 4021     low_0=low_0/num;
        LDR.W    R0,??DataTable61_9
        LDRSH    R0,[R0, #+0]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        SDIV     R0,R0,R4
        LDR.W    R1,??DataTable61_9
        STRH     R0,[R1, #+0]
// 4022   num=0;
??judge_locus_0_3:
        MOVS     R4,#+0
// 4023     
// 4024  // for(i=21;i>7;i--) //中段  40~26    20  15 
// 4025   for(i=25;i>8;i--)
        MOVS     R0,#+25
        B.N      ??judge_locus_0_4
// 4026     if(position[i]){
??judge_locus_0_5:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R1,??DataTable61_8
        LDR      R1,[R1, R0, LSL #+2]
        CMP      R1,#+0
        BEQ.N    ??judge_locus_0_6
// 4027       mid_0+=position[i];
        LDR.W    R1,??DataTable61_10
        LDRH     R1,[R1, #+0]
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R2,??DataTable61_8
        LDR      R2,[R2, R0, LSL #+2]
        ADDS     R1,R2,R1
        LDR.W    R2,??DataTable61_10
        STRH     R1,[R2, #+0]
// 4028       num++;
        ADDS     R4,R4,#+1
// 4029 
// 4030     }
??judge_locus_0_6:
        SUBS     R0,R0,#+1
??judge_locus_0_4:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        CMP      R0,#+9
        BGE.N    ??judge_locus_0_5
// 4031   if(mid_0)
        LDR.W    R0,??DataTable61_10
        LDRSH    R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.N    ??judge_locus_0_7
// 4032     mid_0=mid_0/num;
        LDR.W    R0,??DataTable61_10
        LDRSH    R0,[R0, #+0]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        SDIV     R0,R0,R4
        LDR.W    R1,??DataTable61_10
        STRH     R0,[R1, #+0]
// 4033   num=0;    
??judge_locus_0_7:
        MOVS     R4,#+0
// 4034   
// 4035     for(i=8;i>0;i--)//远段    25~0
        MOVS     R0,#+8
        B.N      ??judge_locus_0_8
// 4036     if(position[i])
??judge_locus_0_9:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R1,??DataTable61_8
        LDR      R1,[R1, R0, LSL #+2]
        CMP      R1,#+0
        BEQ.N    ??judge_locus_0_10
// 4037     {
// 4038       high_0+=position[i];
        LDR.W    R1,??DataTable61_11
        LDRH     R1,[R1, #+0]
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        LDR.W    R2,??DataTable61_8
        LDR      R2,[R2, R0, LSL #+2]
        ADDS     R1,R2,R1
        LDR.W    R2,??DataTable61_11
        STRH     R1,[R2, #+0]
// 4039       num++;
        ADDS     R4,R4,#+1
// 4040     }
??judge_locus_0_10:
        SUBS     R0,R0,#+1
??judge_locus_0_8:
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        CMP      R0,#+1
        BGE.N    ??judge_locus_0_9
// 4041    if(high_0)
        LDR.W    R0,??DataTable61_11
        LDRSH    R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.N    ??judge_locus_0_11
// 4042      high_0=high_0/num;
        LDR.W    R0,??DataTable61_11
        LDRSH    R0,[R0, #+0]
        SXTH     R4,R4            ;; SignExt  R4,R4,#+16,#+16
        SDIV     R0,R0,R4
        LDR.W    R1,??DataTable61_11
        STRH     R0,[R1, #+0]
// 4043    num=0;
??judge_locus_0_11:
        MOVS     R4,#+0
// 4044    
// 4045    low_now_0=low_0;                      //远近中端的坐标位置
        LDR.W    R0,??DataTable61_12
        LDR.W    R1,??DataTable61_9
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
// 4046    mid_now_0=mid_0;
        LDR.W    R0,??DataTable61_13
        LDR.W    R1,??DataTable61_10
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
// 4047    high_now_0=high_0; 
        LDR.W    R0,??DataTable61_14
        LDR.W    R1,??DataTable61_11
        LDRH     R1,[R1, #+0]
        STRH     R1,[R0, #+0]
// 4048   
// 4049    low_0=stages3(low_0,video_Middle,24);        // 22          low,mid,high就是1，0，-1
        MOVS     R2,#+24
        MOVS     R1,#+1
        LDR.W    R0,??DataTable61_9
        LDRSH    R0,[R0, #+0]
        BL       stages3
        LDR.W    R1,??DataTable61_9
        STRH     R0,[R1, #+0]
// 4050    mid_0=stages3(mid_0,video_Middle,22);        // 19
        MOVS     R2,#+22
        MOVS     R1,#+1
        LDR.W    R0,??DataTable61_10
        LDRSH    R0,[R0, #+0]
        BL       stages3
        LDR.W    R1,??DataTable61_10
        STRH     R0,[R1, #+0]
// 4051    high_0=stages3(high_0,video_Middle,18);     //16
        MOVS     R2,#+18
        MOVS     R1,#+1
        LDR.W    R0,??DataTable61_11
        LDRSH    R0,[R0, #+0]
        BL       stages3
        LDR.W    R1,??DataTable61_11
        STRH     R0,[R1, #+0]
// 4052    if(high_0==unclear)
        LDR.W    R0,??DataTable61_11
        LDRSH    R0,[R0, #+0]
        MVNS     R1,#+99
        CMP      R0,R1
        BNE.N    ??judge_locus_0_12
// 4053    {    
// 4054      buff1=low_0*mid_0;
        LDR.W    R0,??DataTable61_9
        LDRH     R0,[R0, #+0]
        LDR.W    R1,??DataTable61_10
        LDRH     R1,[R1, #+0]
        MUL      R1,R1,R0
// 4055      if(mid_0==unclear)
        LDR.W    R0,??DataTable61_10
        LDRSH    R0,[R0, #+0]
        MVNS     R2,#+99
        CMP      R0,R2
        BNE.N    ??judge_locus_0_13
// 4056      { //超急弯
// 4057        if(low_0==0)
        LDR.W    R0,??DataTable61_9
        LDRSH    R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??judge_locus_0_14
// 4058        {              
// 4059          sai_dao_lei_xing_0=0;        
        MOVS     R5,#+0
        B.N      ??judge_locus_0_15
// 4060        }
// 4061        else
// 4062        {          
// 4063          sai_dao_lei_xing_0=1;         
??judge_locus_0_14:
        MOVS     R5,#+1
        B.N      ??judge_locus_0_15
// 4064        }       
// 4065      }
// 4066      else
// 4067      {                                                  
// 4068        if(buff1==1)
??judge_locus_0_13:
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        CMP      R1,#+1
        BNE.N    ??judge_locus_0_16
// 4069        {   //急弯        
// 4070          sai_dao_lei_xing_0=2;                  
        MOVS     R5,#+2
        B.N      ??judge_locus_0_15
// 4071        }
// 4072        else
// 4073        if(buff1==0)
??judge_locus_0_16:
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        CMP      R1,#+0
        BNE.N    ??judge_locus_0_17
// 4074        {  //急弯
// 4075          if(low_0!=0) 
        LDR.W    R0,??DataTable61_9
        LDRSH    R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.N    ??judge_locus_0_18
// 4076          {           
// 4077            sai_dao_lei_xing_0=4;           
        MOVS     R5,#+4
        B.N      ??judge_locus_0_15
// 4078          }
// 4079          else
// 4080          {
// 4081            if(mid_0==0&&low_0==0)
??judge_locus_0_18:
        LDR.W    R0,??DataTable61_10
        LDRSH    R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??judge_locus_0_19
        LDR.W    R0,??DataTable61_9
        LDRSH    R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??judge_locus_0_19
// 4082            {
// 4083              sai_dao_lei_xing_0=5;                         
        MOVS     R5,#+5
// 4084            }            
// 4085            sai_dao_lei_xing_0=6;           
??judge_locus_0_19:
        MOVS     R5,#+6
        B.N      ??judge_locus_0_15
// 4086          }           
// 4087        }
// 4088        else if(buff1==-1)
??judge_locus_0_17:
        MOVS     R0,#-1
        SXTH     R1,R1            ;; SignExt  R1,R1,#+16,#+16
        CMP      R1,R0
        BNE.W    ??judge_locus_0_15
// 4089        { // 超急弯         
// 4090          sai_dao_lei_xing_0=8;                    
        MOVS     R5,#+8
        B.N      ??judge_locus_0_15
// 4091        }
// 4092      }                                         
// 4093    }
// 4094    else
// 4095    {           //high!=unclear时
// 4096      if(mid_0==high_0)
??judge_locus_0_12:
        LDR.W    R0,??DataTable61_10
        LDRSH    R0,[R0, #+0]
        LDR.W    R1,??DataTable61_11
        LDRSH    R1,[R1, #+0]
        CMP      R0,R1
        BNE.N    ??judge_locus_0_20
// 4097      {
// 4098        if(high_0==0)
        LDR.W    R0,??DataTable61_11
        LDRSH    R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??judge_locus_0_21
// 4099        {        //直道         
// 4100          if(high_now_0>70&&high_now_0<90&&mid_now_0>70&&mid_now_0<90&&low_now_0>70&&low_now_0<90)
        LDR.W    R0,??DataTable61_14
        LDRSH    R0,[R0, #+0]
        CMP      R0,#+71
        BLT.N    ??judge_locus_0_22
        LDR.N    R0,??DataTable61_14
        LDRSH    R0,[R0, #+0]
        CMP      R0,#+90
        BGE.N    ??judge_locus_0_22
        LDR.N    R0,??DataTable61_13
        LDRSH    R0,[R0, #+0]
        CMP      R0,#+71
        BLT.N    ??judge_locus_0_22
        LDR.N    R0,??DataTable61_13
        LDRSH    R0,[R0, #+0]
        CMP      R0,#+90
        BGE.N    ??judge_locus_0_22
        LDR.N    R0,??DataTable61_12
        LDRSH    R0,[R0, #+0]
        CMP      R0,#+71
        BLT.N    ??judge_locus_0_22
        LDR.N    R0,??DataTable61_12
        LDRSH    R0,[R0, #+0]
        CMP      R0,#+90
        BGE.N    ??judge_locus_0_22
// 4101          {
// 4102            sai_dao_lei_xing_0=20;
        MOVS     R5,#+20
// 4103            Very_straight=1; 
        LDR.N    R0,??DataTable61_15
        MOVS     R1,#+1
        STRB     R1,[R0, #+0]
        B.N      ??judge_locus_0_15
// 4104          }
// 4105          else sai_dao_lei_xing_0=9;            
??judge_locus_0_22:
        MOVS     R5,#+9
        B.N      ??judge_locus_0_15
// 4106        }
// 4107        else
// 4108        {           /////////////////////////         
// 4109          sai_dao_lei_xing_0=10;                
??judge_locus_0_21:
        MOVS     R5,#+10
        B.N      ??judge_locus_0_15
// 4110        }
// 4111      }
// 4112      else
// 4113      {     //mid!=high时
// 4114        if(low_0==0&&mid_0==0)
??judge_locus_0_20:
        LDR.N    R0,??DataTable61_9
        LDRSH    R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??judge_locus_0_23
        LDR.N    R0,??DataTable61_10
        LDRSH    R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??judge_locus_0_23
// 4115        {         //长大弯         
// 4116          sai_dao_lei_xing_0=12;               
        MOVS     R5,#+12
// 4117        }//else
// 4118        if(mid_0==0&&(low_0*high_0==-1))
??judge_locus_0_23:
        LDR.N    R0,??DataTable61_10
        LDRSH    R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??judge_locus_0_24
        LDR.N    R0,??DataTable61_9
        LDRSH    R0,[R0, #+0]
        LDR.N    R1,??DataTable61_11
        LDRSH    R1,[R1, #+0]
        MULS     R0,R1,R0
        CMN      R0,#+1
        BNE.N    ??judge_locus_0_24
// 4119        {  //大弯         
// 4120          sai_dao_lei_xing_0=13;                  
        MOVS     R5,#+13
// 4121        }//else
// 4122        if(high_0==low_0&&low_0!=mid_0)
??judge_locus_0_24:
        LDR.N    R0,??DataTable61_11
        LDRSH    R0,[R0, #+0]
        LDR.N    R1,??DataTable61_9
        LDRSH    R1,[R1, #+0]
        CMP      R0,R1
        BNE.N    ??judge_locus_0_25
        LDR.N    R0,??DataTable61_9
        LDRSH    R0,[R0, #+0]
        LDR.N    R1,??DataTable61_10
        LDRSH    R1,[R1, #+0]
        CMP      R0,R1
        BEQ.N    ??judge_locus_0_25
// 4123        {    //S弯         
// 4124          sai_dao_lei_xing_0=14;           
        MOVS     R5,#+14
// 4125        }
// 4126        /////////////////////////////////////////////////////
// 4127        if(high_0==0&&(mid_0*low_0==-1))
??judge_locus_0_25:
        LDR.N    R0,??DataTable61_11
        LDRSH    R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??judge_locus_0_26
        LDR.N    R0,??DataTable61_10
        LDRSH    R0,[R0, #+0]
        LDR.N    R1,??DataTable61_9
        LDRSH    R1,[R1, #+0]
        MULS     R0,R1,R0
        CMN      R0,#+1
        BNE.N    ??judge_locus_0_26
// 4128        {        
// 4129          sai_dao_lei_xing_0=15;               
        MOVS     R5,#+15
// 4130        }
// 4131        if(high_0==0&&(mid_0*low_0==1))
??judge_locus_0_26:
        LDR.N    R0,??DataTable61_11
        LDRSH    R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??judge_locus_0_27
        LDR.N    R0,??DataTable61_10
        LDRSH    R0,[R0, #+0]
        LDR.N    R1,??DataTable61_9
        LDRSH    R1,[R1, #+0]
        MULS     R0,R1,R0
        CMP      R0,#+1
        BNE.N    ??judge_locus_0_27
// 4132        {         //长大弯        
// 4133          sai_dao_lei_xing_0=16;                
        MOVS     R5,#+16
// 4134        }
// 4135        if((high==-1&&mid==1&&low!=-1)||(high==1&&mid==-1&&low!=1))
??judge_locus_0_27:
        LDR.N    R0,??DataTable61_16
        LDRSH    R0,[R0, #+0]
        MOVS     R1,#-1
        CMP      R0,R1
        BNE.N    ??judge_locus_0_28
        LDR.N    R0,??DataTable61_17
        LDRSH    R0,[R0, #+0]
        CMP      R0,#+1
        BNE.N    ??judge_locus_0_28
        LDR.W    R0,??DataTable62
        LDRSH    R0,[R0, #+0]
        MOVS     R1,#-1
        CMP      R0,R1
        BNE.N    ??judge_locus_0_29
??judge_locus_0_28:
        LDR.N    R0,??DataTable61_16
        LDRSH    R0,[R0, #+0]
        CMP      R0,#+1
        BNE.N    ??judge_locus_0_15
        LDR.N    R0,??DataTable61_17
        LDRSH    R0,[R0, #+0]
        MOVS     R1,#-1
        CMP      R0,R1
        BNE.N    ??judge_locus_0_15
        LDR.W    R0,??DataTable62
        LDRSH    R0,[R0, #+0]
        CMP      R0,#+1
        BEQ.N    ??judge_locus_0_15
// 4136        {         //长急弯         
// 4137          sai_dao_lei_xing_0=17;         
??judge_locus_0_29:
        MOVS     R5,#+17
// 4138        }
// 4139        /////////////////////////////////////////////////
// 4140      }
// 4141    }
// 4142    return sai_dao_lei_xing_0;  
??judge_locus_0_15:
        MOVS     R0,R5
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        POP      {R1,R4,R5,PC}    ;; return
// 4143 }

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable59:
        DC32     Servo
// 4144 /*************判断赛道类型0END**************/
// 4145 
// 4146 

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 4147 int16 speed_control(void)
// 4148 {
speed_control:
        PUSH     {LR}
        SUB      SP,SP,#+28
// 4149   volatile int16 i=0,E=0,Ec=0;
        MOVS     R0,#+0
        STRH     R0,[SP, #+0]
        MOVS     R0,#+0
        STRH     R0,[SP, #+26]
        MOVS     R0,#+0
        STRH     R0,[SP, #+24]
// 4150   volatile int32 exspeed=0;
        MOVS     R0,#+0
        STR      R0,[SP, #+4]
        B.N      ??speed_control_0
// 4151   volatile struct membership_grade S; 
// 4152   static int16 i_old=0;
// 4153   
// 4154   
// 4155   while(!position[i]&&i<55) i++;   //50
??speed_control_1:
        LDRH     R0,[SP, #+0]
        ADDS     R0,R0,#+1
        STRH     R0,[SP, #+0]
??speed_control_0:
        LDRSH    R0,[SP, #+0]
        LDR.N    R1,??DataTable61_8
        LDR      R0,[R1, R0, LSL #+2]
        CMP      R0,#+0
        BNE.N    ??speed_control_2
        LDRSH    R0,[SP, #+0]
        CMP      R0,#+55
        BLT.N    ??speed_control_1
// 4156   
// 4157       
// 4158   E=54-i;   //i无效点个数    //49    E为有效的个数
??speed_control_2:
        LDRH     R0,[SP, #+0]
        RSBS     R0,R0,#+54
        STRH     R0,[SP, #+26]
// 4159   Ec=i-i_old; //有效点变化率
        LDRH     R0,[SP, #+0]
        LDR.W    R1,??DataTable62_1
        LDRH     R1,[R1, #+0]
        SUBS     R0,R0,R1
        STRH     R0,[SP, #+24]
// 4160   i_old=i;
        LDR.W    R0,??DataTable62_1
        LDRH     R1,[SP, #+0]
        STRH     R1,[R0, #+0]
// 4161   
// 4162   if(i<=5) return speed_top;    //直道全速冲刺  3
        LDRSH    R0,[SP, #+0]
        CMP      R0,#+6
        BGE.N    ??speed_control_3
        LDR.W    R0,??DataTable62_2
        LDRSH    R0,[R0, #+0]
        B.N      ??speed_control_4
// 4163     
// 4164   if(i>5)
??speed_control_3:
        LDRSH    R0,[SP, #+0]
        CMP      R0,#+6
        BLT.N    ??speed_control_5
// 4165   {
// 4166     if(!Ec)//变化率为0的时候
        LDRSH    R0,[SP, #+24]
        CMP      R0,#+0
        BNE.N    ??speed_control_6
// 4167     {
// 4168       //sub_e(E,25,20,50,45);//20，15，45，40通过采集到的点数来算出，speed_s,speed_m,speed_b所占的份额
// 4169   //  sub_e(E,24,0,54,24);       //主要决定十字弯道和直到的过弯速度
// 4170       sub_e(E,35,30,50,45);
        MOVS     R0,#+45
        STR      R0,[SP, #+0]
        MOVS     R3,#+50
        MOVS     R2,#+30
        MOVS     R1,#+35
        LDRH     R0,[SP, #+26]
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        BL       sub_e
// 4171       S.s=sub.s;
        LDR.W    R0,??DataTable62_3
        LDRH     R0,[R0, #+0]
        STRH     R0,[SP, #+8]
// 4172       S.m=sub.m;
        LDR.W    R0,??DataTable62_3
        LDRH     R0,[R0, #+2]
        STRH     R0,[SP, #+10]
// 4173       S.b=sub.b;
        LDR.W    R0,??DataTable62_3
        LDRH     R0,[R0, #+4]
        STRH     R0,[SP, #+12]
// 4174       exspeed=(sub.s*speed_s+sub.m*speed_m+sub.b*speed_b)/100;
        LDR.W    R0,??DataTable62_3
        LDRSH    R0,[R0, #+0]
        LDR.W    R1,??DataTable62_4
        LDRSH    R1,[R1, #+0]
        LDR.W    R2,??DataTable62_3
        LDRSH    R2,[R2, #+2]
        LDR.W    R3,??DataTable62_5
        LDRSH    R3,[R3, #+0]
        MULS     R2,R3,R2
        MLA      R0,R1,R0,R2
        LDR.W    R1,??DataTable62_3
        LDRSH    R1,[R1, #+4]
        LDR.W    R2,??DataTable62_6
        LDRSH    R2,[R2, #+0]
        MLA      R0,R2,R1,R0
        MOVS     R1,#+100
        SDIV     R0,R0,R1
        STR      R0,[SP, #+4]
        B.N      ??speed_control_7
// 4175     }
// 4176     else //变化率不等于0的时候
// 4177     {
// 4178       //exspeed=speed_fuzzy(E,Ec,35,25,50,45,3,1,7,5); 4.8—10：00
// 4179       exspeed=speed_fuzzy(E,Ec,35,30,50,45,3,1,7,5);
??speed_control_6:
        MOVS     R0,#+5
        STR      R0,[SP, #+20]
        MOVS     R0,#+7
        STR      R0,[SP, #+16]
        MOVS     R0,#+1
        STR      R0,[SP, #+12]
        MOVS     R0,#+3
        STR      R0,[SP, #+8]
        MOVS     R0,#+45
        STR      R0,[SP, #+4]
        MOVS     R0,#+50
        STR      R0,[SP, #+0]
        MOVS     R3,#+30
        MOVS     R2,#+35
        LDRSH    R1,[SP, #+24]
        LDRSH    R0,[SP, #+26]
        BL       speed_fuzzy
        STR      R0,[SP, #+4]
// 4180     }
// 4181     return exspeed;
??speed_control_7:
        LDR      R0,[SP, #+4]
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        B.N      ??speed_control_4
// 4182   }
// 4183 }
??speed_control_5:
??speed_control_4:
        ADD      SP,SP,#+28
        POP      {PC}             ;; return

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
??i_old:
        DS8 2
// 4184     
// 4185 
// 4186 

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
// 4187 int16 Motor_control(int16 shift)
// 4188 {
Motor_control:
        PUSH     {LR}
        SUB      SP,SP,#+28
// 4189   volatile int32 speed=0;
        MOVS     R1,#+0
        STR      R1,[SP, #+0]
// 4190   volatile int16 motor_e=0,motor_ec=0;
        MOVS     R1,#+0
        STRH     R1,[SP, #+24]
        MOVS     R1,#+0
        STRH     R1,[SP, #+26]
// 4191   static int16 motor_e_old=0,motor_sum=0;
// 4192    
// 4193   motor_e=shift-pulse; //speed_vari为编码器返回的脉冲数
        LDR.W    R1,??DataTable62_7
        LDRSH    R1,[R1, #+0]
        SUBS     R1,R0,R1
        STRH     R1,[SP, #+24]
// 4194   speed_now=shift;
        LDR.W    R1,??DataTable62_8
        STRH     R0,[R1, #+0]
// 4195   motor_ec=motor_e-motor_e_old;
        LDRH     R0,[SP, #+24]
        LDR.W    R1,??DataTable62_9
        LDRH     R1,[R1, #+0]
        SUBS     R0,R0,R1
        STRH     R0,[SP, #+26]
// 4196   
// 4197   motor_fuzzy(motor_e,motor_ec,16,8,32,24,8,4,16,12);//30  20  50  40  15  10  25  20
        MOVS     R0,#+12
        STR      R0,[SP, #+20]
        MOVS     R0,#+16
        STR      R0,[SP, #+16]
        MOVS     R0,#+4
        STR      R0,[SP, #+12]
        MOVS     R0,#+8
        STR      R0,[SP, #+8]
        MOVS     R0,#+24
        STR      R0,[SP, #+4]
        MOVS     R0,#+32
        STR      R0,[SP, #+0]
        MOVS     R3,#+8
        MOVS     R2,#+16
        LDRSH    R1,[SP, #+26]
        LDRSH    R0,[SP, #+24]
        BL       motor_fuzzy
// 4198   motor_sum=line(motor_e,Motor.I);
        LDR.W    R0,??DataTable62_10
        LDRSH    R1,[R0, #+2]
        LDRSH    R0,[SP, #+24]
        BL       line
        LDR.W    R1,??DataTable62_11
        STRH     R0,[R1, #+0]
// 4199   speed=(long)s_old*motor_p+Motor.P*(long)motor_e+motor_p*(long)motor_sum/2;
        LDR.W    R0,??DataTable62_12
        LDRSH    R0,[R0, #+0]
        MOVS     R1,#+70
        LDR.W    R2,??DataTable62_10
        LDRSH    R2,[R2, #+0]
        LDRSH    R3,[SP, #+24]
        MULS     R2,R3,R2
        MLA      R0,R1,R0,R2
        LDR.W    R1,??DataTable62_11
        LDRSH    R1,[R1, #+0]
        MOVS     R2,#+70
        MULS     R1,R2,R1
        MOVS     R2,#+2
        SDIV     R1,R1,R2
        ADDS     R0,R1,R0
        STR      R0,[SP, #+0]
// 4200   speed=speed/100;
        LDR      R0,[SP, #+0]
        MOVS     R1,#+100
        SDIV     R0,R0,R1
        STR      R0,[SP, #+0]
// 4201   s_old=pulse;
        LDR.W    R0,??DataTable62_12
        LDR.W    R1,??DataTable62_7
        LDRSH    R1,[R1, #+0]
        STRH     R1,[R0, #+0]
// 4202   motor_e_old=motor_e;
        LDR.W    R0,??DataTable62_9
        LDRH     R1,[SP, #+24]
        STRH     R1,[R0, #+0]
// 4203   //if(pulse<50&&speed<0) return 50;
// 4204   return speed;            
        LDR      R0,[SP, #+0]
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        ADD      SP,SP,#+28
        POP      {PC}             ;; return
// 4205 }   

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable61:
        DC32     Servo_Middle

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable61_1:
        DC32     position_now

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable61_2:
        DC32     position_diff

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable61_3:
        DC32     Servo_value

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable61_4:
        DC32     Servo_Right

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable61_5:
        DC32     Servo_Left

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable61_6:
        DC32     Servo_value_old

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable61_7:
        DC32     0x40039010

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable61_8:
        DC32     position

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable61_9:
        DC32     low_0

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable61_10:
        DC32     mid_0

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable61_11:
        DC32     high_0

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable61_12:
        DC32     low_now_0

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable61_13:
        DC32     mid_now_0

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable61_14:
        DC32     high_now_0

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable61_15:
        DC32     Very_straight

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable61_16:
        DC32     `high`

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable61_17:
        DC32     mid

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
??motor_e_old:
        DS8 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
??motor_sum:
        DS8 2
// 4206 
// 4207 
// 4208 /************************主函数****************************/
// 4209 
// 4210 

        SECTION `.text`:CODE:NOROOT(2)
        THUMB
// 4211 void main(void)
// 4212 {  
main:
        PUSH     {R4-R11,LR}
        SUB      SP,SP,#+12
// 4213   uint8 m=0,line_head=0,line_end=0;
        MOVS     R9,#+0
        MOVS     R0,#+0
        STRB     R0,[SP, #+0]
        MOVS     R0,#+0
        STRB     R0,[SP, #+0]
// 4214   uint8 P_stop=0;//停车关速度控制标志位
        MOVS     R0,#+0
        STRB     R0,[SP, #+4]
// 4215   uint8 i,direct=0,LCD_begin=0;
        MOVS     R8,#+0
        MOVS     R0,#+0
        STRB     R0,[SP, #+1]
// 4216   int8 judge=0,set=0;       
        MOVS     R0,#+0
        STRB     R0,[SP, #+0]
        MOVS     R10,#+0
// 4217   int16  exspeed=0,pwm_speed=0;
        MOVS     R4,#+0
        MOVS     R11,#+0
// 4218   int16 start_flag=0,stf=0,stop_flag2=1,stop_flag=0,stop_num=0;
        MOVS     R0,#+0
        STRH     R0,[SP, #+2]
        MOVS     R0,#+0
        STRH     R0,[SP, #+2]
        MOVS     R5,#+1
        MOVS     R6,#+0
        MOVS     R7,#+0
// 4219   
// 4220   DisableInterrupts;
        CPSID i         
// 4221   pllinit180M();
        BL       pllinit180M
// 4222   IO_Init();
        BL       IO_Init
// 4223   LCD_Init();  
        BL       LCD_Init
// 4224   hw_FTM1_init();
        BL       hw_FTM1_init
// 4225   hw_FTM0_init();
        BL       hw_FTM0_init
// 4226   FTM2_QUAD_Init();
        BL       FTM2_QUAD_Init
// 4227   lptmr_pulse_counter(LPTMR_ALT2);
        MOVS     R0,#+2
        BL       lptmr_pulse_counter
// 4228   //gpio_init(PORTA,14,1,1);
// 4229   //gpio_init(PORTA,15,1,1);
// 4230   //gpio_init(PORTA,16,1,1);
// 4231   //gpio_init(PORTA,17,1,1);
// 4232   JIANPAN_ini();
        BL       JIANPAN_ini
// 4233   CH451_ini();
        BL       CH451_ini
// 4234   CH451_WriteCommand(CH451_BCD);
        MOV      R0,#+1408
        BL       CH451_WriteCommand
// 4235   LCD_P6x8Str(0,0,"speed"); 
        LDR.W    R2,??DataTable62_13
        MOVS     R1,#+0
        MOVS     R0,#+0
        BL       LCD_P6x8Str
// 4236   LCD_P6x8Str(0,7,"J_line");//英文字符串显示
        LDR.W    R2,??DataTable62_14
        MOVS     R1,#+7
        MOVS     R0,#+0
        BL       LCD_P6x8Str
// 4237   LCD_P6x8Str_3(50,10,Judge_startline);
        LDR.W    R0,??DataTable62_15
        LDRSH    R2,[R0, #+0]
        MOVS     R1,#+10
        MOVS     R0,#+50
        BL       LCD_P6x8Str_3
// 4238   Car_Speed();
        BL       Car_Speed
// 4239   switch(car_speed)
        LDR.W    R0,??DataTable62_16
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.N    ??main_0
        CMP      R0,#+2
        BEQ.N    ??main_1
        BCC.N    ??main_2
        CMP      R0,#+4
        BEQ.N    ??main_3
        BCC.N    ??main_4
        CMP      R0,#+6
        BEQ.W    ??main_5
        BCC.W    ??main_6
        CMP      R0,#+8
        BEQ.W    ??main_7
        BCC.W    ??main_8
        CMP      R0,#+10
        BEQ.W    ??main_9
        BCC.W    ??main_10
        CMP      R0,#+11
        BEQ.W    ??main_11
        B.N      ??main_12
// 4240   {
// 4241     case 0:speed_Very_straight=170;speed_top=165;speed_b=160;speed_m=155;speed_s=150; speed_ms=145;break;
??main_0:
        LDR.W    R0,??DataTable62_17
        MOVS     R1,#+170
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_2
        MOVS     R1,#+165
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_6
        MOVS     R1,#+160
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_5
        MOVS     R1,#+155
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_4
        MOVS     R1,#+150
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_18
        MOVS     R1,#+145
        STRH     R1,[R0, #+0]
        B.N      ??main_12
// 4242     case 1:speed_Very_straight=180;speed_top=175;speed_b=170;speed_m=165;speed_s=160; speed_ms=155;break;
??main_2:
        LDR.W    R0,??DataTable62_17
        MOVS     R1,#+180
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_2
        MOVS     R1,#+175
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_6
        MOVS     R1,#+170
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_5
        MOVS     R1,#+165
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_4
        MOVS     R1,#+160
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_18
        MOVS     R1,#+155
        STRH     R1,[R0, #+0]
        B.N      ??main_12
// 4243     case 2:speed_Very_straight=200;speed_top=190;speed_b=175;speed_m=170;speed_s=165; speed_ms=160;break;
??main_1:
        LDR.W    R0,??DataTable62_17
        MOVS     R1,#+200
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_2
        MOVS     R1,#+190
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_6
        MOVS     R1,#+175
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_5
        MOVS     R1,#+170
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_4
        MOVS     R1,#+165
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_18
        MOVS     R1,#+160
        STRH     R1,[R0, #+0]
        B.N      ??main_12
// 4244     case 3:speed_Very_straight=210;speed_top=200;speed_b=190;speed_m=180;speed_s=175; speed_ms=175;break;
??main_4:
        LDR.W    R0,??DataTable62_17
        MOVS     R1,#+210
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_2
        MOVS     R1,#+200
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_6
        MOVS     R1,#+190
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_5
        MOVS     R1,#+180
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_4
        MOVS     R1,#+175
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_18
        MOVS     R1,#+175
        STRH     R1,[R0, #+0]
        B.N      ??main_12
// 4245     case 4:speed_Very_straight=220;speed_top=210;speed_b=190;speed_m=180;speed_s=175; speed_ms=170;break;
??main_3:
        LDR.W    R0,??DataTable62_17
        MOVS     R1,#+220
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_2
        MOVS     R1,#+210
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_6
        MOVS     R1,#+190
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_5
        MOVS     R1,#+180
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_4
        MOVS     R1,#+175
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_18
        MOVS     R1,#+170
        STRH     R1,[R0, #+0]
        B.N      ??main_12
// 4246     case 5:speed_Very_straight=230;speed_top=220;speed_b=190;speed_m=180;speed_s=180; speed_ms=175;break;
??main_6:
        LDR.W    R0,??DataTable62_17
        MOVS     R1,#+230
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_2
        MOVS     R1,#+220
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_6
        MOVS     R1,#+190
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_5
        MOVS     R1,#+180
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_4
        MOVS     R1,#+180
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_18
        MOVS     R1,#+175
        STRH     R1,[R0, #+0]
        B.N      ??main_12
// 4247     case 6:speed_Very_straight=240;speed_top=230;speed_b=195;speed_m=180;speed_s=175; speed_ms=175;break; 
??main_5:
        LDR.W    R0,??DataTable62_17
        MOVS     R1,#+240
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_2
        MOVS     R1,#+230
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_6
        MOVS     R1,#+195
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_5
        MOVS     R1,#+180
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_4
        MOVS     R1,#+175
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_18
        MOVS     R1,#+175
        STRH     R1,[R0, #+0]
        B.N      ??main_12
// 4248     case 7:speed_Very_straight=250;speed_top=240;speed_b=200;speed_m=185;speed_s=180; speed_ms=180;break;
??main_8:
        LDR.W    R0,??DataTable62_17
        MOVS     R1,#+250
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_2
        MOVS     R1,#+240
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_6
        MOVS     R1,#+200
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_5
        MOVS     R1,#+185
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_4
        MOVS     R1,#+180
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_18
        MOVS     R1,#+180
        STRH     R1,[R0, #+0]
        B.N      ??main_12
// 4249     case 8:speed_Very_straight=260;speed_top=250;speed_b=210;speed_m=195;speed_s=190; speed_ms=190;break;
??main_7:
        LDR.W    R0,??DataTable62_17
        MOV      R1,#+260
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_2
        MOVS     R1,#+250
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_6
        MOVS     R1,#+210
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_5
        MOVS     R1,#+195
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_4
        MOVS     R1,#+190
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_18
        MOVS     R1,#+190
        STRH     R1,[R0, #+0]
        B.N      ??main_12
// 4250     case 9:speed_Very_straight=270;speed_top=260;speed_b=220;speed_m=200;speed_s=200; speed_ms=190;break;
??main_10:
        LDR.W    R0,??DataTable62_17
        MOV      R1,#+270
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_2
        MOV      R1,#+260
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_6
        MOVS     R1,#+220
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_5
        MOVS     R1,#+200
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_4
        MOVS     R1,#+200
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_18
        MOVS     R1,#+190
        STRH     R1,[R0, #+0]
        B.N      ??main_12
// 4251      case 10:speed_Very_straight=290;speed_top=270;speed_b=230;speed_m=220;speed_s=210; speed_ms=190;break;
??main_9:
        LDR.W    R0,??DataTable62_17
        MOV      R1,#+290
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_2
        MOV      R1,#+270
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_6
        MOVS     R1,#+230
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_5
        MOVS     R1,#+220
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_4
        MOVS     R1,#+210
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_18
        MOVS     R1,#+190
        STRH     R1,[R0, #+0]
        B.N      ??main_12
// 4252       case 11:speed_Very_straight=290;speed_top=270;speed_b=240;speed_m=230;speed_s=220; speed_ms=190;break;
??main_11:
        LDR.W    R0,??DataTable62_17
        MOV      R1,#+290
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_2
        MOV      R1,#+270
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_6
        MOVS     R1,#+240
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_5
        MOVS     R1,#+230
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_4
        MOVS     R1,#+220
        STRH     R1,[R0, #+0]
        LDR.W    R0,??DataTable62_18
        MOVS     R1,#+190
        STRH     R1,[R0, #+0]
        B.N      ??main_12
// 4253   }
// 4254   while(KeyValue!=50)
// 4255   {
// 4256     KeyValue=50;
??main_13:
        LDR.W    R0,??DataTable62_19
        MOVS     R1,#+50
        STRB     R1,[R0, #+0]
// 4257     CH451_GetKeyValue();
        BL       CH451_GetKeyValue
// 4258   }
??main_12:
        LDR.W    R0,??DataTable62_19
        LDRB     R0,[R0, #+0]
        CMP      R0,#+50
        BNE.N    ??main_13
// 4259   //1直接发车，不调节PD，，，2调节PD后发车/
// 4260   while(!direct)
??main_14:
        UXTB     R8,R8            ;; ZeroExt  R8,R8,#+24,#+24
        CMP      R8,#+0
        BNE.N    ??main_15
// 4261   {
// 4262     LCD_P6x8Str(0,1,"TJorFC");     //英文字符串显示      
        LDR.W    R2,??DataTable62_20
        MOVS     R1,#+1
        MOVS     R0,#+0
        BL       LCD_P6x8Str
// 4263     CH451_GetKeyValue();
        BL       CH451_GetKeyValue
// 4264     switch(KeyValue)
        LDR.W    R0,??DataTable62_19
        LDRB     R0,[R0, #+0]
        CMP      R0,#+1
        BEQ.N    ??main_16
        CMP      R0,#+2
        BEQ.N    ??main_17
        CMP      R0,#+5
        BEQ.N    ??main_18
        B.N      ??main_14
// 4265     {
// 4266     case 1:
// 4267       LCD_P6x8Str(40,1,"FC");     //英文字符串显示      
??main_16:
        ADR.N    R2,??main_19     ;; 0x46, 0x43, 0x00, 0x00
        MOVS     R1,#+1
        MOVS     R0,#+40
        BL       LCD_P6x8Str
        B.N      ??main_20
// 4268       while(KeyValue==1)
// 4269       {
// 4270         KeyValue=50;
??main_21:
        LDR.W    R0,??DataTable62_19
        MOVS     R1,#+50
        STRB     R1,[R0, #+0]
// 4271         CH451_GetKeyValue();
        BL       CH451_GetKeyValue
// 4272       }
??main_20:
        LDR.W    R0,??DataTable62_19
        LDRB     R0,[R0, #+0]
        CMP      R0,#+1
        BEQ.N    ??main_21
// 4273       direct=1;
        MOVS     R8,#+1
// 4274       break;
        B.N      ??main_14
// 4275     case 2:
// 4276       LCD_P6x8Str(40,1,"TJ");     //英文字符串显示      
??main_17:
        ADR.N    R2,??main_19+0x4  ;; 0x54, 0x4A, 0x00, 0x00
        MOVS     R1,#+1
        MOVS     R0,#+40
        BL       LCD_P6x8Str
        B.N      ??main_22
// 4277       while(KeyValue==2)
// 4278       {
// 4279         KeyValue=50;
??main_23:
        LDR.W    R0,??DataTable62_19
        MOVS     R1,#+50
        STRB     R1,[R0, #+0]
// 4280         CH451_GetKeyValue();
        BL       CH451_GetKeyValue
// 4281       }
??main_22:
        LDR.W    R0,??DataTable62_19
        LDRB     R0,[R0, #+0]
        CMP      R0,#+2
        BEQ.N    ??main_23
// 4282       LCD_P6x8Str(0,2,"sai_dao_number");     //英文字符串显示  
        LDR.W    R2,??DataTable62_21
        MOVS     R1,#+2
        MOVS     R0,#+0
        BL       LCD_P6x8Str
// 4283       Duo_Ji_PD();
        BL       Duo_Ji_PD
// 4284       LCD_P6x8Str(58,1,"FC");     //英文字符串显示  
        ADR.N    R2,??main_19     ;; 0x46, 0x43, 0x00, 0x00
        MOVS     R1,#+1
        MOVS     R0,#+58
        BL       LCD_P6x8Str
// 4285       direct=1;
        MOVS     R8,#+1
// 4286       break;
        B.N      ??main_14
// 4287     case 5:
// 4288       LCD_P6x8Str(76,0,"LCDgo_on");     //英文字符串显示      
??main_18:
        LDR.W    R2,??DataTable62_22
        MOVS     R1,#+0
        MOVS     R0,#+76
        BL       LCD_P6x8Str
        B.N      ??main_24
// 4289       while(KeyValue==5)
// 4290       {
// 4291         KeyValue=50;
??main_25:
        LDR.W    R0,??DataTable62_19
        MOVS     R1,#+50
        STRB     R1,[R0, #+0]
// 4292         CH451_GetKeyValue();
        BL       CH451_GetKeyValue
// 4293       }
??main_24:
        LDR.W    R0,??DataTable62_19
        LDRB     R0,[R0, #+0]
        CMP      R0,#+5
        BEQ.N    ??main_25
// 4294       LCD_begin=1;
        MOVS     R0,#+1
        STRB     R0,[SP, #+1]
// 4295       break;       
        B.N      ??main_14
// 4296     }
// 4297   }
// 4298   /****************1直接发车，不调节PD，，，2调节PD后发车end*******************/           
// 4299   enable_irq (87);//使能A口中断 ，A24场中断
??main_15:
        MOVS     R0,#+87
        BL       enable_irq
// 4300   UART0_Init();
        BL       UART0_Init
// 4301   //uart_init(UART5,45,115200);
// 4302   DMA0_Init();
        BL       DMA0_Init
// 4303   //hw_pit_init(1,16000);         //pit1中断初始化
// 4304   //enable_irq(89);             //开c9口红外遥控停车中断
// 4305   //enable_pit_interrupt(1);    //使能PIT1中断，，，用于红外计数
// 4306   delays(3);   //延时发车
        MOVS     R0,#+3
        BL       delays
// 4307   lptmr_pulse_counter(LPTMR_ALT2);
        MOVS     R0,#+2
        BL       lptmr_pulse_counter
// 4308   LCD_CLS();
        BL       LCD_CLS
// 4309   hw_pit_init(PIT2,4500000);//100ms
        LDR.W    R1,??DataTable62_23  ;; 0x44aa20
        MOVS     R0,#+2
        BL       hw_pit_init
// 4310   enable_pit_interrupt(PIT2);
        MOVS     R0,#+2
        BL       enable_pit_interrupt
// 4311   EnableInterrupts;
        CPSIE i         
        B.N      ??main_26
// 4312   for(;;) 
// 4313   {	  
// 4314     if(finish==1)
// 4315     {   
// 4316       tu_flag=find_position();
// 4317       if(stop_flag==1)
// 4318       {
// 4319         stop_num++;
// 4320       }
// 4321       m=judge_locus_0();
// 4322       sai_dao_lei_xing=m;
// 4323        if(P_timer>Judge_startline&&dijihang<5&&high_now_0>60&&high_now_0<100&&mid_now_0>60&&mid_now_0<100&&low_now_0>60&&low_now_0<100)
// 4324     {
// 4325      if(stop_flag2)
// 4326      {
// 4327     stop_flag=Judge_startline2();
// 4328     
// 4329      }
// 4330      if(stop_flag==1)
// 4331      {
// 4332        stop_flag2=0;
// 4333      }
// 4334     } 
// 4335 if(stop_flag==1)
// 4336 {
// 4337   if(stop_num>12)
// 4338   {
// 4339     if(pulse>150)
// 4340   {
// 4341       FTM0_C0V=0;
// 4342       FTM0_C3V=350;
// 4343       FTM0_C4V=350;
// 4344       FTM0_C6V=0;
// 4345   }
// 4346   if(pulse>100)
// 4347   {
// 4348       FTM0_C0V=0;
// 4349       FTM0_C3V=250;
// 4350       FTM0_C4V=250;
// 4351       FTM0_C6V=0;
// 4352   }
// 4353   else if(pulse>50)
// 4354   {
// 4355       FTM0_C0V=0;
// 4356       FTM0_C3V=200;
// 4357       FTM0_C4V=200;
// 4358       FTM0_C6V=0;
// 4359   }
// 4360   else if(pulse>30)
// 4361   {
// 4362       FTM0_C0V=0;
// 4363       FTM0_C3V=10;
// 4364       FTM0_C1V=0;
// 4365       FTM0_C5V=0;
// 4366       FTM0_C4V=10;
// 4367       FTM0_C6V=0;
// 4368   }
// 4369 else
// 4370 {     
// 4371      FTM0_C0V=0;
// 4372       FTM0_C3V=0;
// 4373       FTM0_C4V=0;
// 4374       FTM0_C6V=0;
// 4375       FTM0_C1V=0;
// 4376       FTM0_C5V=0;
// 4377 
// 4378 }
// 4379   }
// 4380   else
// 4381   {
// 4382   FTM0_C0V=375;
// 4383       FTM0_C3V=0;
// 4384       FTM0_C4V=0;
// 4385       FTM0_C6V=375;
// 4386   }
// 4387       P_stop=1;
// 4388       stf=1;
// 4389 
// 4390 }
// 4391         
// 4392           
// 4393       
// 4394       if(tu_flag==1)  
// 4395       {
// 4396         set=choose_PD();
// 4397         tu_flag=0;
// 4398       }
// 4399       else
// 4400       {
// 4401         if(Servo_value>(Servo_Middle+200))
// 4402         {
// 4403         Servo_value=limit(Servo_value+250,Servo_Left,Servo_Right);
// 4404         FTM1_C0V=Servo_value;
// 4405          // Servo_value=9320;
// 4406         }
// 4407         if(Servo_value<(Servo_Middle-200))
// 4408         {
// 4409         Servo_value=limit(Servo_value-200,Servo_Left,Servo_Right);
// 4410         FTM1_C0V=Servo_value;
// 4411         //Servo_value=7035;
// 4412         }
// 4413       }
// 4414       if(P_stop==0)
// 4415       {
// 4416         if(P_timer<10) 
// 4417           pwm_speed=375;//发车前1.3s给满占空比,使其快速发车
// 4418         else
// 4419         {
// 4420           if(Very_straight==0)           
// 4421             exspeed=speed_control();
// 4422           if(Very_straight==1)
// 4423           {
// 4424             Very_straight=0;
// 4425             exspeed=speed_Very_straight;
// 4426           }
// 4427           pwm_speed=Motor_control(exspeed);
// 4428         }
// 4429         set_speed(pwm_speed);
// 4430       }
// 4431       
// 4432     hw_pit_init(PIT0,745000);//16ms  使用定时器，这样才是在固定的时间里面得到的脉冲数
// 4433     enable_pit_interrupt(PIT0);
// 4434     /********************LCD数据显示模块*********************/
// 4435     CH451_GetKeyValue();
// 4436     if(KeyValue==10||LCD_begin==1)           //LCD_begin==1
// 4437     {
// 4438       if(LCD_P1==250)LCD_P1=0;
// 4439                     //秒加1
// 4440                   
// 4441       if(++LCD_P1%10)
// 4442       {
// 4443         LCD_CLS();                     
// 4444         LCD_P6x8Str(1,0,"low_0");     //英文字符串显示
// 4445         LCD_P6x8Str_3(45,0,low_now_0);
// 4446         if(low_now)LCD_P6x8Str_1(70,0,low_0);    //如果存在的话就显示，不存在的话就不显示
// 4447         //LCD_P6x8Str(10,0,"Servo.P");
// 4448         LCD_P6x8Str_3(70,0,Servo.P);//显示赛道类型.
// 4449          LCD_P6x8Str_3(90,0,Servo.D);
// 4450         LCD_P6x8Str(1,1,"mid_0");     //英文字符串显示
// 4451         LCD_P6x8Str_3(45,1,mid_now_0);
// 4452         if(mid_now)LCD_P6x8Str_1(70,1,mid_0);
// 4453        LCD_P6x8Str_3(70,1,pulse);
// 4454        
// 4455         LCD_P6x8Str_3(90,1,pulse2);
// 4456         LCD_P6x8Str(1,2,"high_0");     //英文字符串显示
// 4457         LCD_P6x8Str_3(45,2,high_now_0);
// 4458         if(high_now)LCD_P6x8Str_1(70,2,high_0);
// 4459         LCD_P6x8Str(1,3,"position_now");     //英文字符串显示
// 4460         LCD_P6x8Str_3(75,3,position_now);
// 4461         LCD_P6x8Str(1,4,"position_diff");     //英文字符串显示
// 4462         LCD_P6x8Str_FuHao_3(88,4,position_diff);
// 4463         LCD_P6x8Str(1,5,"position_now_low");     //英文字符串显示
// 4464         LCD_P6x8Str_FuHao_3(100,5,position_now_low);
// 4465         LCD_P6x8Str(1,6,"Servo_value"); 
// 4466 
// 4467         LCD_P6x8Str_5(88,6,Servo_value);
// 4468         LCD_P6x8Str(1,7,"dijihang"); 
// 4469         LCD_P6x8Str_5(88,7,dijihang);
// 4470       }
// 4471     }
// 4472     if(KeyValue==6)
// 4473     {
// 4474       if(LCD_P3==250)LCD_P3=0;
// 4475       if(++LCD_P3%10)
// 4476       {
// 4477         LCD_CLS();       
// 4478         for(i=0;i<IMG_ROWS-1;i++)
// 4479         {
// 4480            LCD_PutPixel(position[i],i);
// 4481         }
// 4482        }
// 4483      }             
// 4484       /***************对一些变量进行清零****************/
// 4485       position_now_low=0;
// 4486       position_now=0;
// 4487       position_diff=0;
// 4488       dijihang=0;
// 4489       for(i=0;i<IMG_ROWS;i++)
// 4490       {  
// 4491         position[i]=0;
??main_27:
        UXTB     R8,R8            ;; ZeroExt  R8,R8,#+24,#+24
        LDR.W    R0,??DataTable62_24
        MOVS     R1,#+0
        STR      R1,[R0, R8, LSL #+2]
// 4492       }      
        ADDS     R8,R8,#+1
??main_28:
        UXTB     R8,R8            ;; ZeroExt  R8,R8,#+24,#+24
        CMP      R8,#+55
        BCC.N    ??main_27
// 4493       ramp_flag=0;
        LDR.W    R0,??DataTable62_25
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
// 4494       finish=0;
        LDR.W    R0,??DataTable62_26
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
??main_26:
        LDR.W    R0,??DataTable62_26
        LDRB     R0,[R0, #+0]
        CMP      R0,#+1
        BNE.N    ??main_26
        BL       find_position
        LDR.W    R1,??DataTable62_27
        STR      R0,[R1, #+0]
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        CMP      R6,#+1
        BNE.N    ??main_29
        ADDS     R7,R7,#+1
??main_29:
        BL       judge_locus_0
        MOV      R9,R0
        LDR.W    R0,??DataTable62_28
        STRB     R9,[R0, #+0]
        LDR.W    R0,??DataTable62_15
        LDRSH    R0,[R0, #+0]
        LDR.W    R1,??DataTable62_29
        LDR      R1,[R1, #+0]
        CMP      R0,R1
        BGE.N    ??main_30
        LDR.W    R0,??DataTable62_30
        LDRSH    R0,[R0, #+0]
        CMP      R0,#+5
        BGE.N    ??main_30
        LDR.W    R0,??DataTable62_31
        LDRSH    R0,[R0, #+0]
        CMP      R0,#+61
        BLT.N    ??main_30
        LDR.W    R0,??DataTable62_31
        LDRSH    R0,[R0, #+0]
        CMP      R0,#+100
        BGE.N    ??main_30
        LDR.W    R0,??DataTable62_32
        LDRSH    R0,[R0, #+0]
        CMP      R0,#+61
        BLT.N    ??main_30
        LDR.W    R0,??DataTable62_32
        LDRSH    R0,[R0, #+0]
        CMP      R0,#+100
        BGE.N    ??main_30
        LDR.W    R0,??DataTable62_33
        LDRSH    R0,[R0, #+0]
        CMP      R0,#+61
        BLT.N    ??main_30
        LDR.W    R0,??DataTable62_33
        LDRSH    R0,[R0, #+0]
        CMP      R0,#+100
        BGE.N    ??main_30
        SXTH     R5,R5            ;; SignExt  R5,R5,#+16,#+16
        CMP      R5,#+0
        BEQ.N    ??main_31
        BL       Judge_startline2
        MOVS     R6,R0
??main_31:
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        CMP      R6,#+1
        BNE.N    ??main_30
        MOVS     R5,#+0
??main_30:
        SXTH     R6,R6            ;; SignExt  R6,R6,#+16,#+16
        CMP      R6,#+1
        BNE.W    ??main_32
        SXTH     R7,R7            ;; SignExt  R7,R7,#+16,#+16
        CMP      R7,#+13
        BLT.N    ??main_33
        LDR.W    R0,??DataTable62_7
        LDRH     R0,[R0, #+0]
        CMP      R0,#+151
        BCC.N    ??main_34
        LDR.W    R0,??DataTable62_34  ;; 0x40038010
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
        LDR.W    R0,??DataTable62_35  ;; 0x40038028
        MOV      R1,#+350
        STR      R1,[R0, #+0]
        LDR.W    R0,??DataTable62_36  ;; 0x40038030
        MOV      R1,#+350
        STR      R1,[R0, #+0]
        LDR.W    R0,??DataTable62_37  ;; 0x40038040
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
??main_34:
        LDR.W    R0,??DataTable62_7
        LDRH     R0,[R0, #+0]
        CMP      R0,#+101
        BCC.N    ??main_35
        LDR.W    R0,??DataTable62_34  ;; 0x40038010
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
        LDR.W    R0,??DataTable62_35  ;; 0x40038028
        MOVS     R1,#+250
        STR      R1,[R0, #+0]
        LDR.W    R0,??DataTable62_36  ;; 0x40038030
        MOVS     R1,#+250
        STR      R1,[R0, #+0]
        LDR.W    R0,??DataTable62_37  ;; 0x40038040
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
        B.N      ??main_36
??main_35:
        LDR.W    R0,??DataTable62_7
        LDRH     R0,[R0, #+0]
        CMP      R0,#+51
        BCC.N    ??main_37
        LDR.W    R0,??DataTable62_34  ;; 0x40038010
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
        LDR.W    R0,??DataTable62_35  ;; 0x40038028
        MOVS     R1,#+200
        STR      R1,[R0, #+0]
        LDR.W    R0,??DataTable62_36  ;; 0x40038030
        MOVS     R1,#+200
        STR      R1,[R0, #+0]
        LDR.W    R0,??DataTable62_37  ;; 0x40038040
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
        B.N      ??main_36
??main_37:
        LDR.W    R0,??DataTable62_7
        LDRH     R0,[R0, #+0]
        CMP      R0,#+31
        BCC.N    ??main_38
        LDR.W    R0,??DataTable62_34  ;; 0x40038010
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
        LDR.W    R0,??DataTable62_35  ;; 0x40038028
        MOVS     R1,#+10
        STR      R1,[R0, #+0]
        LDR.W    R0,??DataTable62_38  ;; 0x40038018
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
        LDR.W    R0,??DataTable62_39  ;; 0x40038038
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
        LDR.W    R0,??DataTable62_36  ;; 0x40038030
        MOVS     R1,#+10
        STR      R1,[R0, #+0]
        LDR.W    R0,??DataTable62_37  ;; 0x40038040
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
        B.N      ??main_36
??main_38:
        LDR.W    R0,??DataTable62_34  ;; 0x40038010
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
        LDR.W    R0,??DataTable62_35  ;; 0x40038028
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
        LDR.W    R0,??DataTable62_36  ;; 0x40038030
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
        LDR.W    R0,??DataTable62_37  ;; 0x40038040
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
        LDR.W    R0,??DataTable62_38  ;; 0x40038018
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
        LDR.W    R0,??DataTable62_39  ;; 0x40038038
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
        B.N      ??main_36
??main_33:
        LDR.W    R0,??DataTable62_34  ;; 0x40038010
        MOVW     R1,#+375
        STR      R1,[R0, #+0]
        LDR.W    R0,??DataTable62_35  ;; 0x40038028
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
        LDR.W    R0,??DataTable62_36  ;; 0x40038030
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
        LDR.W    R0,??DataTable62_37  ;; 0x40038040
        MOVW     R1,#+375
        STR      R1,[R0, #+0]
??main_36:
        MOVS     R0,#+1
        STRB     R0,[SP, #+4]
        MOVS     R0,#+1
        STRH     R0,[SP, #+2]
??main_32:
        LDR.W    R0,??DataTable62_27
        LDR      R0,[R0, #+0]
        CMP      R0,#+1
        BNE.N    ??main_39
        BL       choose_PD
        MOV      R10,R0
        LDR.W    R0,??DataTable62_27
        MOVS     R1,#+0
        STR      R1,[R0, #+0]
        B.N      ??main_40
        Nop      
        DATA
??main_19:
        DC8      0x46, 0x43, 0x00, 0x00
        DC8      0x54, 0x4A, 0x00, 0x00
        THUMB
??main_39:
        LDR.W    R0,??DataTable62_40
        LDRSH    R0,[R0, #+0]
        ADDS     R0,R0,#+200
        LDR.W    R1,??DataTable62_41
        LDRSH    R1,[R1, #+0]
        CMP      R0,R1
        BGE.N    ??main_41
        LDR.W    R0,??DataTable62_42
        LDRSH    R2,[R0, #+0]
        LDR.W    R0,??DataTable62_43
        LDRSH    R1,[R0, #+0]
        LDR.N    R0,??DataTable62_41
        LDRH     R0,[R0, #+0]
        ADDS     R0,R0,#+250
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       limit
        LDR.N    R1,??DataTable62_41
        STRH     R0,[R1, #+0]
        LDR.N    R0,??DataTable62_44  ;; 0x40039010
        LDR.N    R1,??DataTable62_41
        LDRSH    R1,[R1, #+0]
        STR      R1,[R0, #+0]
??main_41:
        LDR.N    R0,??DataTable62_41
        LDRSH    R0,[R0, #+0]
        LDR.N    R1,??DataTable62_40
        LDRSH    R1,[R1, #+0]
        SUBS     R1,R1,#+200
        CMP      R0,R1
        BGE.N    ??main_40
        LDR.N    R0,??DataTable62_42
        LDRSH    R2,[R0, #+0]
        LDR.N    R0,??DataTable62_43
        LDRSH    R1,[R0, #+0]
        LDR.N    R0,??DataTable62_41
        LDRH     R0,[R0, #+0]
        SUBS     R0,R0,#+200
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       limit
        LDR.N    R1,??DataTable62_41
        STRH     R0,[R1, #+0]
        LDR.N    R0,??DataTable62_44  ;; 0x40039010
        LDR.N    R1,??DataTable62_41
        LDRSH    R1,[R1, #+0]
        STR      R1,[R0, #+0]
??main_40:
        LDRB     R0,[SP, #+4]
        CMP      R0,#+0
        BNE.N    ??main_42
        LDR.N    R0,??DataTable62_29
        LDR      R0,[R0, #+0]
        CMP      R0,#+10
        BGE.N    ??main_43
        MOVW     R11,#+375
        B.N      ??main_44
??main_43:
        LDR.N    R0,??DataTable62_45
        LDRB     R0,[R0, #+0]
        CMP      R0,#+0
        BNE.N    ??main_45
        BL       speed_control
        MOVS     R4,R0
??main_45:
        LDR.N    R0,??DataTable62_45
        LDRB     R0,[R0, #+0]
        CMP      R0,#+1
        BNE.N    ??main_46
        LDR.N    R0,??DataTable62_45
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
        LDR.N    R0,??DataTable62_17
        LDRSH    R4,[R0, #+0]
??main_46:
        MOVS     R0,R4
        SXTH     R0,R0            ;; SignExt  R0,R0,#+16,#+16
        BL       Motor_control
        MOV      R11,R0
??main_44:
        SXTH     R11,R11          ;; SignExt  R11,R11,#+16,#+16
        MOV      R0,R11
        BL       set_speed
??main_42:
        LDR.N    R1,??DataTable62_46  ;; 0xb5e28
        MOVS     R0,#+0
        BL       hw_pit_init
        MOVS     R0,#+0
        BL       enable_pit_interrupt
        BL       CH451_GetKeyValue
        LDR.N    R0,??DataTable62_19
        LDRB     R0,[R0, #+0]
        CMP      R0,#+10
        BEQ.N    ??main_47
        LDRB     R0,[SP, #+1]
        CMP      R0,#+1
        BNE.W    ??main_48
??main_47:
        LDR.N    R0,??DataTable62_47
        LDRB     R0,[R0, #+0]
        CMP      R0,#+250
        BNE.N    ??main_49
        LDR.N    R0,??DataTable62_47
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
??main_49:
        LDR.N    R0,??DataTable62_47
        LDRB     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.N    R1,??DataTable62_47
        STRB     R0,[R1, #+0]
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R1,#+10
        SDIV     R2,R0,R1
        MLS      R0,R1,R2,R0
        CMP      R0,#+0
        BEQ.W    ??main_48
        BL       LCD_CLS
        LDR.N    R2,??DataTable62_48
        MOVS     R1,#+0
        MOVS     R0,#+1
        BL       LCD_P6x8Str
        LDR.N    R0,??DataTable62_33
        LDRSH    R2,[R0, #+0]
        MOVS     R1,#+0
        MOVS     R0,#+45
        BL       LCD_P6x8Str_3
        LDR.N    R0,??DataTable62_49
        LDRSH    R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.N    ??main_50
        LDR.N    R0,??DataTable62_50
        LDRH     R2,[R0, #+0]
        SXTB     R2,R2            ;; SignExt  R2,R2,#+24,#+24
        MOVS     R1,#+0
        MOVS     R0,#+70
        BL       LCD_P6x8Str_1
??main_50:
        LDR.N    R0,??DataTable62_51
        LDRSH    R2,[R0, #+0]
        MOVS     R1,#+0
        MOVS     R0,#+70
        BL       LCD_P6x8Str_3
        LDR.N    R0,??DataTable62_51
        LDRSH    R2,[R0, #+4]
        MOVS     R1,#+0
        MOVS     R0,#+90
        BL       LCD_P6x8Str_3
        LDR.N    R2,??DataTable62_52
        MOVS     R1,#+1
        MOVS     R0,#+1
        BL       LCD_P6x8Str
        LDR.N    R0,??DataTable62_32
        LDRSH    R2,[R0, #+0]
        MOVS     R1,#+1
        MOVS     R0,#+45
        BL       LCD_P6x8Str_3
        LDR.N    R0,??DataTable62_53
        LDRSH    R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.N    ??main_51
        LDR.N    R0,??DataTable62_54
        LDRH     R2,[R0, #+0]
        SXTB     R2,R2            ;; SignExt  R2,R2,#+24,#+24
        MOVS     R1,#+1
        MOVS     R0,#+70
        BL       LCD_P6x8Str_1
??main_51:
        LDR.N    R0,??DataTable62_7
        LDRSH    R2,[R0, #+0]
        SXTH     R2,R2            ;; SignExt  R2,R2,#+16,#+16
        MOVS     R1,#+1
        MOVS     R0,#+70
        BL       LCD_P6x8Str_3
        LDR.N    R0,??DataTable62_55
        LDRSH    R2,[R0, #+0]
        SXTH     R2,R2            ;; SignExt  R2,R2,#+16,#+16
        MOVS     R1,#+1
        MOVS     R0,#+90
        BL       LCD_P6x8Str_3
        LDR.N    R2,??DataTable62_56
        MOVS     R1,#+2
        MOVS     R0,#+1
        BL       LCD_P6x8Str
        LDR.N    R0,??DataTable62_31
        LDRSH    R2,[R0, #+0]
        MOVS     R1,#+2
        MOVS     R0,#+45
        BL       LCD_P6x8Str_3
        LDR.N    R0,??DataTable62_57
        LDRSH    R0,[R0, #+0]
        CMP      R0,#+0
        BEQ.N    ??main_52
        LDR.N    R0,??DataTable62_58
        LDRH     R2,[R0, #+0]
        SXTB     R2,R2            ;; SignExt  R2,R2,#+24,#+24
        MOVS     R1,#+2
        MOVS     R0,#+70
        BL       LCD_P6x8Str_1
??main_52:
        LDR.N    R2,??DataTable62_59
        MOVS     R1,#+3
        MOVS     R0,#+1
        BL       LCD_P6x8Str
        LDR.N    R0,??DataTable62_60
        LDRSH    R2,[R0, #+0]
        MOVS     R1,#+3
        MOVS     R0,#+75
        BL       LCD_P6x8Str_3
        LDR.N    R2,??DataTable62_61
        MOVS     R1,#+4
        MOVS     R0,#+1
        BL       LCD_P6x8Str
        LDR.N    R0,??DataTable62_62
        LDRH     R2,[R0, #+0]
        SXTB     R2,R2            ;; SignExt  R2,R2,#+24,#+24
        MOVS     R1,#+4
        MOVS     R0,#+88
        BL       LCD_P6x8Str_FuHao_3
        LDR.N    R2,??DataTable62_63
        MOVS     R1,#+5
        MOVS     R0,#+1
        BL       LCD_P6x8Str
        LDR.N    R0,??DataTable62_64
        LDRH     R2,[R0, #+0]
        SXTB     R2,R2            ;; SignExt  R2,R2,#+24,#+24
        MOVS     R1,#+5
        MOVS     R0,#+100
        BL       LCD_P6x8Str_FuHao_3
        LDR.N    R2,??DataTable62_65
        MOVS     R1,#+6
        MOVS     R0,#+1
        BL       LCD_P6x8Str
        LDR.N    R0,??DataTable62_41
        LDRSH    R2,[R0, #+0]
        MOVS     R1,#+6
        MOVS     R0,#+88
        BL       LCD_P6x8Str_5
        LDR.N    R2,??DataTable62_66
        MOVS     R1,#+7
        MOVS     R0,#+1
        BL       LCD_P6x8Str
        LDR.N    R0,??DataTable62_30
        LDRSH    R2,[R0, #+0]
        MOVS     R1,#+7
        MOVS     R0,#+88
        BL       LCD_P6x8Str_5
??main_48:
        LDR.N    R0,??DataTable62_19
        LDRB     R0,[R0, #+0]
        CMP      R0,#+6
        BNE.N    ??main_53
        LDR.N    R0,??DataTable62_67
        LDRB     R0,[R0, #+0]
        CMP      R0,#+250
        BNE.N    ??main_54
        LDR.N    R0,??DataTable62_67
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
??main_54:
        LDR.N    R0,??DataTable62_67
        LDRB     R0,[R0, #+0]
        ADDS     R0,R0,#+1
        LDR.N    R1,??DataTable62_67
        STRB     R0,[R1, #+0]
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        MOVS     R1,#+10
        SDIV     R2,R0,R1
        MLS      R0,R1,R2,R0
        CMP      R0,#+0
        BEQ.N    ??main_53
        BL       LCD_CLS
        MOVS     R8,#+0
        B.N      ??main_55
??main_56:
        MOV      R1,R8
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        UXTB     R8,R8            ;; ZeroExt  R8,R8,#+24,#+24
        LDR.N    R0,??DataTable62_24
        LDR      R0,[R0, R8, LSL #+2]
        UXTB     R0,R0            ;; ZeroExt  R0,R0,#+24,#+24
        BL       LCD_PutPixel
        ADDS     R8,R8,#+1
??main_55:
        UXTB     R8,R8            ;; ZeroExt  R8,R8,#+24,#+24
        CMP      R8,#+54
        BCC.N    ??main_56
??main_53:
        LDR.N    R0,??DataTable62_64
        MOVS     R1,#+0
        STRH     R1,[R0, #+0]
        LDR.N    R0,??DataTable62_60
        MOVS     R1,#+0
        STRH     R1,[R0, #+0]
        LDR.N    R0,??DataTable62_62
        MOVS     R1,#+0
        STRH     R1,[R0, #+0]
        LDR.N    R0,??DataTable62_30
        MOVS     R1,#+0
        STRH     R1,[R0, #+0]
        MOVS     R8,#+0
        B.N      ??main_28
// 4495       /***************对一些变量进行清零END****************/
// 4496       /********************LCD数据显示模块 end*********************/
// 4497     }
// 4498   }	 
// 4499 }

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62:
        DC32     `low`

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_1:
        DC32     ??i_old

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_2:
        DC32     speed_top

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_3:
        DC32     `sub`

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_4:
        DC32     speed_s

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_5:
        DC32     speed_m

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_6:
        DC32     speed_b

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_7:
        DC32     pulse

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_8:
        DC32     speed_now

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_9:
        DC32     ??motor_e_old

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_10:
        DC32     Motor

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_11:
        DC32     ??motor_sum

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_12:
        DC32     s_old

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_13:
        DC32     `?<Constant "speed">`

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_14:
        DC32     `?<Constant "J_line">`

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_15:
        DC32     Judge_startline

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_16:
        DC32     car_speed

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_17:
        DC32     speed_Very_straight

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_18:
        DC32     speed_ms

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_19:
        DC32     KeyValue

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_20:
        DC32     `?<Constant "TJorFC">`

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_21:
        DC32     `?<Constant "sai_dao_number">`

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_22:
        DC32     `?<Constant "LCDgo_on">`

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_23:
        DC32     0x44aa20

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_24:
        DC32     position

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_25:
        DC32     ramp_flag

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_26:
        DC32     finish

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_27:
        DC32     tu_flag

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_28:
        DC32     sai_dao_lei_xing

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_29:
        DC32     P_timer

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_30:
        DC32     dijihang

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_31:
        DC32     high_now_0

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_32:
        DC32     mid_now_0

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_33:
        DC32     low_now_0

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_34:
        DC32     0x40038010

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_35:
        DC32     0x40038028

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_36:
        DC32     0x40038030

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_37:
        DC32     0x40038040

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_38:
        DC32     0x40038018

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_39:
        DC32     0x40038038

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_40:
        DC32     Servo_Middle

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_41:
        DC32     Servo_value

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_42:
        DC32     Servo_Right

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_43:
        DC32     Servo_Left

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_44:
        DC32     0x40039010

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_45:
        DC32     Very_straight

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_46:
        DC32     0xb5e28

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_47:
        DC32     LCD_P1

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_48:
        DC32     `?<Constant "low_0">`

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_49:
        DC32     low_now

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_50:
        DC32     low_0

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_51:
        DC32     Servo

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_52:
        DC32     `?<Constant "mid_0">`

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_53:
        DC32     mid_now

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_54:
        DC32     mid_0

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_55:
        DC32     pulse2

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_56:
        DC32     `?<Constant "high_0">`

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_57:
        DC32     high_now

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_58:
        DC32     high_0

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_59:
        DC32     `?<Constant "position_now">`

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_60:
        DC32     position_now

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_61:
        DC32     `?<Constant "position_diff">`

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_62:
        DC32     position_diff

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_63:
        DC32     `?<Constant "position_now_low">`

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_64:
        DC32     position_now_low

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_65:
        DC32     `?<Constant "Servo_value">`

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_66:
        DC32     `?<Constant "dijihang">`

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable62_67:
        DC32     LCD_P3

        SECTION `.iar_vfe_header`:DATA:REORDER:NOALLOC:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        SECTION __DLIB_PERTHREAD:DATA:REORDER:NOROOT(0)
        SECTION_TYPE SHT_PROGBITS, 0

        SECTION __DLIB_PERTHREAD_init:DATA:REORDER:NOROOT(0)
        SECTION_TYPE SHT_PROGBITS, 0

        END
// 4500 //************************主函数END***********************
// 
// 18 708 bytes in section .bss
//     87 bytes in section .data
//    760 bytes in section .rodata
// 28 270 bytes in section .text
// 
// 28 270 bytes of CODE  memory
//    760 bytes of CONST memory
// 18 795 bytes of DATA  memory
//
//Errors: none
//Warnings: 86
