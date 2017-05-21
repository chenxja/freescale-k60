///////////////////////////////////////////////////////////////////////////////
//                                                                            /
// IAR ANSI C/C++ Compiler V6.30.1.53127/W32 for ARM    13/Aug/2013  20:22:03 /
// Copyright 1999-2011 IAR Systems AB.                                        /
//                                                                            /
//    Cpu mode     =  thumb                                                   /
//    Endian       =  little                                                  /
//    Source file  =  F:\����\С��\��������\0813\src\Sources\C\Component_C\ua /
//                    rt.c                                                    /
//    Command line =  F:\����\С��\��������\0813\src\Sources\C\Component_C\ua /
//                    rt.c -D IAR -D TWR_K60N512 -lCN                         /
//                    F:\����\С��\��������\0813\bin\Flash\List\ -lB          /
//                    F:\����\С��\��������\0813\bin\Flash\List\ -o           /
//                    F:\����\С��\��������\0813\bin\Flash\Obj\ --no_cse      /
//                    --no_unroll --no_inline --no_code_motion --no_tbaa      /
//                    --no_clustering --no_scheduling --debug                 /
//                    --endian=little --cpu=Cortex-M4 -e --fpu=None           /
//                    --dlib_config "E:\CD-EWARM-6301-3142                    /
//                    (1)\arm\INC\c\DLib_Config_Normal.h" -I                  /
//                    F:\����\С��\��������\0813\src\Sources\H\ -I            /
//                    F:\����\С��\��������\0813\src\Sources\H\Component_H\   /
//                    -I F:\����\С��\��������\0813\src\Sources\H\Frame_H\    /
//                    -Ol --use_c++_inline                                    /
//    List file    =  F:\����\С��\��������\0813\bin\Flash\List\uart.s        /
//                                                                            /
//                                                                            /
///////////////////////////////////////////////////////////////////////////////

        NAME uart

        #define SHT_PROGBITS 0x1

        EXTERN disable_irq
        EXTERN enable_irq

        PUBLIC UART5_Init
        PUBLIC disableuartreint
        PUBLIC enableuartreint
        PUBLIC uart_init
        PUBLIC uart_re1
        PUBLIC uart_reN
        PUBLIC uart_send1
        PUBLIC uart_sendN

// F:\����\С��\��������\0813\src\Sources\C\Component_C\uart.c
//    1 //-------------------------------------------------------------------------*
//    2 // �ļ���: uart.c                                                          *
//    3 // ˵  ��: uart����Դ�ļ�                                                  *
//    4 //-------------------------------------------------------------------------*
//    5 
//    6 #include "uart.h"
//    7 
//    8 //-------------------------------------------------------------------------*
//    9 //������: uart_init                                                        *
//   10 //��  ��: ��ʼ��uartxģ�顣                                                *
//   11 //��  ��: uartch:���ں�                                                    *
//   12 //        sysclk:ϵͳ����ʱ�ӣ���MHzΪ��λ                                 *
//   13 //        baud:�����ʣ���9600��38400�ȣ�һ����˵���ٶ�Խ����ͨ��Խ��       *
//   14 //��  ��: ��                                                               *
//   15 //˵  ��:                                                                  *
//   16 //-------------------------------------------------------------------------*

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//   17 void uart_init (UART_MemMapPtr uartch, uint32 sysclk, uint32 baud)
//   18 {
uart_init:
        PUSH     {R4,R5}
//   19 	register uint16 sbr, brfa;
//   20 	uint8 temp;
//   21 
//   22 	//ʹ������
//   23 	if (uartch == UART0_BASE_PTR)
        LDR.N    R3,??DataTable1  ;; 0x4006a000
        CMP      R0,R3
        BNE.N    ??uart_init_0
//   24 	{
//   25 		//��PTD6��ʹ��UART0_TXD����
//   26 		PORTD_PCR6 = PORT_PCR_MUX(0x3);
        LDR.N    R3,??DataTable1_1  ;; 0x4004c018
        MOV      R4,#+768
        STR      R4,[R3, #+0]
//   27 		//��PTD7��ʹ��UART0_RXD
//   28 		PORTD_PCR7 = PORT_PCR_MUX(0x3); 
        LDR.N    R3,??DataTable1_2  ;; 0x4004c01c
        MOV      R4,#+768
        STR      R4,[R3, #+0]
        B.N      ??uart_init_1
//   29 	}else if (uartch == UART1_BASE_PTR)
??uart_init_0:
        LDR.N    R3,??DataTable1_3  ;; 0x4006b000
        CMP      R0,R3
        BNE.N    ??uart_init_2
//   30 	{
//   31 		//��PTC4��ʹ��UART1_TXD����
//   32 		PORTC_PCR4 = PORT_PCR_MUX(0x3); 
        LDR.N    R3,??DataTable1_4  ;; 0x4004b010
        MOV      R4,#+768
        STR      R4,[R3, #+0]
//   33 		
//   34 		//��PTC3��ʹ��UART1_RXD
//   35 		PORTC_PCR3 = PORT_PCR_MUX(0x3); 
        LDR.N    R3,??DataTable1_5  ;; 0x4004b00c
        MOV      R4,#+768
        STR      R4,[R3, #+0]
        B.N      ??uart_init_1
//   36 	}else if (uartch == UART2_BASE_PTR)
??uart_init_2:
        LDR.N    R3,??DataTable1_6  ;; 0x4006c000
        CMP      R0,R3
        BNE.N    ??uart_init_3
//   37 	{
//   38 		//��PTD3��ʹ��UART2_TXD����
//   39 		PORTD_PCR3 = PORT_PCR_MUX(0x3); 
        LDR.N    R3,??DataTable1_7  ;; 0x4004c00c
        MOV      R4,#+768
        STR      R4,[R3, #+0]
//   40 		//��PTD2��ʹ��UART2_RXD
//   41 		PORTD_PCR2 = PORT_PCR_MUX(0x3); 
        LDR.N    R3,??DataTable1_8  ;; 0x4004c008
        MOV      R4,#+768
        STR      R4,[R3, #+0]
        B.N      ??uart_init_1
//   42 	}else if (uartch == UART3_BASE_PTR)
??uart_init_3:
        LDR.N    R3,??DataTable1_9  ;; 0x4006d000
        CMP      R0,R3
        BNE.N    ??uart_init_4
//   43 	{
//   44 		//��PTC17��ʹ��UART3_TXD����
//   45 		PORTC_PCR17 = PORT_PCR_MUX(0x3); 
        LDR.N    R3,??DataTable1_10  ;; 0x4004b044
        MOV      R4,#+768
        STR      R4,[R3, #+0]
//   46 		//��PTC16��ʹ��UART3_RXD
//   47 		PORTC_PCR16 = PORT_PCR_MUX(0x3); 
        LDR.N    R3,??DataTable1_11  ;; 0x4004b040
        MOV      R4,#+768
        STR      R4,[R3, #+0]
        B.N      ??uart_init_1
//   48 	}else if (uartch == UART4_BASE_PTR)
??uart_init_4:
        LDR.N    R3,??DataTable1_12  ;; 0x400ea000
        CMP      R0,R3
        BNE.N    ??uart_init_5
//   49 	{
//   50 		//��PTE24��ʹ��UART4_TXD����
//   51 		PORTE_PCR24 = PORT_PCR_MUX(0x3); 
        LDR.N    R3,??DataTable1_13  ;; 0x4004d060
        MOV      R4,#+768
        STR      R4,[R3, #+0]
//   52 		//��PTE25��ʹ��UART4_RXD
//   53 		PORTE_PCR25 = PORT_PCR_MUX(0x3); 
        LDR.N    R3,??DataTable1_14  ;; 0x4004d064
        MOV      R4,#+768
        STR      R4,[R3, #+0]
        B.N      ??uart_init_1
//   54 	}else if (uartch == UART5_BASE_PTR)
??uart_init_5:
        LDR.N    R3,??DataTable1_15  ;; 0x400eb000
        CMP      R0,R3
        BNE.N    ??uart_init_1
//   55 	{
//   56 		//��PTE8��ʹ��UART5_TXD����
//   57 		PORTD_PCR9 = PORT_PCR_MUX(0x3); 
        LDR.N    R3,??DataTable1_16  ;; 0x4004c024
        MOV      R4,#+768
        STR      R4,[R3, #+0]
//   58 		//��PTE9��ʹ��UART5_RXD
//   59 		PORTD_PCR8 = PORT_PCR_MUX(0x3); 
        LDR.N    R3,??DataTable1_17  ;; 0x4004c020
        MOV      R4,#+768
        STR      R4,[R3, #+0]
//   60 	}
//   61 	 
//   62 	//ʹ�ܴ���ʱ��    
//   63 	if(uartch == UART0_BASE_PTR)
??uart_init_1:
        LDR.N    R3,??DataTable1  ;; 0x4006a000
        CMP      R0,R3
        BNE.N    ??uart_init_6
//   64 		SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;
        LDR.N    R3,??DataTable1_18  ;; 0x40048034
        LDR      R3,[R3, #+0]
        ORRS     R3,R3,#0x400
        LDR.N    R4,??DataTable1_18  ;; 0x40048034
        STR      R3,[R4, #+0]
        B.N      ??uart_init_7
//   65 	else
//   66 		if (uartch == UART1_BASE_PTR)
??uart_init_6:
        LDR.N    R3,??DataTable1_3  ;; 0x4006b000
        CMP      R0,R3
        BNE.N    ??uart_init_8
//   67 			SIM_SCGC4 |= SIM_SCGC4_UART1_MASK;
        LDR.N    R3,??DataTable1_18  ;; 0x40048034
        LDR      R3,[R3, #+0]
        ORRS     R3,R3,#0x800
        LDR.N    R4,??DataTable1_18  ;; 0x40048034
        STR      R3,[R4, #+0]
        B.N      ??uart_init_7
//   68 		else
//   69 			if (uartch == UART2_BASE_PTR)
??uart_init_8:
        LDR.N    R3,??DataTable1_6  ;; 0x4006c000
        CMP      R0,R3
        BNE.N    ??uart_init_9
//   70 				SIM_SCGC4 |= SIM_SCGC4_UART2_MASK;
        LDR.N    R3,??DataTable1_18  ;; 0x40048034
        LDR      R3,[R3, #+0]
        ORRS     R3,R3,#0x1000
        LDR.N    R4,??DataTable1_18  ;; 0x40048034
        STR      R3,[R4, #+0]
        B.N      ??uart_init_7
//   71 			else
//   72 				if(uartch == UART3_BASE_PTR)
??uart_init_9:
        LDR.N    R3,??DataTable1_9  ;; 0x4006d000
        CMP      R0,R3
        BNE.N    ??uart_init_10
//   73 					SIM_SCGC4 |= SIM_SCGC4_UART3_MASK;
        LDR.N    R3,??DataTable1_18  ;; 0x40048034
        LDR      R3,[R3, #+0]
        ORRS     R3,R3,#0x2000
        LDR.N    R4,??DataTable1_18  ;; 0x40048034
        STR      R3,[R4, #+0]
        B.N      ??uart_init_7
//   74 				else
//   75 					if(uartch == UART4_BASE_PTR)
??uart_init_10:
        LDR.N    R3,??DataTable1_12  ;; 0x400ea000
        CMP      R0,R3
        BNE.N    ??uart_init_11
//   76 						SIM_SCGC1 |= SIM_SCGC1_UART4_MASK;
        LDR.N    R3,??DataTable1_19  ;; 0x40048028
        LDR      R3,[R3, #+0]
        ORRS     R3,R3,#0x400
        LDR.N    R4,??DataTable1_19  ;; 0x40048028
        STR      R3,[R4, #+0]
        B.N      ??uart_init_7
//   77 					else
//   78 						SIM_SCGC1 |= SIM_SCGC1_UART5_MASK;
??uart_init_11:
        LDR.N    R3,??DataTable1_19  ;; 0x40048028
        LDR      R3,[R3, #+0]
        ORRS     R3,R3,#0x800
        LDR.N    R4,??DataTable1_19  ;; 0x40048028
        STR      R3,[R4, #+0]
//   79 								
//   80 	//��ֹ���ͽ���
//   81 	UART_C2_REG(uartch) &= ~(UART_C2_TE_MASK
//   82 				| UART_C2_RE_MASK );
??uart_init_7:
        LDRB     R3,[R0, #+3]
        ANDS     R3,R3,#0xF3
        STRB     R3,[R0, #+3]
//   83 	
//   84 	//���ó�8λ��У��ģʽ
//   85 	UART_C1_REG(uartch) = 0;
        MOVS     R3,#+0
        STRB     R3,[R0, #+2]
//   86 	
//   87 	//���㲨���ʣ�����0��1ʹ���ں�ʱ�ӣ���������ʹ������ʱ�ӣ�ϵͳʱ��Ϊ
//   88 	//����ʱ�ӵ�2��
//   89 	if ((uartch == UART0_BASE_PTR) | (uartch == UART1_BASE_PTR))//
        LDR.N    R3,??DataTable1  ;; 0x4006a000
        CMP      R0,R3
        BEQ.N    ??uart_init_12
        MOVS     R3,#+0
        LDR.N    R4,??DataTable1_3  ;; 0x4006b000
        CMP      R0,R4
        BEQ.N    ??uart_init_12
        ORRS     R3,R3,#0x0
        UXTB     R3,R3            ;; ZeroExt  R3,R3,#+24,#+24
        CMP      R3,#+0
        BEQ.N    ??uart_init_13
//   90 		sysclk+=sysclk;
??uart_init_12:
        ADDS     R1,R1,R1
//   91 	
//   92 	sbr = (uint16)((sysclk*1000)/(baud * 16));
??uart_init_13:
        MOV      R3,#+1000
        MUL      R3,R3,R1
        LSLS     R4,R2,#+4
        UDIV     R3,R3,R4
//   93 	temp = UART_BDH_REG(uartch) & ~(UART_BDH_SBR(0x1F));
        LDRB     R4,[R0, #+0]
        ANDS     R4,R4,#0xE0
//   94 	UART_BDH_REG(uartch) = temp |  UART_BDH_SBR(((sbr & 0x1F00) >> 8));
        UXTH     R3,R3            ;; ZeroExt  R3,R3,#+16,#+16
        LSRS     R5,R3,#+8
        ANDS     R5,R5,#0x1F
        ORRS     R4,R5,R4
        STRB     R4,[R0, #+0]
//   95 	UART_BDL_REG(uartch) = (uint8)(sbr & UART_BDL_SBR_MASK);
        STRB     R3,[R0, #+1]
//   96 	brfa = (((sysclk*32000)/(baud * 16)) - (sbr * 32));
        MOV      R4,#+32000
        MULS     R1,R4,R1
        LSLS     R2,R2,#+4
        UDIV     R1,R1,R2
        UXTH     R3,R3            ;; ZeroExt  R3,R3,#+16,#+16
        SUBS     R1,R1,R3, LSL #+5
//   97 	temp = UART_C4_REG(uartch) & ~(UART_C4_BRFA(0x1F));
        LDRB     R2,[R0, #+10]
        ANDS     R4,R2,#0xE0
//   98 	UART_C4_REG(uartch) = temp |  UART_C4_BRFA(brfa);    
        ANDS     R1,R1,#0x1F
        ORRS     R1,R1,R4
        STRB     R1,[R0, #+10]
//   99 	
//  100 	//ʹ�ܷ��ͽ���
//  101 	UART_C2_REG(uartch) |= (UART_C2_TE_MASK
//  102 				| UART_C2_RE_MASK );
        LDRB     R1,[R0, #+3]
        ORRS     R1,R1,#0xC
        STRB     R1,[R0, #+3]
//  103 }
        POP      {R4,R5}
        BX       LR               ;; return
//  104 

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  105 void UART5_Init(void)
//  106 {
UART5_Init:
        PUSH     {R4}
//  107 	unsigned long uartclk_khz=45000;//ʱ��180MHz
        MOVW     R0,#+45000
//  108 	unsigned long baud=115200;//������256000
        MOVS     R1,#+115200
//  109 	unsigned short sbr,brfa;
//  110 	PORTD_PCR8|=PORT_PCR_MUX(3);//��D6��������Ϊģʽ3����UART0_RX
        LDR.N    R2,??DataTable1_17  ;; 0x4004c020
        LDR      R2,[R2, #+0]
        MOV      R3,#+768
        ORRS     R2,R3,R2
        LDR.N    R3,??DataTable1_17  ;; 0x4004c020
        STR      R2,[R3, #+0]
//  111 	PORTD_PCR9|=PORT_PCR_MUX(3);//��D7��������Ϊģʽ3����UART0_TX
        LDR.N    R2,??DataTable1_16  ;; 0x4004c024
        LDR      R2,[R2, #+0]
        MOV      R3,#+768
        ORRS     R2,R3,R2
        LDR.N    R3,??DataTable1_16  ;; 0x4004c024
        STR      R2,[R3, #+0]
//  112 	SIM_SCGC1|=SIM_SCGC1_UART5_MASK;//����UART0ʱ��
        LDR.N    R2,??DataTable1_19  ;; 0x40048028
        LDR      R2,[R2, #+0]
        ORRS     R2,R2,#0x800
        LDR.N    R3,??DataTable1_19  ;; 0x40048028
        STR      R2,[R3, #+0]
//  113 	sbr=(unsigned short)((uartclk_khz*1000)/(baud*16));//���㲢���ò�����
        MOV      R2,#+1000
        MUL      R2,R2,R0
        LSLS     R3,R1,#+4
        UDIV     R2,R2,R3
//  114 	
//  115 	UART5_BDH=(unsigned char)((sbr&0x1F00)>>8);
        UXTH     R2,R2            ;; ZeroExt  R2,R2,#+16,#+16
        ASRS     R3,R2,#+8
        ANDS     R3,R3,#0x1F
        LDR.N    R4,??DataTable1_15  ;; 0x400eb000
        STRB     R3,[R4, #+0]
//  116 	UART5_BDL=(unsigned char)(sbr&0x00FF);
        LDR.N    R3,??DataTable1_20  ;; 0x400eb001
        STRB     R2,[R3, #+0]
//  117 	brfa = (((uartclk_khz*32000)/(baud*16))-(sbr*32));
        MOV      R3,#+32000
        MULS     R0,R3,R0
        LSLS     R1,R1,#+4
        UDIV     R0,R0,R1
        UXTH     R2,R2            ;; ZeroExt  R2,R2,#+16,#+16
        SUBS     R0,R0,R2, LSL #+5
//  118 	UART0_C4 = (unsigned char)(brfa & 0x001F);
        ANDS     R0,R0,#0x1F
        LDR.N    R1,??DataTable1_21  ;; 0x4006a00a
        STRB     R0,[R1, #+0]
//  119 	UART0_C2|=(UART_C2_TE_MASK|UART_C2_RE_MASK);
        LDR.N    R0,??DataTable1_22  ;; 0x4006a003
        LDRB     R0,[R0, #+0]
        ORRS     R0,R0,#0xC
        LDR.N    R1,??DataTable1_22  ;; 0x4006a003
        STRB     R0,[R1, #+0]
//  120 	UART0_C1=0;	
        LDR.N    R0,??DataTable1_23  ;; 0x4006a002
        MOVS     R1,#+0
        STRB     R1,[R0, #+0]
//  121 }
        POP      {R4}
        BX       LR               ;; return

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable1:
        DC32     0x4006a000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable1_1:
        DC32     0x4004c018

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable1_2:
        DC32     0x4004c01c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable1_3:
        DC32     0x4006b000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable1_4:
        DC32     0x4004b010

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable1_5:
        DC32     0x4004b00c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable1_6:
        DC32     0x4006c000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable1_7:
        DC32     0x4004c00c

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable1_8:
        DC32     0x4004c008

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable1_9:
        DC32     0x4006d000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable1_10:
        DC32     0x4004b044

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable1_11:
        DC32     0x4004b040

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable1_12:
        DC32     0x400ea000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable1_13:
        DC32     0x4004d060

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable1_14:
        DC32     0x4004d064

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable1_15:
        DC32     0x400eb000

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable1_16:
        DC32     0x4004c024

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable1_17:
        DC32     0x4004c020

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable1_18:
        DC32     0x40048034

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable1_19:
        DC32     0x40048028

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable1_20:
        DC32     0x400eb001

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable1_21:
        DC32     0x4006a00a

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable1_22:
        DC32     0x4006a003

        SECTION `.text`:CODE:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
??DataTable1_23:
        DC32     0x4006a002
//  122 //-------------------------------------------------------------------------*
//  123 //������: uart_re1                                                         *
//  124 //��  ��: ���н���1���ֽ�                                                  *
//  125 //��  ��: uartch: ���ں�                                                   *
//  126 //         ch:    ���յ����ֽ�                                             *
//  127 //��  ��: �ɹ�:1;ʧ��:0                                                    *
//  128 //˵  ��:                                                                  *
//  129 //-------------------------------------------------------------------------*

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  130 uint8 uart_re1 (UART_MemMapPtr uartch,uint8 *ch)
//  131 {
//  132     uint32 k;
//  133     
//  134     for (k = 0; k < 0xfbbb; k++)//��ʱ������
uart_re1:
        MOVS     R2,#+0
        B.N      ??uart_re1_0
??uart_re1_1:
        ADDS     R2,R2,#+1
??uart_re1_0:
        MOVW     R3,#+64443
        CMP      R2,R3
        BCS.N    ??uart_re1_2
//  135 		if((UART_S1_REG(uartch) & UART_S1_RDRF_MASK)!= 0)//�жϽ��ջ������Ƿ���
        LDRB     R3,[R0, #+4]
        LSLS     R3,R3,#+26
        BPL.N    ??uart_re1_1
//  136 		{
//  137 			*ch = UART_D_REG(uartch);
        LDRB     R0,[R0, #+7]
        STRB     R0,[R1, #+0]
//  138 			return 1; 			//���ܳɹ�
        MOVS     R0,#+1
        B.N      ??uart_re1_3
//  139 		} 
//  140 	if(k>=0xfbbb) 
??uart_re1_2:
        MOVW     R0,#+64443
        CMP      R2,R0
        BCC.N    ??uart_re1_4
//  141 	{
//  142 		return 0;			//����ʧ��
        MOVS     R0,#+0
        B.N      ??uart_re1_3
//  143 	} 
//  144     return 0;
??uart_re1_4:
        MOVS     R0,#+0
??uart_re1_3:
        BX       LR               ;; return
//  145 }
//  146 
//  147 //-------------------------------------------------------------------------*
//  148 //������: uart_send1                                                       *
//  149 //��  ��: ���з���1���ֽ�                                                  *
//  150 //��  ��: uartch: ���ں�                                                   *
//  151 //         ch:    Ҫ���͵��ֽ�                                             *
//  152 //��  ��: ��                                                               *
//  153 //˵  ��:                                                                  *
//  154 //-------------------------------------------------------------------------*

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  155 void uart_send1 (UART_MemMapPtr uartch, uint8 ch)
//  156 {
//  157     //�ȴ����ͻ�������
//  158     while(!(UART_S1_REG(uartch) & UART_S1_TDRE_MASK));
uart_send1:
??uart_send1_0:
        LDRB     R2,[R0, #+4]
        LSLS     R2,R2,#+24
        BPL.N    ??uart_send1_0
//  159     //��������
//  160     UART_D_REG(uartch) = (uint8)ch;
        STRB     R1,[R0, #+7]
//  161  }
        BX       LR               ;; return
//  162 
//  163 //-------------------------------------------------------------------------*
//  164 //������: uart_reN                                                         *
//  165 //��  ��: ���� ����n���ֽ�                                                 *
//  166 //��  ��: uartch: ���ں�                                                   *
//  167 //        buff: ���ջ�����                                                 *
//  168 //		  len:���ճ���                                             *
//  169 //��  ��: 1:�ɹ�;0:ʧ��                                                    *
//  170 //˵  ��:                                                                  *
//  171 //-------------------------------------------------------------------------*

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  172 uint8 uart_reN (UART_MemMapPtr uartch ,uint8* buff,uint16 len)
//  173 {
uart_reN:
        PUSH     {R3-R7,LR}
        MOVS     R4,R0
        MOVS     R5,R1
        MOVS     R6,R2
//  174     uint16 m=0; 
        MOVS     R7,#+0
        B.N      ??uart_reN_0
//  175     while (m < len)
//  176     { 	          
//  177   	    if(0==uart_re1(uartch,&buff[m]))
//  178   	    	return 0;  //����ʧ��
//  179   	    else m++;
??uart_reN_1:
        ADDS     R7,R7,#+1
??uart_reN_0:
        UXTH     R7,R7            ;; ZeroExt  R7,R7,#+16,#+16
        UXTH     R6,R6            ;; ZeroExt  R6,R6,#+16,#+16
        CMP      R7,R6
        BCS.N    ??uart_reN_2
        UXTH     R7,R7            ;; ZeroExt  R7,R7,#+16,#+16
        ADDS     R1,R7,R5
        MOVS     R0,R4
        BL       uart_re1
        CMP      R0,#+0
        BNE.N    ??uart_reN_1
        MOVS     R0,#+0
        B.N      ??uart_reN_3
//  180     } 
//  181     
//  182     return 1;          //���ճɹ�
??uart_reN_2:
        MOVS     R0,#+1
??uart_reN_3:
        POP      {R1,R4-R7,PC}    ;; return
//  183     
//  184 }
//  185 
//  186 //-------------------------------------------------------------------------*
//  187 //������: uart_sendN                                                       *
//  188 //��  ��: ���� ����n���ֽ�                                                 *
//  189 //��  ��: uartch: ���ں�                                                   *
//  190 //        buff: ���ͻ�����                                                 *
//  191 //		  len:���ͳ���                                             *
//  192 //��  ��: ��                                                               *
//  193 //˵  ��:                                                                  *
//  194 //-------------------------------------------------------------------------*

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  195 void uart_sendN (UART_MemMapPtr uartch ,uint8* buff,uint16 len)
//  196 {
uart_sendN:
        PUSH     {R3-R7,LR}
        MOVS     R4,R0
        MOVS     R5,R1
        MOVS     R6,R2
//  197     int i;
//  198 	for(i=0;i<len;i++)
        MOVS     R7,#+0
        B.N      ??uart_sendN_0
//  199     {
//  200 		uart_send1(uartch,buff[i]);
??uart_sendN_1:
        LDRB     R1,[R7, R5]
        MOVS     R0,R4
        BL       uart_send1
//  201     }
        ADDS     R7,R7,#+1
??uart_sendN_0:
        UXTH     R6,R6            ;; ZeroExt  R6,R6,#+16,#+16
        CMP      R7,R6
        BLT.N    ??uart_sendN_1
//  202 }
        POP      {R0,R4-R7,PC}    ;; return
//  203 
//  204 //-------------------------------------------------------------------------*
//  205 //������: enableuartreint                                                  *
//  206 //��  ��: �����ڽ����ж�                                                   *
//  207 //��  ��: uartch: ���ں�                                                   *
//  208 //        irqno: ��Ӧirq��                                                 *
//  209 //��  ��: ��                                                               *
//  210 //˵  ��:                                                                  *
//  211 //-------------------------------------------------------------------------*

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  212 void enableuartreint(UART_MemMapPtr uartch,uint8 irqno)
//  213 {
enableuartreint:
        PUSH     {R7,LR}
//  214 	UART_C2_REG(uartch)|=UART_C2_RIE_MASK;   //����UART�����ж�
        LDRB     R2,[R0, #+3]
        ORRS     R2,R2,#0x20
        STRB     R2,[R0, #+3]
//  215 	enable_irq(irqno);			 //���������ŵ�IRQ�ж�
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        MOVS     R0,R1
        BL       enable_irq
//  216 }
        POP      {R0,PC}          ;; return
//  217 
//  218 //-------------------------------------------------------------------------*
//  219 //������: disableuartreint                                                 *
//  220 //��  ��: �ش��ڽ����ж�                                                   *
//  221 //��  ��: uartch: ���ں�                                                   *
//  222 //        irqno: ��Ӧirq��                                                 *
//  223 //��  ��: ��                                                               *
//  224 //˵  ��:                                                                  *
//  225 //-------------------------------------------------------------------------*

        SECTION `.text`:CODE:NOROOT(1)
        THUMB
//  226 void disableuartreint(UART_MemMapPtr uartch,uint8 irqno)
//  227 {
disableuartreint:
        PUSH     {R7,LR}
//  228 	UART_C2_REG(uartch)&=~UART_C2_RIE_MASK;   //��ֹUART�����ж�
        LDRB     R2,[R0, #+3]
        ANDS     R2,R2,#0xDF
        STRB     R2,[R0, #+3]
//  229 	disable_irq(irqno);			  //�ؽ������ŵ�IRQ�ж�
        UXTB     R1,R1            ;; ZeroExt  R1,R1,#+24,#+24
        MOVS     R0,R1
        BL       disable_irq
//  230 }
        POP      {R0,PC}          ;; return

        SECTION `.iar_vfe_header`:DATA:REORDER:NOALLOC:NOROOT(2)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        SECTION __DLIB_PERTHREAD:DATA:REORDER:NOROOT(0)
        SECTION_TYPE SHT_PROGBITS, 0

        SECTION __DLIB_PERTHREAD_init:DATA:REORDER:NOROOT(0)
        SECTION_TYPE SHT_PROGBITS, 0

        END
//  231 
//  232 
//  233 
//  234 
//  235 
//  236 
//  237 
//  238 
//  239 
//  240 
//  241 
//  242 
//  243 
//  244 
//  245 
// 
// 764 bytes in section .text
// 
// 764 bytes of CODE memory
//
//Errors: none
//Warnings: none
