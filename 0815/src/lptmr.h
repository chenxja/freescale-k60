/********************************************************
【平    台】龙丘CORTEX-M4开发板/系统板
【编    写】龙丘
【Designed】by Chiu Sir
【E-mail  】chiusir@yahoo.cn
【软件版本】V1.0
【最后更新】2012年3月4日
【相关信息参考下列地址】
【网    站】http://www.lqist.cn
【淘宝店铺】http://shop36265907.taobao.com
【dev.env.】IAR6.0 苏大版的
【Target  】CORTEX-M4
【Crystal 】50.000Mhz
【busclock】100.000MHz
【pllclock】 ?MHz
------------------------------------------------   
------------------------------------------------
使用说明:脉冲检测
使用PORTA BIT14,15流水灯程序

用PTC5管脚输入脉冲，串口PTD7输出,9600波特率

测试方法：用跳线链接PTA14到PTC5即可，串口可以输出当前的脉冲数量。

 *********************************************************/


#define GPIO_PIN_MASK      0x1Fu    //0x1f=31,限制位数为0--31有效
#define GPIO_PIN(x)        (((1)<<(x & GPIO_PIN_MASK)))  //把当前位置1

//Pin Select constants for CSR[TPS]
#define HSCMP    0x0
#define LPTMR_ALT1 0x1
#define LPTMR_ALT2 0x2
#define LPTMR_ALT3 0x3

//CSR[TPP] constants
#define RISING 0
#define FALLING LPTMR_CSR_TPP_MASK


uint16 get_counter_value();
void lptmr_clear_registers();

void lptmr_time_counter();
void lptmr_prescale();
void lptmr_internal_ref_input();
void lptmr_lpo_input();
void lptmr_32khz_input();
void lptmr_external_clk_input();
void lptmr_interrupt();
void lptmr_pulse_counter(char pin_select);



char LPTMR_INTERRUPT;  //Global flag variable

/*
 * Timer will trigger interrupt after 5 seconds
 */
void lptmr_interrupt(void)
{
  int compare_value=5000;  //value must be less than 0xFFFF
  LPTMR_INTERRUPT=0; //Clear global variable

  //Reset LPTMR module
  lptmr_clear_registers();

  //printf("\n\n****************************\n");
  //printf("LPTMR Interrupt Example\n");

  /* Enable LPT Interrupt in NVIC*/
  enable_irq(85); //LPTMR Vector is 101. IRQ# is 101-16=85

  /* Configure LPT */
  LPTMR0_CMR=LPTMR_CMR_COMPARE(compare_value);  //Set compare value
  LPTMR0_PSR=LPTMR_PSR_PCS(0x1)|LPTMR_PSR_PBYP_MASK;  //Use LPO clock and bypass prescale
  LPTMR0_CSR=LPTMR_CSR_TIE_MASK;  //Enable LPT interrupt

  //printf("LPTMR using LPO clock with no prescale, and compare value=5000 (5 seconds)\n");
  //printf("Press a key to start counter\n");
  //in_char(); //wait for keyboard press

  LPTMR0_CSR|=LPTMR_CSR_TEN_MASK; //Turn on LPTMR and start counting

  //printf("Counting...\n\n");
  /* Wait for the global variable to be set in LPTMR ISR */
  while(LPTMR_INTERRUPT==0)
  {}

  //printf("Timer should have waited for 5 seconds\n");

  LPTMR0_CSR&=~LPTMR_CSR_TEN_MASK; //Turn off LPT to avoid more interrupts
  //printf("End of LPTMR Interrupt Example\n");
  //printf("****************************\n\n");

  //Reset LPTMR module
  lptmr_clear_registers();
}

void lptmr_isr_example(void)
{
  LPTMR0_CSR|=LPTMR_CSR_TCF_MASK;  //Clear LPT Compare flag
  LPTMR_INTERRUPT=1;  //Set global variable
  //printf("\n\nIn LPT ISR!\n\n");
}

/*
 *  Counting example using the LPO clock.
 *
 *  Sets compare value to 5000. Thus when using the 1Khz LPO clock,
 *   it will take 5 seconds for the Timer Compare Flag to be set.
 *
 */
void lptmr_time_counter()
{
  int compare_value=5000;  //value must be less than 0xFFFF or 65535

  //printf("\n\n****************************\n");
  //printf("LPTMR Time Counting Example\n");

  //Reset LPTMR module
  lptmr_clear_registers();

  /* Configure LPTMR */
  LPTMR0_CMR=LPTMR_CMR_COMPARE(compare_value);  //Set compare value
  LPTMR0_PSR=LPTMR_PSR_PCS(0x1)|LPTMR_PSR_PBYP_MASK;  //Use LPO clock and bypass prescale

  //printf("LPTMR using LPO clock with no prescale, and compare value=5000 (5 seconds)\n");
  //printf("Press a key to start counter\n");
  //in_char(); //wait for keyboard press

  LPTMR0_CSR|=LPTMR_CSR_TEN_MASK; //Turn on LPTMR with default settings

  //Wait for Timer Compare Flag to be set
  while((LPTMR0_CSR & LPTMR_CSR_TCF_MASK)==0)
  {
    //This may not get proper counter data if the CNR is read at the same time it is incremented
    //printf("Current value of counter register CNR is %d\n",get_counter_value());
  }

  //printf("Waited for %d counts\n",compare_value);
  //printf("End of Time Counting Example\n");
  //printf("****************************\n\n");
}

/*
 *  Counting example using the LPO clock and the prescale feature
 *
 *  Sets compare value to 250. Thus when using the 1Khz LPO clock with
 *    LPTMR0_PSR[PRESCALE]=0x4, it will take 8 seconds for Timer Compare Flag
 *    to be set. (1Khz clock/32=31.25Hz clock)
 *
 *  If prescaler was not used, then timer would only wait .25 seconds.
 *
 */
void lptmr_prescale()
{
  int compare_value=250;  //value must be less than 0xFFFF or 65535

  //printf("\n\n****************************\n");
  //printf("LPTMR Time Counting Example with Prescaler\n");

  //Reset LPTMR module
  lptmr_clear_registers();

  /* Configure LPTMR */
  LPTMR0_CMR=LPTMR_CMR_COMPARE(compare_value);  //Set compare value
  LPTMR0_PSR=LPTMR_PSR_PCS(0x1)|LPTMR_PSR_PRESCALE(0x4);  //Use LPO clock and divide by 32

  //printf("LPTMR using LPO clock with PRESCALE=4 and compare value=250 (8 seconds)\n");
  //printf("Press a key to start counter\n");
  //in_char();  //wait for keyboard press

  LPTMR0_CSR|=LPTMR_CSR_TEN_MASK; //Turn on LPTMR with default settings

  //Wait for Timer Compare Flag to be set
  while((LPTMR0_CSR & LPTMR_CSR_TCF_MASK)==0)
  {
    //This may not get proper counter data if the CNR is read at the same time it is incremented
    //printf("Current value of counter register CNR is %d\n",get_counter_value());
  }

  //printf("Waited for %d counts\n",compare_value);
  //printf("End of Time Counting Example with Prescale\n");
  //printf("****************************\n\n");
}
/*
 * Internal Reference Clock (PSC=0x0)
 *   The Internal Reference Clock can come from two clock sources.
 *   If MCG_C2[IRCS]=0, then uses slow internal clock (32kHz)
 *   If MCG_C2[IRCS]=1, then uses fast internal clock (2Mhz)
 *
 *  This example uses fast internal clock. It is pre-scaled to wait for 4 seconds.
 *    Because of trim values, it may be slightly faster or slower than this.
 */
void lptmr_internal_ref_input()
{
  unsigned int compare_value=15625; //4 seconds with prescale=8 and 2Mhz fast clock

  //printf("\n\n****************************\n");
  //printf("Internal Reference Clock Example\n");

  //Reset LPTMR module
  lptmr_clear_registers();

  /* Ensure Internal Reference Clock is Enabled */
  MCG_C1|=MCG_C1_IRCLKEN_MASK;

  //Enable fast internal ref clock by setting MCG_C2[IRCS]=1
  //If wanted to use 32Khz slow mode, set MCG_C2[IRCS]=0 instead
  MCG_C2|=MCG_C2_IRCS_MASK;

  /* Configure LPTMR */
  LPTMR0_CMR=LPTMR_CMR_COMPARE(compare_value);  //Set compare value
  LPTMR0_PSR=LPTMR_PSR_PCS(0x0)|LPTMR_PSR_PRESCALE(0x8);  //Use internal clock prescaled by 512

  //printf("LPTMR using fast internal ref clock with PRESCALE=0x8, and compare value=15625 (4 seconds)\n");
  //printf("Press a key to start counter\n");
  //in_char();

  LPTMR0_CSR|=LPTMR_CSR_TEN_MASK; //Turn on LPT with default settings

  //printf("Counting...\n\n");

  //Wait for Timer Compare Flag to be set
  while((LPTMR0_CSR&LPTMR_CSR_TCF_MASK)==0)
  {
    //This may not get proper counter data if the CNR is read at the same time it is incremented
    ////printf("Current value of counter register CNR is %d\n",get_counter_value());
  }

  //printf("4 seconds should have passed\n");
  //printf("End of Internal reference Clock Source Example\n");
  //printf("****************************\n\n");
}
/*
 * External Reference Clock Example(PSC=0x3)
 *
 * TWR-K60N512 uses a 50MHz external clock
 * TWR-K40X256 uses a 8MHz external clock
 *
 *  Test is pre-scaled to wait for 10 seconds in both cases by adjusting
 *    the compare value.
 */
void lptmr_external_clk_input()
{
  unsigned int compare_value;

  //printf("\n\n****************************\n");
  //printf("External Clock Source Example\n");

  //Reset LPTMR module
  lptmr_clear_registers();

  /* Turn on external reference clock */
  //OSC0_CR |= OSC_CR_ERCLKEN_MASK;  //  selects EXTAL to drive  XOSCxERCLK

  //Set compare value
  compare_value=7630; //~10 seconds with prescale=0xF and 50MHz clock

  /* Configure LPTMR */
  LPTMR0_CMR=LPTMR_CMR_COMPARE(compare_value);  //Set compare value
  LPTMR0_PSR=LPTMR_PSR_PCS(0x3)|LPTMR_PSR_PRESCALE(0xF);  //Use external clock divided by 65536

  //printf("LPTMR using external clock with PRESCALE=0xF, and compare value=%d (10 seconds)\n",compare_value);
  //printf("Press a key to start counter\n");
  //in_char();

  LPTMR0_CSR|=LPTMR_CSR_TEN_MASK; //Turn on LPT with default settings

  //printf("Counting...\n\n");

  //Wait for Timer Compare Flag to be set
  while((LPTMR0_CSR&LPTMR_CSR_TCF_MASK)==0)
  {
    //This may not get proper counter data if the CNR is read at the same time it is incremented
    ////printf("Current value of counter register CNR is %d\n",get_counter_value());
  }

  //printf("Timer should have waited for 10 seconds\n");
  //printf("End of External Clock Example\n");
  //printf("****************************\n\n");
}

/*
 * LPO Clock Example (PSC=0x1)
 *
 *  Test is pre-scaled to wait for 10 seconds. In reality it might be slightly off because
 *   of the trim values for the LPO clock.
 */
void lptmr_lpo_input()
{
  unsigned int compare_value=4000; //4 second delay with the 1khz LPO clock

  //printf("\n\n****************************\n");
  //printf("LPO Clock Source Example\n");

  //Reset LPTMR module
  lptmr_clear_registers();

  /* Configure LPTMR */
  LPTMR0_CMR=LPTMR_CMR_COMPARE(compare_value);  //Set compare value
  LPTMR0_PSR=LPTMR_PSR_PCS(0x1)|LPTMR_PSR_PBYP_MASK;  //Use LPO clock with bypass enabled

  //printf("LPTMR using LPO clock with no prescale, and compare value=4000 (4 seconds)\n");
  //printf("Press a key to start counter\n");
  //in_char();

  LPTMR0_CSR|=LPTMR_CSR_TEN_MASK; //Turn on LPT with default settings

  //printf("Counting...\n\n");

  //Wait for Timer Compare Flag to be set
  while((LPTMR0_CSR&LPTMR_CSR_TCF_MASK)==0)
  {
    //This may not get proper counter data if the CNR is read at the same time it is incremented
    ////printf("Current value of counter register CNR is %d\n",get_counter_value());
  }

  //printf("4 seconds should have passed\n");
  //printf("End of LPO Clock Source Example\n");
  //printf("****************************\n\n");
}

/*
 * 32kHz Input Clock Test (PSC=0x2)
 *   The 32kHz clock (ERCLK32K) can come from two clock sources.
 *   If SOPT1[OSC32KSEL]=1, then it uses 32kHz RTC crystal connected to XTAL32
 *      This is what the code below tests
 *   If SOPT1[OSC32KSEL]=0, then it uses 32kHz System oscillator, and reguires
 *      that the main system clock be a 32kHz crystal. The tower board does not
 *      support that feature.
 *
 *  Test is pre-scaled to wait for 8 seconds.
 */
void lptmr_32khz_input()
{
  unsigned int compare_value=32768; //4 second delay with prescale=1

  //printf("\n\n****************************\n");
  //printf("32 Khz Clock Source Example\n");

  //Reset LPTMR module
  lptmr_clear_registers();

  /*
   * Configure to use 32Khz clock from RTC clock
   */
  //printf("Test using RTC OSC\n");
  SIM_SCGC6|=SIM_SCGC6_RTC_MASK; //Enable RTC registers
  RTC_CR|=RTC_CR_OSCE_MASK;      //Turn on RTC oscillator
  SIM_SOPT1|=SIM_SOPT1_OSC32KSEL_MASK;  //Select RTC OSC as source for ERCLK32K

  /* Configure LPTMR */
  LPTMR0_CMR=LPTMR_CMR_COMPARE(compare_value);  //Set compare value
  LPTMR0_PSR=LPTMR_PSR_PCS(0x2)|LPTMR_PSR_PRESCALE(0x1);  //Use 32khz clock (ERCLK32K) and divide source by 4

  //printf("LPTMR using 32Khz clock with PRESCALE=0x1, and compare value=32768 (4 seconds)\n");
  //printf("Press a key to start counter\n");
  //in_char();

  LPTMR0_CSR|=LPTMR_CSR_TEN_MASK; //Turn on LPT with default settings

  //printf("Counting...\n\n");

  //Wait for Timer Compare Flag to be set
  while((LPTMR0_CSR&LPTMR_CSR_TCF_MASK)==0)
  {
    //This may not get proper counter data if the CNR is read at the same time it is incremented
    ////printf("Current value of counter register CNR is %d\n",get_counter_value());
  }

  //printf("4 seconds should have passed\n");
  //printf("End of 32 Khz Clock Source Example\n");
  //printf("****************************\n\n");
}



/*
 * Counts pulses found on LPTMR0_ALT1, LPTMR0_ALT2, or LPTMR0_ALT3
 *
 *  LPTMR0_ALT1 is pin PORTA19 (ALT6)
 *   On TWR-K60N512, PORTA19 is conected to pin 18 on J5
 * LPTMR0_ALT2 is pin PORTC5 (ALT3).
 *   On K70 Daughter Card, use A70 on the Elevator Board
 * LPTMR0_ALT3 is pin PORTE17 (ALT6)
 *   However this pin is associated with UART2, and setting it to LPTMR_ALT3
 *    interfers with UART communication, so it is not used in this example.
 *
 *
 *
 */
void lptmr_pulse_counter(char pin_select)
{
  unsigned int compare_value=1000;
 // char input;
  //printf("\n\n****************************\n");
  //printf("LPTMR Pulse Counting Example on LPTMR_ALT%d\n\n",pin_select);

  //Reset LPTMR module
  lptmr_clear_registers();

  //Set up GPIO
  if(pin_select==LPTMR_ALT1)
  {
    SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK; //Turn on PORTA clock
    PORTA_PCR19=PORT_PCR_MUX(0x6); //Use ALT6 on PTA19

    //printf("Testing ALT1 pin on PORTA19\n");
    //printf("\tTWR-K70F120M: ALT1 is conected to pin 18 on J15\n");
  }
  else if(pin_select==LPTMR_ALT2)
  {
    SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK; //Turn on PORTC clock
    PORTC_PCR5=PORT_PCR_MUX(0x4); //Use ALT4 on PTC5

    //printf("Testing ALT2 pin on PORTC5\n");
    //printf("\tTWR-K70F120M: ALT2 is conected A70 on TWR-ELEV\n");
  }
  else
  {
    //printf("Invalid pin selected\n");
    //printf("****************************\n");
    return;
  }

  /* Test requires external hardware. Need to confirm if want to run test or not */
  //printf("\nThis test requires a function generator, or another way of producing a pulse signal on the pin specified above. ");
  //printf("Please connect that device to the specified pin\n\n");
  //printf("If you would like to continue with this example, press \"y\". To skip press any other key\n");
  //input=//in_char(); //wait for keyboard press
  //printf("\n");
  //if(input!='y' && input!='Y')
  //{
    //printf("Exiting LPTMR Pulse Counting Example on LPTMR_ALT%d\n",pin_select);
    //printf("****************************\n");
  //  return;
  //}

  LPTMR0_PSR=LPTMR_PSR_PCS(0x1)|LPTMR_PSR_PBYP_MASK; //Use LPO clock but bypass glitch filter
  LPTMR0_CMR=LPTMR_CMR_COMPARE(compare_value);  //Set compare value
  LPTMR0_CSR=LPTMR_CSR_TPS(pin_select)|LPTMR_CSR_TMS_MASK; //Set LPT to use the pin selected, and put in pulse count mode, on rising edge (default)

  //printf("Press any key to start pulse counter\n");
  //in_char(); //wait for keyboard press

  LPTMR0_CSR|=LPTMR_CSR_TEN_MASK; //Turn on LPT

  //Wait for compare flag to be set
  //while((LPTMR0_CSR&LPTMR_CSR_TCF_MASK)==0)
  {
    //This may not get proper counter data if the CNR is read at the same time it is incremented
    //printf("Current value of pulse count register CNR is %d\n",get_counter_value());
  }

  //printf("Detected %d pulses on LPTMR_ALT%d\n",compare_value,pin_select);
  //printf("End of Pulse Counting Example\n");
  //printf("****************************\n");
}


/********************************************************************/

/*
 * Get the current LPTMR Counter Value. 
 *
 * On each read of the LPTMR counter register, software must first write to the 
 * LPTMR counter register with any value. This will synchronize and register the
 * current value of the LPTMR counter register into a temporary register. The 
 * contents of the temporary register are returned on each read of the LPTMR 
 * counter register.
 */
uint16 get_counter_value()
{
  LPTMR0_CNR=0x1;
  return LPTMR0_CNR;
}

/*
 * Zero out all registers.
 *
 */
void lptmr_clear_registers()
{
  LPTMR0_CSR=0x00;
  LPTMR0_PSR=0x00;
  LPTMR0_CMR=0x00;
}

