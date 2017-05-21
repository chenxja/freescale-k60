/********************************************************   
【平    台】龙丘K60X256多功能开发板
【编    写】龙丘
【Designed】by Chiu Sir
【E-mail  】chiusir@yahoo.cn
【软件版本】V1.0
【最后更新】2012年1月3日
【相关信息参考下列地址】
【网    站】http://www.lqist.cn
【淘宝店铺】http://shop36265907.taobao.com
------------------------------------------------
【dev.env.】CodeWarrior 10.1/IAR
【Target  】K60X256
【Crystal 】50.000Mhz
【busclock】???.000MHz
【pllclock】100.000MHz    
***************************
------------------------------------
  使用说明：
下载程序后LED交替闪烁 
  
 *********************************************************/

#define GPIO_PIN_MASK      0x1Fu    //0x1f=31,限制位数为0--31有效
#define GPIO_PIN(x)        (((1)<<(x & GPIO_PIN_MASK)))  //把当前位置1

//give some delay~~
void delay()
{
   int i =0;	
   int j=0;
   for(i=0;i<1000;i++)
	  for(j=0;j<1000;j++)
	      asm("nop");
   
}

int main(void)
{	
	//=========================== Code for test GPIO==================================================
	//开启各个GPIO口的转换时钟
	SIM_SCGC5 = SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;
	
	//设置PORTA pin14,pin15为GPIO口
        PORTA_PCR10=(0|PORT_PCR_MUX(1));
	PORTA_PCR11=(0|PORT_PCR_MUX(1)); 
	PORTA_PCR12=(0|PORT_PCR_MUX(1));
	PORTA_PCR13=(0|PORT_PCR_MUX(1));
	PORTA_PCR14=(0|PORT_PCR_MUX(1));
	PORTA_PCR15=(0|PORT_PCR_MUX(1)); 
	PORTA_PCR16=(0|PORT_PCR_MUX(1));
	PORTA_PCR17=(0|PORT_PCR_MUX(1)); 
	
	//设置PORTA pin10--17为输出方向
	GPIOA_PDDR=GPIO_PDDR_PDD(GPIO_PIN(10)|GPIO_PIN(11)|GPIO_PIN(12)|GPIO_PIN(13)|GPIO_PIN(14)|GPIO_PIN(15)|GPIO_PIN(16)|GPIO_PIN(17));
	

	for(;;)  
	{	   	
		GPIOE_PDIR
                GPIOA_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(14));	//IO口输出低电平，亮	
		GPIOA_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(15));	//IO口输出高电平，灭   	
		GPIOA_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(16));	//IO口输出高电平，灭
		GPIOA_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(17));	//IO口输出高电平，灭
		delay();//暂停
	
	}	
}
