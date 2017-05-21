/********************************************************   
��ƽ    ̨������K60X256�๦�ܿ�����
����    д������
��Designed��by Chiu Sir
��E-mail  ��chiusir@yahoo.cn
������汾��V1.0
�������¡�2012��1��3��
�������Ϣ�ο����е�ַ��
����    վ��http://www.lqist.cn
���Ա����̡�http://shop36265907.taobao.com
------------------------------------------------
��dev.env.��CodeWarrior 10.1/IAR
��Target  ��K60X256
��Crystal ��50.000Mhz
��busclock��???.000MHz
��pllclock��100.000MHz    
***************************
------------------------------------
  ʹ��˵����
���س����LED������˸ 
  
 *********************************************************/

#define GPIO_PIN_MASK      0x1Fu    //0x1f=31,����λ��Ϊ0--31��Ч
#define GPIO_PIN(x)        (((1)<<(x & GPIO_PIN_MASK)))  //�ѵ�ǰλ��1

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
	//��������GPIO�ڵ�ת��ʱ��
	SIM_SCGC5 = SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;
	
	//����PORTA pin14,pin15ΪGPIO��
        PORTA_PCR10=(0|PORT_PCR_MUX(1));
	PORTA_PCR11=(0|PORT_PCR_MUX(1)); 
	PORTA_PCR12=(0|PORT_PCR_MUX(1));
	PORTA_PCR13=(0|PORT_PCR_MUX(1));
	PORTA_PCR14=(0|PORT_PCR_MUX(1));
	PORTA_PCR15=(0|PORT_PCR_MUX(1)); 
	PORTA_PCR16=(0|PORT_PCR_MUX(1));
	PORTA_PCR17=(0|PORT_PCR_MUX(1)); 
	
	//����PORTA pin10--17Ϊ�������
	GPIOA_PDDR=GPIO_PDDR_PDD(GPIO_PIN(10)|GPIO_PIN(11)|GPIO_PIN(12)|GPIO_PIN(13)|GPIO_PIN(14)|GPIO_PIN(15)|GPIO_PIN(16)|GPIO_PIN(17));
	

	for(;;)  
	{	   	
		GPIOE_PDIR
                GPIOA_PDOR &= ~GPIO_PDOR_PDO(GPIO_PIN(14));	//IO������͵�ƽ����	
		GPIOA_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(15));	//IO������ߵ�ƽ����   	
		GPIOA_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(16));	//IO������ߵ�ƽ����
		GPIOA_PDOR |=  GPIO_PDOR_PDO(GPIO_PIN(17));	//IO������ߵ�ƽ����
		delay();//��ͣ
	
	}	
}
