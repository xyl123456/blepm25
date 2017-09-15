#include "sht10.h"
struct SHT11_DATA_STAUCT  SHT11;
static unsigned char flag = 0;

void SHT11_Delay(unsigned char us)
{
	while(us--)
	{
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
	}
}

void SHT11_Delay_Ms(unsigned char ms)
{
	unsigned char i;
	for(i=0;i<ms;i++)
	{
	SHT11_Delay(250);
	SHT11_Delay(250);
	SHT11_Delay(250);
	SHT11_Delay(250);
	}
}

/********************
*��������������1  0  ��ʾ����
ʱ�ӵ�ʱ��Ϊʱ��������������
*********************/
void SHT11_Start(void)
{
	SHT11_DATA_OUT;//����Ϊ���ģʽ
/*-------------------1--------------*/
	SHT11_DATA  = 1;//����Ϊ1

	SHT11_SCL_RESET;//scl=0
	SHT11_Delay(5);
	SHT11_SCL_SET;//scl =1
	
	SHT11_Delay(1);
/*-------------------0--------------*/
	SHT11_DATA =0;//����Ϊ0
	SHT11_SCL_RESET;//scl=0
	SHT11_Delay(5);
	SHT11_SCL_SET;//scl=1

	SHT11_DATA = 1;//SDA=1
	SHT11_SCL_RESET;//SCL=0
	
}

void SHT11_Reset(void)
{
	unsigned char i;
	SHT11_DATA_OUT;//���ģʽ
	SHT11_DATA=1;
	asm("NOP");
	SHT11_SCL_RESET;
	asm("NOP");
	for(i=0;i<9;i++)
	{
		SHT11_SCL_SET;
		asm("NOP");
		SHT11_SCL_RESET;
		asm("NOP");
	}
	SHT11_Start();//����SHT11
	
}

unsigned char SHT11_Write_Byte(unsigned char byte)
{
		unsigned char i;
		unsigned char status =1;//״̬
		SHT11_DATA_OUT;//�������ģʽ
		for(i=0;i<8;i++)
		{
		//һ���ֽ��ɸ�λ����λд��SHT11
		if((byte & 0x80)==0x80) 
			SHT11_DATA =1;
		else
			SHT11_DATA =0;
		//����ʱ����,��ʱ5US,����д��
		SHT11_SCL_SET;
		SHT11_Delay(5);
		SHT11_SCL_RESET;
		byte <<=1;
		}
		SHT11_DATA =1;//�ͷ�������
		SHT11_DATA_IN;//����ģʽ��δ����׼��
		SHT11_SCL_SET;//scl=1
		status = SHT11_DATA;//����Ӧ��
		//Ĭ��״̬
		SHT11_SCL_RESET;//����
		if(status == 0)
			{
			return SUCCESS_1;
			}
		else
			{
			return ERROR;
			}		
}

char SHT11_Read_Byte(unsigned char ack_status)
{
	unsigned char i;
	unsigned char byte =0;
	SHT11_DATA_IN;//����ģʽ
	for(i=0;i<8;i++)
	{
	byte <<=1;//����λ��1λ
	SHT11_SCL_SET;//scl=1
	SHT11_Delay(5);

	if(SHT11_DATA)
		{
		byte |=0x01;
		}
	SHT11_SCL_RESET;//SCL=0
	}

	SHT11_DATA_OUT;//���ģʽ
	SHT11_DATA = !ack_status;//�Ƿ�ҪӦ��
	//����ʱ�ӵȴ�Ӧ��
	SHT11_SCL_SET;
	SHT11_Delay(5);
	SHT11_SCL_RESET;

	SHT11_DATA =1;//�ͷ�������
	return byte;
}

/*MODE ѡ��ɼ�ʪ�Ȼ����¶�
*/
unsigned char SHT11_Order(unsigned char mode)
{
	unsigned char status;//success or error
	unsigned char i;
	unsigned char count = 0;
	if(mode == MODE_TEMP)
		{
		status = SHT11_Write_Byte(SHT11_TEMP);//����¶�
		}
	else if(mode == MODE_HUMI)
		{
		status = SHT11_Write_Byte(SHT11_HUMI);//���ʪ��
		}
	if(status == SUCCESS_1)
		{
		/*д������ɹ�*/
		while(SHT11_DATA)
			{
			for(i=0;i<200;i++)
				{
				SHT11_Delay(250);
				SHT11_Delay(250);
				}//100ms
				count++;
			if(count >=20)	
				{
				status = ERROR;
				break;
				//����2Sδ��Ӧ
				}
			}
		}
	else
		{
		SHT11_Reset();
		}
	return status;
}

void SHT11_Get_Data(void)
{
	unsigned int value =0;
	SHT11_Reset();//��λ
	if(SHT11_Order(MODE_HUMI)==SUCCESS_1)
		{
		SHT11_Delay(250);
		*((unsigned char *)&value+1) = SHT11_Read_Byte(ACK);//��8λ
		*((unsigned char *)&value) = SHT11_Read_Byte(ACK);//��8λ
		SHT11_Read_Byte(noACK);//У���
		}
	SHT11.Humi = value;//��¼ʪ��
	value =0;
	
	SHT11_Reset();
	SHT11_Delay_Ms(200);
	if (SHT11_Order(MODE_TEMP)==SUCCESS_1)
		{
		SHT11_Delay(250);
		*((unsigned char *)&value+1) = SHT11_Read_Byte(ACK);//��8λ
		*((unsigned char *)&value) = SHT11_Read_Byte(ACK);//��8λ
		SHT11_Read_Byte(noACK);//У���
		}
	SHT11.Temp = value;
}

void SHT11_Real_Data(void)
{
	SHT11_Get_Data();//��ȡ���¶Ⱥ�ʪ������
	const float C1 = -2.0468;//
	const float C2 = +0.0367;
	const float C3 = -0.0000015955;
	const float T1 = +0.01;
	const float T2 = +0.00008;

	float rh = SHT11.Humi;//12λʪ��
	float t = SHT11.Temp;//14Ϊ�¶�
	float rh_lin;//�¶ȼ���ֵ
	float rh_true;//��ʵֵ
	float t_C;

	//t_C = t*0.01-40.1;//�����ѹ5V
        t_C = t*0.01-39.6;//�����ѹ3.3V
        if(t_C<15)
          t_C=0;
        else
          t_C=t_C;
	rh_lin = C3*rh*rh+C2*rh+C1;//���ʪ�Ȳ���
	rh_true=(t_C-25)*(T1+T2*rh)+rh_lin;//���ʪ�Ȳ���
	if(rh_true>100)
		rh_true=100;
	if(rh_true<0.1)
		rh_true=0.1;
	#ifndef SHT11_DATA_FLOAT
	SHT11.Temp = (int)t_C;//�����¶�
	SHT11.Humi = (int)rh_true;//����ʪ��
	#else
	SHT11.Temp = t_C;  //�����¶�
	SHT11.Humi = rh_true;//����ʪ��
	#endif
	
}

void SHT11_Finish(void)
{
#ifdef SHT11_DATA_FLOAT
	unsigned int temp;//�м����
#endif
	if(flag == 0)
		{
                //ִ��һ�γ�ʼ��
		SHT11_Init();
		flag =1;
		}
	SHT11_Real_Data();//��ȡ����
	SHT11_Delay_Ms(100);
#ifndef SHT11_DATA_FLOAT
/*--------�¶�ת���ֽ�------------*/
#ifdef SHT11_DATA_BYTE
      SHT11.Temp_byte[0]=(SHT11.Temp>>8)&0xff;
      SHT11.Temp_byte[1]=SHT11.Temp&0xff;
#else
/*--------�¶�ת���ַ�------------*/
if(SHT11.Temp/100 !=0)
	{
	SHT11.Temp_s[0]=SHT11.Temp/100+0x30;//��λ
	SHT11.Temp_s[1]=SHT11.Temp%100/10+0x30;
	SHT11.Temp_s[2]=SHT11.Temp%10+0x30;
	SHT11.Temp_s[3]='\0';
	SHT11.length_temp=3;
	}
else
	{
	if(SHT11.Temp%100/10 !=0)//ʮλ��Ϊ0
		{
		SHT11.Temp_s[0]=SHT11.Temp/10+0x30;//ʮλ
		SHT11.Temp_s[1]=SHT11.Temp%10+0x30;//��λ
		SHT11.Temp_s[2]='\0';
		SHT11.length_temp=2;
		}
	else
		{
		SHT11.Temp_s[0]=SHT11.Temp+0x30;//��λ
		SHT11.Temp_s[1]='\0';
		SHT11.length_temp=1;
		}
	
	}
#endif
/*--------ʪ��ת���ֽ�------------*/
#ifdef SHT11_DATA_BYTE
      SHT11.Humi_byte[0]=(SHT11.Humi>>8)&0xff;
      SHT11.Humi_byte[1]=SHT11.Humi&0xff;
#else
/*--------------ʪ��ת���ַ���-------------*/
	if(SHT11.Humi/100 !=0)
		{
			SHT11.Humi_s[0]=SHT11.Humi/100+0x30;//��λ
			SHT11.Humi_s[1]=SHT11.Humi%100/10+0x30;
			SHT11.Humi_s[2]=SHT11.Humi%10+0x30;
			SHT11.Humi_s[3]='\0';
			SHT11.length_humi=3;
		}
	else
		{
			if(SHT11.Humi%100/10 !=0)
				{
				SHT11.Humi_s[0]=SHT11.Humi/10+0x30;
				SHT11.Humi_s[1]=SHT11.Humi%10+0x30;
				SHT11.Humi_s[3]='\0';
				SHT11.length_humi=2;
				}
			else
				{
				SHT11.Humi_s[0]=SHT11.Humi+0X30;
				SHT11.Humi_s[1]='\0';
				SHT11.length_humi=1;
				}
		}
#endif
#else
//�����˾��ȣ���¼��С������ֵ
/*--------�¶�ת���ַ�------------*/
	temp = (unsigned int)(SHT11.Temp*10);//��¼С������һλ
#ifdef SHT11_DATA_BYTE
        SHT11.Temp_byte[0]=(temp>>8)&0xff;
        SHT11.Temp_byte[1]=temp&0xff;
#else
	if(temp/1000 !=0)//��λ��Ϊ0
		{
		SHT11.Temp_s[0]=temp/1000+0x30;//��λ
		SHT11.Temp_s[1]=temp%1000/100+0x30;//ʮλ
		SHT11.Temp_s[2]=temp%100/10+0x30;//��λ
		SHT11.Temp_s[3]=0x2E;
		SHT11.Temp_s[4]=temp%10+0x30;//С����һλ
		SHT11.Temp_s[5]='\0';
		SHT11.length_temp=5;
		}
	else
	{
		if(temp/100 !=0)//ʮλ��Ϊ0
		{
		SHT11.Temp_s[0]=temp/100+0x30;//ʮλ
		SHT11.Temp_s[1]=temp%100/10+0x30;//��λ
		SHT11.Temp_s[2]=0x2E;
		SHT11.Temp_s[3]=temp%10+0x30;//С����һλ
		SHT11.Temp_s[4]='\0';
		SHT11.length_temp=4;
		}
		else
		{
		SHT11.Temp_s[0]=temp/10+0x30;//��λ
		SHT11.Temp_s[1]=0x2E;
		SHT11.Temp_s[2]=temp%10+0x30;//С����һλ
		SHT11.Temp_s[3]='\0';
		SHT11.length_temp=3;
		}
	}
#endif
/*--------------ʪ��ת���ַ���-------------*/
	temp=(unsigned int)(SHT11.Humi*10);
#ifdef SHT11_DATA_BYTE
        SHT11.Humi_byte[0]=(temp>>8)&0xff;
        SHT11.Humi_byte[1]=temp&0xff;
#else
	if(temp/1000 !=0)
		{
		SHT11.Humi_s[0]=temp/1000+0x30;//��λ
		SHT11.Humi_s[1]=temp%1000/100+0x30;//ʮλ
		SHT11.Humi_s[2]=temp%100/10+0x30;//��λ
		SHT11.Humi_s[3]=0x2E;
		SHT11.Humi_s[4]=temp%10+0x30;//С����һλ
		SHT11.Humi_s[5]='\0';
		SHT11.length_humi=5;
		}
	else
		{
		if(temp/100 !=0)//ʮλ��Ϊ0
			{
			SHT11.Humi_s[0]=temp/100+0x30;//ʮλ
			SHT11.Humi_s[1]=temp%100/10+0x30;//��λ
			SHT11.Humi_s[2]=0x2E;
			SHT11.Humi_s[3]=temp%10+0x30;//С����һλ
			SHT11.Humi_s[4]='\0';
			SHT11.length_humi=4;
			}
		else
			{
			SHT11.Humi_s[0]=temp/10+0x30;//��λ
			SHT11.Humi_s[1]=0x2E;
			SHT11.Humi_s[2]=temp%10+0x30;//С����һλ
			SHT11.Humi_s[3]='\0';
			SHT11.length_humi=3;
			}
		}
#endif
#endif
}