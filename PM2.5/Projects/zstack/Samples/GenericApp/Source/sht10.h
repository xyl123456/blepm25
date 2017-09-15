#ifndef  __SHT10_H__
#define  __SHT10_H__
#include <ioCC2530.h>
//#define SHT11_DATA_FLOAT  0x01 //��������
#define SHT11_DATA_BYTE   //�������ݸ�ʽ���ֽ�

#define SHT11_SCL      P0_7  //ʱ����
#define SHT11_DATA   P0_6  //������

#define  SHT11_Init()   { P0SEL &= ~0xc0;P0DIR |=0xc0; }//io�ڳ�ʼ��

#define SHT11_DATA_IN    P0DIR &= ~0x40;//����Ϊ����ģʽ
#define SHT11_DATA_OUT    P0DIR |=  0x40;//����Ϊ���ģʽ

#define SHT11_SCL_SET         SHT11_SCL=1
#define SHT11_SCL_RESET     SHT11_SCL=0

#define SHT11_TEMP    0x03    //���¶�
#define SHT11_HUMI    0x05    //��ʪ��
#define SHT11_READ    0x07   //��״̬�Ĵ���
#define SHT11_WRITE  0x06    //д״̬�Ĵ���
#define SHT11_RESET   0x1e   //�����λ

#define MODE_TEMP    0    //�¶�ģʽ
#define MODE_HUMI    1    //ʪ��ģʽ

#define  SUCCESS_1    1
#define  ERROR      2
#define  noACK      0
#define  ACK        1

typedef struct SHT11_DATA_STAUCT
{
  unsigned char Temp_byte[2];//�¶��ַ����ֽڱ�ʾ
  unsigned char Humi_byte[2];//ʪ���ַ����ֽڱ�ʾ
#ifndef SHT11_DATA_FLOAT
  int Temp;//�¶�
  int Humi;//ʪ��
  unsigned char Temp_s[4];//�¶��ַ���
  unsigned char Humi_s[4];//ʪ���ַ���
#else
  float  Temp;
  float  Humi;
  unsigned char Temp_s[6];//�¶��ַ���
  unsigned char Humi_s[6];//ʪ���ַ���
  #endif
  unsigned char length_temp;
  unsigned char length_humi;
}SHT11_DATA_STRUCT;

extern SHT11_DATA_STRUCT SHT11;

void SHT11_Delay(unsigned char us);
void SHT11_Delay_Ms(unsigned char ms);
void SHT11_Start(void);
void SHT11_Reset(void);
unsigned char SHT11_Write_Byte(unsigned char byte);
char SHT11_Read_Byte(unsigned char ack_status);
unsigned char SHT11_Order(unsigned char mode);
void SHT11_Get_Data(void);
void SHT11_Real_Data(void);

extern void SHT11_Finish(void);
#endif
