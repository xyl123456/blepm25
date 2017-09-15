#ifndef TYPES_H
#define TYPES_H

//�豸��������
typedef union ConstructNet
{
   unsigned char data_buf[50];
   struct cn_data_t
   {
       unsigned char ATCGCLASS[16];   //AT+CGCLASS="B"\r\n
       unsigned char ATCGATT[12];   //AT+CGATT=1\r\n
       unsigned char ATCIPCSGP[22]; //AT+CIPCSGP=1,"CMNET"\r\n
   }data_core;
}UC_t;
//����ָ����ip��ַ��������
 typedef union DeviceConnect
 {
   unsigned char data_buf[79];
   struct dc_data_t
   {
     unsigned char ATPORT[24];   //AT+CLPORT="TCP","2000"\r\n
     unsigned char ATSTART[43];   //AT+CIPSTART="TCP","113.67.34.179","8086"\r\n
     unsigned char ATCIPSEND[12]; //AT+CIPSEND\r\n
   }data_core;
 }D_CN_t; 
 
//�鿴��ǰ��״̬
 typedef union Viewstate
{
   unsigned char data_buf[14];
   struct vs_data_t
   {
       unsigned char ATCGCLASS[14];   //AT+CIPSTATUS\r\n
   }data_core;
}VS_t;
//�ر�����
 typedef union Closeconnect
{
   unsigned char data_buf[27];
   struct cl_data_t
   {
       unsigned char ATCIPCLOSE[15];   //AT+CIPCLOSE=1\r\n
       unsigned char ATCIPSHUT[12];  //AT+CIPSHUT\r\n
   }data_core;
}CL_t;
//���͵�PWM����
 typedef union Senddata
{
   unsigned char data_buf[15];
   struct sd_data_t
   {
       unsigned char head[3];   //UP:
       unsigned char initial_data1[4]; //��һ��VOUT1 30S������
       unsigned char initial_data2[4]; //�ڶ���VOUT2DE 30S������
       unsigned char tail[2];    //:O
       unsigned char data_changeline[2];//\r\n
       //unsigned char send_dataline[1];  //0X1A
   }data_core;
}SD_t;
//�·�������
 typedef union Cmddata
{
   unsigned char data_buf[9];
   struct cd_data_t
   {
       unsigned char head[3];   //UD:
       unsigned char cmd_data[4]; //�ն��豸�Ŀ�������
       unsigned char tail[2];    //:O
   }data_core;
}CM_t;
//һ��ok����ȷ�Ͻṹ��
typedef union Anserdata
{
   unsigned char data_buf[6];
   struct An_data_t
   {
       unsigned char head[2];   //\r\n
       unsigned char cmd_data[2]; //ok
       unsigned char tail[2];    //\r\n
   }data_core;
}AN_t;


//���͵���ʪ������
 typedef union Tempdata
{
   unsigned char data_buf[13];
   struct Tem_data_t
   {
       unsigned char head[3];   //UP:
       unsigned char temp_data[2]; //��һ���¶ȵ�����
       unsigned char TEMP_C[1];//C
       unsigned char humi_data[2]; //�ڶ���ʪ�ȵ�����
       unsigned char HUMI_H[1];//ת���ַ�%%
       unsigned char tail[2];    //:O
       unsigned char data_changeline[2];//\r\n
       //unsigned char send_dataline[1];  //0X1A
   }data_core;
}TEMP_t;


/*************************************************************************
******************************xdxxxxxxxxx*********************************
***************************************************************************/
//��������
typedef union Head_up
{
   unsigned char data_buf[13];
   struct head_data_t
   {
       unsigned char Head_byte[2];//0xEB 0x90
       unsigned char Data_length[2];//0x00 0x0B
       unsigned char Data_type; //���ݵ�����0x01
       unsigned char MAC_addr[4];   //�豸��ַ 
       unsigned char Check_code[2];//У���룬��ȥ��ͷ�Ͱ�β�����ֽڵ��ۼӺ�
       unsigned char Tial[2];//0x0D 0x0A
   }data_core;
}Head_up_t;

//���ݵǼ�
typedef union Rejest_up
{
   unsigned char data_buf[22];
   struct rejest_data_t
   {
       unsigned char Head_byte[2];//0xEB 0x90
       unsigned char Data_length[2];//0x00 0x14
       unsigned char Data_type; //���ݵ�����0x02
       unsigned char MAC_addr[4];   //�豸��ַ��Ӳ��ID 
       unsigned char Version[3];//Э��汾�ţ�1.0.0λ0x01,0x00,0x00
       unsigned char HardVersion[3];//Ӳ��Э��汾�ţ�1.0.0λ0x01,0x00,0x00
       unsigned char Heart_time[3];//����ʱ����,Ĭ��30��
       unsigned char Check_code[2];//У���룬��ȥ��ͷ�Ͱ�β�����ֽڵ��ۼӺ�
       unsigned char Tial[2];//0x0D 0x0A
   }data_core;
}Rejest_up_t;

//���ݵǼǷ���
typedef union Rejest_down
{
   unsigned char data_buf[14];
   struct rejestdown_data_t
   {
       unsigned char Head_byte[2];//0xEB 0x90
       unsigned char Data_length[2];//0x00 0x0C
       unsigned char Data_type; //���ݵ�����0x03
       unsigned char MAC_addr[4];   //�豸��ַ��Ӳ��ID 
       unsigned char CMD_code; //�����ֶΣ������Ƿ�Ǽǳɹ����ظ��Ǽǵ�
       unsigned char Check_code[2];//У���룬��ȥ��ͷ�Ͱ�β�����ֽڵ��ۼӺ�
       unsigned char Tial[2];//0x0D 0x0A
   }data_core;
}Rejest_down_t;

//��������ϴ�,�ô���Ϊ�¶ȡ�ʪ�ȡ�PM2.5ֵ��PM03ֵ������
typedef union Data_up
{
   unsigned char data_buf[31];
   struct data_up_t
   {
       unsigned char Head_byte[2];//0xEB 0x90
       unsigned char Data_length[2];//0x00 0x17
       unsigned char Data_type; //���ݵ�����0x04
       unsigned char MAC_addr[4];   //�豸��ַ 
       unsigned char PM25[3];//��һ���ֽ�0x01,�����������ݣ�����Ϊ����
       unsigned char PM03[3];//��һ���ֽ�0x0E,�����������ݣ�����Ϊ����
       unsigned char TEM[3];//��һ���ֽ�0x03,������������,����Ϊ1000+����׼��16
       unsigned char HUM[3];//��һ���ֽ�0x04,������������,����Ϊ1000+����׼��16
       unsigned char POW[3];//��һ���ֽ�0x0A,�����������ݣ�����Ϊ�ٷֱ�
       unsigned char POW_STA[3];//��������Ϊ0X01�������Ϊ0x00
       unsigned char Check_code[2];//У���룬��ȥ��ͷ�Ͱ�β�����ֽڵ��ۼӺ�
       unsigned char Tial[2];//0x0D 0x0A
   }data_core;
}Data_up_t;

//�ն�ʱ��У��
typedef union Confirm_up
{
   unsigned char data_buf[13];
   struct confirm_up_t
   {
       unsigned char Head_byte[2];//0xEB 0x90
       unsigned char Data_length[2];//0x00 0x0B
       unsigned char Data_type; //���ݵ�����0x0B
       unsigned char MAC_addr[4];   //�豸��ַ 
       unsigned char Check_code[2];//У���룬��ȥ��ͷ�Ͱ�β�����ֽڵ��ۼӺ�
       unsigned char Tial[2];//0x0D 0x0A
   }data_core;
}Confirm_up_t;

//�·�����ʱ�����
typedef union Time_dp
{
   unsigned char data_buf[19];
   struct time_dp_t
   {
       unsigned char Head_byte[2];//0xEB 0x90
       unsigned char Data_length[2];//0x00 0x13
       unsigned char Data_type; //���ݵ�����0x0C
       unsigned char MAC_addr[4];   //�豸��ַ 
       unsigned char Time_cmd[6];//ʱ���������
       unsigned char Check_code[2];//У���룬��ȥ��ͷ�Ͱ�β�����ֽڵ��ۼӺ�
       unsigned char Tial[2];//0x0D 0x0A
   }data_core;
}Time_dp_t;

//д���豸ID
typedef union Dev_dp
{
   unsigned char data_buf[13];
   struct dev_dp_t
   {
       unsigned char Head_byte[2];//0xEB 0x90
       unsigned char Data_length[2];//0x00 0x0B
       unsigned char Data_type; //���ݵ�����0xFD
       unsigned char MAC_addr[4];   //�豸��ַ,��λ��ǰ��λ�ں�
       unsigned char Check_code[2];//У���룬��ȥ��ͷ�Ͱ�β�����ֽڵ��ۼӺ�
       unsigned char Tial[2];//0x0D 0x0A
   }data_core;
}Dev_dp_t;

//д���豸ID����
typedef union Dev_up
{
   unsigned char data_buf[13];
   struct dec_up_t
   {
       unsigned char Head_byte[2];//0xEB 0x90
       unsigned char Data_length[2];//0x00 0x0D
       unsigned char Data_type; //���ݵ�����0xFE
       unsigned char MAC_addr[4];   //�豸��ַ 
       unsigned char Check_code[2];//У���룬��ȥ��ͷ�Ͱ�β�����ֽڵ��ۼӺ�
       unsigned char Tial[2];//0x0D 0x0A
   }data_core;
}Dev_up_t;


//��������
typedef union Dev_control
{
   unsigned char data_buf[24];
   struct dec_ctl_t
   {
       unsigned char Head_byte[2];//0xEB 0x90
       unsigned char Data_length[2];//0x00 0x18
       unsigned char Data_type; //���ݵ�����0x05
       unsigned char MAC_addr[4];   //�豸��ַ 
       unsigned char Serial_code[8];//������ˮ��
       unsigned char Cmd_code[3];//�����֣����� 81 00 0A ���ػ�81 00 0B
       unsigned char Check_code[2];//У���룬��ȥ��ͷ�Ͱ�β�����ֽڵ��ۼӺ�
       unsigned char Tial[2];//0x0D 0x0A
   }data_core;
}Dev_control_t;

//����������Ӧ
typedef union Dev_control_res
{
   unsigned char data_buf[24];
   struct dec_ctlres_t
   {
       unsigned char Head_byte[2];//0xEB 0x90
       unsigned char Data_length[2];//0x00 0x18
       unsigned char Data_type; //���ݵ�����0x06
       unsigned char MAC_addr[4];   //�豸��ַ 
       unsigned char Serial_code[8];//������ˮ��
       unsigned char Cmd_code[3];//�����֣����� 81 00 0A ���ػ�81 00 0B
       unsigned char Check_code[2];//У���룬��ȥ��ͷ�Ͱ�β�����ֽڵ��ۼӺ�
       unsigned char Tial[2];//0x0D 0x0A
   }data_core;
}Dev_control_res_t;

//����PM2.5����
typedef union PM25_up
{
   unsigned char data_buf[32];
   struct pm25_up_t
   {
       unsigned char Head_byte[2];//0x42 0x4D
       unsigned char Data_length[2];//0x00 0x1c
       unsigned char pm1_bz[2]; //pm1.0��׼������ug/m3
       unsigned char pm25_bz[2];   //pm2.5��׼������ug/m3
       unsigned char pm10_bz[2];   //pm10��׼������ug/m3
       unsigned char pm1_dq[2]; //pm1.0��������ug/m3
       unsigned char pm25_dq[2];   //pm2.5��������ug/m3
       unsigned char pm10_dq[2];   //pm10��������ug/m3
       unsigned char pm03_kq[2]; //pm0.3 0.1������������
       unsigned char pm05_kq[2]; //pm0.5 0.1������������
       unsigned char pm1_kq[2]; //pm1.0 0.1������������
       unsigned char pm25_kq[2];//pm2.5 0.1������������
       unsigned char pm5_kq[2];//pm5 0.1������������
       unsigned char pm10_kq[2];//pm10 0.1������������
       unsigned char bb_code;  //�汾��0x91
       unsigned char error_code;  //������0x00
       unsigned char Check_code[2];//У���룬��ȥ��ͷ�Ͱ�β�����ֽڵ��ۼӺ�
   }data_core;
}PM25_up_t;

#endif