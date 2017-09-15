#ifndef TYPES_H
#define TYPES_H

//设备建立连接
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
//发送指定的ip地址建立连接
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
 
//查看当前的状态
 typedef union Viewstate
{
   unsigned char data_buf[14];
   struct vs_data_t
   {
       unsigned char ATCGCLASS[14];   //AT+CIPSTATUS\r\n
   }data_core;
}VS_t;
//关闭连接
 typedef union Closeconnect
{
   unsigned char data_buf[27];
   struct cl_data_t
   {
       unsigned char ATCIPCLOSE[15];   //AT+CIPCLOSE=1\r\n
       unsigned char ATCIPSHUT[12];  //AT+CIPSHUT\r\n
   }data_core;
}CL_t;
//发送的PWM数据
 typedef union Senddata
{
   unsigned char data_buf[15];
   struct sd_data_t
   {
       unsigned char head[3];   //UP:
       unsigned char initial_data1[4]; //第一个VOUT1 30S的数据
       unsigned char initial_data2[4]; //第二个VOUT2DE 30S的数据
       unsigned char tail[2];    //:O
       unsigned char data_changeline[2];//\r\n
       //unsigned char send_dataline[1];  //0X1A
   }data_core;
}SD_t;
//下发的命令
 typedef union Cmddata
{
   unsigned char data_buf[9];
   struct cd_data_t
   {
       unsigned char head[3];   //UD:
       unsigned char cmd_data[4]; //终端设备的控制命令
       unsigned char tail[2];    //:O
   }data_core;
}CM_t;
//一般ok命令确认结构体
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


//发送的温湿度数据
 typedef union Tempdata
{
   unsigned char data_buf[13];
   struct Tem_data_t
   {
       unsigned char head[3];   //UP:
       unsigned char temp_data[2]; //第一个温度的数据
       unsigned char TEMP_C[1];//C
       unsigned char humi_data[2]; //第二个湿度的数据
       unsigned char HUMI_H[1];//转义字符%%
       unsigned char tail[2];    //:O
       unsigned char data_changeline[2];//\r\n
       //unsigned char send_dataline[1];  //0X1A
   }data_core;
}TEMP_t;


/*************************************************************************
******************************xdxxxxxxxxx*********************************
***************************************************************************/
//心跳命令
typedef union Head_up
{
   unsigned char data_buf[13];
   struct head_data_t
   {
       unsigned char Head_byte[2];//0xEB 0x90
       unsigned char Data_length[2];//0x00 0x0B
       unsigned char Data_type; //数据的类型0x01
       unsigned char MAC_addr[4];   //设备地址 
       unsigned char Check_code[2];//校验码，除去包头和包尾所有字节的累加和
       unsigned char Tial[2];//0x0D 0x0A
   }data_core;
}Head_up_t;

//身份登记
typedef union Rejest_up
{
   unsigned char data_buf[22];
   struct rejest_data_t
   {
       unsigned char Head_byte[2];//0xEB 0x90
       unsigned char Data_length[2];//0x00 0x14
       unsigned char Data_type; //数据的类型0x02
       unsigned char MAC_addr[4];   //设备地址，硬件ID 
       unsigned char Version[3];//协议版本号，1.0.0位0x01,0x00,0x00
       unsigned char HardVersion[3];//硬件协议版本号，1.0.0位0x01,0x00,0x00
       unsigned char Heart_time[3];//心跳时间间隔,默认30秒
       unsigned char Check_code[2];//校验码，除去包头和包尾所有字节的累加和
       unsigned char Tial[2];//0x0D 0x0A
   }data_core;
}Rejest_up_t;

//身份登记返回
typedef union Rejest_down
{
   unsigned char data_buf[14];
   struct rejestdown_data_t
   {
       unsigned char Head_byte[2];//0xEB 0x90
       unsigned char Data_length[2];//0x00 0x0C
       unsigned char Data_type; //数据的类型0x03
       unsigned char MAC_addr[4];   //设备地址，硬件ID 
       unsigned char CMD_code; //命令字段，解释是否登记成功，重复登记等
       unsigned char Check_code[2];//校验码，除去包头和包尾所有字节的累加和
       unsigned char Tial[2];//0x0D 0x0A
   }data_core;
}Rejest_down_t;

//监测数据上传,该代码为温度、湿度、PM2.5值、PM03值、电量
typedef union Data_up
{
   unsigned char data_buf[31];
   struct data_up_t
   {
       unsigned char Head_byte[2];//0xEB 0x90
       unsigned char Data_length[2];//0x00 0x17
       unsigned char Data_type; //数据的类型0x04
       unsigned char MAC_addr[4];   //设备地址 
       unsigned char PM25[3];//第一个字节0x01,后两个是数据，数据为整数
       unsigned char PM03[3];//第一个字节0x0E,后两个是数据，数据为整数
       unsigned char TEM[3];//第一个字节0x03,后两个是数据,数据为1000+数据准换16
       unsigned char HUM[3];//第一个字节0x04,后两个是数据,数据为1000+数据准换16
       unsigned char POW[3];//第一个字节0x0A,后两个是数据，数据为百分比
       unsigned char POW_STA[3];//如果充电则为0X01，不充电为0x00
       unsigned char Check_code[2];//校验码，除去包头和包尾所有字节的累加和
       unsigned char Tial[2];//0x0D 0x0A
   }data_core;
}Data_up_t;

//终端时间校对
typedef union Confirm_up
{
   unsigned char data_buf[13];
   struct confirm_up_t
   {
       unsigned char Head_byte[2];//0xEB 0x90
       unsigned char Data_length[2];//0x00 0x0B
       unsigned char Data_type; //数据的类型0x0B
       unsigned char MAC_addr[4];   //设备地址 
       unsigned char Check_code[2];//校验码，除去包头和包尾所有字节的累加和
       unsigned char Tial[2];//0x0D 0x0A
   }data_core;
}Confirm_up_t;

//下发命令时间更新
typedef union Time_dp
{
   unsigned char data_buf[19];
   struct time_dp_t
   {
       unsigned char Head_byte[2];//0xEB 0x90
       unsigned char Data_length[2];//0x00 0x13
       unsigned char Data_type; //数据的类型0x0C
       unsigned char MAC_addr[4];   //设备地址 
       unsigned char Time_cmd[6];//时间更新命令
       unsigned char Check_code[2];//校验码，除去包头和包尾所有字节的累加和
       unsigned char Tial[2];//0x0D 0x0A
   }data_core;
}Time_dp_t;

//写入设备ID
typedef union Dev_dp
{
   unsigned char data_buf[13];
   struct dev_dp_t
   {
       unsigned char Head_byte[2];//0xEB 0x90
       unsigned char Data_length[2];//0x00 0x0B
       unsigned char Data_type; //数据的类型0xFD
       unsigned char MAC_addr[4];   //设备地址,高位在前地位在后
       unsigned char Check_code[2];//校验码，除去包头和包尾所有字节的累加和
       unsigned char Tial[2];//0x0D 0x0A
   }data_core;
}Dev_dp_t;

//写入设备ID返回
typedef union Dev_up
{
   unsigned char data_buf[13];
   struct dec_up_t
   {
       unsigned char Head_byte[2];//0xEB 0x90
       unsigned char Data_length[2];//0x00 0x0D
       unsigned char Data_type; //数据的类型0xFE
       unsigned char MAC_addr[4];   //设备地址 
       unsigned char Check_code[2];//校验码，除去包头和包尾所有字节的累加和
       unsigned char Tial[2];//0x0D 0x0A
   }data_core;
}Dev_up_t;


//控制命令
typedef union Dev_control
{
   unsigned char data_buf[24];
   struct dec_ctl_t
   {
       unsigned char Head_byte[2];//0xEB 0x90
       unsigned char Data_length[2];//0x00 0x18
       unsigned char Data_type; //数据的类型0x05
       unsigned char MAC_addr[4];   //设备地址 
       unsigned char Serial_code[8];//命令流水号
       unsigned char Cmd_code[3];//命令字，开机 81 00 0A ，关机81 00 0B
       unsigned char Check_code[2];//校验码，除去包头和包尾所有字节的累加和
       unsigned char Tial[2];//0x0D 0x0A
   }data_core;
}Dev_control_t;

//控制命令响应
typedef union Dev_control_res
{
   unsigned char data_buf[24];
   struct dec_ctlres_t
   {
       unsigned char Head_byte[2];//0xEB 0x90
       unsigned char Data_length[2];//0x00 0x18
       unsigned char Data_type; //数据的类型0x06
       unsigned char MAC_addr[4];   //设备地址 
       unsigned char Serial_code[8];//命令流水号
       unsigned char Cmd_code[3];//命令字，开机 81 00 0A ，关机81 00 0B
       unsigned char Check_code[2];//校验码，除去包头和包尾所有字节的累加和
       unsigned char Tial[2];//0x0D 0x0A
   }data_core;
}Dev_control_res_t;

//激光PM2.5数据
typedef union PM25_up
{
   unsigned char data_buf[32];
   struct pm25_up_t
   {
       unsigned char Head_byte[2];//0x42 0x4D
       unsigned char Data_length[2];//0x00 0x1c
       unsigned char pm1_bz[2]; //pm1.0标准颗粒物ug/m3
       unsigned char pm25_bz[2];   //pm2.5标准颗粒物ug/m3
       unsigned char pm10_bz[2];   //pm10标准颗粒物ug/m3
       unsigned char pm1_dq[2]; //pm1.0大气环境ug/m3
       unsigned char pm25_dq[2];   //pm2.5大气环境ug/m3
       unsigned char pm10_dq[2];   //pm10大气环境ug/m3
       unsigned char pm03_kq[2]; //pm0.3 0.1升空气颗粒数
       unsigned char pm05_kq[2]; //pm0.5 0.1升空气颗粒数
       unsigned char pm1_kq[2]; //pm1.0 0.1升空气颗粒数
       unsigned char pm25_kq[2];//pm2.5 0.1升空气颗粒数
       unsigned char pm5_kq[2];//pm5 0.1升空气颗粒数
       unsigned char pm10_kq[2];//pm10 0.1升空气颗粒数
       unsigned char bb_code;  //版本号0x91
       unsigned char error_code;  //错误码0x00
       unsigned char Check_code[2];//校验码，除去包头和包尾所有字节的累加和
   }data_core;
}PM25_up_t;

#endif