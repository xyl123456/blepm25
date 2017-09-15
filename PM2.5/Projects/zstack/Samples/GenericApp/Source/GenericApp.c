/**************************************************************************************************
  Filename:       GenericApp.c
  Revised:        $Date: 2009-03-18 15:56:27 -0700 (Wed, 18 Mar 2009) $
  Revision:       $Revision: 19453 $

  Description:    Generic Application (no Profile).


  Copyright 2004-2009 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED �AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE, 
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com. 
**************************************************************************************************/

/*********************************************************************
  This application isn't intended to do anything useful, it is
  intended to be a simple example of an application's structure.

  This application sends "Hello World" to another "Generic"
  application every 15 seconds.  The application will also
  receive "Hello World" packets.

  The "Hello World" messages are sent/received as MSG type message.

  This applications doesn't have a profile, so it handles everything
  directly - itself.

  Key control:
    SW1:
    SW2:  initiates end device binding
    SW3:
    SW4:  initiates a match description request
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"

#include "GenericApp.h"
#include "DebugTrace.h"

#if !defined( WIN32 )
  #include "OnBoard.h"
#endif

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_uart.h"
#include "hal_adc.h"
#include "OSAL_Nv.h"
#include "sht10.h"
#include "bh1750.h"
#include "types.h"

/*********************************************************************
 * MACROS
 */
#define HAL_LED_OFF   1
#define HAL_LED_ON    0

// This is the max byte count per OTA message.
#if !defined( SERIAL_APP_TX_MAX )
#define SERIAL_APP_TX_MAX  80
#endif

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// This list should be filled with Application specific Cluster IDs.
const cId_t GenericApp_ClusterList[GENERICAPP_MAX_CLUSTERS] =
{
  GENERICAPP_CLUSTERID
};

const SimpleDescriptionFormat_t GenericApp_SimpleDesc =
{
  GENERICAPP_ENDPOINT,              //  int Endpoint;
  GENERICAPP_PROFID,                //  uint16 AppProfId[2];
  GENERICAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  GENERICAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  GENERICAPP_FLAGS,                 //  int   AppFlags:4;
  GENERICAPP_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)GenericApp_ClusterList,  //  byte *pAppInClusterList;
  GENERICAPP_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)GenericApp_ClusterList   //  byte *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in GenericApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t GenericApp_epDesc;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
byte GenericApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // GenericApp_Init() is called.
devStates_t GenericApp_NwkState;


byte GenericApp_TransID;  // This is the unique message ID (counter)

afAddrType_t GenericApp_DstAddr;
byte RxBuf[SERIAL_APP_TX_MAX+1];


/*********************************************************************
 * �漰�ĺ���*/
void GenericApp_HandleCombineKeys(uint16 keys, uint8 keyCounts);
/*********************************************************************
 * APP design
 */
//����״̬
static uint8 running_state;
//CRCУ����
unsigned int Crc_tmp=0;
//����ʱ������
//static uint16 Hour_time_cnt=0;
//static uint8 Day_time_cnt=0;

static uint8 Min_time_cnt=0;
/*********************************************************************
 * �豸�ɼ�����
 */
//static unsigned char Cmd_type;//��������
static unsigned char Dev_mac[4]={0x00,0x00,0x00,0x00};//�豸ID��
#ifdef QTJC_DEV
static unsigned char TVOC_data[3];//��ȩ��ֵ
static unsigned char CO2_data[3];//co2��ֵ
#endif

#ifdef POWER_TEST
static unsigned char POW_data[3];//������ֵ
static unsigned char POW_sta[3];//��س��״̬ 
#endif

static unsigned char PM25_data[3]={0x01,0x00,0x00};//PM2.5��ֵ
static unsigned char PM03_data[3]={0x0E,0x00,0x00};//PM0.3��ֵ
static unsigned char TEM_data[3]={0x03,0x00,0x00};//�¶ȵ�ֵ
static unsigned char HUM_data[3]={0x04,0x00,0x00};//ʪ�ȵ�ֵ

static unsigned char Headline[2]={0xEB,0x90};
static unsigned char Tialline[2]={0x0D,0x0A};

static unsigned char Head_up_len[2]={0x00,0x0B};//�������ݳ���0X0B
static unsigned char Rejest_up_len[2]={0x00,0x14};//����ע���ϱ�����0X14�����ݵǼ�
static unsigned char Data_up_len[2]={0x00,0x1D};//�����ϱ�����0x1D
static unsigned char Confirm_up_len[2]={0x00,0x0B};//ʱ������ȷ��
//static unsigned char Time_dp_len[2]={0x00,0x15};
//static unsigned char Dev_dp_len[2]={0x00,0x0F};
static unsigned char Dev_up_len[2]={0x00,0x0B};//�豸IDд�뷵�س���
static unsigned char Soft_version[3]={0x01,0x00,0x00};//�����汾��
static unsigned char Hard_version[3]={0x01,0x00,0x00};//Ӳ���汾��
static unsigned char Time_set[3]={0x00,0x00,0x1E};//��������ʱ��30S
static unsigned char Start_dev[3]={0x81,0x00,0x0A};//��������
static unsigned char Stop_dev[3]={0x81,0x00,0x0B};//�ػ�����
//����PM2.5���⴫����ͷ��Ϣ
//static unsigned char PM25_Head[2]={0x42,0x4D};
//static unsigned char PM25_len[2]={0x00,0x1C};
static uint8 Receivebuf[40];//�豸IDд������
static uint8 Rece_len=0;//�����ֽ���


Head_up_t     Head_up;
Rejest_up_t   Rejest_up;
Data_up_t     Data_up;
Confirm_up_t  Confirm_up;
Time_dp_t     Time_dp;
Dev_dp_t      Dev_dp;
Dev_up_t      Dev_up;
PM25_up_t     Pm25_up;
Dev_control_t Dev_ctl;
Dev_control_res_t Dev_ctl_res;
/*********************************************************************
 * LOCAL FUNCTIONS
 */
void GenericApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
void GenericApp_HandleKeys( byte shift, byte keys );
void GenericApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void GenericApp_SendTheMessage( void );
void rxCB(uint8 port,uint8 event);//����0�ص�����
void rxCB1(uint8 port,uint8 event);//����1�ص�����
unsigned int Crc16(uint8 buf[],uint8 len);//crcУ�麯��
void Heart_up(void);//�����ϴ�����
void write_to_nvflash(uint8 buf[]);//���豸IDд�뵽NVflash
void read_from_nvflash(void);//��ȡ�豸ID��NVflash
void Regest_dev(void);//�豸ͨ���ϱ�
void Data_up_process(void);//�ɼ������ϱ�
void Get_gass(void);//��ȡ���崫����ֵ
void Get_PM25(void);//��ȡPM2.5
void Get_TEM_HUM();//��ȡ��ʪ��
void Confirm_up_process(void);//ʱ��У���ϴ�
void Dev_dp_process(uint8 buf[]);//д���豸ID
void Dev_up_process(uint8 buf[]);//�豸IDд���ϴ�
void Dev_control_process(uint8 buf[]);//���ػ�����

void Dev_heart_process (uint8 buf[]);//������������

static void Process_data(uint8 buf[],uint8 len);//������������
#ifdef POWER_TEST
void Get_power(void);
#endif

void Start_process(void);//������������
void Stop_process(void);//�ػ���������

void Sleep_cmd(void);//���ߴ�������

/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      GenericApp_Init
 *
 * @brief   Initialization function for the Generic App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void GenericApp_Init( byte task_id )
{
  running_state = 0;//ϵͳ����
  HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );//�ر�PM2.5 
  HalLedSet ( HAL_LED_3, HAL_LED_MODE_OFF );//�رյ�Դ
  
  halUARTCfg_t uartConfig;
  halUARTCfg_t uartConfig1;
  
  GenericApp_TaskID = task_id;
  GenericApp_NwkState = DEV_INIT;
  GenericApp_TransID = 0;

  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

  /*GenericApp_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  GenericApp_DstAddr.endPoint = 0;
  GenericApp_DstAddr.addr.shortAddr = 0;
  */

  // Fill out the endpoint description.
  GenericApp_epDesc.endPoint = GENERICAPP_ENDPOINT;
  GenericApp_epDesc.task_id = &GenericApp_TaskID;
  GenericApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&GenericApp_SimpleDesc;
  GenericApp_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &GenericApp_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( GenericApp_TaskID );
  //UART0,P02 P03
  uartConfig.configured           = TRUE;              // 2x30 don't care - see uart driver.
  uartConfig.baudRate             = HAL_UART_BR_9600;
  uartConfig.flowControl          = FALSE;
  uartConfig.flowControlThreshold = 64;   // 2x30 don't care - see uart driver.
  uartConfig.rx.maxBufSize        = 128;  // 2x30 don't care - see uart driver.
  uartConfig.tx.maxBufSize        = 128;  // 2x30 don't care - see uart driver.
  uartConfig.idleTimeout          = 6;    // 2x30 don't care - see uart driver.
  uartConfig.intEnable            = TRUE; // 2x30 don't care - see uart driver.
  uartConfig.callBackFunc         = rxCB;
  HalUARTOpen (0, &uartConfig); 
  //UART1 P04 P05
  uartConfig1.configured           = TRUE;              // 2x30 don't care - see uart driver.
  uartConfig1.baudRate             = HAL_UART_BR_9600;
  uartConfig1.flowControl          = FALSE;
  uartConfig1.flowControlThreshold = 64;   // 2x30 don't care - see uart driver.
  uartConfig1.rx.maxBufSize        = 128;  // 2x30 don't care - see uart driver.
  uartConfig1.tx.maxBufSize        = 128;  // 2x30 don't care - see uart driver.
  uartConfig1.idleTimeout          = 6;    // 2x30 don't care - see uart driver.
  uartConfig1.intEnable            = TRUE; // 2x30 don't care - see uart driver.
  uartConfig1.callBackFunc         = rxCB1;
  HalUARTOpen (1, &uartConfig1);
  
  read_from_nvflash();//��ȡNV�е��豸ID, to Dev_mac[3]
  // Update the display
  /*
 //ZDOע����Ϣ
  ZDO_RegisterForZDOMsg( GenericApp_TaskID, End_Device_Bind_rsp );
  ZDO_RegisterForZDOMsg( GenericApp_TaskID, Match_Desc_rsp );
  //osal_set_event(GenericApp_TaskID,GENERICAPP_HUMI_MSG_EVT);
  */
  /*********************************************************************
 * ������壬CO2��H2S,��ȩ���к�����
 */
#ifdef  QTJC_DEV 
  HalAdcSetReference (HAL_ADC_REF_AVDD);
  HalAdcInit();//��ʼ��ADC��HAL_ADC_DEC_064
#endif
#ifdef POWER_TEST
  HalAdcSetReference (HAL_ADC_REF_AVDD);
  HalAdcInit();//��ʼ��ADC��HAL_ADC_DEC_064
#endif
  Regest_dev();//�����ϱ�
  
  //�����������¼�,�����¼�
  /*
osal_start_timerEx( GenericApp_TaskID,
                        GENERICAPP_HEART_EVT,
                        GENERICAPP_SEND_MSG_TIMEOUT );
  */

    //�����������¼�,���ݲɼ��ϱ�
osal_start_timerEx( GenericApp_TaskID,
                        GENERICAPP_SEND_DATE_EVT,
                        GENERICAPP_SEND_DATA_TIMEOUT );
}

/*********************************************************************
 * @fn      GenericApp_ProcessEvent
 *
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
UINT16 GenericApp_ProcessEvent( byte task_id, UINT16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
 (void)task_id;  // Intentionally unreferenced parameter
 //GenericApp_TaskID = task_id;

    if ( events & SYS_EVENT_MSG )
    {
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( GenericApp_TaskID );
      while (MSGpkt)
      {
        switch ( MSGpkt->hdr.event ) 
        {
          case KEY_CHANGE:
            GenericApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;
          
        default:
          break;
        }
      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );
      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( GenericApp_TaskID );
    }
    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
    }
  //���������¼�
  if ( events & GENERICAPP_SEND_DATE_EVT )
  {
    //�޸Ļ�ȡPM2.5��ʪ��
    //��ȡTVOC��CO2
    //Get_gass();
    //HalUARTWrite(0,Pm25_up.data_buf, 32);
    Get_PM25();
    Get_TEM_HUM();
#ifdef POWER_TEST
    Get_power();
#endif
    if(running_state==1)
    {
      Data_up_process();//��������
      running_state=0;
      Min_time_cnt=0;
    }
    else
    {
      Min_time_cnt++;
    }
    if(Min_time_cnt==20)
    {//�����Ӻ�����
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );//�ر�PM2.5
      Min_time_cnt=0;
    }
    osal_start_timerEx( GenericApp_TaskID,
                        GENERICAPP_SEND_DATE_EVT,
                        GENERICAPP_SEND_DATA_TIMEOUT );
    return (events ^ GENERICAPP_SEND_DATE_EVT);
  }
   //�����¼�
 /*
 if ( events & GENERICAPP_HEART_EVT )
  {

    //ʱ�����úͼ���
    Hour_time_cnt++;
    if(Hour_time_cnt==2880)
    {
      Hour_time_cnt=0;
      Day_time_cnt++;
    }
    if(Day_time_cnt==30)
    {
      Day_time_cnt=0;
      Confirm_up_process();//30��ʱ��У��
    }
    Heart_up();

    
    if(running_state==1)
    {
      Min_time_cnt++;
    }
   if((Min_time_cnt==50))
   {
      //Sleep_cmd();//��������ģʽ
      Min_time_cnt=0;
      running_state=0;
   }
    osal_start_timerEx( GenericApp_TaskID,
                        GENERICAPP_HEART_EVT, 
                        GENERICAPP_SEND_MSG_TIMEOUT );
    return (events ^ GENERICAPP_HEART_EVT);
  }
*/
  return 0;
}

/*********************************************************************
 * Event Generation Functions
 */

/*********************************************************************
 * @fn      GenericApp_ProcessZDOMsgs()
 *
 * @brief   Process response messages
 *
 * @param   none
 *
 * @return  none
 */
void GenericApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg )
{
  switch ( inMsg->clusterID )
  {
    case End_Device_Bind_rsp:
      if ( ZDO_ParseBindRsp( inMsg ) == ZSuccess )
      {
        // Light LED
        HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
      }
#if defined(BLINK_LEDS)
      else
      {
        // Flash LED to show failure
        HalLedSet ( HAL_LED_4, HAL_LED_MODE_FLASH );
      }
#endif
      break;

    case Match_Desc_rsp:
      {
        ZDO_ActiveEndpointRsp_t *pRsp = ZDO_ParseEPListRsp( inMsg );
        if ( pRsp )
        {
          if ( pRsp->status == ZSuccess && pRsp->cnt )
          {
            GenericApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
            GenericApp_DstAddr.addr.shortAddr = pRsp->nwkAddr;
            // Take the first endpoint, Can be changed to search through endpoints
            GenericApp_DstAddr.endPoint = pRsp->epList[0];

            // Light LED
            HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
          }
          osal_mem_free( pRsp );
        }
      }
      break;
  }
}

/*********************************************************************
 * @fn      GenericApp_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_4
 *                 HAL_KEY_SW_3
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
void GenericApp_HandleKeys( byte shift, byte keys )
{
  zAddrType_t dstAddr;
  
  // Shift is used to make each button/switch dual purpose.
  if ( shift )
  {
    if ( keys & HAL_KEY_SW_1 )
    {
    }
    if ( keys & HAL_KEY_SW_2 )
    {
    }
    if ( keys & HAL_KEY_SW_3 )
    {
    }
    if ( keys & HAL_KEY_SW_6 )
    {

    }
  }
  else
  {
    if ( keys & HAL_KEY_SW_1 )
    {
    }

    if ( keys & HAL_KEY_SW_2 )
    {
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );

      // Initiate an End Device Bind Request for the mandatory endpoint
      dstAddr.addrMode = Addr16Bit;
      dstAddr.addr.shortAddr = 0x0000; // Coordinator
      ZDP_EndDeviceBindReq( &dstAddr, NLME_GetShortAddr(), 
                            GenericApp_epDesc.endPoint,
                            GENERICAPP_PROFID,
                            GENERICAPP_MAX_CLUSTERS, (cId_t *)GenericApp_ClusterList,
                            GENERICAPP_MAX_CLUSTERS, (cId_t *)GenericApp_ClusterList,
                            FALSE );
    }

    if ( keys & HAL_KEY_SW_3 )
    {
    }

    if ( keys & HAL_KEY_SW_4 )
    {
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );
      // Initiate a Match Description Request (Service Discovery)
      dstAddr.addrMode = AddrBroadcast;
      dstAddr.addr.shortAddr = NWK_BROADCAST_SHORTADDR;
      ZDP_MatchDescReq( &dstAddr, NWK_BROADCAST_SHORTADDR,
                        GENERICAPP_PROFID,
                        GENERICAPP_MAX_CLUSTERS, (cId_t *)GenericApp_ClusterList,
                        GENERICAPP_MAX_CLUSTERS, (cId_t *)GenericApp_ClusterList,
                        FALSE );
    }
  }
  if( keys & HAL_KEY_SW_6)
  {
    GenericApp_HandleCombineKeys(keys, halGetKeyCount());
  }
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      GenericApp_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
void GenericApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  
}

/*********************************************************************
 * @fn      GenericApp_SendTheMessage
 *
 * @brief   Send "the" message.
 *
 * @param   none
 *
 * @return  none
 */
void GenericApp_SendTheMessage( void )
{
  afAddrType_t P2P_DstAddr;
  P2P_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  P2P_DstAddr.endPoint = GENERICAPP_ENDPOINT;
  P2P_DstAddr.addr.shortAddr = 0xFFFF; //�ն˶̵�ַ��LCD������ʾ���˴������ն˶̵�ַ�Ϳ��Ե㲥�ˡ�

  if ( AF_DataRequest( &P2P_DstAddr, &GenericApp_epDesc,
                       GENERICAPP_CLUSTERID,
                       1,
                       RxBuf,
                       &GenericApp_TransID,
                       AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    // Successfully requested to be sent.
  }
  else
  {
    // Error occurred in request to send.
  }
}
void GenericApp_HandleCombineKeys(uint16 keys, uint8 keyCounts)
{
  if(keys & HAL_KEY_SW_6)
  {
    switch( keyCounts)
    {
    case 0:
       HalLedSet( HAL_LED_3, HAL_LED_MODE_OFF );
       HalLedSet( HAL_LED_4, HAL_LED_MODE_OFF );
       running_state=0;
      break;
    case 1:
        HalLedSet( HAL_LED_3, HAL_LED_MODE_ON );
        HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
        running_state=1;
      break;
    default: break;
    }
  }
}

static void rxCB(uint8 port,uint8 event)
{ 
  uint8 Uartbuf[40];//�����ַ�������
  uint8 uartlen=0;
  uartlen = HalUARTRead(0,Uartbuf,sizeof(Uartbuf));
  if(uartlen>0){
    if((Uartbuf[0]==0xEB)&&(Uartbuf[uartlen-1]==0x0A)){
      //��ʾһ֡���ݽ���,��������
      Rece_len=uartlen;
      osal_memcpy(Receivebuf,Uartbuf,Rece_len);
      Process_data(Receivebuf,Rece_len);
       Rece_len=0;
        osal_memset(Receivebuf,0,sizeof(Receivebuf));
        osal_memset(Uartbuf,0,sizeof(Uartbuf));
    }else{
      osal_memcpy(Receivebuf+Rece_len,Uartbuf,uartlen);
      Rece_len=Rece_len+uartlen;
      if(Uartbuf[uartlen-1]==0x0A){
        //��װ����,��������
        Process_data(Receivebuf,Rece_len);
        Rece_len=0;
        osal_memset(Uartbuf,0,sizeof(Uartbuf));
      }
    }
  }
}

static void Process_data(uint8 buf[],uint8 len)
{
  uint8 Datalen=len;
  uint8 Databuf[40];
  osal_memcpy(Databuf,buf,Datalen);
  if(Datalen==24)
     {
      //Dev_control_process(Databuf);
      }
  else if(Datalen==13)
     {
       if(Databuf[4]==0xFD)
       {
       //�豸ID�޸�����
            Dev_dp_process(Databuf);//�����·������豸ID
       }
       if(Databuf[4]==0x01)
       {
       //����APP����  
       Dev_heart_process(Databuf);
          //HalUARTWrite(0, "HELLO", 5);
       }
     }
        else {
            Datalen=0;
            osal_memset(Databuf,0,sizeof(Databuf));
        }
}

//����1�ص�����,���¼���PM2.5����
static void rxCB1(uint8 port,uint8 event)
{ 
  unsigned  char Uartbuf[40];//�����ַ�������
  uint8 uartlen=0;
  uartlen = HalUARTRead(1,Uartbuf,sizeof(Uartbuf));
    if((uartlen==32)&&(Uartbuf[0]==0x42))
      {
         osal_memcpy(Pm25_up.data_buf,Uartbuf,32);
         uartlen=0;
         osal_memset(Uartbuf,0,sizeof(Uartbuf));
      }else
      {
         uartlen=0;
         osal_memset(Uartbuf,0,sizeof(Uartbuf));
      }
}
unsigned int Crc16(uint8 buf[],uint8 len)
{
	unsigned int  i,j,c,crc;
	crc=0xFFFF;
	for(i=0;i<len;i++)
		{
		c=*(buf+i)&0xFF;
		crc^=c;
		for(j=0;j<8;j++)
			{
				if(crc & 0x0001)
					{
					crc>>=1;
					crc ^=0xA001;
					}
				else
					{
					crc >>=1;
					}
			}
		}
  return(crc);
}


void write_to_nvflash(uint8 buf[])
{
    osal_nv_item_init(ZCD_DEV_ADDRESS, 1, NULL);
    osal_nv_write(ZCD_DEV_ADDRESS, 0,1, buf);
}
void read_from_nvflash(void)
{
   osal_nv_item_init(ZCD_DEV_ADDRESS, 1, NULL);
   osal_nv_read(ZCD_DEV_ADDRESS, 0, 1, &Dev_mac[3]);
}
//�����¼�

void Heart_up(void)
{
  unsigned char chec_buf[2];
  uint16 chec_int=0;
  int i;
  osal_memcpy(Head_up.data_core.Head_byte,Headline,2);
  osal_memcpy(Head_up.data_core.Data_length,Head_up_len,2);
  Head_up.data_core.Data_type=0x01;
  osal_memcpy(Head_up.data_core.MAC_addr,Dev_mac,4);
  
  for(i=0;i<7;i++)
  {
    chec_int=chec_int+Head_up.data_buf[2+i];
  } 
  chec_buf[0]=chec_int>>8;
  chec_buf[1]=chec_int;
  osal_memcpy(Head_up.data_core.Check_code,chec_buf,2);
  osal_memcpy(Head_up.data_core.Tial,Tialline,2);
  HalUARTWrite(0, Head_up.data_buf, 13);
}
/*********************************************************************
 * �ϵ緢��ע����Ϣ
 */
void Regest_dev(void)
{
  unsigned char chec_buf[2];
  uint16 chec_int=0;
  int i;
  osal_memcpy(Rejest_up.data_core.Head_byte,Headline,2);
  osal_memcpy(Rejest_up.data_core.Data_length,Rejest_up_len,2);
  Rejest_up.data_core.Data_type=0x02;
  osal_memcpy(Rejest_up.data_core.MAC_addr,Dev_mac,4);
  osal_memcpy(Rejest_up.data_core.Version,Soft_version,3);
  osal_memcpy(Rejest_up.data_core.HardVersion,Hard_version,3);
  osal_memcpy(Rejest_up.data_core.Heart_time,Time_set,3);
  for(i=0;i<16;i++)
  {
    chec_int=chec_int+Rejest_up.data_buf[2+i];
  } 
  chec_buf[0]=chec_int>>8;
  chec_buf[1]=chec_int;
  osal_memcpy(Rejest_up.data_core.Check_code,chec_buf,2);
  osal_memcpy(Rejest_up.data_core.Tial,Tialline,2);
  HalUARTWrite(0, Rejest_up.data_buf, 22);
}
/*
�ɼ������ϱ�����
*/
void Data_up_process(void)
{
  unsigned char chec_buf[2];
  uint16 chec_int=0;
  int i;
  osal_memcpy(Data_up.data_core.Head_byte,Headline,2);
  osal_memcpy(Data_up.data_core.Data_length,Data_up_len,2);
  Data_up.data_core.Data_type=0x04;
  osal_memcpy(Data_up.data_core.MAC_addr,Dev_mac,4);
  osal_memcpy(Data_up.data_core.PM25,PM25_data,3);
  osal_memcpy(Data_up.data_core.PM03,PM03_data,3);
  osal_memcpy(Data_up.data_core.TEM,TEM_data,3);
  osal_memcpy(Data_up.data_core.HUM,HUM_data,3);
  osal_memcpy(Data_up.data_core.POW,POW_data,3);
  osal_memcpy(Data_up.data_core.POW_STA,POW_sta,3);
  for(i=0;i<25;i++)
  {
    chec_int=chec_int+Data_up.data_buf[2+i];
  } 
  chec_buf[0]=chec_int>>8;
  chec_buf[1]=chec_int;
  osal_memcpy(Data_up.data_core.Check_code,chec_buf,2);
  osal_memcpy(Data_up.data_core.Tial,Tialline,2);
  HalUARTWrite(0, Data_up.data_buf, 31);
}
#ifdef QTJC_DEV
void Get_gass(void)
{
  uint16 QT_Tmp = HalAdcRead (HAL_ADC_CHANNEL_1, HAL_ADC_RESOLUTION_12);
  uint8 Temp_H=QT_Tmp>>8;    //��λ��ǰ
  uint8 Temp_L=QT_Tmp&0x00ff;//��λ�ں�
  CO2_data[0]=0x07;
  CO2_data[1]=Temp_H;
  CO2_data[2]=Temp_L;
  TVOC_data[0]=0x08;
  TVOC_data[1]=Temp_H;
  TVOC_data[2]=Temp_L;
}
#endif

#ifdef POWER_TEST
void Get_power(void)
{
   POW_data[0]=0x0A;
   POW_data[1]=0x00;
   //uint16 POW_Tmp = HalAdcRead (HAL_ADC_CHANNEL_0, HAL_ADC_RESOLUTION_12);
   uint16 POW_Tmp = HalAdcRead (HAL_ADC_CHANNEL_0, HAL_ADC_RESOLUTION_12);

   if(POW_Tmp>0x0320)
   {
   POW_data[2]=0x64; //100%
   }
   if((POW_Tmp>0x02BC)&&(POW_Tmp<0x0320))
   {
   POW_data[2]=0x50; //80%
   }
   if((POW_Tmp>0x0258)&&(POW_Tmp<0x02BC))
   {
    POW_data[2]=0x3C; //60%
   }
   if((POW_Tmp>0x01F4)&&(POW_Tmp<0x0258))
   {
     POW_data[2]=0x28;//40%
   }
   if(POW_Tmp<0x01F4)
   {
     POW_data[2]=0x0A;//10%
   }
   
   /*
   uint8 POW_H=POW_Tmp>>8;    //��λ��ǰ
   uint8 POW_L=POW_Tmp&0x00ff;//��λ�ں�
   POW_data[0]=0x0A;
   POW_data[1]=POW_H;
   POW_data[2]=POW_L; 
   */
   //��Դ���״̬
   POW_sta[0]=0x0F;
   POW_sta[1]=0x00;
   if(POW_Tmp>0x0355)
   POW_sta[2]=0x01;
   else
   POW_sta[2]=0x00;
}
#endif

void Get_PM25(void)
{
 //��ȡPM25��PM03(1)��ֵ����ȡ���Ǵ����е�ֵ
   osal_memcpy(PM25_data+1,Pm25_up.data_core.pm25_dq,2);
   osal_memcpy(PM03_data+1,Pm25_up.data_core.pm1_dq,2);
}
void Get_TEM_HUM()
{
  SHT11_Finish();//��ȡ��ʪ��
  /*ֱ�ӻ�ȡֵ
  osal_memcpy(TEM_data+1,SHT11.Temp_byte,2);
  osal_memcpy(HUM_data+1,SHT11.Humi_byte,2);
  */
  //��Э�����ֵ��03 E8
  //uint16 data_sh=SHT11.Temp+0x03E6;
  uint16 data_sh=SHT11.Temp_byte[1]+0x03E7;
  TEM_data[1]=(data_sh>>8)&0xff;
  TEM_data[2]=data_sh&0xff;
  osal_memcpy(HUM_data+1,SHT11.Humi_byte,2);
  /*
  uint16 data_hum=SHT11.Humi;
  HUM_data[1]=(data_hum>>8)&0xff;
  HUM_data[2]=data_hum&0xff;
  */
}
void Confirm_up_process(void)
{
  unsigned char chec_buf[2];
  uint16 chec_int=0;
  int i;
  osal_memcpy(Confirm_up.data_core.Head_byte,Headline,2);
  osal_memcpy(Confirm_up.data_core.Data_length,Confirm_up_len,2);
  Confirm_up.data_core.Data_type=0x0B;
  osal_memcpy(Confirm_up.data_core.MAC_addr,Dev_mac,4);
  for(i=0;i<11;i++)
  {
    chec_int=chec_int+Confirm_up.data_buf[2+i];
  } 
  chec_buf[0]=chec_int>>8;
  chec_buf[1]=chec_int;
  osal_memcpy(Confirm_up.data_core.Check_code,chec_buf,2);
  osal_memcpy(Confirm_up.data_core.Tial,Tialline,2);
  HalUARTWrite(0, Confirm_up.data_buf, 23);
}
void Dev_dp_process(uint8 buf[])
{
  osal_memcpy(Dev_dp.data_buf,buf,13);
  unsigned char chec_buf[2];
  uint16 chec_int=0;
  int i;
    for(i=0;i<7;i++)
  {
    chec_int=chec_int + buf[2+i];
  } 
  chec_buf[0]=chec_int>>8;
  chec_buf[1]=chec_int;
  //�ж�У��λ
  if(osal_memcmp(Dev_dp.data_core.Check_code,chec_buf,2))
  {
    osal_memcpy(Dev_mac,Dev_dp.data_core.MAC_addr,4);
    write_to_nvflash(&Dev_mac[3]);
    Dev_up_process(Dev_mac);//���ص�ַ�޸���Ϣ��Ӧ
  }else{
    osal_memset(buf,0,sizeof(buf));
  }
}
//�޸ĵ�ַ��Ϣ����
void Dev_up_process(uint8 buf[])
{
  unsigned char chec_buf[2];
  uint16 chec_int=0;
  int i;
  osal_memcpy(Dev_up.data_core.Head_byte,Headline,2);
  osal_memcpy(Dev_up.data_core.Data_length,Dev_up_len,2);
  Dev_up.data_core.Data_type=0xFE;
  osal_memcpy(Dev_up.data_core.MAC_addr,buf,4);
    for(i=0;i<7;i++)
  {
    chec_int=chec_int + Dev_up.data_buf[2+i];
  } 
  chec_buf[0]=chec_int>>8;
  chec_buf[1]=chec_int;
  osal_memcpy(Dev_up.data_core.Check_code,chec_buf,2);
  osal_memcpy(Dev_up.data_core.Tial,Tialline,2);
  HalUARTWrite(0, Dev_up.data_buf, 13);
}
//�������ƿ�������
void Dev_control_process(uint8 buf[])
{
  osal_memcpy(Dev_ctl.data_buf,buf,24);
  unsigned char chec_buf[2];
  uint16 chec_int=0;
  uint8 cmd_state=0;//0��ʾ�ػ���1��ʾ����
  int i;
    for(i=0;i<18;i++)
  {
    chec_int=chec_int + buf[2+i];
  } 
  chec_buf[0]=chec_int>>8;
  chec_buf[1]=chec_int;
  //�ж�У��λ
  if(osal_memcmp(Dev_ctl.data_core.Check_code,chec_buf,2))
  {
    if(osal_memcmp(Dev_ctl.data_core.MAC_addr,Dev_mac,4))
    {
      if(osal_memcmp(Dev_ctl.data_core.Cmd_code,Start_dev,3))
      {
        //����
        cmd_state=1;
        HalLedSet ( HAL_LED_4, HAL_LED_MODE_ON );
        Start_process();
      }
      if(osal_memcmp(Dev_ctl.data_core.Cmd_code,Stop_dev,3))
      {
        //�ػ�
        cmd_state=0;
        HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );
        Stop_process();
      }
      //������Ϣ
      osal_memcpy(Dev_ctl_res.data_buf,buf,24);
      Dev_ctl_res.data_core.Data_type=0x06;
      HalUARTWrite(0, Dev_ctl_res.data_buf, 24);
      osal_memset(buf,0,sizeof(buf));
      if(cmd_state==1)
      {
     //��������
      Get_PM25();
      Get_TEM_HUM();
      #ifdef POWER_TEST
      Get_power();
      #endif
      Data_up_process(); 
      }
    }else
    {
      osal_memset(buf,0,sizeof(buf));
    }
  }else{
    osal_memset(buf,0,sizeof(buf));
  }
}

//������������
void Start_process(void)
{
  //�����������¼�,���ݲɼ��ϱ�
  osal_stop_timerEx( GenericApp_TaskID,GENERICAPP_SEND_DATE_EVT);
  osal_start_timerEx( GenericApp_TaskID,
                        GENERICAPP_SEND_DATE_EVT,
                        GENERICAPP_SEND_DATA_TIMEOUT );
}
//�ػ���������
void Stop_process(void)
{
  osal_stop_timerEx( GenericApp_TaskID,GENERICAPP_SEND_DATE_EVT);
}


//
void Dev_heart_process (uint8 buf[])
{           
  osal_memcpy(Head_up.data_buf,buf,13);
  unsigned char chec_buf[2];
  uint16 chec_int=0;
  int i;
    for(i=0;i<7;i++)
  {
    chec_int=chec_int + buf[2+i];
  } 
  chec_buf[0]=chec_int>>8;
  chec_buf[1]=chec_int;
  //�ж�У��λ
  if(osal_memcmp(Head_up.data_core.Check_code,chec_buf,2))
  {
    if(osal_memcmp(Head_up.data_core.MAC_addr,Dev_mac,4))
    {
      if(Head_up.data_core.Data_type==0x01)
      {
       HalLedSet ( HAL_LED_4, HAL_LED_MODE_ON );//����PM2.5
       // osal_stop_timerEx( GenericApp_TaskID,GENERICAPP_SEND_DATE_EVT);
        running_state=1; //�յ��������ݣ�ϵͳ����
      }
      else
      {
        HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );
        running_state=0;
      }
    }
    else
    {
      osal_memset(buf,0,sizeof(buf));
    }
  }else
  {
    osal_memset(buf,0,sizeof(buf));
  }
}

//���ߴ�������
void Sleep_cmd(void)
{
   if(running_state==1)
    {
      //ϵͳ����,������������
      //HalLedSet ( HAL_LED_3, HAL_LED_MODE_ON );//������Դ
     /* osal_start_timerEx( GenericApp_TaskID,
                        GENERICAPP_SEND_DATE_EVT,
                        GENERICAPP_SEND_DATA_TIMEOUT );
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_ON );//����PM2.5
      */
    }else
    {
        osal_stop_timerEx( GenericApp_TaskID,GENERICAPP_SEND_DATE_EVT);
        HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );//�ر�PM2.5

    }
}