#ifndef _WM1110_GEOLOCATION_H_
#define _WM1110_GEOLOCATION_H_

#pragma once

#include <Arduino.h>

#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdint.h>


#include <LbmWm1110.hpp>
#include <Lbmx.hpp>
#include <Lbm_Modem_Common.hpp>

#include <WM1110_BLE.hpp>


#if ( WM1110_GEOLOCATION_DBG_ENABLE == 1)
#define WM1110_GEOLOCATION_DBG_PRINTF( ... )  printf (  __VA_ARGS__ )
#else
#define WM1110_GEOLOCATION_DBG_PRINTF( ... )  
#endif

#define TRACKER_SW_MAJOR_VER    (0)
#define TRACKER_SW_MINOR_VER    (5)
#define TRACKER_HW_MAJOR_VER    (1)
#define TRACKER_HW_MINOR_VER    (0)


#define WIFI_AP_RSSI_EN         1
#define WIFI_AP_ADDRESS_SIZE    ( 6 )
#define WIFI_AP_RSSI_SIZE       ( 1 )
#define APP_TASK_LORA_TX_QUEUE_MAX  48 
#define LORAWAN_APP_DATA_MAX_SIZE 242       //Maximum lora data length


/*--------------------------- uplink ---------------------------*/
#define DATA_ID_UP_PACKET_GNSS_RAW          0x12
#define DATA_ID_UP_PACKET_GNSS_END          0x13
#define DATA_ID_UP_PACKET_WIFI_RAW          0x14
#define DATA_ID_UP_PACKET_BLE_RAW           0x15
#define DATA_ID_UP_PACKET_POS_STATUS        0x16

#define DATA_ID_UP_PACKET_USER_SENSOR       0x17
#define DATA_ID_UP_PACKET_FACT_SENSOR       0x18
#define DATA_ID_UP_PACKET_BOOT_DATA         0x19
/*--------------------------- downlink ---------------------------*/
#define DATA_ID_DOWN_PACKET_USER_CODE       0x8E


#define WM1110_RADIO_OTHER_BUSY         (1<<0)
#define WM1110_RADIO_GNSS_BUSY          (1<<1)  
#define WM1110_RADIO_WIFI_BUSY          (1<<2)

//event state
#define TRACKER_STATE_BIT2_MOT_BEG          0x0001
#define TRACKER_STATE_BIT3_MOT_END          0x0002
#define TRACKER_STATE_BIT4_DEV_STATIC       0x0004
#define TRACKER_STATE_BIT5_DEV_SHOCK        0x0008

#define TRACKER_STATE_BIT0_SOS              0x0040
#define TRACKER_STATE_BIT1_USER             0x0080

enum  TrackScanType
{
    Track_Scan_Gps = 0,
    Track_Scan_Wifi = 1,
    Track_Scan_Ble = 4,
};


enum  TrackScanState
{
    Track_None,
    Track_Start,
    Track_Scaning,
    Track_End,
    Track_Stop,
};
enum class LoRaWANStateType
{
    Startup,
    Joining,
    Joined,
    TimeSynch,
    JoinFailed,
};

class WM1110_Geolocation : public LbmxEventHandlers
{
    public:  
        // Variables  
        uint8_t tracker_scan_status = Track_None;  
        bool time_sync_flag = false;

        wifi_mw_event_data_scan_done_t WM1110_wifi_results = {0};   // wifi scaned done datas
        gnss_mw_event_data_scan_done_t WM1110_gnss_results[GNSS_SCAN_GROUP_SIZE_MAX] = {0}; // gnss scaned done datas
        uint8_t gnss_results_index = 0 ;    // gnss scaned done datas index 
        WM1110_Beacons_t WM1110_ble_results[3]; // scaned ibeacon done datas
        uint8_t ble_results_index = 0 ;     // scaned ibeacon done datas index

        uint8_t track_scaned_data_buf[GNSS_SCAN_GROUP_SIZE_MAX][64] = {0};   // tracker scaned done datas      
        uint8_t track_scaned_data_len[GNSS_SCAN_GROUP_SIZE_MAX] = {0};  // tracker scaned done datas len         
    public:
        WM1110_Geolocation(void);  

        static WM1110_Geolocation& getInstance();

        void begin(TrackScanType track_type = Track_Scan_Gps,bool is_sensecap_platform = true);    

        /*!
        *  @brief  Run device operation
        */  
        void run(void);   

        /*!
        *  @brief  LBMX Running processes,it just ensure the normal running of LoRaWAN,User need control the positioning operation by self
        *  @return sleeptime
        */  
        uint32_t lbmxProcess( void );  

        /*!
        *  @brief  LBMX Running and periodically locate
        *  @return sleeptime   
        */  
        uint32_t trackProcess();  

        /*!
        *  @brief  Set up an aiding position for gnss to locate quickly
        *  @param laittude   
        *  @param longitude 
        */  
        void setGnssAidingPosition(float laittude,float longitude); 

        /*!
        *  @brief  Get the aiding position
        *  @param laittude   
        *  @param longitude 
        */    
        void getGnssAidingPosition(float *laittude,float *longitude);

        /*!
        *  @brief  Set the user-defined region for access to LoRaWAN network
        *  @param modem_region   
        */  
        void setCustomRegion(smtc_modem_region_t modem_region); 

        /*!
        *  @brief  Get the region access to the LoRaWAN network
        *  @return smtc_modem_region_t   
        */  
        smtc_modem_region_t getRegion( void );

        /*!
        *  @brief  Set the joining network code for access to LoRaWAN network
        *  @param devEui 
        *  @param joinEui 
        *  @param nwkKey  
        */  
        void setCustomJoinNetworkCode(uint8_t devEui[SMTC_MODEM_EUI_LENGTH], uint8_t joinEui[SMTC_MODEM_EUI_LENGTH], uint8_t nwkKey[SMTC_MODEM_KEY_LENGTH]);   // Set custom code for otaa
        
        /*!
        *  @brief  Get the joining network code that access to LoRaWAN network
        *  @param devEui 
        *  @param joinEui 
        *  @param nwkKey  
        */  
        void getJoinNetworkCode(uint8_t *devEui, uint8_t *joinEui, uint8_t *nwkKey);

        /*!
        *  @brief  Set the positioning period(unit minutes),must large than 3 minutes
        *  @param period_in_min 
        */ 
        void setCustomTrackPeriod( uint16_t period_in_min ); 

        /*!
        *  @brief  Get the positioning period(unit minutes)
        *  @return period_in_min 
        */ 
        uint16_t getTrackPeriod( void );       
 
        /*!
        *  @brief  Set the sensor measurement period(unit minutes),for uplink booting packet datas
        *  @param period_in_min 
        */ 
        void setSensorMeasurementPeriod( uint16_t period_in_min );

        /*!
        *  @brief  Get the sensor measurement period(unit minutes)
        *  @return period_in_min 
        */ 
        uint16_t getSensorMeasurementPeriod( void );

        /*!
        *  @brief  Set the positioning timeout(unit seconds)
        *  @param time_in_s 
        */ 
        void setTrackTimeout( uint16_t time_in_s );  

        /*!
        *  @brief  Get the positioning timeout(unit seconds)
        *  @return time_in_s 
        */ 
        uint16_t getTrackTimeout( void );   

        /*!
        *  @brief  Set up a port for uplink lora data
        *  @param lora_port 
        */ 
        void setUplinkPort(uint8_t lora_port);  

        /*!
        *  @brief  Get the uplink lora data port
        *  @return lora_port 
        */ 
        uint8_t getUplinkPort( void );  

        /*!
        *  @brief  Start a positioning scan
        *  @return bool  
        */ 
        bool startTrackerScan( void ); 

        /*!
        *  @brief  Stop a positioning scan
        */ 
        void stopTrackerScan( void ); 

        /*!
        *  @brief  Get positioning results
        *  @return bool true:success false :fail
        */ 
        bool getTrackResults(void);

        /*!
        *  @brief  Get positioning datas
        *  @param  buf
        *  @param  size  
        *  @return bool true:success false :fail
        */        
        bool getTrackDatas(uint8_t *buf, uint8_t *size);

        /*!
        *  @brief  Display the positioning raw datas
        */ 
        void displayTrackRawDatas( void );  // Display track raw data

        /*!
        *  @brief  Display the positioning datas
        */         
        void dispalyTrackDatas( void );  

        /*!
        *  @brief  Get the utc timestamp(unit seconds)
        *  @return timestamp
        */ 
        uint32_t getUtcTimestamp( void );

        /*!
        *  @brief  Insert packet datas to lora tx queue
        *  @param  buf
        *  @param  len          
        *  @return bool
        */ 
        bool insertIntoTxQueue( uint8_t *buf, uint8_t len ); 

        /*!
        *  @brief  Get current LoRaWAN running state
        *  @param  state      
        */ 
        void getLoRaWANRunningState(LoRaWANStateType *state);

        /*!
        *  @brief  Set positionging event state
        *  @param  state      
        */ 
        void setEventStateAll(uint32_t state);   
        
        /*!
        *  @brief  Get positionging event state
        *  @param  state      
        */ 
        void getEventStateAll(uint32_t *state); 

        /*!
        *  @brief  LoRaWAN runs process light action  
        */ 
        void modemLedActionProcess( void );

        /*!
        *  @brief  Insert positioning result datas to lora tx queue 
        */ 
        void insertTrackResultsIntoQueue( void );   

    private:
        static WM1110_Geolocation* instance_;
    
        // Constants
        static constexpr uint32_t TIME_SYNC_VALID_TIME = 60 * 60 * 24;  // [sec.]   in one day
        static constexpr uint32_t FIRST_UPLINK_DELAY = 20;  // [sec.] send data time after first joined
        static constexpr uint32_t UPLINK_PERIOD = 10;       // [sec.]   send data by period
        static constexpr uint8_t stack_id = 0;

        // Variables
        LbmWm1110& lbmWm1110 = LbmWm1110::getInstance();
        LoRaWANStateType lora_run_state = LoRaWANStateType::Startup;
        uint32_t led_flicker_start_time = 0;

        uint32_t track_state_all = 0;   //  event state
        uint32_t send_retry_type = 0;   
        uint8_t mw_gnss_event_state = 0;

        uint32_t track_period_time = 5*60*1000;   // [msec.]   // track period 
        uint32_t sensor_period_time = 5*60*1000;   // [msec.]  // read sensor period  
        uint32_t track_timeout = 2*60*1000; // [msec.]  //track timeout 
        
        uint8_t track_scan_type = Track_Scan_Gps;
        bool sensecap_platform  = true;
  
        uint32_t start_scan_timestamp = 0;

        uint8_t uplink_port = 5;  // LoRaWAN PORT for SenseCAP
        uint32_t data_tx_toa = 0;         
        bool check_tx_flag = false;

        bool tx_offline_data_flag = false;
        bool cache_offline_data_flag = true;
        uint32_t last_tx_timestamp = 0;

        uint32_t tx_confirmed_count_backup = 0;
        uint32_t tx_confirmed_count = 0;

        //Network access certificate
        uint8_t dev_eui[8]  = {0};
        uint8_t join_eui[8] = {0};
        uint8_t app_key[16] = {0};        
        smtc_modem_region_t region = SMTC_MODEM_REGION_EU_868;

        //aiding position
        float gnss_aiding_position_latitude = 22.576814;
        float gnss_aiding_position_longitude = 113.922068;

        //track result data
        uint8_t wifi_result_buffer[4+( WIFI_AP_RSSI_SIZE + WIFI_AP_ADDRESS_SIZE ) * WIFI_MAX_RESULTS + 4];

        //lora tx queue
        uint8_t tx_queue_data_len[APP_TASK_LORA_TX_QUEUE_MAX] = {0};
        bool tx_confirmed_queue_buf[APP_TASK_LORA_TX_QUEUE_MAX] = {0};
        uint8_t tx_queue_data_buf[APP_TASK_LORA_TX_QUEUE_MAX][LORAWAN_APP_DATA_MAX_SIZE] = {0};
        uint8_t tx_queue_in_index = 0;
        uint8_t tx_queue_out_index = 0;

        //lora rx buffer
        uint8_t rx_data_buf[LORAWAN_APP_DATA_MAX_SIZE] = {0};
        uint8_t rx_data_len = 0;
        
    private:

        bool getLbmxRadioBusyState(uint8_t *busy_type);

        // void insertTrackResultsIntoQueue( void );  

        void packTrackResults( void );  

        bool txProcess( void );

        void incrementTxConfirmedCount( void );

        uint32_t getTxConfirmedCount( void );

        bool cacheOfflineDatas( uint8_t *buf, uint8_t len );

        bool uplinkOfflineDatas( void ); 

        void insertBootingMessageIntoQueue( void );  

        smtc_modem_region_t getRegionFromDefault(void);

        void getDefaultProfileListByRegion(smtc_modem_region_t modem_region,uint8_t *buf);

        void printLbmVersion( ralf_t* ralf );

        bool sendFrame( uint8_t* buffer, uint8_t length, bool tx_confirmed, bool emergency );

        bool insertIntoTxQueue(  uint8_t *buf, uint8_t len, bool confirmed, bool emergency  );  
        
        void printGnssScanedDatas(const gnss_mw_event_data_scan_done_t* data);

        void restartClockSynchService( void );

        static void modemEventHandler();
        
    protected:

        //LoRaWAN 
        virtual void reset(const LbmxEvent& event);
        virtual void joined(const LbmxEvent& event);
        virtual void joinFail(const LbmxEvent& event);
        virtual void alarm(const LbmxEvent& event);
        virtual void time(const LbmxEvent& event);
        virtual void almanacUpdate(const LbmxEvent& event);
        virtual void txDone(const LbmxEvent& event);
        virtual void downData(const LbmxEvent& event);

        //gnss scan
        virtual void gnssScanDone(const LbmxEvent& event);
        virtual void gnssScanCancelled(const LbmxEvent& event);
        virtual void gnssTerminated(const LbmxEvent& event);
        virtual void gnssErrorNoTime(const LbmxEvent& event);
        virtual void gnssErrorAlmanacUpdate(const LbmxEvent& event);
        virtual void gnssScanStopped(const LbmxEvent& event);
        virtual void gnssErrorNoAidingPosition(const LbmxEvent& event);

        //wifi scan        
        virtual void wifiScanDone(const LbmxEvent& event);
        virtual void wifiTerminated(const LbmxEvent& event);
        virtual void wifiScanStopped(const LbmxEvent& event);
        virtual void wifiScanCancelled(const LbmxEvent& event);
        virtual void wifiErrorUnknown(const LbmxEvent& event);

        virtual void decodeDownlinkData( uint8_t *buf, uint8_t len );
};


#endif /* _WM1110_GEOLOCATION_H_ */
