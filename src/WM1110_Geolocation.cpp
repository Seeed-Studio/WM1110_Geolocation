#include "WM1110_Geolocation.hpp"

#include "WM1110_Storage.hpp"
#include "WM1110_Param_Var/WM1110_Param_Var.h"

#include "lbm/smtc_modem_hal/smtc_modem_hal.h"

#include <Lbm_Modem_Common.hpp>
#include "lbm/smtc_modem_core/radio_drivers/lr11xx_driver/src/lr11xx_driver_version.h"

#define TRACKER_GPS_EN      1
#define TRACKER_WIFI_EN     1
#define TRACKER_BLE_EN      1
#define GNSS_RAW_SCAN_SEND_MAX 2

// uint8_t adr_custom_list_eu868_default[16] = { 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5 }; // SF12,SF12,SF12,SF11,SF11,SF11,SF10,SF10,SF10,SF9,SF9,SF9,SF8,SF8,SF7,SF7
uint8_t adr_custom_list_us915_default[16] = { 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3 }; // SF9,SF9,SF9,SF9,SF9,SF8,SF8,SF8,SF8,SF8,SF7,SF7,SF7,SF7,SF7
uint8_t adr_custom_list_au915_default[16] = { 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5 }; // SF9,SF9,SF9,SF9,SF9,SF8,SF8,SF8,SF8,SF8,SF7,SF7,SF7,SF7,SF7
uint8_t adr_custom_list_as923_default[16] = { 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5 }; // SF9,SF9,SF9,SF9,SF9,SF8,SF8,SF8,SF8,SF8,SF7,SF7,SF7,SF7,SF7
uint8_t adr_custom_list_kr920_default[16] = { 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 5 }; // SF12,SF12,SF12,SF11,SF11,SF11,SF10,SF10,SF10,SF9,SF9,SF9,SF8,SF8,SF7,SF7
uint8_t adr_custom_list_in865_default[16] = { 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 5 }; // SF12,SF12,SF12,SF11,SF11,SF11,SF10,SF10,SF10,SF9,SF9,SF9,SF8,SF8,SF7,SF7
// uint8_t adr_custom_list_ru864_default[16] = { 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5 }; // SF12,SF12,SF12,SF11,SF11,SF11,SF10,SF10,SF10,SF9,SF9,SF9,SF8,SF8,SF7,SF7


uint8_t adr_custom_list_eu868_default[16] = { 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5 }; // SF9,SF9,SF9,SF9,SF9,SF8,SF8,SF8,SF8,SF8,SF7,SF7,SF7,SF7,SF7,SF7
uint8_t adr_custom_list_ru864_default[16] =  { 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5 }; // SF9,SF9,SF9,SF9,SF9,SF8,SF8,SF8,SF8,SF8,SF7,SF7,SF7,SF7,SF7,SF7





WM1110_Geolocation* WM1110_Geolocation::instance_ = nullptr;

WM1110_Geolocation& WM1110_Geolocation::getInstance()
{
    if (instance_ == nullptr)
    {
        instance_ = new WM1110_Geolocation();
    }

    return *instance_;
}

WM1110_Geolocation::WM1110_Geolocation(void)
{

}

#if 1 // public function

void WM1110_Geolocation::begin(TrackScanType track_type,bool is_sensecap_platform)
{
    // wm1110_storage.begin();
    // wm1110_storage.loadBootConfigParameters();

    // wm1110_ble.begin();
    // wm1110_ble.setName();
    // wm1110_ble.setStartParameters();

    printf("\n---------- Booting ----------\n");
    printf("Wio Tracker 1110 Dev Board\n");
    printf("Firmware Version: v%d.%d\n",TRACKER_SW_MAJOR_VER,TRACKER_SW_MINOR_VER);

    // Obtain network access configuration parameters
    memcpy(dev_eui, app_param.lora_info.DevEui, SMTC_MODEM_EUI_LENGTH);
    memcpy(join_eui, app_param.lora_info.JoinEui, SMTC_MODEM_EUI_LENGTH);
    memcpy(app_key, app_param.lora_info.AppKey, SMTC_MODEM_KEY_LENGTH);
    region = getRegionFromDefault();

    lbmWm1110.attachGnssPrescan([](void* context){ digitalWrite(PIN_GNSS_LNA, HIGH); });
    lbmWm1110.attachGnssPostscan([](void* context){ digitalWrite(PIN_GNSS_LNA, LOW); });

    lbmWm1110.begin();

    //Initialize gnss middleware
    gnss_mw_init( lbmWm1110.getRadio(), stack_id );
    gnss_mw_custom_enable_copy_send_buffer();
    gnss_mw_set_constellations( GNSS_MW_CONSTELLATION_GPS_BEIDOU );

    // /* Set user defined assistance position */
    gnss_mw_set_user_aiding_position( gnss_aiding_position_latitude, gnss_aiding_position_longitude );

    //Initialize wifi middleware
    wifi_mw_init(lbmWm1110.getRadio(), stack_id);
    wifi_mw_send_bypass(true);  

    //
    track_scan_type = track_type;
    sensecap_platform = is_sensecap_platform; 
    switch(track_scan_type)
    {
        case Track_Scan_Gps:
            if(sensecap_platform == false)
            {
                uplink_port = 192;  // LORAWAN PORT for loracloud
            }
            track_timeout = 2*60*1000;
            break;
        case Track_Scan_Wifi:
            if(sensecap_platform == false)
            {
                uplink_port = 197;  // LORAWAN PORT for loracloud
            }
            track_timeout = 4*1000;
            break;            
        case Track_Scan_Ble:
            track_timeout = 5*1000;
            break;
        default:
            break;
    }
    track_period_time = app_append_param.position_interval*60*1000;
    sensor_period_time = app_append_param.sample_interval*60*1000;
}

void WM1110_Geolocation::run(void)
{
    static constexpr char region_str[][20]={"EU_868","AS_923_GRP1","US_915", "AU_915","CN_470","WW2G4","AS_923_GRP2","AS_923_GRP3","IN_865","KR_920","RU_864","CN_470_RP_1_0", "AS_923_GRP4","AS_923_HELIUM_1","AS_923_HELIUM_2","AS_923_HELIUM_3","AS_923_HELIUM_4","AS_923_HELIUM_1B"};
    printf("DevEUI:");
    for(uint8_t u8i = 0;u8i < 8; u8i++)
    {
        printf("%02x",dev_eui[u8i]);
    }
    printf("\r\n");
    printf("Region:%s\r\n",region_str[region-1]);

    printf("Scan geolocation Interval(min):%lu\r\n",track_period_time/60/1000);

    printf("Read sensor data Interval(min):%lu\r\n",sensor_period_time/60/1000);

    LbmxEngine::begin(lbmWm1110.getRadio(), &WM1110_Geolocation::modemEventHandler);

    printLbmVersion(lbmWm1110.getRadio());
}

uint32_t WM1110_Geolocation::lbmxProcess( void )
{
    uint32_t sleepTime = LbmxEngine::doWork(); 
    return  sleepTime;
}

uint32_t WM1110_Geolocation::trackProcess()
{
    uint32_t consume_time = 0;
    bool result = false;
    static uint32_t start_scan_time = 0;  
    uint32_t expectedTime = 1000; 
    uint32_t sleepTime = LbmxEngine::doWork(); 

    modemLedActionProcess();

    if(time_sync_flag == true)
    {
        if(sleepTime > 300)
        {  
            uint32_t now_time = smtc_modem_hal_get_time_in_ms( );
            switch(tracker_scan_status)
            {
                case Track_None:
                case Track_Start:
                    if(now_time - start_scan_time > track_period_time ||(start_scan_time == 0))
                    {
                        if(startTrackerScan())
                        {
                            printf("Start tracker scan\r\n");
                            start_scan_time = smtc_modem_hal_get_time_in_ms( );
                            consume_time = start_scan_time - now_time;
                        }
                        else
                        {
                            consume_time = smtc_modem_hal_get_time_in_ms() - now_time;                    
                        } 
                    }
                    else
                    {
                        if((start_scan_time != 0)&&(start_scan_time + track_period_time)>now_time)
                        {
                            expectedTime = start_scan_time + track_period_time - now_time;
                        }
                        else
                        {
                            expectedTime = 60;    
                        }    
                    }
                    break;
                case Track_Scaning:
                    if(smtc_modem_hal_get_time_in_ms( ) - start_scan_time > track_timeout)
                    {
                        stopTrackerScan();    
                        result = getTrackResults( );
                        if(result)
                        {
                            displayTrackRawDatas();
                        } 
                    }
                    else
                    {
                        expectedTime = start_scan_time + track_timeout - smtc_modem_hal_get_time_in_ms( );
                    }
                    break;
                case Track_End:
                    result = getTrackResults( );
                    if(result)
                    {
                        displayTrackRawDatas();
                    }
                    stopTrackerScan(); 
                    expectedTime = 60;
                    break;
                case Track_Stop:

                    printf("Stop tracker scan\r\n");
                    //Insert  position data to lora tx buffer 
                    insertTrackResultsIntoQueue();
                    consume_time = smtc_modem_hal_get_time_in_ms( ) - now_time; 
                    tracker_scan_status = Track_None;
                    expectedTime = 60;
                    break;
                default :
                    break;
            }
            sleepTime = sleepTime - consume_time; 
        }
    }

    sleepTime = min( sleepTime,  expectedTime);
    return  sleepTime;
}



void WM1110_Geolocation::setGnssAidingPosition(float laittude,float longitude)
{
    gnss_aiding_position_latitude = laittude;
    gnss_aiding_position_longitude = longitude;

    gnss_mw_set_user_aiding_position( gnss_aiding_position_latitude, gnss_aiding_position_longitude );
}

void WM1110_Geolocation::getGnssAidingPosition(float *laittude,float *longitude)
{
    *laittude = gnss_aiding_position_latitude;
    *longitude = gnss_aiding_position_longitude;
}

void WM1110_Geolocation::setCustomRegion(smtc_modem_region_t modem_region)   
{
    region = modem_region;
}

smtc_modem_region_t WM1110_Geolocation::getRegion()
{
    return region;
}

void WM1110_Geolocation::setCustomJoinNetworkCode(uint8_t devEui[SMTC_MODEM_EUI_LENGTH], uint8_t joinEui[SMTC_MODEM_EUI_LENGTH], uint8_t nwkKey[SMTC_MODEM_KEY_LENGTH])
{
    memcpy(dev_eui, devEui, SMTC_MODEM_EUI_LENGTH);
    memcpy(join_eui, joinEui, SMTC_MODEM_EUI_LENGTH);
    memcpy(app_key, nwkKey, SMTC_MODEM_KEY_LENGTH);    
}
      
void WM1110_Geolocation::getJoinNetworkCode(uint8_t *devEui, uint8_t *joinEui, uint8_t *nwkKey)
{
    memcpy(devEui, dev_eui, SMTC_MODEM_EUI_LENGTH);
    memcpy(joinEui, join_eui, SMTC_MODEM_EUI_LENGTH);
    memcpy(nwkKey, app_key, SMTC_MODEM_KEY_LENGTH);    
}

void WM1110_Geolocation::setCustomTrackPeriod( uint16_t period_in_min )
{
    if(track_period_time < 3)
    {
        track_period_time = 3;
    }
    track_period_time = period_in_min*60*1000;
}   

uint16_t WM1110_Geolocation::getTrackPeriod( void )
{
    return track_period_time/60/1000;
}  

void WM1110_Geolocation::setSensorMeasurementPeriod( uint16_t period_in_min )
{
    sensor_period_time = period_in_min*60*1000;
}   

uint16_t WM1110_Geolocation::getSensorMeasurementPeriod( void )
{
    return sensor_period_time/60/1000;
}  

void WM1110_Geolocation::setTrackTimeout( uint16_t time_in_s )
{
    track_timeout = time_in_s*1000;
} 

uint16_t WM1110_Geolocation::getTrackTimeout( void )
{
    return track_timeout/1000;
}      

void WM1110_Geolocation::setUplinkPort(uint8_t lora_port)
{
    uplink_port = lora_port;    
}

uint8_t WM1110_Geolocation::getUplinkPort( void )
{
    return uplink_port;
}  

bool WM1110_Geolocation::startTrackerScan( void )
{
    uint8_t scan_busy_status = 0;
    bool radio_is_busy = false;

    start_scan_timestamp = getUtcTimestamp( );
    radio_is_busy = getLbmxRadioBusyState(&scan_busy_status);

    if(track_scan_type == Track_Scan_Gps)
    {
        if(radio_is_busy)
        {
            if(scan_busy_status&WM1110_RADIO_GNSS_BUSY)
            {
                tracker_scan_status = Track_Scaning;
                return true;                
            }
            else
            {
                return false;
            }
        }
        else
        {
            gnss_results_index = 0;
            gnss_mw_custom_send_buffer_num = 0;
            for( uint8_t i = 0; i < GNSS_SCAN_GROUP_SIZE_MAX; i++ )
            {
                gnss_mw_custom_send_buffer_len[i] = 0;
                memset( gnss_mw_custom_send_buffer[i], 0, 64 );
            }
            uint8_t app_gnss_scan_mode = GNSS_MW_MODE_MOBILE;
            mw_return_code_t gnss_rc = gnss_mw_scan_start( (gnss_mw_mode_t)app_gnss_scan_mode, 0 ); /* start ASAP */
            if( gnss_rc == MW_RC_OK )
            {
                tracker_scan_status = Track_Scaning;
                return true;
            }
            else
            {
                WM1110_GEOLOCATION_DBG_PRINTF( "Failed to start GNSS scan\n" );
                return false;
            }
        } 
    }
    else if(track_scan_type == Track_Scan_Wifi)
    {
        if(radio_is_busy)
        {
            if(scan_busy_status&WM1110_RADIO_WIFI_BUSY)
            {
                tracker_scan_status = Track_Scaning;
                return true;                
            }
            else
            {
                return false;
            }
        }
        else
        {
            memset(&WM1110_wifi_results,0,sizeof(wifi_mw_event_data_scan_done_t));
            mw_return_code_t wifi_rc = wifi_mw_scan_start( 0 );
            if( wifi_rc == MW_RC_OK )
            {
                tracker_scan_status = Track_Scaning;
                return true;
            }
            else
            {
                WM1110_GEOLOCATION_DBG_PRINTF( "Failed to start Wi-Fi scan\n" );
                return false;
            }        
        }
    }
    else
    {
        ble_results_index = 0;
        memset(&WM1110_ble_results,0,sizeof(WM1110_Beacons_t));
        bool ret = wm1110_ble.startScan();
        if( ret == true )
        {
            tracker_scan_status = Track_Scaning;
            return true;
        }
        else
        {
            WM1110_GEOLOCATION_DBG_PRINTF( "Failed to start scan iBeacon\n" );
            return false;
        }       
    }
    return false;
}

void WM1110_Geolocation::stopTrackerScan( void )
{
    if(track_scan_type == Track_Scan_Gps)
    {
        gnss_mw_scan_cancel( );
        tracker_scan_status = Track_Stop;
    }
    else if(track_scan_type == Track_Scan_Wifi)
    {
        wifi_mw_scan_cancel( );
        tracker_scan_status = Track_Stop;
    }
    else
    {
        wm1110_ble.stopScan();
        tracker_scan_status = Track_Stop;        
    }
}

bool WM1110_Geolocation::getTrackResults( void )
{
    if(track_scan_type == Track_Scan_Gps)
    {
        if( gnss_mw_custom_send_buffer_num )
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    else if(track_scan_type == Track_Scan_Wifi)
    {
        if( WM1110_wifi_results.nbr_results )
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        bool ret = wm1110_ble.getScanedResults();
        if( ret == true)
        {
            ble_results_index = wm1110_ble.beacon_results_num;
            for(uint8_t u8i = 0; u8i < ble_results_index;u8i++)
            {
                memcpy( &WM1110_ble_results[u8i], &wm1110_ble.beacon_results_buf[u8i], sizeof(WM1110_Beacons_t) );         
            }
            return true;
        }
        return ret;
    }
    return false;
}

bool WM1110_Geolocation::getTrackDatas(uint8_t *buf, uint8_t *size)
{
    if(track_scan_type == Track_Scan_Gps)
    {
        if( gnss_mw_custom_send_buffer_num )
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    else if(track_scan_type == Track_Scan_Wifi)
    {
        uint32_t utc = 0;
        if( WM1110_wifi_results.nbr_results )
        {
            uint8_t wifi_buffer_size = 0;

            // TODO - get wifi scan utc
            utc = getUtcTimestamp( );

            memcpyr( wifi_result_buffer, ( uint8_t * )( &utc ), 4 );
            wifi_buffer_size += 4;

            /* Concatenate all results in send buffer */
            for( uint8_t i = 0; i < WM1110_wifi_results.nbr_results; i++ )
            {
                /* Copy Access Point MAC address in result buffer */
                memcpy( &wifi_result_buffer[wifi_buffer_size], WM1110_wifi_results.results[i].mac_address, WIFI_AP_ADDRESS_SIZE );  
                wifi_buffer_size += WIFI_AP_ADDRESS_SIZE;

                #if WIFI_AP_RSSI_EN
                /* Copy Access Point RSSI address in result buffer (if requested) */
                wifi_result_buffer[wifi_buffer_size] = WM1110_wifi_results.results[i].rssi;
                wifi_buffer_size += WIFI_AP_RSSI_SIZE; 
                #endif
            }

            if( WM1110_wifi_results.nbr_results < WIFI_MAX_RESULTS )
            {
                for( uint8_t i = 0; i < ( WIFI_MAX_RESULTS - WM1110_wifi_results.nbr_results ); i++  )
                {
                    memset( &wifi_result_buffer[wifi_buffer_size], 0xff, ( WIFI_AP_ADDRESS_SIZE + WIFI_AP_RSSI_SIZE ));   
                    wifi_buffer_size += ( WIFI_AP_ADDRESS_SIZE + WIFI_AP_RSSI_SIZE );  
                }
            }
            
            if( buf ) memcpy( buf, wifi_result_buffer, wifi_buffer_size );
            if( size ) memcpy( size, &wifi_buffer_size, 1 );

            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        bool ret = wm1110_ble.getScanedDatas(buf,size);      
        if( ret == true )
        {
            return true;
        }
        return false;
    }
    return false;
}

void WM1110_Geolocation::displayTrackRawDatas( void )
{
    if(track_scan_type == Track_Scan_Gps)
    {
        if( gnss_mw_custom_send_buffer_num )
        {
            printf( "GNSS Raw:\r\n" );
        }
        for( uint8_t i = 0; i < gnss_mw_custom_send_buffer_num; i++ )
        {
            for( uint8_t j = 0; j < gnss_mw_custom_send_buffer_len[i]; j++ )
            {
                printf( "%02X", gnss_mw_custom_send_buffer[i][j] );
            }
            printf( "\r\n" );
        }
    }
    else if(track_scan_type == Track_Scan_Wifi)
    {
        printf( "Wi-Fi Scaned results: %u\r\n", WM1110_wifi_results.nbr_results );
        for( uint8_t i = 0; i < WM1110_wifi_results.nbr_results; i++ )
        {
            for( uint8_t j = 0; j < WIFI_AP_ADDRESS_SIZE; j++ )
            {
                printf( "%02X ", WM1110_wifi_results.results[i].mac_address[j] );
            }
            printf( "\r\n");
            printf( "Channel: %d, ", WM1110_wifi_results.results[i].channel );
            printf( "Type: %d, ", WM1110_wifi_results.results[i].type );
            printf( "RSSI: %d\r\n", WM1110_wifi_results.results[i].rssi );
        }
    }
    else
    {
        printf( "Scaned iBeacon results: %u\r\n", ble_results_index );
        wm1110_ble.displayScanedDatas();
    }
}
void WM1110_Geolocation::dispalyTrackDatas( void )
{
    printf( "Tracker Scaned results: \r\n");    
    packTrackResults();
    for(uint8_t u8i = 0; u8i < 3; u8i++)
    {
        if(track_scaned_data_len[u8i])
        {
            for(uint8_t u8j = 0; u8j < track_scaned_data_len[u8i]; u8j++)
            {
                printf("%02x",track_scaned_data_buf[u8i][u8j]);
            } 
            printf("\r\n");           
        }
    }
}


uint32_t WM1110_Geolocation::getUtcTimestamp(void)
{
    uint32_t timestamp_now =0;
    if(time_sync_flag)
    {
        timestamp_now = apps_modem_common_get_utc_time( );
    }
    if(timestamp_now != 0)
    {
        return timestamp_now;
    }
    return smtc_modem_hal_get_time_in_s( );
}

bool WM1110_Geolocation::insertIntoTxQueue( uint8_t *buf, uint8_t len )
{
    bool result = false;
    smtc_modem_status_mask_t modem_status;
    smtc_modem_get_status( 0, &modem_status );
    if(( modem_status & SMTC_MODEM_STATUS_JOINED ) == SMTC_MODEM_STATUS_JOINED )
    { 
        WM1110_GEOLOCATION_DBG_PRINTF("User datas insert:");
        for(uint8_t u8j = 0; u8j < len; u8j++ )
        {
            WM1110_GEOLOCATION_DBG_PRINTF("%02x",buf[u8j]);
        }
        WM1110_GEOLOCATION_DBG_PRINTF("\r\n");     

        uint8_t in = tx_queue_in_index  % APP_TASK_LORA_TX_QUEUE_MAX;
        tx_queue_data_len[in] = len;
        tx_confirmed_queue_buf[in] = true;
        for( uint8_t j = 0; j < len; j++ )
        {
            tx_queue_data_buf[in][j] = buf[j];
        }
        tx_queue_in_index = (tx_queue_in_index + 1) % APP_TASK_LORA_TX_QUEUE_MAX;
    }
    else
    {
        result = false;
    }

    return result;    
}

void WM1110_Geolocation::getLoRaWANRunningState(LoRaWANStateType *state)
{
    *state = lora_run_state;
}

void WM1110_Geolocation::setEventStateAll(uint32_t state)
{
    track_state_all |= state;
}

void WM1110_Geolocation::getEventStateAll(uint32_t *state)
{
    *state = track_state_all;
}
#endif

#if 1 // private function

bool WM1110_Geolocation::getLbmxRadioBusyState(uint8_t *busy_type)
{
    uint8_t busy_status = 0;
    smtc_modem_stack_state_t state;
    smtc_modem_get_stack_state( stack_id, &state );

    if(state != SMTC_MODEM_STACK_STATE_IDLE)
    {
        busy_status |= WM1110_RADIO_OTHER_BUSY;
    }
    if(gnss_mw_custom_get_scan_busy())
    {
        busy_status |= WM1110_RADIO_GNSS_BUSY;
    }
    if(wifi_mw_custom_get_scan_busy())
    {
        busy_status |= WM1110_RADIO_WIFI_BUSY;        
    }
    *busy_type = busy_status;

    if(busy_status == 0)
    {
        return false;
    }
    return true;
}

void WM1110_Geolocation::packTrackResults( void )
{
    uint8_t motion_index_temp = 0;

    // motion_index_temp = motion_index;

    for(uint8_t u8i = 0; u8i < GNSS_SCAN_GROUP_SIZE_MAX; u8i++)
    {
        memset(track_scaned_data_buf[u8i],0,64);
        track_scaned_data_len[u8i] = 0;
    }

    if(sensecap_platform)
    {
        //------------------------------------ sensecap ----------------------------------------------
        // Location failure---------------
        if( gnss_mw_custom_send_buffer_num == 0 && WM1110_wifi_results.nbr_results == 0 && ble_results_index == 0 )
        {
            // Location failure packet type
            track_scaned_data_buf[0][0] = DATA_ID_UP_PACKET_POS_STATUS;
            track_scaned_data_len[0] += 1;

            // Location failure reason
            if(( time_sync_flag == false ) || ( mw_gnss_event_state == GNSS_MW_EVENT_ERROR_NO_TIME ))
            {
                mw_gnss_event_state = 0;
                track_scaned_data_buf[0][1] = 13; // sync time fail
            }
            else if( mw_gnss_event_state == GNSS_MW_EVENT_ERROR_ALMANAC_UPDATE )
            {
                mw_gnss_event_state = 0;
                track_scaned_data_buf[0][1] = 14; // almanac too old
            }
            else    // timeout
            {
                track_scaned_data_buf[0][1] = track_scan_type + 1;
            }
            track_scaned_data_len[0] += 1;

            // event  status
            track_scaned_data_buf[0][2] = ( track_state_all >> 16 ) & 0xff;
            track_scaned_data_buf[0][3] = ( track_state_all >> 8 ) & 0xff;
            track_scaned_data_buf[0][4] = ( track_state_all ) & 0xff;
            track_scaned_data_len[0] += 3;

            // utc time
            uint32_t utc_temp = start_scan_timestamp;
            memcpyr( track_scaned_data_buf[0] + track_scaned_data_len[0], ( uint8_t * )( &utc_temp ), 4 );
            track_scaned_data_len[0] += 4;
             
        }
        //Location success----------
    #if TRACKER_GPS_EN
        if( gnss_mw_custom_send_buffer_num )
        {
            uint8_t gnss_raw_num = 0;
            uint8_t len_max_index = 0;
            uint8_t len_array[GNSS_SCAN_GROUP_SIZE_MAX] = { 0 };

            for( uint8_t i = 0; i < gnss_mw_custom_send_buffer_num; i++ )
            {
                gnss_mw_custom_send_buffer[i][0] &= 0x7f; // clear all byte[0] high bit7
                len_array[i] = i;
            }
            // Sort by gnss length 
            for( uint8_t i = 0; i < gnss_mw_custom_send_buffer_num; i++ )
            {
                for( uint8_t j = i; j < gnss_mw_custom_send_buffer_num; j++ )
                {
                    if( gnss_mw_custom_send_buffer_len[len_array[i]] < gnss_mw_custom_send_buffer_len[len_array[j]] )
                    {
                        len_max_index = len_array[i];
                        len_array[i] = len_array[j];
                        len_array[j] = len_max_index;
                    }
                }
            }
            // no more than two packets of raw data will be sent at a time
            if( gnss_mw_custom_send_buffer_num > GNSS_RAW_SCAN_SEND_MAX )
            {
                gnss_raw_num = GNSS_RAW_SCAN_SEND_MAX;
            }
            else
            {
                gnss_raw_num = gnss_mw_custom_send_buffer_num;
            }
            gnss_mw_custom_send_buffer[len_array[gnss_raw_num - 1]][0] |= 0x80; // set last raw byte[0] high bit7

            uint8_t fragment_total = gnss_raw_num + 1;  // 

            // gnss datas need to subcontract delivery,these are gnss datas package
            for( uint8_t i = 0; i < gnss_raw_num; i++ ) 
            {
                // packet id
                track_scaned_data_buf[i][0] = DATA_ID_UP_PACKET_GNSS_RAW;
                track_scaned_data_len[i] += 1;

                // fragment data,package num 
                track_scaned_data_buf[i][1] = ( fragment_total << 4 ) + i; // fragment data,package num 
                track_scaned_data_len[i] += 1;

                // group id
                memcpyr( track_scaned_data_buf[i] + track_scaned_data_len[i], ( uint8_t * )( &track_gnss_group_id ), 2 ); // group id
                track_scaned_data_len[i] += 2;

                // raw data len
                track_scaned_data_buf[i][4] = gnss_mw_custom_send_buffer_len[len_array[i]];
                track_scaned_data_len[i] += 1;

                // raw data            
                memcpy( track_scaned_data_buf[i] + track_scaned_data_len[i], gnss_mw_custom_send_buffer[len_array[i]], gnss_mw_custom_send_buffer_len[len_array[i]] );
                track_scaned_data_len[i] += gnss_mw_custom_send_buffer_len[len_array[i]];

            }
            WM1110_GEOLOCATION_DBG_PRINTF("Track_scaned_data_len :%u\r\n",track_scaned_data_len[gnss_raw_num]);
            // packet id
            track_scaned_data_buf[gnss_raw_num][0] = DATA_ID_UP_PACKET_GNSS_END;
            track_scaned_data_len[gnss_raw_num] += 1;

            // event  status
            track_scaned_data_buf[gnss_raw_num][1] = ( track_state_all >> 16 ) & 0xff;
            track_scaned_data_buf[gnss_raw_num][2] = ( track_state_all >> 8 ) & 0xff;
            track_scaned_data_buf[gnss_raw_num][3] = ( track_state_all ) & 0xff;
            track_scaned_data_len[gnss_raw_num] += 3;   

            // motion index        
            track_scaned_data_buf[gnss_raw_num][4] = motion_index_temp;
            track_scaned_data_len[gnss_raw_num] += 1;

            // utc time
            uint32_t utc_temp = start_scan_timestamp;
            memcpyr( track_scaned_data_buf[gnss_raw_num] + track_scaned_data_len[gnss_raw_num], ( uint8_t * )( &utc_temp ), 4 );
            track_scaned_data_len[gnss_raw_num] += 4;

            // fragment data
            track_scaned_data_buf[gnss_raw_num][track_scaned_data_len[gnss_raw_num]] = ( fragment_total << 4 ) + gnss_raw_num; 
            track_scaned_data_len[gnss_raw_num] += 1;

            // group id
            memcpyr( track_scaned_data_buf[gnss_raw_num] + track_scaned_data_len[gnss_raw_num], ( uint8_t * )( &track_gnss_group_id ), 2 ); 
            track_scaned_data_len[gnss_raw_num] += 2;   

            printf( "GNSS raw group id: 0x%04x\r\n", track_gnss_group_id );
            track_gnss_group_id ++;
            wm1110_storage.writeGnssGroupID();

        }
    #endif

    #if TRACKER_WIFI_EN
        if( WM1110_wifi_results.nbr_results )
        {
            track_scaned_data_buf[0][0] = DATA_ID_UP_PACKET_WIFI_RAW;
            track_scaned_data_len[0] += 1;
            // event  status
            track_scaned_data_buf[0][1] = ( track_state_all >> 16 ) & 0xff;
            track_scaned_data_buf[0][2] = ( track_state_all >> 8 ) & 0xff;
            track_scaned_data_buf[0][3] = ( track_state_all ) & 0xff;
            track_scaned_data_len[0] += 3;

            // motion index  
            track_scaned_data_buf[0][4] = motion_index_temp;
            track_scaned_data_len[0] += 1;

            // utc + wifi
            uint32_t utc_temp = start_scan_timestamp;
            memcpyr( track_scaned_data_buf[0] + 5, ( uint8_t * )( &utc_temp ), 4 );
            track_scaned_data_len[0] += 4;   

            for( uint8_t u8i = 0; u8i < WM1110_wifi_results.nbr_results; u8i++ )
            {
                /* Copy Access Point MAC address in result buffer */
                memcpy( &track_scaned_data_buf[0][track_scaned_data_len[0]], WM1110_wifi_results.results[u8i].mac_address, WIFI_AP_ADDRESS_SIZE ); 
                track_scaned_data_len[0] += WIFI_AP_ADDRESS_SIZE; 

                #if WIFI_AP_RSSI_EN
                /* Copy Access Point RSSI address in result buffer (if requested) */
                track_scaned_data_buf[0][track_scaned_data_len[0]] = WM1110_wifi_results.results[u8i].rssi;
                track_scaned_data_len[0] += WIFI_AP_RSSI_SIZE;  
                #endif
            }
            for( uint8_t i = 0; i < ( WIFI_MAX_RESULTS - WM1110_wifi_results.nbr_results ); i++  )
            {
                memset( &track_scaned_data_buf[0][track_scaned_data_len[0]], 0xff, ( WIFI_AP_ADDRESS_SIZE + WIFI_AP_RSSI_SIZE ));     
                track_scaned_data_len[0] += ( WIFI_AP_ADDRESS_SIZE + WIFI_AP_RSSI_SIZE );           
            }  
        
        }
    #endif

    #if TRACKER_BLE_EN
        if( ble_results_index )
        {
            track_scaned_data_buf[0][0] = DATA_ID_UP_PACKET_BLE_RAW;
            track_scaned_data_len[0] += 1;

            // event  status
            track_scaned_data_buf[0][1] = ( track_state_all >> 16 ) & 0xff;
            track_scaned_data_buf[0][2] = ( track_state_all >> 8 ) & 0xff;
            track_scaned_data_buf[0][3] = ( track_state_all ) & 0xff;
            track_scaned_data_len[0] += 3;

            // motion index 
            track_scaned_data_buf[0][4] = motion_index_temp;
            track_scaned_data_len[0] += 1;


            //utc time,the previous UTC time is overwritten here(utc + wifi)
            uint32_t utc_temp = start_scan_timestamp;
            memcpyr( track_scaned_data_buf[0] + 5, ( uint8_t * )( &utc_temp ), 4 );

            track_scaned_data_len[0] += 4;

            //
            // fill in ibeacon mac&rssi
            for( uint8_t i = 0; i < ble_results_index; i ++ )
            {
                memcpyr( &track_scaned_data_buf[0][track_scaned_data_len[0]], ( uint8_t *)( &WM1110_ble_results[i].mac ), 6 );
                track_scaned_data_len[0] += IBEACON_MAC_ADDRESS_SIZE;     

                track_scaned_data_buf[0][track_scaned_data_len[0]] = WM1110_ble_results[i].rssi_;
                track_scaned_data_len[0] += IBEACON_RSSI_SIZE;       

            }

            // if Not enough, fill in 0xff
            if( ble_results_index < BLE_BEACON_SEND_MUM )
            {
                for( uint8_t i = 0; i < ( BLE_BEACON_SEND_MUM - ble_results_index ); i++ )
                {

                    memset( &track_scaned_data_buf[0][track_scaned_data_len[0]], 0xff, 7 );
                    track_scaned_data_len[0] += IBEACON_MAC_ADDRESS_SIZE;    
                    track_scaned_data_len[0] += IBEACON_RSSI_SIZE;     

                }
            }
        }
        track_state_all = 0;
    #endif

    //------------------------------------ SenseCap ----------------------------------------------
    }
    else
    {
        #if TRACKER_GPS_EN
        if(gnss_mw_custom_send_buffer_num)
        {
            for( uint8_t u8i = 0; u8i < gnss_mw_custom_send_buffer_num; u8i++ )
            {
                memcpy(track_scaned_data_buf[u8i],gnss_mw_custom_send_buffer[u8i],gnss_mw_custom_send_buffer_len[u8i]);
                track_scaned_data_len[u8i] = gnss_mw_custom_send_buffer_len[u8i];
            }            
        }
        #endif
        #if TRACKER_WIFI_EN
        if( WM1110_wifi_results.nbr_results) 
        {
            /* Add the payload format tag */
            track_scaned_data_buf[0][track_scaned_data_len[0]] = WIFI_MW_PAYLOAD_MAC_RSSI;
            track_scaned_data_len[0] += 1;

            /* Concatenate all results in send buffer */
            for( uint8_t i = 0; i < WM1110_wifi_results.nbr_results; i++ )
            {
                track_scaned_data_buf[0][track_scaned_data_len[0]] = WM1110_wifi_results.results[i].rssi;
                track_scaned_data_len[0] += WIFI_AP_RSSI_SIZE;

                /* Copy Access Point MAC address in result buffer */
                memcpy( &track_scaned_data_buf[0][track_scaned_data_len[0]], WM1110_wifi_results.results[i].mac_address, WIFI_AP_ADDRESS_SIZE );
                track_scaned_data_len[0] += WIFI_AP_ADDRESS_SIZE;
            }
        }
        #endif
        #if TRACKER_BLE_EN
        if( ble_results_index )
        {
            track_scaned_data_buf[0][0] = DATA_ID_UP_PACKET_BLE_RAW;
            track_scaned_data_len[0] += 1;

            // event  status
            track_scaned_data_buf[0][1] = ( track_state_all >> 16 ) & 0xff;
            track_scaned_data_buf[0][2] = ( track_state_all >> 8 ) & 0xff;
            track_scaned_data_buf[0][3] = ( track_state_all ) & 0xff;
            track_scaned_data_len[0] += 3;

            // motion index 
            track_scaned_data_buf[0][4] = motion_index_temp;
            track_scaned_data_len[0] += 1;


            //utc time,the previous UTC time is overwritten here(utc + wifi)
            uint32_t utc_temp = start_scan_timestamp;
            memcpyr( track_scaned_data_buf[0] + 5, ( uint8_t * )( &utc_temp ), 4 );

            track_scaned_data_len[0] += 4;

            // fill in ibeacon mac&rssi
            for( uint8_t i = 0; i < ble_results_index; i ++ )
            {
                memcpyr( &track_scaned_data_buf[0][track_scaned_data_len[0]], ( uint8_t *)( &WM1110_ble_results[i].mac ), 6 );
                track_scaned_data_len[0] += IBEACON_MAC_ADDRESS_SIZE;     

                track_scaned_data_buf[0][track_scaned_data_len[0]] = WM1110_ble_results[i].rssi_;
                track_scaned_data_len[0] += IBEACON_RSSI_SIZE;       

            }

            // if Not enough, fill in 0xff
            if( ble_results_index < BLE_BEACON_SEND_MUM )
            {
                for( uint8_t i = 0; i < ( BLE_BEACON_SEND_MUM - ble_results_index ); i++ )
                {
                    memset( &track_scaned_data_buf[0][track_scaned_data_len[0]], 0xff, 7 );
                    track_scaned_data_len[0] += IBEACON_MAC_ADDRESS_SIZE;    
                    track_scaned_data_len[0] += IBEACON_RSSI_SIZE;     

                }
            }
        }
        #endif
    }
}

bool WM1110_Geolocation::txProcess( void )
{
    static uint8_t packet_send_cnt = 0;

    smtc_modem_status_mask_t modem_status;
    smtc_modem_region_t cur_region;
    uint8_t  dutycycle_enable = 0;

    static uint8_t last_time_tx_res = false;
    static uint8_t out = 0;

    uint32_t tx_now_timestemp = smtc_modem_hal_get_time_in_ms( );

    ASSERT_SMTC_MODEM_RC( smtc_modem_get_region(0, &cur_region));
    
    ASSERT_SMTC_MODEM_RC( smtc_modem_get_duty_cycle_enable(&dutycycle_enable));

    if(dutycycle_enable == 1)
    {
        if((cur_region == SMTC_MODEM_REGION_EU_868) || (cur_region == SMTC_MODEM_REGION_RU_864))
        {
            if(tx_now_timestemp<(last_tx_timestamp+data_tx_toa * 99))  //Limit dutycycle to less than 1% 
            {
                last_time_tx_res = false;
                return false;
            }
        }
    }

    if( check_tx_flag ) //need check confirmed last time send packet
    {
        check_tx_flag = false;
        uint32_t count_temp = getTxConfirmedCount( );
        if( tx_confirmed_count_backup >= count_temp )    //unconfirmed
        {
            if(cache_offline_data_flag == true) // send fail need save offline data to littlefs
            {
                if(tx_offline_data_flag)
                {
                    tx_offline_data_flag = false;
                }
                else
                {
                    if(packet_send_cnt >= 2)
                    {
                        packet_send_cnt = 0;
                        bool data_cache_ret = cacheOfflineDatas( tx_queue_data_buf[tx_queue_out_index], tx_queue_data_len[tx_queue_out_index] );
                        if(data_cache_ret)
                        {
                            printf("Unable to receive confirm Ack from LNS, Cache the current packet:");
                            for(uint8_t u8i = 0; u8i < tx_queue_data_len[tx_queue_out_index]; u8i++ )
                            {
                                printf("%02x",tx_queue_data_buf[tx_queue_out_index][u8i]);
                            }
                            printf("\r\n"); 
                        }
                        tx_queue_data_len[out] = 0;   //Clear the last data
                        out = (++tx_queue_out_index % APP_TASK_LORA_TX_QUEUE_MAX);
                    }  
                }
            }
            else    // don't need save offline data 
            {
                if(send_retry_type == 1) //just confirm once,then unconfirm
                {
                    if(packet_send_cnt >= 1)
                    {
                        packet_send_cnt = 0;
                        tx_confirmed_queue_buf[tx_queue_out_index] = false;     
                    }
                } 
                else if(send_retry_type == 0) //confirm twice,then unconfirm 
                {
                    if(packet_send_cnt >= 2)
                    {
                        packet_send_cnt = 0;
                        tx_confirmed_queue_buf[tx_queue_out_index] = false; 
                    }    
                }
                else
                {
                    packet_send_cnt = 0;
                    tx_queue_data_len[out] = 0;   //Clear the last data
                    out = (++tx_queue_out_index % APP_TASK_LORA_TX_QUEUE_MAX);                
                }
            }
        }
        else    //confirmed
        {
            if( tx_offline_data_flag )
            {
                tx_offline_data_flag = false;
                int8_t i = 3;
                while( i-- )
                {
                    int8_t ret  = wm1110_storage.deletePositionData( 1 );
                    if(ret > 0)
                    {
                        printf( "Delete old position data ok\r\n" );
                        break;
                    }
                }
            }
            else
            {
				packet_send_cnt = 0;
                tx_queue_data_len[out] = 0;   //Clear the last data
                out = (++tx_queue_out_index % APP_TASK_LORA_TX_QUEUE_MAX);    //
            }
        }
    }
    else
    {
        if(last_time_tx_res == true)
        {
			packet_send_cnt = 0;
            tx_queue_data_len[out] = 0;   //Clear the last data
            out = (++tx_queue_out_index % APP_TASK_LORA_TX_QUEUE_MAX);
        }
    }
    
    if( tx_queue_data_len[out] ) // The current packet to be sent is not empty
    {
        smtc_modem_get_status( 0, &modem_status );
        uint8_t radio_busy_status;
        bool radio_is_busy = false;
        radio_is_busy = getLbmxRadioBusyState(&radio_busy_status);

        if(( modem_status & SMTC_MODEM_STATUS_JOINED ) == SMTC_MODEM_STATUS_JOINED &&( radio_is_busy == false ))
        {
            bool result = sendFrame( tx_queue_data_buf[out], tx_queue_data_len[out], tx_confirmed_queue_buf[out], false );
            if( result )
            {
                printf("Sending Uplink Payload:");
                for(uint8_t u8i = 0; u8i < tx_queue_data_len[out]; u8i++ )
                {
                    printf("%02x ",tx_queue_data_buf[out][u8i]);
                }
                printf("\r\n"); 

                last_tx_timestamp = smtc_modem_hal_get_time_in_ms( );
                tx_queue_out_index = out;
                
                smtc_modem_get_toa_status( &data_tx_toa,tx_queue_data_len[out] + 13 ); //Calculates the air time of the current packet

                if( tx_confirmed_queue_buf[out] )
                {
                    packet_send_cnt ++;
                    check_tx_flag = true;
                    tx_offline_data_flag = false;
                    tx_confirmed_count_backup = getTxConfirmedCount( );
                }
                else
                {
                    check_tx_flag = false;
                }
                last_time_tx_res = true;
                return true;
            }
        }
        else
        {
            last_time_tx_res = false;
            return false;
        }
    }
    else
    {
        if(cache_offline_data_flag == true)
        {
            bool offline_status = uplinkOfflineDatas();
            if(offline_status == true)
            {
                last_time_tx_res = true;
                return true;                
            }
        }
        last_time_tx_res = false;
        return false;
    }
    last_time_tx_res = false;
    return false;    
}

void WM1110_Geolocation::incrementTxConfirmedCount( void )
{
    tx_confirmed_count++;
}

uint32_t WM1110_Geolocation::getTxConfirmedCount( void )
{
    return tx_confirmed_count;
}

bool WM1110_Geolocation::cacheOfflineDatas( uint8_t *buf, uint8_t len )
{
    bool result = false;
    uint8_t ret = 0;

    if(sensecap_platform == false)
    {
        return false;   
    }

    memset(( uint8_t * )( &pos_msg_param ), 0, sizeof( pos_msg_param ));

    pos_msg_param.pos_type = buf[0];

    if( pos_msg_param.pos_type != DATA_ID_UP_PACKET_GNSS_RAW )
    {
        memcpyr(( uint8_t * )( &pos_msg_param.pos_status ), buf + 1, 4 );
        memcpyr(( uint8_t * )( &pos_msg_param.utc_time ), buf + 5, 4 );
    }

    switch( pos_msg_param.pos_type )
    {
        case DATA_ID_UP_PACKET_GNSS_RAW:
        {
            pos_msg_param.context_count = 1;
            pos_msg_param.context.gps_context.zone_flag = buf[1]; // fragment data
            pos_msg_param.context.gps_context.gnss_len = buf[4]; // GNSS raw lenght
            memcpyr(( uint8_t * )( &pos_msg_param.context.gps_context.group_id ), buf + 2, 2 ); // group id
            memcpy( pos_msg_param.context.gps_context.gnss_res, buf + 5, pos_msg_param.context.gps_context.gnss_len ); // GNSS raw data            
        }
        break;
        case DATA_ID_UP_PACKET_GNSS_END:
        {
            pos_msg_param.context_count = 1;
            pos_msg_param.context.gps_context.zone_flag = buf[9]; // fragment data
            memcpyr(( uint8_t * )( &pos_msg_param.context.gps_context.group_id ), buf + 10, 2 ); // group id
            pos_msg_param.context.gps_context.gnss_len = 0; // GNSS raw lenght
        }
        break;
        case DATA_ID_UP_PACKET_WIFI_RAW:
        {
            pos_msg_param.context_count = ( len - 9 ) / 7;  
            for( uint8_t i = 0; i < pos_msg_param.context_count; i++ )
            {
                memcpy( &pos_msg_param.context.wifi_context[i].wifi_mac, buf + 9 + i * 7, 6 );
                memcpy( &pos_msg_param.context.wifi_context[i].cur_rssi, buf + 9 + i * 7 + 6, 1 );
            }
        }
        break;
        case DATA_ID_UP_PACKET_BLE_RAW:
        {
            pos_msg_param.context_count = ( len - 9 ) / 7;
            for( uint8_t i = 0; i < pos_msg_param.context_count; i++ )
            {
                memcpy( &pos_msg_param.context.beac_context[i].beac_mac, buf + 9 + i * 7, 6 );
                memcpy( &pos_msg_param.context.beac_context[i].cur_rssi, buf + 9 + i * 7 + 6, 1 );
            }
        }
        break;
        case DATA_ID_UP_PACKET_POS_STATUS:
        {
            pos_msg_param.context_count = 1;
        }
        break;
        case DATA_ID_UP_PACKET_USER_SENSOR:
        {
            pos_msg_param.context_count = 1;
            pos_msg_param.context.sensor_context.len = len-1;
            memcpy( &pos_msg_param.context.sensor_context.sensor_data, &buf[1], len-1 );

        }
        break;
        case DATA_ID_UP_PACKET_FACT_SENSOR:
        {
            pos_msg_param.context_count = 1;
            pos_msg_param.context.sensor_context.len = len-1;
            memcpy( &pos_msg_param.context.sensor_context.sensor_data, &buf[1], len-1 );
        }
        break;  
        default:
        break;
    }

    ret = wm1110_storage.writePositionData(  );
    if( ret == true ) 
    {
        result = true;
    }
    else
    {
        result = false;
    } 
    return result;
}

bool  WM1110_Geolocation::uplinkOfflineDatas( void )
{
    smtc_modem_status_mask_t modem_status;
    pos_msg_param_t pos_msg;

    uint8_t offine_data_len_temp = 0;
    uint8_t offine_data_buf_temp[64] = { 0 };

    if(sensecap_platform == false)
    {
        return false;   
    }

    uint16_t pos_msg_cnt = wm1110_storage.getPositionDataCount( );
    if( pos_msg_cnt == 0 )
    {
        return false;
    }

    if( wm1110_storage.readSinglePositionData(&pos_msg, 1))
    {
        offine_data_buf_temp[0] = pos_msg.pos_type;
        if( pos_msg.pos_type != DATA_ID_UP_PACKET_GNSS_RAW )
        {
            memcpyr( offine_data_buf_temp + 1, ( uint8_t * )( &pos_msg.pos_status ), 4 );
            memcpyr( offine_data_buf_temp + 5, ( uint8_t * )( &pos_msg.utc_time ), 4 );
        }

        switch( pos_msg.pos_type )
        {
            case DATA_ID_UP_PACKET_WIFI_RAW:
            {
                for( uint8_t i = 0; i < pos_msg.context_count; i++ )
                {
                    memcpy( offine_data_buf_temp + 9 + i * 7, &pos_msg.context.wifi_context[i].wifi_mac, 6 );
                    memcpy( offine_data_buf_temp + 9 + i * 7 + 6, &pos_msg.context.wifi_context[i].cur_rssi, 1 );
                }
                offine_data_len_temp = 9 + pos_msg.context_count * 7;
            }
            break;

            case DATA_ID_UP_PACKET_BLE_RAW:
            {
                for( uint8_t i = 0; i < pos_msg.context_count; i++ )
                {
                    memcpy( offine_data_buf_temp + 9 + i * 7, &pos_msg.context.beac_context[i].beac_mac, 6 );
                    memcpy( offine_data_buf_temp + 9 + i * 7 + 6, &pos_msg.context.beac_context[i].cur_rssi, 1 );
                }
                offine_data_len_temp = 9 + pos_msg.context_count * 7;
            }
            break;

            case DATA_ID_UP_PACKET_GNSS_RAW:
            {
                offine_data_buf_temp[1] = pos_msg.context.gps_context.zone_flag; // fragment data
                offine_data_buf_temp[4] = pos_msg.context.gps_context.gnss_len; // GNSS raw lenght
                memcpyr( offine_data_buf_temp + 2, ( uint8_t * )( &pos_msg.context.gps_context.group_id ), 2 ); // group id
                memcpy( offine_data_buf_temp + 5, pos_msg.context.gps_context.gnss_res, pos_msg.context.gps_context.gnss_len ); // GNSS raw data
                offine_data_len_temp = 5 + pos_msg.context.gps_context.gnss_len;
            }
            break;

            case DATA_ID_UP_PACKET_GNSS_END:
            {
                offine_data_buf_temp[9] = pos_msg.context.gps_context.zone_flag; // fragment data
                memcpyr( offine_data_buf_temp + 10, ( uint8_t * )( &pos_msg.context.gps_context.group_id ), 2 ); // group id
                offine_data_len_temp = 12;
            }
            break;

            case DATA_ID_UP_PACKET_POS_STATUS:
            {
                offine_data_len_temp = 9;
            }
            break;
            case DATA_ID_UP_PACKET_USER_SENSOR:
            {
                memcpy( &offine_data_buf_temp[1], ( uint8_t * )( pos_msg.context.sensor_context.sensor_data ), pos_msg.context.sensor_context.len );
                offine_data_len_temp = pos_msg.context.sensor_context.len+1;
            }
            break;
            case DATA_ID_UP_PACKET_FACT_SENSOR:
            {
                memcpy( &offine_data_buf_temp[1], ( uint8_t * )( pos_msg.context.sensor_context.sensor_data ), pos_msg.context.sensor_context.len );
                offine_data_len_temp = pos_msg.context.sensor_context.len+1;
            }
            break;            
            default:
            {
                int8_t ret  = wm1110_storage.deletePositionData( 1 );
                if(ret > 0)
                {

                }
                return false;
            }
            break;
        }

        smtc_modem_get_status( 0, &modem_status );
        uint8_t radio_busy_status;
        bool radio_is_busy = false;
        radio_is_busy = getLbmxRadioBusyState(&radio_busy_status);
        if(( modem_status & SMTC_MODEM_STATUS_JOINED ) == SMTC_MODEM_STATUS_JOINED &&( radio_is_busy == false ))
        {
            bool result = sendFrame( offine_data_buf_temp, offine_data_len_temp, true, false );
            if( result )
            {
                printf("Sending the cached packet: ");
                for(uint8_t u8i = 0; u8i < offine_data_len_temp; u8i++ )
                {
                    printf("%02x",offine_data_buf_temp[u8i]);
                }
                printf("\r\n"); 

                last_tx_timestamp = smtc_modem_hal_get_time_in_ms( );
                smtc_modem_get_toa_status( &data_tx_toa,offine_data_len_temp + 13 );
                check_tx_flag = true;
                tx_offline_data_flag = true;
                tx_confirmed_count_backup = getTxConfirmedCount( );
                return true;
            }
        }
    }
    else
    {
        printf( "Read old position data fail\r\n" );
        return false;
    }
    return false;
}

void WM1110_Geolocation::insertBootingMessageIntoQueue( void )
{
    uint8_t booting_data_buf_temp[64];
    uint8_t booting_data_len_temp;
    memset( booting_data_buf_temp, 0, sizeof( booting_data_buf_temp ));
    booting_data_len_temp = 0;

    booting_data_buf_temp[0] = DATA_ID_UP_PACKET_BOOT_DATA;
    booting_data_len_temp += 1;

    //battry  status
    booting_data_buf_temp[1] = 0x80;
    booting_data_len_temp += 1;

    //firmware Version
    booting_data_buf_temp[2] = TRACKER_SW_MAJOR_VER;
    booting_data_buf_temp[3] = TRACKER_SW_MINOR_VER;
    booting_data_len_temp += 2;

    //hardware Version
    booting_data_buf_temp[4] = TRACKER_HW_MAJOR_VER;
    booting_data_buf_temp[5] = TRACKER_HW_MINOR_VER;
    booting_data_len_temp += 2;

    //track scan type
    booting_data_buf_temp[6] = track_scan_type;
    booting_data_len_temp += 1;

    //read sensor data interval in minutes
    booting_data_buf_temp[7] = (sensor_period_time>>8)&0xFF;
    booting_data_buf_temp[8] = sensor_period_time&0xFF;
    booting_data_len_temp += 2;

    //track scan data interval in minutes
    booting_data_buf_temp[9] = (track_period_time>>8)&0xFF;
    booting_data_buf_temp[10] = track_period_time&0xFF;;
    booting_data_len_temp += 2;

    WM1110_GEOLOCATION_DBG_PRINTF("Booting datas insert:");
    for(uint8_t u8i = 0; u8i < booting_data_len_temp; u8i++ )
    {
        WM1110_GEOLOCATION_DBG_PRINTF("%02x",booting_data_buf_temp[u8i]);
    }
    WM1110_GEOLOCATION_DBG_PRINTF("\r\n"); 

    insertIntoTxQueue( booting_data_buf_temp, booting_data_len_temp, true, false );        

}

smtc_modem_region_t WM1110_Geolocation::getRegionFromDefault(void)
{
    smtc_modem_return_code_t rc = SMTC_MODEM_RC_OK;
    
    smtc_modem_region_t lorawan_region = SMTC_MODEM_REGION_EU_868; // can be change by user

    uint8_t lorawan_region_sub_band = 2; // can be change by user

    switch( app_param.lora_info.ActiveRegion )
    {
        case 0:
        case 15:
            lorawan_region = SMTC_MODEM_REGION_AS_923_GRP1;
        break;

        case 16:
            lorawan_region = SMTC_MODEM_REGION_AS_923_GRP2;
        break;

        case 17:
            lorawan_region = SMTC_MODEM_REGION_AS_923_GRP3;
        break;

        case 18:
            lorawan_region = SMTC_MODEM_REGION_AS_923_GRP4;
        break;
        
        case 1:
            lorawan_region = SMTC_MODEM_REGION_AU_915;
        break;
        
        case 5:
            lorawan_region = SMTC_MODEM_REGION_EU_868;
        break;
        
        case 6:
            lorawan_region = SMTC_MODEM_REGION_KR_920;
        break;
    
        case 7:
            lorawan_region = SMTC_MODEM_REGION_IN_865;
        break;
        
        case 8:
            lorawan_region = SMTC_MODEM_REGION_US_915;
        break;
    
        case 9:
            lorawan_region = SMTC_MODEM_REGION_RU_864;
        break;
        
        case 10:
            lorawan_region = SMTC_MODEM_REGION_AS_923_HELIUM_1;
        break;
        
        case 11:
            lorawan_region = SMTC_MODEM_REGION_AS_923_HELIUM_2;
        break;
        
        case 12:
            lorawan_region = SMTC_MODEM_REGION_AS_923_HELIUM_3;
        break;
        
        case 13:
            lorawan_region = SMTC_MODEM_REGION_AS_923_HELIUM_4;
        break;
        
        case 14:
            lorawan_region = SMTC_MODEM_REGION_AS_923_HELIUM_1B;
        break;
        
        default:
        break;
    }
    lorawan_region_sub_band =  app_param.lora_info.ChannelGroup + 1;
    if( lorawan_region == SMTC_MODEM_REGION_US_915 || lorawan_region == SMTC_MODEM_REGION_AU_915 )
    {
        rc = smtc_modem_set_region_sub_band( stack_id, lorawan_region_sub_band );
        if( rc != SMTC_MODEM_RC_OK )
        {
            HAL_DBG_TRACE_ERROR( "smtc_modem_set_region_sub_band failed (%d)\n", rc );
        }
    }

    return lorawan_region;
}

void WM1110_Geolocation::getDefaultProfileListByRegion(smtc_modem_region_t modem_region,uint8_t *buf)
{
    switch(modem_region)
    {
        case SMTC_MODEM_REGION_EU_868:
            memcpy(buf,adr_custom_list_eu868_default,16);
            break;

        case SMTC_MODEM_REGION_AS_923_GRP1:
        case SMTC_MODEM_REGION_AS_923_GRP2:
        case SMTC_MODEM_REGION_AS_923_GRP3:
        case SMTC_MODEM_REGION_AS_923_GRP4:
        case SMTC_MODEM_REGION_AS_923_HELIUM_1:
        case SMTC_MODEM_REGION_AS_923_HELIUM_2:
        case SMTC_MODEM_REGION_AS_923_HELIUM_3:
        case SMTC_MODEM_REGION_AS_923_HELIUM_4:
        case SMTC_MODEM_REGION_AS_923_HELIUM_1B:  
            memcpy(buf,adr_custom_list_as923_default,16);
            break;

        case SMTC_MODEM_REGION_US_915:
            memcpy(buf,adr_custom_list_us915_default,16);
            break;

        case SMTC_MODEM_REGION_AU_915:
            memcpy(buf,adr_custom_list_au915_default,16);
            break;

        case SMTC_MODEM_REGION_IN_865:
            memcpy(buf,adr_custom_list_in865_default,16);
            break;

        case SMTC_MODEM_REGION_RU_864:
            memcpy(buf,adr_custom_list_ru864_default,16);
            break;

        case SMTC_MODEM_REGION_KR_920:
            memcpy(buf,adr_custom_list_kr920_default,16);
            break;
        
        case SMTC_MODEM_REGION_CN_470_RP_1_0:
        case SMTC_MODEM_REGION_WW2G4:
        case SMTC_MODEM_REGION_CN_470:

        default:
            
            break;
    }
}

void WM1110_Geolocation::printLbmVersion(ralf_t* ralf)
{
#ifdef LR11XX
    lr11xx_system_version_t systemVersion;
    if (smtc_modem_suspend_before_user_radio_access() != SMTC_MODEM_RC_OK) abort();
    lr11xx_system_get_version(ralf->ral.context, &systemVersion);
    if (smtc_modem_resume_after_user_radio_access() != SMTC_MODEM_RC_OK) abort();
#endif

    smtc_modem_version_t modemVersion;
    if (smtc_modem_get_modem_version(&modemVersion) != SMTC_MODEM_RC_OK) abort();

    smtc_modem_lorawan_version_t lorawanVersion;
    if (smtc_modem_get_lorawan_version(&lorawanVersion) != SMTC_MODEM_RC_OK) abort();

    smtc_modem_lorawan_version_t rpVersion;
    if (smtc_modem_get_regional_params_version(&rpVersion) != SMTC_MODEM_RC_OK) abort();

    mw_version_t gnssVersion;
    if (gnss_mw_get_version(&gnssVersion) != MW_RC_OK) abort();

    mw_version_t wifiVersion;
    if (wifi_mw_get_version(&wifiVersion) != MW_RC_OK) abort();

    printf("Semtech LBM versions:\n");
#ifdef LR11XX
    printf("-LR11xx system    = hw:0x%02x, type:0x%02x, fw:0x%04x\n", systemVersion.hw, systemVersion.type, systemVersion.fw);
    printf("-LR11xx driver    = %s\n", lr11xx_driver_version_get_version_string());
#endif
    printf("-Modem            = v%d.%d.%d\n", modemVersion.major, modemVersion.minor, modemVersion.patch);
    printf("-LoRaWAN Stack    = v%d.%d.%d.%d\n", lorawanVersion.major, lorawanVersion.minor, lorawanVersion.patch, lorawanVersion.revision);
    printf("-Regional params  = v%d.%d.%d.%d\n", rpVersion.major, rpVersion.minor, rpVersion.patch, rpVersion.revision);
    printf("-GNSS             = v%d.%d.%d\n", gnssVersion.major, gnssVersion.minor, gnssVersion.patch);
    printf("-Wi-Fi            = v%d.%d.%d\n", wifiVersion.major, wifiVersion.minor, wifiVersion.patch);
}

bool WM1110_Geolocation::sendFrame( uint8_t* buffer, uint8_t length, bool tx_confirmed, bool emergency )
{
    uint8_t tx_max_payload;
    int32_t duty_cycle;

    /* Check if duty cycle is available */
    ASSERT_SMTC_MODEM_RC( smtc_modem_get_duty_cycle_status( &duty_cycle ));
    if( duty_cycle < 0 )
    {
        WM1110_GEOLOCATION_DBG_PRINTF( "Duty-cycle limitation - next possible uplink in %d ms \n\n", duty_cycle );
        return false;
    }
    
    ASSERT_SMTC_MODEM_RC( smtc_modem_get_next_tx_max_payload( stack_id, &tx_max_payload ));
    if( length > tx_max_payload )
    {
        WM1110_GEOLOCATION_DBG_PRINTF( "Not enough space in buffer - send empty uplink to flush MAC commands \n" );
        ASSERT_SMTC_MODEM_RC( smtc_modem_request_empty_uplink( stack_id, true, uplink_port, tx_confirmed ));
        return false;
    }
    else
    {
        smtc_modem_return_code_t result;
        uint8_t radio_busy_status;
        if( getLbmxRadioBusyState(&radio_busy_status ) == false)
        {
            printf( "Request uplink at %lu\n", smtc_modem_hal_get_time_in_ms( ));
            if( emergency )
            {
                result = smtc_modem_request_emergency_uplink( stack_id, uplink_port, tx_confirmed, buffer, length );
            }
            else
            {
                result = smtc_modem_request_uplink( stack_id, uplink_port, tx_confirmed, buffer, length );
            }

            ASSERT_SMTC_MODEM_RC( result );

            if( result == SMTC_MODEM_RC_OK )
            {
                return true;
            }
            else return false;
        }
        else
        {
            WM1110_GEOLOCATION_DBG_PRINTF( "Radio is not idle\n" );
            return false;
        }
    }
}

bool WM1110_Geolocation::insertIntoTxQueue( uint8_t *buf, uint8_t len, bool confirmed, bool emergency  )
{
    bool result = false;
    smtc_modem_status_mask_t modem_status;
    smtc_modem_get_status( 0, &modem_status );
    if(( modem_status & SMTC_MODEM_STATUS_JOINED ) == SMTC_MODEM_STATUS_JOINED )
    {     
        uint8_t in = tx_queue_in_index  % APP_TASK_LORA_TX_QUEUE_MAX;
        tx_queue_data_len[in] = len;
        tx_confirmed_queue_buf[in] = confirmed;
        for( uint8_t j = 0; j < len; j++ )
        {
            tx_queue_data_buf[in][j] = buf[j];
        }
        tx_queue_in_index = (tx_queue_in_index + 1) % APP_TASK_LORA_TX_QUEUE_MAX;
    }
    else
    {
        result = false;
    }

    return result;
}

void WM1110_Geolocation::modemLedActionProcess(void )
{
    switch (lora_run_state)
    {
        case LoRaWANStateType::Startup:
            ledOff(LED_BUILTIN);
            break;
        case LoRaWANStateType::Joining:
            if (millis() % 2000 < 100) ledOn(LED_BUILTIN); else ledOff(LED_BUILTIN);
            break;
        case LoRaWANStateType::Joined:
            if(led_flicker_start_time > 0)
            {
                digitalToggle(LED_BUILTIN);
                if((led_flicker_start_time+2000)<smtc_modem_hal_get_time_in_ms( ))
                {
                    ledOff(LED_BUILTIN);
                    led_flicker_start_time = 0;
                }
            }
            break;
        case LoRaWANStateType::TimeSynch:
            break;
        case LoRaWANStateType::JoinFailed:
            break;
        default:
            break;
    }    
}

void WM1110_Geolocation::insertTrackResultsIntoQueue( void ) 
{
    packTrackResults();
    for(uint8_t u8i = 0; u8i < GNSS_SCAN_GROUP_SIZE_MAX; u8i++)
    {
        if(track_scaned_data_len[u8i])
        {
            insertIntoTxQueue( track_scaned_data_buf[u8i], track_scaned_data_len[u8i], true, false ); 
            
            if(sensecap_platform)
            {
                switch(track_scaned_data_buf[u8i][0])
                {
                    case DATA_ID_UP_PACKET_POS_STATUS:
                        WM1110_GEOLOCATION_DBG_PRINTF("Location failure insert:");
                        break;
                    case DATA_ID_UP_PACKET_GNSS_RAW:
                        WM1110_GEOLOCATION_DBG_PRINTF("GNSS results data insert:");
                        break;
                    case DATA_ID_UP_PACKET_GNSS_END:
                        WM1110_GEOLOCATION_DBG_PRINTF("GNSS end data insert:");
                        break;
                    case DATA_ID_UP_PACKET_WIFI_RAW:
                        WM1110_GEOLOCATION_DBG_PRINTF("Wi-Fi results data insert:\r\n");
                        break;
                    case DATA_ID_UP_PACKET_BLE_RAW:
                        WM1110_GEOLOCATION_DBG_PRINTF("BLE results data insert:");
                        break;
                    default:
                        break;
                }
            }
            else
            {
                if(track_scan_type == Track_Scan_Gps)
                {
                    WM1110_GEOLOCATION_DBG_PRINTF("GNSS results data insert:");
                }
                else if(track_scan_type == Track_Scan_Wifi)
                {
                    WM1110_GEOLOCATION_DBG_PRINTF("Wi-Fi results data insert:");
                }
                else
                {
                    WM1110_GEOLOCATION_DBG_PRINTF("BLE results data insert:");
                }
            }
            for(uint8_t u8j = 0; u8j < track_scaned_data_len[u8i]; u8j++ )
            {
                WM1110_GEOLOCATION_DBG_PRINTF("%02x",track_scaned_data_buf[u8i][u8j]);
            }
            WM1110_GEOLOCATION_DBG_PRINTF("\r\n");            
        }
    }
}

void WM1110_Geolocation::printGnssScanedDatas(const gnss_mw_event_data_scan_done_t* data)
{
    if (data != nullptr)
    {
        printf("SCAN_DONE info:\n");
        printf("-- token: 0x%02X\n", data->token);
        printf("-- is_valid: %d\n", data->is_valid);
        printf("-- number of valid scans: %u\n", data->nb_scans_valid);
        for (uint8_t i = 0; i < data->nb_scans_valid; ++i)
        {
            printf("-- scan[%d][%lu] (%u SV - %d): ", i, data->scans[i].timestamp, data->scans[i].nb_svs, data->scans[i].nav_valid);
            for (uint8_t j = 0; j < data->scans[i].nav_size; ++j)
            {
                printf("%02X", data->scans[i].nav[j]);
            }
            printf("\n");
            for (uint8_t j = 0; j < data->scans[i].nb_svs; ++j)
            {
                printf("   SV_ID %u:\t%ddB\n", data->scans[i].info_svs[j].sv_id, data->scans[i].info_svs[j].cnr);
            }
        }
        printf("-- power consumption: %lu uah\n", data->power_consumption_uah);
        printf("-- mode: %d\n", data->context.mode);
        printf("-- assisted: %d\n", data->context.assisted);
        if (data->context.assisted)
        {
            printf("-- aiding position: (%.6f, %.6f)\n", data->context.aiding_position_latitude, data->context.aiding_position_longitude);
        }
        printf("-- almanac CRC: 0X%08lX\n", data->context.almanac_crc);
        printf("-- almanac update required: %d\n", data->context.almanac_update_required);
        printf("-- indoor detected: %d\n", data->indoor_detected);
        if (data->aiding_position_check_size > 0)
        {
            printf("-- APC (%u): ", data->aiding_position_check_size);
            for (uint8_t i = 0; i < data->aiding_position_check_size; ++i)
            {
                printf("%02X", data->aiding_position_check_msg[i]);
            }
            printf("\n");
        }
    }
}

void WM1110_Geolocation::restartClockSynchService( void )
{
    smtc_modem_status_mask_t modem_status;
    smtc_modem_get_status( 0, &modem_status );
    if(( modem_status & SMTC_MODEM_STATUS_JOINED ) == SMTC_MODEM_STATUS_JOINED )
    {
        smtc_modem_time_stop_sync_service( stack_id );
        smtc_modem_time_start_sync_service( stack_id, SMTC_MODEM_TIME_ALC_SYNC );
    }

}

void WM1110_Geolocation::modemEventHandler()
{
    static LbmxEvent event;
    while (event.fetch())
    {
        WM1110_Geolocation::getInstance().invoke(event);
    }    
}
#endif

#if 1 // protected function

void WM1110_Geolocation::reset(const LbmxEvent& event)
{
    printf("----- Reset LR1110 to start Join the network -----\n");
    if (LbmxEngine::setRegion(region) != SMTC_MODEM_RC_OK) abort();
    if (LbmxEngine::setOTAA(dev_eui, join_eui, app_key) != SMTC_MODEM_RC_OK) abort();

    if (smtc_modem_dm_set_info_interval(SMTC_MODEM_DM_INFO_INTERVAL_IN_DAY, 1) != SMTC_MODEM_RC_OK) abort();
    {
        const uint8_t infoField = SMTC_MODEM_DM_FIELD_ALMANAC_STATUS;
        if (smtc_modem_dm_set_info_fields(&infoField, 1) != SMTC_MODEM_RC_OK) abort();
    }

    printf("Request for Join the LoRaWAN network\n");
    if (LbmxEngine::joinNetwork() != SMTC_MODEM_RC_OK) abort();

    lora_run_state = LoRaWANStateType::Joining;
}

void WM1110_Geolocation::joined(const LbmxEvent& event)
{
    printf("----- JOINED -----\n");
    lora_run_state = LoRaWANStateType::Joined;

    uint8_t adr_custom_list_region[16] = { 0 };
    //Configure ADR, It is necessary to set up ADR,Tx useable payload must large than 51 bytes
    getDefaultProfileListByRegion(region,adr_custom_list_region);
    if (smtc_modem_adr_set_profile(0, SMTC_MODEM_ADR_PROFILE_CUSTOM, adr_custom_list_region) != SMTC_MODEM_RC_OK) abort();              //adr_custom_list_region  CUSTOM_ADR  
    
    if (smtc_modem_time_set_sync_interval_s(TIME_SYNC_VALID_TIME / 3) != SMTC_MODEM_RC_OK) abort();     // keep call order
    if (smtc_modem_time_set_sync_invalid_delay_s(TIME_SYNC_VALID_TIME) != SMTC_MODEM_RC_OK) abort();    // keep call order

    printf("Send uplink packet to request current time from Cloud.\n");
    if (smtc_modem_time_start_sync_service(0, SMTC_MODEM_TIME_ALC_SYNC) != SMTC_MODEM_RC_OK) abort();
    
    printf("Start the timer every %lds to check if there are data need to be sent.\n",UPLINK_PERIOD);
    if (LbmxEngine::startAlarm(FIRST_UPLINK_DELAY) != SMTC_MODEM_RC_OK) abort();

    printf("Set up the first boot packet after Joined\n");
    insertBootingMessageIntoQueue();
    led_flicker_start_time = smtc_modem_hal_get_time_in_ms( );

}

void WM1110_Geolocation::joinFail(const LbmxEvent& event)
{
    printf("----- Join fail -----\n");
    lora_run_state = LoRaWANStateType::JoinFailed;
}

void WM1110_Geolocation::time(const LbmxEvent& event)
{
    printf("----- Receieved downlink Time from Cloud -----\n");
    if (event.event_data.time.status == SMTC_MODEM_EVENT_TIME_NOT_VALID) return;

    static bool first = true;
    if (first)
    {
        lora_run_state = LoRaWANStateType::TimeSynch;
        if(time_sync_flag == false)
        {
            time_sync_flag = true;
        }
        printf("Sync time succeed:current time in UNIX format:%lu\r\n",apps_modem_common_get_utc_time( ));
        // Configure transmissions
        if (smtc_modem_set_nb_trans(0, 1) != SMTC_MODEM_RC_OK) abort();
        if (smtc_modem_connection_timeout_set_thresholds(0, 0, 0) != SMTC_MODEM_RC_OK) abort();
        first = false;
    }
}

void WM1110_Geolocation::alarm(const LbmxEvent& event)
{
    static uint8_t sync_time_interval = 0;
    printf("----- Timer for %lds -----\n",UPLINK_PERIOD);
    if(txProcess())
    {
        sync_time_interval = 0;
        ledOn(LED_BUILTIN);
    }
    else
    {
        sync_time_interval++;
        if((time_sync_flag == false) && (sync_time_interval >= 12))
        {
            printf("ReSend uplink packet to request current time from Cloud.\n");            
            restartClockSynchService();
            sync_time_interval = 0;
        }
    }
    if (LbmxEngine::startAlarm(UPLINK_PERIOD) != SMTC_MODEM_RC_OK) abort();
}

void WM1110_Geolocation::almanacUpdate(const LbmxEvent& event)
{
    printf( "----- Checking if the Almanac need to update -----\n" );
    if( event.event_data.almanac_update.status == SMTC_MODEM_EVENT_ALMANAC_UPDATE_STATUS_REQUESTED )
    {
        printf( "The stored Almanac is not completed: sending request to LoRa Cloud for Almanac update\n" );
    }
    else
    {
        printf( "The stored Almanac is completed\n" );
    }
}

void WM1110_Geolocation::txDone(const LbmxEvent& event)
{
    static uint32_t uplink_count = 0;

    printf( "----- Send LoRa Uplink Packet Done -----\n" );

    ledOff(LED_BUILTIN);
    if( event.event_data.txdone.status == SMTC_MODEM_EVENT_TXDONE_CONFIRMED )
    {
        incrementTxConfirmedCount();
    }
    uint32_t tick = smtc_modem_hal_get_time_in_s( );
    uint32_t confirmed_count = getTxConfirmedCount();
    printf( "Send LoRa Transmit Packet Done at RTC time %lu s,Uplink Packet: %lu,Confirm Ack Packet: %lu\r\n", tick, ++uplink_count, confirmed_count );        
}

void WM1110_Geolocation::downData(const LbmxEvent& event)
{
    uint8_t port;
    printf("Downlink received:\n");
    printf("  - LoRaWAN Fport = %d\n", event.event_data.downdata.fport);
    printf("  - Payload size  = %d\n", event.event_data.downdata.length);
    printf("  - RSSI          = %d dBm\n", event.event_data.downdata.rssi - 64);
    printf("  - SNR           = %d dB\n", event.event_data.downdata.snr / 4);

    if (event.event_data.downdata.length != 0)
    {
        port = event.event_data.downdata.fport;
        gnss_mw_handle_downlink(event.event_data.downdata.fport, event.event_data.downdata.data, event.event_data.downdata.length);
        if( port == uplink_port )
        {
            rx_data_len = event.event_data.downdata.length;
            memcpy( rx_data_buf, event.event_data.downdata.data, rx_data_len );
            decodeDownlinkData( rx_data_buf, rx_data_len );
            memset(rx_data_buf,0,rx_data_len);
            rx_data_len = 0;

        }
    }

}
 
void WM1110_Geolocation::gnssScanDone(const LbmxEvent& event)
{
    printf("----- The Semtech LBM finished scanning the GNSS -----\n");

    if(gnss_results_index>=4)
    {
        gnss_results_index = 0;
    }
    gnss_mw_get_event_data_scan_done(&WM1110_gnss_results[gnss_results_index]);

    // printGnssScanedDatas(&WM1110_gnss_results[gnss_results_index]);
    float latitude_dif, longitude_dif;
    latitude_dif = fabs( WM1110_gnss_results[gnss_results_index].context.aiding_position_latitude - gnss_aiding_position_latitude );
    longitude_dif = fabs( WM1110_gnss_results[gnss_results_index].context.aiding_position_longitude - gnss_aiding_position_longitude );

    /* Store the new assistance position only if the difference is greater than the conversion error */
    if(( latitude_dif > ( float ) 0.03 ) || ( longitude_dif > ( float ) 0.03 ))
    {
        gnss_aiding_position_latitude  = WM1110_gnss_results[gnss_results_index].context.aiding_position_latitude;
        gnss_aiding_position_longitude = WM1110_gnss_results[gnss_results_index].context.aiding_position_longitude;
        // TODO, save aiding position to nvds
        int32_t lat_temp = gnss_aiding_position_latitude * 1000000;
        int32_t long_temp = gnss_aiding_position_longitude * 1000000;
        printf( "Receieved downlink for newer aiding position:  latitude:%ld, longitude:%ld\r\n", lat_temp, long_temp );

    }

    if (WM1110_gnss_results[gnss_results_index].context.almanac_update_required)
    {
        const uint8_t dmAlmanacStatus = SMTC_MODEM_DM_FIELD_ALMANAC_STATUS;
        if (smtc_modem_dm_request_single_uplink(&dmAlmanacStatus, 1) != SMTC_MODEM_RC_OK) abort();
    }
    gnss_results_index++;
}

void WM1110_Geolocation::gnssTerminated(const LbmxEvent& event)
{
    printf("----- Preparing the scaned GNSS data -----\n");

    gnss_mw_event_data_terminated_t eventData;
    gnss_mw_get_event_data_terminated(&eventData);
    printf("The number of scanned raw GNSS dataset:\n");
    printf("-- Raw GNSS dataset: %u\n", eventData.nb_scans_sent);
    printf("-- If there is an an aiding position check message has been sent over the air: %s\n", eventData.aiding_position_check_sent == true ? "YES":"NO");

}

void WM1110_Geolocation::gnssScanCancelled(const LbmxEvent& event)
{
    printf("----- GNSS - %s -----\n", event.getGnssEventString(GNSS_MW_EVENT_TERMINATED).c_str());
}

void WM1110_Geolocation::gnssErrorNoTime(const LbmxEvent& event)
{
    printf("----- GNSS - %s -----\n", event.getGnssEventString(GNSS_MW_EVENT_ERROR_NO_TIME).c_str());

    if (smtc_modem_time_trigger_sync_request(0) != SMTC_MODEM_RC_OK) abort();
    mw_gnss_event_state = GNSS_MW_EVENT_ERROR_NO_TIME;

}

void WM1110_Geolocation::gnssErrorAlmanacUpdate(const LbmxEvent& event)
{
    printf("----- GNSS - %s -----\n", event.getGnssEventString(GNSS_MW_EVENT_ERROR_ALMANAC_UPDATE).c_str());

    const uint8_t dmAlmanacStatus = SMTC_MODEM_DM_FIELD_ALMANAC_STATUS;
    if (smtc_modem_dm_request_single_uplink(&dmAlmanacStatus, 1) != SMTC_MODEM_RC_OK) abort();
    mw_gnss_event_state = GNSS_MW_EVENT_ERROR_ALMANAC_UPDATE;
}

void WM1110_Geolocation::gnssErrorNoAidingPosition(const LbmxEvent& event)
{
    printf("----- GNSS - %s -----\n", event.getGnssEventString(GNSS_MW_EVENT_ERROR_ALMANAC_UPDATE).c_str());

}

void WM1110_Geolocation::gnssScanStopped(const LbmxEvent& event)
{
    tracker_scan_status = Track_End;
    gnss_mw_custom_clear_scan_busy();
}

void WM1110_Geolocation::wifiScanDone(const LbmxEvent& event)
{
    printf("----- Wi-Fi - %s -----\n", event.getWifiEventString(WIFI_MW_EVENT_SCAN_DONE).c_str());

    wifi_mw_get_event_data_scan_done(&WM1110_wifi_results);
    wifi_mw_display_results(&WM1110_wifi_results);
}

void WM1110_Geolocation::wifiTerminated(const LbmxEvent& event)
{
    printf("----- Wi-Fi - %s -----\n", event.getWifiEventString(WIFI_MW_EVENT_TERMINATED).c_str());

    wifi_mw_event_data_terminated_t eventData;
    wifi_mw_get_event_data_terminated(&eventData);
}

void WM1110_Geolocation::wifiScanCancelled(const LbmxEvent& event)
{
    printf("----- Wi-Fi - %s -----\n", event.getWifiEventString(WIFI_MW_EVENT_TERMINATED).c_str());
}

void WM1110_Geolocation::wifiErrorUnknown(const LbmxEvent& event)
{
    printf("----- Wi-Fi - %s -----\n", event.getWifiEventString(WIFI_MW_EVENT_TERMINATED).c_str());
}

void WM1110_Geolocation::wifiScanStopped(const LbmxEvent& event)
{
    tracker_scan_status = Track_End;
    wifi_mw_custom_clear_scan_busy();
}

void WM1110_Geolocation::decodeDownlinkData( uint8_t *buf, uint8_t len )
{
    uint8_t data_id = 0;

    if( buf && len )
    {
        data_id = buf[0];
        switch( data_id )
        {
            case DATA_ID_DOWN_PACKET_USER_CODE:
                {

                }
                break;

            default:
                break;
        }
    }    
}

#endif

