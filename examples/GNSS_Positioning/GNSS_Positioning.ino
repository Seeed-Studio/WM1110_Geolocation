#include <Arduino.h>

#include <WM1110_Geolocation.hpp>
#include <WM1110_BLE.hpp>
#include <WM1110_Storage.hpp>
#include <WM1110_At_Config.hpp>
#include <Tracker_Peripheral.hpp>

static constexpr uint32_t EXECUTION_PERIOD = 50;    // [msec.]

static WM1110_Geolocation& wm1110_geolocation = WM1110_Geolocation::getInstance();

//receice cmd buf & size
uint8_t cmd_data_buf[244] = {0};
uint8_t cmd_data_size = 0;

//sensor 
float x; 
float y; 
float z;
float temperature; 
float humidity;

uint8_t buf[40];
uint8_t size = 0;

uint32_t start_sensor_read_time = 0; 
uint32_t sensor_read_priod = 0; 
void setup()
{
    Serial.begin(115200);
    while (!Serial) delay(100);  

    wm1110_storage.begin();
    wm1110_storage.loadBootConfigParameters();

    wm1110_ble.begin();
    wm1110_ble.setName();
    wm1110_ble.setStartParameters();
    wm1110_ble.startAdv();

    wm1110_geolocation.begin();

    tracker_peripheral.begin();

    wm1110_at_config.begin();

    sensor_read_priod = wm1110_geolocation.getSensorMeasurementPeriod();
    sensor_read_priod = sensor_read_priod*60*1000;

    wm1110_geolocation.run();
}

void loop()
{    
    uint32_t consume_time = 0;
    static uint32_t now_time = 0;
    static uint32_t start_sensor_read_time = 0; 

    uint32_t sleepTime = wm1110_geolocation.trackProcess();

    if(wm1110_geolocation.time_sync_flag)
    {
        if(sleepTime > 500)
        { 
            now_time = smtc_modem_hal_get_time_in_ms();
            if(now_time - start_sensor_read_time > sensor_read_priod ||(start_sensor_read_time == 0))
            {
                printf("Reading sensor data...\r\n");
                tracker_peripheral.measureLIS3DHTRDatas(&x,&y,&z);
                tracker_peripheral.measureSHT4xDatas(&temperature,&humidity);
                tracker_peripheral.packUplinkSensorDatas();
                tracker_peripheral.displaySensorDatas();
                tracker_peripheral.displayUplinkSensorDatas();
                tracker_peripheral.getUplinkSensorDatas( buf, &size );   
                // Insert all sensor data to lora tx buffer
                wm1110_geolocation.insertIntoTxQueue(buf,size);
                start_sensor_read_time = smtc_modem_hal_get_time_in_ms( );
                consume_time = start_sensor_read_time - now_time; 
                sleepTime = sleepTime - consume_time;
            }
        }
    }
    if(wm1110_ble.getBleRecData(cmd_data_buf,&cmd_data_size))
    {
        cmd_parse_type = 1;
        wm1110_at_config.parseCmd((char *)cmd_data_buf,cmd_data_size);
        memset(cmd_data_buf,0,cmd_data_size);
        cmd_data_size = 0;
        cmd_parse_type = 0;
    }
    if(wm1110_ble.getBleStatus() == BleRunState::StateDisconnect)
    {
        smtc_modem_hal_reset_mcu();
    }
    delay(min(sleepTime, EXECUTION_PERIOD));
}

////////////////////////////////////////////////////////////////////////////////
