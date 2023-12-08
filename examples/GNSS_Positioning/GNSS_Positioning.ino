#include <Arduino.h>

#include <WM1110_Geolocation.hpp>
#include <WM1110_Storage.hpp>

// Set a execution period
static constexpr uint32_t EXECUTION_PERIOD = 50;    // [msec.]

// Instance
static WM1110_Geolocation& wm1110_geolocation = WM1110_Geolocation::getInstance();

void setup()
{
    // Initializes the debug output
    Serial.begin(115200);
    while (!Serial) delay(100);     // Wait for ready  

    // Initializes the storage area
    wm1110_storage.begin();
    wm1110_storage.loadBootConfigParameters(); // Load all parameters (WM1110_Param_Var.h)

    //The default location mode is GNSS, and uplink the data to the SenseCAP platform
    wm1110_geolocation.begin();

    // Start running
    wm1110_geolocation.run();
}

void loop()
{    
    // Run process 
    // sleepTime is the desired sleep time for LoRaWAN's next task
    uint32_t sleepTime = wm1110_geolocation.trackProcess();
    
    //delay
    delay(min(sleepTime, EXECUTION_PERIOD));
}

////////////////////////////////////////////////////////////////////////////////
