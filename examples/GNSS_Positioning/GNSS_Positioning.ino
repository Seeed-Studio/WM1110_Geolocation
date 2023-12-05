#include <Arduino.h>

#include <WM1110_Geolocation.hpp>
#include <WM1110_Storage.hpp>

static constexpr uint32_t EXECUTION_PERIOD = 50;    // [msec.]

static WM1110_Geolocation& wm1110_geolocation = WM1110_Geolocation::getInstance();

void setup()
{
    Serial.begin(115200);
    while (!Serial) delay(100);  

    wm1110_storage.begin();
    wm1110_storage.loadBootConfigParameters();

    wm1110_geolocation.begin();

    wm1110_geolocation.run();
}

void loop()
{    

    uint32_t sleepTime = wm1110_geolocation.trackProcess();

    delay(min(sleepTime, EXECUTION_PERIOD));
}

////////////////////////////////////////////////////////////////////////////////
