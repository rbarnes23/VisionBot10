/**
 * @ Author: Richard Barnes
 * @ Create Time: 2022-05-04 04:27:53
 * @ Modified by: Richard Barnes
 * @ Modified time: 2022-05-15 18:02:35
 * @ Modified time: 2022-06-28 03:56:30
 */
boolean firstReading = true;
// The TinyGPS++ object
#include <TinyGPS++.h>

TinyGPSPlus gps;
TaskHandle_t TaskGPS;
// Config config(geo);


void gpsWorker(void *arg)
{
    mavlink_gps_raw_int_t gpsMav;
    const uint16_t GPS_SAMPLERATE_DELAY_MS = 500;
    const TickType_t xInterval = pdMS_TO_TICKS(GPS_SAMPLERATE_DELAY_MS);
    GPSSerial.begin(GPSBAUD);
    vTaskDelay(xInterval);
    // Main Loop for Thread
    while (1)
    {
        while (GPSSerial.available() > 0)
            if (gps.encode(GPSSerial.read()))
            {
                if (gps.location.isUpdated())
                {
                    gpsMav.time_usec = micros(); // gps.time.age();
                    gpsMav.lat = (int)(gps.location.lat() * 1e7);
                    gpsMav.lon = (int)(gps.location.lng() * 1e7);
                    gpsMav.alt = gps.altitude.value();
                    gpsMav.eph = 0;
                    gpsMav.epv = gps.hdop.value();
                    gpsMav.vel = gps.speed.mps();
                    gpsMav.cog = gps.course.value();
                    gpsMav.fix_type = (uint8_t)(gps.altitude.meters() > 0) ? 3 : 2;
                    gpsMav.satellites_visible = gps.satellites.value();
                    
                }
            }
        if (millis() > 5000 && gps.charsProcessed() < 10)
        {
            Serial.println("No GPS detected: check wiring.");
            while (true)
                ;
        }
        if (gpsMav.fix_type > 0)
        {
            send_gps_msg(gpsMav);
        }

        // Delay between GPS samples
        vTaskDelay(xInterval);
    }
    /* delete a task when finish,
    this will never happen because this is infinity loop */
    vTaskDelete(NULL);
}

// Declare a semaphore handle.
void initGps()
{
    delay(1000);
    xTaskCreate(&gpsWorker, "GPS Task", 4096, NULL, 3, &TaskGPS);
}