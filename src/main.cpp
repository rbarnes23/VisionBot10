/**
 * @ Author: Richard Barnes
 * @ Create Time: 2022-06-27 04:28:41
 * @ Modified by: Richard Barnes
 * @ Modified time: 2022-06-28 04:55:41
 * @ Description:
 */

#include <Arduino.h>
#include "globals.h"
#include <ArduinoJson.h>
#include <Wire.h>

using namespace std;

Point2d getPoint(uint16_t i)
{
  return _points[i];
}

// Find the last item on the path
int getEnd()
{
  return _points.size();
}

size_t addPoint(Point2d point)
{
  _points.push_back(point);
  return _points.size();
}

void SerialWorker(void *arg)
{
  const uint16_t SERIAL_DELAY_MS = 20; 
  TickType_t xInterval = pdMS_TO_TICKS(SERIAL_DELAY_MS);
  TickType_t xLastWakeTime;

  while (!telem)
  {
  }
  xLastWakeTime = xTaskGetTickCount();
  uint16_t cnt = 0;
  uint8_t wp_complete = '0';
  while (1)
  {
    while (telem.available())
    {
      if (wp_complete == '0')
      {
        _points.clear();
        StaticJsonDocument<8192> jsonDoc;
        DeserializationError error = deserializeJson(jsonDoc, telem);
        if (error)
        {
          Serial.println(error.c_str());
          return;
        }
        uint16_t jsonsize = jsonDoc["mission"]["items"].size();
        double hpLat = jsonDoc["mission"]["plannedHomePosition"][0];
        double hpLon = jsonDoc["mission"]["plannedHomePosition"][1];
        for (int i = 0; i < jsonsize; i++)
        {
          int no = jsonDoc["mission"]["items"][i]["doJumpId"];
          double lat = jsonDoc["mission"]["items"][i]["params"][4];
          double lon = jsonDoc["mission"]["items"][i]["params"][5];
          if (no == 0)
          {
            lat = hpLat;
            lon = hpLon;
          }
          double alt = jsonDoc["mission"]["items"][0]["Altitude"];
          uint8_t altMode = jsonDoc["mission"]["items"][0]["AltitudeMode"];
          uint8_t cmd = jsonDoc["mission"]["items"][0]["command"];
          // Point2d point;
          Point2d point = make_pair(lat, lon);
          size_t pSize = addPoint(point);
          Serial.println("Lat: " + String(point.first) + "Lon: " + String(point.second));
        }
        cnt++;
        Serial.printf("CNT: %i\n", cnt);
        Serial.println("HPLat: " + String(hpLat) + " HPLon: " + String(hpLon));
        telem.write("1");
        wp_complete = '1';
      }
      vTaskDelayUntil(&xLastWakeTime, xInterval);
    }
  }
}

void setup()
{
  Serial.begin(115200);
  // Setup Serial Connection
  telem.begin(115200);
  //Wire.setSDA(18);
  //Wire.setSCL(19);

  //xTaskCreate(&SerialWorker, NULL, 4096, NULL, 2, NULL);
  xTaskCreate(heartbeat, NULL, 2048, NULL, 5, NULL);
  initGps();
  //xTaskCreate(send_parameters, NULL, 4096, NULL,0, NULL);
  vTaskStartScheduler();
}

void loop()
{
}