#include <Arduino.h>
#include "esp_camera.h"

// This "main.cpp" file is derived from following source code:
// https://github.com/espressif/arduino-esp32/tree/master/libraries/ESP32/examples/Camera/CameraWebServer
// Permitted for use under LGPL v2.1 
// https://github.com/espressif/arduino-esp32/blob/master/LICENSE.md

// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

// ===================
// Select camera model
// ===================
// #define CAMERA_MODEL_WROVER_KIT // Has PSRAM
// #define CAMERA_MODEL_ESP_EYE // Has PSRAM
// #define CAMERA_MODEL_ESP32S3_EYE // Has PSRAM
// #define CAMERA_MODEL_M5STACK_PSRAM // Has PSRAM
#define CAMERA_MODEL_M5STACK_V2_PSRAM // M5Camera version B Has PSRAM
// #define CAMERA_MODEL_M5STACK_WIDE // Has PSRAM
// #define CAMERA_MODEL_M5STACK_ESP32CAM // No PSRAM
// #define CAMERA_MODEL_M5STACK_UNITCAM // No PSRAM
// #define CAMERA_MODEL_AI_THINKER // Has PSRAM
// #define CAMERA_MODEL_TTGO_T_JOURNAL // No PSRAM
// #define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM
//  ** Espressif Internal Boards **
// #define CAMERA_MODEL_ESP32_CAM_BOARD
// #define CAMERA_MODEL_ESP32S2_CAM_BOARD
// #define CAMERA_MODEL_ESP32S3_CAM_LCD

#include "camera_pins.h"


// function declarations:
void setVFlip(boolean vFlip);
void setHMirror(boolean hMirror);
void setupCamera();
void setupCameraWithSize(framesize_t size);
void captureAndTransfer();

// Setup M5Camera
void setup()
{
  // Use "Serial" for logging
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  // Use "Serial2" to transfer captured JPEG
  // RX: G4, TX: G13
  Serial2.begin(115200, SERIAL_8N1, G4, G13);
}

// Main loop function called cyclic
void loop()
{
  if (Serial2.available() > 0)
  { // Listening Serial2 to receive commands from M5Stack
    String line = Serial2.readStringUntil('\n');
    Serial2.println(line); // Echo received commands

    if (line.startsWith("SETUP_SIZE:"))
    { // "SETUP_SIZE:" command used for set picture frame size
      // See framesize_t in sensor.h
      String sizeString = line.substring(strlen("SETUP_SIZE:"));
      framesize_t frameSize = (framesize_t)(sizeString.toInt());
      if (frameSize >= FRAMESIZE_INVALID)
      {
        frameSize = FRAMESIZE_QSXGA;
      }
      else if (frameSize == 0)
      {
        frameSize = FRAMESIZE_UXGA;
      }
      setupCameraWithSize(frameSize);
      Serial2.printf("setup completed with size:%d\n", frameSize);
    }

    else if (line.startsWith("CAPTURE:"))
    { // CAPTURE: command used to start capture and transfer data
      captureAndTransfer();
      Serial2.println("capture finished");
    }

    else if (line.startsWith("SHUTDOWN:"))
    { // SHUTDOWN: command not implemented
      Serial2.println("shutdown not implemented");
    }

    else if (line.startsWith("SETUP_VFLIP:"))
    { // SETUP_VFLIP: command to set vertical flip of camera
      String paramString = line.substring(strlen("SETUP_VFLIP:"));
      setVFlip(paramString == "TRUE");
    }

    else if (line.startsWith("SETUP_HMIRROR:"))
    { // SETUP_HMIRROR: command to set horizontal mirroring of camera
      String paramString = line.substring(strlen("SETUP_HMIRROR:"));
      setHMirror(paramString == "TRUE");
    }
  }
}

// function definitions:

// Set vertical flip
void setVFlip(boolean vFlip)
{
  sensor_t *s = esp_camera_sensor_get();
  s->set_vflip(s, vFlip ? 1 : 0);
}

// Set horizontal mirroring
void setHMirror(boolean hMirror)
{
  sensor_t *s = esp_camera_sensor_get();
  s->set_hmirror(s, hMirror ? 1 : 0);
}

// Setup camera with Default setting
void setupCamera()
{
  setupCameraWithSize(FRAMESIZE_UXGA);
}

// Setup camera with designated frame size
// See framesize_t in sensor.h
void setupCameraWithSize(framesize_t size)
{
  // Camera init
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = size;
  config.pixel_format = PIXFORMAT_JPEG; // for streaming
  // config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if (config.pixel_format == PIXFORMAT_JPEG)
  {
    if (psramFound())
    {
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    }
    else
    {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  }
  else
  {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed with error 0x%x", err);
    Serial2.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID)
  {
    s->set_vflip(s, 1);       // flip it back
    s->set_brightness(s, 1);  // up the brightness just a bit
    s->set_saturation(s, -2); // lower the saturation
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

// Setup LED FLash if LED pin is defined in camera_pins.h
#if defined(LED_GPIO_NUM)
  setupLedFlash(LED_GPIO_NUM);
#endif
}


// This "captureAndTransfer" function is derived from following source code:
// https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/Camera/CameraWebServer/app_httpd.cpp

// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// captureAndTransfer:
// 1. capture image
// 2. Send frame buffer size before trasfer JPEG image
// 3. Send frame buffer in JPEG format
void captureAndTransfer()
{
  // capture image
  camera_fb_t *fb = esp_camera_fb_get();
  if (fb == NULL)
  {
    Serial2.println("esp_camera_fb_get failed");
  }
  else
  {
    Serial2.printf("image size: %u\n", fb->len);
  }

  if (fb->format == PIXFORMAT_JPEG)
  {
    // JPEG_SIZE: command used to send frame buffer size before trasfer JPEG image
    Serial2.printf("JPEG_SIZE:%d\n", fb->len);

    // JPEG_START: command used to notify start transfer binary data after \n 
    Serial2.println("JPEG_START:builtin");
    byte *buffer = fb->buf;
    size_t bytesToSend = fb->len;
    while (bytesToSend > 0)
    {
      size_t index = fb->len - bytesToSend;
      size_t sentBytes = Serial2.write((const char *)(buffer + index), bytesToSend);
      bytesToSend -= sentBytes;
    }

    // Notify transfer finished
    Serial2.println("\n\njpeg sent");
  }
  else
  {
    // This camera can't encode JPEG in frame buffer
    Serial2.println("JPEG_FAILED:have to encode");
  }

  // return frame buffer after using
  esp_camera_fb_return(fb);
}


