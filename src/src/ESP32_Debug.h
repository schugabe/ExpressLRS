#pragma once

#include "LoRaRadioLib.h"

extern SX127xDriver Radio;

void DEBUG_PrintRadioPacketStats()
{
    Serial.print("SPItime: ");
    Serial.println(Radio.TXspiTime);
    Serial.print("TotalTime: ");
    Serial.println(Radio.TimeOnAir);
    Serial.print("HeadRoom: ");
    Serial.println(Radio.HeadRoom);
    Serial.print("PacketCount(HZ): ");
    Serial.println(Radio.PacketCount);
}

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_task_wdt.h"

TaskHandle_t xidle_task;

static int idle_cnt = 0;

static void idle_task(void *parm)

{
    //esp_task_wdt_delete(xidle_task); //remove task from the WDT
    //esp_task_wdt_delete(xTaskGetIdleTaskHandleForCPU(0));
    //esp_task_wdt_delete(xTaskGetIdleTaskHandleForCPU(1));

    while (1 == 1)
    {
        int64_t now = esp_timer_get_time(); // time anchor
        vTaskDelay(0 / portTICK_RATE_MS);
        int64_t now2 = esp_timer_get_time();
        idle_cnt += (now2 - now) / 1000; // diff
    }
}

static void mon_task(void *parm)

{
    while (1 == 1)
    {
        // Note the trick of saving it on entry, so print time
        // is not added to our timing.

        float new_cnt = (float)idle_cnt; // Save the count for printing it ...

        // Compensate for the 100 ms delay artifact: 900 ms = 100%
        float cpu_percent = ((99.9 / 90.) * new_cnt) / 10;
        printf("%.0f%%  ", 100 - cpu_percent);
        fflush(stdout);
        idle_cnt = 0; // Reset variable
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}