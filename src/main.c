/****************************************************************************
http://retro.moe/unijoysticle2

Copyright 2019 Ricardo Quesada

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
****************************************************************************/

#include <stddef.h>

#include <btstack_port_esp32.h>
#include <btstack_run_loop.h>
#include <hci_dump.h>
#include <hci_dump_embedded_stdout.h>

#include "sdkconfig.h"
#include "uni_esp32.h"
#include "uni_main.h"

// #include <uni_platform_pc_debug.h>
#include <uni_log.h>
#include <uni_version.h>
#include <uni_property.h>
#include <uni_bt_setup.h>
#include <uni_console.h>
#include <uni_bluetooth.h>

#include "uni_platform_motor.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <driver/gpio.h>

static struct uni_platform *_platform;

#ifdef __cplusplus
extern "C"
{
#endif
    int app_main(void);
#ifdef __cplusplus
}
#endif


int app_main(void)
{
    
    // hci_dump_open(NULL, HCI_DUMP_STDOUT);

#ifdef CONFIG_BLUEPAD32_UART_OUTPUT_ENABLE
    uni_esp32_enable_uart_output(1);
#else
    // Adafruit Airlift modules have the UART RX/TX (GPIO 1 / 3) wired with the
    // controller so they can't be used for logging. In fact they can generate
    // noise and can break the communication with the controller.
    uni_esp32_enable_uart_output(0);
#endif

    // Configure BTstack for ESP32 VHCI Controller
    btstack_init();

    _platform = uni_platform_motor_create();

    // hci_dump_init(hci_dump_embedded_stdout_get_instance());

    // Init Bluepad32, and does not return.
    logi("Bluepad32 (C) 2016-2022 Ricardo Quesada and contributors.\n");
    logi("Version: v" UNI_VERSION "\n");

    // Honoring with BT copyright
    logi("BTStack: Copyright (C) 2017 BlueKitchen GmbH.\n");

    uni_property_init();
    // uni_platform_init(argc, argv);
    uni_hid_device_init();

    // Continue with bluetooth setup.
    uni_bt_setup();

    // #if CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE
    uni_console_init();
    // #endif  // CONFIG_BLUEPAD32_CONSOLE_ENABLE
    uni_bluetooth_enable_new_connections_safe(true);

    // Does not return.
    btstack_run_loop_execute();

    return 0;
}

struct uni_platform *uni_get_platform(void)
{
    return _platform;
}
