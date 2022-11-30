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

// Debug version

#include "uni_platform_motor.h"

#include <stdio.h>
#include <string.h>

#include "uni_bluetooth.h"
#include "uni_debug.h"
#include "uni_gamepad.h"
#include "uni_hid_device.h"

#include <Motor.h>
#include <motor_controller.h>

#include <driver/gpio.h>

//
// Globals
//
static int g_enhanced_mode = 0;
static int g_delete_keys = 0;

// PC Debug "instance"
typedef struct motor_instance_s
{
    uni_gamepad_seat_t gamepad_seat; // which "seat" is being used
} motor_instance_t;

// Declarations
static void trigger_event_on_gamepad(uni_hid_device_t *d);
static motor_instance_t *get_motor_instance(uni_hid_device_t *d);

static AF_DCMotor motor_left(1);
static AF_DCMotor motor_right(2);
static AF_DCMotor motor_weapon(3);

//
// Platform Overrides
//
static void motor_init(int argc, const char **argv)
{
    logi("motor: init()\n");
    for (int i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "--enhanced") == 0 || strcmp(argv[i], "-e") == 0)
        {
            g_enhanced_mode = 1;
            logi("Enhanced mode enabled\n");
        }
        if (strcmp(argv[i], "--delete") == 0 || strcmp(argv[i], "-d") == 0)
        {
            g_delete_keys = 1;
            logi("Stored keys will be deleted\n");
        }
    }

    logi("Starting...\n");

#if 0
    uni_gamepad_mappings_t mappings = GAMEPAD_DEFAULT_MAPPINGS;

    // Inverted axis with inverted Y in RY.
    mappings.axis_x = UNI_GAMEPAD_MAPPINGS_AXIS_RX;
    mappings.axis_y = UNI_GAMEPAD_MAPPINGS_AXIS_RY;
    mappings.axis_ry_inverted = true;
    mappings.axis_rx = UNI_GAMEPAD_MAPPINGS_AXIS_X;
    mappings.axis_ry = UNI_GAMEPAD_MAPPINGS_AXIS_Y;

    // Invert A & B
    mappings.button_a = UNI_GAMEPAD_MAPPINGS_BUTTON_B;
    mappings.button_b = UNI_GAMEPAD_MAPPINGS_BUTTON_A;

    uni_gamepad_set_mappings(&mappings);
#endif
}

static void motor_on_init_complete(void)
{
    gpio_config_t config = {};
    config.pin_bit_mask = (1ULL << GPIO_NUM_2);
    config.mode = GPIO_MODE_OUTPUT;
    config.pull_up_en = GPIO_PULLUP_DISABLE;
    config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    config.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&config);

    gpio_set_level(GPIO_NUM_2, 0);

    logi("motor: on_init_complete()\n");
}

static void motor_on_device_connected(uni_hid_device_t *d)
{
    logi("motor: device connected: %p\n", d);
}

static void motor_on_device_disconnected(uni_hid_device_t *d)
{
    logi("motor: device disconnected: %p\n", d);

    motor_left.run(RELEASE);
    motor_left.setSpeed(0);
    motor_right.run(RELEASE);
    motor_right.setSpeed(0);
    motor_weapon.run(RELEASE);
    motor_weapon.setSpeed(0);
    gpio_set_level(GPIO_NUM_2, 0);
}

static int motor_on_device_ready(uni_hid_device_t *d)
{
    logi("motor: device ready: %p\n", d);
    motor_instance_t *ins = get_motor_instance(d);
    ins->gamepad_seat = GAMEPAD_SEAT_A;
    gpio_set_level(GPIO_NUM_2, 1);
    trigger_event_on_gamepad(d);
    return 0;
}

static void motor_on_gamepad_data(uni_hid_device_t *d, uni_gamepad_t *gp)
{
    static uint8_t leds = 0;
    static uint8_t enabled = true;
    static uni_gamepad_t prev = {0};

    if (memcmp(&prev, gp, sizeof(*gp)) == 0)
    {
        return;
    }
    prev = *gp;
    // Print device Id before dumping gamepad.
    logi("(%p) ", d);
    uni_gamepad_dump(gp);

    // Debugging
    // Axis ry: control rumble
    if ((gp->buttons & BUTTON_A) && d->report_parser.set_rumble != NULL)
    {
        d->report_parser.set_rumble(d, 128, 128);
    }
    // Buttons: Control LEDs On/Off
    if ((gp->buttons & BUTTON_B) && d->report_parser.set_player_leds != NULL)
    {
        d->report_parser.set_player_leds(d, leds++ & 0x0f);
    }
    // Axis: control RGB color
    if ((gp->buttons & BUTTON_X) && d->report_parser.set_lightbar_color != NULL)
    {
        uint8_t r = (gp->axis_x * 256) / 512;
        uint8_t g = (gp->axis_y * 256) / 512;
        uint8_t b = (gp->axis_rx * 256) / 512;
        d->report_parser.set_lightbar_color(d, r, g, b);
    }

    struct direction_t motor_direction;
    calculate_motor_directions(&motor_direction, gp->axis_x, gp->axis_y);
    if (motor_direction.left > 0)
    {
        motor_left.run(FORWARD);
    }
    else if (motor_direction.left < 0)
    {
        motor_left.run(BACKWARD);
    }
    motor_left.setSpeed(abs(motor_direction.left));

    if (motor_direction.right > 0)
    {
        motor_right.run(FORWARD);
    }
    else if (motor_direction.right < 0)
    {
        motor_right.run(BACKWARD);
    }
    motor_right.setSpeed(abs(motor_direction.right));

    int16_t weapon = 0;
    if ((gp->buttons & 0x02) != 0)
    {
        weapon = 255;
    }
    else if ((gp->buttons & 0x04) != 0)
    {
        weapon = -255;
    }
    printf("weapon %d\n", weapon);
    if (weapon > 0)
    {
        motor_weapon.run(FORWARD);
    }
    else if (weapon < 0)
    {
        motor_weapon.run(BACKWARD);
    }
    motor_weapon.setSpeed(abs(weapon));
}

static int32_t motor_get_property(uni_platform_property_t key)
{
    logi("motor: get_property(): %d\n", key);
    if (key != UNI_PLATFORM_PROPERTY_DELETE_STORED_KEYS)
        return -1;
    return g_delete_keys;
}

static void motor_on_oob_event(uni_platform_oob_event_t event, void *data)
{
    logi("motor: on_device_oob_event(): %d\n", event);

    if (event != UNI_PLATFORM_OOB_GAMEPAD_SYSTEM_BUTTON)
    {
        logi("motor_on_device_gamepad_event: unsupported event: 0x%04x\n", event);
        return;
    }

    uni_hid_device_t *d = (uni_hid_device_t *)data;

    if (d == NULL)
    {
        loge("ERROR: motor_on_device_gamepad_event: Invalid NULL device\n");
        return;
    }

    motor_instance_t *ins = get_motor_instance(d);
    ins->gamepad_seat = ins->gamepad_seat == GAMEPAD_SEAT_A ? GAMEPAD_SEAT_B : GAMEPAD_SEAT_A;

    trigger_event_on_gamepad(d);
}

//
// Helpers
//
static motor_instance_t *get_motor_instance(uni_hid_device_t *d)
{
    return (motor_instance_t *)&d->platform_data[0];
}

static void trigger_event_on_gamepad(uni_hid_device_t *d)
{
    motor_instance_t *ins = get_motor_instance(d);

    if (d->report_parser.set_rumble != NULL)
    {
        d->report_parser.set_rumble(d, 0x80 /* value */, 15 /* duration */);
    }

    if (d->report_parser.set_player_leds != NULL)
    {
        d->report_parser.set_player_leds(d, ins->gamepad_seat);
    }

    if (d->report_parser.set_lightbar_color != NULL)
    {
        uint8_t red = (ins->gamepad_seat & 0x01) ? 0xff : 0;
        uint8_t green = (ins->gamepad_seat & 0x02) ? 0xff : 0;
        uint8_t blue = (ins->gamepad_seat & 0x04) ? 0xff : 0;
        d->report_parser.set_lightbar_color(d, red, green, blue);
    }
}

//
// Entry Point
//
struct uni_platform *uni_platform_motor_create(void)
{
    static struct uni_platform plat = {};

    plat.name = "Motor";
    plat.init = motor_init;
    plat.on_init_complete = motor_on_init_complete;
    plat.on_device_connected = motor_on_device_connected;
    plat.on_device_disconnected = motor_on_device_disconnected;
    plat.on_device_ready = motor_on_device_ready;
    plat.on_oob_event = motor_on_oob_event;
    plat.on_gamepad_data = motor_on_gamepad_data;
    plat.get_property = motor_get_property;

    return &plat;
}
