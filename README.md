| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-C61 | ESP32-H2 | ESP32-H21 | ESP32-P4 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | --------- | -------- | --------- | -------- | -------- | -------- |

# Blink Example

(See the README.md file in the upper level 'examples' directory for more information about examples.)

This example demonstrates how to blink a LED by using the GPIO driver or using the [led_strip](https://components.espressif.com/component/espressif/led_strip) library if the LED is addressable e.g. [WS2812](https://cdn-shop.adafruit.com/datasheets/WS2812B.pdf). The `led_strip` library is installed via [component manager](main/idf_component.yml).

For the ESP32-S3 configuration in this project, one external WS2812 is driven
from GPIO48 using the RMT peripheral. It cycles through red, green, blue, white,
and off once per second.

The project also contains a modular TB6612 DC motor example:

* `GPIO16` -> TB6612 `AIN1`
* `GPIO17` -> TB6612 `AIN2`
* `GPIO18` -> TB6612 `PWMA`
* `GPIO41` <- encoder phase `A`
* `GPIO42` <- encoder phase `B`
* TB6612 `STBY` is assumed to be tied to `3.3V`

The demo uses a 20 kHz PWM signal and runs a low/medium forward and reverse
sequence with a coast interval before each direction change. Encoder speed is
reported as counts per second and mapped to the WS2812: stopped is off, low
speed is blue, medium speed is green, and high speed is red.

The TB6612 driver is a reusable instance-based ESP-IDF component. Create a
`tb6612_t` object, initialize that object once, and pass its address to every
later operation:

```c
#include "tb6612.h"

static tb6612_t motor = TB6612_CONFIG_DEFAULT_WITH_ENCODER(
    GPIO_NUM_16, GPIO_NUM_17, GPIO_NUM_18,
    GPIO_NUM_41, GPIO_NUM_42,
    0 // set quadrature counts per revolution to enable RPM
);

ESP_ERROR_CHECK(tb6612_init(&motor));
ESP_ERROR_CHECK(tb6612_set_speed(&motor, 500));  // forward, 50%

tb6612_speed_t speed;
ESP_ERROR_CHECK(tb6612_read_speed(&motor, &speed));
printf("speed: %.1f counts/s\n", speed.counts_per_second);
if (speed.rpm_valid) {
    printf("speed: %.1f RPM\n", speed.rpm);
}

ESP_ERROR_CHECK(tb6612_coast(&motor));
ESP_ERROR_CHECK(tb6612_set_speed(&motor, -300)); // reverse, 30%
ESP_ERROR_CHECK(tb6612_brake(&motor));
ESP_ERROR_CHECK(tb6612_deinit(&motor));
```

To use more than one motor, create another `tb6612_t` and assign it a different
LEDC channel. A timer may be shared only when PWM frequency and resolution are
the same. Call `tb6612_read_speed()` periodically; the library measures the
real elapsed time between calls. Set `encoder_counts_per_revolution` to the
four-edge quadrature count per output-shaft revolution when RPM is required.

## How to Use Example

Before project configuration and build, be sure to set the correct chip target using `idf.py set-target <chip_name>`.

### Hardware Required

* A development board with normal LED or addressable LED on-board (e.g., ESP32-S3-DevKitC, ESP32-C6-DevKitC etc.)
* A USB cable for Power supply and programming

For the included ESP32-S3 WS2812 example, connect:

* `GPIO48` -> WS2812 `DIN`
* `5V` (or the voltage required by your module) -> WS2812 `VCC`
* `GND` -> WS2812 `GND` (the ESP32-S3 and WS2812 must share ground)

Power the motor from a suitable external supply through the TB6612. The motor
supply, TB6612 logic supply, ESP32-S3, encoder, and WS2812 must share ground.
Do not power the motor from an ESP32-S3 GPIO or from the board's 3.3V regulator.

See [Development Boards](https://www.espressif.com/en/products/devkits) for more information about it.

### Configure the Project

Open the project configuration menu (`idf.py menuconfig`).

In the `Example Configuration` menu:

* Select the LED type in the `Blink LED type` option.
  * Use `GPIO` for regular LED
  * Use `LED strip` for addressable LED
* If the LED type is `LED strip`, select the backend peripheral
  * `RMT` is only available for ESP targets with RMT peripheral supported
  * `SPI` is available for all ESP targets
* Set the GPIO number used for the signal in the `Blink GPIO number` option.
* Set the blinking period in the `Blink period in ms` option.

### Build and Flash

Run `idf.py -p PORT flash monitor` to build, flash and monitor the project.

(To exit the serial monitor, type ``Ctrl-]``.)

See the [Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for full steps to configure and use ESP-IDF to build projects.

## Example Output

As you run the example, you will see the LED blinking, according to the previously defined period. For the addressable LED, you can also change the LED color by setting the `led_strip_set_pixel(led_strip, 0, 16, 16, 16);` (LED Strip, Pixel Number, Red, Green, Blue) with values from 0 to 255 in the [source file](main/blink_example_main.c).

```text
I (315) example: Example configured to blink addressable LED!
I (325) example: Turning the LED OFF!
I (1325) example: Turning the LED ON!
I (2325) example: Turning the LED OFF!
I (3325) example: Turning the LED ON!
I (4325) example: Turning the LED OFF!
I (5325) example: Turning the LED ON!
I (6325) example: Turning the LED OFF!
I (7325) example: Turning the LED ON!
I (8325) example: Turning the LED OFF!
```

Note: The color order could be different according to the LED model.

The pixel number indicates the pixel position in the LED strip. For a single LED, use 0.

## Troubleshooting

* If the LED isn't blinking, check the GPIO or the LED type selection in the `Example Configuration` menu.

For any technical queries, please open an [issue](https://github.com/espressif/esp-idf/issues) on GitHub. We will get back to you soon.
