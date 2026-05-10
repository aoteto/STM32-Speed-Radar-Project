# Speed Radar

An open-source speed radar project built with the Arduino framework for the STM32 Nucleo-F446RE board.

The system measures the speed of an object passing between two IR sensors. It uses an OLED display, joystick, push buttons, RGB LED, buzzer, and EEPROM-based storage for measurement history and settings.

This project delivered for Yaşar University, EEE 3134 Microcontrollers course. You can see project report on /docs folder.

## Features

- Bidirectional speed measurement using two IR sensors
- Speed display in `cm/s` or `km/h`
- Adjustable sensor distance
- Adjustable speed limit
- Stores the latest 100 speed records in EEPROM
- Statistics screen with total, average, maximum, and over-limit count
- Histogram view for recorded speeds
- OLED menu interface
- RGB LED status feedback
- Buzzer alerts
- Hardware test screen
- Short-press reset and long-press standby mode using the F button

## Hardware

The project is configured for the `nucleo_f446re` board in PlatformIO.

Required parts:

- STM32 Nucleo-F446RE
- SH1107 SPI OLED display
- 2 IR obstacle sensors
- Joystick module
- Push buttons
- RGB LED
- Buzzer
- Resistors and jumper wires

## Pinout

| Component | Pin |
| --- | --- |
| OLED DC | PA8 |
| OLED RST | PB10 |
| OLED CS | PB6 |
| Joystick X | A0 / PA0 |
| Joystick Y | A1 / PA1 |
| Joystick SW | PA9 |
| Button A | PA10 |
| Button B | PB3 |
| Button C | PB5 |
| Button D | PB4 |
| Button F / Reset-Power | PB8 |
| IR sensor 1 | PC1 |
| IR sensor 2 | A3 |
| Buzzer | PC7 |
| RGB red | PC9 |
| RGB green | PC8 |
| RGB blue | PC6 |

Buttons are configured as `INPUT_PULLUP`, so each button should pull the pin to GND when pressed.

## Installation

1. Install PlatformIO.
2. Clone the repository:

```bash
git clone https://github.com/aoteto/STM32-Speed-Radar-Project.git
cd speed-radar
```

3. Connect the STM32 board to your computer over USB.
4. Build and upload the firmware:

```bash
pio run --target upload
```

5. Open the serial monitor:

```bash
pio device monitor
```

The serial monitor baud rate is `115200`.

The firmware uses the `Adafruit_SH110X` library for the OLED display. If the build fails with a missing SH110X dependency, add the Adafruit SH110X library to `lib_deps` in `platformio.ini`.

## Usage

After power-up, the device shows a short splash animation and then opens the main menu.

Use the joystick up/down directions to move through the menu. Press `C` to enter the selected menu and `B` to go back.

Main menu items:

- `SPEED RADAR`: speed measurement screen
- `STATISTICS`: saved measurement statistics
- `HARDWARE TEST`: button, joystick, and sensor test screen
- `SETTINGS`: distance, limit, unit, LED, buzzer, and memory reset settings

## Speed Measurement

1. Open the `SPEED RADAR` menu.
2. Press `C` to arm the radar.
3. When an object blocks the first IR sensor, timing starts.
4. When the object blocks the second IR sensor, timing stops.
5. Speed is calculated with:

```text
speed = sensor distance / elapsed time
```

The default sensor distance is `20 cm`. You can change this value in the settings menu.

If the second sensor is not triggered within 5 seconds, the measurement times out and must be started again.

## LED and Buzzer Feedback

- Main menu: green LED
- Sensor trigger: short blue flash
- Measurement below limit: white LED
- Measurement near the limit: orange LED
- Measurement above limit: red LED and buzzer alarm

LED and buzzer output can be enabled or disabled separately from the settings menu.

## Statistics

The `STATISTICS` screen shows:

- Total saved records
- Average speed
- Maximum speed
- Number of over-limit measurements

Press `A` to switch between text statistics and histogram view.

The system stores up to 100 measurements. After 100 records, new measurements overwrite the oldest ones.

## Settings

Use the joystick to move through the `SETTINGS` menu.

Available settings:

- `DIST`: distance between the two IR sensors
- `LIM`: speed limit
- `UNIT`: speed unit, `cm/s` or `km/h`
- `LED`: RGB LED on/off
- `BUZZER`: buzzer on/off
- `RESET MEMORY`: clears saved measurement history

Distance and speed limit are adjusted with joystick left/right. Unit, LED, buzzer, and memory reset are controlled with the `C` button.

Settings are written to the EEPROM buffer and flushed to persistent memory when leaving the radar or settings menu.

## F Button

The `F` button has two actions:

- Short press: software reset
- Long press: enter or exit standby mode

In standby mode, the OLED is turned off and LED/buzzer outputs are disabled.

## EEPROM Layout

The project stores records and settings in EEPROM.

| Address | Data |
| --- | --- |
| 0-3 | Record count |
| 4-7 | Next write index |
| 8-407 | 100 speed records |
| 408 | LED setting |
| 412 | Buzzer setting |
| 416 | Sensor distance |
| 420 | Speed limit |
| 424 | Speed unit |

## Development

Build:

```bash
pio run
```

Upload:

```bash
pio run --target upload
```

Serial monitor:

```bash
pio device monitor
```

Main libraries used by the firmware:

- Arduino framework
- SPI
- Adafruit GFX
- Adafruit SH110X
- EEPROM

## Contact

Email: contact@ozturkarda.com  
Website: https://www.ozturkarda.com

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Notes

- Set the sensor distance according to your physical build. An incorrect distance directly causes incorrect speed readings.
- Keep both IR sensors aligned and place them perpendicular to the object path for better accuracy.
- Ambient light can affect IR sensors. Adjust the sensor module threshold if needed.
