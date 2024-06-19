# Custom Shelly Pro 2 App

This repository contains a custom app for the Shelly Pro 2. It is based on ESP IDF SDK.

The app uses Wifi, MQTT, OTA and OneWire for DS18B20 sensors. Ethernet via LAN8720 is not implemented because it is currently not required.

## Flashing

:heavy_exclamation_mark: The device ground is wired to the `L` connector, risk of shock due high voltage. :heavy_exclamation_mark:

I recommend to flash the device without power connected by using power supply via flash connector.

At the side of the module is a 7 pin 1.27 mm connector.
```
                 +------+
                 |      |
       +---------+      |
       |                |
+------+                |
|        # GND          |
|        # FLASH/GPIO0  |
|        # RESET        |
|        # 3.3 V        |
|        # RX           |
|        # TX           |
|        # 12 V         |
|                       |
|                       |
+------+                |
       |                |
       +---------+      |
                 |      |
                 +------+
```

## Hardware

Jumper 2: Input pins

|         |         |
|---------|---------|
|  3.3 V  | GPIO 38 |
| GPIO 39 |   GND   |

Jumper 3: Power supply

|      |     |
|------|-----|
| 12 V | GND |
| 12 V | GND |

Jumper 4: Relay 1 Connector

|       |         |         |         |         |         |
|-------|---------|---------|---------|---------|---------|
| 3.3 V | GPIO 12 | GPIO 14 | GPIO 13 | GPIO  0 | GPIO 16 |
|  GND  |    QA   | GPIO  2 | GPIO 36 |   12 V  | GPIO 32 |

* GPIO 36: 10k NTC (10k V-Div, B-Const: 3350)
* QA: SN74HC595B

Jumper 5: Relay 2 Connector

|       |         |         |         |         |         |
|-------|---------|---------|---------|---------|---------|
| 3.3 V | GPIO 12 | GPIO 14 | GPIO 13 | GPIO 15 | GPIO 34 |
|  GND  |    QB   | GPIO  2 | GPIO 37 |   12 V  | GPIO 33 |

* GPIO 37: 10k NTC (10k V-Div, B-Const: 3350)
* QB: SN74HC595B

## Hardware Mods

WIP