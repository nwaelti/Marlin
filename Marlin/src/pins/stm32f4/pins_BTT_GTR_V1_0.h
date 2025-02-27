 /**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

#include "env_validate.h"

#if E_STEPPERS > MAX_E_STEPPERS
  #error "Marlin extruder/hotends limit! Increase MAX_E_STEPPERS to continue."
#elif HOTENDS > 8 || E_STEPPERS > 8
  #error "BIGTREE GTR V1.0 supports up to 8 hotends / E steppers."
#endif

#define BOARD_INFO_NAME "BTT GTR V1.0"

#define USES_DIAG_JUMPERS
#define HAS_OTG_USB_HOST_SUPPORT                  // USB Flash Drive support
#define M5_EXTENDER                               // The M5 extender is attached
// #define NUM_SERVO_PLUGS 5

// Onboard I2C EEPROM
#define I2C_EEPROM
#define MARLIN_EEPROM_SIZE                        0x2000  // 8K (24C64)

//
// Servos
//
#define SERVO0_PIN                             PB11  // BLTOUCH
// #define SOL0_PIN                            PC7   // Toolchanger
// #define SERVO1_PIN                          PI5   // SERVO a
// #define SERVO2_PIN                          PE9   // SERVO b
// #define SERVO3_PIN                          PE11  // SERVO c

#if ENABLED(TOOL_SENSOR)
  #define TOOL_SENSOR1_PIN                  PH6
  #define TOOL_SENSOR2_PIN                  PI4
  //#define TOOL_SENSOR3_PIN                PF4
#else
  #define PS_ON_PIN                         PH6
#endif

//
// Trinamic Stallguard pins
//
#define X_DIAG_PIN                          PF2   // X-
#define Y_DIAG_PIN                          PC13  // Y-
#define Z_DIAG_PIN                          PE0   // Z-
#define Y2_DIAG_PIN                         PG14  // X+
#define Z2_DIAG_PIN                         PG9   // Y+
#define I_DIAG_PIN                          PD3   // Z+

#define J_DIAG_PIN                          PI4   // Y2-
#define K_DIAG_PIN                          PF4   // Z2-
#define E0_DIAG_PIN                         PF6   // M3
#define E1_DIAG_PIN                         PI7   // M4
#define E2_DIAG_PIN                         PF12  // M5

//
// Limit Switches
//
#ifdef X_STALL_SENSITIVITY
  #define X_STOP_PIN                  X_DIAG_PIN
#endif

#ifdef Y_STALL_SENSITIVITY
  #define Y_STOP_PIN                  Y_DIAG_PIN
#endif

#ifdef Z_STALL_SENSITIVITY
  #define Z_STOP_PIN                  Z_DIAG_PIN
#endif

#ifdef K_STALL_SENSITIVITY
  #define K_STOP_PIN                  K_DIAG_PIN
#endif

//
// Pins on the extender
//
#if ENABLED(M5_EXTENDER)
  // #define X2_STOP_PIN                       PI4   // M5 M1_STOP
  // #define K_STOP_PIN                       PF12  // M5 M5_STOP
  // #define Z2_STOP_PIN                       PF4   // M5 M2_STOP
  // #define Z3_STOP_PIN                       PI7   // M5 M4_STOP
  // #define Z4_STOP_PIN                       PF6   // M5 M3_STOP
#endif

#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                   PH11  // Z Probe must be PH11
#endif

//
// Steppers
//
#define X_STEP_PIN                          PC15
#define X_DIR_PIN                           PF0
#define X_ENABLE_PIN                        PF1
#ifndef X_CS_PIN
  #define X_CS_PIN                          PC14
#endif

#define Y_STEP_PIN                          PE3
#define Y_DIR_PIN                           PE2
#define Y_ENABLE_PIN                        PE4
#ifndef Y_CS_PIN
  #define Y_CS_PIN                          PE1
#endif

#define Z_STEP_PIN                          PB8
#define Z_DIR_PIN                           PB7   // PB7
#define Z_ENABLE_PIN                        PB9
#ifndef Z_CS_PIN
  #define Z_CS_PIN                          PB5
#endif

#define Y2_STEP_PIN                      PG12
#define Y2_DIR_PIN                       PG11
#define Y2_ENABLE_PIN                    PG13
#ifndef Y2_CS_PIN
  #define Y2_CS_PIN                      PG10
#endif

#define Z2_STEP_PIN                      PD6
#define Z2_DIR_PIN                       PD5
#define Z2_ENABLE_PIN                    PD7
#ifndef Z2_CS_PIN
  #define Z2_CS_PIN                      PD4
#endif

#define I_STEP_PIN                       PD1
#define I_DIR_PIN                        PD0
#define I_ENABLE_PIN                     PD2
#ifndef I_CS_PIN
  #define I_CS_PIN                       PC12
#endif

#if ENABLED(M5_EXTENDER)

  #define J_STEP_PIN                          PF3
  #define J_DIR_PIN                           PG3
  #define J_ENABLE_PIN                        PF8
  #ifndef J_CS_PIN
    #define J_CS_PIN                          PG4
  #endif

  #define K_STEP_PIN                          PD14
  #define K_DIR_PIN                           PD11
  #define K_ENABLE_PIN                        PG2
  #ifndef K_CS_PIN
    #define K_CS_PIN                          PE15
  #endif

  #define E0_STEP_PIN                         PE12
  #define E0_DIR_PIN                          PE10
  #define E0_ENABLE_PIN                       PF14
  #ifndef E0_CS_PIN
    #define E0_CS_PIN                         PE7
  #endif

  #define E1_STEP_PIN                       PG0
  #define E1_DIR_PIN                        PG1
  #define E1_ENABLE_PIN                     PE8
  #ifndef E1_CS_PIN
    #define E1_CS_PIN                       PF15
  #endif

  #define E2_STEP_PIN                       PH12
  #define E2_DIR_PIN                        PH15
  #define E2_ENABLE_PIN                     PI0
  #ifndef E2_CS_PIN
    #define E2_CS_PIN                       PH14
  #endif

#endif

//
// Software SPI pins for TMC2130 stepper drivers
//
#if ENABLED(TMC_USE_SW_SPI)
  #ifndef TMC_SW_MOSI
    #define TMC_SW_MOSI                     PG15
  #endif
  #ifndef TMC_SW_MISO
    #define TMC_SW_MISO                     PB6
  #endif
  #ifndef TMC_SW_SCK
    #define TMC_SW_SCK                      PB3
  #endif
#endif

#if HAS_TMC_UART
  /**
   * TMC2208/TMC2209 stepper drivers
   *
   * Hardware serial communication ports.
   * If undefined software serial is used according to the pins below
   */
  //#define X_HARDWARE_SERIAL  Serial1
  //#define X2_HARDWARE_SERIAL Serial1
  //#define Y_HARDWARE_SERIAL  Serial1
  //#define Y2_HARDWARE_SERIAL Serial1
  //#define Z_HARDWARE_SERIAL  Serial1
  //#define Z2_HARDWARE_SERIAL Serial1
  //#define E0_HARDWARE_SERIAL Serial1
  //#define E1_HARDWARE_SERIAL Serial1
  //#define E2_HARDWARE_SERIAL Serial1
  //#define E3_HARDWARE_SERIAL Serial1  // M5 MOTOR 1
  //#define E4_HARDWARE_SERIAL Serial1  // M5 MOTOR 2
  //#define E5_HARDWARE_SERIAL Serial1  // M5 MOTOR 3
  //#define E6_HARDWARE_SERIAL Serial1  // M5 MOTOR 4
  //#define E7_HARDWARE_SERIAL Serial1  // M5 MOTOR 5

  #define X_SERIAL_TX_PIN                   PC14
  #define X_SERIAL_RX_PIN        X_SERIAL_TX_PIN

  #define Y_SERIAL_TX_PIN                   PE1
  #define Y_SERIAL_RX_PIN        Y_SERIAL_TX_PIN

  #define Z_SERIAL_TX_PIN                   PB5
  #define Z_SERIAL_RX_PIN        Z_SERIAL_TX_PIN

  #define Y2_SERIAL_TX_PIN                PG10
  #define Y2_SERIAL_RX_PIN    Y2_SERIAL_TX_PIN

  #define Z2_SERIAL_TX_PIN                PD4
  #define Z2_SERIAL_RX_PIN    Z2_SERIAL_TX_PIN

  #define I_SERIAL_TX_PIN                PC12
  #define I_SERIAL_RX_PIN     I_SERIAL_TX_PIN

  #if ENABLED(M5_EXTENDER)
    #define J_SERIAL_TX_PIN                  PG4
    #define J_SERIAL_RX_PIN      J_SERIAL_TX_PIN

    #define K_SERIAL_TX_PIN                  PE15
    #define K_SERIAL_RX_PIN      K_SERIAL_TX_PIN

    #define E0_SERIAL_TX_PIN                  PE7
    #define E0_SERIAL_RX_PIN      E0_SERIAL_TX_PIN

    #define E1_SERIAL_TX_PIN                PF15
    #define E1_SERIAL_RX_PIN      E1_SERIAL_TX_PIN

    #define E2_SERIAL_TX_PIN                PH14
    #define E2_SERIAL_RX_PIN      E2_SERIAL_TX_PIN
  #endif

  // Reduce baud rate to improve software serial reliability
  #define TMC_BAUD_RATE                    19200
#endif

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PC1   // T1 <-> E0
#define TEMP_1_PIN                          PC2   // T2 <-> E1
#define TEMP_2_PIN                          PC3   // T3 <-> E2

#if ENABLED(M5_EXTENDER)
  #define TEMP_3_PIN                        PA3   // M5 TEMP1
  #define TEMP_4_PIN                        PF9   // M5 TEMP2
  #define TEMP_5_PIN                        PF10  // M5 TEMP3
  #define TEMP_6_PIN                        PF7   // M5 TEMP4
  #define TEMP_7_PIN                        PF5   // M5 TEMP5
#endif

#define TEMP_BED_PIN                        PC0   // T0 <-> Bed

// SPI for MAX Thermocouple
// Uses a separate SPI bus
// If you have a two-way thermocouple, you can customize two TEMP_x_CS_PIN pins (x:0~1)

#define TEMP_0_CS_PIN                       PH9   // GTR K-TEMP
#define TEMP_0_SCK_PIN                      PI1   // SCK
#define TEMP_0_MISO_PIN                     PI2   // MISO
//#define TEMP_0_MOSI_PIN                   ...   // For MAX31865

#define TEMP_1_CS_PIN                       PH2   // M5 K-TEMP
#define TEMP_1_SCK_PIN           TEMP_0_SCK_PIN
#define TEMP_1_MISO_PIN         TEMP_0_MISO_PIN
//#define TEMP_1_MOSI_PIN       TEMP_0_MOSI_PIN

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PB1   // Heater0
#define HEATER_1_PIN                        PA1   // Heater1
#define HEATER_2_PIN                        PB0   // Heater2

#if ENABLED(M5_EXTENDER)
  #define HEATER_3_PIN                      PD15  // M5 HEAT1
  #define HEATER_4_PIN                      PD13  // M5 HEAT2
  #define HEATER_5_PIN                      PD12  // M5 HEAT3
  #define HEATER_6_PIN                      PE13  // M5 HEAT4
  #define HEATER_7_PIN                      PI6   // M5 HEAT5
#endif

#define HEATER_BED_PIN                      PA2   // Hotbed

#define FAN_PIN                             PE5   // Fan0
#define FAN1_PIN                            PE6   // Fan1
#define FAN2_PIN                            PC8   // Fan2

#if ENABLED(M5_EXTENDER)
  #define FAN3_PIN                          PI5   // M5 FAN1
  #define FAN4_PIN                          PE9   // M5 FAN2
  #define FAN5_PIN                          PE11  // M5 FAN3
  #define FAN6_PIN                        PC9   // M5 FAN4
  #define FAN7_PIN                        PE14  // M5 FAN5
#endif

#ifndef SDCARD_CONNECTION
  #define SDCARD_CONNECTION              ONBOARD
#endif

//
// By default the LCD SD (SPI2) is enabled
// Onboard SD is on a completely separate SPI bus, and requires
// overriding pins to access.
//
#if SD_CONNECTION_IS(LCD)

  #define SD_DETECT_PIN              EXP2_07_PIN
  #define SDSS                       EXP2_04_PIN

#elif SD_CONNECTION_IS(ONBOARD)

  #define SDSS                              PA4
  #define SD_SS_PIN                         SDSS
  #define SD_SCK_PIN                        PA5
  #define SD_MISO_PIN                       PA6
  #define SD_MOSI_PIN                       PA7
  #define SD_DETECT_PIN                     PC4

#elif SD_CONNECTION_IS(CUSTOM_CABLE)
  #error "CUSTOM_CABLE is not a supported SDCARD_CONNECTION for this board"
#endif

/**
 *                ------                                     ------
 * (BEEPER) PC11 | 1  2 | PA15 (BTN_ENC)        (MISO) PB14 | 1  2 | PB13 (SCK)
 * (LCD_EN) PC10 | 3  4 | PA8  (LCD_RS)      (BTN_EN1) PD10 | 3  4 | PB12 (SD_SS)
 * (LCD_D4) PG8    5  6 | PG7  (LCD_D5)      (BTN_EN2) PH10   5  6 | PB15 (MOSI)
 * (LCD_D6) PG6  | 7  8 | PG5  (LCD_D7)    (SD_DETECT) PB10 | 7  8 | RESET
 *           GND | 9 10 | 5V                            GND | 9 10 | --
 *                ------                                     ------
 *                 EXP1                                       EXP2
 */
#define EXP1_01_PIN                         PC11
#define EXP1_02_PIN                         PA15
#define EXP1_03_PIN                         PC10
#define EXP1_04_PIN                         PA8
#define EXP1_05_PIN                         PG8
#define EXP1_06_PIN                         PG7
#define EXP1_07_PIN                         PG6
#define EXP1_08_PIN                         PG5

#define EXP2_01_PIN                         PB14
#define EXP2_02_PIN                         PB13
#define EXP2_03_PIN                         PD10
#define EXP2_04_PIN                         PB12
#define EXP2_05_PIN                         PH10
#define EXP2_06_PIN                         PB15
#define EXP2_07_PIN                         PB10

#undef TP
#undef M5_EXTENDER
