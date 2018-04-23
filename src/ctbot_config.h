/*
 * This file is part of the c't-Bot ATmega framework.
 * Copyright (c) 2018 Timo Sandmann
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file    ctbot_config.h
 * @brief   Configuration settings
 * @author  Timo Sandmann
 * @date    15.04.2018
 */

#ifndef SRC_CTBOT_CONFIG_H_
#define SRC_CTBOT_CONFIG_H_

#include <cstdint>
#include <type_traits>


namespace ctbot {

/**
 * @brief Helper class to represent ATmega port and pin registers
 * @tparam OFFSET: Address offset for the port to be used
 *
 * @startuml{ATmegaPinConfig.png}
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 *  class ctbot::ATmegaPinConfig <template <uint16_t OFFSET>> {
 *  }
 * @enduml
 */
template <uint16_t OFFSET>
class ATmegaPinConfig {
public:
    static constexpr intptr_t PINR = OFFSET + 0; /**< Address of PINR register */
    static constexpr intptr_t DDR = OFFSET + 1; /**< Address of DDR register */
    static constexpr intptr_t PORT = OFFSET + 2; /**< Address of PORT register */
};

/**
 * @brief Register addresses for ATmega1284P
 *
 * @startuml{ATmega1284P.png}
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 *  class ctbot::ATmega1284P {
 *  }
 * @enduml
 */
class ATmega1284P {
public:
    using PORT_A = ATmegaPinConfig<0x00 + 0x20>; /**< Addresses of Port A */
    using PORT_B = ATmegaPinConfig<0x03 + 0x20>; /**< Addresses of Port B */
    using PORT_C = ATmegaPinConfig<0x06 + 0x20>; /**< Addresses of Port C */
    using PORT_D = ATmegaPinConfig<0x09 + 0x20>; /**< Addresses of Port D */

    static constexpr intptr_t OCR_1A { 0x88 }; /**< Address of OCR1A register (timer 1) */
    static constexpr intptr_t OCR_1B { 0x8a }; /**< Address of OCR1B register (timer 1) */
    static constexpr intptr_t OCR_3A { 0x98 }; /**< Address of OCR3A register (timer 3) */
    static constexpr intptr_t OCR_3B { 0x9a }; /**< Address of OCR3B register (timer 3) */

    static constexpr intptr_t TCN_T0 { 0x26 + 0x20 }; /**< Address of TCNT0 register (timer 0) */
    static constexpr uint8_t WGM_01 { 1 };
    static constexpr uint8_t CS_00 { 0 };
    static constexpr uint8_t CS_01 { 1 };
    static constexpr uint8_t OCIE_0A { 1 };

    static constexpr intptr_t PCIFR_ { 0x3b }; /**< Address of PCIFR register (pin change interrupt flag) */
    static constexpr intptr_t PCICR_ { 0x68 }; /**< Address of PCICR register (pin change interrupt control) */
    static constexpr intptr_t PCMSK_0 { 0x6b }; /**< Address of PCMSK0 register (pin change interrupt mask) */
    static constexpr intptr_t PCMSK_1 { 0x6c }; /**< Address of PCMSK1 register (pin change interrupt mask) */
    static constexpr intptr_t PCMSK_2 { 0x6d }; /**< Address of PCMSK2 register (pin change interrupt mask) */
    static constexpr intptr_t PCMSK_3 { 0x73 }; /**< Address of PCMSK3 register (pin change interrupt mask) */
};

/**
 * @brief Configuration settings
 *
 * @startuml{CtBotConfig.png}
 *  set namespaceSeparator ::
 *  skinparam classAttributeIconSize 0
 *  class ctbot::CtBotConfig {
 *      +{static} LED_TEST_AVAILABLE : constexpr bool
 *      +{static} LCD_TEST_AVAILABLE : constexpr bool
 *      +{static} ENA_TEST_AVAILABLE : constexpr bool
 *      +{static} SENS_LCD_TEST_AVAILABLE : constexpr bool
 *      +{static} UART0_BAUDRATE : constexpr uint32_t = 115200UL
 *      +{static} UART0_RX_PULLUP_ON : constexpr bool = true
 *      +{static} ENC_L_USE_PC5 : constexpr bool = false
 *      +{static} RC5_ADDR : constexpr uint8_t = 6
 *  }
 *  ctbot::ATmega1284P <|-- ctbot::CtBotConfig
 * @enduml
 */
class CtBotConfig : public ATmega1284P {
public:
    static constexpr bool LED_TEST_AVAILABLE { true }; /**< Statically activate or deactivate LED test */
    static constexpr bool LCD_TEST_AVAILABLE { false }; /**< Statically activate or deactivate LCD test */
    static constexpr bool ENA_TEST_AVAILABLE { false }; /**< Statically activate or deactivate ENA test */
    static constexpr bool SENS_LCD_TEST_AVAILABLE { true }; /**< Statically activate or deactivate sensor display test */

    static constexpr uint32_t UART0_BAUDRATE { 115200UL }; /**< Baud rate used for Uart 0 */
    static constexpr bool UART0_RX_PULLUP_ON { true }; /**< Enable (true) or disable (false) pullup for Uart0 RX pin */

    static constexpr bool ENC_L_USE_PC5 { false }; /**< Configuration setting for use of PC5 instead of PB4 for ENC_L */


    /* ENC_L: PB4 or PC5 */
    using ENC_L_REG = std::conditional<ENC_L_USE_PC5, PORT_C, PORT_B>::type; /**< Port and pin register addresses for ENC_L signal */
    static constexpr uint8_t ENC_L_PIN { ENC_L_USE_PC5 ? 5 : 4 }; /**< Pin number of ENC_L signal */

    /* ENC_R: PD3 */
    using ENC_R_REG = PORT_D; /**< Port and pin register addresses for ENC_R signal */
    static constexpr uint8_t ENC_R_PIN { 3 }; /**< Pin number of ENC_R signal */

    /* FERNBED: PB1 */
    using RC5_REG = PORT_B; /**< Port and pin register addresses for RC5 signal */
    static constexpr auto RC5_PCMSK = PCMSK_1; /**< Pin change interrupt mask register for RC5 signal */
    static constexpr uint8_t RC5_PIN { 1 }; /**< Pin number of RC5 signal */
    static constexpr uint8_t RC5_PCI { 1 }; /**< Pin change interrupt number of RC5 signal */
    static constexpr uint8_t RC5_ADDR { 6 }; /**< RC5 address of used remote control */

    /* KLAPPE: PD6 */
    using SHUTTER_REG = PORT_D; /**< Port and pin register addresses for KLAPPE signal */
    static constexpr uint8_t SHUTTER_PIN { 6 }; /**< Pin number of KLAPPE signal */

    /* SCHRANKE: PB0 */
    using TRANSPORT_REG = PORT_B; /**< Port and pin register addresses for SCHRANKE signal */
    static constexpr uint8_t TRANSPORT_PIN { 0 }; /**< Pin number of SCHRANKE signal */

    /* FEHLER: PB2 */
    using ERROR_REG = PORT_B; /**< Port and pin register addresses for FEHLER signal */
    static constexpr uint8_t ERROR_PIN { 2 }; /**< Pin number of FEHLER signal */

    /* analog signals */
    static constexpr uint8_t DISTANCE_L_PIN { 0 }; /**< Pin number of DISTANCE_L signal */
    static constexpr uint8_t DISTANCE_R_PIN { 1 }; /**< Pin number of DISTANCE_R signal */
    static constexpr uint8_t LINE_L_PIN { 2 }; /**< Pin number of LINE_L signal */
    static constexpr uint8_t LINE_R_PIN { 3 }; /**< Pin number of LINE_R signal */
    static constexpr uint8_t LDR_L_PIN { 4 }; /**< Pin number of LDR_L signal */
    static constexpr uint8_t LDR_R_PIN { 5 }; /**< Pin number of LDR_R signal */
    static constexpr uint8_t BORDER_L_PIN { 6 }; /**< Pin number of BORDER_L signal */
    static constexpr uint8_t BORDER_R_PIN { 7 }; /**< Pin number of BORDER_R signal */

    /* display */
    using DISPLAY_REG = PORT_C; /**< Port and pin register addresses for display signals */
    static constexpr uint8_t DISPLAY_RS_PIN { 0 }; /**< Pin number of display RS signal */
    static constexpr uint8_t DISPLAY_SCK_PIN { 1 }; /**< Pin number of display shift register SCK signal */
    static constexpr uint8_t DISPLAY_RW_PIN { 1 }; /**< Pin number of display RW signal */
    static constexpr uint8_t DISPLAY_RCK_PIN { 2 }; /**< Pin number of display shift register RCK signal */
    static constexpr uint8_t DISPLAY_ENA_PIN { 2 }; /**< Pin number of display ENA signal */
    static constexpr uint8_t DISPLAY_READY_PIN { 5 }; /**< Pin number of display READY signal */

    /* ena */
    static constexpr uint8_t ENA_SCK_PIN { 3 }; /**< Pin number of ena shift register SCK signal */
    static constexpr uint8_t ENA_RCK_PIN { 1 }; /**< Pin number of ena shift register RCK signal */

    /* shift */
    using SHIFT_REG = PORT_C; /**< Port and pin register addresses for shift register signals */
    static constexpr uint8_t SHIFT_SDATA_PIN { 0 }; /**< Pin number of shift register SDATA signal */

    /* leds */
    static constexpr uint8_t LED_SCK_PIN { 4 }; /**< Pin number of led shift register SCK signal */
    static constexpr uint8_t LED_RCK_PIN { 1 }; /**< Pin number of led shift register RCK signal */

    /* motors */
    using MOT_L_PWM_REG = PORT_D; /**< Port and pin register addresses for left motor pwm signal */
    using MOT_L_DIR_REG = PORT_C; /**< Port and pin register addresses for left motor direction signal */
    static constexpr uint8_t MOT_L_PWM_PIN { 5 }; /**< Pin number of left motor pwm signal */
    static constexpr uint8_t MOT_L_DIR_PIN { 6 }; /**< Pin number of left motor direction signal */
    using MOT_R_PWM_REG = PORT_D; /**< Port and pin register addresses for right motor pwm signal */
    using MOT_R_DIR_REG = PORT_C; /**< Port and pin register addresses for right motor direction signal */
    static constexpr uint8_t MOT_R_PWM_PIN { 4 }; /**< Pin number of right motor pwm signal */
    static constexpr uint8_t MOT_R_DIR_PIN { 7 }; /**< Pin number of right motor direction signal */
    static constexpr intptr_t MOT_L_OCR { OCR_1A }; /**< Address of left motor pwm OCR register */
    static constexpr intptr_t MOT_R_OCR { OCR_1B }; /**< Address of right motor pwm OCR register */
    static constexpr uint16_t MOT_PWM_MAX { 16000 }; /**< Maximum PWM value for motors */

    /* servos */
    using SERVO_1_REG = PORT_B; /**< Port and pin register addresses for servo 1 pwm signal */
    static constexpr uint8_t SERVO_1_PIN { 3 }; /**< Pin number of servo 1 pwm signal */
    static constexpr intptr_t SERVO_1_OCR { OCR_3A }; /**< Address of servo 1 OCR register */
    using SERVO_2_REG = PORT_D; /**< Port and pin register addresses for servo 2 pwm signal */
    static constexpr uint8_t SERVO_2_PIN { 7 }; /**< Pin number of servo 2 pwm signal */
    static constexpr intptr_t SERVO_2_OCR { OCR_3B }; /**< Address of servo 2 OCR register */

    /* uart0 */
    using UART0_RX_REG = PORT_D; /**< Port and pin register addresses for uart0 signals */
    static constexpr uint8_t UART0_RX_PIN { 0 }; /**< Pin number of uart0 RX signal */
};

} /* namespace ctbot */

#endif /* SRC_CTBOT_CONFIG_H_ */
