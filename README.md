# c't-Bot ATmega framework

This is a very basic framework for the [c't-Bot][ctBot] with an ATmega1284P controller. It is licensed under the terms of the [GPLv3 license](LICENSE.md).

## Summary

This code was created for testing a new and simplified design of the c't-Bot software framework. The basic idea is to have a modular structured framework implemented in C++ built on modern language features. Although it doesn't use the [Arduino software framework][ArduinoCore] for some reasons (see below), there are many similarities to an Arduino program, such as the usage of external libraries, etc.

### Notice

Consider this as experimental code. **If it breaks, you get to keep both pieces.**

## Differences against the [original c't-Bot software framework][ctBotSW]

* programming language: using C++ instead of C to have
  * object oriented programming,
  * a strict modular structure,
  * banishment of macros, `#define`, `#ifdef` and other unsafe stuff of the last century.

* [PlatformIO] build system, because of:
  * independence of IDE and environment, just use what you like for editing and building the code.
  * works on common Linux distributions, macOS, Windows.
  * simple and minimalistic tool setup, you just have to install PlatformIO Core (see below).
  * it is an [open source][PIOGithub] project.
  * ...

* consistently reduced functionality
  * no behaviors available
  * support for basic components only
    * UART console command interface (*115200 Baud* with *8N1* configuration)
      * connect with USB-2-Bot adapter and terminal program like minicom or WiPort and telnet
      * type `help` + `[ENTER]` the get a list of supported commands
    * LEDs: `Leds::on()`, `Leds::off()`, `Leds::set()` implemented, see [leds.h](src/leds.h)
    * LCD: `Display::print()` and `Display::printf()` implemented, see [display.h](src/display.h)
    * standard sensors (excluding mouse sensor), raw sensor values are reported on LC display or via console
    * remote control (`HQ RC Univers 29` only in standard setup, see `RemoteControl::RemoteControl()` in [remote_control.cpp](src/remote_control.cpp) for mapped keys)
    * motors with speed control
      * use `set speed [LEFT] [RIGHT]` to set speed of left / right wheel as percentage values (negative values to drive backwards)
      * alternatively use remote control to increase / decrease speed of wheels, see [Usage](#Usage)
      * *ToDo*: PID calibration has to be done / fine tuned
    * 2 servos
  * ATmega1284P support only
  * simple test cases implemented (see [tests.h](src/tests.h) for details)
    * LedTest: chaser lights
    * LcdTest: prints some nice stuff on the LC display
    * EnaTest: activates ENAs one by one in an endless loop
    * SensorLcdTest: displays all sensor information like the 'sensor display screen' of the original c't-Bot  software (see `SensorLcdTest::run()` in [tests.cpp](src/tests.cpp) for details)

## Setup

1. install PlatformIO core as described [here][PIOInstall]
    * can be skipped, if using VS Code IDE with [PlatformIO extension][PlatformIOVSC]
1. clone this git repository: `git clone https://github.com/tsandmann/ctbot-atmega`
1. change to cloned repo: `cd ctbot-atmega`
1. initialize build system for...
    * commandline build: `platformio init`
    * [VS Code][VSCode] project: `platformio init --ide vscode`
    * [Eclipse CDT][EclipseCDT] project: `platformio init --ide eclipse`
    * any other environment supported by [PlatformIO][PlatformIOIDE]

## Usage

1. build project
    * commandline: `plaformio run`
    * VS Code: use “Build” button on the PlatformIO toolbar or shortcut (`ctrl/cmd+alt+b`)
    * Eclipse CDT: `Project` -> `Build Project` or shortcut (`ctrl/cmd+b`)
1. upload firmware image
    * flash `firmware.hex` (located in folder `.pioenvs/1284p16m/`) using avrdude as described [here][WikiFlash].
      * **todo**: update for use of platformio upload target
1. use a terminal program (e.g. minicom) or telnet to connect
    * if you use minicom:
      * goto `Serial port setup` settings and set `Serial Device` to your serial device, `Bps/Par/Bits` to `115200 8N1` and `Hardware Flow Control` to `No` as well as `Software Flow Control` to `No`
      * goto `Screen and keyboard` settings and set `Add carriage return` to `Yes`
    * if you use a WiPort: `telnet [IP_ADDRESS] 10002`
      * to switch to line mode echo press `CTRL + ]` and enter `mode line`
      * to disable local character press `CTRL + ]` and enter `mode -litecho`

1. have fun with the command line interface
    * type `help` to get a list and description of available commands
1. cruise around with the c't-Bot using
    * your remote control: **arrow keys** for *forward*, *backward*, *left*, *right* and **power button** for *stop*
    * the command line interface: `set speed 30 30` for 30% of max speed on both wheels, `set speed 0 0` (or just `set speed`) to stop
1. press **play** on remote control get a little easter egg on the command line interface :) or **I/II** to shutdown the bot :(

## Why it doesn't use the Arduino software framework

* hard coded usage of timer0
* uart buffer space not adjustable
* ...

## Notices

* if you use the ATmega with a 20 MHz clock, set `board_f_cpu` in [platformio.ini](platformio.ini) to `20000000UL`
* for a c't-Bot with the *hardware SPI patch*, set `CtBotConfig::ENC_L_USE_PC5` in [ctbot_config.h](src/ctbot_config.h) to `true`
* to build the documentation with Doxygen: `doxygen Doxyfile`
  * [PlantUML] has to be installed, to build the UML diagrams
  * documentation is located here: [doc/html/index.html](doc/html/index.html)
  * documentation is highly incomplete
* conventions:
  * indentation is done by 4 (four) spaces for each level, **never** ever use tabs (`\t` | `HT`)
  * follow the [C++ Core Guidelines]
  * documentation is done with Doxygen, use [Doxygen style 1]
  * more to come soon
* ...

[ctBot]: https://www.heise.de/ct/artikel/c-t-Bot-und-c-t-Sim-284119.html
[ArduinoCore]: https://github.com/arduino/ArduinoCore-avr
[ctBotSW]: https://github.com/tsandmann/ct-bot
[PlatformIO]: https://platformio.org
[PIOGithub]: https://github.com/platformio/platformio-core
[PIOInstall]: http://docs.platformio.org/en/latest/installation.html
[PlatformIOVSC]: http://docs.platformio.org/en/latest/faq.html#faq-install-shell-commands
[VSCode]: https://github.com/Microsoft/vscode
[EclipseCDT]: https://eclipse.org
[PlatformIOIDE]: http://docs.platformio.org/en/latest/ide.html#ide
[WikiFlash]: https://www.heise.de/ct/projekte/machmit/ctbot/wiki/Flash
[PlantUML]: http://plantuml.com
[C++ Core Guidelines]: https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md
[Doxygen style 1]: https://www.stack.nl/~dimitri/doxygen/manual/docblocks.html#cppblock
