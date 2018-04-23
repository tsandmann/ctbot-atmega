# C++ Utils AVR

C++ utility library for platforms where no full C++ standard library is available, written for C++14. Currently support for AVR devices only.

It is licensed under the terms of the GPLv3 license, see [LICENSE.md](LICENSE.md).

## Features

* [std::function](http://en.cppreference.com/w/cpp/utility/functional/function) like functor implementation (see [functor.h](src/functor.h)). Be aware, that this is **not** a full replacement for std::function. Addionally it allows for a binding of parameters, see doxygen documentation for details.
* µController related helper functions [utils_avr.h](src/utils_avr.h), especially for AVR (namespace `avr`).
* ATmega1284P serial port driver [serial_port.h](src/serial_port.h) based on [Arduino] implementation.
* Wrapper for std::cout, std::cerr and std::cin to ATmega serial port driver (see [serial_iostream_avr.cpp](src/serial_iostream_avr.cpp))

### Notice

Consider this as experimental code. **If it breaks, you get to keep both pieces.**

[Arduino]: https://github.com/arduino/ArduinoCore-avr
