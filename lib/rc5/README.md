RC5 remote control decoder library
==========================================

This is an library for decoding infrared remote control commands encoded
with the Philips RC5 protocol. It is based on the article
"An Efficient Algorithm for Decoding RC5 Remote Control Signals"
by Guy Carpenter, Oct 2001.

For more information see http://clearwater.com.au/code/rc5.

This library supports the extended RC5 protocol which uses the second
stop bit S2 as an extension to the command value.

See also http://www.sbprojects.com/knowledge/ir/rc5.php

This library if a fork of the original [RC5 library by guyc](https://github.com/guyc/RC5) - it is adapted
for use with the c't-Bot ATmega framework and ported to C++14.
