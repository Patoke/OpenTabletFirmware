# OpenTabletFirmware

Currently a clean room decompilation project that aims at rebuilding the source code for the Gaomon M10K Pro tablet

The goal is to expand this project to support other tablets and MCUs in the Gaomon and Huion lineups, and hopefully eventually support all major tablet manufacturers

I aim for this project to get rid of firmware limitations hardcoded into Gaomon and Huion tablets, this way utilizing all the resources these tablets have to offer in terms of polling rate, pen support, smoothing, sampling, etc...

TODO list:

* How to flash firmware?
* How to use the compiled binaries?
* How to compile?
* How to patch firmware for unsupported tablets? (remove hardware smoothing, boost polling rates, common/shared patches, etc...)
* Requirements
* Credits
* Basically everything... Remember to make it as cool as possible

## STATUS: NOT FUNCTIONAL

Currently this project does **NOT** construct and send HID reports back to the OS and has **NOT** been tested on actual hardware because of this, please do not try to flash this into your tablets, for now, look at this code as a deep dive look into how EMR tablets work and how Gaomon implemented their systems

DISCLAIMER: This project does not use any code from Gaomon or Huion and is made completely from scratch via reverse engineering of the publicly available firmware files provided by Gaomon and Huion for their firmware update tools, the only resources used to develop this project have been the GD32F350 MCU firmware library and manuals