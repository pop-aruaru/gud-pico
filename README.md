Import from [notrogud-pico](https://github.com/notro/gud-pico.git)  

Raspberry Pi Pico GUD USB Display
---------------------------------

GUD implementation for the Raspberry Pi Pico with a LCD driven by ST7796,ILI9488,ILI9341.

Support Raspbarry Pi pico and pico2 , xiao, xiao-rp2350.

The ```PICO_SDK_PATH``` env var should point to the Pico SDK.

Build
```
$ git clone 
$ cd gud-pico
$ mkdir build && cd build
$ cmake ..
$ make

```

The default VID:PID won't be supported in the host driver before Linux v5.15 is out (it's present in the rPi 5.10 backport). Can be changed in `libraries/gud_pico/tusb_config.h`.
