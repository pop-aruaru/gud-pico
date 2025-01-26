Import from [notrogud-pico](https://github.com/notro/gud-pico.git)  

Raspberry Pi Pico GUD USB Display
---------------------------------

GUD implementation LCD driven by ST7796,ILI9488,ILI9341.

Support Raspbarry Pi pico and pico2 , xiao, xiao-rp2350.

The ```PICO_SDK_PATH``` env var should point to the Pico SDK.

Build
```
$ git clone https://github.com/ShojiMiyanishi/gud-pico.git
$ cd gud-pico
$ mkdir build && cd build
$ cmake  -DPICO_PAORD=seeed_xiao_rp2040 ../
or
$ cmake  -DPICO_PAORD=seeed_xiao_rp2350 ../
$ cmake  -DPICO_PAORD=pico2 ../
$ cmake  -DPICO_PAORD=pico ../
$ make -j8

```

The default VID:PID won't be supported in the host driver before Linux v5.15 is out (it's present in the rPi 5.10 backport). Can be changed in `libraries/gud_pico/tusb_config.h`.
