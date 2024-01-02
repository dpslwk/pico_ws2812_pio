# WS2812 Library for Pico using PIO's (? and FreeRTOS)
Based on  
https://mcuoneclipse.com/2023/04/02/rp2040-with-pio-and-dma-to-address-ws2812b-leds/  
https://github.com/ErichStyger/mcuoneclipse/tree/master/Examples/RaspberryPiPico/pico_LedCube  
by Erich Styger

```bash
rm -rvf build
cmake -B build -S . -DEXAMPLE_BUILD=ON -DFREERTOS_DIR=/Users/matt/Dropbox/Work/LWK/Pico-RP2040/github/FreeRTOS-Kernel
cd build
make
```
