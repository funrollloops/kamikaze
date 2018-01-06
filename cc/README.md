```bash
sudo apt install \
  cmake ninja-build git build-essential pkg-config \
  qt4-default \
  libavcodec-dev libavformat-dev libswscale-dev libavutil-dev \
  libjpeg-dev
mkdir build
cd build
cmake -GNinja ..
ninja
```

Test the Arduino communication code with `robot_test`:
```bash
ninja -C build/src-build robot_test && build/src-build/robot_test
```

The main program is `kamikaze`; run with all features enabled:
```bash
ninja -C build kamikaze &&
build/kamikaze --logtostderr \
  --spi /dev/spidev0.0 \
  --save_directory=$(pwd)/output --save_video
```
--spi enables talking to the slave over SPI, which will only work on a
Raspberry Pi connected to the daughterboard with a flashed Atmega328p (see the
arduino/ folder).  On a dev machine leave this flag off to stub out all
daughterboard communication.
