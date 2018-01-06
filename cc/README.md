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
