## Build instructions

```bash
sudo apt install \
  cmake ninja-build git build-essential pkg-config \
  qt4-default \
  libavcodec-dev libavformat-dev libswscale-dev libavutil-dev \
  libjpeg-dev libboost-all-dev
mkdir build
cd build
cmake -GNinja ..
ninja
```

## Run the Blaster controller
The main program is `kamikaze`; run with all features enabled:
```bash
ninja -C build kamikaze &&
build/kamikaze --logtostderr \
  --spi /dev/spidev0.0 \
  --save_directory=$(pwd)/output
```

`--spi` enables talking to the slave over SPI, which will only work on a
Raspberry Pi connected to the daughterboard with a flashed Atmega328p (see the
arduino/ folder). If using the prototype that communicates over USB Serial,
pass `--arduinoio_tty=/dev/ttyACM0` (or ACM1 etc) instead of `--spi`.

On a dev machine leave off both `--spi` and `--arduinoio_tty` to stub out all
daughterboard communication.

## CLI for controlling the hardware

Test the Arduino communication code with `robot_test`:
```bash
ninja -C build robot_test && build/robot_test
```

## Run the uploader

The uploader code lives in `uploader/`. Currently, it only monitors the shots
directory and prints out the files that it would have uploaded. To run it, first
install the requirements in uploader/requirements.txt:

`$ sudo pip install -r uploader/requirements.txt`

Next, obtain a copy of aws_credentials.secret, which contains the AWS
credentials in the boto3 format.

Then run it with an environment variable, like a normal python script:

`AWS_SHARED_CREDENTIALS_FILE=aws_credentials.secret python uploader/uploader.py`

## Install services (optional)

Install the systemd service to auto-start the controller with
```bash
sudo systemctl enable $(pwd)/kamikaze-controller.service
./configure_autossh.sh  # Might need some configuration.
```
