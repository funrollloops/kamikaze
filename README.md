## Build instructions

```bash
sudo apt install \
  cmake ninja-build git build-essential pkg-config qt4-default python3-pip \
  libavcodec-dev libavformat-dev libswscale-dev libavutil-dev libjpeg-dev
pip install conan
mkdir -p build/Release
(cd build/Release && conan install ../.. --build=missing)
cmake -B build/Release -DCMAKE_BUILD_TYPE=Release -GNinja \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_CXX_CLANG_TIDY=/usr/bin/clang-tidy-14
ninja -C build/Release
```

Using a virtualenv, CC=clang-14, CXX=clang++-14 are recommended. conan should
use the same compiler for dependencies as the main build. direnv can be used to
automate loading the virtualenv and setting the compiler options.

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

## The uploader binary

The uploader binary lives in `uploader/`. It monitors the `shots` directory, or
any directory that is passed to it, and uploads any files that it finds that
[meet the specification](https://github.com/audiodude/kamikaze/blob/master/uploader/file_collector.py#L11).
To run it, first install the requirements in uploader/requirements.txt:

`$ sudo pip install -r uploader/requirements.txt`

Next, obtain a copy of aws_credentials.secret, which contains the AWS
credentials in the boto3 format. This format is documented on
[the boto3 site](https://boto3.amazonaws.com/v1/documentation/api/latest/guide/configuration.html#shared-credentials-file)
Basically, if you have an `aws_access_key_id` and an `aws_secret_access_key`,
you can easily create this plain text file for yourself. Otherwise, you can
obtain credentials from a teammate.

Finally, run the binary  with an environment variable, like a normal python
script:

`AWS_SHARED_CREDENTIALS_FILE=aws_credentials.secret python uploader/uploader.py --watch_dir=/path/to/shots --upload_db=path/to/uploads.sqlite`

The `AWS_SHARED_CREDENTIALS_FILE` environment variable is consumed by the boto3
AWS integration library and used automatically to connect to S3, where the
files are uploaded to. Also note that by default the images are uploaded to the
`blaster-gallery` bucket in the `US West (N. California)` region.

The `--watch_dir` flag is the directory that will be monitored for images
in the specified format.

The `--upload_db` flag is the path to the sqlite database file that will be
used to track uploads. It's perfectly fine to pass the path to a file that does
not exist yet, and it will be created.

See also the repo that handles the display of these images (private):
[raygun](https://github.com/audiodude/raygun)

## Install services (optional)

Install the systemd service to auto-start the controller with
```bash
sudo systemctl enable $(pwd)/kamikaze-controller.service
./configure_autossh.sh  # Might need some configuration.
```
