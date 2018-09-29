# Odroid-GO as a Sphero BB-8 remote control

Simple program to transform you Odroid-GO into a remote control for
Sphero BB-8.

## Build

1. Download the *Espressif IoT Development Framework* fork for odroid go:
`git clone https://github.com/OtherCrashOverride/esp-idf.git`
2. Set the environment variable to this directory:
`export IDF_PATH=<...>/esp-idf`
3. Setup the toolchain as described in the [ESP-IDF Programming
Guide](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/linux-setup.html).
You should have updated your `PATH` environment variable to add the toolchain
programs. 
4. Clone this repository and enter the created directory.
5. Pull the submodules: `git submodule init && git submodule update`
6. Run `make menuconfig` to change your BB-8 bluetooth MAC address
7. Compile: `make`
8. Flash your odroid-go: `make flash`

## Licence

Part of the code is by Antoine Sirinelli under the WTFPL.

I have used code snippets from the ESP-IDF examples and re-used some odroid-go
specific codes from
[OtherCrashOverride](https://github.com/OtherCrashOverride).
