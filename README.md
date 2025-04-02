# GameInn band firmware


### Prerequisites
In Ubuntu 19.10 following packages are required:

For SDK
- gcc-arm-none-eabi
- binutils-arm-none-eabi
For band
- hidrd

```bash
sudo apt update && sudo apt install gcc-arm-none-eabi binutils-arm-none-eabi hidrd openocd
```
### Nordicsemi nRF5 SDK setup

- download SDK
- unzip it and link directory to SDK
- update the Makefile.posix with path to arm-eabi-nne-gcc
- ~~update Makefile.sdk for proper location of config directory~~
- update Makefile.sdk for proper location of sdk_config.h file

```bash
wget https://www.nordicsemi.com/-/media/Software-and-other-downloads/SDKs/nRF5/Binaries/nRF5SDK153059ac345.zip
unzip nRF5SDK153059ac345.zip
ln -s nRF5_SDK_15.3.0_59ac345/ SDK
sed -i -E 's/(GNU_INSTALL_ROOT \?\=)(.*)/\1 \/usr\/bin\//' SDK/components/toolchain/gcc/Makefile.posix
#sed -i 's/..\/config/\$\{PROJ_DIR\}\/config/' Makefile.sdk
sed -i 's/..\/config/\./' Makefile.sdk
```

### Build instructions

```
make
```

### Flashing prerequisites

Install Nordic Semi Flash tools
````bash
mkdir tools
cd tools
wget https://www.nordicsemi.com/-/media/Software-and-other-downloads/Desktop-software/nRF-command-line-tools/sw/Versions-10-x-x/nRFCommandLineTools1040Linuxamd64tar.gz
tar -xzf *.gz
sudo dpkg -i JLink_Linux_V650b_x86_64.deb nRF-Command-Line-Tools_10_4_0_Linux-amd64.deb
````

### Programming

````bash
make flash
````

## License

Copyright (c) 2024 OmniChip Sp. z o.o.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.