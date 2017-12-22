# stm32c8t6-make
Use make to cross-platform compile arm code for stm32c8t6.
```shell
git submodule add http://github.com/stm32f103c8t6/stm32f103c8t6-make
```
## System requirement
* System: MacOS Linux
* Compiler : gcc-arm
* Driver : st-link 

### Compiler gcc-arm
* Here to get
  * [ARM official (compressed about 100M) ](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads)
  * [github.com/gcc-arm (1.2G)](http://github.com/gcc-arm)
* Keep subfolders
* Recommended installation location
  * ~/gcc-arm

### Driver stlink
* Here to get
  * [github.com/texane/stlink](https://github.com/texane/stlink)
  * [github.com/gcc-arm/gcc-arm-stlink](https://github.com/gcc-arm/gcc-arm-stlink)

## How-To
```
cd workspace
git submodule add http://github.com/stm32f103c8t6/stm32f103c8t6-make

cd stm32f103c8t6-make
make
```

## Targets for make
* make
  * need gcc-arm installed
  * create a folder "build" under folder stm32f103c8t6-make
  * compile all c file and include headers under ../src
  * compile all c file and include headers under folders starts with stm32f103c8t6 (../stm32f103c8t6-*)
  * compile all c file and include headers under ../src/stm32f103c8t6-*
  * create files .bin .hex .elf in folder stm32f103c8t6-make/build
* make write
  * all the make above
  * means if you want, you can only 'make write'
  * need driver stlink installed 
  * write the binary file to device
* make flash
  * alias of make write
* make clean
  * remove folder: build/ .dep/ 

## Suggest
use <b>git submodule add</b> manage stm32f103c8t6 modules
