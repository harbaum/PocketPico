# PocketPico - ILI9341 edition

This has been forked from [Slintaks PocketPico](https://github.com/slintak/PocketPico). It
adds support for the ILI9341 family of displays with 320x240 pixels resolution
and available in various sizes from 2.0 to 3.5 inch. Together with a simple 1.5x scaler,
the emulator can make efficient use of the screen estate.

![PocketPico on ILI9341.](./docs/pocketpico_ili9341.jpeg)

This is meant to be mounted into a
[Lego Gameboy #72046](https://www.lego.com/de-de/product/game-boy-72046) where
the 2.8 inch display nicely fills the entire display area.

Since this is to be used as a single game setup there's no SD card and the ILI9341
display routines have thus not been adopted for the SD card menu.

## Lego installation

Some extensive dremeling is required to make the display fit into the Lego Gameboy.

![Dremeled display](./docs/stripped_display.jpeg)

The Pico and e.g. the I²S amp can then be taped to the rear side of the display.
Using some [two stripes of thick double-sided tape](https://www.tesa.com/de-de/buero-und-zuhause/tesa-powerbond-indoor.html)
allows to route the wires underneath the Pico.

![Pico mounted](./docs/pico_mounted.jpeg)

## Wiring

All components need to be wired as depicted.

![Wiring](./docs/wiring.png)

## Flashing games

Without SD card we need another means to install the game cartridge
rom. With typically 2 or more Megabytes of flash memory, the RP2040
has plenty of space for cartridge images. Instead of loading from SD
card into flash the
[Pico Debug Probe](https://www.raspberrypi.com/documentation/microcontrollers/debug-probe.html)
may be used to upload game cartridges from a PC into the RP2040's
flash memory. E.g. ```tetris.gb``` can be installed like so:

```
openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 5000" -c "program ../roms/tetris.gb 0x10100000 verify exit"
```

An alternative is to use [Microsofts uf2conv tool](https://github.com/microsoft/uf2/blob/master/utils/uf2conv.md) to convert the gameboy cartridge images into regular ```uf2``` files. These
can then be uploaded the usual way via file transfer. This upload will _not_ overwrite the
main firmware but will only replace the cartridge data.

```
$ ./uf2conv.py tetris.gb --family RP2040 --base 0x10100000 --convert --output tetris.uf2 
Converted to uf2, output size: 65536, start address: 0x10100000
Wrote 65536 bytes to tetris.uf2
```

# PocketPico (The GameBoy Emulator for RP2040)

This is a fork of the [Pico-GB GameBoy emulator for RP2040 from YouMakeTech](https://github.com/YouMakeTech/Pico-GB) which itself is a fork of the [RP2040-GB Game Boy (DMG) emulator from deltabeard](https://github.com/deltabeard/RP2040-GB).

PocketPico is a Game Boy (DMG) emulator Peanut-GB on the custom PCB with Raspberry Pi RP2040 microcontroller, using an ILI9225 screen. Runs at 55-60 fps with audio and more than 70 fps without audio emulation. With frame skip and interlacing, can run at up to 120 fps.

The [YouMakeTech fork](https://github.com/YouMakeTech/Pico-GB) includes these changes:
* push buttons support
* overclocking to 266MHz for more accurate framerate (~60 FPS)
* I2S sound support (44.1kHz 16 bits stereo audio)
* SD card support (store roms and save games) + game selection menu
* automatic color palette selection for some games (emulation of Game Boy Color Bootstrap ROM) + manual color palette selection

I made these changes:
* include `pico-sdk`, `peanut-gb`, `minigb_apu` and `hedley` as Git submodules
* use latest versions (from master branch) of the `peanut-gb` and `minigb_apu`
* use 32.768 kHz sample rate for audio (default value for latest `minigb_apu`)
* drawing to LCD is offloaded to DMA and PIO, second MCU core is no longer needed
* moved audio processing to the second MCU core

# Hardware

**More informations soon!**

![PocketPico hardware.](./docs/PocketPico-revision-A.png)
![PocketPico hardware.](./docs/PocketPico-prototype.jpg)

# Flashing the firmware

* Download `PocketPico.uf2` from the [releases page](https://github.com/slintak/PocketPico/releases)
* Connect the PocketPico to your computer by USB-C cable. Turn on the device.
* Push and hold the `1` button on the PocketPico located at the back (this is `BOOTSEL` button).
* Press and release the `2` button on the PocketPico located next to the `1` button (this is `RESET` button). You can release all button, now.
* The drive RPI-RP2 should appear on your computer as a new removable disk.
* Drag and drop the UF2 file on to the RPI-RP2 drive. The PocketPico will reboot and will now run the emulator.

# Preparing the SD card

The micro SD card is used to store game roms and save game progress. For this project, you will need a FAT 32 formatted micro SD card with roms you legally own. Roms must have the .gb extension.

* Insert your micro SD card in a computer and format it as FAT 32
* Copy your .gb files to the SD card root folder (subfolders are not supported at this time)
* Insert the SD card into the micro SD card slot using a Micro SD adapter

# Building from source

Make sure your system has git, cmake and pyOCD (requires working Python3 installation):

```
$ sudo apt-get install git cmake
$ python3 -m pip install -U pyocd
```

Clone this repository and all its submodules:

```
$ git clone https://github.com/slintak/PocketPico.git
$ git submodules update --init --recursive
```

Build source code

```
$ make configure
$ make build
```

The firmware is located in `./build/PocketPico.bin` and `./build/PocketPico.uf2`. Flashing firmware during the development requires [Raspberry Pi Debug Probe](https://www.raspberrypi.com/products/debug-probe/).

```
$ pyocd flash -t rp2040 ./build/PocketPico.bin
```

# Known issues and limitations

* No copyrighted games are included with PocketPico / Pico-GB / RP2040-GB. For this project, you will need a FAT 32 formatted Micro SD card with roms you legally own. Roms must have the .gb extension.
* The PocketPico / Pico-GB emulator is able to run at full speed on the Pico, at the expense of emulation accuracy. Some games may not work as expected or may not work at all. RP2040-GB is still experimental and not all features are guaranteed to work.
* PocketPico is only compatible with original Game Boy DMG games (not compatible with Game Boy Color or Game Boy Advance games).
* The emulator overclocks the Pico in order to get the emulator working fast enough. Overclocking can reduce the Pico’s lifespan.
