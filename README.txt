In order to compile this firmware and flash the target board you need to perform the steps below:

1. Create a working directory (if it does not already exist) to setup the toolchain.

	mkdir ~/development
	cd ~/development

2. Install some dependencies using the commands below.

	sudo apt-get update
	sudo apt-get install flex bison libgmp3-dev libmpfr-dev \
	    libncurses5-dev libmpc-dev autoconf texinfo build-essential \
	    libftdi-dev python-yaml zlib1g-dev libtool
	sudo apt-get install libusb-1.0-0 libusb-1.0-0-dev
       
3. On Ubuntu 16.04, you can do (and skip to step 5):
	sudo apt-get install gcc-arm-none-eabi

3. Go to the GNU Tools for ARM Embedded Processors page and download the most recent tarball for Linux. 
Then extract it (including the top-level “gcc-arm-none-eabi-…” folder) into ~/development.

	tar -xvf gcc-arm-none-eabi-4_9-2015q1-20150306-linux.tar.bz2 -C ~/development

4. Add the extracted directory’s bin folder to your user’s path.
	
	export PATH=$PATH:$HOME/development/gcc-arm-none-eabi-4_9-2015q1/bin

5. Download the “TivaWare for Tiva C Series” package from TI’s Tiva C Series Software section. You will 
be asked to create a login after clicking the download link in order to get the file. The current version 
is tested on SW-TM4C-2.1.0.12573.exe.

6. Extract the exe file to a new tivaware folder in development using the commands below.
	
	cd ~/development
	mkdir tivaware
	cd tivaware/
	// Go download exe from link above, and place it in the tivaware folder
	mv ~/Downloads/SW-TM4C-2.1.0.12573.exe . // don't miss the dot
	unzip SW-TM4C-2.1.0.12573.exe

7. Compile with make.
	
	make

8. lm4flash is the utility we use to flash our target board. Grab it from GitHub and compile the source.
	
	cd ~/development
	git clone git://github.com/utzig/lm4tools.git
	cd lm4tools/lm4flash/
	make

9. Run make to build the firmware. The firmware path may be different so make sure you enter the correct 
directory, in our case we also place it in the development directory.
	
	cd ~/development/pmtctrl-fw
	// Name is usually the name of the city where the system is installed
	make DESIGN=name

10. If the board is connected using the ICDI (In Circuit Debug Interface) jump to step 11. Otherwise, you
need to flash your board using a tiva board connected via JTAG to your board. You can follow the instructions
for the "Debug Out" mode here http://austinblackstoneengineering.com/jtag-and-the-stellaris-launchpad/. Briefly
you need to perform the steps below:
    - Remove Power Jumper from the tiva board
    - Connect wires from the JTAG pins (TCK, TMS, TDO, TDI, EXT DBG, TXD, RXD, RESET) below the top of the tiva
      board to the target board.
    - Wire together a common ground wire between the tiva board and the target board.
    - Connect your PC to the tiva board via USB (using the ICDI) and perform step 11.

11. Run this to flash over USB. Remember to run with sudo.

	cd ~/development/lm4tools/lm4flash/
	sudo ./lm4flash ~/development/pmtctrl-fw/gcc/main.bin
