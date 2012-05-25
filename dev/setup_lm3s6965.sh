 #
 #  This script is used to set up OpenOCD with FTDI drivers to get the TI LM3S6965 playing nice with GDB.
 #  This was written for Arch Linux (or anything that uses pacman)
 #
 # ----------------------------------------------------------------------------
 # "THE BEER-WARE LICENSE" (Revision 42):
 # <phil.behnke@gmail.com> wrote this file. As long as you retain this notice you
 # can do whatever you want with this stuff. If we meet some day, and you think
 # this stuff is worth it, you can buy me a beer in return - Phil Behnke
 # ----------------------------------------------------------------------------
 #

yaourt -S arm-none-eabi

INTERFACE_CFG_URL=http://andrewsterian.com/Downloads/ARM/luminary-ek-lm3s6965.cfg
BOARD_CFG_URLhttp://andrewsterian.com/Downloads/ARM/ek-lm3s6965.cfg


mkdir ~/OpenOCD
cd ~/OpenOCD

#Important: the libFTDI drivers from Intra2net are used instead of the ftd2xx drivers from FTDI.
#The development package must be used
sudo pacman -S --noconfirm libftdi

#get OpenOCD
sudo pacman -S --noconfirm openocd

wget $INTERFACE_CFG_URL 
wget $BOARD_CFG_URL

sudo cp ek-lm3s6965.cfg /usr/local/share/openocd/scripts/board
sudo cp luminary-ek-lm3s6965.cfg /usr/local/share/openocd/scripts/interface

#make sure openocd is ran as root!
echo "Ta da!  Use \"sudo openocd -f /usr/local/share/openocd/scripts/board/ek-lm3s6965.cfg\" to start the OpenOCD server."





