#!/bin/sh

[ "$(whoami)" != "root" ] && exec sudo -- "$0" "$@"

#Restore basrc
cp /etc/skel/.bashrc ~/

#Delete UDEV rules
if [ -f /etc/udev/rules.d/10-tas.rules ]; then
	sudo rm /etc/udev/rules.d/10-tas.rules
fi

exit
