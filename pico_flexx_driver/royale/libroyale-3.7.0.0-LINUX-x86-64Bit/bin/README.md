To run the Royale viewer, please execute royaleviewer.sh, it will 
automatically set the right path to the necessary libraries.

Execute the following command:
./royaleviewer.sh

The script sets the environment to use the libraries for Royale and Qt5.4.

Permissions on the USB device
-----------------------------

The directory "driver/udev" contains udev-rules files.
See https://wiki.debian.org/udev for explanation how
udev rules work.

If you are using ubuntu, copy the .rules file to
/etc/udev/rules.d/ to use pmd devices without administrative
privileges (you will need root rights once to copy the file
to the destination):

sudo cp ../driver/udev/10-royale-ubuntu.rules /etc/udev/rules.d

After copying the udev rule, please unplug and reinsert the camera.
