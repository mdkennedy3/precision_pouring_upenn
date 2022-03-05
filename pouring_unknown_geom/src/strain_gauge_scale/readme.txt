To run this needs the HX711 library found at: 

	https://github.com/bogde/HX711.git

This must be copied into the arduino libraries folder, then the library should be added (sketch->add library) then add folder HX711. 
Folder may be autoloaded on linux system, again arduino library folder, include library, manage library (then HX711 should be there)

make sure that arduino is run with sudo priveledges in order to send and receieve over the usb

Make ROS arduino compatible: 
First make ros libraries available to arduino:
http://wiki.ros.org/rosserial_arduino/Tutorials

compile messages with 
  rm -rf ros_lib
  rosrun rosserial_arduino make_libraries.py . (in sketch/libraries folder)
this should find your messages 

then for ros to see messages:

sudo chmod 666 /dev/ttyACM0 (or whatever port its on)
run rosserial_python serial_node.py /dev/ttyACM0
