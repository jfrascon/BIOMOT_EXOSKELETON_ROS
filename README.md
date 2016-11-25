### What is this program for?

CAN communication with BIOMOT exoskeleton actuators using ROS.

### How do I get set up?

The root folder contains both the nodes to be run on the BeagleBoard Black [BBB](in scr_bbb) and on a remote PC (in src). To compile, put the folder in a catkin-workspace and run catkin_make (see ROS documentation for setup and initialization of the catkin workspace: http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

To build the nodes provided with a user interface (ending with "Ui"), Qt libraries need to be set-up on your system.

Note that the standard c++11 is used to manage threads, so be sure that your compiler supports it.

To handle interrupts, some functions from the glib2 library were used. To set-up it on the BBB run: -sudo apt-get install libgtk2.0-dev

To get automatic start-up on the BBB, just add the script launch_ROS_nodes_in_bbb.sh to crontab on the bbb with the option restart.

To start the whole GUI run the script launch_ROS_nodes_in_client.sh on the PC connected to the BBB (note that to source ROS files, the scripts use absolute paths and fixed IP adresses that need to be changed).

### Contribution guidelines

Please refer to the file BioMot_Project.pdf for further information.

### Who do I talk to?

Juan Francisco Rascón Crespo: jfrascon@gmail.com  
Marco Matteo Bassa: bassamarco91@gmail.com
