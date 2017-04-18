# brachI/Oplexus
The brachI/Oplexus software is a digital nerve center for connecting human interfaces to robotic arms. The initial release of this software includes support for controlling our open source robotic platform - [The Bento Arm](https://github.com/blincdev/Bento-Arm-Hardware) – with an Xbox 360 controller. Future releases will include additional human interfaces and robotic arms.

NOTE: brachI/Oplexus is pronounced 'brack-I-O-plexus' and is inspired by the anatomical term 'brachial plexus' which is the main network of nerves that connects the brain and spinal cord to your arm.

## Getting Started
Instructions for installing, running, and modifying the software can be found in [brachIOplexus_User_Guide_rev1.pdf](https://github.com/blincdev/brachIOplexus/blob/master/brachIOplexus_User_Guide_rev1.pdf) The user guide will be your main goto document while setting up the software for the first time and includes references to the other files in the repository:

* __brachIOplexus_installer_V0_6.zip:__ A standalone installer file that can be used to install the software without having to dig into the source code or install visual studio. Also available from the [releases tab](https://github.com/blincdev/brachIOplexus/releases)
* __MTT GUI.sln:__ The main visual studio project file for opening the source code. The project was built in Visual Studio Express 2015.
* __xbox_controller_mapping.png:__ A graphical illustration of the mapping between Xbox controller axes and joint velocities on the Bento Arm

## Contributing
You can share your experience, new design, ideas, feature requests, or questions by contacting us through the [BLINCdev website](https://blincdev.ca/).

If you would like to contribute to future official releases of the brachI/Oplexus we recommend contacting us on the forums to coordinate with our development team. To get started you will need to [fork this repo](https://help.github.com/articles/fork-a-repo/) and once your modification or enhancement is complete submit a [pull request](https://help.github.com/articles/using-pull-requests/).

## License
The brachI/Oplexus software is released under the [GNU General Public License v3](https://www.gnu.org/licenses/gpl.html). A local copy is available in the repository.

## Libraries
The following open source libraries and interfaces were used in this project. Their respective licenses are included for reference in the libraries folder. 

* [XInputDotNet](https://github.com/speps/XInputDotNet): used to communicate with Xbox controllers
* [DynamixelSDK](https://github.com/ROBOTIS-GIT/DynamixelSDK): used to communicate with Dynamixel Actuators in The Bento Arm

 