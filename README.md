# brachI/Oplexus
The brachI/Oplexus software is a digital nerve center for connecting human interfaces to robotic arms. The latest release of this software includes support for controlling our open source robotic platform - [The Bento Arm](https://github.com/blincdev/Bento-Arm-Hardware) – with an Xbox 360 controller, Myo Armband, or keyboard. Future releases will include additional human interfaces and robotic arms.

NOTE: brachI/Oplexus is pronounced 'brack-I-O-plexus' and is inspired by the anatomical term 'brachial plexus' which is the main network of nerves that connects the brain and spinal cord to your arm.

## Getting Started
Instructions for installing, running, and modifying the software can be found in [brachIOplexus_User_Guide.pdf](https://github.com/blincdev/brachIOplexus/blob/master/brachIOplexus_User_Guide.pdf) The user guide will be your main goto document while setting up the software for the first time and includes references to the other files in the repository:

* __brachIOplexus_installer_V1.zip:__ A standalone installer file that can be used to install the software without having to dig into the source code or install visual studio. Also available from the [releases tab](https://github.com/blincdev/brachIOplexus/releases)
* __MTT GUI.sln:__ The main visual studio project file for opening the source code. The project was built in Visual Studio Express 2015.

NOTE: If you want to modify the project files in Visual Studio Express 2015 you will need to unblock  brachIOplexus-master.zip before extracting as described in the [following guide](https://blogs.msdn.microsoft.com/delay/p/unblockingdownloadedfile/).

## Contributing
You can share your experience, new design, ideas, feature requests, or questions by contacting us through the [BLINCdev website](https://blincdev.ca/).

If you would like to contribute to future official releases of the brachI/Oplexus we recommend contacting us on the forums to coordinate with our development team. To get started you will need to [fork this repo](https://help.github.com/articles/fork-a-repo/) and once your modification or enhancement is complete submit a [pull request](https://help.github.com/articles/using-pull-requests/).

## License
The brachI/Oplexus software is released under the [GNU General Public License v3](https://www.gnu.org/licenses/gpl.html). A local copy is available in the repository.

## Libraries
The following libraries and interfaces were used in this project. Their respective licenses are included for reference in the libraries folder. 

* [XInputDotNet](https://github.com/speps/XInputDotNet): used to communicate with Xbox controllers
* [DynamixelSDK](https://github.com/ROBOTIS-GIT/DynamixelSDK): used to communicate with Dynamixel Actuators in The Bento Arm
* [MyoSharp](https://github.com/ROBOTIS-GIT/DynamixelSDK): used to communicate with the Myo Armband
* [Simple Moving Average Algorithm](https://www.codeproject.com/Articles/17860/A-Simple-Moving-Average-Algorithm): used to smooth the muscle signals from the Myo Armband
* [Simulink Real-Time API For Microsoft .NET Framework](https://www.mathworks.com/help/xpc/api/using-api-for-net-framework.html): will be used in a future release to communicate with commercial electromyography (EMG) systems


 