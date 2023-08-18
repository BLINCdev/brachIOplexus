# BentoArm_Utilities
Bento Arm utilities based off udp communication with bento arm such as state recorder, playback, kinematics and RL loops

---
## State Recorder

State recorder simply records a series of each joint angle in a csv file.

To use state recorder TX is enabled and on the proper port and UDP - Communication is connected.  

If you would like to record the arm movements by manually moving it nothing extra needs to be done, but if the arm is 
currently locked in its position you will have to ensure the Torque is off, and you may have to power the arm off and on
again. If you would like to use the built-in controllers to record the motion ensure that the controller is connected 
and the Torque On is enabled in the Bento Arm menu.

Simply run

```python3 state_recorder.py```

By default, printing the current position is disabled.  Ctrl-C the program or hit the stop button twice in PyCharm to 
finish recording, write the csv file, and end the program.

---
## State Player

State player will play back a previously recorded csv file.  Ensure that Torque On is enabled in the Bento Arm menu to
enable robot movement.

To use simply run

```python3 state_player.py <name_of_csv_file>```

---
### Normalized State

By default the joint states are normalized between 0 and 1 rather than the raw joint states of values typically around 
the range of 1500 - 3000.  These raw states are what is required by the BrachIOPlexus RX functions for moving the robot.
By default the python application will take in a range from [0,1] as it is more stateful / RL friendly and will convert
these values to the raw states.  You can override this by setting the normalized argument to false in both the player 
and recorder.