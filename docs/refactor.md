# REFACTORING:

## Using the Extract Method to refactor the code.
### Extract Method:
You have a code fragment that can be grouped together. <br/>
Turn the fragment into a method whose name explains the purpose of the method.<br/>

- Files: robot.cc, robot.h <br/>
  Methods changed - Added a new method void RobotStateUpdate() and modified the method
  TimestepUpdate().<br/>

- Line numbers 63-105 in robot.cc and line numbers 77-88 in robot.h. <br/>

- From my earlier implementation, I was checking if the robot is hungry, under a state of collision and if food is on or off within the
UpdatetimeStep() method. But this is not intuitive because this method is defined for only updating the robot's motion and position at every time step and not for checking for the various states robot can be in. Hence checking for these states and carrying out the required actions within timestepUpdate() method is not logical. Therefore, pulled this fragment of code out from this method and put it in a new method called RobotStateUpdate() that has the sole purpose of checking which state the robot is in and then depending on the state carrying out the required actions. From my earlier implementation, I was checking if the robot is hungry or under a state of collision within the UpdatetimeStep() method. But, TimestepUpdate() focuses on updating the robot's position and velocity after the specified duration of time has passed. Hence it is not a good idea to check if robot is hungry or under a state of collision within this method since this is not the intended purpose of the TimestepUpdate() method. Moreover, this is not intuitive because this method is defined for only updating the robot's motion and position at every time step and not for checking for the various states robot can be in. Hence checking for these states and carrying out the required actions within TimestepUpdate() method is not logical. Therefore, pulled this fragment of code out from this method and put it in a new method called RobotStateUpdate() that has the sole purpose of checking which state the robot is in and then depending on the state carrying out the required actions. Furthermore, from design standpoint, it is always better to have methods do very specific jobs and not a number of things and methods should also be named in accordance with the tasks they are required to perform. Hence I decided to extract a method from TimestepUpdate() so that TimestepUpdate() just updates the robot position and motion after specified time has elapsed and all the checks for the various states the robots can be in (and consequently carrying out the required action) is done in the RobotStateUpdate() method. <br/>

