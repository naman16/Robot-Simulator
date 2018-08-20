# CSCI3081W Project

## Overview of the Software Project
The goal of this software project is to develop a robot simulator in which robot behaviors are visualized within a graphics window. The project makes use of the OpenGl (https://www.opengl.org/) Graphics library and the nanogui library (https://github.com/wjakob/nanogui) for graphics and interaction with the user. The robots have been modeled after Braitenberg Vehicles, created by Valentino Braitenberg, which use simple designs to explain complex behavior. The simulation environment has multiple robots, as well as stimuli - lights and food. Robots exhibit different behaviors in reaction to these stimuli, depending on their behavior towards the stimulus. Depending on the user's preference, a desired number of robots can be made to fear and explore light. Furthermore, depending on whether food is on or off as per the user's requirement, robots start behaving differently as they get hungrier in the simulation. 

## User Guide for Technical Users
The program execution begins from the Controller which instantiates the Arena and the GraphicsArenaViewer. Arena is the main class for the simulation and it handles the creation and interactions among all the entities. Arena is at the center of this whole simulation. GraphicArenaViewer on the other hand, is responsible for the Graphics of the simulation as well as for handling interactions with and instructions from the user. Controller mediates the communication between the GraphicsArenaViewer and Arena. In a nutshell, the Controller is first called when the simulation begins, then the Controller instantiates Arena (which creates all the entities in the arena) and the GraphicsArenaViewer(which uses MinGfx to open a graphics window in which all the actions are viewed). Once the creation is done, UpdateSimulation() of GraphicsArenaViewer is called repeatedly (until the application graphics window is closed) and this method advances time by a certain amount and communicates this time to the Controller which then communicates the time to the Arena. Arena then executes the UpdateEntitiesTimestep() method which is responsible for updating every entity in the arena at every timestep and then checking for interactions among the entities. This is the general flow of the control for the simulation. 


In this simulation, there can be multiple lights, robots and food. Furthermore, robots and light are mobile whereas food is immobile. Due to these reasons, the Arena has a vector of robots, mobile entities (that include both lights and robots) and entities (that include food, lights and robots). Vectors are used for this simulation because the number of each type of entity is based on the user and is not fixed. Hence vectors help in making the project more robust since it provides the user the flexibility of adding as many lights, robots and food as they want. Moreover, using vectors make it easier to deal with the interactions between different entities since we we can just loop through the entities, mobile entities or robots and check for interactions and colllisions. 

The robots have light and food sensors to decide its behavior towards lights and food respectively. For light sensors, based on the distance of the robot from the light a sensor reading is calculated and then depending on the behavior of the robot towards light, a particular motor connection is implemented. The extent to which the motion of the robot gets impacted depends on the sensor reading and the sensor reading in turn depends on the distance of the robot from the light. If the robot fears light, it implement positive, direct sensor-motor connections, meaning the reading from the right sensor impacts the velocity of the right wheel, and the left sensor impacts the left wheel. If the robot explores light, it implements negative, crossed sensor-motor connections meaning the reading from the right sensor impacts the velocity of the left wheel, and the left sensor impacts the right wheel. When the robot becomes hungry, then it actively starts looking for food and becomes aggressive.
Aggression is implemented with positive, crossed sensor-motor connections. This means that there is a positive correlation between the right sensor and left wheel, and between the left sensor and right wheel. Again, the extent to which it moves aggressively depends on the food sensor reading, which in turn depends on the distance of the robot from the food. 

One key design decision that was made was how the sensors receive information about the stimuli they are sensing. This was done using an Observer Pattern where the stimuli are the subjects and the robots are observers of stimuli, in that a Robot would register to receive information from the Arena about stimuli that it is actively sensing. At each time step, the robot receives information about every single stimulus in the arena from the Arena class since arena contains all the stimuli. (Even though the information is being passed through the arena, the robot observes and gets affected only by the stimuli.) Then based on whether the stimulus is light or food, robot uses its light or food sensors to calculate a sensor reading value pertaining to that particular stimulus to gauge how much is that stimuli affecting the robot. After which, depending on the value of the reading and its behavior towards that stimulus, the robot first decides whether its behavior needs to be changed and if so, it updates its motion and behavior accordingly.

Another important design decision was made regarding how to handle the different types of motion that a robot can have. 
Separate classes for each type of robot motion have been defined and each of them is a child class of MotionHandlerRobot. The rationale for creating separate classes is that based on the robot’s nature and behavior, its motion can vary so there are separate classes for dealing with each specific motion. Within the robot class, however, there is instance of only MotionHandlerRobot to handle all kinds of robot motion. Since MotionHandlerRobot is the parent class of all the different types of robot motion, we use the instance of MotionHandlerRobot itself, defined within robot, to call the class of the specific motion (which will be one of the child classes of MotionHandlerRobot) that is required, rather than creating instances of all possible motions within robot and then using those instances to call the required class. This Strategy Pattern is used because a robot has more than one motion and based on robot’s behavior and the stimulus, the robot decides which motion to implement and this decision is made at runtime. Since the decision is made at runtime, having just an instance of MotionHandlerRobot and then using this instance to call the specific motion as and when the situation is encountered allows to make use of polymorphism, thereby bypassing the need to create separate instances for each kind of motion within the robot.


## User Guide for Non-Technical Users

Robots are modeled after the Braitenberg Vehicles and they have 2 light sensors and 2 food sensors at the same location. The robots and lights are mobile and food is immobile. Users, based on their preference, can have any ratio of robots fear or explore lights. Fear motion of robots is implemented with positive, direct sensor-motor connections, meaning the reading from the right sensor impacts the velocity of the right wheel, and the left sensor impacts the left wheel. Due to this reason, the robots that fear lights, are always moving away from the light sources. Exploratory motion of robots is implemented with negative, crossed sensor-motor connections meaning the reading from the right sensor impacts the velocity of the left wheel, and the left sensor impacts the right wheel. Due to this reason, the robots that explore lights start moving slower as they get closer to light sources. Furthermore, if the food option is turned on by the user, the robots can get hungry if they don't get food for 30 seconds. Once the robot gets hungry, it starts actively sensing for food and becomes aggressive. Aggression motion of robots is implemented with positive, crossed sensor-motor connections. This means that there is a positive correlation between the right sensor and left wheel, and between the left sensor and right wheel.

The application window opens with a menu to the left side that has the following labels - within "Simulation Control" - "Play" and "Welcome"; within Arena Configuration -  "Number of Robots", "Number of Lights", "Number of Food", "Food On Off", "Percentage Fear", "LightSensor Sensitivity". 
- "Number of Robots" - This slider is used to specify the number of robots needed in the simulation. The default value is 5 and the user can move the slider to have any number of robots between 0-10. 
- "Number of Lights" - This slider is used to specify the number of lights needed in the simulation. The default value is 0 and the user can move the slider to have any number of lights between 0-5. 
- "Number of Food" - This slider is used to specify the number of food entities needed in the simulation. The default value is 0 and the user can move the slider to have any number of food between 0-5. 
- "Food On Off" - This slider is used to specify if the food feature is turned on/off for the simulation. If the food is turned on, then the robot should get hungry and require food. If the food is turned off, then robot does not get hungry and hence does not require food. 
- "Percentage Fear" - This slider specifies the percentage of robots that fear light in the simulation.  The default value is 0% and the user can move the slider to choose any percentage of robots between 0-100% that fear light. The remaining robots(if any) will then explore lights. 
- "LightSensor Sensitivity" - This slider is used to specify the sensitivity of the light sensor in the simulation. This light sensitivity value is used as the base value in the sensor reading calculation. The value for light sensitivity ranges between 1.0-1.1 and the default value is 1.0. 

Once the user specifies the value for all these fields, he/she can then click on the "Play" button and this begins the simulation. Once the simulation begins, the "Play" button becomes "Pause"  and the "New Game" becomes visible. The user can then at any point in the simulation, pause or start a new simulation by clicking on these buttons. Once the "Pause" button is pressed, the simulation is paused and the button becomes "Play". The user can then resume the simulation by clicking on the "Play" button. When the user selects the "New Game" button, it resets the arena and stops the simulation. The user can then specify values for each field within Arena Configuration to configure the arena and once the values have been selected, the user can click on the "Play" button to begin the simulation once again. 




