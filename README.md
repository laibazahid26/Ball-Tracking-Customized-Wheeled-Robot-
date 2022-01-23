### Introduction and Objectives: 

In this assignment we implemented and simulated three behaviors of MIRO Robot. Those three behaviors are:

* Normal 
* Sleep  
* Play  

These three behaviors were implemented inside a finite state machine which was built using ROS library called SMACH. This node is named as assignment2.py file. This assignment is built up on the assignment 1. The difference is that the assignment 1 was a high level assignment, we did not have the real simulation of the robot, we just had to simulate the behaviors on the terminal.

Whereas, assignment number 2 is a low level assignment, which mean  that we actually deal with the real robot in simulation and deal with velocity commands as well.   

Following are the objectives of this assignment. 

* Make a wheeled dog robot with additional links and joint. 
*  Build the complete ROS architecture and simulate it on Gazebo.
*  Launch the complete assignment using a launch file. 

### Software Architecture:

Below is the image of my architecture. 

![alt text](https://github.com/laibazahid26/exp_rob_assignment_2/blob/master/architecture.png?raw=true)

The camera takes the picture of the entire arena. The arena is a fixed 8x8 units area. This arena may or may not have a ball and has a robot. The “Rigid Body Detector” node takes data from the camera and detects all the rigid bodies on the image and find their coordinates as well for example, the robot and the ball can be present on the image. Depending if the ball was detected by the Rigid Body Detector or not ROS parameter server detectPlayFlag is set. There is another parameter used for checking in which state the finite state machine currently is, and that parameter is called as state. Basing on these two parameters the “The Command Manager” node produces a final target. If the ball was detected then OpenCV was used to follow the ball. If the ball was not present then a random target can be produced (normal behavior). The “Command Manger” node then gives the target to the “Path Planner” node and the “Path Planner” node after knowing the current position of the robot and the target, publishes a complete trajectory which the robot needs to follow. Finally, the “Robot Controller” node generates velocity commands one by one to reach to each vector.  

### List of messages of the proposed Architecture:

* ROS Parameter Server sets the playflag, that is playflag is set to 1, if the ‘play’ command was given, otherwise this flag stays 0. ROS Parameter server also sets the parameter ‘state’. This paramter is a string and can contain the values like normal state, sleep state and, play_state.  
* The Rigid Body Detector node provides the position of the ball in the arena as a vector. This vector will be of type geometry_msgs/Point.msg. 
* The Command Manager node gives a target which is the destination of the robot and this target is a vector. This vector will be of type geometry_msgs/Point.msg.  
* Path planner node provides a vector of vector. Which means it contains all the points which the robot needs to follow to reach to the final destination. 
* Robot Controller node will generate velocity commands to reach to each vector and it will have type Twist. 


### State Diagram:

Below is the state diagram of this architecture. This finite state machine lies inside the command manager node of my architecture.

![alt text](https://github.com/laibazahid26/exp_rob_assignment_2/blob/master/finiteStateMachine.png?raw=true)

As can be seen, there are three states, i.e. Normal, Sleep and, Play. The robot comes to life in being in the Normal state. Since we were required to ‘decide’ upon when the robot goes to sleep, we decided that whenever the robot finishes the normal behavior ‘or’ the ball was not detected for 10 seconds, then the robot goes to the Sleep behavior. We also decided that the robot will come out of the Sleep behavior when it finishes its sleeping time, i.e. after finishing sleep time, the robot goes back to the Normal state. 

On the other hand, if the robot is in the Normal state and a ball is detected, then the robot enters in the Play behavior (not in the Sleep behavior, because in my architecture, Play behavior has priority over Sleep behavior). After finishing the Play behavior, the robot goes back to the Normal behavior. 

### Packages and File List:

We were already provided with a package called exp_experiment 2. Inside this folder we were given with an action server implemented for moving the ball. The action server is launched in the launch file.  Using this action server, we can move the ball inside our arena by publishing x,y,z coordinates on topic '/ball/cmd_vel'. Inside the ‘action’ folder lies an action file in which we define the type and format of the goal, result, and feedback topics for the action. Inside the config folder we define the motor configuration for out robot inside a yaml file. In yaml file we have to describe all the controllers for all the joints and their pid parameters along with joint state controller. The launch file is placed inside the launch folder.  Whereas, inside the script folder lies our two nodes for this assignment: one node is called as assignment2.py and the other as go_to_point_ball.py. Inside the urdf folder, lies the xacro and gazebo files which together describe the description and functionalities of the robot, ball and the human to be spawned in the Gazebo simulation. To be specific, xacro file describes elements of the robots i.e. the links, joints, their mass, inertia and their complete geometry. Whereas in gazebo file we define the plugins i.e. additional properties needed for simulation purposes in Gazebo. Inside the world folder there lies a world file which describes the environment of our Gazebo world.  

### Steps to Control the Robot:

* There is a package called “ROS Control”. Install this package by using the following commands:

  `sudo apt-get install ros-[ROS_version]-ros-control ros-[ROS_version]-ros-controllers`
      
  `sudo apt-get install ros-[ROS_version]-gazebo-ros-pkgs ros-[ROS_version]-gazebo-ros-control`
      
 This package at allows for setting robot links positions and adds the functionality of letting the arm go to the goal position.

* Add <transmission tag> in your xacro file, for each joint of the robot we have to put a <transmission > tag. In this assignment we just used one transmission tag because we wanted to control just one joint (the joint for rotating the head).

* In the gazebo file define the plugin. 

* Inside the config folder we have a yaml file which is the motor configuration file, here we describe the joint state controller and for controlling each joint we define a controller and the pid parameters. 

* Finally in the launch file, we need to spawn to launch a node which is of pkg = ‘controller-manager’.
      
### Installation and Running Procedure:

* Clone the repository by going on the link:
	
  `https://github.com/laibazahid26/exp_rob_assignment_2`
	
* Place this folder inside the src folder of your workspace. 
* Go in your workspace using terminal and then do a catkin_make.
* In the previous step you were in your workspace. Now move to the scripts folder of your exp_assignment2 package and give running permissions to it by writing the following command:
	
  `chmod +x assignment2.py`
	
* Now, we are in the position to run the assignment completely. We can do it by launching the .launch file by running the following command on the terminal. 
	
  `roslaunch exp_assignment2 gazebo_world.launch` 

After running the above command you will see the robot waking up in the Normal behavior and then depending on the presence of the ball will shift to some other state. You can publish make the ball move by publishing on the '/ball/cmd_vel'. You can make the ball vanish as well by giving a minus coordinate in the z-axis.  

### Working Hypothesis and Environment: 

This package simulates the Normal, Sleep and, Play behavior of a real robot simulated in the Gazebo environment. SMACH library was used to implement the finite state machine. These behaviors are implemented as classes in the assignment2.py file. On certain outcomes, the transition from one state/behavior to the other state happens.  

The important part of this architecture is randomizing various tasks performed by the robot. So, for example in Normal behavior the robot is supposed to move randomly. The question arises that how many times does the robot moves randomly? The answer is that, it does it randomly. In our architecture, this random value comes between 1 to 5 times. There are other functionalities as well which have been made random using the same logic, for example, how many seconds should the robot sleep? Same logic is used for dealing with this question as well. Sleeping time is randomized between  4 to 7 seconds. 
 
Below I’ll explain the working of each class, function, and, call-back, built inside our assignment2.py node. 

* main()

	In the main function first we have initialized the ROS parameters, we have just subscribed and 	published to a few topics. 

	Topics subscribed: 
	1. "/robot/odom"  (call-back function: newOdom)
	2. "/robot/camera1/image_raw/compressed"  ( Call-back function: detectBall)
       
	Topics published: 
	1. "/robot/cmd_vel"

	And in the main we have added the three behaviors/states in the Finite State Machine.      

* newOdom()
	
	This function is the callback of the the topic /robot/odom extracts the x and y location of the robot and converts the angle, which is in the quaternion form, of the robot with respect to the world frame into radians. These x,y,theta have been made global because they will be used in may other functions as well. 

* class Normal()
	
	In Normal state a random number times (from 1 to 5) random coordinates were generated within the area. These coordinates were then passed to the 	           go_to_goal() function, which takes care of moving the robot to the generated random coordinate. This function also checks the parameter "detectBall Flag", if the flag is 1, this means that the ball was detected, and then it transitions to the PLAY state. If no ball was detected, it then transitions to the SLEEP state after completing this behavior.

* go_to_goal()

	This function generates the velocity commands to move the robot to the coordinate received in the parameter. This function also keeps checking the parameter "detectBallFlag", and if this parameter is 1 and the robot is in the NORMAL state, the function stops, because this means that now the robot is not required to go to the given coordinate, and the robot just needs to follow the ball, and the function is no longer needed.
	
	Important information about the ‘Path Planner’ and the ‘Robot Controller’ node which are built inside the go_to_goal() function: 
The important thing to note here is that in the assignment we have implemented the Path Planner node and the Robot controller node in the same node built inside go_to_goal_function() 	and they work together. Following is their combined functionality:

	Keep rotating the robot until the robot gets aligned with the goal position.
	
	speed = Twist() 
	
	speed.linear.x = 0.0 
	
	speed.angular.z = 0.2
	
	
	Once the robot is aligned with the target keep generating the linear velocity till the robot has reached the goal. 
	
	speed = Twist() 
	
	speed.angular.z = 0.0
	
	speed.linear.x = 0.5 

* detectBall()
      
	This function is the callback of the Camera Image topic. This function uses Open CV to process the image received, and detects the contours of Green ball in that image. If any contour is found, it turns the Parameter "detectBallFlag" to 1, otherwise turns it to 0.

* class Play():
	In PLAY state the robot has to use its camera to track and follow the green ball. In this state, the image received from the Robot's camera is processed using Open CV to detect any contours of the green ball.
    	
	If any contour is found, the Robot is directed to go near the ball, otherwise the Robot is directed to rotate, and find the ball. Once the Robot has reached near the ball, the robot rotates its head right, and left upto 45 degrees and then again looks for the ball.
    
	The robot searches for the ball for 10 seconds. This functionality is implemented using a timer.
      	

### System’s limitations:

1. If the robot has to go to a certain target from the second and third quadrant, the controller does not work in a efficient way. After the robot has aligned 	  itself one time to the goal and then when it starts to move linearly towards the goal, the robot stopped and turn again in order to again aligns itself with the goal. 
	
	We believe this happens because when the robot has aligned itself once with the goal by generating angular velocity, then it produces linear velocity to move towards the goal. Because of inertia and wheel movement the robot gets a little nonaligned from the goal position and takes the complete revolution in order to align.
      
2. The robot takes the complete turn irrespective of the direction of the goal location. 

3. There is no way to come out of the Play behavior unless the Play behavior finishes. Only after finishing the Play behavior the robot goes back to the Normal behavior.
       
4. Error handling is not done in the system.
5. The launching this node, the node can’t be interrupted (by Ctrl-C) and has to be closed by completely by shutting the terminal. 

### Possible technical Improvements:

1. The controller could be made more efficient in terms of rotating while keeping in view the goal location.  

### Authors and Contacts:

I collaborated with a fellow colleague for doing this assignment. Only the writing of readme file and making the github repository was done individually.  
Below I mention the names and contacts of both of us. 

Author 1: Laiba Zahid (S4853477)

email ID: laibazahid26@gmail.com

Author 2: Syed Muhammad Raza Rizvi (S4853521)		

email ID: smrazarizvi96@gmail.com


### References:
    1. https://www.theconstructsim.com/ros-qa-053-how-to-move-a-robot-to-a-certain-point-using-twist/
 
