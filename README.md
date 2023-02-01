# robotic_vision

This project contains vision detection for a simulated robot arm in ROS. 

The code detects the joints from a visual input. As the joints move, vector concepts are used to determine the state of the robotic arm (The position of a joint is expressed in the angle by which it deviates from its default position). 

The first task is a robotic arm the joints of which only rotate around one axis. In the second, the rotate around 2 axes. making the task more complex due to ambiguity. 
