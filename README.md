# Simulation for USV local path planning using Potential field vector

This is a small personnal project I have been working on. As a robotic engineering student, I wanted to use my theoritical knowledge on path planning and control for a practical use. 

This project aims to simulate a small Unmanned Surface Vehicle equipped with a GPS and an IMU. A complete physics moddel is made for the Surface system with fake drivers. 

The user can use the sensors data and implement algorithms to complete different tasks. 

## First task: Going to waypoints by avoiding obstacles.

The first mission that my USV is able to fulfill is to go to a set of waypoints ( by order for now ). It uses potential field vector to avoid random obstacles in its way. 

## The end goal

The end goal is to have a complete path planning using multiple USV to intercept multiple targets ( moving slowly in comparison to the USVs), each following a set of waypoint intelligently by using certain research algorithms. 

Ideas that I have for now: 
- Global path planning: A* or RRT*.
- Local, potential field vector
- Order waypoints using the nearest neigbourgh frist 


## Disclaimer

For a quick but temporary result to test the potential field vector algorithm, I got help from Claude to get the code for the graphical interface and the plot.py script. 

This will csoon be deleted in order to have a proper graphical interface, that will be made by my hand. 
