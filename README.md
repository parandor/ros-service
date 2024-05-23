Challenge: 

The research team have developed a new locus_ai component for our robots to detect butterflies in real-time. Parameters for the butterfly model are available on from a web-service running in the cloud. 

How would you (in a few hours) extend the bringup to enable real-time reconfiguration of the detection system? What suggestions can you make for larger changes in the system architecture?
How do your changes make the reconfiguration process smoother or more reliable?



Design Assumptions: 

Assume that robots are resource constrained, battery powered, and on an unreliable wi-fi network. 
Assume that the Flask webserver is running in the cloud and the robots are running on a client network behind a NAT/firewall. 
The client network will have multiple robots, for this scenario assume 2 robots are deployed. 
Finally, a single robot will have hundereds of nodes that will be requiring parameters

Included within this folder is a sample ROS2 project as well as a Flask web-service to help get you started in your design. Please see the README on its execution.
The sample contains a service that defaults with a "Hello World" parameter, as well as a launch.py file that injects a new parameter on launch. Please feel free to adjust this accordingly as you see fit. 

Guidelines: 

1) Please do not go too overboard with your solution, however it should try and cover the main points. 
2) Should ideally take a couple of hours. 
3) Provide some level explanation on your solution as well as how to run it.
4) BONUS: 
    a) Leverage docker to setup/bring-up the individual nodes
    b) How would your design extend to support hundreds of robots on a client network

Build:

`docker build -t ros2_service_project .`

Run:

`docker run -it --rm ros2_service_project`