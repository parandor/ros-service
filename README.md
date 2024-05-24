# Challenge: 

The research team have developed a new locus_ai component for our robots to detect butterflies in real-time. Parameters for the butterfly model are available on from a web-service running in the cloud. 

## How to Extend Bringup for Real-Time Reconfiguration

1. **Enable Real-Time Reconfiguration**: Implement communication between the ROS 2 nodes and the web service to fetch updated parameters in real-time.

2. **Dynamic Reconfiguration**: Integrate ROS 2's dynamic reconfigure package to allow nodes to adjust their behavior based on parameter updates.

3. **Node Restart Mechanism**: Implement a mechanism for nodes to gracefully restart when necessary to apply new parameters.

## Suggestions for Larger Changes in System Architecture

1. **Decentralized Parameter Management**: Distribute parameter management across multiple nodes to reduce load and improve scalability.

2. **Offline Parameter Caching**: Implement offline caching of parameters on robots to handle intermittent connectivity issues.

3. **Edge Computing**: Explore edge computing solutions to offload parameter processing tasks from resource-constrained robots.

# Design Assumptions:

- Robots are resource constrained, battery powered, and on an unreliable Wi-Fi network.
- The Flask webserver is running in the cloud, and robots are behind a NAT/firewall in a client network.
- Multiple robots are deployed in the client network.
- A single robot will have hundereds of nodes that will be requiring parameters
- The client network will have multiple robots. For this scenario assume 2 robots are deployed.

# Included Samples:

A sample ROS2 project and Flask web-service are provided. Please see the README for execution instructions.

# Guidelines:

1. Please do not go too overboard with your solution, however it should try and cover the main points.
2. Aim for completion within a couple of hours.
3. Provide explanation on the solution and how to run it.
4. Bonus: 
    - Use Docker to set up and bring up individual nodes.
    - Extend design to support hundreds of robots on a client network.

## Build:

`docker build -t ros2_service_project .`

## Run:

`docker run -it --rm ros2_service_project`

## Trigger Configuration Update:

### Config Update No Payload

`curl -X POST -H "Content-Type: application/json" -d '{"trigger_update": true}' http://localhost:6000/update_parameters_no_payload`

### Config Update With Payload

`curl -X POST -H "Content-Type: application/json" -d '{
  "trigger_update": true,
  "parameters": [
    { "my_parameter": "from_flask" }
  ]
}' http://localhost:6000/update_parameters
`