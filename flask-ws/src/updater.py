from flask import Flask, request, jsonify
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import json

app = Flask(__name__)
rclpy.init(args=None)

class ROSPublisher(Node):
    def __init__(self):
        super().__init__('parameter_publisher')
        self.publisher_no_payload = self.create_publisher(String, 'parameter_update_no_payload', 10)
        self.publisher = self.create_publisher(String, 'parameter_update', 10)

    def publish_update(self, message):
        msg = String()
        msg.data = message
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')

    def publish_update_no_payload(self, message):
        msg = String()
        msg.data = message
        self.publisher_no_payload.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')

ros_publisher = ROSPublisher()

# This is an API that will provide an up to date and most recent, perhaps even versioned, 
# configuration parameters payload. 
@app.route('/update_parameters', methods=['POST'])
def update_parameters():
    data = request.get_json()
    if data and 'trigger_update' in data and data['trigger_update']:
        # Extract the parameters array
        parameters = data.get('parameters', [])
        
        # Check if the parameters list is empty
        if not parameters:
            return jsonify({"error": "Parameters array is empty"}), 400
        
        # Convert the parameters array to a JSON string
        parameters_str = json.dumps(parameters)
        
        # Forward the entire parameters array to the ROS node
        ros_publisher.publish_update(parameters_str)
        
        return jsonify({"status": "Parameters updated successfully"}), 200
    else:
        return jsonify({"error": "Invalid request"}), 400
    
# This is an API end point for notification of configuration update without the payload
# Consequently, each ros node would go ahead and fetch the configuration as needed.
# To reduce network constraints, this can be broken down into autonomy pillars, 
# eg: localization, deployment, path planning, perception, etc. 
# Issue with this approach is the overhead and duplication of having each ros node query
# the web server for config.
@app.route('/update_parameters_no_payload', methods=['POST'])
def update_parameters_no_payload():
    data = request.get_json()
    if data and 'trigger_update' in data and data['trigger_update']:
        # Simulate fetching parameters and publish the update
        ros_publisher.publish_update_no_payload('New parameters fetched from the server')
        return jsonify({"status": "Parameters updated successfully"}), 200
    else:
        return jsonify({"error": "Invalid request"}), 400
    
def run_ros_node():
    rclpy.spin(ros_publisher)
    ros_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    # Start the ROS node in a separate thread
    ros_thread = threading.Thread(target=run_ros_node)
    ros_thread.start()

    # Start the Flask server
    app.run(host='0.0.0.0', port=6000)
