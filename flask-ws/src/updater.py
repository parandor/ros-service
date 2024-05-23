from flask import Flask, request, jsonify
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading

app = Flask(__name__)
rclpy.init(args=None)

class ROSPublisher(Node):
    def __init__(self):
        super().__init__('parameter_publisher')
        self.publisher = self.create_publisher(String, 'parameter_update', 10)

    def publish_update(self, message):
        msg = String()
        msg.data = message
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')

ros_publisher = ROSPublisher()

@app.route('/update_parameters', methods=['POST'])
def update_parameters():
    data = request.get_json()
    if data and 'trigger_update' in data and data['trigger_update']:
        # Simulate fetching parameters and publish the update
        ros_publisher.publish_update('New parameters fetched from the server')
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
