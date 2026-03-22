import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import json
import threading
import websocket

class WebsocketClientNode(Node):
    def __init__(self):
        super().__init__('websocket_client_node')
        
        # Grab the car's namespace to use as its unique ID (e.g., 'car1')
        self.car_id = self.get_namespace().strip('/') or 'car_unknown'
        
        # Connect to your central server (Replace with actual server IP)
        self.ws_url = "ws://<CENTRAL_SERVER_IP>:8765"
        self.ws = None
        self.setup_websocket()

        # Subscribe to the car's own localization pose
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.pose_callback,
            10)

    def setup_websocket(self):
        self.ws = websocket.WebSocketApp(
            self.ws_url,
            on_message=self.on_message,
            on_error=self.on_error,
            on_close=self.on_close
        )
        self.ws.on_open = self.on_open
        
        # Run the websocket listener in a separate thread
        self.ws_thread = threading.Thread(target=self.ws.run_forever)
        self.ws_thread.daemon = True
        self.ws_thread.start()

    def on_open(self, ws):
        self.get_logger().info(f'{self.car_id} connected to central server.')

    def on_message(self, ws, message):
        # This triggers when OTHER cars broadcast their data
        data = json.loads(message)
        self.get_logger().info(f"Received data from {data.get('car_id')}: x={data.get('x')}, y={data.get('y')}")
        
        # TODO: Pass this data to your Costmap2D plugin to represent the moving obstacle

    def on_error(self, ws, error):
        self.get_logger().error(f'Websocket error: {error}')

    def on_close(self, ws, close_status_code, close_msg):
        self.get_logger().warn('Disconnected from central server.')

    def pose_callback(self, msg):
        # When this car gets its own pose, push it to the shared websocket
        if self.ws and self.ws.sock and self.ws.sock.connected:
            payload = {
                'car_id': self.car_id,
                'x': round(msg.pose.pose.position.x, 3),
                'y': round(msg.pose.pose.position.y, 3),
                # Add size and velocity data here later
            }
            self.ws.send(json.dumps(payload))

def main(args=None):
    rclpy.init(args=args)
    node = WebsocketClientNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()