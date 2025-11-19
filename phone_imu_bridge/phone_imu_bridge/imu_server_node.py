import asyncio
import json
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import websockets
import threading

class ImuServerNode(Node):
    def __init__(self):
        super().__init__('imu_server_node')
        self.publisher_ = self.create_publisher(Imu, 'phone_imu', 10)
        self.get_logger().info('IMU Server Node started. Listening on 0.0.0.0:5000')
        
        # Start WebSocket server in a separate thread
        self.server_thread = threading.Thread(target=self.start_server_thread, daemon=True)
        self.server_thread.start()

    def start_server_thread(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        start_server = websockets.serve(self.handle_connection, "0.0.0.0", 5000)
        loop.run_until_complete(start_server)
        loop.run_forever()

    async def handle_connection(self, websocket):
        self.get_logger().info('New client connected')
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    self.publish_imu(data)
                except json.JSONDecodeError:
                    self.get_logger().warn('Received invalid JSON')
                except Exception as e:
                    self.get_logger().error(f'Error processing message: {e}')
        except websockets.exceptions.ConnectionClosed:
            self.get_logger().info('Client disconnected')

    def publish_imu(self, data):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "phone_imu_link"

        # Covariance matrices. They are published for algothimns to use, but we can't
        # get them from the phone IMU.
        # 1e-6 for diagonal elements is a common default for "trustworthy" sensors
        # 0.01 might be more realistic for a phone
        # TODO: Make this customizable
        cov_val = 0.01
        msg.linear_acceleration_covariance = [
            cov_val, 0.0, 0.0,
            0.0, cov_val, 0.0,
            0.0, 0.0, cov_val
        ]
        msg.angular_velocity_covariance = [
            cov_val, 0.0, 0.0,
            0.0, cov_val, 0.0,
            0.0, 0.0, cov_val
        ]
        msg.orientation_covariance = [
            cov_val, 0.0, 0.0,
            0.0, cov_val, 0.0,
            0.0, 0.0, cov_val
        ]

        if 'accel' in data:
            # Convert from g to m/s^2
            g_to_accel = 9.81
            msg.linear_acceleration.x = float(data['accel'].get('x', 0)) * g_to_accel
            msg.linear_acceleration.y = float(data['accel'].get('y', 0)) * g_to_accel
            msg.linear_acceleration.z = float(data['accel'].get('z', 0)) * g_to_accel

        if 'gyro' in data:
            msg.angular_velocity.x = float(data['gyro'].get('x', 0))
            msg.angular_velocity.y = float(data['gyro'].get('y', 0))
            msg.angular_velocity.z = float(data['gyro'].get('z', 0))

        if 'orientation' in data:
            # Expo DeviceMotion returns alpha, beta, gamma (Euler angles)
            # alpha: Z axis (0-360), beta: X axis (-180 to 180), gamma: Y axis (-90 to 90)
            # We need to convert these to Quaternion
            alpha = float(data['orientation'].get('alpha', 0))
            beta = float(data['orientation'].get('beta', 0))
            gamma = float(data['orientation'].get('gamma', 0))
            
            q = self.euler_to_quaternion(alpha, beta, gamma)
            msg.orientation.x = q[0]
            msg.orientation.y = q[1]
            msg.orientation.z = q[2]
            msg.orientation.w = q[3]
        else:
            # Default identity quaternion
            msg.orientation.w = 1.0
        
        self.publisher_.publish(msg)

    def euler_to_quaternion(self, alpha, beta, gamma):
        # Convert Euler angles to Quaternion
        
        roll = beta
        pitch = gamma
        yaw = alpha

        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)

        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = ImuServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
