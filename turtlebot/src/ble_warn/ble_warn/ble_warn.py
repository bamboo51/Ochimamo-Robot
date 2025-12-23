import asyncio
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from bless import BlessServer, GATTCharacteristicProperties, GATTAttributePermissions
import logging
import threading

# Configs
SERVICE_UUID = "0000abcd-0000-1000-8000-00805f9b34fb"
CHARACTERISTIC_UUID = "00001234-0000-1000-8000-00805f9b34fb"
DANGER_DISTANCE = 0.55
STALE_DATA_TIMEOUT = 2.0 # seconds - remove track IDs
MAX_TRACK_IDS = 8

class DangerBroadcaster(Node):
    def __init__(self):
        super().__init__('ble_warn')
        self.subscription = self.create_subscription(
            String, "/people/wall_distance", self.distance_callback, 10
        )

        # thread-safe data structures
        self.lock = threading.Lock()
        self.latest_distances = {} # {track_id: {'distance': float, 'timestamp': Time}}
        self.current_mask = 0x00

        logging.basicConfig(level=logging.INFO)
        self.get_logger().info("Risk Broadcaster Started.")
    
    def distance_callback(self, msg):
        """Parse incoming distance messages and store with timestamp"""
        try:
            if ":" not in msg.data:
                self.get_logger().warn(f"invalid message format: {msg.data}")
                return
        
            parts = msg.data.split(":")
            if len(parts) != 2:
                self.get_logger().warn(f"Expected 2 parts, got {len(parts)}: {msg.data}")
                return
            
            track_id = int(parts[0])
            dist = float(parts[1])

            if track_id <= 0:
                self.get_logger().warn(f"Invalid track_id: {track_id}")
                return
            if track_id > MAX_TRACK_IDS:
                self.get_logger().warn(f"Track ID {track_id} exceeds max {MAX_TRACK_IDS}, ignoring")
                return
            
            with self.lock:
                self.latest_distances[track_id] = {
                    'distance': dist,
                    'timestamp': self.get_clock().now()
                }
        except ValueError as e:
            self.get_logger().warn(f"Parse error in {msg.data}: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in distance_callback: {e}")

    def compute_danger_mask(self):
        """
        Returns a single byte where each bit represents a danger flag for an ID:
        Bit 0 = ID 1, Bit 1 = ID 2, etc.
        Also removes stale entries.
        """
        mask = 0x00
        current_time = self.get_clock().now()

        with self.lock:
            stale_ids = []

            for track_id, data in self.latest_distances.items():
                age_ns = (current_time.nanoseconds - data['timestamp'].nanoseconds)
                age_sec = age_ns / 1e9

                if age_sec > STALE_DATA_TIMEOUT:
                    stale_ids.append(track_id)
                    continue

                if data['distance'] < DANGER_DISTANCE:
                    bit_position = track_id - 1
                    if 0<=bit_position<8:
                        mask |= (1<<bit_position)
            
            for track_id in stale_ids:
                self.get_logger().info(f"Removing stale track_id: {track_id}")
                del self.latest_distances[track_id]
        return mask & 0xFF

async def run(node, loop):
    node.get_logger().info("Initializing BLE Server")

    server = BlessServer(name="DANGER_BEACON", loop=loop)

    try:
        await server.add_new_service(SERVICE_UUID)
        char_flags = (
            GATTCharacteristicProperties.read |
            GATTCharacteristicProperties.notify |
            GATTCharacteristicProperties.indicate
        )
        permissions = (
            GATTAttributePermissions.readable |
            GATTAttributePermissions.writeable
        )

        await server.add_new_characteristic(
            SERVICE_UUID,
            CHARACTERISTIC_UUID,
            char_flags,
            value=b'\x00',
            permissions=permissions
        )

        node.get_logger().info("Starting BLE Advertising")
        await server.start()
        node.get_logger().info("BLE Advertising Active!")

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)
            new_mask = node.compute_danger_mask()

            if new_mask != node.current_mask:
                node.current_mask = new_mask
                node.get_logger().info(
                    f"Danger Mask Updated: {bin(new_mask)} (0x{new_mask:02X})"
                )

                value_byte = bytes([new_mask])

                characteristic = server.get_characteristic(CHARACTERISTIC_UUID)
                if characteristic:
                    characteristic.value = value_byte

                    #notify connected clients
                    server.update_value(SERVICE_UUID, CHARACTERISTIC_UUID)
                else:
                    node.get_logger().error("Failed to get characteristic")
            await asyncio.sleep(0.1)

    except asyncio.CancelledError:
        node.get_logger().info("Async task cancelled")
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received")
    except Exception as e:
        node.get_logger().error(f"Error in main loop: {e}")
    finally:
        node.get_logger().info("Cleaning up...")
        try:
            await server.stop()
        except Exception as e:
            node.get_logger().error(f"Error during cleanup: {e}")
            if "Event loop is closed" not in str(e):
                node.get_logger().error(f"Error during BLE cleanup: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DangerBroadcaster()

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    main_task = None

    try:
        main_task = loop.create_task(run(node, loop))
        loop.run_until_complete(main_task)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested")
        if main_task and not main_task.done():
            main_task.cancel()
            try:
                loop.run_until_complete(main_task)
            except asyncio.CancelledError:
                pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Clean up ROS
        try:
            if not node._handle.pointer == 0:  # Check if node is still valid
                node.destroy_node()
        except Exception:
            pass
        
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        
        # Close event loop last
        try:
            loop.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()