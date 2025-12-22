import asyncio
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from bless import (
    BlessServer,
    BlessGATTCharacteristic,
    GATTCharacteristicProperties,
    GATTAttributePermissions
)
import logging

# --- CONFIGURATION ---
SERVICE_UUID = "0000abcd-0000-1000-8000-00805f9b34fb"
CHARACTERISTIC_UUID = "00001234-0000-1000-8000-00805f9b34fb"
DANGER_DISTANCE = 0.55

class DangerBroadcaster(Node):
    def __init__(self):
        super().__init__('ble_warn')
        self.subscription = self.create_subscription(
            String, '/people/wall_distance', self.distance_callback, 10
        )
        self.latest_distances = {}
        self.server = None
        self.current_mask = 0x00
        
        # Configure logging to suppress BLE spam
        logging.basicConfig(level=logging.INFO)
        self.get_logger().info("Risk Broadcaster Started.")

    def distance_callback(self, msg):
        try:
            if ':' in msg.data:
                parts = msg.data.split(':')
                track_id = int(parts[0]) # Assuming ID is integer 1, 2, 3...
                dist = float(parts[1])
                self.latest_distances[track_id] = dist
        except Exception:
            pass
            
    def compute_danger_mask(self):
        """
        Returns a single byte where each bit represents a danger flag for an ID.
        Bit 0 = ID 1, Bit 1 = ID 2, etc.
        """
        mask = 0x00
        for track_id, dist in self.latest_distances.items():
            if dist < DANGER_DISTANCE:
                # Map ID 1 -> Bit 0 (1<<0)
                # Map ID 2 -> Bit 1 (1<<1)
                if track_id > 0:
                    mask |= (1 << (track_id - 1))
        
        # Limit to 1 byte (0-255)
        return mask & 0xFF

async def run(node, loop):
    node.get_logger().info("Initializing BLE Server...")
    
    # Initialize Bless Server
    server = BlessServer(name="DANGER_BEACON", loop=loop)
    
    # Setup Service & Characteristic
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
    
    node.get_logger().info("Advertising Started!")
    await server.start_advertising(server.services)
    
    try:
        while rclpy.ok():
            # 1. ROS Spin
            rclpy.spin_once(node, timeout_sec=0)
            
            # 2. Logic Update
            new_mask = node.compute_danger_mask()
            
            if new_mask != node.current_mask:
                node.current_mask = new_mask
                node.get_logger().info(f"Broadcasting Danger Mask: {bin(new_mask)}")
                
                # Update GATT Value (Broadcasters usually update usage value)
                # Note: BLESS currently supports updating the GATT DB. 
                # Pure custom manufacturing data update in BLESS can be tricky, 
                # but updating value + notify is standard.
                value_byte = bytes([new_mask])
                
                # Update local value
                server.get_characteristic(
                    CHARACTERISTIC_UUID
                ).value = value_byte
                
                # Notify Connected Clients (if any, though we are broadcast focused)
                server.update_value(
                    SERVICE_UUID,
                    CHARACTERISTIC_UUID
                )
                
            await asyncio.sleep(0.1)
            
    except asyncio.CancelledError:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Stopping Advertising...")
        await server.stop_advertising()
        await server.stop()
        node.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init()
    node = DangerBroadcaster()
    
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    
    try:
        loop.run_until_complete(run(node, loop))
    except (KeyboardInterrupt, asyncio.CancelledError):
        pass

if __name__ == "__main__":
    main()