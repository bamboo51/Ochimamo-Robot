import asyncio
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from bleak import BleakScanner, BleakClient, BleakError

# --- CONFIGURATION ---
SERVICE_UUID = "0000abcd-0000-1000-8000-00805f9b34fb".lower()
CHARACTERISTIC_UUID = "00001234-0000-1000-8000-00805f9b34fb".lower()
DANGER_DISTANCE = 0.55

ID_TO_MAC_MAP = {
    "1": "AA:BB:CC:DD:EE:FF",
    "2": "11:22:33:44:55:66",
}

class DangerMonitorNode(Node):
    def __init__(self):
        super().__init__('ble_warn')
        self.subscription = self.create_subscription(
            String, '/people/wall_distance', self.distance_callback, 10
        )
        
        # Store the latest danger status per Track ID
        # Format: {'1': 0.5, '2': 2.5}
        self.latest_distances = {}
        self.get_logger().info("Multi-Device Monitor Started.")

    def distance_callback(self, msg):
        try:
            if ':' in msg.data:
                parts = msg.data.split(':')
                track_id = parts[0]
                dist = float(parts[1])
                
                # Update the shared dictionary
                self.latest_distances[track_id] = dist
        except Exception:
            pass

async def scan_for_devices():
    """Scans for 5 seconds and returns a LIST of valid devices."""
    print("Scanning for 5 seconds...")
    found_devices = []
    devices = await BleakScanner.discover(timeout=5.0, return_adv=True)
    
    for address, (device, adv_data) in devices.items():
        if adv_data.service_uuids:
            uuids = [str(u).lower() for u in adv_data.service_uuids]
            if SERVICE_UUID in uuids:
                print(f"found: {device.name} [{address}]")
                found_devices.append(device)
    
    print(f"Scan complete. Found {len(found_devices)} devices.")
    return found_devices

async def client_task(node, device):
    """
    Manages the connection for A SINGLE device.
    We will spawn one of these tasks for every M5Stack we find.
    """
    address = device.address
    print(f"Task started for {address}")

    async with BleakClient(device,timeout=10.0) as client:
        print(f"Connected to {address}")
        
        while rclpy.ok():
            if not client.is_connected:
                print(f"{address} disconnected.")
                break

            # 1. Determine which Track ID owns this M5Stack
            # (Reverse lookup: Find ID where MAC matches this client)
            owner_id = None
            for tid, mac in ID_TO_MAC_MAP.items():
                if mac.lower() == address.lower():
                    owner_id = tid
                    break
            
            should_alarm = False
            
            # Logic: If we know the owner, check ONLY their distance.
            #    If we don't know the owner (unmapped), check ALL distances (Broadcast mode).
            if owner_id:
                # Targeted Mode
                dist = node.latest_distances.get(owner_id)
                if dist is not None and dist < DANGER_DISTANCE:
                    print(f"DANGER for Person {owner_id} ({dist}m) -> Alerting {address}")
                    should_alarm = True
            else:
                # Broadcast Mode (Panic): If ANYONE is in danger, alert unmapped devices
                # (You can disable this else block if you want strict mapping only)
                for dist in node.latest_distances.values():
                    if dist < DANGER_DISTANCE:
                        should_alarm = True
                        break

            # 3. Send Signal
            try:
                if should_alarm:
                    await client.write_gatt_char(CHARACTERISTIC_UUID, b'1', response=True)
                else:
                    await client.write_gatt_char(CHARACTERISTIC_UUID, b'0', response=True)
            except Exception as e:
                print(f"Write failed for {address}: {e}")
                break

            await asyncio.sleep(0.2)
    
    print(f"Task ended for {address}")

async def ros_spin_loop(node):
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0)
        await asyncio.sleep(0.01)

async def async_main():
    rclpy.init()
    node = DangerMonitorNode()

    devices = await scan_for_devices()
    
    if not devices:
        print("No M5Stacks found. Exiting.")
        return

    # 2. Create a task for ROS, and a task for EACH device
    tasks = [ros_spin_loop(node)]
    
    for d in devices:
        tasks.append(client_task(node, d))

    try:
        await asyncio.gather(*tasks)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

def main(args=None):
    try:
        asyncio.run(async_main())
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()