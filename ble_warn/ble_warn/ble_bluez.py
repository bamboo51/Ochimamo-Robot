import asyncio
import threading
import logging
import subprocess
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String

from bluez_peripheral.gatt.service import Service
from bluez_peripheral.gatt.characteristic import characteristic, CharacteristicFlags as CharFlags
from bluez_peripheral.advert import Advertisement
from bluez_peripheral.util import Adapter
from dbus_next.aio.message_bus import MessageBus
from dbus_next.constants import BusType

"""
Configs
"""
SERVICE_UUID = "0000abcd-0000-1000-8000-00805f9b34fb"
CHAR_UUID    = "00001234-0000-1000-8000-00805f9b34fb"

DANGER_DISTANCE     = 1.2
STALE_DATA_TIMEOUT  = 2.0
MAX_TRACK_IDS       = 8

BLE_NAME = "DANGER_BEACON"

# BLE Service
class SafetyService(Service):
    def __init__(self):
        super().__init__(SERVICE_UUID, primary=True)
        self._mask = 0x00
        self._lock = threading.Lock()

    @characteristic(CHAR_UUID, CharFlags.NOTIFY | CharFlags.READ)
    def danger_characteristic(self, options):
        with self._lock:
            return bytes([self._mask])

    def set_mask(self, mask: int):
        """Set mask and notify if changed"""
        with self._lock:
            if mask == self._mask:
                return False  # No change
            self._mask = mask
            try:
                self.danger_characteristic.changed(bytes([mask]))
                return True  # Changed
            except Exception as e:
                logging.error(f"Failed to notify characteristic change: {e}")
                return False
    
    def force_notify(self):
        """Force send notification with current mask value"""
        with self._lock:
            try:
                self.danger_characteristic.changed(bytes([self._mask]))
                return True
            except Exception as e:
                logging.error(f"Failed to force notify: {e}")
                return False

# BLE Thread
class BLEThread(threading.Thread):
    def __init__(self):
        super().__init__(daemon=False)
        self.loop = None
        self.service = SafetyService()
        self.ready = threading.Event()
        self.stop_event = threading.Event()
        self.advert = None
        self.bus = None
        self.adapter = None

    def run(self):
        # Force Bluetooth adapter ON
        try:
            logging.info("Ensuring Bluetooth adapter is powered on...")
            subprocess.run(["bluetoothctl", "power", "on"], check=False)
        except Exception as e:
            logging.warning(f"Could not run bluetoothctl: {e}")

        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        try:
            self.loop.run_until_complete(self._run())
        except Exception as e:
            logging.error(f"BLE thread crashed: {e}")
        finally:
            self.loop.close()

    async def _run(self):
        try:
            self.bus = await MessageBus(bus_type=BusType.SYSTEM).connect()
            self.adapter = await Adapter.get_first(self.bus)
            if self.adapter is None:
                raise RuntimeError("No Bluetooth adapter found")
            
            logging.info(f"Using adapter: {self.adapter}")
            
            # Register service directly to the bus
            await self.service.register(self.bus)
            
            # Create and register advertisement
            self.advert = Advertisement(
                localName=BLE_NAME,
                serviceUUIDs=[SERVICE_UUID],
                appearance=0,
                timeout=0
            )
            await self.advert.register(self.bus, self.adapter)

            logging.info(f"BLE GATT server advertising as '{BLE_NAME}'")
            self.ready.set()

            # Wait for stop signal
            while not self.stop_event.is_set():
                await asyncio.sleep(0.5)

        except Exception as e:
            logging.error(f"BLE setup failed: {e}")
            self.ready.set()
            raise
        finally:
            await self._cleanup()

    async def _cleanup(self):
        """Properly cleanup BLE resources"""
        try:
            if self.service:
                await self.service.unregister()
            # Advertisement cleanup is handled automatically
            logging.info("BLE resources cleaned up")
        except Exception as e:
            logging.error(f"Error during BLE cleanup: {e}")

    def update_mask(self, mask: int):
        """Thread-safe method to update the BLE characteristic"""
        if self.loop is None or not self.loop.is_running():
            logging.warning("BLE loop not ready, skipping mask update")
            return
        
        try:
            future = asyncio.run_coroutine_threadsafe(
                self._update(mask),
                self.loop
            )
            future.result(timeout=0.1)
        except asyncio.TimeoutError:
            logging.warning("BLE update timed out")
        except Exception as e:
            logging.error(f"Failed to update BLE mask: {e}")

    def force_notify(self):
        """Force send notification with current mask"""
        if self.loop is None or not self.loop.is_running():
            return
        
        try:
            future = asyncio.run_coroutine_threadsafe(
                self._force_notify(),
                self.loop
            )
            future.result(timeout=0.1)
        except Exception as e:
            logging.error(f"Failed to force notify: {e}")

    async def _update(self, mask: int):
        self.service.set_mask(mask)

    async def _force_notify(self):
        self.service.force_notify()

    def stop(self):
        """Signal the thread to stop"""
        self.stop_event.set()
        if self.loop and self.loop.is_running():
            self.loop.call_soon_threadsafe(self.stop_event.set)

# ROS Node
class DangerBroadcaster(Node):
    def __init__(self, ble_thread: BLEThread):
        super().__init__("ble_warn")
        self.ble = ble_thread
        self.lock = threading.Lock()
        self.latest = {}
        self.last_mask = 0x00
        self.danger_active = False

        self.create_subscription(
            String,
            "/people/wall_distance",
            self.distance_callback,
            10
        )

        # Update BLE every 200ms
        self.create_timer(0.2, self.update_ble)
        self.get_logger().info("ROS2 BLE Broadcaster Started")

    def distance_callback(self, msg):
        try:
            parts = msg.data.split(":")
            if len(parts) != 2:
                self.get_logger().warning(f"Invalid message format: {msg.data}")
                return
            
            tid = int(parts[0])
            dist = float(parts[1])

            if tid <= 0 or tid > MAX_TRACK_IDS:
                self.get_logger().warning(f"Track ID {tid} out of range")
                return
            
            with self.lock:
                self.latest[tid] = (dist, self.get_clock().now())
        except ValueError as e:
            self.get_logger().warning(f"Failed to parse message '{msg.data}': {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in distance_callback: {e}")

    def compute_mask(self):
        mask = 0
        now = self.get_clock().now()

        with self.lock:
            stale = []
            for tid, (dist, ts) in self.latest.items():
                age = (now.nanoseconds - ts.nanoseconds) / 1e9
                if age > STALE_DATA_TIMEOUT:
                    stale.append(tid)
                elif dist < DANGER_DISTANCE:
                    mask |= (1 << (tid - 1))
            
            for tid in stale:
                del self.latest[tid]

        return mask & 0xFF
    
    def update_ble(self):
        try:
            current_mask = self.compute_mask()
            
            # Check if there's any danger
            has_danger = current_mask != 0
            
            if has_danger:
                # DANGER: Always send (even if mask hasn't changed)
                self.ble.update_mask(current_mask)
                if not self.danger_active:
                    self.get_logger().warn(f"DANGER DETECTED: mask=0x{current_mask:02X}")
                    self.danger_active = True
                self.last_mask = current_mask
                
            elif self.danger_active:
                # Was in danger, now safe: Send SAFE once
                self.ble.update_mask(0x00)
                self.get_logger().info("DANGER CLEARED: Sending SAFE")
                self.danger_active = False
                self.last_mask = 0x00
                
            # else: No danger and wasn't in danger - do nothing
            
        except Exception as e:
            self.get_logger().error(f"Error updating BLE: {e}")

def main():
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

    ble_thread = None
    node = None
    executor = None

    try:
        # Start BLE
        logging.info("Starting BLE thread...")
        ble_thread = BLEThread()
        ble_thread.start()

        # Wait for BLE to be ready
        if not ble_thread.ready.wait(timeout=10):
            raise RuntimeError("BLE failed to start within timeout")
        
        logging.info("BLE thread ready")

        # Start ROS
        logging.info("Initializing ROS2...")
        rclpy.init()
        node = DangerBroadcaster(ble_thread)

        executor = SingleThreadedExecutor()
        executor.add_node(node)

        logging.info("Starting ROS2 executor...")
        executor.spin()

    except KeyboardInterrupt:
        logging.info("Received keyboard interrupt, shutting down...")
    except Exception as e:
        logging.error(f"Fatal error: {e}")
    finally:
        # Cleanup in reverse order
        if executor:
            executor.shutdown()
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if ble_thread:
            logging.info("Stopping BLE thread...")
            ble_thread.stop()
            ble_thread.join(timeout=5)
            if ble_thread.is_alive():
                logging.warning("BLE thread did not stop gracefully")
        
        logging.info("Shutdown complete")

if __name__ == "__main__":
    main()
