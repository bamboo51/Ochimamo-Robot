import asyncio
import threading
import logging
import time
from typing import Optional, Dict, Tuple, Any

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String

from dbus_next.aio.message_bus import MessageBus
from dbus_next.service import ServiceInterface, method, dbus_property
from dbus_next.signature import Variant
from dbus_next.constants import BusType, PropertyAccess

# ============================================================
# CONFIG
# ============================================================
# This defines the "shape" of the beacon
LE_ADVERTISEMENT_IFACE = 'org.bluez.LEAdvertisement1'
DBUS_OM_IFACE = 'org.freedesktop.DBus.ObjectManager'
DBUS_PROP_IFACE = 'org.freedesktop.DBus.Properties'

BLUEZ_SERVICE_NAME = 'org.bluez'
ADVERTISING_MANAGER_IFACE = 'org.bluez.LEAdvertisingManager1'

# Your Beacon Settings
DANGER_DISTANCE = 1.5
STALE_DATA_TIMEOUT = 2.0
MAX_TRACK_IDS = 8
MANUFACTURER_ID = 0xCCCC  # Test ID. Use 0x004C (Apple) or others for specific parsing.


# ============================================================
# DBUS ADVERTISEMENT CLASS
# ============================================================
class DangerAdvertisement(ServiceInterface):
    def __init__(self, index):
        super().__init__(LE_ADVERTISEMENT_IFACE)
        self.index = index
        self._type = 'broadcast'  # 'broadcast' is non-connectable (perfect for beacons)
        self._local_name = 'DANGER'
        self._manufacturer_data = {MANUFACTURER_ID: bytes([0x00])}
        self.path = f'/org/bluez/example/advertisement{index}'

    def update_mask(self, mask: int):
        # Update internal state
        new_data = {MANUFACTURER_ID: bytes([mask & 0xFF])}
        print(f">>> BLE UPDATE: ID=0x{MANUFACTURER_ID:04X} Data=[0x{mask:02X}]")

        self._manufacturer_data = new_data
        
        # Signal DBus that properties changed so BlueZ updates the packet
        self.emit_properties_changed({
            'ManufacturerData': {
                k: Variant('ay', v) for k, v in self._manufacturer_data.items()
            }
        })

    @method()
    def Release(self):
        logging.info(f'{self.path}: Released!')

    @dbus_property(access=PropertyAccess.READ)
    def Type(self) -> 's':
        return self._type

    @dbus_property(access=PropertyAccess.READ)
    def LocalName(self) -> 's':
        return self._local_name

    @dbus_property(access=PropertyAccess.READ)
    def ManufacturerData(self) -> 'a{qv}':
        # Transforms python dict to DBus Variant format
        return {
            k: Variant('ay', v) for k, v in self._manufacturer_data.items()
        }


# ============================================================
# BLE THREAD
# ============================================================
class BLEThread(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self.ready = threading.Event()
        self.stop_event = threading.Event()
        self.advertisement: Optional[DangerAdvertisement] = None
        self.bus: Optional[MessageBus] = None
        self.manager = None
        self.adapter_path = '/org/bluez/hci0'

    def run(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        try:
            self.loop.run_until_complete(self._run())
        except Exception as e:
            logging.error(f"BLE thread crashed: {e}")
        finally:
            self.loop.run_until_complete(self._cleanup())
            self.loop.close()

    async def _run(self):
        # 1. Connect to System DBus 
        self.bus = await MessageBus(bus_type=BusType.SYSTEM).connect()

        # 2. Find BlueZ Adapter (usually hci0)
        # We assume hci0 is at /org/bluez/hci0. 
        # A robust implementation might query ObjectManager, but this is standard.
        
        # 3. Setup Advertisement Object
        self.advertisement = DangerAdvertisement(0)
        self.bus.export(self.advertisement.path, self.advertisement)

        # 4. Register with BlueZ Advertising Manager
        intr = await self.bus.introspect(BLUEZ_SERVICE_NAME, self.adapter_path)
        proxy = self.bus.get_proxy_object(BLUEZ_SERVICE_NAME, self.adapter_path, intr)
        self.manager = proxy.get_interface(ADVERTISING_MANAGER_IFACE)

        logging.info("Registering Advertisement...")
        await self.manager.call_register_advertisement(
            self.advertisement.path,
            {}
        )
        
        logging.info("BLE Beacon Active.")
        self.ready.set()

        # Keep alive
        while not self.stop_event.is_set():
            await asyncio.sleep(0.5)

    async def _cleanup(self):
        # Unregister advertisement gracefully
        if self.manager and self.advertisement:
            try:
                await self.manager.call_unregister_advertisement(self.advertisement.path)
                logging.info("Advertisement unregistered")
            except Exception as e:
                logging.warning(f"Failed to unregister advertisement: {e}")
        
        if self.bus:
            self.bus.disconnect()

    def update_mask(self, mask: int):
        if self.loop and self.loop.is_running() and self.advertisement:
            # Thread-safe property update
            self.loop.call_soon_threadsafe(self.advertisement.update_mask, mask)

    def stop(self):
        self.stop_event.set()


# ============================================================
# ROS NODE (Standard)
# ============================================================
class DangerBroadcaster(Node):
    def __init__(self, ble: BLEThread):
        super().__init__("ble_warn")
        self.ble = ble
        self.lock = threading.Lock()
        self.latest: Dict[int, Tuple[float, float]] = {}
        self.last_mask = 0
        self.last_update_time = time.time()

        self.create_subscription(String, "/people/wall_distance", self.distance_callback, 10)
        self.create_timer(0.1, self.update_ble)
        self.get_logger().info("ROS2 BLE Broadcaster running")

    def distance_callback(self, msg: String):
        try:
            tid_s, dist_s = msg.data.split(":")
            tid = int(tid_s)
            dist = float(dist_s)
            if 1 <= tid <= MAX_TRACK_IDS:
                with self.lock:
                    self.latest[tid] = (dist, time.monotonic())
        except Exception as e:
            self.get_logger().debug(f"Failed to parse distance message: {e}")

    def compute_mask(self) -> int:
        now = time.monotonic()
        mask = 0
        with self.lock:
            expired = []
            for tid, (dist, ts) in self.latest.items():
                if now - ts > STALE_DATA_TIMEOUT:
                    expired.append(tid)
                elif dist < DANGER_DISTANCE:
                    mask |= (1 << (tid - 1))
            for tid in expired:
                del self.latest[tid]
        return mask & 0xFF

    def update_ble(self):
        mask = self.compute_mask()
        now = time.time()
        
        # Send if: mask changed OR 1 second passed (heartbeat)
        time_since_last = now - self.last_update_time
        mask_changed = (mask != self.last_mask)
        need_heartbeat = (time_since_last >= 1.0)
        
        if mask_changed or need_heartbeat:
            # Update BLE
            self.ble.update_mask(mask)
            self.last_update_time = now
            
            # Only log if mask actually changed (not for heartbeats)
            if mask_changed:
                if mask:
                    self.get_logger().warn(f"DANGER mask 0x{mask:02X}")
                else:
                    self.get_logger().info("Zone clear")
                self.last_mask = mask


# ============================================================
# MAIN
# ============================================================
def main():
    logging.basicConfig(level=logging.INFO)
    
    # 1. Start BLE Thread
    ble = BLEThread()
    ble.start()

    if not ble.ready.wait(timeout=10):
        logging.error("BLE Failed to start (Is bluetoothd running?)")
        # Don't return immediately, let the cleanup happen in finally
    
    # 2. Start ROS Node
    rclpy.init()
    node = DangerBroadcaster(ble)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the ROS executor (stops callbacks)
        executor.shutdown()
        node.destroy_node()
        
        # --- FIXED SECTION ---
        # Only shutdown ROS if it hasn't been shut down by the Ctrl+C handler yet
        if rclpy.ok():
            rclpy.shutdown()
        # ---------------------

        # Stop the BLE thread
        ble.stop()
        ble.join(timeout=2)

if __name__ == "__main__":
    main()
