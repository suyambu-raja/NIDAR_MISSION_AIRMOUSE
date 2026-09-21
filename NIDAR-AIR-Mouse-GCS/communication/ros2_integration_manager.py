"""
Master ROS 2 Integration Manager for NIDAR AirMouse GCS.
Coordinates the subscriber bridge, video bridge, and command bridge.
Automatically detects whether ROS 2 (rclpy) is available, runs bridges in background
threads when active, and gracefully falls back to simulation mode without crashing.
"""
import asyncio
import threading
from typing import Optional, Any

try:
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

from communication.ros2_bridge import ROS2BridgeNode
from communication.video_bridge import VideoBridgeNode
from communication.command_bridge import CommandBridgeNode


class ROS2IntegrationManager:
    """
    Master coordinator for live ROS 2 backend integration.
    """

    def __init__(self, message_queue, server=None, grid_mgr=None):
        self.message_queue = message_queue
        self.server = server
        self.grid_mgr = grid_mgr or (getattr(server, "grid_mgr", None) if server else None)

        self.ros2_bridge: Optional[ROS2BridgeNode] = None
        self.video_bridge: Optional[VideoBridgeNode] = None
        self.command_bridge: Optional[CommandBridgeNode] = None

        self._executor = None
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self.is_ros2_active = False

        # Hook into server's command handler if server is provided
        if self.server and hasattr(self.server, "_handle_client_command"):
            self._hook_server_commands()

    def _hook_server_commands(self):
        """Intercepts client commands sent to server and forwards them to CommandBridge."""
        original_handler = self.server._handle_client_command

        async def intercepted_handle_client_command(cmd_data: dict):
            if self.command_bridge:
                try:
                    self.command_bridge.handle_command(cmd_data)
                except Exception as e:
                    print(f"[ROS2IntegrationManager] Command bridge error: {e}")
            return await original_handler(cmd_data)

        self.server._handle_client_command = intercepted_handle_client_command

    def start(self):
        """Starts ROS 2 bridge nodes in a dedicated daemon background thread."""
        if self._running:
            return
        self._running = True

        if not ROS2_AVAILABLE:
            print("[ROS2IntegrationManager] Notice: rclpy not installed in environment.")
            print("[ROS2IntegrationManager] Seamlessly using built-in SIMULATION fallback engine.")
            self.command_bridge = CommandBridgeNode()
            return

        try:
            if not rclpy.ok():
                rclpy.init()

            loop = None
            try:
                loop = asyncio.get_running_loop()
            except RuntimeError:
                pass

            self.ros2_bridge = ROS2BridgeNode(
                message_queue=self.message_queue,
                loop=loop,
                grid_mgr=self.grid_mgr
            )
            self.video_bridge = VideoBridgeNode(
                message_queue=self.message_queue,
                loop=loop
            )
            self.command_bridge = CommandBridgeNode()

            self._executor = MultiThreadedExecutor(num_threads=4)
            self._executor.add_node(self.ros2_bridge)
            self._executor.add_node(self.video_bridge)
            self._executor.add_node(self.command_bridge)

            self._thread = threading.Thread(target=self._worker, daemon=True, name="ROS2BridgeExecutor")
            self._thread.start()
            self.is_ros2_active = True

            # If connected to live ROS 2, switch GCS mode flag
            if self.server:
                self.server.slam_mode = "REALTIME"

            print("[ROS2IntegrationManager] Live ROS 2 bridges successfully started and spinning!")

        except Exception as e:
            print(f"[ROS2IntegrationManager] Failed to start ROS 2 bridges ({e}). Falling back to simulation.")
            self.is_ros2_active = False

    def _worker(self):
        """Executor worker thread."""
        try:
            if self._executor:
                self._executor.spin()
        except Exception:
            pass

    def stop(self):
        """Gracefully shuts down ROS 2 nodes and executor thread."""
        self._running = False
        if not self.is_ros2_active:
            return

        print("[ROS2IntegrationManager] Shutting down ROS 2 bridges...")
        try:
            if self._executor:
                self._executor.shutdown()
            if self.ros2_bridge:
                self.ros2_bridge.destroy_node()
            if self.video_bridge:
                self.video_bridge.destroy_node()
            if self.command_bridge:
                self.command_bridge.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
            self._thread = None

        self.is_ros2_active = False
        print("[ROS2IntegrationManager] ROS 2 bridges stopped.")
