#!/usr/bin/env python3
import threading
import queue
import time
import rclpy
import os
from rclpy.context import Context
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from pyrobosim_msgs.msg import RobotState
from std_msgs.msg import Float32


# Configuration
PRIVATE_DOMAIN_ID = 42

# Thread-safe queue used to pass messages from private context -> public context.
message_queue : queue.Queue[RobotState] = queue.Queue(maxsize=100)

# ---------------------------
# Node running in private domain (talks to Node A)
# ---------------------------
class BridgePrivateNode(Node):
    def __init__(self, context: Context):
        super().__init__('bridge_private', namespace='', context=context)

        # Subscribe to private topic exposed by Node A
        # (Example topic name: "/internal/data")
        self.sub = self.create_subscription(RobotState, '/robot/robot_state', self._on_private_msg, 10)

        # # Create a client to call private service of Node A
        # self.srv_client = self.create_client(Trigger, '/internal/do_something')

        # # Wait for service (non-blocking: you may add timeout logic)
        # if not self.srv_client.wait_for_service(timeout_sec=5.0):
        #     self.get_logger().warn('Private service /internal/do_something not available (yet).')

    def _on_private_msg(self, msg: RobotState):
        # Called in private executor thread. Decide what subset to expose publicly.

        self.get_logger().debug(f"Received private msg: {msg}")

        try:
            message_queue.put_nowait(msg)
        except queue.Full:
            self.get_logger().warn("Message queue full — dropping private->public message")
    
    # def call_private_service(self, wait_timeout=5.0):
    #     """Synchronous wrapper to call the private service in this context. Returns bool, message."""
    #     req = Trigger.Request()
    #     future = self.srv_client.call_async(req)
    #     # spin until future complete inside *this* context
    #     rclpy.spin_until_future_complete(self, future, timeout_sec=wait_timeout)
    #     if future.done():
    #         res = future.result()
    #         success = getattr(res, 'success', False)
    #         message = getattr(res, 'message', '')
    #         return success, message
    #     else:
    #         return False, 'timeout or no response from private service'


# ---------------------------
# Node running in public domain (exposes a public API)
# ---------------------------
class BridgePublicNode(Node):
    def __init__(self, context: Context, private_node: BridgePrivateNode, stop_event: threading.Event):
        # Important: pass the public context when creating this node (done from caller)
        super().__init__('bridge_public', namespace='', context=context)

        self.private_node = private_node  # reference to the private node object in other context
        self.stop_event = stop_event

        # Public publisher that external world can subscribe to
        self.pub = self.create_publisher(Float32, '/robot/battery_level', 10)

        self.consumer_thread = threading.Thread(target=self._consumer_loop, daemon=True)
        self.consumer_thread.start()

    def _consumer_loop(self):
        """Publish any queued messages that private node enqueued."""
        while not self.stop_event.is_set():

            # Robot status message from private node
            try:
                original_robot_status = message_queue.get_nowait()
                msg = Float32()
                msg.data = original_robot_status.battery_level
                self.pub.publish(msg)
            except queue.Empty:
                pass
            
            time.sleep(0.05)  # avoid busy loop

    def graceful_shutdown(self):
        self.get_logger().info("Shutting down consumer thread...")
        # If you had a more complex thread, you'd signal it to stop here.
        self.consumer_thread.join(timeout=2.0)
        self.get_logger().info("Consumer thread stopped.")

    # def handle_public_service(self, request, response):
    #     # A public client called /public/do_something.
    #     # Forward the call to the private node's client synchronously.
    #     # Note: We must call the private node's helper in the *private context*.
    #     # We have a reference to the private node object (in another context).
    #     self.get_logger().info("Received public service request; forwarding to private service...")

    #     # Use the private node's synchronous helper that internally spins its context
    #     success, message = self.private_node.call_private_service(wait_timeout=5.0)

    #     response.success = success
    #     response.message = f"Bridge result: {message}"
    #     return response


# ---------------------------
# Helpers to init contexts, nodes, executors, threads
# ---------------------------
def init_contexts_and_nodes(stop_event: threading.Event):
    # Create private context and init
    private_ctx = Context()
    rclpy.init(context=private_ctx, args=None, domain_id=PRIVATE_DOMAIN_ID)
    # Create private node with its context
    private_node = BridgePrivateNode(context=private_ctx)

    # Create public context and init
    public_ctx = Context()
    rclpy.init(context=public_ctx, args=None)
    # Create public node in public context
    public_node = BridgePublicNode(context=public_ctx, private_node=private_node, stop_event=stop_event)

    # Create executors for each context
    private_exec = SingleThreadedExecutor(context=private_ctx)
    public_exec = SingleThreadedExecutor(context=public_ctx)

    private_exec.add_node(private_node)
    public_exec.add_node(public_node)

    return {
        'private': {'ctx': private_ctx, 'node': private_node, 'exec': private_exec},
        'public': {'ctx': public_ctx, 'node': public_node, 'exec': public_exec},
    }


def spin_executor_in_thread(exec_obj, stop_event):
    # Spin until stop_event is set
    try:
        while not stop_event.is_set():
            exec_obj.spin_once(timeout_sec=0.1)
    except Exception as e:
        # Log or handle as needed
        print(f"Executor thread exception: {e}")


def main():
    # NOTE: before running this whole process, make sure environment variables
    # for domains are set as desired. Another approach is to set domain ids
    # at rclpy.init call time via context options if you prefer.
    #
    # For example, export ROS_DOMAIN_ID in your shell before running:
    #   export ROS_DOMAIN_ID=42  # for private node A/bridge private if launching separately
    #
    # Here we assume the domains are controlled externally; rclpy.Contexts tie to the current ROS_DOMAIN_ID env.

    # Get ROS_DOMAIN_ID from environment or default to 0
    ros_domain_id = int(os.environ.get('ROS_DOMAIN_ID', '0'))
    if ros_domain_id == PRIVATE_DOMAIN_ID:
        print(f"Warning: ROS_DOMAIN_ID={ros_domain_id} is the same as PRIVATE_DOMAIN_ID={PRIVATE_DOMAIN_ID}. This may cause conflicts. Exiting already")
        exit(1)

    # Run executors in background threads
    stop_event = threading.Event()

    # Build contexts/nodes/executors
    parts = init_contexts_and_nodes(stop_event)
    private_node = parts['private']['node']
    public_node = parts['public']['node']
    private_exec = parts['private']['exec']
    public_exec = parts['public']['exec']

    private_thread = threading.Thread(target=spin_executor_in_thread, args=(private_exec, stop_event), daemon=True)
    public_thread = threading.Thread(target=spin_executor_in_thread, args=(public_exec, stop_event), daemon=True)

    private_thread.start()
    public_thread.start()

    try:
        print("Bridge running. Press Ctrl-C to exit.")
        while True:
            time.sleep(0.5)  # main thread can watch health, metrics, or react to signals
    except KeyboardInterrupt:
        print("Shutting down bridge...")
    finally:
        stop_event.set()
        public_node.graceful_shutdown()

        # Wait for threads to finish
        private_thread.join(timeout=2.0)
        public_thread.join(timeout=2.0)

        # Destroy nodes and shutdown contexts
        parts['public']['exec'].remove_node(parts['public']['node'])
        parts['private']['exec'].remove_node(parts['private']['node'])
        parts['public']['node'].destroy_node()
        parts['private']['node'].destroy_node()

        # Shutdown rclpy for both contexts
        rclpy.shutdown(context=parts['public']['ctx'])
        rclpy.shutdown(context=parts['private']['ctx'])
        print("Shutdown complete.")


if __name__ == '__main__':
    main()