#!/usr/bin/env python3
import re
import threading
import queue
import time
import rclpy
import os
from rclpy.context import Context
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.action import ActionClient, ActionServer
from rclpy.action import CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle, GoalStatus

from pyrobosim_msgs.msg import RobotState, TaskAction
from simple_pyrobosim_msgs.msg import SimpleRobotState

from pyrobosim_msgs.srv import RequestWorldState
from simple_pyrobosim_msgs.srv import IsDoorOpen

from pyrobosim_msgs.msg import ExecutionResult
from pyrobosim_msgs.action import ExecuteTaskAction
from simple_pyrobosim_msgs.action import Navigate, Pick, Place, ApproachObject, ApproachDoor, Open
from rclpy.action.client import ClientGoalHandle

# Configuration
PRIVATE_DOMAIN_ID = 42

# Thread-safe queue used to pass messages from private context -> public context.
message_queue : queue.Queue[RobotState] = queue.Queue(maxsize=100)

# ---------------------------
# Node running in private domain (talks to Node PyRobosim)
# ---------------------------
class PyRobosimBridgePrivate(Node):
    def __init__(self, context: Context):
        super().__init__('pyrobosim_bridge_private', namespace='', context=context)

        # Subscribe to private topic exposed by pyrobosim node
        self._robot_status_sub = self.create_subscription(RobotState, '/robot/robot_state', self._on_private_robot_status_msg, 10)

        # Create a client to call private service of pyrobosim node
        self._world_state_srv_client = self.create_client(RequestWorldState, '/request_world_state')

        # Create action client for executing tasks
        self._exec_action_client = ActionClient(self, ExecuteTaskAction, '/execute_action')

        # Wait for service (non-blocking: you may add timeout logic)
        if not self._world_state_srv_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Private service /request_world_state not available (yet).')
        
        # Wait for action server
        if not self._exec_action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Action server /execute_action not available (yet).')

    def _on_private_robot_status_msg(self, msg: RobotState):
        # Called in private executor thread. Decide what subset to expose publicly.
        self.get_logger().debug(f"Received robot status msg: {msg}")
        try:
            message_queue.put_nowait(msg)
        except queue.Full:
            self.get_logger().warn("Message queue full — dropping private->public message")
    
    def request_world_status(self, wait_timeout=2.0) -> tuple[RequestWorldState.Response | None, str]:
        """Synchronous wrapper to call the private service in this context. Returns res | None, message."""

        if not self._world_state_srv_client.service_is_ready():
            if not self._world_state_srv_client.wait_for_service(timeout_sec=2.0):
                return None, 'Internal server error'

        req = RequestWorldState.Request()
        future = self._world_state_srv_client.call_async(req)

        # spin until future complete inside *this* context
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < wait_timeout:
            time.sleep(0.01)  # Small sleep to avoid busy waiting
        
        if future.done():
            res = future.result()
            return res, 'ok'
        else:
            return None, 'timeout or no response from private service'
    
    def execute_task_async(self, task_action):
        """Send a task goal to the action server asynchronously."""

        if not self._exec_action_client.server_is_ready():
            # Wait for action server
            if not self._exec_action_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().warn('Action server /execute_action not available (yet).')
            else:
                return None

        if not self._exec_action_client.server_is_ready():
            self.get_logger().error('Action server /execute_action is not ready')
            return None
        
        goal_msg = ExecuteTaskAction.Goal()
        goal_msg.action = task_action  # task_action should be a TaskAction message
        
        self.get_logger().info(f'Sending goal: {goal_msg}')
        future = self._exec_action_client.send_goal_async(goal_msg)
        return future
    
    def execute_task_sync(self, task_action, cancel_event: threading.Event, timeout_sec=2.0):
        """Send a task goal and wait for completion synchronously."""
        
        goal_future = self.execute_task_async(task_action)
        if goal_future is None:
            return False, 'Action server not ready'
        
        # Wait for goal acceptance
        start_time = time.time()
        while not goal_future.done() and (time.time() - start_time) < timeout_sec:
            time.sleep(0.01)
        
        if not goal_future.done():
            return False, 'Goal acceptance timeout'
        
        goal_handle: ClientGoalHandle = goal_future.result()
        if not goal_handle.accepted:
            return False, 'Goal was rejected'
        
        self.get_logger().info('Goal accepted, waiting for result...')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        start_time = time.time()
        while not result_future.done() and not cancel_event.is_set():
            time.sleep(0.01)
        
        if result_future.done():
            result_response = result_future.result()
            status = result_response.status
            actual_result: ExecuteTaskAction.Result = result_response.result
            success = status == GoalStatus.STATUS_SUCCEEDED and actual_result.execution_result.status == ExecutionResult.SUCCESS
            return success, actual_result.execution_result.message
        elif cancel_event.is_set():
            goal_handle.cancel_goal_async()
            return False, 'Cancelled' 
        else:
            return False, 'Result timeout'

def valid_room(hallways, room_name):
    for hallway in hallways:
        if hallway.room_start == room_name or hallway.room_end == room_name:
            return True
    return False
    
# ---------------------------
# Node running in public domain (exposes a public API)
# ---------------------------
class PyRobosimBridgePublic(Node):
    def __init__(self, context: Context, private_node: PyRobosimBridgePrivate, stop_event: threading.Event):
        # Important: pass the public context when creating this node (done from caller)
        super().__init__('pyrobosim_bridge_public', namespace='', context=context)

        self._private_node = private_node  # reference to the private node object in other context
        self._stop_event = stop_event
        self._cancel_event = threading.Event()

        self._curr_robot_status: None | RobotState = None
        self._curr_room = ''

        # Public publisher that external world can subscribe to
        self._simple_robot_state_pub = self.create_publisher(SimpleRobotState, '/robot/robot_state', 10)

        # Public service server that external world can call
        self._is_door_open_srv_server = self.create_service(IsDoorOpen, '/is_door_open', self._handle_is_door_open_service)

        self._action_server_navigate = ActionServer(self, Navigate, '/robot/navigate', execute_callback=self._execute_navigate_callback, cancel_callback=self._cancel_callback)
        self._action_server_approach_obj = ActionServer(self, ApproachObject, '/robot/approach_object', execute_callback=self._execute_approach_obj_callback, cancel_callback=self._cancel_callback)
        self._action_server_approach_hall = ActionServer(self, ApproachDoor, '/robot/approach_door', execute_callback=self._execute_approach_door_callback, cancel_callback=self._cancel_callback)

        self._action_server_pick = ActionServer(self, Pick, '/robot/pick', execute_callback=self._execute_pick_callback, cancel_callback=self._cancel_callback)
        self._action_server_place = ActionServer(self, Place, '/robot/place', execute_callback=self._execute_place_callback, cancel_callback=self._cancel_callback)
        self._action_server_open = ActionServer(self, Open, '/robot/open', execute_callback=self._execute_open_callback, cancel_callback=self._cancel_callback)

        self._consumer_thread = threading.Thread(target=self._consumer_loop, daemon=True)
        self._consumer_thread.start()
        

    def _consumer_loop(self):
        """Publish any queued messages that private node enqueued."""
        # Use the private node's synchronous helper that internally spins its context
        rooms = set()
        while not self._stop_event.is_set():
            if len(rooms) == 0:
                curr_world, message = self._private_node.request_world_status(wait_timeout=2.0)
                if curr_world is not None:
                    for hallway in curr_world.state.hallways:
                        rooms.add(hallway.room_start)
                        rooms.add(hallway.room_end)
            
            # Robot status message from private node
            try:
                self._curr_robot_status = message_queue.get_nowait()
                msg = SimpleRobotState()
                msg.battery_level = self._curr_robot_status.battery_level
                msg.executing_action = self._curr_robot_status.executing_action
                msg.holding_object = self._curr_robot_status.holding_object
                msg.manipulated_object = self._curr_robot_status.manipulated_object
                msg.last_visited_location = self._curr_robot_status.last_visited_location
                if self._curr_robot_status.last_visited_location in rooms:
                    self._curr_room = self._curr_robot_status.last_visited_location
                
                self._simple_robot_state_pub.publish(msg)
            except queue.Empty:
                pass
            
            time.sleep(0.05)  # avoid busy loop

    def _handle_is_door_open_service(self, request: IsDoorOpen.Request, response: IsDoorOpen.Response):

        # Use the private node's synchronous helper that internally spins its context
        res, message = self._private_node.request_world_status(wait_timeout=2.0)

        found = False
        if res is not None:
            for hallway in res.state.hallways:  # list of hallway info
                if (hallway.room_start == request.room1 and hallway.room_end == request.room2) or (hallway.room_start == request.room2 and hallway.room_end == request.room1):
                    response.door_open = hallway.is_open
                    response.message = f""
                    found = True
                    break
            if not found:
                response.message = f"The selected hallway between {request.room1} and {request.room2} does not exist"
        else:
            response.door_open = False  # default/fallback
            response.message = f"Internal error"
        return response
    
    def _process_execute_task_action(self, goal_handle : ServerGoalHandle, task_action: TaskAction, result):
        self._cancel_event = threading.Event()
        result.success, result.message = self._private_node.execute_task_sync(task_action, cancel_event=self._cancel_event)
        
        if goal_handle.is_active:
            goal_handle.succeed() if result.success else goal_handle.abort()
        
        return result
    
    def _cancel_callback(self, goal_handle: ServerGoalHandle):
        if goal_handle.is_active:
            self._cancel_event.set()
            return CancelResponse.ACCEPT
        return CancelResponse.REJECT
    
    def _execute_navigate_callback(self, goal_handle : ServerGoalHandle):
        request: Navigate.Goal = goal_handle.request
        
        # Use the private node's synchronous helper that internally spins its context
        curr_world, message = self._private_node.request_world_status(wait_timeout=2.0)
        if curr_world is None:
            goal_handle.abort()
            return Navigate.Result(success=False, message='Internal error')

        if not valid_room(curr_world.state.hallways, request.target_room):    
            goal_handle.abort()
            return Navigate.Result(success=False, message='Target is not a valid room')

        # Create TaskAction message properly
        task_action = TaskAction()
        task_action.robot = 'robot'
        task_action.type = 'navigate'
        task_action.target_location = request.target_room
        
        return self._process_execute_task_action(goal_handle, task_action, Navigate.Result())

    def _execute_approach_obj_callback(self, goal_handle : ServerGoalHandle):
        request: ApproachObject.Goal = goal_handle.request
        
        # Use the private node's synchronous helper that internally spins its context
        curr_world, message = self._private_node.request_world_status(wait_timeout=2.0)
        if curr_world is None:
            goal_handle.abort()
            return ApproachObject.Result(success=False, message='Internal error')
        
        # Check that approachable object is in the same room in which we are located
        curr_location = self._curr_room
        approach_ok = False
        for approachable_obj in curr_world.state.locations:
            if request.object == approachable_obj.name and approachable_obj.parent == curr_location:
                approach_ok = True
                break
        
        if not approach_ok:
            goal_handle.abort()
            return ApproachObject.Result(success=False, message=str(f"Object {request.object} to approach is not present in this room"))

        # Create TaskAction message properly
        task_action = TaskAction()
        task_action.robot = 'robot'
        task_action.type = 'navigate'
        task_action.target_location = request.object
        
        return self._process_execute_task_action(goal_handle, task_action, ApproachObject.Result())
    
    def _execute_approach_door_callback(self, goal_handle : ServerGoalHandle):
        request: ApproachDoor.Goal = goal_handle.request
        
        # Create TaskAction message properly
        task_action = TaskAction()
        task_action.robot = 'robot'
        task_action.type = 'navigate'
        task_action.target_location = str(f"hall_{request.room1}_{request.room2}")
        
        return self._process_execute_task_action(goal_handle, task_action, ApproachDoor.Result())
    
    def _execute_pick_callback(self, goal_handle : ServerGoalHandle):
        request: Pick.Goal = goal_handle.request

        # Create TaskAction message properly
        task_action = TaskAction()
        task_action.robot = 'robot'
        task_action.type = 'pick'
        task_action.object = request.object
        
        return self._process_execute_task_action(goal_handle, task_action, Pick.Result())
    
    def _execute_place_callback(self, goal_handle : ServerGoalHandle):
        request: Place.Goal = goal_handle.request
        
        if self._curr_robot_status.holding_object and self._curr_robot_status.manipulated_object == request.object:
            # Create TaskAction message properly
            task_action = TaskAction()
            task_action.robot = 'robot'
            task_action.type = 'place'
            task_action.object = request.object
            
            return self._process_execute_task_action(goal_handle, task_action, Place.Result())
        else:
            goal_handle.abort()
            return Place.Result(success=False, message=str(f"Robot is not currently holding {request.object}"))

    def _execute_open_callback(self, goal_handle : ServerGoalHandle):
        request: Open.Goal = goal_handle.request
        
        # Use the private node's synchronous helper that internally spins its context
        curr_world, message = self._private_node.request_world_status(wait_timeout=2.0)
        if curr_world is None:
            goal_handle.abort()
            return Open.Result(success=False, message='Internal error')

        # Create TaskAction message properly
        task_action = TaskAction()
        task_action.robot = 'robot'
        task_action.type = 'open'
        task_action.target_location = request.target
        
        return self._process_execute_task_action(goal_handle, task_action, Open.Result())
    
    def graceful_shutdown(self):
        self.get_logger().info("Shutting down consumer thread...")
        # If you had a more complex thread, you'd signal it to stop here.
        self._consumer_thread.join(timeout=2.0)
        self.get_logger().info("Consumer thread stopped.")


# ---------------------------
# Helpers to init contexts, nodes, executors, threads
# ---------------------------
def init_contexts_and_nodes(stop_event: threading.Event):
    # Create private context and init
    private_ctx = Context()
    rclpy.init(context=private_ctx, args=None, domain_id=PRIVATE_DOMAIN_ID)
    # Create private node with its context
    private_node = PyRobosimBridgePrivate(context=private_ctx)

    # Create public context and init
    public_ctx = Context()
    rclpy.init(context=public_ctx, args=None)
    # Create public node in public context
    public_node = PyRobosimBridgePublic(context=public_ctx, private_node=private_node, stop_event=stop_event)

    # Create executors for each context
    private_exec = SingleThreadedExecutor(context=private_ctx)
    public_exec = MultiThreadedExecutor(context=public_ctx, num_threads=4)

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
    # private_node = parts['private']['node']
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