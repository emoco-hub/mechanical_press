import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange
from std_msgs.msg import Bool, Float64
from std_srvs.srv import Trigger
from rclpy.parameter import Parameter
import time

class PressNode(Node):

    def __init__(self):
        super().__init__('mechanical_press')

        # Declare all parameters with type and ranges
        self.declare_typed_param('manual.force', 5.0, 0.1, 10.0, 0.1)
        self.declare_typed_param('manual.velocity', 300.0, 60.0, 600.0, 1.0)

        self.declare_typed_param('approach.force', 3.0, 0.1, 10.0, 0.1)
        self.declare_typed_param('approach.velocity', 200.0, 60.0, 600.0, 1.0)

        self.declare_typed_param('press.force', 10.0, 0.1, 10.0, 0.1)
        self.declare_typed_param('press.velocity', 100.0, 60.0, 600.0, 1.0)

        self.declare_typed_param('return.force', 4.0, 0.1, 10.0, 0.1)
        self.declare_typed_param('return.velocity', 400.0, 60.0, 600.0, 1.0)

        # Position parameters don't need validation ranges
        self.declare_parameter('home.position', 0.0)
        self.declare_parameter('start.position', 0.0)
        self.declare_parameter('end.position', 0.0)

        # NOTE: Mock position
        self.current_position = 0.0
        self.motion_timer = self.create_timer(0.05, self.update_position_sim)

        # Manual control dead man's grip
        self.up_sub = self.create_subscription(Bool, 'manual/up', self.deadman_cb_factory('up'), 10)
        self.down_sub = self.create_subscription(Bool, 'manual/down', self.deadman_cb_factory('down'), 10)

        self.deadman_state = {'up': False, 'down': False}
        self.deadman_timeout = 0.2
        self.last_active = {'up': 0.0, 'down': 0.0}

        self.deadman_timer = self.create_timer(0.05, self.check_deadman)

        # Position publisher
        self.position_pub = self.create_publisher(Float64, 'position', 10)

        # Position setting services
        self.srv_home = self.create_service(Trigger, 'save_home', self.save_pos_cb_factory('home'))
        self.srv_start = self.create_service(Trigger, 'save_start', self.save_pos_cb_factory('start'))
        self.srv_end = self.create_service(Trigger, 'save_end', self.save_pos_cb_factory('end'))

        # Action trigger services
        self.srv_home_trigger = self.create_service(Trigger, 'home', self.trigger_cb_factory('home'))
        self.srv_approach = self.create_service(Trigger, 'approach', self.trigger_cb_factory('approach'))
        self.srv_press = self.create_service(Trigger, 'press', self.trigger_cb_factory('press'))
        self.srv_stop = self.create_service(Trigger, 'stop', self.stop_cb)

        self.get_logger().info("Press node ready.")

    def declare_typed_param(self, name, default, min_val, max_val, step):
        descriptor = ParameterDescriptor(
            floating_point_range=[FloatingPointRange(
                from_value=float(min_val),
                to_value=float(max_val),
                step=float(step)
            )],
        description=f"{name} (range: {min_val}-{max_val}, step: {step})"
        )
        self.declare_parameter(name, default, descriptor)

    # NOTE: Mock
    def update_position_sim(self):
        dt = 0.05
        velocity = self.get_parameter('manual.velocity').value / 60.0  # mm/sec
        moved = False

        if self.deadman_state['up']:
            self.current_position += velocity * dt
            moved = True
        elif self.deadman_state['down']:
            self.current_position -= velocity * dt
            moved = True

        # self.get_logger().info(
        #     f"Pos: {self.current_position:.2f} | Vel: {velocity:.2f} | Up: {self.deadman_state['up']} | Down: {self.deadman_state['down']} | Moved: {moved}"
        # )

        msg = Float64()
        msg.data = self.current_position
        self.position_pub.publish(msg)

    def get_position(self):
        # Replace with actual read from motor
        return self.current_position

    def save_pos_cb_factory(self, label):
        def cb(req, res):
            pos = self.get_position()
            self.set_parameters([Parameter(f"{label}.position", Parameter.Type.DOUBLE, pos)])
            self.get_logger().info(f"Saved {label} position: {pos}")
            self.save_params_to_file()
            res.success = True
            res.message = f"{label} position saved"
            return res
        return cb

    def trigger_cb_factory(self, phase):
        def cb(req, res):
            self.get_logger().info(f"Trigger: {phase}")
            # TODO: Call CANopen node: set mode + values + start motor
            res.success = True
            return res
        return cb

    def stop_cb(self, req, res):
        self.get_logger().info("Stopping motor.")
        # TODO: Call CANopen node stop service
        res.success = True
        return res

    def deadman_cb_factory(self, direction):
        def cb(msg):
            if msg.data:
                self.last_active[direction] = self.get_clock().now().nanoseconds / 1e9
                if not self.deadman_state[direction]:
                    self.get_logger().info(f"Manual {direction} activated")
                    self.deadman_state[direction] = True
                    # TODO: Call CANopen node to start motion in {direction}
            else:
                if self.deadman_state[direction]:
                    self.get_logger().info(f"Manual {direction} deactivated by user input")
                    self.deadman_state[direction] = False
                    # TODO: Call CANopen node to stop motion for {direction}
        return cb

    def check_deadman(self):
        now = self.get_clock().now().nanoseconds / 1e9
        for direction in ['up', 'down']:
            if self.deadman_state[direction] and now - self.last_active[direction] > self.deadman_timeout:
                self.get_logger().info(f"Manual {direction} deactivated due to timeout")
                self.deadman_state[direction] = False
                # TODO: Call CANopen node to stop motion

    def save_params_to_file(self):
        try:
            from rclpy.parameter import Parameter
            import yaml
            param_dict = {}
            for param in self._parameters:
                if "." not in param:
                    continue
                section, key = param.split('.', 1)
                param_dict.setdefault(section, {})[key] = self.get_parameter(param).value
            with open('/tmp/mechanical_press_params.yaml', 'w') as f:
                yaml.dump({'ros__parameters': param_dict}, f)
        except Exception as e:
            self.get_logger().error(f"Failed to save parameters: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PressNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
