#!/usr/bin/env python3
"""
Keyboard control node for Autoware with GUI - works with launch files.
Uses tkinter GUI to capture keyboard events without requiring TTY.
"""

import signal
import threading
import tkinter as tk
from tkinter import ttk
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from tier4_control_msgs.msg import GateMode
from tier4_external_api_msgs.srv import Engage
from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import GearCommand, GearReport, VelocityReport
from autoware_vehicle_msgs.msg import Engage as EngageMsg


class KeyboardControlGUI(Node):
    """
    Keyboard control node with GUI for Autoware manual control.
    Uses arrow keys for intuitive control.
    """

    def __init__(self):
        super().__init__('keyboard_control_gui')

        # Declare parameters with defaults
        # Defaults sized for the small car, so a launch without the config file is safe too
        self.declare_parameter('speed_step_ms', 0.1)
        self.declare_parameter('steering_step_deg', 1.0)
        self.declare_parameter('max_speed_ms', 1.0)
        self.declare_parameter('max_steer_deg', 22.5)
        self.declare_parameter('publish_rate', 30.0)

        # Get parameters
        speed_step_ms = self.get_parameter('speed_step_ms').value
        steering_step_deg = self.get_parameter('steering_step_deg').value
        max_speed_ms = self.get_parameter('max_speed_ms').value
        max_steer_deg = self.get_parameter('max_steer_deg').value
        publish_rate = self.get_parameter('publish_rate').value

        # Validate parameters
        epsilon = 0.001
        if speed_step_ms <= epsilon:
            raise ValueError(f'speed_step_ms must be positive (got {speed_step_ms})')
        if steering_step_deg <= epsilon:
            raise ValueError(f'steering_step_deg must be positive (got {steering_step_deg})')
        if max_speed_ms < 0.0:
            raise ValueError(f'max_speed_ms must be non-negative (got {max_speed_ms})')
        if max_steer_deg < 0.0:
            raise ValueError(f'max_steer_deg must be non-negative (got {max_steer_deg})')
        if publish_rate <= epsilon:
            raise ValueError(f'publish_rate must be positive (got {publish_rate})')

        # Convert to SI units
        self.STEP_SPEED = speed_step_ms  # Already in m/s
        self.STEP_STEER_ANGLE = steering_step_deg * 3.14159 / 180.0  # deg to rad
        self.MAX_SPEED = max_speed_ms  # Already in m/s
        self.MAX_STEER_ANGLE = max_steer_deg * 3.14159 / 180.0  # deg to rad

        # Calculate min/max step counts
        import math
        self.max_speed_steps = int(math.floor(self.MAX_SPEED / self.STEP_SPEED))
        self.min_speed_steps = -self.max_speed_steps
        self.max_steer_steps = int(math.floor(self.MAX_STEER_ANGLE / self.STEP_STEER_ANGLE))
        self.min_steer_steps = -self.max_steer_steps

        self.get_logger().info(f'Speed step: {speed_step_ms:.2f} m/s')
        self.get_logger().info(f'  Step range: {self.min_speed_steps} to {self.max_speed_steps} steps')
        self.get_logger().info(f'Steering step: {steering_step_deg}° ({self.STEP_STEER_ANGLE:.4f} rad)')
        self.get_logger().info(f'  Step range: {self.min_steer_steps} to {self.max_steer_steps} steps')
        self.get_logger().info(f'Max speed: {max_speed_ms:.2f} m/s')
        self.get_logger().info(f'Max steering: {max_steer_deg}° ({self.MAX_STEER_ANGLE:.4f} rad)')

        # State variables - use step counts
        self.gear_type = GearCommand.DRIVE
        self.speed_steps = 0  # Integer step count
        self.steering_steps = 0  # Integer step count

        # Initialize calculated values
        self.target_velocity = 0.0
        self.steering_tire_angle = 0.0
        # Calculate initial values from step counts (both are 0, so result is 0.0)
        self.calculate_values()

        # Status variables
        self.gate_mode = GateMode.AUTO
        self.current_engage = False
        self.current_gear_type = GearReport.PARK
        self.current_velocity = 0.0

        # Topic configuration
        self.control_cmd_topic = '/external/selected/control_cmd'
        self.gear_cmd_topic = '/external/selected/gear_cmd'

        # Publishers (will be recreated when topics change)
        self.pub_gate_mode = self.create_publisher(
            GateMode, '/control/gate_mode_cmd', QoSProfile(depth=1))
        self.pub_control_cmd = None
        self.pub_gear_cmd = None
        self.create_publishers()

        # Subscribers
        self.sub_gate_mode = self.create_subscription(
            GateMode, '/control/current_gate_mode', self.on_gate_mode, 10)
        self.sub_engage = self.create_subscription(
            EngageMsg, '/api/autoware/get/engage', self.on_engage_status, 10)
        self.sub_velocity = self.create_subscription(
            VelocityReport, '/vehicle/status/velocity_status', self.on_velocity, 1)
        self.sub_gear = self.create_subscription(
            GearReport, '/vehicle/status/gear_status', self.on_gear, 10)

        # Service client
        self.client_engage = self.create_client(Engage, '/api/autoware/set/engage')

        # Timer for publishing commands
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.publish_cmd)

        # GUI update timer (10 Hz)
        self.gui_update_timer = self.create_timer(0.1, self.update_gui_callback)

        # GUI reference (will be set by GUI thread)
        self.gui = None

        self.get_logger().info('Keyboard control GUI node initialized')

    def create_publishers(self):
        """Create or recreate publishers based on current topics."""
        # Destroy old publishers if they exist
        if self.pub_control_cmd:
            self.destroy_publisher(self.pub_control_cmd)
        if self.pub_gear_cmd:
            self.destroy_publisher(self.pub_gear_cmd)

        # Create new publishers
        self.pub_control_cmd = self.create_publisher(
            Control, self.control_cmd_topic, QoSProfile(depth=1))
        self.pub_gear_cmd = self.create_publisher(
            GearCommand, self.gear_cmd_topic, QoSProfile(depth=1))

        self.get_logger().info('Publishers created:')
        self.get_logger().info(f'  Control: {self.control_cmd_topic}')
        self.get_logger().info(f'  Gear: {self.gear_cmd_topic}')

    def set_topics(self, control_topic, gear_topic):
        """Set custom topics and recreate publishers."""
        self.control_cmd_topic = control_topic
        self.gear_cmd_topic = gear_topic
        self.create_publishers()

    def on_gate_mode(self, msg):
        """Callback for gate mode updates."""
        self.gate_mode = msg.data

    def on_engage_status(self, msg):
        """Callback for engage status updates."""
        self.current_engage = msg.engage

    def on_velocity(self, msg):
        """Callback for velocity updates."""
        self.current_velocity = msg.longitudinal_velocity

    def on_gear(self, msg):
        """Callback for gear status updates."""
        self.current_gear_type = msg.report

    def toggle_manual_control(self):
        """Toggle between AUTO and EXTERNAL gate modes."""
        switch_to_external = (self.gate_mode != GateMode.EXTERNAL)

        if self.gate_mode == GateMode.EXTERNAL:
            # Switch to AUTO
            msg = GateMode()
            msg.data = GateMode.AUTO
            self.pub_gate_mode.publish(msg)
            self.get_logger().info('Switched to AUTO mode')
        else:
            # Switch to EXTERNAL
            msg = GateMode()
            msg.data = GateMode.EXTERNAL
            self.pub_gate_mode.publish(msg)

            # Engage
            if not self.client_engage.service_is_ready():
                self.get_logger().warn('Engage service not available')
            else:
                req = Engage.Request()
                req.engage = True
                self.client_engage.call_async(req)

            self.get_logger().info('Switched to EXTERNAL mode')

        return switch_to_external

    def update_gear_cmd(self, gear_type):
        """Update gear command."""
        self.gear_type = gear_type

    def calculate_values(self):
        """Calculate actual velocity and steering angle from step counts."""
        self.target_velocity = max(min(
            self.STEP_SPEED * self.speed_steps,
            self.MAX_SPEED), -self.MAX_SPEED)
        self.steering_tire_angle = max(min(
            self.STEP_STEER_ANGLE * self.steering_steps,
            self.MAX_STEER_ANGLE), -self.MAX_STEER_ANGLE)

    def get_status_dict(self):
        """Get current status as dictionary."""
        gear_map = {
            GearReport.PARK: 'PARK',
            GearReport.REVERSE: 'REVERSE',
            GearReport.DRIVE: 'DRIVE',
            GearReport.LOW: 'LOW'
        }

        mode_map = {
            GateMode.AUTO: 'AUTO',
            GateMode.EXTERNAL: 'EXTERNAL'
        }

        return {
            'engage': 'Ready' if self.current_engage else 'Not Ready',
            'gate_mode': mode_map.get(self.gate_mode, 'Unknown'),
            'gear': gear_map.get(self.gear_type, '?'),
            'target_speed_ms': self.target_velocity,
            'current_speed_ms': self.current_velocity,
            'angle_deg': self.steering_tire_angle * 180 / 3.14159,
        }

    def publish_cmd(self):
        """Publish control commands at regular intervals."""
        # Publish control command
        control = Control()
        control.stamp = self.get_clock().now().to_msg()
        control.lateral.steering_tire_angle = self.steering_tire_angle

        # Negative velocity for reverse
        real_target_velocity = self.target_velocity * (
            -1 if self.gear_type == GearReport.REVERSE else 1)
        control.longitudinal.velocity = real_target_velocity

        # Calculate acceleration
        acceleration = min(max(
            (self.target_velocity - abs(self.current_velocity)) * 0.5, -1.0), 1.0)
        control.longitudinal.acceleration = acceleration

        self.pub_control_cmd.publish(control)

        # Publish gear command
        gear_cmd = GearCommand()
        gear_cmd.command = self.gear_type
        self.pub_gear_cmd.publish(gear_cmd)

    def update_gui_callback(self):
        """Timer callback to update GUI from ROS thread."""
        if self.gui:
            self.gui.update_status(self.get_status_dict())

    def process_key(self, key):
        """Process keyboard input."""
        changed = False

        # Mode control
        if key == 'z':
            self.toggle_manual_control()
        elif key == 'x':
            self.update_gear_cmd(GearCommand.DRIVE)
            self.get_logger().info('Gear: DRIVE')
        elif key == 'c':
            self.update_gear_cmd(GearCommand.REVERSE)
            self.get_logger().info('Gear: REVERSE')
        elif key == 'v':
            self.update_gear_cmd(GearCommand.PARK)
            self.get_logger().info('Gear: PARK')

        # Speed control - arrow keys modify step counts
        elif key == 'Up':
            self.speed_steps = min(self.speed_steps + 1, self.max_speed_steps)
            changed = True
        elif key == 'Down':
            self.speed_steps = max(self.speed_steps - 1, self.min_speed_steps)
            changed = True
        elif key == 'space':
            self.speed_steps = 0
            changed = True
            self.get_logger().info('STOP!')

        # Steering control - arrow keys modify step counts
        elif key == 'Left':
            self.steering_steps = min(self.steering_steps + 1, self.max_steer_steps)
            changed = True
        elif key == 'Right':
            self.steering_steps = max(self.steering_steps - 1, self.min_steer_steps)
            changed = True
        elif key == 'Return':
            self.steering_steps = 0
            changed = True
            self.get_logger().info('Steering centered')

        if changed:
            self.calculate_values()

    def set_gui(self, gui):
        """Set GUI reference."""
        self.gui = gui


class ControlGUI:
    """GUI window for keyboard control."""

    def __init__(self, node):
        self.node = node
        self.node.set_gui(self)

        # Create main window
        self.root = tk.Tk()
        self.root.title("AutoSDV Keyboard Control")
        self.root.geometry("600x500")
        self.root.configure(bg='#2b2b2b')

        # Style configuration
        style = ttk.Style()
        style.theme_use('clam')
        style.configure('Title.TLabel', font=('Arial', 16, 'bold'),
                       background='#2b2b2b', foreground='#ffffff')
        style.configure('Status.TLabel', font=('Arial', 12),
                       background='#2b2b2b', foreground='#00ff00')
        style.configure('Help.TLabel', font=('Arial', 10),
                       background='#2b2b2b', foreground='#aaaaaa')
        style.configure('Value.TLabel', font=('Arial', 14, 'bold'),
                       background='#2b2b2b', foreground='#00ffff')
        style.configure('Topic.TLabel', font=('Arial', 10),
                       background='#2b2b2b', foreground='#ffaa00')

        # Status update flag
        self.status_data = {}

        # Topic presets (must be defined before create_widgets)
        self.topic_presets = {
            'External (Standard)': {
                'control': '/external/selected/control_cmd',
                'gear': '/external/selected/gear_cmd'
            },
            'Direct (Bypass)': {
                'control': '/control/command/control_cmd',
                'gear': '/control/command/gear_cmd'
            },
            'Custom': {
                'control': '',
                'gear': ''
            }
        }

        self.preset_var = tk.StringVar(value='External (Standard)')
        self.custom_control_var = tk.StringVar(value='/external/selected/control_cmd')
        self.custom_gear_var = tk.StringVar(value='/external/selected/gear_cmd')

        # Create widgets and bind keys
        self.create_widgets()
        self.bind_keys()

    def create_widgets(self):
        """Create GUI widgets."""
        # Title
        title = ttk.Label(self.root, text="AutoSDV Keyboard Control",
                         style='Title.TLabel')
        title.pack(pady=10)

        # Status frame
        status_frame = tk.Frame(self.root, bg='#2b2b2b')
        status_frame.pack(pady=10, padx=20, fill='x')

        # Status labels
        self.engage_label = ttk.Label(status_frame, text="Engage: Not Ready",
                                     style='Status.TLabel')
        self.engage_label.pack(anchor='w', pady=2)

        self.mode_label = ttk.Label(status_frame, text="Mode: AUTO",
                                   style='Status.TLabel')
        self.mode_label.pack(anchor='w', pady=2)

        self.gear_label = ttk.Label(status_frame, text="Gear: PARK",
                                   style='Status.TLabel')
        self.gear_label.pack(anchor='w', pady=2)

        # Topic selection frame
        topic_frame = tk.Frame(self.root, bg='#2b2b2b', relief='ridge', borderwidth=2)
        topic_frame.pack(pady=10, padx=20, fill='x')

        topic_title = ttk.Label(topic_frame, text="Output Topic Preset:",
                               style='Status.TLabel')
        topic_title.pack(anchor='w', padx=10, pady=5)

        # Preset dropdown
        preset_frame = tk.Frame(topic_frame, bg='#2b2b2b')
        preset_frame.pack(fill='x', padx=10, pady=5)

        ttk.Label(preset_frame, text="Preset:", style='Help.TLabel').pack(side='left', padx=5)

        preset_combo = ttk.Combobox(
            preset_frame,
            textvariable=self.preset_var,
            values=list(self.topic_presets.keys()),
            state='readonly',
            width=25)
        preset_combo.pack(side='left', padx=5)
        preset_combo.bind('<<ComboboxSelected>>', lambda e: self.on_preset_change())

        # Custom topic entry fields (initially hidden)
        self.custom_frame = tk.Frame(topic_frame, bg='#2b2b2b')

        # Control command topic
        control_frame = tk.Frame(self.custom_frame, bg='#2b2b2b')
        control_frame.pack(fill='x', padx=10, pady=3)
        ttk.Label(control_frame, text="Control Topic:", style='Help.TLabel', width=15).pack(side='left')
        self.control_entry = tk.Entry(
            control_frame, textvariable=self.custom_control_var,
            bg='#1a1a1a', fg='#ffffff', insertbackground='#ffffff',
            font=('Arial', 9), width=40)
        self.control_entry.pack(side='left', padx=5, fill='x', expand=True)

        # Gear command topic
        gear_frame = tk.Frame(self.custom_frame, bg='#2b2b2b')
        gear_frame.pack(fill='x', padx=10, pady=3)
        ttk.Label(gear_frame, text="Gear Topic:", style='Help.TLabel', width=15).pack(side='left')
        self.gear_entry = tk.Entry(
            gear_frame, textvariable=self.custom_gear_var,
            bg='#1a1a1a', fg='#ffffff', insertbackground='#ffffff',
            font=('Arial', 9), width=40)
        self.gear_entry.pack(side='left', padx=5, fill='x', expand=True)

        # Apply button for custom topics
        apply_btn = tk.Button(
            self.custom_frame, text="Apply Custom Topics",
            command=self.apply_custom_topics,
            bg='#0066cc', fg='#ffffff', activebackground='#0088ff',
            font=('Arial', 10, 'bold'), relief='raised', borderwidth=2)
        apply_btn.pack(pady=5)

        # Current topics display
        self.topic_info_label = ttk.Label(
            topic_frame,
            text="Publishing to: /external/selected/*",
            style='Topic.TLabel')
        self.topic_info_label.pack(anchor='w', padx=10, pady=5)

        # Control values frame
        values_frame = tk.Frame(self.root, bg='#2b2b2b')
        values_frame.pack(pady=10, padx=20, fill='x')

        self.speed_label = ttk.Label(values_frame, text="Speed: 0.0 km/h",
                                    style='Value.TLabel')
        self.speed_label.pack(anchor='w', pady=2)

        # Speed range
        max_speed = self.node.MAX_SPEED
        speed_range_label = ttk.Label(
            values_frame,
            text=f"  Range: -{max_speed:.2f} to +{max_speed:.2f} m/s",
            style='Help.TLabel')
        speed_range_label.pack(anchor='w', pady=0)

        self.angle_label = ttk.Label(values_frame, text="Angle: 0.0°",
                                    style='Value.TLabel')
        self.angle_label.pack(anchor='w', pady=2)

        # Steering range
        max_steer_deg = self.node.MAX_STEER_ANGLE * 180 / 3.14159
        steer_range_label = ttk.Label(
            values_frame,
            text=f"  Range: -{max_steer_deg:.1f}° to +{max_steer_deg:.1f}°",
            style='Help.TLabel')
        steer_range_label.pack(anchor='w', pady=0)

        # Help text
        separator = ttk.Separator(self.root, orient='horizontal')
        separator.pack(fill='x', padx=20, pady=10)

        help_frame = tk.Frame(self.root, bg='#2b2b2b')
        help_frame.pack(pady=10, padx=20, fill='both', expand=True)

        # Get step sizes from node configuration
        speed_step = self.node.STEP_SPEED
        steering_step_deg = self.node.STEP_STEER_ANGLE * 180 / 3.14159

        help_text = f"""
Controls:
  ↑/↓       : Increase/Decrease speed ({speed_step:.2f} m/s)
  ←/→       : Turn left/right ({steering_step_deg:.1f}°)
  Space     : Stop (speed = 0)
  Enter     : Center steering

  z         : Toggle AUTO/EXTERNAL mode
  x         : Gear DRIVE
  c         : Gear REVERSE
  v         : Gear PARK

  q/Esc     : Quit

Focus this window and use keyboard!
        """

        help_label = ttk.Label(help_frame, text=help_text,
                              style='Help.TLabel', justify='left')
        help_label.pack(anchor='w')

        # Focus indicator
        focus_label = ttk.Label(self.root,
                               text="Click here to focus for keyboard input",
                               style='Status.TLabel')
        focus_label.pack(pady=5)

    def bind_keys(self):
        """Bind keyboard events."""
        self.root.bind('<Key>', self.on_key_press)
        self.root.bind('<Up>', lambda e: self.node.process_key('Up'))
        self.root.bind('<Down>', lambda e: self.node.process_key('Down'))
        self.root.bind('<Left>', lambda e: self.node.process_key('Left'))
        self.root.bind('<Right>', lambda e: self.node.process_key('Right'))
        self.root.bind('<space>', lambda e: self.node.process_key('space'))
        self.root.bind('<Return>', lambda e: self.node.process_key('Return'))
        self.root.bind('<Escape>', lambda e: self.quit())
        self.root.bind('<q>', lambda e: self.quit())

        # Click to focus
        self.root.bind('<Button-1>', lambda e: self.root.focus_set())

    def on_preset_change(self):
        """Handle preset selection change."""
        preset = self.preset_var.get()

        if preset == 'Custom':
            # Show custom entry fields
            self.custom_frame.pack(fill='x', padx=10, pady=10)
        else:
            # Hide custom entry fields
            self.custom_frame.pack_forget()

            # Apply preset topics
            topics = self.topic_presets[preset]
            self.node.set_topics(topics['control'], topics['gear'])
            self.update_topic_display(topics['control'], topics['gear'])

    def apply_custom_topics(self):
        """Apply custom topics from entry fields."""
        control_topic = self.custom_control_var.get().strip()
        gear_topic = self.custom_gear_var.get().strip()

        if not control_topic or not gear_topic:
            # Show error - topics cannot be empty
            self.topic_info_label.config(
                text="ERROR: Topics cannot be empty!",
                foreground='#ff0000')
            return

        # Apply custom topics
        self.node.set_topics(control_topic, gear_topic)
        self.update_topic_display(control_topic, gear_topic)

    def update_topic_display(self, control_topic, gear_topic):
        """Update topic info display."""
        # Simplify display if both topics share a common prefix
        if '/' in control_topic and '/' in gear_topic:
            control_parts = control_topic.rsplit('/', 1)
            gear_parts = gear_topic.rsplit('/', 1)

            if control_parts[0] == gear_parts[0]:
                # Same prefix
                prefix = control_parts[0]
                text = f"Publishing to: {prefix}/*"
            else:
                text = f"Control: {control_topic}\nGear: {gear_topic}"
        else:
            text = f"Control: {control_topic} | Gear: {gear_topic}"

        self.topic_info_label.config(text=text, foreground='#ffaa00')

    def on_key_press(self, event):
        """Handle key press events."""
        if event.keysym not in ['Up', 'Down', 'Left', 'Right', 'space', 'Return', 'Escape']:
            self.node.process_key(event.char)

    def update_status(self, status):
        """Update status display."""
        self.status_data = status

        # Schedule GUI update in main thread
        self.root.after(0, self._update_gui)

    def _update_gui(self):
        """Update GUI widgets (must be called from main thread)."""
        if not self.status_data:
            return

        s = self.status_data

        self.engage_label.config(text=f"Engage: {s.get('engage', 'Unknown')}")
        self.mode_label.config(text=f"Mode: {s.get('gate_mode', 'Unknown')}")
        self.gear_label.config(text=f"Gear: {s.get('gear', '?')}")
        self.speed_label.config(
            text=f"Speed: {s.get('target_speed_ms', 0.0):.2f} m/s "
                 f"(actual: {s.get('current_speed_ms', 0.0):.2f})")
        self.angle_label.config(text=f"Angle: {s.get('angle_deg', 0.0):.1f}°")

    def quit(self):
        """Quit application."""
        self.root.quit()
        rclpy.shutdown()

    def run(self):
        """Run GUI main loop."""
        self.root.mainloop()


def main(args=None):
    """Main function."""
    rclpy.init(args=args)

    node = KeyboardControlGUI()

    # Create and run GUI in main thread
    gui = ControlGUI(node)

    # Signal handler for Ctrl-C
    def signal_handler(sig, frame):
        """Handle Ctrl-C gracefully."""
        gui.quit()

    signal.signal(signal.SIGINT, signal_handler)

    # Run ROS spinning in separate thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # Run GUI in main thread
    gui.run()

    # Cleanup
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
