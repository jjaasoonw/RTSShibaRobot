"""
Combined controller for Spot robot with speech recognition support.
"""

from controller import Robot, Motor, Camera, LED
import socket
import threading
import math
import time
import speech_recognition as sr
import random

class ERS7Robot:
    # Constants
    NUMBER_OF_JOINTS = 16  # Updated for ERS-7
    NUMBER_OF_CAMERAS = 1  # ERS-7 has one camera
    NUMBER_OF_LEDS = 8
    TIME_STEP = 32  # ms
    PORT = 12345
    BUFFER_SIZE = 1024

    def __init__(self):
        # Initialize the robot
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # Initialize camera with proper error handling and debug output
        self.camera = self.robot.getDevice('PRM:/r1/c1/c2/c3/i1-FbkImageSensor:F1')
        if self.camera:
            self.camera.enable(self.timestep)
            # Add debug info about camera
        else:
            print("ERROR: Camera not found")
        
        # Initialize distance sensor properly with error handling
        try:
            self.sensor = self.robot.getDevice('ds')  # Update with correct sensor name
            if self.sensor:
                self.sensor.enable(self.timestep)
                print("Distance sensor initialized successfully")
            else:
                print("ERROR: Distance sensor not found")
        except Exception as e:
            print(f"ERROR initializing distance sensor: {e}")
            self.sensor = None
        
        # Constants
        self.TIME_STEP = 32  # ms
        self.PORT = 12345
        self.BUFFER_SIZE = 1024
        
        # Initialize motors with correct ERS-7 motor names
        self.motors = {}
        
        
        self.sensor = self.robot.getDevice('ds')
        self.sensor.enable(self.timestep)
        
        # Motor names from the device tree
        self.motor_names = {
            # Head and face motors
            'neck_tilt': 'PRM:/r1/c1-Joint2:11',
            'head_pan': 'PRM:/r1/c1/c2-Joint2:12',
            'head_tilt': 'PRM:/r1/c1/c2/c3-Joint2:13',
            'left_ear': 'PRM:/r1/c1/c2/c3/e5-Joint4:15',
            'right_ear': 'PRM:/r1/c1/c2/c3/e6-Joint4:16',
            'jaw': 'PRM:/r1/c1/c2/c3/c4-Joint2:14',
            
            # Tail motors - Updated with correct names from the image
            'tail_tilt': 'PRM:/r6/c1-Joint2:61',
            'tail_pan': 'PRM:/r6/c2-Joint2:62',
            
            # Right foreleg
            'right_foreleg_j1': 'PRM:/r4/c1-Joint2:41',
            'right_foreleg_j2': 'PRM:/r4/c1/c2-Joint2:42',
            'right_foreleg_j3': 'PRM:/r4/c1/c2/c3-Joint2:43',
            
            # Right hindleg
            'right_hindleg_j1': 'PRM:/r5/c1-Joint2:51',
            'right_hindleg_j2': 'PRM:/r5/c1/c2-Joint2:52',
            'right_hindleg_j3': 'PRM:/r5/c1/c2/c3-Joint2:53',
            
            # Left foreleg
            'left_foreleg_j1': 'PRM:/r2/c1-Joint2:21',
            'left_foreleg_j2': 'PRM:/r2/c1/c2-Joint2:22',
            'left_foreleg_j3': 'PRM:/r2/c1/c2/c3-Joint2:23',
            
            # Left hindleg
            'left_hindleg_j1': 'PRM:/r3/c1-Joint2:31',
            'left_hindleg_j2': 'PRM:/r3/c1/c2-Joint2:32',
            'left_hindleg_j3': 'PRM:/r3/c1/c2/c3-Joint2:33'
        }
        
        # Initialize all motors and set to standing position
        for name, device_name in self.motor_names.items():
            motor = self.robot.getDevice(device_name)
            if motor:
                self.motors[name] = motor
                motor.setPosition(0.0)
                motor.setVelocity(1.0)
            else:
                print(f"Warning: Motor {device_name} not found")

        # Update the leg motor references for easier access
        self.legs = {
            'right_fore': {
                'j1': self.motors.get('right_foreleg_j1'),
                'j2': self.motors.get('right_foreleg_j2'),
                'j3': self.motors.get('right_foreleg_j3')
            },
            'right_hind': {
                'j1': self.motors.get('right_hindleg_j1'),
                'j2': self.motors.get('right_hindleg_j2'),
                'j3': self.motors.get('right_hindleg_j3')
            },
            'left_fore': {
                'j1': self.motors.get('left_foreleg_j1'),
                'j2': self.motors.get('left_foreleg_j2'),
                'j3': self.motors.get('left_foreleg_j3')
            },
            'left_hind': {
                'j1': self.motors.get('left_hindleg_j1'),
                'j2': self.motors.get('left_hindleg_j2'),
                'j3': self.motors.get('left_hindleg_j3')
            }
        }

        # Set initial standing position
        self.ensure_standing()

        # Initialize command server
        self.server_socket = None
        self.client_socket = None
        self.setup_server()

        # Remove autonomous behavior parameters and just walk forward
        self.walk_forward()

        # Add new state variables
        self.is_alert = False
        self.is_wandering = True
        self.last_wander_time = 0
        self.wander_direction_time = 0
        self.current_wander_direction = None

        
        # Update motor mappings to include correct tail motor names
        self.tail_motors = {
            'tail_pan': self.motors.get('tail_pan'),  # Use the name from motor_names dictionary
            'tail_tilt': self.motors.get('tail_tilt')  # Use the name from motor_names dictionary
        }

    def setup_server(self):
        """Setup TCP server for receiving commands."""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind(('localhost', self.PORT))
        self.server_socket.listen(1)

    def accept_client(self):
        """Accept client connection in a separate thread."""
        while True:
            self.client_socket, addr = self.server_socket.accept()
            print(f"Connected to {addr}")
            self.handle_commands()

    def handle_commands(self):
        """Handle incoming commands from the speech recognition client."""
        while True:
            try:
                data = self.client_socket.recv(self.BUFFER_SIZE)
                if not data:
                    break
                
                command = data.decode().strip()
                print(f"Received command: {command}")
                
                if command == "stand up":
                    self.stand()
                elif command == "sit down":
                    self.sit_down()
                elif command == "lie down":
                    self.lie_down()
                elif command == "give hand":
                    self.give_hand()
                elif command == "move forward":
                    self.walk_forward()
                elif command == "move backward":
                    self.move_backward()
                elif command == "turn left":
                    self.turn_left()
                elif command == "turn right":
                    self.turn_right()
                
            except socket.error:
                break
        
        self.client_socket.close()

    def step(self):
        """Perform one simulation step."""
        return self.robot.step(self.TIME_STEP)

    def movement_decomposition(self, target, duration):
        """Smoothly move joints to target positions."""
        steps = int(duration * 1000 / self.TIME_STEP)
        current_position = [motor.getTargetPosition() for motor in self.motors.values()]
        step_difference = [(t - c) / steps for t, c in zip(target, current_position)]
        
        for _ in range(steps):
            for motor_name, motor in self.motors.items():
                current_position[motor_names.index(motor_name)] += step_difference[motor_names.index(motor_name)]
                motor.setPosition(current_position[motor_names.index(motor_name)])
            if self.step() == -1:
                break

    # Movement functions
    def stand(self):
        """Make the robot stand"""
        # Set neutral standing position for all legs
        for leg in self.legs.values():
            if leg['j1']: leg['j1'].setPosition(0.0)    # Center position
            if leg['j2']: leg['j2'].setPosition(-0.15)  # Slightly bent, within limits
            if leg['j3']: leg['j3'].setPosition(0.3)    # Slightly bent
        
        # Set head and tail to neutral position
        if 'neck_tilt' in self.motors: self.motors['neck_tilt'].setPosition(0.0)
        if 'head_pan' in self.motors: self.motors['head_pan'].setPosition(0.0)
        if 'head_tilt' in self.motors: self.motors['head_tilt'].setPosition(0.0)
        if 'tail_tilt' in self.motors: self.motors['tail_tilt'].setPosition(0.0)
        if 'tail_pan' in self.motors: self.motors['tail_pan'].setPosition(0.0)

    def sit_down(self, duration=2.0):
        """Make the robot sit by adjusting leg positions"""
        # Set sitting positions for each leg, respecting joint limits
        sitting_positions = {
            # Front legs stay relatively straight but slightly bent
            'right_fore': {'j1': 0.2, 'j2': -0.25, 'j3': 0.3},
            'left_fore': {'j1': -0.2, 'j2': -0.25, 'j3': 0.3},
            # Hind legs fold under the body - adjusted angles for better tucking
            'right_hind': {'j1': -1.2, 'j2': -0.25, 'j3': 0.8},  # Changed j1 direction for right hind leg
            'left_hind': {'j1': -1.2, 'j2': -0.25, 'j3': 0.8}    # Left hind leg stays the same
        }
        
        # Phase 2 adjustments also modified for better position
        front_adjust = {
            'right_fore': {'j1': 0.2, 'j2': -0.2, 'j3': 0.5},   # Increased j3 bend
            'left_fore': {'j1': -0.2, 'j2': -0.2, 'j3': 0.5},   # Increased j3 bend
            'right_hind': sitting_positions['right_hind'],       # Keep hind legs in place
            'left_hind': sitting_positions['left_hind']
        }
        
        # Execute the motion in two phases
        # Phase 1: Begin folding hind legs while keeping front legs stable
        steps = int(duration * 500 / self.TIME_STEP)  # First half of the motion
        current_positions = {}
        
        # Get current positions
        for leg_name, leg in self.legs.items():
            current_positions[leg_name] = {
                'j1': leg['j1'].getTargetPosition() if leg['j1'] else 0,
                'j2': leg['j2'].getTargetPosition() if leg['j2'] else 0,
                'j3': leg['j3'].getTargetPosition() if leg['j3'] else 0
            }
        
        # Calculate step sizes for first phase
        step_sizes = {}
        for leg_name in self.legs:
            step_sizes[leg_name] = {
                joint: (sitting_positions[leg_name][joint] - current_positions[leg_name][joint]) / steps
                for joint in ['j1', 'j2', 'j3']
            }
        
        # Execute first phase - fold hind legs
        for _ in range(steps):
            for leg_name, leg in self.legs.items():
                for joint in ['j1', 'j2', 'j3']:
                    if leg[joint]:
                        current_positions[leg_name][joint] += step_sizes[leg_name][joint]
                        leg[joint].setPosition(current_positions[leg_name][joint])
            
            if self.robot.step(self.timestep) == -1:
                break
        
        # Phase 2: Lower the body by adjusting front legs
        for leg_name, leg in self.legs.items():
            for joint in ['j1', 'j2', 'j3']:
                if leg[joint]:
                    leg[joint].setPosition(front_adjust[leg_name][joint])
        
        # Set tail position for sitting pose
        if 'tail_tilt' in self.motors:
            self.motors['tail_tilt'].setPosition(0.5)  # Tail slightly up

    def lie_down(self, duration=2.0):
        """Make the robot lie down on its left side in a sleeping position"""
        # Phase 1: First lower the body
        lower_positions = {
            'right_fore': {'j1': 0.1, 'j2': -0.25, 'j3': 0.4},
            'left_fore': {'j1': -0.1, 'j2': -0.25, 'j3': 0.4},
            'right_hind': {'j1': 0.1, 'j2': -0.25, 'j3': 0.4},
            'left_hind': {'j1': -0.1, 'j2': -0.25, 'j3': 0.4}
        }
        
        # Apply lowering position
        for leg_name, positions in lower_positions.items():
            for joint, pos in positions.items():
                if self.legs[leg_name][joint]:
                    self.legs[leg_name][joint].setPosition(pos)
        
        # Wait for stability
        for _ in range(20):
            if self.robot.step(self.timestep) == -1:
                return
        
        # Phase 2: Rotate to the left side while moving right legs forward
        side_positions = {
            # Left legs (bottom side) - supporting the fall
            'left_fore': {'j1': 0.4, 'j2': -0.15, 'j3': 0.3},
            'left_hind': {'j1': 0.4, 'j2': -0.15, 'j3': 0.3},
            # Right legs (top side) - moved forward
            'right_fore': {'j1': -0.4, 'j2': -0.2, 'j3': 0.4},  # Forward position
            'right_hind': {'j1': -0.4, 'j2': -0.2, 'j3': 0.4}   # Forward position
        }
        
        # Apply side lying position gradually
        steps = int(duration * 500 / self.timestep)  # Half the duration for this phase
        current_positions = {}
        
        # Get current positions
        for leg_name, leg in self.legs.items():
            current_positions[leg_name] = {
                'j1': leg['j1'].getTargetPosition() if leg['j1'] else 0,
                'j2': leg['j2'].getTargetPosition() if leg['j2'] else 0,
                'j3': leg['j3'].getTargetPosition() if leg['j3'] else 0
            }
        
        # Calculate step sizes
        step_sizes = {}
        for leg_name in self.legs:
            step_sizes[leg_name] = {
                joint: (side_positions[leg_name][joint] - current_positions[leg_name][joint]) / steps
                for joint in ['j1', 'j2', 'j3']
            }
        
        # Gradually move to side lying position
        for step in range(steps):
            # Calculate progress (0 to 1)
            progress = step / steps
            
            # Apply non-linear easing for more natural movement
            eased_progress = progress * progress  # Quadratic easing
            
            for leg_name, leg in self.legs.items():
                for joint in ['j1', 'j2', 'j3']:
                    if leg[joint]:
                        target = current_positions[leg_name][joint] + (step_sizes[leg_name][joint] * steps * eased_progress)
                        leg[joint].setPosition(target)
            
            if self.robot.step(self.timestep) == -1:
                return
        
        # Final adjustments for sleeping pose
        final_positions = {
            'left_fore': {'j1': 0.4, 'j2': -0.15, 'j3': 0.3},
            'left_hind': {'j1': 0.4, 'j2': -0.15, 'j3': 0.3},
            'right_fore': {'j1': -0.5, 'j2': -0.2, 'j3': 0.4},
            'right_hind': {'j1': -0.5, 'j2': -0.2, 'j3': 0.4}
        }
        
        # Apply final position
        for leg_name, positions in final_positions.items():
            for joint, pos in positions.items():
                if self.legs[leg_name][joint]:
                    self.legs[leg_name][joint].setPosition(pos)
        
        # Set head and tail positions for sleeping pose - adjusted within limits
        if 'neck_tilt' in self.motors:
            self.motors['neck_tilt'].setPosition(0.05)  # Slight tilt
        if 'head_pan' in self.motors:
            self.motors['head_pan'].setPosition(-0.2)   # Turn head slightly left
        if 'head_tilt' in self.motors:
            self.motors['head_tilt'].setPosition(0.0)   # Level head
        if 'tail_tilt' in self.motors:
            self.motors['tail_tilt'].setPosition(0.1)   # Relaxed tail position
        if 'tail_pan' in self.motors:
            self.motors['tail_pan'].setPosition(0.0)    # Center tail
        
        # Optional: Add ear positions for a relaxed look
        if 'left_ear' in self.motors:
            self.motors['left_ear'].setPosition(0.0)
        if 'right_ear' in self.motors:
            self.motors['right_ear'].setPosition(0.0)

    def turn_left(self):
        """Make the robot turn left with smoother motion"""
        # Ensure standing position before turning
        self.ensure_standing()
        
        # Smaller steps for smoother motion
        for step in range(8):  # Increased number of steps for smoother motion
            # Phase 1: Slightly lift diagonal legs
            if step % 2 == 0:
                # Lift diagonal pair (right front and left back)
                positions = {
                    'right_fore': {'j1': 0.2, 'j2': -0.15, 'j3': 0.3},
                    'left_hind': {'j1': 0.2, 'j2': -0.15, 'j3': 0.3},
                    # Keep other legs planted
                    'left_fore': {'j1': -0.1, 'j2': -0.1, 'j3': 0.2},
                    'right_hind': {'j1': -0.1, 'j2': -0.1, 'j3': 0.2}
                }
            else:
                # Lift alternate diagonal pair (left front and right back)
                positions = {
                    'left_fore': {'j1': -0.2, 'j2': -0.15, 'j3': 0.3},
                    'right_hind': {'j1': -0.2, 'j2': -0.15, 'j3': 0.3},
                    # Keep other legs planted
                    'right_fore': {'j1': 0.1, 'j2': -0.1, 'j3': 0.2},
                    'left_hind': {'j1': 0.1, 'j2': -0.1, 'j3': 0.2}
                }
            
            # Apply positions
            for leg_name, pos in positions.items():
                for joint, angle in pos.items():
                    if self.legs[leg_name][joint]:
                        self.legs[leg_name][joint].setPosition(angle)
            
            # Small delay for smooth motion
            for _ in range(3):  # Reduced wait time for smoother motion
                if self.robot.step(self.timestep) == -1:
                    return
        
        # Return to standing position
        self.ensure_standing()

    def turn_right(self):
        """Make the robot turn right with smoother motion"""
        # Ensure standing position before turning
        self.ensure_standing()
        
        # Smaller steps for smoother motion
        for step in range(8):  # Increased number of steps for smoother motion
            # Phase 1: Slightly lift diagonal legs
            if step % 2 == 0:
                # Lift diagonal pair (right front and left back)
                positions = {
                    'right_fore': {'j1': -0.2, 'j2': -0.15, 'j3': 0.3},
                    'left_hind': {'j1': -0.2, 'j2': -0.15, 'j3': 0.3},
                    # Keep other legs planted
                    'left_fore': {'j1': 0.1, 'j2': -0.1, 'j3': 0.2},
                    'right_hind': {'j1': 0.1, 'j2': -0.1, 'j3': 0.2}
                }
            else:
                # Lift alternate diagonal pair (left front and right back)
                positions = {
                    'left_fore': {'j1': 0.2, 'j2': -0.15, 'j3': 0.3},
                    'right_hind': {'j1': 0.2, 'j2': -0.15, 'j3': 0.3},
                    # Keep other legs planted
                    'right_fore': {'j1': -0.1, 'j2': -0.1, 'j3': 0.2},
                    'left_hind': {'j1': -0.1, 'j2': -0.1, 'j3': 0.2}
                }
            
            # Apply positions
            for leg_name, pos in positions.items():
                for joint, angle in pos.items():
                    if self.legs[leg_name][joint]:
                        self.legs[leg_name][joint].setPosition(angle)
            
            # Small delay for smooth motion
            for _ in range(3):  # Reduced wait time for smoother motion
                if self.robot.step(self.timestep) == -1:
                    return
        
        # Return to standing position
        self.ensure_standing()

    def walk_forward(self):
        """Make the robot walk forward with coordinated leg movements"""
        # Ensure standing position before walking
        self.ensure_standing()
        
        amplitude = 0.2  # Reduced from 0.3 to stay within limits
        period = 1000  # ms
        
        # Joint limits
        J2_MIN = -0.2618  # Minimum angle for J2 joints (about -15 degrees)
        J2_MAX = 0.5236   # Maximum angle for J2 joints (about 30 degrees)
        
        start_time = self.robot.getTime() * 1000
        while (self.robot.getTime() * 1000 - start_time) < period:
            phase = 2 * math.pi * ((self.robot.getTime() * 1000 - start_time) % period) / period
            
            # Check obstacle sensors with proper error handling
            try:
                if self.sensor:
                    distance = self.sensor.getValue()
                    if distance < 500:  # Obstacle detected
                        print("Obstacle detected at distance:", distance)
                        self.stop_action()
                        if self.robot.step(self.timestep) == -1:
                            break
                else:
                    print("Warning: No distance sensor available")
            except Exception as e:
                print(f"Error reading distance sensor: {e}")
            
            # Diagonal leg pairs move together
            # Left fore and right hind
            for leg, phase_offset in [
                ('left_fore', 0),
                ('right_hind', 0),
                ('right_fore', math.pi),
                ('left_hind', math.pi)
            ]:
                if all(self.legs[leg].values()):
                    # J1 (shoulder/hip) movement
                    self.legs[leg]['j1'].setPosition(amplitude * math.sin(phase + phase_offset))
                    
                    # J2 (upper leg) movement - respect joint limits
                    j2_position = -0.15 - 0.1 * math.sin(phase + phase_offset)
                    j2_position = max(J2_MIN, min(J2_MAX, j2_position))
                    self.legs[leg]['j2'].setPosition(j2_position)
                    
                    # J3 (lower leg) movement
                    self.legs[leg]['j3'].setPosition(0.3 + 0.1 * math.sin(phase + phase_offset))
            
            if self.robot.step(self.timestep) == -1:
                break

    def move_backward(self):
        """Make the robot walk backward with coordinated leg movements"""
        # Ensure standing position before walking
        self.ensure_standing()
        
        amplitude = 0.2  # Movement amplitude
        period = 1000  # ms
        
        # Joint limits
        J2_MIN = -0.2618  # Minimum angle for J2 joints (about -15 degrees)
        J2_MAX = 0.5236   # Maximum angle for J2 joints (about 30 degrees)
        
        start_time = self.robot.getTime() * 1000
        while (self.robot.getTime() * 1000 - start_time) < period:
            phase = 2 * math.pi * ((self.robot.getTime() * 1000 - start_time) % period) / period
            
            # Similar to walk_forward but with reversed phase for backward motion
            for leg, phase_offset in [
                ('left_fore', math.pi),  # Reversed phase
                ('right_hind', math.pi), # Reversed phase
                ('right_fore', 0),       # Reversed phase
                ('left_hind', 0)         # Reversed phase
            ]:
                if all(self.legs[leg].values()):
                    # J1 (shoulder/hip) movement - reversed from walk_forward
                    self.legs[leg]['j1'].setPosition(-amplitude * math.sin(phase + phase_offset))
                    
                    # J2 (upper leg) movement - respect joint limits
                    j2_position = -0.15 - 0.1 * math.sin(phase + phase_offset)
                    j2_position = max(J2_MIN, min(J2_MAX, j2_position))
                    self.legs[leg]['j2'].setPosition(j2_position)
                    
                    # J3 (lower leg) movement
                    self.legs[leg]['j3'].setPosition(0.3 + 0.1 * math.sin(phase + phase_offset))
            
            if self.robot.step(self.timestep) == -1:
                break

    def give_hand(self, duration=2.0):
        """Make the robot sit and give its right hand"""
        # Define sitting position angles
        sit_positions = {
            'right_fore': {'j1': 0.2, 'j2': -0.2, 'j3': 0.3},
            'left_fore': {'j1': -0.2, 'j2': -0.2, 'j3': 0.3},
            'right_hind': {'j1': -0.8, 'j2': -0.2, 'j3': 0.6},
            'left_hind': {'j1': -0.8, 'j2': -0.2, 'j3': 0.6}
        }
        
        # Check if already in sitting position by comparing current angles
        is_sitting = True
        tolerance = 0.2  # Tolerance for angle comparison
        
        for leg_name, leg in self.legs.items():
            for joint in ['j1', 'j2', 'j3']:
                if leg[joint]:
                    current_pos = leg[joint].getTargetPosition()
                    target_pos = sit_positions[leg_name][joint]
                    if abs(current_pos - target_pos) > tolerance:
                        is_sitting = False
                        break
            if not is_sitting:
                break
        
        # Only perform sit action if not already sitting
        if not is_sitting:
            # Apply sitting position
            for leg_name, positions in sit_positions.items():
                for joint, pos in positions.items():
                    if self.legs[leg_name][joint]:
                        self.legs[leg_name][joint].setPosition(pos)
            
            # Wait for stability in sitting position
            for _ in range(20):
                if self.robot.step(self.timestep) == -1:
                    return
        
        # Phase 2: Raise right hand
        hand_positions = {
            'right_fore': {'j1': 0.2, 'j2': -0.2, 'j3': 0.6},  # Adjusted angles for right hand
            'left_fore': {'j1': -0.2, 'j2': -0.2, 'j3': 0.3},  # Keep left hand stable
            'right_hind': {'j1': -0.8, 'j2': -0.2, 'j3': 0.6}, # Keep hind legs in sitting position
            'left_hind': {'j1': -0.8, 'j2': -0.2, 'j3': 0.6}
        }
        
        # Gradually raise the hand
        steps = int(duration * 500 / self.timestep)
        current_positions = {}
        
        # Get current positions
        for leg_name, leg in self.legs.items():
            current_positions[leg_name] = {
                'j1': leg['j1'].getTargetPosition() if leg['j1'] else 0,
                'j2': leg['j2'].getTargetPosition() if leg['j2'] else 0,
                'j3': leg['j3'].getTargetPosition() if leg['j3'] else 0
            }
        
        # Calculate step sizes
        step_sizes = {}
        for leg_name in self.legs:
            step_sizes[leg_name] = {
                joint: (hand_positions[leg_name][joint] - current_positions[leg_name][joint]) / steps
                for joint in ['j1', 'j2', 'j3']
            }
        
        # Smoothly raise the hand
        for step in range(steps):
            # Calculate progress (0 to 1)
            progress = step / steps
            
            # Apply non-linear easing for more natural movement
            eased_progress = progress * progress  # Quadratic easing
            
            for leg_name, leg in self.legs.items():
                for joint in ['j1', 'j2', 'j3']:
                    if leg[joint]:
                        target = current_positions[leg_name][joint] + (step_sizes[leg_name][joint] * steps * eased_progress)
                        leg[joint].setPosition(target)
        
        if self.robot.step(self.timestep) == -1:
            return
    
        # Hold the hand up position for a moment
        for _ in range(40):
            if self.robot.step(self.timestep) == -1:
                return
    
        # Optional: Add a slight tail wag while holding hand up
        if 'tail_pan' in self.motors:
            self.motors['tail_pan'].setPosition(0.2)  # Wag tail to one side
    
        # Add friendly head tilt
        if 'head_tilt' in self.motors:
            self.motors['head_tilt'].setPosition(0.05)  # Slight head tilt
    
        # Hold the pose with tail wag for a moment
        for _ in range(20):
            if self.robot.step(self.timestep) == -1:
                return
        
        # Return to standing position
        self.ensure_standing()

    def ensure_standing(self):
        """Ensure the robot is in a stable standing position"""
        # Set legs to a stable standing position
        standing_positions = {
            'right_fore': {'j1': 0.0, 'j2': -0.1, 'j3': 0.2},
            'left_fore': {'j1': 0.0, 'j2': -0.1, 'j3': 0.2},
            'right_hind': {'j1': 0.0, 'j2': -0.1, 'j3': 0.2},
            'left_hind': {'j1': 0.0, 'j2': -0.1, 'j3': 0.2}
        }
        
        # Apply standing position
        for leg_name, positions in standing_positions.items():
            for joint, pos in positions.items():
                if self.legs[leg_name][joint]:
                    self.legs[leg_name][joint].setPosition(pos)
        
        # Set head and tail to neutral position
        if 'neck_tilt' in self.motors: self.motors['neck_tilt'].setPosition(0.0)
        if 'head_pan' in self.motors: self.motors['head_pan'].setPosition(0.0)
        if 'head_tilt' in self.motors: self.motors['head_tilt'].setPosition(0.0)
        if 'tail_tilt' in self.motors: self.motors['tail_tilt'].setPosition(0.0)
        if 'tail_pan' in self.motors: self.motors['tail_pan'].setPosition(0.0)
        
        # Wait for stability
        for _ in range(10):  # Give time for position to stabilize
            self.robot.step(self.timestep)

    def clap_hands(self, duration=2.0):
        """Make the robot clap its front paws together while standing"""
        # First ensure stable standing position with slightly bent hind legs
        stabilize_positions = {
            # Front legs lifted for clapping - using angles within joint limits
            'right_fore': {'j1': 0.3, 'j2': -0.25, 'j3': 0.8},
            'left_fore': {'j1': -0.3, 'j2': -0.25, 'j3': 0.8},
            # Hind legs in normal standing position for stability
            'right_hind': {'j1': 0.0, 'j2': -0.2, 'j3': 0.4},
            'left_hind': {'j1': 0.0, 'j2': -0.2, 'j3': 0.4}
        }
        
        # Apply initial position
        for leg_name, positions in stabilize_positions.items():
            for joint, pos in positions.items():
                if self.legs[leg_name][joint]:
                    self.legs[leg_name][joint].setPosition(pos)
        
        # Wait for stability
        for _ in range(20):
            if self.robot.step(self.timestep) == -1:
                return
        
        # Clapping motion
        claps = 3  # Number of claps
        for _ in range(claps):
            # Move front paws apart
            apart_positions = {
                'right_fore': {'j1': 0.4, 'j2': -0.25, 'j3': 0.8},
                'left_fore': {'j1': -0.4, 'j2': -0.25, 'j3': 0.8}
            }
            
            for leg_name, positions in apart_positions.items():
                for joint, pos in positions.items():
                    if self.legs[leg_name][joint]:
                        self.legs[leg_name][joint].setPosition(pos)
            
            # Short delay
            for _ in range(10):
                if self.robot.step(self.timestep) == -1:
                    return
            
            # Bring front paws together
            together_positions = {
                'right_fore': {'j1': 0.1, 'j2': -0.25, 'j3': 0.8},
                'left_fore': {'j1': -0.1, 'j2': -0.25, 'j3': 0.8}
            }
            
            for leg_name, positions in together_positions.items():
                for joint, pos in positions.items():
                    if self.legs[leg_name][joint]:
                        self.legs[leg_name][joint].setPosition(pos)
            
            # Short delay
            for _ in range(10):
                if self.robot.step(self.timestep) == -1:
                    return
        
        # Return to standing position
        self.ensure_standing()

    def wag_tail(self, duration=1.0, intensity=0.3):
        """Make the robot wag its tail with given intensity and duration"""
        if not self.tail_motors.get('tail_pan'):
            print("Tail pan motor not found!")
            print("Available tail motors:", self.tail_motors)
            print("All motors:", self.motors)
            return
        
        start_time = self.robot.getTime()
        frequency = 2.0  # Wags per second
        
        while self.robot.getTime() - start_time < duration:
            # Calculate tail position using sine wave
            time_passed = self.robot.getTime() - start_time
            tail_pos = intensity * math.sin(2 * math.pi * frequency * time_passed)
            
            # Set tail position using the correct motor
            self.tail_motors['tail_pan'].setPosition(tail_pos)
            
            # Optional: Add slight body movement for more realistic wagging
            if time_passed % (1/frequency) < 0.1:
                for leg_name, leg in self.legs.items():
                    if leg['j1']:
                        current_pos = leg['j1'].getTargetPosition()
                        leg['j1'].setPosition(current_pos + 0.02 * tail_pos)
            
            if self.robot.step(self.timestep) == -1:
                break

    def happy_wag(self, duration=3.0):
        """Make the robot do an enthusiastic tail wag with head movement"""
        # Start with a happy head movement
        if 'head_tilt' in self.motors:
            self.motors['head_tilt'].setPosition(0.05)
        if 'ear_left' in self.motors:
            self.motors['ear_left'].setPosition(0.0)
        if 'ear_right' in self.motors:
            self.motors['ear_right'].setPosition(0.0)
        
        # Do an enthusiastic tail wag
        if not self.tail_motors.get('tail_pan'):
            print("Tail pan motor not found!")
            return
        
        start_time = self.robot.getTime()
        frequency = 3.0  # Faster wags for happy wagging
        
        while self.robot.getTime() - start_time < duration:
            time_passed = self.robot.getTime() - start_time
            intensity = 0.6 + 0.1 * math.sin(time_passed)  # Varying intensity
            tail_pos = intensity * math.sin(2 * math.pi * frequency * time_passed)
            
            # Set tail position using the correct motor
            self.tail_motors['tail_pan'].setPosition(tail_pos)
            
            # Add slight head movement synchronized with tail
            if 'head_pan' in self.motors:
                head_pos = 0.1 * math.sin(2 * math.pi * frequency * time_passed)
                self.motors['head_pan'].setPosition(head_pos)
            
            if self.robot.step(self.timestep) == -1:
                break
        
        # Return head to neutral position
        if 'head_pan' in self.motors:
            self.motors['head_pan'].setPosition(0.0)

    def wander(self):
        """Make the robot wander around randomly with occasional tail wagging"""
        current_time = self.robot.getTime()
        
        # Change direction every 3 seconds
        if current_time - self.wander_direction_time > 3.0:
            self.wander_direction_time = current_time
            # Randomly choose a new direction
            choice = random.random()
            if choice < 0.3:
                self.current_wander_direction = self.walk_forward
            elif choice < 0.45:
                self.current_wander_direction = self.turn_left
            elif choice < 0.6:
                self.current_wander_direction = self.turn_right
            elif choice < 0.7:  # 10% chance to wag tail
                self.current_wander_direction = None
                self.wag_tail(duration=1.0, intensity=0.4)  # Playful tail wag
            else:
                self.current_wander_direction = None  # Brief pause
            
        # Execute current wandering direction
        if self.current_wander_direction:
            self.current_wander_direction()

    def detect_ball(self):
        """Detect a red ball in the camera image"""
        if not self.camera:
            print("ERROR: No camera available")
            return False, None
        
        # Get the image from the camera
        image = self.camera.getImage()
        if not image:
            print("ERROR: No image captured")
            return False, None
        
        width = self.camera.getWidth()
        height = self.camera.getHeight()
        
        # Look for red ball pixels
        ball_x = 0
        ball_y = 0
        ball_pixels = 0
        
        # Sample pixels more frequently for small ball detection
        for y in range(0, height-2, 1):  # Sample every pixel for better detection
            for x in range(0, width-2, 1):
                # Get RGB values - each pixel is 4 bytes (RGBA)
                offset = 4 * (y * width + x)
                r = image[offset]
                g = image[offset + 1]
                b = image[offset + 2]
                
                # Check specifically for red color with more lenient thresholds
                if (r > 150 and           # Reduced threshold for red
                    g < 120 and           # Increased threshold for green
                    b < 120 and           # Increased threshold for blue
                    r - max(g, b) > 50):  # Reduced difference requirement
                    ball_x += x
                    ball_y += y
                    ball_pixels += 1
        
        # Increased minimum pixels to consider it a ball
        if ball_pixels > 15:  # Changed from 10 to 15 pixels
            ball_x = ball_x / ball_pixels
            ball_y = ball_y / ball_pixels
            # Convert to relative position (-1 to 1, where 0 is center)
            relative_x = (ball_x - width/2) / (width/2)
            print(f"Red ball detected with {ball_pixels} pixels at relative_x: {relative_x}")
            return True, relative_x
            
        return False, None

    def find_and_sit_by_ball(self):
        """Find a red ball and sit next to it"""
        # First ensure we're standing
        self.ensure_standing()
        
        # Search for ball
        ball_found, relative_x = self.detect_ball()
        
        if ball_found:
            print(f"Red ball found at relative position: {relative_x}")
            
            # If ball is centered enough
            if abs(relative_x) < 0.2:  # Ball is roughly centered
                # Get another image to check distance
                image = self.camera.getImage()
                if not image:
                    return False
                    
                width = self.camera.getWidth()
                height = self.camera.getHeight()
                
                # Count red pixels to estimate distance
                red_pixels = 0
                total_pixels = 0
                
                for y in range(0, height-2, 1):  # Sample every pixel
                    for x in range(0, width-2, 1):
                        total_pixels += 1
                        offset = 4 * (y * width + x)
                        r = image[offset]
                        g = image[offset + 1]
                        b = image[offset + 2]
                        
                        # Check specifically for red color with more lenient thresholds
                        if (r > 150 and 
                            g < 120 and 
                            b < 120 and 
                            r - max(g, b) > 50):
                            red_pixels += 1
                
                # Calculate percentage of red pixels
                red_percentage = (red_pixels / total_pixels) * 100
                print(f"Distance check: {red_pixels} red pixels ({red_percentage:.2f}% of image)")
                
                # Adjusted thresholds for smaller balls
                if red_pixels > 200:  # Reduced from 400 to 200
                    print("Very close to red ball, moving to sit beside it")
                    # Turn slightly to position for sitting beside ball
                    if relative_x > 0:  # Ball is on the right
                        # Take smaller turn for closer positioning
                        for _ in range(2):
                            self.turn_left()
                            if self.robot.step(self.timestep) == -1:
                                return False
                        # Smaller steps forward for precise positioning
                        for _ in range(2):
                            self.walk_forward()
                            if self.robot.step(self.timestep) == -1:
                                return False
                    else:  # Ball is on the left
                        # Take smaller turn for closer positioning
                        for _ in range(2):
                            self.turn_right()
                            if self.robot.step(self.timestep) == -1:
                                return False
                        # Smaller steps forward for precise positioning
                        for _ in range(2):
                            self.walk_forward()
                            if self.robot.step(self.timestep) == -1:
                                return False
                    
                    print("In position, sitting down right next to red ball")
                    self.sit_down()
                    return True
                else:
                    print("Not close enough, moving closer to red ball")
                    # Take smaller steps when getting close
                    if red_pixels > 100:  # Reduced from 200 to 100
                        print("Getting very close, taking small steps")
                        for _ in range(2):
                            self.walk_forward()
                            if self.robot.step(self.timestep) == -1:
                                return False
                    else:
                        self.walk_forward()
            else:
                # Turn towards ball
                if relative_x > 0:
                    print("Turning right towards red ball")
                    self.turn_right()
                else:
                    print("Turning left towards red ball")
                    self.turn_left()
        else:
            print("Searching for red ball...")
            self.turn_right()
        
        return False

    def setup_speech_recognition(self):
        """Setup speech recognition."""
        self.recognizer = sr.Recognizer()
        self.current_action = None
        self.commands = {
            "stand up": self.stand,
            "sit down": self.sit_down,
            "lie down": self.lie_down,
            "give hand": self.give_hand,
            "forward": self.walk_forward,
            "backward": self.move_backward,
            "turn left": self.turn_left,
            "turn right": self.turn_right,
            "stop": self.stop_action,
            "clap": self.clap_hands,
            "happy": self.happy_wag,
            "make noise": self.bark,
            "find ball": self.find_and_sit_by_ball  # This line already exists
        }
        
        # Start speech recognition in a separate thread
        speech_thread = threading.Thread(target=self.listen_for_commands)
        speech_thread.daemon = True
        speech_thread.start()

    def stop_action(self):
        """Stop the current continuous action"""
        self.current_action = None
        self.ensure_standing()

    def listen_for_commands(self):
        """Listen for voice commands using speech recognition."""
        while True:
            with sr.Microphone() as source:
                print("Listening..." if self.is_alert else "Wandering...")
                try:
                    audio = self.recognizer.listen(source)
                    text = self.recognizer.recognize_google(audio).lower()
                    print(f"Recognized: {text}")
                    
                    # Check for name call first
                    if "shiba" in text:
                        print("Shiba is now alert and listening for commands!")
                        self.is_alert = True
                        self.is_wandering = False
                        self.current_action = None
                        # Make the dog look alert and wag tail
                        if 'head_tilt' in self.motors:
                            self.motors['head_tilt'].setPosition(0.05)
                        if 'ear_left' in self.motors:
                            self.motors['ear_left'].setPosition(0.0)
                        if 'ear_right' in self.motors:
                            self.motors['ear_right'].setPosition(0.0)
                        # Happy tail wag when called
                        self.happy_wag()
                        continue
                    
                    # Only process other commands if alert
                    if self.is_alert:
                        if "go play" in text:
                            print("Shiba is going to play!")
                            self.is_alert = False
                            self.is_wandering = True
                            self.current_action = None
                            # Happy tail wag before going to play
                            self.happy_wag()
                            continue
                            
                        if "stop" in text:
                            print("Stopping current action")
                            self.stop_action()
                        else:
                            for key, command_func in self.commands.items():
                                if key in text:
                                    print(f"Executing command: {key}")
                                    self.current_action = command_func
                                    command_func()
                                    break
                            
                except sr.UnknownValueError:
                    print("Could not understand audio")
                except sr.RequestError as e:
                    print(f"Could not request results; {e}")
                except Exception as e:
                    print(f"Error in speech recognition: {e}")

    def bark(self, duration=1.0, num_barks=2):
        """Make the robot bark by moving its jaw"""
        if 'jaw' not in self.motors:
            print("Jaw motor not found!")
            return
        
        # Slightly lift front legs to prepare for barking
        front_leg_positions = {
            'right_fore': {'j1': 0.05, 'j2': 0.0, 'j3': 0.0},
            'left_fore': {'j1': 0.05, 'j2': 0.0, 'j3': 0.0}
        }
        
        # Set front legs position
        for leg_name, positions in front_leg_positions.items():
            for joint, pos in positions.items():
                if self.legs[leg_name][joint]:
                    self.legs[leg_name][joint].setPosition(pos)
        
        # Do multiple barks
        for _ in range(num_barks):
            # Print bark sound
            print("Woof!")
            
            # Open jaw very slightly
            self.motors['jaw'].setPosition(-0.2)
            
            # Hold for a moment
            for _ in range(5):
                if self.robot.step(self.timestep) == -1:
                    return
            
            # Close jaw
            self.motors['jaw'].setPosition(0.0)
            
            # Pause between barks
            for _ in range(8):
                if self.robot.step(self.timestep) == -1:
                    return
        
        # Return to neutral position
        self.ensure_standing()

    def run(self):
        """Main control loop."""
        # Start both command handling thread and speech recognition
        command_thread = threading.Thread(target=self.accept_client)
        command_thread.daemon = True
        command_thread.start()
        
        # Initialize speech recognition
        self.setup_speech_recognition()
        
        print("Shiba is initialized")
        
        # Main simulation loop
        while True:
            if self.step() == -1:
                break
            
            # Execute current action if one is set
            if self.current_action:
                self.current_action()
            # Wander if not alert and no specific action
            elif self.is_wandering:
                self.wander()

def main():
    robot = ERS7Robot()
    robot.run()

if __name__ == "__main__":
    main() 