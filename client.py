import paho.mqtt.client as mqtt
import json
import time
from pipuck.pipuck import PiPuck
import random
import math

# Define variables and callbacks
Broker = "192.168.178.56"  # Replace with your broker address
Port = 1883 # standard MQTT port
pi_puck_id = '17'
x_bound = 2.0
y_bound = 1.0
new_message = [True]
speed = 400

puck_dict = {}

# function to handle connection
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe("robot_pos/all")
    client.subscribe(f"robot/{pi_puck_id}")

# function to handle incoming messages
def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode())
        if msg.topic == "robot_pos/all":
            # Check if the message is a dictionary
            puck_dict.update(data)
        elif msg.topic == f"robot/{pi_puck_id}":
            new_message[0] = True
    except json.JSONDecodeError:
        print(f'invalid json: {msg.payload}')

# Initialize MQTT client
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(Broker, Port, 60)

client.loop_start() # Start listening loop in separate thread

# Initialize the PiPuck
pipuck = PiPuck(epuck_version=2)

# Set the robot's speed, e.g. with
# pipuck.epuck.set_motor_speeds(speed,-speed)

def check_bounds(x, y, radius = 0.0):
    if x < 0 + radius or x > x_bound - radius:
        return False
    if y < 0 + radius or y > y_bound - radius:
        return False
    return True

def collsion_detected(x, y, radius = 0.0):
    # Check if the robot is within the bounds of the arena
    if x < 0 + radius or x > x_bound - radius:
        return True, None
    if y < 0 + radius or y > y_bound - radius:
        return True, None
    # Check for collision with other robots
    for key, value in puck_dict.items():
        if key != pi_puck_id:
            pos = value.get('position')
            if pos:
                other_x = pos[0]
                other_y = pos[1]
                distance = ((x - other_x) ** 2 + (y - other_y) ** 2) ** 0.5
                if distance < radius:
                    return True, key
    return False, None

def get_position(id=pi_puck_id):
    data = puck_dict.get(id)
    if data:
        pos = data.get('position')
        if pos:
            x = pos[0]
            y = pos[1]
            angle = data.get('angle')
            return x, y, angle
    else:
        print(f"No data for PiPuck ID: {id}")
    return None, None, None

# battery = pipuck.get_battery_state()
# print(f"Charging?: {battery[0]}, Battery Voltage: {battery[1]}V, Battery percentage: {battery[2]}%")

# At the top of your code, define states
STATE_IDLE = 0
STATE_TURNING = 1
STATE_MOVING = 2
STATE_RANDOM = 3
STATE_TARGET = 4
current_state = STATE_IDLE
target_angle = 0
target_pipuck_id = '44'
target_x = None
target_y = None

# Then in your main loop:
for i in range(9999999):
    # Process MQTT and update position first
    x, y, angle = get_position()
    print(f"Current State: {current_state}, Position: ({x}, {y}), Angle: {angle}")
    
    if x is None or y is None or angle is None:
        print("Waiting for position data...")
        time.sleep(0.1)
        continue
    if current_state == STATE_IDLE:
        # Your normal behavior when idle
        if collsion_detected(x, y, radius=0.1)[0]:
            # Start turning state
            target_angle = (angle + 180) % 360
            current_state = STATE_TURNING
            pipuck.epuck.set_motor_speeds(-speed, speed)
        elif target_pipuck_id is not None:
            t_x, t_y, t_angle = get_position(target_pipuck_id)
            if t_x is not None and t_y is not None:
                if check_bounds(t_x, t_y):
                    # Move towards the target pipuck
                    current_state = STATE_TARGET
                    pipuck.epuck.set_motor_speeds(speed, speed)
        else:
            random_direction = random.randint(0, 3)
            if random_direction == 0:
                current_state = STATE_RANDOM

    elif current_state == STATE_TARGET:
        # Move towards the target pipuck
        target_x, target_y, target_angle = get_position(target_pipuck_id)
        if target_x is not None and target_y is not None:
            angle1 = math.degrees(math.atan2(target_y - y, target_x - x))
            # now get the angle from the y-axis to the target
            target_angle = (-angle1 + 90) % 360
            print(f"Target Angle: {target_angle}")
            # turn towards the target
            if not (target_angle > angle + 5 or target_angle < angle - 5):
                # Move towards the target
                pipuck.epuck.set_motor_speeds(speed, speed)
            else:
                # Turn towards the target
                # Calculate the smallest difference between angles (handling wrap-around)
                angle_diff = (target_angle - angle + 540) % 360 - 180
                turn_speed = max(5 * abs(angle_diff), 100)
                if angle_diff > 0:
                    pipuck.epuck.set_motor_speeds(turn_speed, -turn_speed)
                else:
                    pipuck.epuck.set_motor_speeds(-turn_speed, turn_speed)
            if collsion_detected(x, y, radius=0.1)[0]:
                # Start turning state
                target_angle = (angle + 180) % 360
                current_state = STATE_TURNING
                pipuck.epuck.set_motor_speeds(-speed, speed)
            # Check if we've reached the target
            if abs(target_x - x) < 0.1 and abs(target_y - y) < 0.1:
                # Stop moving towards the target
                current_state = STATE_IDLE
                pipuck.epuck.set_motor_speeds(0, 0)
                # get random new target
                old_target_pipuck_id = target_pipuck_id
                target_pipuck_id = random.choice(list(puck_dict.keys()))
                while target_pipuck_id == pi_puck_id or target_pipuck_id == old_target_pipuck_id:
                    target_pipuck_id = random.choice(list(puck_dict.keys()))
                print(f"New target: {target_pipuck_id}")
        else:
            current_state = STATE_IDLE
            pipuck.epuck.set_motor_speeds(0, 0)
            old_target_pipuck_id = target_pipuck_id
            target_pipuck_id = random.choice(list(puck_dict.keys()))
            while target_pipuck_id == pi_puck_id or target_pipuck_id == old_target_pipuck_id:
                target_pipuck_id = random.choice(list(puck_dict.keys()))
            print(f"New target: {target_pipuck_id}")

    elif current_state == STATE_RANDOM:
        # Randomly choose a direction to turn
        random_direction = random.randint(0, 2)
        if random_direction == 0:
            pipuck.epuck.set_motor_speeds(speed, -speed)
        elif random_direction == 1:
            pipuck.epuck.set_motor_speeds(-speed, speed)
        elif random_direction == 2:
            current_state = STATE_IDLE
            pipuck.epuck.set_motor_speeds(speed, speed)
            
    elif current_state == STATE_TURNING:
        # Check if we've reached the desired angle
        if not (angle > target_angle + 15 or angle < target_angle - 15):
            # Transition to moving state
            current_state = STATE_MOVING
            pipuck.epuck.set_motor_speeds(speed, speed)
        else:
            # Continue turning
            pipuck.epuck.set_motor_speeds(-speed, speed)
    
    elif current_state == STATE_MOVING:
        # Check if we've moved enough or detected a collision
        if not collsion_detected(x, y, radius=0.1)[0]:
            current_state = STATE_IDLE
    
    time.sleep(0.1)
	
    
# Stop the MQTT client loop
pipuck.epuck.set_motor_speeds(0,0)
client.loop_stop()  
