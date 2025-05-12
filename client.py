import paho.mqtt.client as mqtt
import json
import time
from pipuck.pipuck import PiPuck
import random

# Define variables and callbacks
Broker = "192.168.178.56"  # Replace with your broker address
Port = 1883 # standard MQTT port
pi_puck_id = '17'
x_bound = 2.0
y_bound = 1.0
x_pos = 0.0
y_pos = 0.0

puck_dict = {}

# function to handle connection
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe("robot_pos/all")

# function to handle incoming messages
def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode())
        # Check if the message is a dictionary
        puck_dict.update(data)
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
# pipuck.epuck.set_motor_speeds(1000,-1000)

def check_bounds(x, y, radius = 0.0):
    if x < 0 + radius or x > x_bound - radius:
        return False
    if y < 0 + radius or y > y_bound - radius:
        return False
    return True

def get_position():
    data = puck_dict.get(pi_puck_id)
    if data:
        pos = data.get('position')
        if pos:
            x = pos[0]
            y = pos[1]
            angle = data.get('angle')
            return x, y, angle
    else:
        print(f"No data for PiPuck ID: {pi_puck_id}")
    return None, None
    
# Function to get the closest wall
# Returns 0 for left, 1 for right, 2 for top, 3 for bottom
def closest_wall(x, y):
    # Calculate the distance to each wall
    left_distance = x
    right_distance = x_bound - x
    top_distance = y_bound - y
    bottom_distance = y

    # Find the closest wall
    distances = [left_distance, right_distance, top_distance, bottom_distance]
    closest_wall_index = distances.index(min(distances))
    
    return closest_wall_index
# angle == 0 => robot is facing top
# angle == 90 => robot is facing right
# angle == 180 => robot is facing bottom
# angle == 270 => robot is facing left
# Returns the angle to the wall in degrees
def get_angle_to_wall(wall_index, angle):
    # Calculate the angle to the wall based on the wall index and robot's angle
    if wall_index == 0:  # left wall
        return (360 - angle) % 180

for i in range(5000):
    # TODO: Do your stuff here
    # Print the updated dictionary
    print(f"Updated puck_dict: {puck_dict}")
    # Get the current position of the robot
    x, y, angle = get_position()
    print(f"Current position: {x}, {y}")
    # drive randomly
    if x is not None and y is not None:
        if check_bounds(x, y, radius=0.):
            # randomly choose a direction
            direction = random.choice(['left', 'right', 'forward'])
            if direction == 'left':
                # turn to the left
                pipuck.epuck.set_motor_speeds(900, 1000)
            elif direction == 'right':
                # turn to the right
                pipuck.epuck.set_motor_speeds(1000, 900)
            else:
                # move forward
                pipuck.epuck.set_motor_speeds(1000, 1000)
            
        else:
            # Get the closest wall
            wall_index = closest_wall(x, y)
            # Get the angle to the wall
            angle_to_wall = get_angle_to_wall(wall_index, angle)
            print(f"Angle to wall: {angle_to_wall}")
            # Turn away from the wall
            # the smaller the angle, the slower the turn
            if angle_to_wall < 90:
                pipuck.epuck.set_motor_speeds(1000, 1000)
            elif angle_to_wall < 180:
                pipuck.epuck.set_motor_speeds(900, 1000)
            elif angle_to_wall < 270:
                pipuck.epuck.set_motor_speeds(1000, 900)
            else:
                pipuck.epuck.set_motor_speeds(1000, 1000)
    else:
        # If no position data, stop the robot
        pipuck.epuck.set_motor_speeds(0, 0)
    
    time.sleep(0.3)
	
    
# Stop the MQTT client loop
pipuck.epuck.set_motor_speeds(0,0)
client.loop_stop()  
