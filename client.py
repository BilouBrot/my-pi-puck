import paho.mqtt.client as mqtt
import json
import time
from pipuck.pipuck import PiPuck

# Define variables and callbacks
Broker = "192.168.178.56"  # Replace with your broker address
Port = 1883 # standard MQTT port
pi_puck_id = '16'
x_bound = 2.0
y_bound = 1.0

puck_dict = {}

# function to handle connection
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe("robot_pos/all")
    client.subscribe(f"robot/{pi_puck_id}")

# function to handle incoming messages
def on_message(client, userdata, msg):
    global new_message
    try:
        data = json.loads(msg.payload.decode())
        if msg.topic == "robot_pos/all":
            # Check if the message is a dictionary
            puck_dict.update(data)
        elif msg.topic == f"robot/{pi_puck_id}":
            new_message += 1
    except json.JSONDecodeError:
        print(f'invalid json: {msg.payload}')

# Initialize MQTT client
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(Broker, Port, 60)

client.loop_start() # Start listening loop in separate thread

# Initialize the PiPuck
pipuck = PiPuck(epuck_version=1)

# Set the robot's speed, e.g. with
# pipuck.epuck.set_motor_speeds(1000,-1000)

def check_bounds(x, y, radius = 0.0):
    if x < 0 + radius or x > x_bound - radius:
        return False
    if y < 0 + radius or y > y_bound - radius:
        return False
    return True

def collsion_detected(x, y, radius = 0.0):
    # Check if the robot is within the bounds of the arena
    if x < 0 + radius or x > x_bound - radius:
        return True
    if y < 0 + radius or y > y_bound - radius:
        return True
    # Check for collision with other robots
    for key, value in puck_dict.items():
        if key != pi_puck_id:
            pos = value.get('position')
            if pos:
                other_x = pos[0]
                other_y = pos[1]
                distance = ((x - other_x) ** 2 + (y - other_y) ** 2) ** 0.5
                if distance < radius:
                    return True
    return False

def get_position():
    data = puck_dict.get(pi_puck_id)
    if data:
        pos = data.get('position')
        if pos:
            x = pos[0]
            y = pos[1]
            return x, y
    else:
        print(f"No data for PiPuck ID: {pi_puck_id}")
    return None, None
    

for i in range(5000):
    # TODO: Do your stuff here
    # Print the updated dictionary
    print(f"Updated puck_dict: {puck_dict}")
    pipuck.epuck.set_inner_leds(True, True)
    # Get the current position of the robot
    x, y = get_position()
    print(f"Current position: {x}, {y}")
    # drive randomly
    if x is not None and y is not None:
        if collsion_detected(x, y, radius=0.05):
            # turn to the left
            pipuck.epuck.set_motor_speeds(-1000, 1000)
            time.sleep(0.5)
            # move forward
            pipuck.epuck.set_motor_speeds(1000, 1000)
            
        else:
            if i % 100 == 0:
                # turn to the left
                pipuck.epuck.set_motor_speeds(-1000, 1000)
                time.sleep(0.5)
            if i % 100 == 50:
                # turn to the right
                pipuck.epuck.set_motor_speeds(1000, -1000)
                time.sleep(0.5)
            pipuck.epuck.set_motor_speeds(1000, 1000)
    else:
        # If no position data, stop the robot
        pipuck.epuck.set_motor_speeds(0, 0)
    
    time.sleep(0.3)
	
    
# Stop the MQTT client loop
pipuck.epuck.set_motor_speeds(0,0)
client.loop_stop()  
