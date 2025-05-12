import paho.mqtt.client as mqtt
import json
import time
from pipuck.pipuck import PiPuck

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
        # Print the updated dictionary
        print(f"Updated puck_dict: {puck_dict}")
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

def check_bounds(x, y):
    if x < 0 or x > x_bound:
        return False
    if y < 0 or y > y_bound:
        return False
    return True

def get_position():
    data = puck_dict.get('2')
    if data:
        pos = data.get('position')
        if pos:
            x = pos[0]
            y = pos[1]
            if check_bounds(x, y):
                return x, y
            else:
                print(f"Position out of bounds: {x}, {y}")
    else:
        print(f"No data for PiPuck ID: {pi_puck_id}")
    return None, None

for _ in range(1000):
    # TODO: Do your stuff here
    # Get the current position of the robot
    x, y = get_position()
    print(f"Current position: {x}, {y}")
    time.sleep(1)
	
    
# Stop the MQTT client loop
pipuck.epuck.set_motor_speeds(0,0)
client.loop_stop()  
