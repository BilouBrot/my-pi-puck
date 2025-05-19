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
    return None, None, None

# battery = pipuck.get_battery_state()
# print(f"Charging?: {battery[0]}, Battery Voltage: {battery[1]}V, Battery percentage: {battery[2]}%")

for i in range(99999):
    # TODO: Do your stuff here
    # Print the updated dictionary
    if new_message[0]:
        pipuck.set_leds_rgb(red = True, green = False, blue = False)
        new_message[0] = False
    else:
        pipuck.set_leds_rgb(red = False, green = False, blue = False)
    # Get the current position of the robot
    x, y, angle = get_position()
    # drive randomly
    if x is not None and y is not None:
        
        collision = collsion_detected(x, y, radius=0.1)
        
        if collision[0]:
            print(f"Collision detected at position: {x}, {y}")
            if collision[1] is not None:
                # send message "Hello" to topic robot/<key>
                client.publish(f"robot/{collision[1]}", "Hello")
                print(f"Collision with robot {collision[1]} detected!")
            # turn to the left
            desired_angle = (angle + 180) % 360
            print(f"Desired angle: {desired_angle}, Current angle: {angle}")
            current_angle = angle
            while current_angle > desired_angle + 15 or current_angle < desired_angle - 15:
                pipuck.epuck.set_motor_speeds(-speed, speed)
                time.sleep(0.2)
                # get the new position
                x, y, current_angle = get_position()
                print(f"Current angle: {current_angle}")
            # move forward
            x, y, angle = get_position()
            while collsion_detected(x, y, radius=0.1)[0]:
                pipuck.epuck.set_motor_speeds(speed, speed)
                time.sleep(0.2)
                x, y, angle = get_position()
            time.sleep(0.5)
            
        else:
            if i % 50 == 0:
                # turn to the left
                pipuck.epuck.set_motor_speeds(-speed, speed)
            elif i % 50 == 25:
                # turn to the right
                pipuck.epuck.set_motor_speeds(speed, -speed)
            else:
                # move forward
                pipuck.epuck.set_motor_speeds(speed, speed)
    else:
        # If no position data, stop the robot
        pipuck.epuck.set_motor_speeds(0, 0)
    
    time.sleep(0.3)
	
    
# Stop the MQTT client loop
pipuck.epuck.set_motor_speeds(0,0)
client.loop_stop()  
