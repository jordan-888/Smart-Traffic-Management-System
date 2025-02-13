import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

TRIGB_1, ECHOB_1,TRIGB_2, ECHOB_2  = 14,2,3,4
TRIGA_1, ECHOA_1,TRIGA_2, ECHOA_2 = 27,17,22,10
TRIGC_1, ECHOC_1,TRIGC_2, ECHOC_2= 19,26,6,13
TRIGD_1, ECHOD_1,TRIGD_2, ECHOD_2= 0,5,9,11

RED_A,GREEN_A,YELLOW_A = 7,12,1
RED_B,GREEN_B,YELLOW_B = 23,15,18
RED_C,GREEN_C,YELLOW_C = 8,25,24
RED_D,GREEN_D,YELLOW_D = 21,16,20

GPIO.setup(TRIGA_1, GPIO.OUT)
GPIO.output(TRIGA_1, False)
GPIO.setup(TRIGA_2, GPIO.OUT)
GPIO.output(TRIGA_2, False)
GPIO.setup(TRIGB_1, GPIO.OUT)
GPIO.output(TRIGB_1, False)
GPIO.setup(TRIGB_2, GPIO.OUT)
GPIO.output(TRIGB_2, False)
GPIO.setup(TRIGC_1, GPIO.OUT)
GPIO.output(TRIGC_1, False)
GPIO.setup(TRIGC_2, GPIO.OUT)
GPIO.output(TRIGC_2, False)
GPIO.setup(TRIGD_1, GPIO.OUT)
GPIO.output(TRIGD_1, False)
GPIO.setup(TRIGD_2, GPIO.OUT)
GPIO.output(TRIGD_2, False)

GPIO.setup(RED_A, GPIO.OUT)
GPIO.setup(RED_B, GPIO.OUT)
GPIO.setup(YELLOW_A, GPIO.OUT)
GPIO.setup(YELLOW_B, GPIO.OUT)
GPIO.setup(GREEN_A, GPIO.OUT)
GPIO.setup(GREEN_B, GPIO.OUT)
GPIO.setup(RED_C, GPIO.OUT)
GPIO.setup(RED_D, GPIO.OUT)
GPIO.setup(YELLOW_C, GPIO.OUT)
GPIO.setup(YELLOW_D, GPIO.OUT)
GPIO.setup(GREEN_C, GPIO.OUT)
GPIO.setup(GREEN_D, GPIO.OUT)

GPIO.setup(ECHOA_1, GPIO.IN)
GPIO.setup(ECHOA_2, GPIO.IN)
GPIO.setup(ECHOB_1, GPIO.IN)
GPIO.setup(ECHOB_2, GPIO.IN)
GPIO.setup(ECHOC_1, GPIO.IN)
GPIO.setup(ECHOC_2, GPIO.IN)
GPIO.setup(ECHOD_1, GPIO.IN)
GPIO.setup(ECHOD_2, GPIO.IN)

def control_lights(red,yellow,green,state):
    GPIO.output(red, state == 'red')
    GPIO.output(green, state == 'green')
    GPIO.output(yellow, state == 'yellow')

MIN_DISTANCE = 3
MAX_DISTANCE = 15
STATIONARY_TIME = 3

def get_distance(TRIG,ECHO):
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    # Measure the time of the echo signal
    pulse_start = time.time()
    pulse_end = time.time()

    # Wait for the ECHO signal to go high
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()

    # Wait for the ECHO signal to go low
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    # Calculate distance in cm
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # Speed of sound: 34300 cm/s / 2
    print(distance)
    return round(distance, 2)

def detect_stationary_objects(trig,echo,n,m):
    object_count = 0
    last_distance= None
    i=0
    print("counting for lane:",n,'Sensor:',m)
    try:
        while i<=1:
            distance = get_distance(trig,echo)
            # Detect a new object if distance changes significantly
            if last_distance is None or abs(distance - last_distance) > 5:
                if MIN_DISTANCE < distance < MAX_DISTANCE:
                    object_count += 1
                    print(f"Object {object_count} detected at {distance} cm")
            
            last_distance = distance
            time.sleep(1)
            i+=1
    
    except KeyboardInterrupt:
        print("counting stopped")
            
    return object_count

def round_robin_traffic():
    try:
        while True:
            control_lights(RED_A,YELLOW_A,GREEN_A, 'red')
            control_lights(RED_B,YELLOW_B,GREEN_B, 'red')
            control_lights(RED_C,YELLOW_C,GREEN_C, 'red')
            control_lights(RED_D,YELLOW_D,GREEN_D, 'red')
            obj1 = detect_stationary_objects(TRIGA_1, ECHOA_1, 1,1) + detect_stationary_objects(TRIGA_2,ECHOA_2,1,2)  # Lane 1 object count
            obj2 = detect_stationary_objects(TRIGB_1, ECHOB_1, 2,1) + detect_stationary_objects(TRIGB_2,ECHOB_2,2,2)   # Lane 2 object count
            obj3 = detect_stationary_objects(TRIGC_1, ECHOC_1, 3,1) + detect_stationary_objects(TRIGC_2,ECHOC_2,3,2) # Lane 1 object count
            obj4 = detect_stationary_objects(TRIGD_1, ECHOD_1, 4,1) + detect_stationary_objects(TRIGD_2,ECHOD_2,4,2)

            total_time = 30 # Total cycle time in seconds
            total_objects = obj1 + obj2 + obj3 + obj4 if obj1+obj2+obj3+obj4>0 else 1

            # Calculate time allocation for each lane
            time_lane1 = (obj1 / total_objects) * total_time
            time_lane2 = (obj2 / total_objects) * total_time
            time_lane3 = (obj3 / total_objects) * total_time
            time_lane4 = (obj4 / total_objects) * total_time
            print("the vehicles in lane1:",obj1)
            print("the vehicles in lane2:",obj2)
            print("the vehicles in lane3:",obj3)
            print("the vehicles in lane4:",obj4)
            control_lights(RED_A,YELLOW_A,GREEN_A, 'yellow')
            control_lights(RED_B,YELLOW_B,GREEN_B, 'red')
            control_lights(RED_C,YELLOW_C,GREEN_C, 'red')
            control_lights(RED_D,YELLOW_D,GREEN_D, 'red')
            print("time for laneA:",time_lane1)
            print("time for laneB:",time_lane2)
            print("time for laneC:",time_lane3)
            print("time for laneD:",time_lane4)

            # Lane A green
            control_lights(RED_A,YELLOW_A,GREEN_A, 'green')
            time.sleep(time_lane1)
            control_lights(RED_A,YELLOW_A,GREEN_A, 'yellow')
            time.sleep(3)
            
            # Lane C green
            control_lights(RED_A,YELLOW_A,GREEN_A, 'red')
            control_lights(RED_C,YELLOW_C,GREEN_C, 'green')
            time.sleep(time_lane3)
            control_lights(RED_C,YELLOW_C,GREEN_C, 'yellow')
            time.sleep(3)
            
            # Lane D green
            control_lights(RED_C,YELLOW_C,GREEN_C, 'red')
            control_lights(RED_D,YELLOW_D,GREEN_D, 'green')
            time.sleep(time_lane4)
            control_lights(RED_D,YELLOW_D,GREEN_D, 'yellow')
            time.sleep(3)
            
            # Lane B green
            control_lights(RED_B,YELLOW_B,GREEN_B, 'green')
            control_lights(RED_D,YELLOW_D,GREEN_D, 'red')
            time.sleep(time_lane2)
            control_lights(RED_B,YELLOW_B,GREEN_B, 'yellow')
            time.sleep(3)

            control_lights(RED_A, YELLOW_A, GREEN_A, 'red')
            control_lights(RED_B, YELLOW_B, GREEN_B, 'red')
            control_lights(RED_C, YELLOW_C, GREEN_C, 'red')
            control_lights(RED_D, YELLOW_D, GREEN_D, 'red')
            
    except KeyboardInterrupt:
        print("Traffic system stopped.")
    finally:
        GPIO.cleanup()

round_robin_traffic()