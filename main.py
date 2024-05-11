from STservo_sdk import *
import math
import time
from uosc.server import parse_message
from uosc.client import Client
import network
import socket
import wifi
import uasyncio as asyncio
import urandom

wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(wifi.WIFI2[0], wifi.WIFI2[1])

while not wlan.isconnected():
    print("Waiting for connection...")
    time.sleep(1)
print("Connected:", wlan.ifconfig())
# UDP Socket Setup
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', 9000))
sock.setblocking(False)

class OSCStuff:
    def __init__(self):
        self.default_sleep_time  = 0.013
        self.default_step_height = 6
        self.default_speed = 2500
        
        self.sleep_time = self.default_sleep_time
        self.step_length = 0
        self.side_step_length = 0
        self.step_height = 6
        self.forward_offset = 6
        self.forward = 0
        self.side_offset = 0
        self.speed = self.default_speed
        self.base_height_offset = 0
        self.turn = 0
        self.walking = False
        self.dancing = False
        self.base_stance = False
        self.time = 100
        
        self.lock = asyncio.Lock()

    def read_packet(self):
        try:
            pkt = sock.recv(200)
            if not pkt:
                return None, None
            else:
                addr, tags, value = parse_message(pkt)
                return addr, value
        except:
            return None, None

    async def update_messages(self):
        file = "setup.txt"
        while True:
            addr, message = self.read_packet()
            if addr:
                # async with self.lock:
                if addr == "/data":
                    #self.step_length = int((message[1] - 0.5) * 40)
                    self.forward = (message[1] - 0.5) * 2
                    self.side_step_length = -(int((message[0] - 0.5) * 20))
                    self.turn = -(message[2] - 0.5)
                    self.walking = int(message[3])
                    self.forward_offset = int((message[4]-0.5)*20)
                    self.step_height = int(message[5]*20)
                    self.speed = int(message[6] * 3100)
                    self.side_offset = int((message[7]-0.5)*20)
                    self.base_height_offset = int((message[8]-0.5)*20)
                    self.offset1 = message[9]
                    self.offset2 = message[10]
                    self.step_length = int(message[11] * 40)
                    print(self.offset1, self.offset2)
                if addr == "/sleep_time":
                    self.sleep_time = message[0] * self.default_sleep_time * 10
                if addr == "/save":
                    with open(file, "w") as f:
                        f.write(f"{self.speed}\n{self.forward_offset}\n{self.step_height}\n{self.sleep_time}")
            await asyncio.sleep_ms(50)

class Robot:
    def __init__(self, messages, base_y = -30):
        self.port_handler = PortHandler(tx_pin=0, rx_pin=1, baudrate=1000000)
        self.servo_controller = scscl(self.port_handler)
        self.messages = messages
        self.servo_time = 100

        self.base_Y = base_y
        self.baseheight = 80

        self.servo_offsets = {12: 26, 1: -15, 2: -11, 3: 13, 4: 18, 5: 0, 6: 4, 7: 30, 8: -1, 9: -7, 10: 23, 11: -16}

        self.leg_configuration = {
            "front_right": {"hip": 3, "upper": 2, "lower": 1},
            "front_left": {"hip": 6, "upper": 5, "lower": 4},
            "back_left": {"hip": 11, "upper": 10, "lower": 7},
            "back_right": {"hip": 12, "upper": 8, "lower": 9}
        }

        self.directions = {
            "front_right": {"hip": -1, "upper": 1, "lower": -1},
            "front_left": {"hip": 1, "upper": -1, "lower": 1},
            "back_left": {"hip": -1, "upper": -1, "lower": 1},
            "back_right": {"hip": 1, "upper": 1, "lower": -1}
        }

        self.movement_limits = {
            "hip": 24,
            "upper": 70,
            "lower": 140,
        }
    
        self.servo_positions = {
            "front_left": {"hip": 0, "upper": 0, "lower": 0},
            "front_right": {"hip": 0, "upper": 0, "lower": 0},
            "back_left": {"hip": 0, "upper": 0, "lower": 0},
            "back_right": {"hip": 0, "upper": 0, "lower": 0},
        }
        self.servo_target_positions = {}
        self.init_position()
    
    def init_position(self):
        self.move_leg_to_coordinate("back_right", [self.messages.forward_offset, self.base_Y + self.messages.side_offset, -self.baseheight + self.messages.base_height_offset], 100, 10000)
        self.move_leg_to_coordinate("back_left", [self.messages.forward_offset, self.base_Y - self.messages.side_offset, -self.baseheight + self.messages.base_height_offset], 100, 10000)
        self.move_leg_to_coordinate("front_right", [self.messages.forward_offset, self.base_Y + self.messages.side_offset, -self.baseheight + self.messages.base_height_offset],100, 10000)
        self.move_leg_to_coordinate("front_left", [self.messages.forward_offset, self.base_Y - self.messages.side_offset, -self.baseheight + self.messages.base_height_offset], 100, 10000)
        self.execute_movement()

    def checkdomain(self, D):
            if D > 1 or D < -1:
                print("____OUT OF DOMAIN____")
                if D > 1: 
                    D = 0.99
                    return D
                elif D < -1:
                    D = -0.99
                    return D
            else:
                return D
            
    def _inverse_kinematics(self, coord):
        coxa = 45
        femur = 45
        tibia = 47

        D = (coord[1]**2 + (-coord[2])**2 - coxa**2 + (-coord[0])**2 - femur**2 - tibia**2) / (2 * tibia * femur)
        D = self.checkdomain(D)
        gamma = math.atan2(-math.sqrt(1 - D**2), D)
        tetta = -math.atan2(coord[2], coord[1]) - math.atan2(math.sqrt(coord[1]**2 + (-coord[2])**2 - coxa**2), -coxa)
        alpha = math.atan2(-coord[0], math.sqrt(coord[1]**2 + (-coord[2])**2 - coxa**2)) - math.atan2(tibia * math.sin(gamma), femur + tibia * math.cos(gamma))
        gamma = math.degrees(gamma)
        tetta = math.degrees(tetta)
        alpha = math.degrees(alpha)

        angles = {"hip":-tetta, "upper":alpha, "lower":gamma}
        return angles

    def get_position_from_degrees(self, degrees, leg, joint):
        # Check and apply the movement limits based on servo type
        if degrees > self.movement_limits[joint]:
            print("Exceeding movement limits", degrees, joint)
            degrees = self.movement_limits[joint]
        elif degrees < -self.movement_limits[joint]:
            degrees = -self.movement_limits[joint]
            print("Exceeding movement limits", degrees, joint)

        target_position = int(1024 / 2 + self.directions[leg][joint] * degrees / 300 * 1024) + self.servo_offsets[self.leg_configuration[leg][joint]]
        return target_position
    
    def move_leg_to_coordinate(self, leg, coord, time, speed):
        angles = self._inverse_kinematics(coord)
        self.set_servos_degrees(leg, angles, time, speed)
        
    def set_servos_degrees(self, leg, angles, _time=100, base_speed=2800):
        max_distance = 0
        distances = {}

        # Calculate the distance each servo needs to move
        for joint, target_angle in angles.items():
            current_angle = self.servo_positions[leg][joint]
            distance = abs(target_angle - current_angle)
            distances[joint] = distance
            if distance > max_distance:
                max_distance = distance

        # If max_distance is 0, avoid division by zero later on
        if max_distance == 0:
            max_distance = 1

        # Set speeds proportionally and move servos
        for joint, distance in distances.items():
            # Calculate proportional speed
            speed = int(base_speed * (distance / max_distance))
            if speed < 1:
                speed = 1  # Ensure minimum speed to avoid stagnation

            servo_id = self.leg_configuration[leg][joint]
            target_position = self.get_position_from_degrees(degrees=angles[joint], leg=leg, joint=joint)
            if servo_id == 0:
                continue
            self.servo_controller.RegWritePos(servo_id, target_position, _time, speed)
            self.servo_target_positions[servo_id] = target_position
        self.servo_positions[leg] = angles
    
    def execute_movement(self):
        self.servo_controller.RegAction()

    def update_cycle(self, leg, forward_offset, side_offset, base_height_offset, step_height, forward, side_step_length, turn):
        cycle = [
            [forward_offset, self.base_Y, -self.baseheight + base_height_offset],  # Starting position
            [forward_offset - forward, self.base_Y, -self.baseheight + base_height_offset],  # Move forward
            [forward_offset - forward, self.base_Y, -self.baseheight + step_height + base_height_offset],  # Lift up
            [forward_offset, self.base_Y, -self.baseheight + step_height + base_height_offset]  # Move back to start height
        ]
        turn_adjustment = turn * 30
        # side step logic
        if leg == "front_right" or leg == "back_right":
            cycle[1][1] += side_step_length
            cycle[2][1] += side_step_length 
        else:
            cycle[1][1] -= side_step_length
            cycle[2][1] -= side_step_length

        #turning logic
        if leg == "front_right":
            cycle[1][1] -= turn_adjustment
            cycle[2][1] -= turn_adjustment
        elif leg == "back_right":
            cycle[1][1] += turn_adjustment
            cycle[2][1] += turn_adjustment
        elif leg == "front_left":
            cycle[1][1] += turn_adjustment
            cycle[2][1] += turn_adjustment
        else: #back_left
            cycle[1][1] -= turn_adjustment
            cycle[2][1] -= turn_adjustment

        # side balance logic
        if leg == "front_right" or leg == "back_right":
            cycle[0][1] += side_offset
            cycle[1][1] += side_offset
            cycle[2][1] += side_offset
            cycle[3][1] += side_offset

        else:
            cycle[0][1] -= side_offset
            cycle[1][1] -= side_offset
            cycle[2][1] -= side_offset
            cycle[3][1] -= side_offset

        return cycle

    async def walk3(self, forward_offset, step_height, speed, side_offset, base_height_offset):
        leg_pair1 = ["front_right", "back_left"]
        leg_pair2 = ["front_left", "back_right"]

        cycle_index1 = 0
        cycle_index2 = 2
        time = self.servo_time

        for i in range(4):  
            for leg in leg_pair1:
                forward = self.messages.forward
                side_step_length = self.messages.side_step_length
                turn = self.messages.turn
                cycle = self.update_cycle(leg, forward_offset, side_offset, base_height_offset, step_height, forward, side_step_length, turn)
                coord = cycle[cycle_index1]
                self.move_leg_to_coordinate(leg, coord, time, speed)

            for leg in leg_pair2:
                forward = self.messages.forward
                side_step_length = self.messages.side_step_length
                turn = self.messages.turn
                cycle = self.update_cycle(leg, forward_offset, side_offset, base_height_offset, step_height, forward, side_step_length, turn)
                coord = cycle[cycle_index2]
                self.move_leg_to_coordinate(leg, coord, time, speed)
            self.execute_movement()
            await self.monitor_servo_positions()
            cycle_index1 = (cycle_index1 + 1) % len(cycle)
            cycle_index2 = (cycle_index2 + 1) % len(cycle)

    async def walk4(self):
        legs = ["front_right", "front_left", "back_right", "back_left"]
        offset1 = self.messages.offset1
        offset2 = self.messages.offset2
        if offset1 > 0.5:
            offsets = [0, math.pi/2, math.pi, 3*math.pi/2]
        else:
            offsets = [0, math.pi, math.pi, 0]
        #offsets = [0, math.pi * offset1, math.pi * offset2, math.pi * (offset1 + offset2)]

        diameter = 20
        x_prev = {leg: 0 for leg in legs}
        speed = self.messages.speed
        forward_offset = self.messages.forward_offset
        side_offset = self.messages.side_offset
        turn = self.messages.turn
        base_height_offset = self.messages.base_height_offset
        self.base_Y = -45 + (offset2 - 0.5) * 2 * 15
        base_coord = [forward_offset, self.base_Y, -self.baseheight + base_height_offset]
        forward = self.messages.forward
        step_height = self.messages.step_height
        step_length = self.messages.step_length
        side_step_length = self.messages.side_step_length

        # Loop through each leg, applying a phase offset
        for i, leg in enumerate(legs):
            offset = offsets[i]
            timestamp = time.ticks_ms() / 1000 * 10 + offset

            # Calculate new coordinates
            x = math.sin(timestamp) * forward * step_length
            y_adjustment = math.sin(timestamp)
            z = math.cos(timestamp) * step_height
    
            if z < 0:
                z = 0

            current_speed = speed if x >= x_prev[leg] else speed / 3
            x_prev[leg] = x

            coord = base_coord[:]
            coord[0] += x
            coord[2] += z

            #side step logic
            if leg == "front_right" or leg == "back_right":
                coord[1] += side_step_length * y_adjustment
            else:
                coord[1] -= side_step_length * y_adjustment

            # side balance logic
            if leg == "front_right" or leg == "back_right":
                coord[1] += side_offset
            else:
                coord[1] -= side_offset

            # turn logic
            turn_adjustment = turn * 30
            if leg == "front_right":
                coord[1] -= turn_adjustment * y_adjustment
            elif leg == "back_right":
                coord[1] += turn_adjustment * y_adjustment
            elif leg == "front_left":
                coord[1] += turn_adjustment * y_adjustment
            else: #back_left
                coord[1] -= turn_adjustment * y_adjustment          

            
            self.move_leg_to_coordinate(leg, coord, 100, current_speed)
        self.execute_movement()
        await asyncio.sleep_ms(10)

    async def monitor_servo_positions(self, pos_threshold=20):
        start_time = time.ticks_ms()
        while True:
            all_at_target = True
            for servo_id, target_pos in self.servo_target_positions.items():
                
                if servo_id is 0:
                    continue
                current_pos = self.servo_controller.ReadPos(servo_id)[0]
                time_elapsed = time.ticks_diff(time.ticks_ms(), start_time)
                print(time_elapsed)
                if abs(target_pos - current_pos) > pos_threshold:
                    all_at_target = False
                    break
            if all_at_target:
                break
            await asyncio.sleep_ms(5)




    async def move_servo_smoothly(self, servo_controller, servo_id, speed, target_pos):
        speeds = [speed] #, speed * 1, speed * 0.8, speed * 0.6]
        proportions = [1] #[0.8, 0.9, 0.95, 1]
        pos_threshold = 10

        current_pos = servo_controller.ReadPos(servo_id)[0]
        distance = target_pos - current_pos
        step_positions = [current_pos + int(distance * proportion) for proportion in proportions]

        for step_pos, speed in zip(step_positions, speeds):
            servo_controller.WritePos(servo_id, step_pos, 100, int(speed))
            while True:
                await asyncio.sleep(0.01)
                current_pos = servo_controller.ReadPos(servo_id)[0]
                if abs(step_pos - current_pos) <= pos_threshold:
                    break

async def main():
    messages = OSCStuff()
    robot = Robot(messages)
    update_message_task = asyncio.create_task(messages.update_messages())

    while True:
        
        async with messages.lock:
            speed = messages.speed
            step_height = messages.step_height
            side_offset = messages.side_offset
            forward_offset = messages.forward_offset
            base_height_offset = messages.base_height_offset
            if messages.walking:
                await robot.walk4()
                #await robot.walk3(forward_offset = forward_offset, step_height = step_height, speed = speed, side_offset = side_offset, base_height_offset = base_height_offset)
            else:
                robot.init_position()
                await asyncio.sleep_ms(20)

if __name__ == "__main__":
    asyncio.run(main())