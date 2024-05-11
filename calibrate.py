from STservo_sdk import *
port_handler = PortHandler(tx_pin=0, rx_pin=1, baudrate=1000000)
import time
servo_controller = scscl(port_handler)
offsets = [0]*13
while True:
    id = input("Enter ID: ")
    if id == "q":
        break
    else:
        while True:
            offset = input("Enter offset: ")
            if offset == "q":
                break
            else:
                servo_controller.WritePos(int(id), 512 + int(offset), 10, 10000)
                offsets[int(id)] = int(offset)
print(offsets)
# leg_configuration = {
#     "front_right": {"hip": 3, "upper": 2, "lower": 1},
#     "front_left": {"hip": 6, "upper": 5, "lower": 4},
#     "back_left": {"hip": 11, "upper": 10, "lower": 7},
#     "back_right": {"hip": 12, "upper": 8, "lower": 9}
# }

# offsets = [0, -10, 0, 20, 20, 10, 0, -10, 20, -5, 25, -10, 20]
# servo_offsets = {leg: {} for leg in leg_configuration}

# # # Assigning the offsets to each joint based on the servo IDs
# # for leg, joints in leg_configuration.items():
# #     for joint, id in joints.items():
# #         servo_offsets[leg][joint] = offsets[id]  # Directly using the servo ID as the index

# print(servo_offsets)

# print(servo_offsets)