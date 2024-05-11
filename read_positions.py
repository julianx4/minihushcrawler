from STservo_sdk import *
port_handler = PortHandler(tx_pin=0, rx_pin=1, baudrate=1000000)
import time
servo_controller = scscl(port_handler)
positions = {}
offsets = {}
servo_list = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]
while True:
    for id in servo_list:
        position, _, _ = servo_controller.ReadPos(id)
        positions[id] = position
        offsets[id] = position - 512
    time.sleep(0.2)
    print(positions)
    print(offsets)


positions = {10: 539, 1: 496, 2: 501, 3: 529, 4: 538, 5: 518, 6: 524, 7: 505, 8: 523, 9: 513}

for id, pos in position.items():
    offset = pos - 512
    print(id, offset)

# for id in servo_list:
#     target_pos = offsets[id]
#     servo_controller.WritePos(id, target_pos, 10, 10000)
