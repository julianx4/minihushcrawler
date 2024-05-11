from STservo_sdk import *
port_handler = PortHandler(tx_pin=0, rx_pin=1, baudrate=1000000)
import time
servo_controller = scscl(port_handler)

while True:
    new_id = input("Enter ID: ")
    servo_controller.WritePos(int(new_id), 512 - 40, 10, 10000)
    time.sleep(0.2)
    servo_controller.WritePos(int(new_id), 512 + 40, 10, 10000)
    time.sleep(0.2)
    servo_controller.WritePos(int(new_id), 512, 10, 1000)
