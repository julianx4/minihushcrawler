from STservo_sdk import *
port_handler = PortHandler(tx_pin=0, rx_pin=1, baudrate=1000000)
import time
servo_controller = scscl(port_handler)

new_id = input("Enter new ID: ")
servo_controller.changeServoID(254, int(new_id))
time.sleep(0.2)
servo_controller.WritePos(int(new_id), 200, 10, 10000)
time.sleep(0.8)
servo_controller.WritePos(int(new_id), 512, 10, 1000)
