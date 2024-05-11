
import time
from machine import I2C, Pin
powerpin_bno55 = Pin(14, Pin.OUT)
powerpin_bno55.value(1)
time.sleep(0.5)



import time
from bno055 import *
from personsensor import PersonSensor

i2c_gyro = I2C(id = 0, scl=Pin(13), sda=Pin(12))
i2c_camera = I2C(id = 1, scl=Pin(27), sda=Pin(26))

imu = BNO055(i2c_gyro)
personsensor = PersonSensor(i2c_camera)
while True:
    time.sleep(0.2)
    # if not calibrated:
    #     calibrated = imu.calibrated()
    #     print('Calibration required: sys {} gyro {} accel {} mag {}'.format(*imu.cal_status()))
    # #print('Temperature {}°C'.format(imu.temperature()))
    #print('Mag       x {:5.0f}    y {:5.0f}     z {:5.0f}'.format(*imu.mag()))
    #print('Gyro      x {:5.2f}    y {:5.2f}     z {:5.2f}'.format(*imu.gyro()))
    #print('Accel     x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*imu.accel()))
    #print('Lin acc.  x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*imu.lin_acc()))
    #print('Gravity   x {:5.1f}    y {:5.1f}     z {:5.1f}'.format(*imu.gravity()))
    print('Heading     {:4.0f} roll {:4.0f} pitch {:4.0f}'.format(*imu.euler()))
    print(personsensor.get_faces())