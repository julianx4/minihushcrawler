from machine import I2C, Pin
import ustruct
import time

PERSON_SENSOR_I2C_ADDRESS = 0x62
PERSON_SENSOR_I2C_HEADER_FORMAT = "BBH"
PERSON_SENSOR_I2C_HEADER_BYTE_COUNT = ustruct.calcsize(PERSON_SENSOR_I2C_HEADER_FORMAT)

PERSON_SENSOR_FACE_FORMAT = "BBBBBBbB"
PERSON_SENSOR_FACE_BYTE_COUNT = ustruct.calcsize(PERSON_SENSOR_FACE_FORMAT)

PERSON_SENSOR_FACE_MAX = 4
PERSON_SENSOR_RESULT_FORMAT = PERSON_SENSOR_I2C_HEADER_FORMAT + "B" + PERSON_SENSOR_FACE_FORMAT * PERSON_SENSOR_FACE_MAX + "H"
PERSON_SENSOR_RESULT_BYTE_COUNT = ustruct.calcsize(PERSON_SENSOR_RESULT_FORMAT)

PERSON_SENSOR_DELAY = 0.2

class PersonSensor:
    def __init__(self, i2c):
        self.i2c = i2c

    def get_faces(self):
        try:
            read_bytes = self.i2c.readfrom(PERSON_SENSOR_I2C_ADDRESS, PERSON_SENSOR_RESULT_BYTE_COUNT)
        except OSError as error:
            print("No person sensor data found")
            print(error)
            time.sleep(PERSON_SENSOR_DELAY)
            return
        offset = 0
        (pad1, pad2, payload_bytes) = ustruct.unpack_from(PERSON_SENSOR_I2C_HEADER_FORMAT, read_bytes, offset)
        offset += PERSON_SENSOR_I2C_HEADER_BYTE_COUNT

        (num_faces,) = ustruct.unpack_from("B", read_bytes, offset)
        num_faces = int(num_faces)
        offset += 1

        faces = []
        for i in range(num_faces):
            (box_confidence, box_left, box_top, box_right, box_bottom, id_confidence, id, is_facing) = ustruct.unpack_from(PERSON_SENSOR_FACE_FORMAT, read_bytes, offset)
            offset += PERSON_SENSOR_FACE_BYTE_COUNT
            face = {
                "box_confidence": box_confidence,
                "box_left": box_left,
                "box_top": box_top,
                "box_right": box_right,
                "box_bottom": box_bottom,
                "id_confidence": id_confidence,
                "id": id,
                "is_facing": is_facing,
            }
            faces.append(face)
        checksum = ustruct.unpack_from("H", read_bytes, offset)
        return num_faces, faces
