import smbus
import time
import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO

SW_RESET_PIN = 17
SW_XFER_PIN = 27

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(SW_RESET_PIN, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(SW_XFER_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

x = 0.0
y = 0.0
z = 0.0

broker_address="127.0.0.1"

bus = smbus.SMBus(1)  # default on Pi B+ older models can be 0
addr = 0x42  # address MGC3130

def reset():
    GPIO.output(SW_RESET_PIN, GPIO.LOW)
    time.sleep(.1)
    GPIO.output(SW_RESET_PIN, GPIO.HIGH)
    time.sleep(.5) # Datasheet delay of 200ms plus change

def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    reset()


def mqtt_publish(x, y, z):
        message = str(x)+" "+ str(y)+" "+str(z)
        client.publish("xyz", message)
        print message

def setup():
    try:
        # set runtime parameters for receiving all gestures
        GPIO.setup(SW_XFER_PIN, GPIO.OUT, initial=GPIO.LOW)
        bus.write_i2c_block_data(addr,  0x10, [0x00, 0x00, 0xA2,   0x85, 0x00,    0x00, 0x00,   0b01111111, 0x00, 0x00, 0x00,  0b01111111, 0x00, 0x00, 0x00])
        GPIO.setup(SW_XFER_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        time.sleep(0.1)
    except IOError:
        print Exception("I2C IO error!")

def main():
    while True:
        if not GPIO.input(SW_XFER_PIN):
            GPIO.setup(SW_XFER_PIN, GPIO.OUT, initial=GPIO.LOW)
            try:
                data = bus.read_i2c_block_data(addr, 0x00, 26)  # Read the sensor data payload

                data_size = data.pop(0)  # strip the unnecessary info
                data_flags = data.pop(0)
                data_seq = data.pop(0)
                data_ident = data.pop(0)

                data_configmask = data.pop(0) | data.pop(0) << 8
                data_timestamp = data.pop(0)  # counter
                data_sysinfo = data.pop(0)

                data_xyz = data[12:20]

                x, y, z = (
                    round((data_xyz[1] << 8 | data_xyz[0]) / 65536.0, 4),
                    round((data_xyz[3] << 8 | data_xyz[2]) / 65536.0, 4),
                    round((data_xyz[5] << 8 | data_xyz[4]) / 65536.0, 4)
                )

                mqtt_publish(x, y, z)
                time.sleep(0.05)

            except:
                print 'Error... exiting...'
                break

            GPIO.setup(SW_XFER_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)


client = mqtt.Client()
client.connect(broker_address, 1883, 60)
client.on_connect = on_connect
setup()

main()

client.loop_forever()




