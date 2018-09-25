import smbus
import time

bus = smbus.SMBus(1)

addr = 0x42

try:
    #set runtime parameters for receiving all gestures
    bus.write_i2c_block_data(addr, 0x10,[0x00, 0x00, 0xA2, 0x85, 0x00, 0x00, 0x00, 0x7F, 0x00, 0x00, 0x00, 0x7F, 0x00,0x00, 0x00])
    time.sleep(0.5)
except IOError:
    print Exception("I2C IO error!")

while True:
    try:
        data = bus.read_i2c_block_data(addr, 0x00, 26)  #Read the sensor data payload

        d_size = data.pop(0)  #strip the unnececary info
        d_flags = data.pop(0)
        d_seq = data.pop(0)
        d_ident = data.pop(0)


        d_configmask = data.pop(0) | data.pop(0) << 8
        d_timestamp = data.pop(0)  # counter
        d_sysinfo = data.pop(0)

        d_gesture = data[2:6]  # get the gesture codes

        # print d_gesture [0:2]
        if d_gesture[0:2] == [3, 16]:
            print "ew"
        elif d_gesture[0:2] == [2, 16]:
            print "we"
        elif d_gesture[0:2] == [5, 16]:
            print "ns"
        elif d_gesture[0:2] == [4, 16]:
            print "sn"
        time.sleep(0.01)

    except:
        print 'Error... exiting...'
        break
