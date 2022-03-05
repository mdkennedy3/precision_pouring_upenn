import os
import numpy as np
import cv2
import sys
import time
import socket
from string import join


import rospkg
rospack = rospkg.RosPack()

HOST = ''
IN_PORT = 5000
OUT_PORT = 5001

# input_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# output_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# input_socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
# output_socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
input_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
output_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)

try:
    # output_socket.connect((HOST, OUT_PORT))
    output_socket.connect(os.path.join(rospack.get_path("liquid_level_detection_nn"), "docker", "output_socket"))
except socket.error as msg:
    print 'Connect failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
    sys.exit()




if os.path.exists(os.path.join(rospack.get_path("liquid_level_detection_nn"), "docker", "input_socket")):
    os.remove(os.path.join(rospack.get_path("liquid_level_detection_nn"), "docker", "input_socket"))
try:
    # input_socket.bind((HOST, IN_PORT))
    input_socket.bind(os.path.join(rospack.get_path("liquid_level_detection_nn"), "docker", "input_socket"))
except socket.error as msg:
    print 'Bind failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
    sys.exit()





print "Waiting for connection to input_socket"
input_socket.listen(1)



# output_socket.listen(1)

conn, addr = input_socket.accept()
print 'Connected with ' + str(addr)
# start_new_thread(callback ,(conn,))

buff = []
buffer_length = 0

while True:
    # data = input_socket.recv(4096)
    # data = conn.recv(245760)
    data = conn.recv(300000)
    if len(data) < 245760:
        if len(data) > 0:
            print "Incomplete image, ", len(data)
            # buff = buff + data
            buff.append(data)
            buffer_length += len(data)

        if buffer_length == 245760:
            print "Completed buffer"
            data = ''.join(buff)
            buff = []
            buffer_length = 0
        else:
            continue

    print "Got data:"
    image = np.fromstring(data, dtype=np.uint8)
    print image.shape
    image = image.reshape((256, 320, 3))
    print image.shape
    # print data
    # cv2.imshow('TEST', image)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows() 
    # output_socket.sendall('Tes345t\n\r\n')
    try:
        # input_socket.sendall('Tes345t\r\n')
        # output_socket.sendall(data)
        output_socket.sendall(image.tostring())
    except socket.error as msg:
        print "Send failed: ", msg
        input_socket.close()
        output_socket.close()
    # sys.exit()
    print "Sent data"