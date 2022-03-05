import os
import numpy as np
import socket

import sys
import torch
from torch.autograd import Variable
from torch.utils.data import Dataset, DataLoader
from model import HED
from PIL import Image
# from cStringIO import StringIO
print("Started image")
# create instance of HED model
net = HED(pretrained=False)
net.cuda()
print ("Loading network")
# load the weights for the model
net.load_state_dict(torch.load('/volume/HED.pth'))



input_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
output_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)


try:
    output_socket.connect(os.path.join("/volume", "output_socket"))
    # output_socket.connect("output_socket")
except socket.error as msg:
    print('Connect failed in docker. Error Code : ' + str(msg))
    sys.exit()


# if os.path.exists("input_socket"):
#     os.remove("input_socket")
# try:
#     input_socket.bind("input_socket")
if os.path.exists(os.path.join("/volume", "input_socket")):
    os.remove(os.path.join("/volume", "input_socket"))
try:
    input_socket.bind(os.path.join("/volume", "input_socket"))
except socket.error as msg:
    print('Bind failed in docker. Error Code : ' + str(msg))
    sys.exit()


input_socket.listen(1)


def grayTrans(img):
    """ Converts from pytorch tensor to numpy image
    """ 
    img = img.numpy()[0][0]*255.0
    img = (img).astype(np.uint8)
    return img

conn, addr = input_socket.accept()

buff = []
buffer_length = 0

while True:
    # data = conn.recv(482040)
    data = []

    try:
        data = conn.recv(300000)
    except socket.error as err:
        print("Receive failed in docker: ", str(err))
    # if len(data) < 245760*2*2:
    if len(data) < 482040:
        if len(data) > 0:
            buff.append(data)
            buffer_length += len(data)

        if buffer_length == 482040:
        # if buffer_length == 245760*2*2:
            data = b''.join(buff)
            buff = []
            buffer_length = 0
        else:
            continue

    image = np.fromstring(data, dtype=np.uint8)
    image = image.reshape((412, 390, 3))
    


    # tmp = image.copy()
    # image[:, :, 0] = tmp[:, :, 2]
    # image[:, :, 1] = tmp[:, :, 1]
    # image[:, :, 2] = tmp[:, :, 0]

    

    # image = image.reshape((256*2, 320*2, 3))
    image = image.astype(np.float32)
    image = image/255.0
    # img = Image.fromarray(image, 'RGB')
    # img.save('/volume/test2.png')

    image -= np.array((0.485, 0.456, 0.406))
    image /= np.array((0.229, 0.224, 0.225))
    image = image.transpose((2,0,1))

    image = image.reshape((1, 3, 412, 390))
    # image = image.reshape((1, 3, 256*2, 320*2))
    image = torch.from_numpy(image)

    # orig = np.zeros((image.shape[2], image.shape[3], 3), dtype=np.uint8)
    # orig[:,:,0] = (image.numpy()[0][2]*255).astype(np.uint8)
    # orig[:,:,1] = (image.numpy()[0][1]*255).astype(np.uint8)
    # orig[:,:,2] = (image.numpy()[0][0]*255).astype(np.uint8)

    
    # print("Saved")
    image = Variable(image.cuda())
    s1,s2,s3,s4,s5,s6 = net.forward(image)
    output = grayTrans(s6.data.cpu())
    

    # f = StringIO()
    # np.savez_compressed(f,frame=output)
    # f.seek(0)
    # output_string = f.read()
    try:
        # output_socket.sendall(output_string)
        output_socket.sendall(output.tostring())
    except socket.error as msg:
        print("Send failed in docker: ", str(msg))
        input_socket.close()
        output_socket.close()
