#!/bin/python3
import picamera2
import cv2
from picamera2 import Picamera2
from IPython.display import display, Image
import os,sys
import getpass

print('sys.argv         : ', sys.argv)
print('type(sys.argv)   : ', type(sys.argv))
print('len(sys.argv)    : ', len(sys.argv))
if len(sys.argv)==1:
    fb_device = '/dev/fb0'
else:
    n=int(sys.argv[1])
    fb_device ='/dev/fb'+str(n)
if os.system('getent group video | grep -q "\b'+ getpass.getuser() +'\b"') == 1:
    # User not in video group
    print("This command will be run as root, please allow it in order to get framebuffer work!")
    os.system("sudo adduser " + getpass.getuser() + " video")
    print("Done! Now you can use framebuffer without root!")

print(f'fb_device:{fb_device}')
# Now user in video group
# Get screen info

# Get screen size
try:
    _ = open("/sys/class/graphics/fb0/virtual_size", "r")
    __ = _.read()
    screenx,screeny = [int(i) for i in __.split(",")]
    _.close()
    # Get bit per pixel
    _ = open("/sys/class/graphics/fb0/bits_per_pixel", "r")
    bpp = int(_.read()[:2])
    _.close()
except Error as e:
    print(e)
    print(f'fb_device:{fb_device}')
    print(f'{sys.argv[0]} 0..n')
    
print(f'screen x,y,bpp:{screenx},{screeny},{bpp}')

camera = Picamera2()
video_config=camera.create_video_configuration()
transform=video_config['transform']
transform.vflip=True 
transform.hflip=True
camera.configure(video_config)
camera.start() # capture開始
img=camera.capture_array()
qvga=cv2.resize(img,(screenx,screeny))
qvga3=cv2.cvtColor(qvga,cv2.COLOR_BGR2RGBA)#色がおかしい 肌色が紫になるので。
try:
    with open(fb_device, 'wb') as f:
        for i in range(0,1000):
            img=camera.capture_array()
            qvga=cv2.resize(img,(screenx,screeny))
            qvga3=cv2.cvtColor(qvga,cv2.COLOR_BGR2RGBA)#色がおかしい 肌色が紫になるので。
            # デバイスファイルへの書き込み
            f.write(qvga3)
            f.seek(0)
except PermissionError as e:
    print(e)
    print(f'fb_device:{fb_device}')
    print(f'{sys.argv[0]} 0..n')
    
camera.stop()
camera.close()
print('end')
