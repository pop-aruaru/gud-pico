import picamera2
import cv2
from picamera2 import Picamera2
from IPython.display import display, Image

camera = Picamera2()
video_config=camera.create_video_configuration()
transform=video_config['transform']
transform.vflip=True 
transform.hflip=True
camera.configure(video_config)
camera.start() # capture開始
img=camera.capture_array()
qvga=cv2.resize(img,(480,320))
qvga3=cv2.cvtColor(qvga,cv2.COLOR_BGR2RGBA)#色がおかしい 肌色が紫になるので。
with open('/dev/fb0', 'wb') as f:
    for i in range(0,1000):
        img=camera.capture_array()
        qvga=cv2.resize(img,(480,320))
        qvga3=cv2.cvtColor(qvga,cv2.COLOR_BGR2RGBA)#色がおかしい 肌色が紫になるので。
        # デバイスファイルへの書き込み
        f.write(qvga3)
        f.seek(0)

camera.stop()
camera.close()
print('end')
