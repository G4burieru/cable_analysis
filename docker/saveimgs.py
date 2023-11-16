from ifm3dpy.device import O3R
from ifm3dpy.framegrabber import FrameGrabber, buffer_id
import cv2
import asyncio
import time
import os

distance = 0.0
close = False
start_time = 0.0


def callback(self):
    rgb = cv2.imdecode(self.get_buffer(buffer_id.JPEG_IMAGE), cv2.IMREAD_UNCHANGED)
    timestr = time.strftime("images/%Y%m%d-%H%M%s.jpg")
    if(time.time() < start_time + 29):
        cv2.imwrite(timestr, rgb)

def callback2(frame):
    dist = frame.get_buffer(buffer_id.RADIAL_DISTANCE_IMAGE)
    (width, height) = dist.shape
    global distance
    global close
    global start_time
    distance = (dist[width//2,height//2])
    if(distance<=0.8 and distance>0 and close == False):
        close = True
        start_time = time.time()
    elif(distance==0 and close == True):
        close = False

# Initialize the objects
o3r = O3R()
fg = FrameGrabber(o3r, pcic_port=50010)
fg2 = FrameGrabber(o3r, pcic_port=50012)

# Change port to RUN state
config=o3r.get()
config["ports"]["port0"]["state"]= "RUN"
config["ports"]["port2"]["state"]= "RUN"
config["ports"]["port2"]["mode"]= "standard_range4m"
o3r.set(config)

# Register a callback and start streaming frames
fg2.start([buffer_id.NORM_AMPLITUDE_IMAGE,buffer_id.RADIAL_DISTANCE_IMAGE,buffer_id.XYZ])
fg2.on_new_frame(callback2)
fg.start([buffer_id.JPEG_IMAGE])
fg.on_new_frame(callback)

time.sleep(999999)
# Stop the streaming
fg.stop()
