import cv2
import numpy as np
import os
import pyrealsense2 as rs
import time
from datetime import datetime


# Set the directory to save the dataset
dataset_directory = "images"

# Create the dataset directory if it doesn't exist
if not os.path.exists(dataset_directory):
    os.makedirs(dataset_directory)

# Configure RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)


# Start streaming
pipeline.start(config)

try:
    frame_count = 0
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        current_datetime = datetime.now()
        name = current_datetime.strftime("%Y-%m-%d-%H-%M-%S")


        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Save color images
        color_filename = os.path.join(dataset_directory, f"color_{name}.png")

         # Get the distance at a specific pixel (e.g., center of the image)
        y, x = depth_image.shape[:2]
        center_pixel_distance = depth_frame.get_distance(x // 2, y // 2)

        if center_pixel_distance <= 0.8 and center_pixel_distance != 0:

            cv2.imwrite(color_filename, color_image)

            print(f"Saved frame {frame_count}")

            frame_count += 1
            time.sleep(1)

except KeyboardInterrupt:
    pass
finally:
    # Stop streaming
    pipeline.stop()
