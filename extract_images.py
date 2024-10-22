#!/usr/bin/env python

import rosbag
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import os

# Define the bag file and output directory
bag_file = 'vivekdata.bag'
output_dir = 'extracted_images'

# Create output directory if it doesn't exist
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# Open the bag file
with rosbag.Bag(bag_file, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=['/duckie2/camera_node/image/compressed']):
        # Convert the CompressedImage message to a numpy array
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        # Generate a filename for each image
        image_filename = os.path.join(output_dir, f'image_{t.to_nsec()}.jpg')
        
        # Save the image
        cv2.imwrite(image_filename, image)
        print(f'Saved: {image_filename}')

print("Image extraction complete.")