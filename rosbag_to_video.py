import rosbag
import cv2
import numpy as np
import os
from sensor_msgs.msg import CompressedImage
import rospy

def convert_rosbag_to_video(bag_file, image_topic, output_video_file, fps=30):
    # Create a directory to store extracted images
    image_folder = 'extracted_images'
    os.makedirs(image_folder, exist_ok=True)

    # Read the bag file
    with rosbag.Bag(bag_file, 'r') as bag:
        print(f"Reading from bag file: {bag_file}")
        image_count = 0

        for topic, msg, t in bag.read_messages(topics=[image_topic]):
            if topic == image_topic:
                # Decode the compressed image
                np_arr = np.frombuffer(msg.data, np.uint8)
                img_array = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                if img_array is None:
                    print(f"Failed to decode image at index {image_count}. Skipping...")
                    continue

                # Save the image to the folder
                image_filename = os.path.join(image_folder, f'image_{image_count:04d}.jpg')
                cv2.imwrite(image_filename, img_array)
                image_count += 1

    print(f"Extracted {image_count} images.")

    if image_count == 0:
        print("No images extracted. Please check the topic name.")
        return

    # Create a video from the extracted images
    images = [os.path.join(image_folder, f'image_{i:04d}.jpg') for i in range(image_count)]
    
    # Determine the width and height from the first image
    frame = cv2.imread(images[0])
    height, width, layers = frame.shape

    # Define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Using mp4 codec
    video = cv2.VideoWriter(output_video_file, fourcc, fps, (width, height))

    for image in images:
        video.write(cv2.imread(image))

    video.release()
    print(f"Video saved as {output_video_file}")

if __name__ == '__main__':
    bag_file = 'run1.bag'  # Replace with your ROS bag file
    image_topic = '/duckie2/camera_node/image/compressed'  # Use the correct image topic
    output_video_file = 'output_video.mp4'  # Desired output video file name

    convert_rosbag_to_video(bag_file, image_topic, output_video_file)
