import rclpy
from rclpy.node import Node
from rosbag2_py import StorageOptions, ConverterOptions, SequentialReader, SequentialWriter, TopicMetadata

import os
import yaml
from datetime import datetime

# Constants
BASE_ROSBAG_PATH = os.path.expanduser('~/ros_ws/src/rosbag2_tools/rosbag_analyze/rosbags')

class RosbagSplitter(Node):
    def __init__(self):
        super().__init__('rosbag_splitter_node')
        # Declare parameters with default values
        self.declare_parameter('bag_name', 'case_bag')  
        self.declare_parameter('output_uri','~/ros_ws/src/rosbag2_tools/rosbag_analyze/created_rosbags')
        self.declare_parameter('split_duration', 60)

        # Get parameters
        self.bag_name = self.get_parameter('bag_name').value
        self.input_uri = os.path.join(BASE_ROSBAG_PATH, self.bag_name)
        self.output_uri = self.get_parameter('output_uri').value
        self.split_duration = self.get_parameter('split_duration').value

        self.reader = None
        self.writer = None
        self.split_counter = 1
        self.start_time = None

    def rosbag_reader(self):
        #Read the rosbag file
        storage_options = StorageOptions(uri=self.input_uri, storage_id='sqlite3')
        converter_options = ConverterOptions("cdr", "cdr")
        self.reader = SequentialReader()
        self.reader.open(storage_options, converter_options)

    def rosbag_writer(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        #Write the new rosbag file
        output_uri = f"{self.output_uri}_{self.bag_name}_{self.split_counter}_{timestamp}" #Generate a new name for each splitted rosbag
        storage_options_write = StorageOptions(uri=output_uri, storage_id='sqlite3')
        self.writer = SequentialWriter()
        converter_options = ConverterOptions("cdr", "cdr")
        self.writer.open(storage_options_write, converter_options)

        #Get all topics and types in rosbag
        topic_types = self.reader.get_all_topics_and_types()
        for data in topic_types:
            self.create_topics(data.name, data.type) #TODO Only desired topics can be created as an option

    def create_topics(self, name, type, format='cdr'):
        #Create a new rosbag topic for each topic in the splitted rosbag
        topic = TopicMetadata(name=name, type=type, serialization_format=format)
        self.writer.create_topic(topic)

    def split_rosbags(self):
        while self.reader.has_next():
            topic, data, t = self.reader.read_next()

            if self.start_time is None:
                self.start_time = t #Get time as a nano seconds

            time_diff_seconds = (t - self.start_time) / 1e9 #Convert nanoseconds to seconds

            if time_diff_seconds >= self.split_duration:
                self.close_writer()
                self.split_counter += 1
                self.rosbag_writer()
                self.start_time = t #Reset the timer

            self.writer.write(topic, data, t) # Write a data to new rosbag

    def close_writer(self):
        #Save the new rosbag file
        del self.writer
        self.get_logger().info(f"Closing the writer for the {self.split_counter} rosbag.")

    def run(self):
        self.rosbag_reader()
        self.rosbag_writer()
        self.split_rosbags()

    def file_is_exists(self): #Check if the input file exists.
        if not os.path.exists(self.input_uri):
            self.get_logger().error(f"The file {self.input_uri} does not exist.")
            rclpy.shutdown()
        else:
            self.get_logger().info(f"The file {self.input_uri} exists.")
            return True

    def check_split_duration(self): #Check if split duration is smaller than total duration of rosbag
        metadata_path = os.path.join(self.input_uri, 'metadata.yaml')
        try:
            with open(metadata_path, 'r') as f:
                metadata = yaml.safe_load(f)
                
            total_duration_ns = metadata['rosbag2_bagfile_information']['duration']['nanoseconds']
            total_duration_sec = total_duration_ns / 1e9
            
            if total_duration_sec < self.split_duration:
                self.get_logger().error(f"Split duration ({self.split_duration}s) is larger than total duration ({total_duration_sec:.2f}s)")
                rclpy.shutdown()
            
            self.get_logger().info(f"Total duration: {total_duration_sec:.2f}s, Split duration: {self.split_duration}s")
            return True
        except FileNotFoundError:
            self.get_logger().error(f"Metadata file not found at {metadata_path}")
            rclpy.shutdown()
    
def main(args=None):
    rclpy.init(args=args)

    split = RosbagSplitter()
    if split.file_is_exists() and split.check_split_duration():
        split.run()
        split.close_writer()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
