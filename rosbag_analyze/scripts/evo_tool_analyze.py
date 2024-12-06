import os
import subprocess
import yaml

class RosbagOdometryAnalyzer:
    def __init__(self, input_dir, evo_tool_dir):
        #Initialize directory
        self.input_dir = input_dir
        self.evo_tool_dir = evo_tool_dir
        self.metrics_dir = os.path.join(evo_tool_dir, "metrics") #The metrics file must be in the location
        self.visualization_dir = os.path.join(evo_tool_dir, "visualization") #The visualization file must be in the location

    def get_subdirectories(self):
        return [d for d in os.listdir(self.input_dir) if os.path.isdir(os.path.join(self.input_dir, d))]

    def execute_command(self, command):
        #execute shell command 
        try:
            subprocess.run(command, check=True)
        except subprocess.CalledProcessError as e:
            print(f"Error running command: {' '.join(command)}")
            print(f"Error: {e}")

    def analyze_evo_traj(self, rosbags_dir):
        rosbags_dir_path = os.path.join(self.input_dir, rosbags_dir)
        traj_plot = os.path.join(self.visualization_dir, f"{rosbags_dir}_traj_plot.png")
        traj_results = os.path.join(self.metrics_dir, f"{rosbags_dir}_traj_results.json")
        
        # TODO CLI options can be dynamic
        evo_traj_command = [
            "evo_traj", "bag2", rosbags_dir_path,
            "--all_topics", "--plot_mode", "xy", 
            "--save_plot", traj_plot, "--save_table", traj_results,
            "--silent"
        ]
        print(f"Running evo_traj for {rosbags_dir}...")
        self.execute_command(evo_traj_command)
        print(f"  Traj Plot -> {traj_plot}, Traj Results -> {traj_results}")

    def analyze_evo_rpe(self, rosbags_dir):
        rosbags_dir_path = os.path.join(self.input_dir, rosbags_dir)
        rpe_plot = os.path.join(self.visualization_dir, f"{rosbags_dir}_rpe_plot.png")
        rpe_results = os.path.join(self.metrics_dir, f"{rosbags_dir}_rpe_results.zip")
        
        # TODO CLI options can be dynamic
        evo_rpe_command = [
            "evo_rpe", "bag2", rosbags_dir_path, "/casestudy/reference_pose", "/casestudy/predicted_pose",
            "--plot_mode", "xy", "--save_plot", rpe_plot,
            "--save_results", rpe_results,
            "--silent"
        ]
        print(f"Running evo_rpe for {rosbags_dir}...")
        self.execute_command(evo_rpe_command)
        print(f"  RPE Plot -> {rpe_plot}, RPE Results -> {rpe_results}")

    def analyze_evo_ape(self, rosbags_dir):
        rosbags_dir_path = os.path.join(self.input_dir, rosbags_dir)
        ape_plot = os.path.join(self.visualization_dir, f"{rosbags_dir}_ape_plot.png")
        ape_results = os.path.join(self.metrics_dir, f"{rosbags_dir}_ape_results.zip")
        
        # TODO CLI options can be dynamic
        evo_ape_command = [
            "evo_ape", "bag2", rosbags_dir_path, "/casestudy/reference_pose", "/casestudy/predicted_pose",
            "--plot_mode", "xy", "--save_plot", ape_plot,
            "--save_results", ape_results,
            "--silent"
        ]
        print(f"Running evo_ape for {rosbags_dir}...")
        self.execute_command(evo_ape_command)
        print(f"  APE Plot -> {ape_plot}, APE Results -> {ape_results}")

    def run(self, rosbags_dir):
        
        print(f"Starting analysis for {rosbags_dir}...\n")
        self.analyze_evo_traj(rosbags_dir)
        self.analyze_evo_rpe(rosbags_dir)
        self.analyze_evo_ape(rosbags_dir)
        print(f"\nAnalysis complete for {rosbags_dir}.\n")

    def check_rosbag_exists(self, rosbags_dir): #TODO check rosbag folder is exist in directory
        pass

    def check_message_count(self, rosbags_dir): #TODO Check message_count in metadata.yaml if its 0 skip
        pass

def main():
    # Default path
    default_input_dir = os.path.expanduser('~/ros_ws/src/rosbag2_tools/rosbag_analyze/created_rosbags') # get rosbag from this location, Only the folders to be analyzed need to be in the location.
    default_evo_tool_dir = os.path.expanduser('~/ros_ws/src/rosbag2_tools/rosbag_analyze/evo_tool') # save output to this location

    # Get a path information from the user if its a different location
    input_dir = input(f"Enter inputs directory (default: {default_input_dir}): ").strip() or default_input_dir
    evo_tool_dir = input(f"Enter results directory (default: {default_evo_tool_dir}): ").strip() or default_evo_tool_dir

    analyzer = RosbagOdometryAnalyzer(input_dir, evo_tool_dir)

    # Get the subdirectories
    rosbags_dirs = analyzer.get_subdirectories()
    for d in rosbags_dirs:
        analyzer.run(d)

if __name__ == "__main__":
    main()
