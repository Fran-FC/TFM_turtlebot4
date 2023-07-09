import os
from subprocess import Popen
import time 
import argparse
import signal

slam_algo_dict = {
    "ros2 run hector_mapping hector_mapping_node" : "hector",
    "ros2 launch turtlebot4_navigation slam.launch.py" : "slam_toolbox",
    "ros2 run slam_gmapping slam_gmapping" : "gmapping",
    "ros2 launch cartographer_ros turtlebot4_online.launch.py" : "cartographer",
}

map_save_command = "ros2 run nav2_map_server map_saver_cli -f {0} --occ 0.7 --free 0.3"

parser = argparse.ArgumentParser()
parser.add_argument("--bag_file", default= "bag/rosbag_imu/rosbag2_2023_06_14-12_21_15_0.db3",help="path to bag file")
parser.add_argument("--bag_rate", default= 1, help="Bag play speed rate")
parser.add_argument("--it", default=10, help="Num of iterations per slam algorithm")
args = parser.parse_args()

bag_info = os.popen(f"ros2 bag info {args.bag_file}").read()
bag_duration = bag_info.split('Duration:')[1].split('\n')[0].strip()[:-1]

for slam_command in slam_algo_dict.keys():
    os.system("ros2 daemon stop; ros2 daemon start")
    print("Beggining to execute %s" % slam_command)
    for i in range(int(args.it)):
        print("Iteration %d of %s" % (i, slam_command))
        slam_process = Popen(slam_command.split())
        bag_process = Popen(f"ros2 bag play {args.bag_file} -r {args.bag_rate}".split())

        play_duration = float(bag_duration)/float(args.bag_rate)
        
        print("Sleeping for %d seconds" % play_duration)
        time.sleep(play_duration)

        while bag_process.poll() == None:
            print("Waiting to finish bag...")
            time.sleep(0.1)

        print("Saving map")
        map_dir = "scripts/map_benchmark/"+slam_algo_dict[slam_command]+"/"
        map_relative_path = map_dir+slam_algo_dict[slam_command]+"_"+str(i)
        try:
            os.listdir(map_dir)
        except:
            os.mkdir(map_dir)

        map_save_process = Popen(map_save_command.format(map_relative_path).split())

        while map_save_process.poll() == None:
            time.sleep(0.1)
        
        print("Terminating all processes")
        slam_process.terminate()

        os.system("ps aux | grep %s |  awk 'NR>1{print prev} {prev=$2}' | xargs -I {} kill {}" % slam_algo_dict[slam_command])
#        os.system("ps aux | grep %s |  awk 'NR>1{print prev} {prev=$2}' | xargs -I {} kill {}" % "robot_state_publisher")

        print("All processes terminated")
    
