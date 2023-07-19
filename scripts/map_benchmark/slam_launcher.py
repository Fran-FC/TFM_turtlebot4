import os
from subprocess import Popen
import time 
import argparse
import pandas as pd
import psutil
import threading

parser = argparse.ArgumentParser()
parser.add_argument("--bag_file", default= "bag/rosbag_imu/rosbag2_2023_06_14-12_21_15_0.db3",help="path to bag file")
parser.add_argument("--bag_rate", default= 1, help="Bag play speed rate")
parser.add_argument("--it", default=10, help="Num of iterations per slam algorithm")
parser.add_argument("--output_csv", default="mem_cpu_usage.csv", help="Output of cpu and mem usage stats")
args = parser.parse_args()

mem_cpu_data_columns = ["Algorithm", "Iteration", "Timestamp", "CPU%", "Mem%"]
mem_cpu_csv = args.output_csv

slam_algo_dict = {
    "ros2 run hector_mapping hector_mapping_node" : {
        "name": "hector",
        "occ_threshold": "0.696",
        "free_threshold": "0.6",
    },
    "ros2 launch turtlebot4_navigation slam.launch.py" : {
        "name": "slam_toolbox",
        "occ_threshold": "0.987",
        "free_threshold": "0.9",
    },
    "ros2 run slam_gmapping slam_gmapping" : {
        "name": "gmapping",
        "occ_threshold": "0.642",
        "free_threshold": "0.6",
    },
    "ros2 launch cartographer_ros turtlebot4_online.launch.py" : {
        "name": "cartographer",
        "occ_threshold": "0.2243", 
        "free_threshold": "0.22"
    }
}

map_save_command = "ros2 run nav2_map_server map_saver_cli -f %s --occ %s --free %s"

# get the rosbag file original duration in seconds 
bag_duration = float(os.popen("ros2 bag info %s"%args.bag_file).read().\
                     split('Duration:')[1].split('\n')[0].strip()[:-1])
# get real bag play duration
play_duration = bag_duration/float(args.bag_rate)

def measure_mem_cpu(algorithm, iteration):
    mem_cpu_data = []
    timer = 0
    ini = time.time()
    while timer < play_duration:
        timer = time.time() - ini

        cpu_usage = psutil.cpu_percent() 
        memory_usage = psutil.virtual_memory().percent
        mem_cpu_data.append([algorithm, iteration, int(timer), cpu_usage, memory_usage])

        time.sleep(play_duration/20)
    
    df = pd.DataFrame(mem_cpu_data, columns=mem_cpu_data_columns)

    print("Dataframe of algorithm %s and iteration %d done" % (algorithm, iteration))
    if mem_cpu_csv in os.listdir():
        df.to_csv(mem_cpu_csv, mode="a", header=False, index=False)
    else:
        df.to_csv(mem_cpu_csv, index=False)
    
    return

for slam_command in slam_algo_dict.keys():
    print("Beggining to execute %s" % slam_command)

    for i in range(int(args.it)):
        print("Iteration %d of %s" % (i, slam_command))

        slam_process = Popen(slam_command.split())
        bag_process = Popen(("ros2 bag play %s -r %s"%(args.bag_file, args.bag_rate)).split())
        
        print("Sleeping for %d seconds" % play_duration)

        # start measuring cpu and mem usage in parallel 
        th = threading.Thread(target=measure_mem_cpu, args=[slam_algo_dict[slam_command]["name"], i])
        th.start()

        time.sleep(play_duration)
        
        while bag_process.poll() == None:
            print("Waiting to finish bag...")
            time.sleep(0.5)

        print("Saving map")

        map_dir = "scripts/map_benchmark/"+slam_algo_dict[slam_command]["name"]+"/"
        map_relative_path = map_dir+slam_algo_dict[slam_command]["name"]+"_"+str(i)
        try:
            os.listdir(map_dir)
        except:
            os.mkdir(map_dir)

        map_command = map_save_command%(map_relative_path, 
                                        slam_algo_dict[slam_command]["occ_threshold"], 
                                        slam_algo_dict[slam_command]["free_threshold"])

        map_save_process = Popen(map_command.split())

        while map_save_process.poll() == None:
            print("Waiting to save map...")
            time.sleep(0.1)
        
        print("Terminating all processes")
        th.join()
        slam_process.terminate()
        os.system("ps aux | grep %s |  awk 'NR>1{print prev} {prev=$2}' | xargs -I {} kill {}" % slam_algo_dict[slam_command]["name"])
        os.system("ps aux | grep %s |  awk 'NR>1{print prev} {prev=$2}' | xargs -I {} kill {}" % "robot_state_publisher")

        print("All processes terminated")