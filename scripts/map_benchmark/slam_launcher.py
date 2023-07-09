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
args = parser.parse_args()

mem_cpu_data = []
mem_cpu_data_columns = ["Algorithm", "Iteration", "Timestamp", "CPU%", "Mem%"]
mem_cpu_csv = "mem_cpu_usage.csv"

slam_algo_dict = {
    "ros2 run hector_mapping hector_mapping_node" : "hector",
    "ros2 launch turtlebot4_navigation slam.launch.py" : "slam_toolbox",
    "ros2 run slam_gmapping slam_gmapping" : "gmapping",
    "ros2 launch cartographer_ros turtlebot4_online.launch.py" : "cartographer",
}

map_save_command = "ros2 run nav2_map_server map_saver_cli -f %s --occ 0.7 --free 0.3"

# get the rosbag file original duration in seconds 
bag_duration = float(os.popen("ros2 bag info %s"%args.bag_file).read().\
                     split('Duration:')[1].split('\n')[0].strip()[:-1])
# get real bag play duration
play_duration = bag_duration/float(args.bag_rate)

def measure_mem_cpu(algorithm, iteration):
    timer = 0
    ini = time.time()
    while timer < play_duration:
        cpu_usage = [psutil.cpu_percent() for _ in range(3)]
        memory_usage = psutil.virtual_memory().percent
        mem_cpu_data.append([algorithm, iteration, timer, cpu_usage, memory_usage])

        timer = time.time() - ini
        time.sleep(2)
    
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
        th = threading.Thread(target=measure_mem_cpu, args=[slam_algo_dict[slam_command], i])
        th.start()

        time.sleep(play_duration)
        
        while bag_process.poll() == None:
            print("Waiting to finish bag...")
            time.sleep(0.5)

        th.join()

        print("Saving map")

        map_dir = "scripts/map_benchmark/"+slam_algo_dict[slam_command]+"/"
        map_relative_path = map_dir+slam_algo_dict[slam_command]+"_"+str(i)
        try:
            os.listdir(map_dir)
        except:
            os.mkdir(map_dir)

        map_save_process = Popen((map_save_command%map_relative_path).split())

        while map_save_process.poll() == None:
            print("Waiting to save map...")
            time.sleep(0.1)
        
        print("Terminating all processes")

        slam_process.terminate()

        os.system("ps aux | grep %s |  awk 'NR>1{print prev} {prev=$2}' | xargs -I {} kill {}" % slam_algo_dict[slam_command])
#        os.system("ps aux | grep %s |  awk 'NR>1{print prev} {prev=$2}' | xargs -I {} kill {}" % "robot_state_publisher")

        print("All processes terminated")