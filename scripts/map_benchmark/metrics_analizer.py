import cv2 as cv
import numpy as np
import argparse
import pandas as pd
import glob 
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser()
parser.add_argument("--input_csv", default="mem_cpu_usage.csv", help="Input of cpu and mem usage stats")
args = parser.parse_args()

occupied_value = 0
free_value = 254
unknown_value = 205

def get_occupied_proportion(image):
    image = image.astype(float)
    
    occupied_proportion = np.sum(image == occupied_value) / image.size
    return occupied_proportion

def get_num_corners(image):
    _, binary_image = cv.threshold(image, unknown_value, free_value, cv.THRESH_BINARY)#remove unknown values
    image = np.float32(binary_image)
    # Apply corner detection
    dst = cv.cornerHarris(image, blockSize=3, ksize=3, k=0.04)

    # Threshold the corner response
    threshold = 0.01 * dst.max()
    corners = np.where(dst > threshold)

    return len(corners[0])

def get_num_enclosed_areas(image):
    _, binary_image = cv.threshold(image, unknown_value, free_value, cv.THRESH_BINARY)#remove unknown values
    num_labels, _ = cv.connectedComponents(binary_image)

    return num_labels - 1

def get_map_metrics(slam_algorithms):
    algo_map_root_dir = "scripts/map_benchmark/"

    arr_occ_proportions = []
    arr_num_corners     = []
    arr_encl_areas      = []
    for slam_algo in slam_algorithms:
        pgm_files = glob.glob("%s%s/*.pgm" % (algo_map_root_dir, slam_algo))

        algo_occ  = []
        algo_corn = []
        algo_encl = []
        for pgm in pgm_files:
            image = cv.imread(pgm, -1)

            algo_occ.append(get_occupied_proportion(image))
            algo_corn.append(get_num_corners(image))
            algo_encl.append(get_num_enclosed_areas(image))

        arr_occ_proportions.append(algo_occ)
        arr_num_corners.append(algo_corn)
        arr_encl_areas.append(algo_encl)


    fig, ax = plt.subplots()
    ax.boxplot(arr_occ_proportions)
    ax.set_xticklabels(slam_algorithms)
    plt.savefig("occupacy_proportion.png")
    
    fig, ax = plt.subplots()
    ax.boxplot(arr_num_corners)
    ax.set_xticklabels(slam_algorithms)
    plt.savefig("num_corners.png")

    fig, ax = plt.subplots()
    ax.boxplot(arr_encl_areas)
    ax.set_xticklabels(slam_algorithms)
    plt.savefig("encl_areas.png")

    return

def get_perf_plot(df, metric):
    fig, ax = plt.subplots()
    for slam_algo in df.Algorithm.unique():
        algo_rows = df.loc[df.Algorithm == slam_algo]
        # Group by 'Timestamp' and calculate the mean and standard deviation of CPU%
        grouped_df = algo_rows.groupby('Timestamp')[metric].agg(['mean', 'std']).reset_index()

        # Extract the required data
        timestamps = grouped_df['Timestamp'].to_numpy()
        mean_percentages = grouped_df['mean'].to_numpy()
        std_percentages = grouped_df['std'].to_numpy()

        # Create the line plot

        # Plot the average CPU% curve
        ax.plot(timestamps, mean_percentages, label=slam_algo)

        # Fill the area between mean +/- std with shading
        ax.fill_between(timestamps, mean_percentages - std_percentages, mean_percentages + std_percentages, alpha=0.3)

        # Labels and title
        ax.set_xlabel('Timestamp')
        ax.set_ylabel(metric)
        ax.set_title('Variability of %s over Time' % metric)

        # Show the legend
        ax.legend()

        # Show the plot
    plt.savefig("%s_plot.png" % metric)

    return

if __name__ == "__main__":
    df = pd.read_csv(args.input_csv)
    get_perf_plot(df, "CPU%")
    get_perf_plot(df, "Mem%")

    get_map_metrics(df.Algorithm.unique())