import paramiko, re
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import argparse
import subprocess



parser = argparse.ArgumentParser()
parser.add_argument("--remote_connection", default=False, help="Measure wifi connectivity from a remote machine")
parser.add_argument("--remote_ip", default="10.42.0.21")
parser.add_argument("--remote_username", default="ubuntu")
parser.add_argument("--interface", default="wlo1")
args = parser.parse_args()

if args.remote_connection:
    # SSH connection details
    host = '10.42.0.21'
    port = 22
    username = 'ubuntu'

    # Create an SSH client
    client = paramiko.SSHClient()
    client.load_system_host_keys()
    # Connect to the remote server using the identity file
    client.connect(hostname=host, username=username)
    

# Remote command
command = "iwconfig {} | grep -i quality".format(args.interface)


max_length = 500
# Create a deque with a maximum size of 10
numbers1 = deque([], maxlen=max_length)
# numbers2 = deque([], maxlen=10)

# Create a figure and an axis for plotting
fig, ax = plt.subplots()

# Set the y-axis limit
ax.set_ylim(0, 75)

# Create an empty line object for the plot
line1, = ax.plot([], [], lw=2, label="Quality (x/70)")
# line2, = ax.plot([], [], lw=2, label="Level (-dBm)")

# Set up the legend
ax.legend(loc='upper left')

def update(i):
    if args.remote_connection:
        stdin, stdout, stderr = client.exec_command(command)
        output = stdout.read().decode()
    else:
        output = subprocess.check_output(command, shell=True).decode("utf-8")

    print("output: %s" % output)
    
    level = int(re.findall(r"(\d+)\s+dBm", output)[0])
    level = 1000/level
    quality = re.search(r"\d+/\d+", output).group(0)
    i = int(quality.split("/")[0])

    numbers1.append(i)
    # numbers2.append(level)

    line1.set_data(range(len(numbers1)), numbers1)
    # line2.set_data(range(len(numbers2)), numbers2)

    ax.relim()
    ax.autoscale_view()
    
    # Adjust the x-axis limits for better visualization
    ax.set_xlim(0, max_length)
    
    return line1,# line2,


fig.suptitle('Real-time wifi signal')
ani = FuncAnimation(fig, update, interval=0, blit=True)

# Display the plot
plt.show()

client.close()