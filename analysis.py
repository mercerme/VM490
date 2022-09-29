
import numpy as np
import pandas as pd
import os
import json
import re


import matplotlib.pyplot as plt
import seaborn as sns

# #####################
# analysis parameters
# #####################

# the number of data points to discard from the beginning of the data
# before doing the analysis to remove any effects on the 
# simulation from starting with no traffic
BURN = 20
# number of data points to use when computing the early values
# for comparison over time
FRONT_AVG = 30
# number of data points to use when computing the late values
# for comparison over time
BACK_AVG = 30
# folder that contains the csv files with the data
DATA_FOLDER = "FCFS Radius 900 Turning"

# #####################


# load the metadata
params = json.load( open( DATA_FOLDER+"/params.json" ) )

avgs = np.zeros((params["INCREMENTS"], params["INCREMENTS"], params["REPS"]))
firstAvgs = np.zeros((params["INCREMENTS"], params["INCREMENTS"], params["REPS"]))
lastAvgs = np.zeros((params["INCREMENTS"], params["INCREMENTS"], params["REPS"]))

# loop through the files for each individual run in the data folder
directory = os.fsencode(DATA_FOLDER)
for file in os.listdir(directory):
     filename = DATA_FOLDER+"/"+os.fsdecode(file)
     if not filename.endswith(".csv"):
     	continue

     # pull the traffic density data for that run 
     # out of the filename
     nameArgs = re.split('\(|,|\)',os.fsdecode(file))
     x = int(nameArgs[1])
     y = int(nameArgs[2])
     rep = int(nameArgs[3])
     # handle the empty file from the runs with no cars
     if os.stat(filename).st_size < 5:
     	avgs[x, y, rep] = 0
     	firstAvgs[x, y, rep] = 0
     	lastAvgs[x, y, rep] = 0
     	continue

     # read in the data and calculate the scheduled depart time and total time
     # lost due to traffic for each car in the data file
     df = pd.read_csv(filename)
     df["tripinfo_scheduledDepart"] = df["tripinfo_depart"] - df["tripinfo_departDelay"]
     df["totalTime"] = df["tripinfo_timeLoss"] + df["tripinfo_departDelay"]

     # sort by the scheduled depart time and remove the burn period
     df.sort_values(by=["tripinfo_scheduledDepart"], inplace=True)
     df.drop(df.index[:BURN], inplace=True)

     # calculate the overall average, early in the simulation average
     # and late in the simulation average time lost
     avgs[x, y, rep] = df.mean()["totalTime"]
     firstAvgs[x, y, rep] = df.head(FRONT_AVG).mean()["totalTime"]
     lastAvgs[x, y, rep] = df.tail(BACK_AVG).mean()["totalTime"]

# use the early and late averages to calculate how the average changes over time
difAvgs = lastAvgs - firstAvgs

# set up the plots and average across repetitions of the simulation
ticklabels = ["{:0.3f}".format(i / params["INCREMENTS"]) for i in range(params["INCREMENTS"])]
avgs = np.sum(avgs, axis=2) / params["REPS"]
avgs = np.flip(avgs, axis=0)
difAvgs = np.sum(difAvgs, axis=2) / params["REPS"]
difAvgs = np.flip(difAvgs, axis=0)
# find where the average changed by more than a threshold
difAvgsNeg = difAvgs > 10


# display the plots of results
fig, ax =plt.subplots(1,2, figsize=(14, 7))

ax1 = sns.heatmap(avgs, ax = ax[0], xticklabels = ticklabels, yticklabels = ticklabels[::-1], cbar_kws={'label':'avg time lost per trip (seconds)'})
ax1.set_title("average time of {:d} seconds of random cars over {:d} reps \n with {} pad and {:0.3f} speed \n radius {:d} and algo {:s} and turning {} \n".format(params["GENCARS"], params["REPS"], params["PAD"], params["SPEED"], params["RADIUS"], params["ALGO"], params["TURNING"]))
ax1.set(xlabel="Prob of car arrival in east/west direction each timestep", ylabel="Prob of car arrival in north/south direction each timestep")


cover = np.ones_like(difAvgsNeg)
ax2 = sns.heatmap(avgs, ax = ax[1], mask = difAvgsNeg, xticklabels = ticklabels, yticklabels = ticklabels[::-1], cbar_kws={'label':'avg time lost per trip (seconds)'})
ax2.set_title("average time of {:d} seconds of random cars over {:d} reps \n with {} pad and {:0.3f} speed with low dif (<= 10) \n radius {:d} and algo {:s}  and turning {} \n".format(params["GENCARS"], params["REPS"], params["PAD"], params["SPEED"], params["RADIUS"], params["ALGO"], params["TURNING"]))
ax2.set(xlabel="Prob of car arrival in east/west direction each timestep", ylabel="Prob of car arrival in north/south direction each timestep")
plt.show()








