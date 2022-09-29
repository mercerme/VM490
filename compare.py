
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

# first folder that contains the csv files with the data to compare
DATA_FOLDER_1 = "All Way Stop with Turning"
# second folder that contains the csv files with the data to compare
DATA_FOLDER_2 = "FCFS Radius 250 Turning"
# NB: the two datasets being compared must have the same value of the 
# `increment` parameter

# #####################

########################
# a function to process the data in a folder 
# input
#  - folder: the path to a folder with processed csv data
# 
# output 
#  - avgs: a matrix with the average time lost per 
#          car for each of the different traffic configurations
def process_folder(folder):

     # load the metadata
     params = json.load( open( folder+"/params.json" ) )

     # loop through the files for each individual run in the data folder
     avgs = np.zeros((params["INCREMENTS"], params["INCREMENTS"], params["REPS"]))
     directory = os.fsencode(folder)
     for file in os.listdir(directory):
          filename = folder+"/"+os.fsdecode(file)
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
          	continue

          # read in the data and calculate the scheduled depart time and total time
          # lost due to traffic for each car in the data file
          df = pd.read_csv(filename)
          df["tripinfo_scheduledDepart"] = df["tripinfo_depart"] - df["tripinfo_departDelay"]
          df["totalTime"] = df["tripinfo_timeLoss"] + df["tripinfo_departDelay"]

          # sort by the scheduled depart time and remove the burn period
          df.sort_values(by=["tripinfo_scheduledDepart"], inplace=True)
          df.drop(df.index[:BURN], inplace=True)
          # calculate the overall average
          avgs[x, y, rep] = df.mean()["totalTime"]

     # average over repetitions
     avgs = np.sum(avgs, axis=2) / params["REPS"]
     avgs = np.flip(avgs, axis=0)

     return avgs



# process both of the folders
params1 = json.load( open( DATA_FOLDER_1+"/params.json" ) )
avgs1 = process_folder(DATA_FOLDER_1)
params2 = json.load( open( DATA_FOLDER_2+"/params.json" ) )
avgs2 = process_folder(DATA_FOLDER_2)

# make sure the results are of comparable dimensions
if params1["INCREMENTS"] != params2["INCREMENTS"]:
     raise ValueError("Compared Experiments do not match")


# display the plots of results
ticklabels = ["{:0.3f}".format(i / params1["INCREMENTS"]) for i in range(params1["INCREMENTS"])]

fig, ax =plt.subplots(1,3, figsize=(21, 7))

ax1 = sns.heatmap(avgs1, ax = ax[0], xticklabels = ticklabels, yticklabels = ticklabels[::-1], cbar_kws={'label':'avg time lost per trip (seconds)'})
ax1.set_title("average time of {:d}  seconds of random cars over {:d} reps \n with {} pad and {:0.3f} speed \n radius {:d} and algo {:s} and turning {} \n".format(params1["GENCARS"], params1["REPS"], params1["PAD"], params1["SPEED"], params1["RADIUS"], params1["ALGO"], params1["TURNING"]))
ax1.set(xlabel="Prob of car arrival in east/west direction each timestep", ylabel="Prob of car arrival in north/south direction each timestep")


ax2 = sns.heatmap(avgs2, ax = ax[1], xticklabels = ticklabels, yticklabels = ticklabels[::-1], cbar_kws={'label':'avg time lost per trip (seconds)'})
ax2.set_title("average time of {:d}  seconds of random cars over {:d} reps \n with {} pad and {:0.3f} speed \n radius {:d} and algo {:s}  and turning {} \n".format(params2["GENCARS"], params2["REPS"], params2["PAD"], params2["SPEED"], params2["RADIUS"], params2["ALGO"], params2["TURNING"]))
ax2.set(xlabel="Prob of car arrival in east/west direction each timestep", ylabel="Prob of car arrival in north/south direction each timestep")


ax3 = sns.heatmap(avgs1 - avgs2, ax = ax[2], xticklabels = ticklabels, yticklabels = ticklabels[::-1], cmap = "coolwarm", center = 0, cbar_kws={'label':'difference in avg time lost per trip (seconds)'})
ax3.set_title("difference plot of first minus second \n")
ax3.set(xlabel="Prob of car arrival in east/west direction each timestep", ylabel="Prob of car arrival in north/south direction each timestep")

plt.show()








