# -*- coding: utf-8 -*-
"""
Created on Wed Jul 11 22:06:42 2018

clean raw RP data

@author: Margot Paez
"""
# The easist thing to do is keep this script in the same folder and the data files
# The script outputs a file without the comments and keeps the same name as the original
# but appends "_cleaned_rawdata" to the name.
# For example the original file is "rover-run.txt", the cleaned file will be called
# "rover-run_cleaned_rawdata.txt".

# open file
filename = "7_13_15_11_wheel_3legged.txt" # enter text file name here
name,ext = filename.split('.')
outfilename = name+"_cleaned_rawdata."+ext
rawdata = []

file = open(filename,'r')

# read each line and check for comments
for line in file:
    if "%" not in line:
        rawdata.append(line)

file.close()

# save new file with only datapoints
rawfile = open(outfilename,'w')

for line in rawdata:
    rawfile.write(line)

rawfile.close()
