import scipy.interpolate as scipl
import numpy as np
import csv

def distance():
    filename = 'distance_data.csv'
    camera_data = []
    real_data = []

    with open(filename, newline='') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            if row:
                camera_data.append(float(row[0]))
                real_data.append(float(row[1]))

    f_sci=scipl.CubicSpline(camera_data,real_data)
    



