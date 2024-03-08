import numpy as np
from scipy.optimize import curve_fit
import csv

def tenth_order_func(x, a, b, c, d, e, f, g, h, i, j, k):
    return a * x**10 + b * x**9 + c * x**8 + d * x**7 + e * x**6 + f * x**5 + g * x**4 + h * x**3 + i * x**2 + j * x + k

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

    popt, pcov = curve_fit(tenth_order_func, camera_data, real_data)

    x_value = 100
    y_value = tenth_order_func(x_value, *popt)
    print(y_value)

distance()