
import csv
import numpy as np

def get_contact_values(filename):
    fileid = open(filename,'rb')
    readed_file = csv.reader(fileid, delimiter = ',')
    matrix = []
    for row in readed_file:
        matrix.append(row)
    matrix = matrix[1:]
    fileid.close()
    return matrix

