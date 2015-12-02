# You want to make sure your version produces better error rates than this :)

import sys

#filename = sys.argv[1]
#x, y = open(filename, 'r').readlines()[-1].split(',')

#with open('prediction.txt', 'w') as f:
#    for _ in range(60):
#        print >> f, '%s,%s' % (x.strip(), y.strip())


#This is an implementation of a kalman filter in two demensions 
#it takes in pairs of integers that represent the location of a robot
#in x,y space and outputs a file containing 60 predicted locations for the robot

#written by William Scott for GT-CS8803 fall 2014

from math import *
from os.path import abspath, exists
import re
import string


class matrix:
    # implements basic operations of a matrix class
    def __init__(self, value):
        self.value = value
        self.dimx = len(value)
        self.dimy = len(value[0])
        if value == [[]]:
            self.dimx = 0
    
    def zero(self, dimx, dimy):
        # check if valid dimensions
        if dimx < 1 or dimy < 1:
            raise ValueError, "Invalid size of matrix"
        else:
            self.dimx = dimx
            self.dimy = dimy
            self.value = [[0 for row in range(dimy)] for col in range(dimx)]
    
    def identity(self, dim):
        # check if valid dimension
        if dim < 1:
            raise ValueError, "Invalid size of matrix"
        else:
            self.dimx = dim
            self.dimy = dim
            self.value = [[0 for row in range(dim)] for col in range(dim)]
            for i in range(dim):
                self.value[i][i] = 1
    
    def show(self):
        for i in range(self.dimx):
            print self.value[i]
        print ' '
    
    def __add__(self, other):
        # check if correct dimensions
        if self.dimx != other.dimx or self.dimy != other.dimy:
            raise ValueError, "Matrices must be of equal dimensions to add"
        else:
            # add if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j] = self.value[i][j] + other.value[i][j]
            return res
    
    def __sub__(self, other):
        # check if correct dimensions
        if self.dimx != other.dimx or self.dimy != other.dimy:
            raise ValueError, "Matrices must be of equal dimensions to subtract"
        else:
            # subtract if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j] = self.value[i][j] - other.value[i][j]
            return res
    
    def __mul__(self, other):
        # check if correct dimensions
        if self.dimy != other.dimx:
            raise ValueError, "Matrices must be m*n and n*p to multiply"
        else:
            # subtract if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, other.dimy)
            for i in range(self.dimx):
                for j in range(other.dimy):
                    for k in range(self.dimy):
                        res.value[i][j] += self.value[i][k] * other.value[k][j]
            return res
    
    def transpose(self):
        # compute transpose
        res = matrix([[]])
        res.zero(self.dimy, self.dimx)
        for i in range(self.dimx):
            for j in range(self.dimy):
                res.value[j][i] = self.value[i][j]
        return res
    
    # Thanks to Ernesto P. Adorio for use of Cholesky and CholeskyInverse functions
    
    def Cholesky(self, ztol=1.0e-5):
        # Computes the upper triangular Cholesky factorization of
        # a positive definite matrix.
        res = matrix([[]])
        res.zero(self.dimx, self.dimx)
        
        for i in range(self.dimx):
            S = sum([(res.value[k][i])**2 for k in range(i)])
            d = self.value[i][i] - S
            if abs(d) < ztol:
                res.value[i][i] = 0.0
            else:
                if d < 0.0:
                    raise ValueError, "Matrix not positive-definite"
                res.value[i][i] = sqrt(d)
            for j in range(i+1, self.dimx):
                S = sum([res.value[k][i] * res.value[k][j] for k in range(self.dimx)])
                if abs(S) < ztol:
                    S = 0.0
                res.value[i][j] = (self.value[i][j] - S)/res.value[i][i]
        return res
    
    def CholeskyInverse(self):
        # Computes inverse of matrix given its Cholesky upper Triangular
        # decomposition of matrix.
        res = matrix([[]])
        res.zero(self.dimx, self.dimx)
        
        # Backward step for inverse.
        for j in reversed(range(self.dimx)):
            tjj = self.value[j][j]
            S = sum([self.value[j][k]*res.value[j][k] for k in range(j+1, self.dimx)])
            res.value[j][j] = 1.0/tjj**2 - S/tjj
            for i in reversed(range(j)):
                res.value[j][i] = res.value[i][j] = -sum([self.value[i][k]*res.value[k][j] for k in range(i+1, self.dimx)])/self.value[i][i]
        return res
    
    def inverse(self):
        aux = self.Cholesky()
        res = aux.CholeskyInverse()
        return res
    
    def __repr__(self):
        return repr(self.value)


########################################

def filter(x, P):
    #this is the kalman filter implementation from class
    for n in range(len(measurements)):
        
        # prediction
        x = (F * x) + u
        P = F * P * F.transpose()
        
        # measurement update
        Z = matrix([measurements[n]])
        y = Z.transpose() - (H * x)
        S = H * P * H.transpose() + R
        K = P * H.transpose() * S.inverse()
        x = x + (K * y)
        P = (I - (K * H)) * P

    #put new x value in temp array
    temp = "".join(str(x.value[0]))
    #put new y value in temp array
    temp = temp + "".join(str(x.value[1]))
    
    #remove extra characters
    temp = temp.replace("]", " ")
    temp = temp.replace("[", "")
    
    #convert numbers to float then int
    temp = [float(x) for x in temp.split()]
    temp = [int(float(x)+.5) for x in temp]
    
    #update the measuerments array by
    #adding new x,y to end of measurements array
    measurements.append(temp)
    #and poping off the first (oldest) element
    measurements.pop(0)
    
    #append the new x,y to predictions array
    predictions.append(temp)

########################################

#arrays to use - probably could do this with fewer
robot_data = []
measurements = []
temp=[]
predictions = []
location = []

#let user know the calculations have started
print "\nstarting prediction calculations..."

#set path to test file
#f = abspath("testdata.txt")
f = sys.argv[1]
#open the file
file_handle = open(f, 'r')
#x, y = open(filename, 'r').readlines()[-1].split(',')

#read the lines into a list, remove the extra stuff [],/n and replace
#the space between the numbers with a comma, append this data to the robot_data array
#close file when done
with open(f) as f:
    for line in f:
#        line = line.translate(None, '[]')
#        line = line.translate(None, '\n')
#        line = line.translate(None, ',')
#        line = re.sub(' ', ",",line )
        robot_data.append(line)
f.closed

#filename = sys.argv[1]
#x, y = open(filename, 'r').readlines()[-1].split(',')
#with open('prediction.txt', 'w') as f:
#    for _ in range(60):
#        print >> f, '%s,%s' % (x.strip(), y.strip())


#iterate through the robaot_data array and split the values into pairs at the commas
#append this to a new array called measurements
#measurements will hold the x,y values from the data file to run through the kalman filter       
for i in range(len(robot_data)):
    measurements.append(string.split(robot_data[i], ',')) 
for k in range(len(measurements)):
    for j in range(2):
        measurements[k][j] = int(measurements[k][j])

#set up matrix values
#time interval
dt = 0.1
# external motion
u = matrix([[0.], [0.], [0.], [0.]]) 
# initial uncertainty - low in location, high in velocity
P =  matrix([[0.,0.,0.,0.], [0.,0.,0.,0.], [0.,0.,1000.,0.], [0.,0.,0.,1000.]])
# next state function
F =  matrix([[1.,0.,dt,0.], [0.,1.,0.,dt], [0., 0., 1., 0.], [0.,0.,0.,1.]])
# measurement function
H =  matrix([[1.,0.,0.,0.], [0.,1.,0.,0.]])
# measurement uncertainty
R =  matrix([[.1,0.], [.0,.1]])
# identity matrix
I =  matrix([[1.,0.,0.,0.], [0.,1.,0.,0.], [0.,0.,1.,0.], [0.,0.,0.,1.]])

#run the filter 60 times to determine the next 60 locations
for i in range(60):  
    # initial state (location and velocity)
    x = matrix([[measurements[0][0]], [measurements[0][1]], [0.], [0.]])   
    #run the kalman filter on the meausurement
    filter(x, P)

#open the output file prediction.txt and write the new locations to it
file = open("prediction.txt", "w")
for i in range(len(predictions)):
    #set new x and y locations
    newX = predictions[i][0]
    newY = predictions[i][1]
    #put the new x and y coordinates into an array location
    location = newX,newY
    #write the data fromt the location array into the output file
    file.write(str(location[0])+','+str(location[1])+"\n")
#close the file
file.close()
#alert the user that the calculations are complete
print "\nrun complete - see prediction.txt for output of predicted x,y coordinates\n"

#end of program - William Scott