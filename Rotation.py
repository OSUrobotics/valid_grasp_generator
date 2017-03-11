#find the angle of rotation

radius = input('Enter the radius of the glass:')

a= [int(radius),0]
b1= input('Enter x coordinate of point to be rotated:')
b2= input('Enter y coordinate of point to be rotated:')
b= [int(b1),int(b2)]

import math

def findangle(p1,p2):
    #Find theta of rotation using dot product
    theta = math.acos((p1[0]*p2[0]+p1[1]*p2[1])/(math.hypot(p1[0],p1[1])*math.hypot(p2[0],p2[1])))
    return theta

angle= findangle(a,b)

print(math.degrees(angle))#Check


#Rotation of the point to the x-axis using the rotation matrix

if b[1] < 0:
    new_angle = angle
else:
    new_angle = 2*math.pi - angle

print(math.degrees(new_angle))#check

new_x= b[0]*math.cos(new_angle) - b[1]*math.sin(new_angle)
new_y= b[0]*math.sin(new_angle) + b[1]*math.cos(new_angle)

new_b= [new_x, new_y]

print(new_b)
