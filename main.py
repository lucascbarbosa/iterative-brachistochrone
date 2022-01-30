from asyncore import poll2
from ctypes import pointer
import numpy as np


class Environment(object):
    def __init__(self,origin,target,num_points):
        self.origin = origin
        self.target = target
        self.num_points = num_points

    def set_points(self):
        points = []
        d_x = self.target[0] - self.origin[0]
        d_y = self.target[1] - self.origin[1]
        points.append(self.origin)
        for i in range(1,self.num_points-2):
            points.append([self.origin[0]+d_x*i/(self.num_points-1),self.origin[1]+d_y*i/(self.num_points-1)])

        points.append(self.target)
        points = np.array(points)
        self.points = np.array(points)

class Optimizer(object):
    def __init__(self,g,v0,alpha):
        self.g = g
        self.v0 = v0
        self.alpha = alpha

    def get_d(self,point1,point2):
        d = np.sqrt((point1[0]-point2[0])**2+(point1[1]-point2[1])**2)
        d = np.round(d,3)
        return d

    def get_theta(self,point1,point2):
        theta = np.arctan((point1[1]-point2[1])/(point2[0]-point1[0]))
        theta = np.round(theta,3)
        return theta

    def get_a(self,theta):
        a = self.g*np.sin(theta)
        a = np.round(a,3)
        return a

    def get_v(self,v1,theta1,point1,point2):
        theta2 = self.get_theta(point1,point2)
        v0 = v1*np.cos(theta1-theta2)
        a = self.get_a(theta2)
        d = self.get_d(point1,point2)
        v = np.sqrt(v1**2+2*a*d)
        v = np.round(v,3)
        return v

    def get_t(self,v1,theta1,point1,point2):
        theta2 = self.get_theta(point1,point2)
        v1 = v1*np.cos(theta1-theta2)
        d = self.get_d(point1,point2)
        a = self.get_a(theta2)
        c = -d
        b = v1 
        a = a/2
        # print([a,b,c])
        ts = np.round(np.roots([a,b,c]),3)
        t = np.where(ts > 0)[0]
        return ts[t][0]
    
    def get_t_total(self,points):
        v1 = self.v0
        t_total = 0

        for i in range(-1,len(points)-2):
            point1 = points[i+1]
            point2 = points[i+2]
            theta2 = self.get_theta(point1,point2)
            if i == -1:
                point0 = [0,0]
                theta1 = theta2
            else:
                point0 = points[i]
                theta1 = self.get_theta(point0,point1)
            t = self.get_t(v1,theta1,point1,point2)
            t_total += t
            v1 = self.get_v(v1,theta1,point1,point2)

        t_total = np.round(t_total,3)
        return t_total
    
if __name__ == "__main__":
    
    origin = [0,10]
    target = [10,0]
    num_points = 10

    env = Environment(origin,target,num_points)
    env.set_points()
    g = 9.8
    alpha = 0.1
    v0 = 0
    opt = Optimizer(g,v0,alpha)
    t_total = opt.get_t_total(env.points)
    print(f'Total time: {t_total}')