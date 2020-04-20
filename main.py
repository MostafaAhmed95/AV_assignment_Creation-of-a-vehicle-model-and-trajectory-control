import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from way_points import *
k_e = 0.3
kp=1
ki=0.2
kd=0.01
integral_v=0
previous_error=0
previous_throttle=0
Kp_ld = 0.8
min_ld = 10
L = 3
x_data=[]
y_data=[]
class Vehicle():
    def __init__(self):
        # ==================================
        #  Parameters
        # ==================================

        #normal_param
        self.xc = 0
        self.yc = 0
        self.theta = 0
        self.delta = 0
        self.beta = 0

        self.L = 2
        self.lr = 1.2
        self.w_max = 1.22

        self.sample_time = 0.01

        # Throttle to engine torque
        self.a_0 = 400
        self.a_1 = 0.1
        self.a_2 = -0.0002

        # Gear ratio, effective radius, mass + inertia
        self.GR = 0.35
        self.r_e = 0.3
        self.J_e = 10
        self.m = 2000
        self.g = 9.81

        # Aerodynamic and friction coefficients
        self.c_a = 1.36
        self.c_r1 = 0.01

        # Tire force
        self.c = 10000
        self.F_max = 10000

        # State variables
        self.v = 1
        self.a = 0
        self.w_e = 70
        self.w_e_dot = 0

        self.sample_time = 0.01
    def reset(self):
        # reset state variables
        self.v = 5
        self.a = 0
        self.w_e = 100
        self.w_e_dot = 0
        self.xc = 0
        self.yc = 0
        self.theta = 0
        self.delta = 0
        self.beta = 0

    def step(self, v, steer):
        self.xc=self.xc+(v*np.cos(self.theta+self.beta))*0.01 #multiplied by 0.01 because it is the sample time
        self.yc=self.yc+(v*np.sin(self.theta+self.beta))*0.01
        self.delta = steer
        self.theta= self.theta+((v*np.cos(self.beta)*np.tan(self.delta))/self.L)*0.01
        self.beta=np.arctan(self.lr*np.tan(self.delta)/self.L)
    '''
    def lat_con(self,desired_x,desired_y,desired_v,):
        #I will use stanley

        #first calculte heading error
        yaw_path = np.arctan2(desired_y-self.yc,desired_x-self.xc)
        yaw_diff_heading= yaw_path-self.theta

        if yaw_diff_heading >np.pi:
            yaw_diff_heading -= 2*np.pi
        if yaw_diff_heading < - np.pi:
            yaw_diff_heading += 2 * np.pi

        #crosstrack error
        crosstrack_error = np.sqrt((desired_y-self.yc)**2+(desired_x-self.xc)**2)
        yaw_cross_track = np.arctan2(desired_y-self.yc, desired_x-self.xc)
        yaw_path2ct = yaw_path - yaw_cross_track

        if yaw_path2ct > np.pi:
            yaw_path2ct -= 2 * np.pi
        if yaw_path2ct < -np.pi:
            yaw_path2ct += 2 * np.pi
        if yaw_path2ct > 0 :
            crosstrack_error = abs(crosstrack_error)

        else:
            crosstrack_error =- abs(crosstrack_error)

        yaw_diff_crosstrack = np.arctan(k_e * crosstrack_error / self.v)

        #calculate the steering
        steer_expect = yaw_diff_crosstrack + yaw_diff_heading
        if steer_expect > np.pi:
            steer_expect -= 2 * np.pi
        if steer_expect < -np.pi:
            steer_expect += 2 * np.pi
        steer_expect = min(1.22,steer_expect)
        steer_expect = max(-1.22,steer_expect)

        return steer_expect
    '''
    def lat_con(self,desired_x,desired_y):
        x_rear = self.xc - L * np.cos(self.theta) / 2
        y_rear = self.yc - L * np.sin(self.theta) / 2
        lookahead_distance = max(min_ld, Kp_ld * self.v)
        alpha = np.arctan2(desired_y-y_rear,desired_x-x_rear)-self.theta
        steer = np.arctan2(2*L*np.sin(alpha),lookahead_distance)
        return steer

    def long_con(self, desired_v, integral_v):
        global previous_error
        global previous_throttle
        s_t = self.sample_time
        e_vel = desired_v - self.v

        integral_v = integral_v + e_vel*s_t

        derivate = (e_vel - previous_error)/s_t

        acc = kp * e_vel +ki * integral_v + kd * derivate

        if acc>0:
            throttle_output = (np.tanh(acc)+1)/2
            if throttle_output- previous_throttle > 0.1:
                throttle_output = previous_throttle

        else:
            throttle_output=0

        previous_error = e_vel
        previous_throttle = throttle_output
        return throttle_output

    def update_v(self,throttle,alpha):
        Te = throttle * (self.a_0 + self.a_1 * self.w_e + self.a_2 * self.w_e ** 2)
        Fair = self.c_a * self.v ** 2
        Rx = self.c_r1 * self.v
        Fg = self.m * self.g * np.sin(alpha)
        Fl = Fair + Rx + Fg

        # torque equation
        self.w_e_dot = (Te - self.GR * self.r_e * Fl) / self.J_e
        w_w = self.GR * self.w_e
        s = (w_w * self.r_e - self.v) / self.v
        if abs(s) < 1:
            Fx = self.c * s
        else:
            Fx = self.F_max

        # acceleration equation
        self.a = (Fx - Fl) / self.m

        self.w_e = self.w_e + (self.w_e_dot * self.sample_time)
        self.v = self.v + (self.a * self.sample_time)


model=Vehicle()
model.xc=waypoints_np[0][0]
model.yc=waypoints_np[0][1]
model.v=waypoints_np[0][2]
for i in range(len(waypoints_np)):

    
    desired_x = waypoints_np[i][0]
    desired_y = waypoints_np[i][0]
    desired_v = waypoints_np[i][0]

    steer=model.lat_con(desired_x,desired_y)
    throttle=model.long_con(desired_v,integral_v)
    model.update_v(throttle,0)#assume no inclination in the track

    x_data.append(model.xc)
    y_data.append(model.yc)
    model.step(model.v, steer)


plt.axis('equal')
plt.plot(x_data, y_data, label='Model')
plt.legend()
plt.show()
