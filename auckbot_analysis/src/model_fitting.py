#!/usr/bin/env python
import roslib
import rospy
import numpy as np 
import matplotlib.pyplot as plt
import scipy.io as sio
import datetime

from auckbot_gazebo.msg import MotorCurrents
from geometry_msgs.msg import Twist

DEBUG = False

class ModelFitting:
    def __init__(self):
        rospy.init_node('model_fitting', anonymous=True)
        self.currentTopic = '/current'
        self.velocityTopic = '/cmd_vel'
        self.currents = []
        self.velocitys = []
        self.accelerations = []
        self.times = []
        self.maxCurrents = [0, 0, 0, 0, 0]
        self.maxVelocitys = [0, 0, 0]
        self.waiting = 0
        self.maxWaiting = 1000000
        self.messagesRecieved = False
        self.starttime = self.toSec(rospy.Time.now())
        self.rate = 1 # every x seconds
        self.rateDenom = 1E4
        self.bufferCurrents = {0: [0, 0, 0, 0, 0]}
        self.bufferVelocitys = {0: [0, 0, 0]}
        self.bufferAccelerations = {0: [0, 0, 0]}
        self.lockCurrents = False
        self.lockVelocitys = False
        
        rospy.Subscriber(self.currentTopic, MotorCurrents, self.callback)
        rospy.Subscriber(self.velocityTopic, Twist, self.callback)
        rospy.loginfo("initialized")
        rospy.Timer(rospy.Duration(self.rate), self.timerCallback)

    def timerCallback(self, event):
        timerec = self.now()
        if self.messagesRecieved:
            while self.lockCurrents:
                rospy.sleep(self.rate/self.rateDenom)
            self.lockCurrents = True
            self.makeMean(self.bufferCurrents, timerec)
            self.lockCurrents = False
            while self.lockVelocitys:
                rospy.sleep(self.rate/self.rateDenom)
            self.lockVelocitys = True
            self.makeMean(self.bufferVelocitys, timerec)
            self.lockVelocitys = False
            if self.bufferVelocitys.keys()[0] > 0:
                self.times.append(timerec)

    def makeMean(self, buffer, starttime):
        if len(buffer.keys()) == 1:
            thetime = buffer.keys()[0]
            lastval = buffer[thetime]
            length = len(buffer[thetime])
            if buffer.keys()[0] > 0:
                if length is 5: # current
                    self.currents.append(lastval)
                elif length is 3: # velocity
                    self.velocitys.append(lastval)
                    # acceleration
                    lastacc = self.bufferAccelerations[self.bufferAccelerations.keys()[0]]
                    self.accelerations.append(lastacc)
                    self.bufferAccelerations.clear()
                    self.bufferAccelerations[starttime] = lastacc
            buffer.clear()
            buffer[starttime] = lastval
        elif len(buffer.keys()) > 1:
            length = len(buffer[buffer.keys()[0]])
            temp = np.zeros(length)
            lasttime = buffer.keys()[0]
            totalduration = 0
            for key in buffer.keys():
                if key != lasttime:
                    duration = key-lasttime
                    if duration < 0:
                        duration = -1 * duration
                    onetemp = duration * np.array(buffer[lasttime])
                    temp += onetemp
                    totalduration += duration
                    lasttime = key
            temp /= totalduration
            if totalduration is 0:
                rospy.logwarn("totalduration is 0")
                rospy.logwarn(str(buffer))
            if DEBUG:
                rospy.loginfo("totalduration: " + str(totalduration))
            lastval = buffer[lasttime]
            buffer.clear()
            buffer[starttime+self.rate] = lastval
            if buffer.keys()[0] > 0:
                if length is 5: # current
                    self.currents.append(temp.tolist())
                    if DEBUG:
                        rospy.loginfo("mean currents: " + str(temp.tolist()))
                elif length is 3: # velocity
                    self.velocitys.append(temp.tolist())
                    # acceleration
                    totalduration = 0
                    temp = 0
                    lasttime = self.bufferAccelerations.keys()[0]
                    for key in self.bufferAccelerations.keys():
                        if key != lasttime:
                            duration = key-lasttime
                            if duration < 0:
                                duration = -1 * duration
                            onetemp = duration * np.array(self.bufferAccelerations[lasttime])
                            temp += onetemp
                            totalduration += duration
                            lasttime = key
                    temp /= totalduration
                    lastacc = self.bufferAccelerations[lasttime]
                    self.bufferAccelerations.clear()
                    self.bufferAccelerations[starttime+self.rate] = lastacc
                    self.accelerations.append(temp.tolist())
                    if DEBUG:
                        rospy.loginfo("mean velocitys: " + str(temp.tolist()))
                        rospy.loginfo("mean accelerations: " + str(temp.tolist()))
            else:
                rospy.logerr("makeMean called with incorrect buffer type")
        else:
            rospy.logwarn("makeMean called with empty buffer - consider increasing rate")

    def waitingIt(self):
        self.waiting = self.waiting+1
        if (self.waiting > self.maxWaiting) and self.messagesRecieved:
            #rospy.loginfo("max currents: " + str(self.maxCurrents))
            #rospy.loginfo("max velocitys: " + str(self.maxVelocitys))
            rospy.logwarn("no message recieved for quite some time")
            self.messagesRecieved = False
            self.fixData()
            self.toFile()
            self.printData()
        return self.messagesRecieved
        
    def callback(self, msg):
        self.messagesRecieved = True
        self.waiting = 0
        if DEBUG:
            rospy.loginfo(self.toString (msg))
        self.save(msg, self.now())
    
    def toString(self, msg):
        if isinstance(msg, MotorCurrents):
            return ("{:.1f}s :: main: {:.2f}A, 1: {:.2f}A, 2: {:.2f}A, 3: {:.2f}A, 4: {:.2f}A"
                    .format(self.now(), msg.main, msg.motor_1, msg.motor_2, msg.motor_3, msg.motor_4))
        elif isinstance(msg, Twist):
            return ("x: {:.2f}, y: {:.2f}, th: {:.2f}"
                    .format(msg.linear.x, msg.linear.x, msg.angular.z))

    def save(self, msg, timenow):
        l = self.serialize(msg)
        if isinstance(msg, MotorCurrents): # current
            # check lock
            while self.lockCurrents:
                rospy.sleep(self.rate/self.rateDenom)
            self.lockCurrents = True
            self.bufferCurrents[timenow] = l
            # unlock
            self.lockCurrents = False
            # ---------
            for i in range(0, len(l)):
                if l[i] > self.maxCurrents[i]:
                    self.maxCurrents[i] = l[i]
        elif isinstance(msg, Twist): # velocity
            # check lock
            while self.lockVelocitys:
                rospy.sleep(self.rate/self.rateDenom)
            self.lockVelocitys = True 
            self.bufferVelocitys[timenow] = l
            # acceleration
            lastentry = self.bufferVelocitys.keys()[-1]
            speeddiff = np.array(l) - np.array(self.bufferVelocitys[lastentry])
            duration = timenow - lastentry
            if duration == 0:
                self.bufferAccelerations[timenow] = [0., 0., 0.]
            else:
                self.bufferAccelerations[timenow] = (speeddiff / duration).tolist()
            # unlock
            self.lockVelocitys = False            
            # ---------
            for i in range(0, len(l)):
                if abs(l[i]) > self.maxVelocitys[i]:
                    self.maxVelocitys[i] = abs(l[i])

    def serialize(self, msg):
        if isinstance(msg, MotorCurrents):
            return [msg.main, msg.motor_1, msg.motor_2, msg.motor_3, msg.motor_4]
        elif isinstance(msg, Twist):
            return [msg.linear.x, msg.linear.y, msg.angular.z]

    def now(self):
        return self.toSec(rospy.Time.now()) - self.starttime

    def toSec(self, time):
        return float(time.to_sec()) + time.to_nsec() / 10E9

    def toFile(self):
        name = (str(datetime.datetime.now())).replace(" ", "_") + ".mat"
        sio.savemat(name, 
                    {'time': self.times,
                    'current': self.currents,
                    'velocity': self.velocitys,
                    'acceleration': self.accelerations})

    def printData(self):
        # currents
        plt.subplot(3, 1, 1)
        c0, = plt.plot(np.array(self.times), np.array(self.currents)[:,0], '-')
        c1, = plt.plot(np.array(self.times), np.array(self.currents)[:,1], '-')
        c2, = plt.plot(np.array(self.times), np.array(self.currents)[:,2], '-')
        c3, = plt.plot(np.array(self.times), np.array(self.currents)[:,3], '-')
        c4, = plt.plot(np.array(self.times), np.array(self.currents)[:,4], '-')
        plt.legend((c0, c1, c2, c3, c4), ('main', 'motor 1', 'motor 2', 'motor 3', 'motor 4'))
        # velocitys
        plt.subplot(3, 1, 2)
        v0, = plt.plot(np.array(self.times), np.array(self.accelerations)[:,0], '-')
        v1, = plt.plot(np.array(self.times), np.array(self.accelerations)[:,1], '-')
        v2, = plt.plot(np.array(self.times), np.array(self.accelerations)[:,2], '-')
        plt.legend((v0, v1, v2), (r'$a_x$', r'$a_y$', r'$a_\theta$'))
        # velocitys
        plt.subplot(3, 1, 3)
        v0, = plt.plot(np.array(self.times), np.array(self.velocitys)[:,0], '-')
        v1, = plt.plot(np.array(self.times), np.array(self.velocitys)[:,1], '-')
        v2, = plt.plot(np.array(self.times), np.array(self.velocitys)[:,2], '-')
        plt.legend((v0, v1, v2), (r'$v_x$', r'$v_y$', r'$v_\theta$'))
        # show ...
        plt.show()

    def fixData(self):
        l = min(len(self.currents), len(self.velocitys), len(self.accelerations), len(self.times))
        self.currents = (np.array(self.currents)[1:l,:]).tolist()
        self.velocitys = (np.array(self.velocitys)[1:l,:]).tolist()
        self.accelerations = (np.array(self.accelerations)[1:l,:]).tolist()
        self.times = (np.array(self.times)[1:l]).tolist()

if __name__ == '__main__':
    mf = ModelFitting()   
    while not rospy.is_shutdown(): # and mf.waitingIt()):
        mf.waitingIt()
    rospy.loginfo("shutdown")