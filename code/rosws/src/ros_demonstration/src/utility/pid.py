#!/usr/bin/python

import rospy

class Pid (object):
    def __init__(self, p, i, d, min_out, max_out):
        if __name__ == '__main__':
            rospy.init_node('pid')
        self.p = p
        self.i = i
        self.d = d
        self.min_out = min_out
        self.max_out = max_out
        self.accumulated_error = 0
        self.last_error = 0
        self.time_last_called = rospy.get_rostime()

    def calculate(self, cur_value, goal_value):
        error = goal_value - cur_value
        current_time = rospy.get_rostime()
        # REVIEW: maybe the time delta needs to be limited to a range? otherwise integral / derivate terms can become really big
        time_delta = (current_time - self.time_last_called).to_nsec() * 1E-9 # get nanosec diff and convert to seconds
        # rospy.logwarn(time_delta)
        self.time_last_called = current_time
        # proportional
        p_term = self.p * error
        # integral
        self.accumulated_error += error * time_delta
        i_term = self.i * self.accumulated_error
        # derivative
        # prevent division by zero
        if(time_delta == 0):
            time_delta = 0.0001
        d_term = self.d * (error - self.last_error) / time_delta # if we're moving towards the goal: last_error > error -> d_term negative
        self.last_error = error

        # output
        pid_value = p_term + i_term + d_term
        limited_pid_value = self.limitOutput(pid_value)

        # rospy.loginfo("p_term %lf, i_term %lf, d_term %lf, out %lf, lim_out %lf", p_term, i_term, d_term, pid_value, limited_pid_value)

        return limited_pid_value

    def limitOutput(self, output):
        if (output > self.max_out):
            return self.max_out
        elif (output < self.min_out):
            return self.min_out
        else:
            return output

    def reset(self):
        self.last_error = 0
        self.accumulated_error = 0

if __name__ == '__main__':
    pid = Pid(0.2, 0, 0.01, -15, 15)
    r = rospy.Rate(10)
    r.sleep() 
    val = 10
    while(not rospy.is_shutdown()):
        out = pid.calculate(val, 11)
        val += out
        rospy.loginfo(val)
        r.sleep()