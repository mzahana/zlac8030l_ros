from time import time

import numpy as np


class PID:
    def __init__(self, kp=0, ki=0, kd=0):
        # PID gains
        self.kp=kp
        self.ki=ki
        self.kd=kd

        # This is used in the integral part of the PID
        self.last_int=0

        self.last_err = 0

        self.last_time = time()

        # Maximum integrator value
        self.MAX_INT=20000

    def update(self, err):
        '''
        This functions computes the control output u_k given the desired and feedback signals

        Params
        --
        - err [float] Error signal = desired - actual

        Returns
        --
        - u_k [float] control output signal
        '''

        ########### >>>>> Provide your implementation here <<<<< ###########
        #compute dt
        dt = time() - self.last_time
        self.last_time = time()

        # Proportional part
        p=self.kp*err

        # Integral part
        i=self.last_int + self.ki*err*dt
        if(abs(i) > self.MAX_INT):
            i = (i/abs(i))*self.MAX_INT

        self.last_int = i

        # Derivative part
        d=self.kd*(err-self.last_err)/dt
        # Update
        self.last_err = err

        u_k = p+i+d

        return(u_k)
