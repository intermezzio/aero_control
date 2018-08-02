import numpy as np
class PIDController:
    def __init__(self, kp=1, ki=0.1, kd=0.1, params=dict()):
        """
        Create a PID controller!
        kp, ki, and kd are defaulted at 1
        that's about it

        """

        self.ki = ki # initialize ki, kd, and kp
        self.kd = kd
        self.kp = kp

        self.errors = list() # list of past errors
        self.allErr = 0 # sum of all errors

        self.cmds = list() # list of past commands

        self.derivAvg = 5
        return

    def __add__(self, error): # add error with + sign
        self.errors.append(error)
        return
    def __radd__(self, error): # add error with + sign
        self.errors.append(error)
        return

    def append(self, error): # append error
        # print(error)
        # print("hiii")
        self.__add__(error)
        return

    def adjust(self, error=None): # give an error and get a new command
        if error != None: # add error if inputted
            self.errors.append(error)
        elif len(self.errors) == 0: # add 0 to error if empty
            self.errors = [0]
        # print "errors: %s"%self.errors
        adjusted = 0 # running total of new command, kp ki and kd add to this
        if self.kp:
            p = self.p_control() # do p control
        if self.ki:
            i = self.i_control() # do i control
        if self.kd:
            d = self.d_control() # do d control

        self.cmds.append( (p,i,d,p+i+d) ) # add new cmd to list
        return p+i+d

    def p_control(self):
        if not self.kp:
            return 0
        error = self.errors[-1]
        newcmd = -error * self.kp # formula for kp
        return newcmd

    def i_control(self):
        # print "doing i control"
        if not self.ki or len(self.errors) < 1:
            return 0 # can't integrate empty lists
        # check for saturation
        error = self.errors[-1]
        self.allErr += error
        newcmd = self.allErr * self.ki # multiply sum by ki
        return newcmd

    def d_control(self):
        # print "doing d control"
        if not self.kd or len(self.errors) < 10:
            return 0
        # print "d approved"
        der = self.derivAvg
        oldsum = sum(self.errors[-2*der:-der])/der # make sum of old errors
        newsum = sum(self.errors[-der:])/der # make sum of new errors
        slope = (newsum - oldsum)/der
        newcmd = -slope * self.kd
        return newcmd

    def printControl(self):
        # print len(self.errors)
        # print len(self.cmds)
        return zip(self.errors, self.cmds)


if __name__ == "__main__":
    pid = PIDController(kp=0.5, kd = 0.5)
    # print "\n\n\nwow\n\n\n" if pid else "\n\n\rfalsse\n\n\n"
    # print pid.errors
    for i in range(10):
        pid.adjust(i - 3.)
    for i in pid.printControl():
        print i
