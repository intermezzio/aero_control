import itertools
class PIDController:
    freq = .1
    osc_period = 1
    def __init__(self, kp=1, ki=0.0, kd=0.0, group=5, r=0.33, smooth=True, timeStep=None):
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

        self.derivAvg = group
        if timeStep != None:
            PIDController.freq = timeStep
        self.iterations = 0

        self.tuneStatus = 0 # how far it's tuned:
        # 0: no p, i, or d
        # 1: p only
        # 2: p, i
        # 3: p, i, d

        self.smooth = smooth
        self.r = r
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
        p = self.p_control() # do p control
        i = self.i_control() # do i control
        d = self.d_control() # do d control
        adjusted = p+i+d
        if self.smooth:
            adjusted = self.smoothCmd(adjusted)
        self.cmds.append( (p,i,d, adjusted) ) # add new cmd to list
        self.iterations += 1
        print "step: %d cmd: %.2f"%(self.iterations, adjusted)
        return adjusted

    def p_control(self):
        if not self.kp:
            return 0
        error = self.errors[-1]
        newcmd = error * self.kp # formula for kp
        return newcmd

    def i_control(self):
        # print "doing i control"
        if not self.ki or len(self.errors) < 1:
            return 0 # can't integrate empty lists
        # check for saturation
        error = self.errors[-1]
        self.allErr += error * freq
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

    def ziegler_nichols_tuning(self):
        print "Open znt"
        if self.iterations < 20 or self.iterations * PIDController.freq < 2:
            print "--skip"
            return
        print "Starting tuning session"
        if self.tuneStatus == 0: # if tuning p
            print "Tuning Kp, Kp = %f"%self.kp
            # calculate time to recover
            recoveries = self.recoverTime()
            # if time is too large increase p

        elif self.tuneStatus == 1:
            print "Tuning Ki, Ki = %f"%self.ki
            oscills = oscillations() # count oscillations to see if converging
            #if oscills
        return

    def recoverTime(self):
        errors = self.errors
        recoveries = list(itertools.groupby(errors, lambda x: x > 0))
        print len(recoveries)
        print len(list(recoveries[0][1]))
        #lengths = [sum(i for i in x[1]) for x in recoveries]
        #print lengths
        return

    def oscillations(self, prev=20):
        # calculate sign changes in the last few elements
        testdata = self.errors[-prev:] # prev data to test for patterns
        changes = len(list(itertools.groupby(x, lambda x: x > 0))) + 1
        # calculate the amount of sign changes
        return changes

    def smoothCmd(self, curr, r=None, prev=None):
        if prev == None:
            prev=min(self.iterations, 5)
        if prev < 2:
            return curr
        if r == None:
            r = self.r
        #print "smoothing: r=%f and curr=%f"%(r, curr)
        cmds = self.cmds[-prev::-1]
        offset = map(lambda (i,x): (x[-1]) * ( r ** float(i+1)), enumerate(cmds))
        offsetSum = sum(offset)
        #print "offset: %f"%(offsetSum)
        newcmd = offsetSum / ((1-r**prev) / (1-r)) # finite geometric seq
        # use a geometric series to do the follwoing:
        # cmd + 1/4 * cmd[-1] + 1/16 * cmd[-2] ... etc
        print "newcmd: %f"%(newcmd)
        return newcmd
    
    def printControl(self):
        # print len(self.errors)
        # print len(self.cmds)
        return zip(self.errors, self.cmds)


if __name__ == "__main__":
    pid = PIDController(kp=0.5, ki=0, kd=0)
    # print "\n\n\nwow\n\n\n" if pid else "\n\n\rfalsse\n\n\n"
    # print pid.errors
    for i in range(5):
        pid.adjust(i - 2.5)
    for i in range(5):
        pid.adjust(2.5 - i)
    '''
    for i in range(5):
        pid.adjust(i - 2.5)
    for i in range(5):
        pid.adjust(2.5 - i)

    for i in range(5):
        pid.adjust(i - 2.5)
    for i in range(5):
        pid.adjust(2.5 - i)

    for i in range(5):
        pid.adjust(i - 2.5)
    for i in range(5):
        pid.adjust(2.5 - i)
    
    #log = pid.printControl()
    #for i, l in enumerate(log):
    #    print "%d: %s %s"%(i,l[0], l[-1][-1])
    '''
    # pid.ziegler_nichols_tuning()
