class PIDController:
    def __init__(self, kp=1, ki=0.1, kd=0.1, params=dict()):
        """
        Create a PID controller!
        kp, ki, and kd are defaulted at 1
        that's about it

        """

        self.ki = ki if ki else 0 # initialize ki, kd, and kp
        self.kd = kd if kd else 0
        self.kp = kp if kp or (not kd and not ki) else 0

        self.errors = np.array(list()) # list of past errors
        self.allErr = 0 # sum of all errors

        self.cmds = np.array(list()) # list of past commands

        self.derivAvg = 5
        return

    def __add__(self, error): # add error with + sign
        self.errors.append(error)
        return
    def __radd__(self, error): # add error with + sign
        self.errors.append(error)
        return

    def append(self, error): # append error
        print(error)
        # print("hiii")
        self.__add__(error)
        return

    def adjust(self, error=None): # give an error and get a new command
        if error != None: # add error if inputted
            self.errors.append(error)
        if len(self.errors) == 0: # add 0 to error if empty
            self.errors = [0]

        adjusted = 0 # running total of new command, kp ki and kd add to this
        if self.kp:
            adjusted += self.p_control() # do p control
        if self.ki:
            pass# adjusted += i_control() # do i control
        if self.kd:
            pass# adjusted += d_control() # do d control

        self.cmds.append(adjusted) # add new cmd to list
        return adjusted

    def p_control(self):
        if not self.kp:
            return 0
        error = self.errors[-1]
        newcmd = -error * self.kp # formula for kp
        return newcmd

    def i_control(self):
        if not self.ki or self.errors.shape[0] < 1:
            return 0 # can't integrate empty lists
        # check for saturation
        error = self.errors[-1]
        self.allErr += error
        newcmd = self.allErr * self.ki # multiply sum by ki
        return newcmd

    def d_control(self):
        if not self.kd or self.errors.shape[0] < 10:
            return 0
        der = self.derivAvg
        oldsum = np.mean(self.errors[-2*der:-der]) # make sum of old errors
        newsum = np.mean(self.errors[-der:]) # make sum of new errors
        slope = (newsum - oldsum)/der
        newcmd = -slope * self.kd
        return newcmd

    def printControl(self):
        return zip(self.errors, self.cmds)


if __name__ == "__main__":
    pid = PIDController()
    pid.append(10)
    print "\n\n\nwow\n\n\n" if pid else "\n\n\rfalsse\n\n\n"
    # print pid.errors
    print pid.adjust()
