class PIDController:
    def __init__(self, kp=1, ki=1, kd=1, p=True, i=True, d=True, params=dict()):
        self.ki = ki if i else 0
        self.kd = kd if d else 0
        self.kp = kp if p or (not d and not i) else 0

        self.errors = list()
        self.cmds = list()
        return

    def __add__(self, error):
        self.errors.append(error)
        return
    def __radd__(self, error):
        self.errors.append(error)
        return

    def append(self, error):
        print(error)
        # print("hiii")
        self.__add__(error)
        return

    def adjust(self, error=None):
        if error != None:
            self.errors.append(error)
        if len(self.errors) == 0:
            self.errors = [0]

        adjusted = 0
        if self.kp:
            adjusted += self.p_control()
        # if self.ki:
            # adjusted += i_control()
        # if self.kd:
            # adjusted += d_control()

        self.cmds.append(adjusted)
        return adjusted

    def p_control(self):
        if not self.kp:
            return 0
        error = self.errors[-1]
        newcmd = error * self.kp
        return newcmd
        # newError +=
if __name__ == "__main__":
    pid = PIDController()
    pid.append(10)
    print "\n\n\nwow\n\n\n" if pid else "\n\n\rfalsse\n\n\n"
    # print pid.errors
    print pid.adjust()