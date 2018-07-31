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

    def append(self, error):
        self.__add__(error)
        return

    def adjust(self, error=None):
        if error != None:
            self.errors.append(error)

        adjusted = 0
        adjusted += - self.kp * self.errors[-1]
        return adjusted

        # newError +=
if __name__ == "__main__":
    pid = PIDController(kp=0.75)
    pid += 10
    print bool(pid)
    print pid.errors
    print pid.adjust()
