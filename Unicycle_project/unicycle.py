import math

class Unicycle:
    def __init__(self, x = 0.0, y = 0.0, th = 0.0):
        self.x = x
        self.y = y
        self.th = th

    def model(self, v,w,dt):
        #Euler method for discrete time
        self.x += dt * v * math.cos(self.th)
        self.y += dt * v * math.sin(self.th)
        self.th += dt * w

        # [-pi, pi]
        self.th = (self.th + math.pi) % (2*math.pi) - math.pi
        return self.x, self.y, self.th

    
