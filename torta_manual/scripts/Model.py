import numpy as np

class kinematicModel:
    def __init__ (self, Vx, Vy, W, wheel_radius = 0, lx = 0, ly = 0):
        
        self.Vx = Vx
        self.Vy = Vy
        self.W = W
        self.wheel_radius = wheel_radius
        self.lx = lx
        self.ly = ly
    
    def mecanum_4_vel(self):
        
        scalar = 1/self.wheel_radius
        
        sum = self.lx + self.ly

        fixed_array = np.array([[1, -1, -sum],
                                [1,  1,  sum],
                                [1,  1, -sum],
                                [1, -1,  sum]])
        
        input_array = np.array([self.Vx, self.Vy, self.W])

        outside_matrix = np.dot(scalar * fixed_array, input_array)

        return outside_matrix