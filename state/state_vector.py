import numpy as np
class StateVector14:
    def __init__(self, x0: np.ndarray):
        self.state = x0
    def unpack(self):
        self.x = self.state[0]
        self.y = self.state[1]
        self.z = self.state[2]

        self.roll = self.state[3]
        self.pitch = self.state[4]
        self.yaw = self.state[5]

        self.z_fl = self.state[6]
        self.z_fr  = self.state[7]
        self.z_rl = self.state[8]
        self.z_rr = self.state[9]

        self.w_fl = self.state[10]
        self.w_fr = self.state[11]
        self.w_rl = self.state[12]
        self.w_rr = self.state[13]

        # FIRST DERIVATIVES #

        self.x_dt = self.state[14]
        self.y_dt = self.state[15]
        self.z_dt = self.state[16]

        self.roll_dt = self.state[17]
        self.pitch_dt = self.state[18]
        self.yaw_dt = self.state[19]

        self.z_fl_dt = self.state[20]
        self.z_fr_dt = self.state[21]
        self.z_rl_dt = self.state[22]
        self.z_rr_dt = self.state[23]

        self.w_fl_dt = self.state[24]
        self.w_fr_dt = self.state[25]
        self.w_rl_dt = self.state[26]
        self.w_rr_dt = self.state[27]



    def pack (self):
        self.state = np.array([
            self.x,
            self.y,
            self.z,
            self.roll,
            self.pitch,
            self.yaw,
            self.z_fl,
            self.z_fr,
            self.z_rl,
            self.z_rr,
            self.w_fl,
            self.w_fr,
            self.w_rl,
            self.w_rr,
            self.x_dt,
            self.y_dt,
            self.z_dt,
            self.roll_dt,
            self.yaw_dt,
            self.z_fl_dt,
            self.z_fr_dt,
            self.z_rl_dt,
            self.z_rr_dt,
            self.w_fl_dt,
            self.w_fr_dt,
            self.w_rl_dt,
            self.w_rr_dt
        ])