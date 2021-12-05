import odrive, atexit
import numpy as np
from time import sleep
from colorama import Fore, Style

def print_odrive_config(odrv, axis):
    print(f"max current: {axis.motor.config.current_lim}")
    print(f"regen current: {odrv.config.dc_max_negative_current}")
    print(f"calibration current: {axis.motor.config.calibration_current}")
    print(f"max velocity: {axis.controller.config.vel_limit}")
    print(f"pole pairs: {axis.motor.config.pole_pairs}")
    print(f"torque constant: {axis.motor.config.torque_constant}")
    print(f"motor type: {axis.motor.config.motor_type}")
    print(f"encoder cpr: {axis.encoder.config.cpr}")

def configure_odrive(axis):
    axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION
    sleep(5)
    assert(axis.motor.is_calibrated)

    axis.encoder.config.use_index = True
    sleep(5)
    assert(axis.error == 0)

    axis.encoder.config.pre_calibrated = True
    axis.config.startup_encoder_index_search = True
    axis.motor.config.pre_calibrated = True

class impedance_controller():
    def __init__(self, axis, k=0, d=0):
        self.axis = axis
        self.k = k
        self.d = d
        self.th = 0
        self.th_offset = 0
        self.dth = 0
        self.th_desired = axis.encoder.pos_estimate
        self.dth_desired = 0

    def start_current_control(self):
        self.axis.requested_state = 1 #AXIS_STATE_IDLE
        self.axis.controller.config.control_mode = 1 #CONTROL_MODE_TORQUE_CONTROL
        self.axis.controller.input_torque = 0
        self.axis.requested_state = 8 #AXIS_STATE_CLOSED_LOOP_CONTROL

    def stop_current_control(self):
        self.axis.requested_state = 1 #AXIS_STATE_IDLE
        self.axis.controller.input_torque = 0

    def set_offset(self):
        self.th_offset = self.axis.encoder.pos_estimate

    def run(self, verbose = False, pallete = None):
        self.th = self.axis.encoder.pos_estimate
        self.dth = self.axis.encoder.vel_estimate
        self.current = self.k*(self.th_desired - (self.th - self.th_offset)) + self.d*(self.dth_desired - self.dth)
        self.axis.controller.input_torque = self.current

        if verbose:
            # gay impedance controller when

            readable = lambda x: "{:6.3f}".format(x)

            if pallete == 'RYG':
                output = f"{Fore.RED}th: {readable(self.th - self.th_offset)}{Style.RESET_ALL} "
                output += f"{Fore.YELLOW}dth: {readable(self.dth)}{Style.RESET_ALL} "
                output += f"{Fore.GREEN}current: {readable(self.current)}{Style.RESET_ALL}  "

            elif pallete == 'BIV':
                output = f"{Fore.CYAN}th: {readable(self.th - self.th_offset)}{Style.RESET_ALL} "
                output += f"{Fore.BLUE}dth: {readable(self.dth)}{Style.RESET_ALL} "
                output += f"{Fore.MAGENTA}current: {readable(self.current)}{Style.RESET_ALL} "

            else:
                self.stop_current_control()
                raise RuntimeError("Pick a color, fuckwit.")

            return output

        else:
            return None

    def get_state_vector(self):
        return [self.th - self.th_offset, self.dth]
    
    def get_log(self):
        return [self.th - self.th_offset, self.dth, self.current]

## Setup odrive and impedance controllers

odrv = odrive.find_any()
# th1_controller = impedance_controller(odrv.axis0, k=0, d=0)
# th2_controller = impedance_controller(odrv.axis1, k=0, d=0)
th1_controller = impedance_controller(odrv.axis0, k=200, d=4)
th2_controller = impedance_controller(odrv.axis1, k=300, d=6)
th1_controller.start_current_control()
th2_controller.start_current_control()

def set_pose(pose):
    th1_controller.th_desired = np.array(pose)[0]
    th1_controller.dth_desired = np.array(pose)[1]
    th2_controller.th_desired = np.array(pose)[2]
    th2_controller.dth_desired = np.array(pose)[3]

def set_pose_calibration():
    set_pose([0, 0, 0, 0])

def set_pose_stance():
    set_pose([-0.587, 0, 0.292, 0])

def set_pose_preflight():
    set_pose([0.102, 0, 0.518, 0])

## Register exit handler to stop odrives at program exit

def stop_everything():
    th1_controller.stop_current_control()
    th2_controller.stop_current_control()
    print("\nHard abort. Didn't feel like letting me down gently, didn't ya?")

atexit.register(stop_everything)


def execute_trajectory(trajectory):
    log = []
    for pose in trajectory:
        set_pose(pose)
        output = th1_controller.run(verbose=True, pallete='RYG')
        output += "   |   "
        output += th2_controller.run(verbose=True, pallete='BIV')
        output += "\n"

        log.append(th1_controller.get_log() + th2_controller.get_log())
        print(output)

        sleep(0.010)
    
    return np.array(log)

def run(duration = 0):
    if duration:
        for _ in range(round(duration*100)):
            output = th1_controller.run(verbose=True, pallete='RYG')
            output += "   |   "
            output += th2_controller.run(verbose=True, pallete='BIV')
            output += "\n"
            print(output)
            sleep(0.010)

    else:
        while(True):
            output = th1_controller.run(verbose=True, pallete='RYG')
            output += "   |   "
            output += th2_controller.run(verbose=True, pallete='BIV')
            output += "\n"
            print(output)
            sleep(0.010)

trajectory = np.load('trajectory.npy')

# get calibration position
input("Press enter to set calibration position.")
th1_controller.set_offset()
th2_controller.set_offset()
print(f"Calibration position set to th1: {th1_controller.th_offset}, th2: {th2_controller.th_offset}")
input()
th1_controller.k = 20
th2_controller.k = 20
set_pose(trajectory[0])
run(3)
th1_controller.k = 200
th2_controller.k = 300
run(3)

output = execute_trajectory(trajectory)
run(3)

stop_everything()

import matplotlib.pyplot as plt
plt.plot(output[:,0], label = 'th1')
plt.plot(output[:,1], label = 'dth1')
plt.plot(output[:,2], label = 'u1')
plt.plot(output[:,3], label = 'th2')
plt.plot(output[:,4], label = 'dth2')
plt.plot(output[:,5], label = 'u2')
plt.legend()
plt.show()




# want to record trajectory after we push the button
# input("ready to record trajectory. press enter to begin recording.")
# trajectory = []
# for _ in range(300):
#     output = th1_controller.run(verbose=True, pallete='RYG')
#     output += "   |   "
#     output += th2_controller.run(verbose=True, pallete='BIV')
#     output += "\n"
#     print(output)

#     trajectory.append(th1_controller.get_state_vector() + th2_controller.get_state_vector())

#     sleep(0.010)



# juicy bits
# set_pose_preflight()
#run(3)
#set_pose_stance()
#run()

#trajectory = np.array(trajectory)
#np.save('trajectory.npy', trajectory)