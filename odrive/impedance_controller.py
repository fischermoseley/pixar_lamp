import odrive, atexit
import numpy as np
from time import sleep
from colorama import Fore, Style

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
        return [self.th - self.th_offset, self.dth, self.current, self.axis.motor.current_control.Iq_measured]

class pixar_lamp():
    def __init__(self, odrv, th1_gains, th2_gains):
        self.th1_controller = impedance_controller(odrv.axis0, k=th1_gains[0], d=th1_gains[1])
        self.th2_controller = impedance_controller(odrv.axis1, k=th2_gains[0], d=th2_gains[1])
        self.th1_controller.start_current_control()
        self.th2_controller.start_current_control()

        ## Register exit handler to stop odrives at program exit
        atexit.register(self.stop_everything)

    def set_offsets(self):
        self.th1_controller.set_offset()
        self.th2_controller.set_offset()

    def set_pose(self, pose):
        np_pose = np.array(pose)
        self.th1_controller.th_desired = np_pose[0]
        self.th1_controller.dth_desired = np_pose[1]
        self.th2_controller.th_desired = np_pose[2]
        self.th2_controller.dth_desired = np_pose[3]

    def set_pose_calibration(self):
        self.set_pose([0, 0, 0, 0])

    def set_pose_stance(self):
        self.set_pose([-0.587, 0, 0.292, 0])

    def set_pose_preflight(self):
        self.set_pose([0.102, 0, 0.518, 0])

    def stop_everything(self):
        self.th1_controller.stop_current_control()
        self.th2_controller.stop_current_control()

        print("\nHard abort. Didn't feel like letting me down gently, didn't ya?")

    def execute_trajectory(self, trajectory):
        log = []
        for pose in trajectory:
            self.set_pose(pose)
            output = self.th1_controller.run(verbose=True, pallete='RYG')
            output += "   |   "
            output += self.th2_controller.run(verbose=True, pallete='BIV')
            output += "\n"

            log.append(self.th1_controller.get_log() + self.th2_controller.get_log())
            print(output)

            sleep(0.010)
        
        return np.array(log)

    def run(self, duration = 0):
        if duration:
            for _ in range(round(duration*100)):
                output = self.th1_controller.run(verbose=True, pallete='RYG')
                output += "   |   "
                output += self.th2_controller.run(verbose=True, pallete='BIV')
                output += "\n"
                print(output)
                sleep(0.010)

        else:
            while(True):
                output = self.th1_controller.run(verbose=True, pallete='RYG')
                output += "   |   "
                output += self.th2_controller.run(verbose=True, pallete='BIV')
                output += "\n"
                print(output)
                sleep(0.010)

    def record_trajectory(self, duration):
        self.th1_controller.k = 0
        self.th1_controller.d = 0
        self.th2_controller.k = 0
        self.th2_controller.d = 0

        trajectory = []
        for _ in range(round(duration*100)):
            output = self.th1_controller.run(verbose=True, pallete='RYG')
            output += "   |   "
            output += self.th2_controller.run(verbose=True, pallete='BIV')
            output += "\n"
            print(output)

            trajectory.append(self.th1_controller.get_state_vector() + self.th2_controller.get_state_vector())

            sleep(0.010)

        return np.array(trajectory)

## Setup odrive and impedance controllers
lamp = pixar_lamp(odrive.find_any(), (200, 4), (300, 6))

# load trajectory from file
trajectory = np.load('trajectory.npy')


# get calibration position
input("Press enter to set calibration position.")
lamp.set_offsets()
print(f"Calibration position set to th1: {lamp.th1_controller.th_offset}, th2: {lamp.th2_controller.th_offset}")
lamp.set_pose_calibration()

#input("Press enter to record trajectory")
#np.save('trajectory.npy', lamp.record_trajectory(3))
#lamp.run()


input(f"Press any key to go to next position")
lamp.th1_controller.k = 20
lamp.th1_controller.d = 0.1
lamp.th2_controller.k = 20
lamp.th2_controller.d = 0.1
lamp.set_pose(trajectory[0])
lamp.run(3)
lamp.th1_controller.k = 200
lamp.th1_controller.d = 4
lamp.th2_controller.k = 300
lamp.th2_controller.d = 6
lamp.run(3)


# execute trajectory
output = lamp.execute_trajectory(trajectory)
lamp.run(3)

lamp.stop_everything()

import matplotlib.pyplot as plt
plt.plot(output[:,0], label = 'th1')
plt.plot(output[:,1], label = 'dth1')
plt.plot(output[:,2], label = 'u1_commanded')
plt.plot(output[:,3], label = 'u1_received')
plt.plot(output[:,4], label = 'th2')
plt.plot(output[:,5], label = 'dth2')
plt.plot(output[:,6], label = 'u2_commanded')
plt.plot(output[:,7], label = 'u2_received')
plt.legend()
plt.savefig('outputs/trajectory.png')
plt.show()