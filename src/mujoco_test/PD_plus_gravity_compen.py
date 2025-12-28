import mujoco
import time
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mujoco.viewer import launch_passive
#from matplotlib import pyplot

# Load model
model = mujoco.MjModel.from_xml_path("/home/drunk/mujoco_menagerie/universal_robots_ur5e/scene.xml")
data = mujoco.MjData(model)
viewer = launch_passive(model, data)
# Simulation timestep
dt = model.opt.timestep
'''
log = {
    "t": [],
    "q": [],
    "q_des": [],
    "dq": [],
    "tau": [],
    "tau_pd": [],
    "g_hat": [],
}
'''

print("Number of joints:", model.nq)
print("Number of actuators:", model.nu)

'''
while True:
    # ---- READ JOINT STATES (THIS IS THE SUBSCRIBE PART) ----
    q  = data.qpos.copy()   # joint positions
    dq = data.qvel.copy()   # joint velocities

    print("q:", np.round(q, 3))
    print("dq:", np.round(dq, 3))
    print("-" * 40)

    # ---- STEP SIMULATION ----
    
'''



class PhantomButton(Node):
    def __init__(self):
        super().__init__('phantom_Button')
        self.gravity = np.zeros(6)
        #self.kp = 200
        #self.ki = 10
        #self.kd = 50
        #self.prev_error = np.zeros(6)
        #self.integral = 0.0
        self.tau_min = np.array([-150, -150, -100, -28, -28, -28])
        self.tau_max = np.array([ 150,  150,  100,  28,  28,  28])
        self.t = 0.0

        self.kp = 15*np.array([100, 100, 100, 30, 20, 10])
        self.kd = 30*np.array([10, 10, 10, 3, 2, 1])

        #q_des = data.qpos.copy()  # hold initial pose

        self.button_sub = self.create_publisher(
            String,
            '/string/msgs',
            10
        )

        self.timer = self.create_timer(0.01, self.publish_cmd)  # 100 Hz

    def publish_cmd(self):
        msg = String()
        q  = data.qpos.copy()   # joint positions
        dq = data.qvel.copy()   # joint velocities
        #q_des = data.qpos.copy()
        q_des = np.array([0.0, -1.57, 1.57, 0.0, 0.0, 0.0])
        error = q_des - q

        #self.integral += error * dt
        #derivative = (error - self.prev_error) / dt
        #self.prev_error = error.copy()
        tau1 = self.kp*(error) - self.kd*dq
        

        self.gravity = self.gravity + (self.kp*(error) - self.gravity)*0.02
        tau2 = tau1 + self.gravity
        tau = np.minimum(np.maximum(tau2, self.tau_min), self.tau_max)
        '''
        output = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )'''
        #q_des1 = np.array([0.0, output, 1.57, 0.0, 0.0, 0.0])
        data.ctrl[:] = tau



        msg.data = f"{q}\nJoint_velocities: {dq}"# 6-DOF UR5
        self.button_sub.publish(msg)

        self.t += dt
        '''
        log["t"].append(self.t)
        log["q"].append(q.copy())
        log["q_des"].append(q_des.copy())
        log["dq"].append(dq.copy())
        log["tau"].append(tau.copy())
        log["tau_pd"].append(tau1.copy())
        log["g_hat"].append(self.gravity.copy())
        '''
        viewer.sync()
        mujoco.mj_step(model, data)
        time.sleep(dt)
        
                


def main(args=None):
    rclpy.init(args=args)
    node = PhantomButton()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    finally:

        viewer.close()
        node.destroy_node()
        '''        # --------- CONVERT LOGS TO NUMPY ----------
        for k in log:
            log[k] = np.array(log[k])

        # --------- TORQUE DECOMPOSITION PLOT ----------
        fig, axes = pyplot.subplots(3, 2, figsize=(12, 8), sharex=True)

        for i, ax in enumerate(axes.flatten()):
            ax.plot(log["tau"][:, i], label="total")
            ax.plot(log["tau_pd"][:, i], "--", label="PD")
            ax.plot(log["g_hat"][:, i], ":", label="gravity")
            ax.set_title(f"Joint {i+1}")
            ax.grid(True)

        axes[0, 0].legend()
        fig.suptitle("Torque Decomposition per Joint")
        pyplot.xlabel("Time [s]")
        pyplot.tight_layout()
        pyplot.show()

        # --------- POSITION TRACKING PLOT ----------
        fig, axes = pyplot.subplots(3, 2, figsize=(12, 8), sharex=True)

        for i, ax in enumerate(axes.flatten()):
            ax.plot(log["t"], log["q"][:, i], label="q")
            ax.plot(log["t"], log["q_des"][:, i], "--", label="q_des")
            ax.set_title(f"Joint {i+1}")
            ax.grid(True)

        axes[0, 0].legend()
        fig.suptitle("Joint Position Tracking")
        pyplot.xlabel("Time [s]")
        pyplot.tight_layout()
        pyplot.show()
        '''

if __name__ == '__main__':
    main()
