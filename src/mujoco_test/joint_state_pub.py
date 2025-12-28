import mujoco
import time
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Load model
model = mujoco.MjModel.from_xml_path("/home/drunk/mujoco_menagerie/universal_robots_ur5e/scene.xml")
data = mujoco.MjData(model)

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
        msg.data = f"{q}\nJoint_velocities: {dq}"# 6-DOF UR5
        self.button_sub.publish(msg)




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
