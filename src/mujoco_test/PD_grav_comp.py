#import mujoco
import time
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
#from mujoco.viewer import launch_passive
from matplotlib import pyplot as plt

# Load model
#model = mujoco.MjModel.from_xml_path("/home/drunk/mujoco_menagerie/universal_robots_ur5e/scene.xml")
#data = mujoco.MjData(model)
#viewer = launch_passive(model, data)
# Simulation timestep
#dt = model.opt.timestep

log = {
    "t": [],
    "q": [],
    "q_des": [],
    "dq": [],
    "tau": [],
    "tau_pd": [],
    "g_hat": [],
}


#print("Number of joints:", model.nq)
#print("Number of actuators:", model.nu)

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



class PD_controller(Node):
    def __init__(self):
        super().__init__('PD_controller')
        self.gravity = np.zeros(6)
        #self.kp = 200
        #self.ki = 10
        #self.kd = 50
        #self.prev_error = np.zeros(6)
        #self.integral = 0.0
        self.q = np.zeros(6)
        self.dq = np.zeros(6)

        self.q_des = np.array([0, -1.57, 1.57, 0, 0, 0])
        self.tau_min = np.array([-150, -150, -100, -28, -28, -28])
        self.tau_max = np.array([ 150,  150,  100,  28,  28,  28])
        self.t = 0.0

        self.kp = 15*np.array([100, 100, 100, 30, 20, 10])
        self.kd = 30*np.array([10, 10, 10, 3, 2, 1])
        self.gravity_hat = np.zeros(6)

        #q_des = data.qpos.copy()  # hold initial pose
        self.joint_pos = self.create_subscription(
            Float64MultiArray,
            '/joint_states/pos',
            self.pos_callback,
            10
        )

        self.joint_vel = self.create_subscription(
            Float64MultiArray,
            '/joint_states/vel',
            self.vel_callback,
            10
        )

        self.joint_cmd = self.create_publisher(
            Float64MultiArray,
            '/joint_torque/cmd',
            10
        )


        self.control_timer = self.create_timer(0.01, self.command_callback)

    def pos_callback(self, msg):
        self.q = np.array(msg.data)

    def vel_callback(self, msg1):
        self.dq = np.array(msg1.data)

    def command_callback(self):

        if self.q.shape[0] != 6 or self.dq.shape[0] != 6:
            return

        error = self.q_des -self.q

        tau_pd = self.kp*error - self.kd*self.dq
        self.gravity_hat += (self.kp*error - self.gravity_hat)*0.02

        tau = tau_pd + self.gravity_hat
        tau = np.clip(tau, self.tau_min, self.tau_max)


        msg = Float64MultiArray()
        msg.data = tau.tolist()

        #print(f"{error}")
        self.joint_cmd.publish(msg)        


def main(args=None):
    rclpy.init(args=args)
    node = PD_controller()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    finally:

        #viewer.close()
        node.destroy_node()




if __name__ == '__main__':
    main()
