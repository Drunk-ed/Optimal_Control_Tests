import mujoco
import time
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from mujoco.viewer import launch_passive


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


print("Number of joints:", model.nq)
print("Number of actuators:", model.nu)


while True:
    # ---- READ JOINT STATES (THIS IS THE SUBSCRIBE PART) ----
    q  = data.qpos.copy()   # joint positions
    dq = data.qvel.copy()   # joint velocities

    print("q:", np.round(q, 3))
    print("dq:", np.round(dq, 3))
    print("-" * 40)

    # ---- STEP SIMULATION ----
    
'''



class Mujoco_Bridge(Node):
    def __init__(self):
        super().__init__('phantom_Button')



        self.joint_pos = self.create_publisher(
            Float64MultiArray,
            '/joint_states/pos',
            10
        )

        self.joint_vel = self.create_publisher(
            Float64MultiArray,
            '/joint_states/vel',
            10
        )

        self.sub = self.create_subscription(
            Float64MultiArray,
            '/joint_torque/cmd',
            self.command_callback,
            10
        )

        self.timer = self.create_timer(0.01, self.pos_callback)
        self.timer1 = self.create_timer(0.01, self.vel_callback)
        

    '''
    def publish_cmd(self):
        msg = Float64MultiArray()
        q  = data.qpos.copy()   # joint positions
        msg.data = q.tolist()
        self.joint_pos.publish(msg)
        self.get_logger().info("Pubhisher is all good")


        


    def publish1_cmd(self):
        msg1 = Float64MultiArray() 
        dq = data.qvel.copy()   # joint velocities
        msg1.data = dq.tolist()
        self.joint_pos.publish(msg1)
        self.get_logger().info("Publisher1 is all good")

        '''

    def pos_callback(self):

        msg = Float64MultiArray()
        q  = data.qpos.copy()   # joint positions
        msg.data = q.tolist()
        self.joint_pos.publish(msg)



    def vel_callback(self):
        msg1 = Float64MultiArray()
        dq = data.qvel.copy()   # joint velocities
        msg1.data = dq.tolist()
        self.joint_vel.publish(msg1)



    def command_callback(self,msg2):
        tau = msg2.data

        data.ctrl[:] = tau
        viewer.sync()
        mujoco.mj_step(model, data)
        time.sleep(dt)


def main(args=None):
    rclpy.init(args=args)
    node = Mujoco_Bridge()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    finally:

        viewer.close()
        node.destroy_node()


 

if __name__ == '__main__':
    main()
