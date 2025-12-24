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

print("Number of joints:", model.nq)
print("Number of actuators:", model.nu)

# Run forever (like real hardware)
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
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()