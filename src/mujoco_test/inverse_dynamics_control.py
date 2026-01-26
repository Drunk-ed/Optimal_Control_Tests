import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np

NB = 6
parent = [-1, 0, 1, 2, 3, 4]

joint_axis = [
    np.array([0, 0, 1]),
    np.array([0, 1, 0]),
    np.array([0, 1, 0]),
    np.array([1, 0, 0]),
    np.array([0, 1, 0]),
    np.array([1, 0, 0])
]
mass = [4.0, 3.761, 8.058, 2.846, 1.37, 0.365]
com = [
    np.array([0.0, -0.00193, 0.02561]),
    np.array([-0.2125, 0.0, 0.11336]),
    np.array([-0.2422, 0.0, 0.0265]),
    np.array([0.0, -0.01634, -0.0018]),
    np.array([0.0, 0.01634, -0.0018]),
    np.array([0.0, 0.0, -0.001159])

]
'''
inertia = [
    np.diag([0,0,0.00657286]),
    np.diag([0.0, 0.0, 0.33247207]),
    np.diag([0.0, 0.0, 0.07848510]),
    np.diag([0.0, 0.0, 0.00126079]),
    np.diag([0.0, 0.0, 0.00096614]),
    np.diag([0.0, 0.0, 0.00025756])
]
'''

I_shoulder = np.array([
    [0.00700210, 0.00000073, -0.00001053],
    [0.00000073, 0.00648091, 0.00049994],
    [-0.00001053, 0.00049994, 0.00657286]
])

I_upper_arm = np.array([
    [0.01505885, -0.00005400, 0.00000563],
    [-0.00005400, 0.33388086, -0.00000181],
    [0.00000563, -0.00000181, 0.33247207]
])

I_forearm = np.array([
    [0.00399632, -0.00001365, 0.00137272],
    [-0.00001365, 0.07879254, -0.00000660],
    [0.00137272, -0.00000660, 0.07848510]
])

I_wrist1 = np.array([
    [0.00165491, -0.00000282, -0.00000438],
    [-0.00000282, 0.00135962, 0.00010157],
    [-0.00000438, 0.00010157, 0.00126279]
])

I_wrist2 = np.array([
    [0.00135617, -0.00000274, 0.00000444],
    [-0.00000274, 0.00127827, -0.00005048],
    [0.00000444, -0.00005048, 0.00096614]
])

I_wrist3 = np.array([
    [0.00018694, 0.00000006, -0.00000017],
    [0.00000006, 0.00018908, -0.00000092],
    [-0.00000017, -0.00000092, 0.00025756]
])

inertia = [I_shoulder, I_upper_arm, I_forearm, I_wrist1, I_wrist2, I_wrist3]

class Inverse_Dynamics_controller(Node):
    def __init__(self):
        super().__init__('Inverse_Dynamics_Controller')

        self.q = None
        self.dq = None
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
        self.Kp = 15*np.diag([100, 100, 100, 30, 20, 10])
        self.Kd = 30*np.diag([10, 10, 10, 3, 2, 1])

        self.control_timer = self.create_timer(0.01, self.command_callback)

    def pos_callback(self, msg):
        self.q = np.array(msg.data)

    def vel_callback(self, msg1):
        self.dq = np.array(msg1.data)



    def skew(self,v):
        return np.array([
            [0, -v[2], v[1]],
            [v[2], 0, -v[0]],
            [-v[1], v[0], 0]
        ])

    def spatial_cross_motion(self,v):
        w, vlin = v[:3], v[3:]
        return np.block([
            [self.skew(w), np.zeros((3,3))],
            [self.skew(vlin), self.skew(w)]
        ])

    def spatial_cross_force(self,v):
        return -self.spatial_cross_motion(v).T

    def spatial_inertia(self, m, r, Icom):
        C = self.skew(r)
        return np.block([
            [Icom + m * C @ C.T, m * C],
            [m * C.T, m * np.eye(3)]
        ])

    def motion_subspace(self, axis):
        S = np.zeros((6,1))
        S[:3,0] = axis
        return S

    def forward_recursion(self, q, dq, ddq, gravity):
        V = [np.zeros(6) for _ in range(NB)]
        A = [np.zeros(6) for _ in range(NB)]

        A[0][3:] = -gravity

        for i in range(NB):
            S = self.motion_subspace(joint_axis[i])
            vJ = (S * dq[i]).flatten()

            if parent[i] == -1:
                V[i] = vJ
                A[i] = (S*ddq[i]).flatten() + self.spatial_cross_motion(V[i]) @ vJ
            else:
                V[i] = V[parent[i]] + vJ
                A[i] = A[parent[i]] + S.flatten()*ddq[i] + self.spatial_cross_motion(V[i]) @ vJ

        return V, A

    def backward_recursion(self, V, A):
        F = [np.zeros(6) for _ in range(NB)]
        tau = np.zeros(NB)

        for i in reversed(range(NB)):
            I = self.spatial_inertia(mass[i], com[i], inertia[i])

            F[i] = I @ A[i] + self.spatial_cross_force(V[i]) @ (I @ V[i])

            S = self.motion_subspace(joint_axis[i])
            tau[i] = (S.T @ F[i]).item()

            if parent[i] != -1:
                F[parent[i]] += F[i]

        return tau

    def inverse_dynamics(self, q, dq, ddq):
        gravity = np.array([0, 0, -9.81])
        V, A = self.forward_recursion(q, dq, ddq, gravity)
        tau = self.backward_recursion(V, A)
        return tau



    def inverse_dynamics_control(self, q, dq, q_des):
        e = q_des - q
        ddq_des = self.Kp @ e - self.Kd @ dq
        return self.inverse_dynamics(q, dq, ddq_des)

    def command_callback(self):

        if self.q is None or self.dq is None:
            return
        
        if self.q.shape[0] != 6 or self.dq.shape[0] != 6:
            self.get_logger().warn("Waiting for full 6-DOF joint state")
            return
        self.q_des = np.array([0, -1.57, 1.57, 0, 0, 0])

        tau = self.inverse_dynamics_control(self.q, self.dq, self.q_des)


        msg = Float64MultiArray()
        msg.data = tau.tolist()

        #print(f"{error}")
        self.joint_cmd.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Inverse_Dynamics_controller()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    finally:

        #viewer.close()
        node.destroy_node()




if __name__ == '__main__':
    main()
