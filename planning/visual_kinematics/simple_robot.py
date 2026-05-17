import math

class SimpleFrame:
    def __init__(self, matrix=None):
        if matrix is None:
            # Identity 4x4 matrix
            self.matrix = [[1, 0, 0, 0],
                           [0, 1, 0, 0],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]]
        else:
            self.matrix = matrix

    def __mul__(self, other):
        if isinstance(other, SimpleFrame):
            result = [[0]*4 for _ in range(4)]
            for i in range(4):
                for j in range(4):
                    for k in range(4):
                        result[i][j] += self.matrix[i][k] * other.matrix[k][j]
            return SimpleFrame(result)
        else:
            raise TypeError("Multiplication only supported between SimpleFrame instances")

    def inverse(self):
        # Inverse of a homogeneous transform matrix assuming rotation matrix is orthogonal
        R = [row[:3] for row in self.matrix[:3]]
        t = [self.matrix[i][3] for i in range(3)]

        # Transpose of rotation matrix
        R_T = [[R[j][i] for j in range(3)] for i in range(3)]

        # -R_T * t
        t_inv = [-sum(R_T[i][j]*t[j] for j in range(3)) for i in range(3)]

        inv_matrix = [R_T[0] + [t_inv[0]],
                      R_T[1] + [t_inv[1]],
                      R_T[2] + [t_inv[2]],
                      [0, 0, 0, 1]]
        return SimpleFrame(inv_matrix)

    @staticmethod
    def from_dh(d, a, alpha, theta):
        sa = math.sin(alpha)
        ca = math.cos(alpha)
        st = math.sin(theta)
        ct = math.cos(theta)
        matrix = [
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ]
        return SimpleFrame(matrix)

    def translation(self):
        return [self.matrix[i][3] for i in range(3)]

    def __str__(self):
        return '\n'.join(['\t'.join(f"{item:.4f}" for item in row) for row in self.matrix])


class SimpleRobot:
    def __init__(self, dh_params_list):
        """
        dh_params_list: list of tuples (d, a, alpha, theta)
        """
        self.dh_params_list = dh_params_list
        self.num_joints = len(dh_params_list)
        self.joint_angles = [params[3] for params in dh_params_list]

    def forward_kinematics(self, joint_angles=None):
        if joint_angles is None:
            joint_angles = self.joint_angles
        T = SimpleFrame()  # identity
        for i, (d, a, alpha, _) in enumerate(self.dh_params_list):
            theta = joint_angles[i]
            A = SimpleFrame.from_dh(d, a, alpha, theta)
            T = T * A
        return T

    def inverse_kinematics(self, target_position, target_orientation=None):
        """
        Basic inverse kinematics for a 6 DOF robot arm.
        This example assumes a simple 6 DOF arm with standard DH parameters.
        target_position: [x, y, z]
        target_orientation: ignored in this simple implementation
        Returns: list of joint angles (radians)
        Note: This is a simplified geometric IK solution for demonstration.
        """
        if self.num_joints != 6:
            raise NotImplementedError("Inverse kinematics implemented only for 6 DOF robot")

        x, y, z = target_position

        # Extract DH parameters for convenience
        d = [param[0] for param in self.dh_params_list]
        a = [param[1] for param in self.dh_params_list]
        alpha = [param[2] for param in self.dh_params_list]

        # Simplified IK assuming first three joints position the wrist center
        # and last three joints orient the end effector (ignored here)

        # Calculate wrist center position (ignoring orientation)
        # For simplicity, assume wrist center = target_position - d6 along z axis
        d6 = d[5]
        wx = x
        wy = y
        wz = z - d6

        # Calculate joint 1 angle
        theta1 = math.atan2(wy, wx)

        # Calculate distances for triangle formed by joints 2,3 and wrist center
        r = math.sqrt(wx**2 + wy**2)
        s = wz - d[0]

        # Lengths of links
        L2 = a[1]
        L3 = a[2]

        # Law of cosines for joint 3
        D = (r**2 + s**2 - L2**2 - L3**2) / (2 * L2 * L3)
        if abs(D) > 1:
            raise ValueError("Target is out of reach")

        theta3 = math.acos(D)

        # Calculate joint 2 angle
        theta2 = math.atan2(s, r) - math.atan2(L3 * math.sin(theta3), L2 + L3 * math.cos(theta3))

        # For joints 4,5,6 (wrist orientation), set to zero for simplicity
        theta4 = 0.0
        theta5 = 0.0
        theta6 = 0.0

        return [theta1, theta2, theta3, theta4, theta5, theta6]

    def print_forward_kinematics(self, joint_angles=None):
        T = self.forward_kinematics(joint_angles)
        print("Forward Kinematics Transformation Matrix:")
        print(T)
