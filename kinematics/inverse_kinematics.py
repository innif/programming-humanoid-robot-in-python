"""In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
"""
import numpy as np
from numpy import matrix, random, linalg
from scipy.linalg import pinv

from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def extract_from_T(self, trans_matrix: matrix) -> list:
        x = trans_matrix[0, -1]
        y = trans_matrix[1, -1]
        z = trans_matrix[2, -1]
        if trans_matrix[0, 0] == 1:
            angle = np.arctan2(trans_matrix[2, 1], trans_matrix[1, 1])
        elif trans_matrix[1, 1] == 1:
            angle = np.arctan2(trans_matrix[0, 2], trans_matrix[0, 0])
        elif trans_matrix[2, 2] == 1:
            angle = np.arctan2(trans_matrix[0, 1], trans_matrix[0, 0])
        else:
            angle = np.arctan2(trans_matrix[0, 1] + trans_matrix[0, 2], trans_matrix[0, 0])  # Pitch-Yaw Angle

        return [x, y, z, angle]

    def make_joint_dict(self, effector_name, angles) -> dict:
        out = {}
        for key in self.joint_names:
            out[key] = 0
        for key, val in zip(self.chains[effector_name], angles):
            out[key] = val
        return out

    def inverse_kinematics(self, effector_name, transform):
        """solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        """
        joint_chain = self.chains[effector_name]
        links = []
        for joint_name in joint_chain:
            if joint_name.startswith("Head"):
                x, y, z = self.links.get(joint_name)
            elif joint_name.startswith("L"):
                x, y, z = self.links.get(joint_name[1:])
            elif joint_name.startswith("R"):
                x, y, z = self.links.get(joint_name[1:])
                y = -y
            else:
                raise Exception("invalid joint-name")
            links.append((x, y, z))
        num_links = len(links) - 1
        lambda_ = 1

        # Zufälligen Winkelvektor definieren
        joint_angles = random.random(num_links) * 1e-5
        target = matrix(self.extract_from_T(transform)).T  # Zielvektor definieren

        for i in range(1000):
            # Winkelvektor testen
            angle_dict = self.make_joint_dict(effector_name, joint_angles)
            self.forward_kinematics(angle_dict)  # Ts = forward_kinematics(transform, links, theta)  # T0 = root transformation
            Te = [self.transforms[chain_name] for chain_name in self.chains[effector_name]]
            # Transformation des letzten Links auslesen
            endpoint = Te[-1]  # Te = matrix([from_trans(Ts[-1])]).T
            # aus Transformation x,y,z,t auslesen
            result = matrix([self.extract_from_T(endpoint)]).T
            # Fehlervektor berechnen
            e = target - result
            # Fehlerwerte auf max_step constraint
            # e[e > max_step] = max_step
            # e[e < -max_step] = -max_step

            T = matrix([self.extract_from_T(transform_m) for transform_m in Te[1:]]).T
            J = np.zeros(T.shape)
            dT = result - T
            J[0, :] = -dT[2, :]  # x
            J[1, :] = dT[1, :]  # y
            J[2, :] = dT[0, :]  # z
            J[-1, :] = 1  # angular
            d_theta = lambda_ * pinv(J) * e
            joint_angles += np.array(d_theta.T)[0]
            if linalg.norm(d_theta) < 1e-4:
                break
        return joint_angles

    def set_transforms(self, effector_name, transform):
        """solve the inverse kinematics and control joints use the results"""
        # YOUR CODE HERE
        angles = self.inverse_kinematics(effector_name, transform)
        names = self.chains[effector_name][:-1]
        times = [[0, 1]] * len(names)
        keys = [[[a, [3, 0., 0.], [3, 0., 0.]]] * len(times[0]) for a in angles]
        self.keyframes = (names, times, keys)  # the result joint angles have to fill in

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = 0.26
    agent.set_transforms('LLeg', T)
    agent.run()
