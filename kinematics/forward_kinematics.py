'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    the local_trans has to consider different joint axes and link parameters for different joints
'''

# add PYTHONPATH
import math
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from numpy.matlib import matrix, identity

from joint_control.recognize_posture import PostureRecognitionAgent


class ForwardKinematicsAgent(PostureRecognitionAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        self.chains = {'Head': ['HeadYaw', 'HeadPitch'],
                       'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowRoll', 'LElbowYaw'],
                       'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
                       'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll'],
                       'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll']
                       }
        self.links = {"Head-Offset": (0, 0, 126.5),
                      "HeadYaw": (0, 0, 0),
                      "HeadPitch": (0, 0, 0),

                      "Arm-Offset": (0, 0, 126.5),
                      "ShoulderPitch": (0, 0, 0),
                      "ShoulderRoll": (105, 15, 0),
                      "ElbowYaw": (0, 0, 0),
                      "ElbowRoll": (55.95, 0, 0),

                      "Leg-Offset": (0, 50, -85),
                      "HipYawPitch": (0, 0, 0),
                      "HipRoll": (0, 0, 0),
                      "HipPitch": (0, 0, -100),
                      "KneePitch": (0, 0, -102.9),
                      "AnklePitch": (0, 0, 0),
                      "AnkleRoll": (0, 0, 0)}

    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def local_trans(self, joint_name, joint_angle):
        """calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        """
        if joint_name.startswith("Head"):
            x, y, z = self.links.get(joint_name)
        elif joint_name.startswith("L"):
            x, y, z = self.links.get(joint_name[1:])
        elif joint_name.startswith("R"):
            x, y, z = self.links.get(joint_name[1:])
            y = -y
        else:
            return identity(4)  # error
        T = matrix([[0, 0, 0, 0],
                    [0, 0, 0, 0],
                    [0, 0, 0, 0],
                    [x, y, z, 1]], dtype='float64')

        cos = math.cos(joint_angle)
        sin = math.sin(joint_angle)

        dual_axis: bool = "YawPitch" in joint_name

        if "Roll" in joint_name:
            # X-Transform
            m = [[1,   0,    0, 0],
                 [0, cos, -sin, 0],
                 [0, sin,  cos, 0],
                 [0,   0,    0, 0]]
            M = matrix(m)
            if dual_axis:
                M = M/2
            T += M
        if "Pitch" in joint_name:
            # Y-Transform
            m = [[ cos, 0, sin, 0],
                 [   0, 1,   0, 0],
                 [-sin, 0, cos, 0],
                 [   0, 0,   0, 0]]
            M = matrix(m)
            if dual_axis:
                M = M / 2
            T += M
        if "Yaw" in joint_name:
            # Z-Transform
            m = [[ cos, sin, 0, 0],
                 [-sin, cos, 0, 0],
                 [   0,   0, 1, 0],
                 [   0,   0, 0, 0]]
            M = matrix(m)
            if dual_axis:
                M = M / 2
            T += M
        return T

    def forward_kinematics(self, joints):
        """forward kinematics

        :param joints: {joint_name: joint_angle}
        """
        for chain_name, chain_joints in self.chains.items():
            T = identity(4)

            for joint in chain_joints:
                angle = joints[joint]
                Tl = self.local_trans(joint, angle)
                T = T*Tl

                self.transforms[joint] = T


if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
