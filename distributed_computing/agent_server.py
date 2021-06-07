'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import threading

from xmlrpc.server import SimpleXMLRPCServer
import os
import sys
import time

sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from kinematics.inverse_kinematics import InverseKinematicsAgent


class ServerAgent(InverseKinematicsAgent):
    """ ServerAgent provides RPC service """

    # YOUR CODE HERE
    def __init__(self):
        pass
        self.server = SimpleXMLRPCServer(("localhost", 8080), allow_none=True)
        self.server.register_function(self.get_angle, "get_angle")
        self.server.register_function(self.set_angle, "set_angle")
        self.server.register_function(self.execute_keyframes, "execute_keyframes")
        self.server.register_function(self.get_transform, "get_transform")
        self.server.register_function(self.set_transform, "set_transform")
        self.server.register_function(self.get_posture, "get_posture")
        self.server.register_function(self.hello, "hello")
        self.server_thread = threading.Thread(target=self.server.serve_forever)
        self.server_thread.start()
        super(ServerAgent, self).__init__()

    def hello(self):
        print("hello")
        return "world"

    def get_angle(self, joint_name):
        """get sensor value of given joint"""
        try:
            return self.perception.joint[joint_name]
        except Exception as e:
            print(e)
            return "error"

    def set_angle(self, joint_name, angle):
        """set target angle of joint for PID controller"""
        try:
            self.target_joints[joint_name] = angle
            return "success"
        except Exception as e:
            print(e)
            return "error"

    def get_posture(self):
        """return current posture of robot"""
        try:
            return self.recognize_posture(self.perception)
        except Exception as e:
            print(e)
            return "error"

    def execute_keyframes(self, keyframes):
        """ execute keyframes, note this function is blocking call, e.g. return until keyframes are executed """
        self.start_time = None
        self.keyframe_running = True
        self.keyframes = keyframes
        while self.keyframe_running:
            time.sleep(0.01)

    def get_transform(self, name):
        """ get transform with given name """
        try:
            return self.transforms[name]
        except Exception as e:
            print(e)
            return "error"

    def set_transform(self, effector_name, transform):
        """ solve the inverse kinematics and control joints use the results """
        try:
            super(ServerAgent, self).set_transform(effector_name, transform)
            return "success"
        except Exception as e:
            print(e)
            return "error"


if __name__ == '__main__':
    agent = ServerAgent()
    agent.run()
