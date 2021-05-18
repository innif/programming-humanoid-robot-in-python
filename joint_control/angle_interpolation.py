'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import wipe_forehead as keyframe
from software_installation.spark_agent import Perception

class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.start_time = None

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception: Perception):
        target_joints = {}

        name_list, time_list, key_list = keyframes

        if self.start_time is None:
            self.start_time = perception.time
        time_now = perception.time - self.start_time

        finished_joints = []

        for name_index, name in enumerate(name_list):  # for each joint
            times = time_list[name_index]
            keys = key_list[name_index]

            if time_now > times[-1]:
                finished_joints.append(name)
                continue

            current_time_index = 0
            time_next = 0
            time_prev = 0
            for j, time in enumerate(times):
                if time > time_now:
                    current_time_index = j
                    time_next = time
                    break
                time_prev = time

            p0 = perception.joint.get(name)
            if p0 is None:
                p0 = 0
            p1 = p0

            if current_time_index > 0:  # [float angle, Handle1, Handle2]
                p0 = keys[current_time_index - 1][0]  # start-point
                p1 = p0 + keys[current_time_index - 1][2][2]  # start-point Handle angle

            p3 = keys[current_time_index][0]  # dest-point
            p2 = p3 + keys[current_time_index][1][2]  # dest-point Handle angle

            t_diff = time_next - time_prev
            i = (time_now - time_prev) / t_diff

            b = 0
            b += (1-i)**3 * p0
            b += (1-i)**2 * 3*i*p1
            b += (1-i) * 3*(i**2)*p2
            b += (i**3)*p3

            target_joints[name] = b

        return target_joints


if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = keyframe()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
