#!/usr/bin/env python
import numpy

import control_msgs.msg
import rospy

import actionlib
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from slipping_control_py.gripper import Gripper


class GiskardDonbotFingerHack(object):

    def __init__(self):

        self.vel_thr = 0.01
        self.wait_for_motion_vel_thr = 0.001
        self.position_thr = 0.01
        self.min_slipping_avoidance_duration = 0.9

        rospy.init_node('giskard_donbot_finger_hack')

        self.topic_joint_state = '/body/joint_states'

        self.gripper = Gripper(False)

        # Input Action
        self._as = actionlib.SimpleActionServer('/whole_body_controller/follow_joint_trajectory',
                                                control_msgs.msg.FollowJointTrajectoryAction,
                                                execute_cb=self.callback, auto_start=False)
        self._as.register_preempt_callback(self.preempt_cb)
        self._as.start()

        # Output Action
        self._ac = actionlib.SimpleActionClient('/whole_body_controller/delayed_follow_joint_trajectory',
                                                control_msgs.msg.FollowJointTrajectoryAction)

        rospy.sleep(1)
        rospy.loginfo('HACK: Waiting for giskard Server')
        self._ac.wait_for_server()

        self.success = False

        self.pubdbg = rospy.Publisher('/hack', String, queue_size=1)

        rospy.loginfo('HACK: Initialized')
        rospy.spin()

    def preempt_cb(self):

        self._ac.cancel_goal()
        self._as.set_preempted()
        self.success = False

    def callback(self, goal):
        self.success = True

        rospy.loginfo('=======================\n')
        rospy.loginfo('HACK: received goal')

        grasped = self.checkGrasped()

        if grasped:
            timestamps, action_type = self.computeSlippingAvoidanceActivations(goal)
            position, _ = self.getJointState()

        self._ac.send_goal(goal, done_cb=None, active_cb=None, feedback_cb=self.feedback_cb)

        if grasped:
            rospy.loginfo('HACK GOAL SENT:')
            rospy.loginfo(timestamps)
            rospy.loginfo(action_type)
            time_zero = self.waitForMotion(position, self.getFinalPos(goal))
            self.executeActions(time_zero, timestamps, action_type)

        # Wait for result
        self._ac.wait_for_result()

        if self.success:
            result = self._ac.get_result()
            if result and result.error_code != control_msgs.msg.FollowJointTrajectoryResult.SUCCESSFUL:
                rospy.logerr(u'didn\'t receive successful from {} {}'.format(self._ac.action_client.ns, result))
            self._as.set_succeeded(result)
        else:
            self._as.set_succeeded()

    def getFinalPos(self, goal):
        """
        :type goal: control_msgs.msg._FollowJointTrajectoryGoal.FollowJointTrajectoryGoal
        :return:
        """
        j_names = [u'ur5_shoulder_pan_joint',
                   u'ur5_shoulder_lift_joint',
                   u'ur5_elbow_joint',
                   u'ur5_wrist_1_joint',
                   u'ur5_wrist_2_joint',
                   u'ur5_wrist_3_joint'
                   ]
        last_point = goal.trajectory.points[-1]
        position = []
        for name in j_names:
            i = goal.trajectory.joint_names.index(name)
            position.append(last_point.positions[i])
        return position

    def feedback_cb(self, feedback):
        self._as.publish_feedback(feedback)

    def checkGrasped(self):
        return self.gripper.is_grasped()

    def computeSlippingAvoidanceActivations(self, goal):
        """
        :type goal: control_msgs.msg._FollowJointTrajectoryGoal.FollowJointTrajectoryGoal
        :return:
        """
        refills_finger_joint_index = goal.trajectory.joint_names.index(u'refills_finger_joint')
        # First action is pivoting
        timestamps = []
        action_type = []
        for point in goal.trajectory.points:
            finger_vel = point.velocities[refills_finger_joint_index]
            if abs(finger_vel) <= self.vel_thr and (len(timestamps) == 0 or action_type[-1] == 'pivoting'):
                timestamps.append(point.time_from_start)
                action_type.append('slipping_avoidance')
            else:
                if abs(finger_vel) > self.vel_thr and (len(timestamps) > 0 and action_type[-1] == 'slipping_avoidance'):
                    timestamps.append(point.time_from_start)
                    action_type.append('pivoting')
        # Last action is pivoting
        timestamps.append(goal.trajectory.points[-1].time_from_start)
        action_type.append('pivoting')
        # add element with Duration(0) it id does not exist
        if timestamps[0] > rospy.Duration.from_sec(0.0):
            timestamps.insert(0, rospy.Duration.from_sec(0.0))
            action_type.insert(0, 'pivoting')

        # Remove period where slipping_avoidance is too short
        i = 0
        while i < (len(timestamps)-1):
            if action_type[i] == 'slipping_avoidance':
                # check duration
                duration_ = timestamps[i+1] - timestamps[i]
                if duration_ < rospy.Duration.from_sec(self.min_slipping_avoidance_duration):
                    rospy.logwarn('HACK: removing short slipping_avoidance')
                    timestamps.pop(i)
                    action_type.pop(i)
            i = i+1

        # Find duplicated
        i = 0
        while i < (len(action_type)-1):
            if action_type[i] == action_type[i+1]:
                timestamps.pop(i+1)
                action_type.pop(i+1)
            else:
                i = i+1

        # Last check maybe useless
        if len(action_type) == 1:
            if action_type[0] == 'slipping_avoidance':
                rospy.logerr('HACK: Only one action that is slipping_avoidance! change to pivoting...')
                action_type[0] = 'pivoting'
        elif len(action_type) == 0:
            rospy.logerr('HACK: NO ACTION AFTER THE ALGORITHM... I should not be here...')
            timestamps.insert(0, rospy.Duration.from_sec(0.0))
            action_type.insert(0, 'pivoting')

        return timestamps, action_type

    def getJointState(self):
        joint_state = rospy.wait_for_message(self.topic_joint_state, JointState)  # type: JointState
        # Get only the arm values
        j_names = [ u'ur5_shoulder_pan_joint',
                    u'ur5_shoulder_lift_joint',
                    u'ur5_elbow_joint',
                    u'ur5_wrist_1_joint',
                    u'ur5_wrist_2_joint',
                    u'ur5_wrist_3_joint'
                  ]
        position = []
        velocity = []
        for name in j_names:
            i = joint_state.name.index(name)
            position.append(joint_state.position[i])
            velocity.append(joint_state.velocity[i])
        return position, velocity

    def waitForMotion(self, position, final_j_position):

        rospy.loginfo('HACK: wait for motion')
        _, velocity = self.getJointState()
        while(numpy.linalg.norm(numpy.array(position) - numpy.array(final_j_position))
              > self.position_thr
              and
              numpy.linalg.norm(velocity) < self.wait_for_motion_vel_thr):
            _, velocity = self.getJointState()

        rospy.loginfo('HACK: wait for motion END')

        return rospy.Time.now()

    def executeActions(self, time_zero, time_from_starts, actions_type):

        time_now = rospy.Time.now()
        last_time = time_zero + time_from_starts[-1]
        next_index = 0
        next_time = time_zero + time_from_starts[next_index]
        last_action = ''

        # While not end
        while time_now < last_time:

            # check if now is less than nex
            if time_now < next_time:
                # in this case i can wait for the nex_time
                print("HACK: sleeping for ", (next_time - time_now).to_sec())
                rospy.sleep(next_time - time_now)
                # now i can activate the action
                rospy.loginfo('HACK: Execution of action %i', next_index)
                self.executeAction(actions_type[next_index])
                last_action = actions_type[next_index]
                # prepare for nex while iteration
                next_index = next_index + 1
                # if there are no more action -> break the while
                if next_index >= len(time_from_starts):
                    break
                next_time = time_zero + time_from_starts[next_index]
                print("Hack: next_time= ", next_time.to_sec())

            # if time_now is > than next_time
            else:
                # update the next_index (find the next time)
                for jj in range(next_index+1, len(time_from_starts)):
                    rospy.logwarn("HACK: Missed action %i", jj-1)
                    next_time = time_zero + time_from_starts[jj]
                    print("Hack: next_time= %i", next_time.to_sec())
                    if time_now < next_time:
                        # ok found
                        next_index = jj
                        break
                    # check if arrived at the end
                    if jj == len(time_from_starts)-1:
                        # end of all actions!
                        rospy.logwarn('HACK: End of all actions... with delay. I should not be here...')
                        next_index = jj
                # make sure to apply the action now
                rospy.loginfo('HACK: Execution of action %i', next_index-1)
                self.executeAction(actions_type[next_index-1])
                last_action = actions_type[next_index]
                rospy.logwarn('HACK: Action executed with delay')

            # Update variables for the next while cycle
            time_now = rospy.Time.now()

        # Last action must be pivoting?
        if last_action != 'pivoting':
            rospy.loginfo('HACK: Execution of last pivoting action')
            self.executeAction('pivoting')

    def executeAction(self, action_str):
        if action_str == 'pivoting':
            self.pubdbg.publish('GripperPivoting')
            rospy.loginfo('HACK: GripperPivoting')
            self.gripper.gripper_pivoting()
        elif action_str == 'slipping_avoidance':
            self.pubdbg.publish('SlippingAvoidance')
            rospy.loginfo('HACK: SlippingAvoidance')
            self.gripper.slipping_avoidance()
        else:
            raise Exception('Invalid action_str!')


if __name__ == '__main__':
    h = GiskardDonbotFingerHack()
