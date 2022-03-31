#!/usr/bin/env python
"""
BSD 3-Clause License
Copyright (c) 2022, Mohamed Abdelkader Zahana
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

"""
ROS driver for the ZLAC8030L motor controller
Reuquires: ZALAC8030L_CAN_controller, https://github.com/mzahana/ZLAC8030L_CAN_controller
"""

from time import time
import rospy
import tf
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from ZLAC8030L_CAN_controller.canopen_controller import MotorController
from differential_drive import DiffDrive
from zlac8030l_ros.msg import State

class Driver:
    def __init__(self):
        self._can_channel = rospy.get_param("~can_channel", "can0")
        self._bus_type = rospy.get_param("~bus_type", "socketcan")
        self._bitrate = rospy.get_param("~bitrate", 500000)
        self._eds_file = rospy.get_param("~eds_file","")
        # self._wheel_ids = rospy.get_param("~wheel_ids", []) # TODO needs checking
        self._wheel_ids = {"fl":1, "bl":2, "br":3, "fr":4}
        self._flip_direction = {self._wheel_ids["fl"]: -1, self._wheel_ids["bl"]: -1, self._wheel_ids["br"]: 1, self._wheel_ids["fr"]: 1}
        
        self._wheel_radius = rospy.get_param("~wheel_radius", 0.194)

        self._track_width = rospy.get_param("~track_width", 0.8)

        self._max_vx = rospy.get_param("~max_vx", 2.0)
        self._max_w = rospy.get_param("~max_w", 1.57)

        self._odom_frame = rospy.get_param("~odom_frame", "odom_link")
        self._robot_frame = rospy.get_param("~robot_frame", "base_link")

        self._loop_rate = rospy.get_param("~loop_rate", 100.0)

        self._cmd_timeout = rospy.get_param("~cmd_timeout", 0.1)

        self._diff_drive = DiffDrive(self._wheel_radius, self._track_width)

        # last time stamp. Used in odometry calculations
        self._last_odom_dt = time()

        # Last time a velcoity command was received
        self._last_cmd_t = time()

        try:
            self._network = MotorController(channel=self._can_channel, bustype=self._bus_type, bitrate=self._bitrate, node_ids=None, debug=True, eds_file=self._eds_file)
        except Exception as e:
            rospy.logerr("Could not create CAN network object. Error: %s", e)
            exit(0)

        rospy.logwarn("\n ** cmd_vel must be published at rate more than %s Hz ** \n", 1/self._cmd_timeout)

        # ------------------- Subscribers ----------------#
        rospy.Subscriber("cmd_vel", Twist, self.cmdVelCallback)

        # ------------------- Publishers ----------------#
        self._odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
        self._vel_pub = rospy.Publisher("forward_vel", Float64, queue_size=10)

        self._fr_state_pub = rospy.Publisher("front_right_motor/state", State, queue_size=10)
        self._br_state_pub = rospy.Publisher("back_right_motor/state", State, queue_size=10)
        self._fl_state_pub = rospy.Publisher("front_left_motor/state", State, queue_size=10)
        self._bl_state_pub = rospy.Publisher("back_left_motor/state", State, queue_size=10)

        # ------------------- Services ----------------#

        # TF broadcaster
        self._tf_br = tf.TransformBroadcaster()

        rospy.loginfo("** Driver initialization is done **\n")

    def rpmToRps(self, rpm):
        return rpm / 9.5493

    def rpsToRpm(self, rad):
        return rad * 9.5493

    def cmdVelCallback(self, msg):
        self._last_cmd_t = time()

        sign_x = -1 if msg.linear.x <0 else 1
        sign_w = -1 if msg.angular.z <0 else 1
        
        vx = msg.linear.x
        w = msg.angular.z

        if (abs(vx) > self._max_vx):
            rospy.logwarn_throttle(1, "Commanded linear velocity %s is more than maximum magnitude %s", vx, self._max_vx)
            vx = sign_x * self._max_vx
        if (abs(w) > self._max_w):
            rospy.logwarn_throttle(1, "Commanded angular velocity %s is more than maximum magnitude %s", w, self._max_w)
            w = sign_w * self._max_w

        # Compute wheels velocity commands [rad/s]
        (wl, wr) = self._diff_drive.calcWheelVel(vx,w)

        # TODO convert rad/s to rpm
        wl_rpm = self.rpsToRpm(wl)
        wr_rpm = self.rpsToRpm(wr)

        # Send target velocity to the controller
        try:
            self._network.setVelocity(node_id=self._wheel_ids["fl"], vel=wl_rpm * self._flip_direction[self._wheel_ids["fl"]])
            self._network.setVelocity(node_id=self._wheel_ids["bl"], vel=wl_rpm * self._flip_direction[self._wheel_ids["bl"]])
            self._network.setVelocity(node_id=self._wheel_ids["br"], vel=wr_rpm * self._flip_direction[self._wheel_ids["br"]])
            self._network.setVelocity(node_id=self._wheel_ids["fr"], vel=wr_rpm * self._flip_direction[self._wheel_ids["fr"]])
        except Exception as e:
            rospy.logerr_throttle(1, "Error in setting wheel velocity: %s", e)

    def pubOdom(self):
        """Computes & publishes odometry msg
        """
        try:
            v_dict = self._network.getVelocity(node_id=self._wheel_ids["fl"])
            vel = v_dict['value']* self._flip_direction[self._wheel_ids["fl"]]
            self._diff_drive._fl_vel = self.rpmToRps(vel)

            v_dict = self._network.getVelocity(node_id=self._wheel_ids["bl"])
            vel = v_dict['value']* self._flip_direction[self._wheel_ids["bl"]]
            self._diff_drive._bl_vel = self.rpmToRps(vel)

            v_dict = self._network.getVelocity(node_id=self._wheel_ids["br"])
            vel = v_dict['value']* self._flip_direction[self._wheel_ids["br"]]
            self._diff_drive._br_vel = self.rpmToRps(vel)
            
            v_dict = self._network.getVelocity(node_id=self._wheel_ids["fr"])
            vel = v_dict['value']* self._flip_direction[self._wheel_ids["fr"]]
            self._diff_drive._fr_vel = self.rpmToRps(vel)

            now = time()

            dt= now - self._last_odom_dt
            self._last_odom_dt = now

            odom = self._diff_drive.calcRobotOdom(dt)

            msg = Odometry()

            time_stamp = rospy.Time.now()
            msg.header.stamp = time_stamp

            msg.pose.pose.position.x = odom["x"]
            msg.pose.pose.position.y = odom["y"]
            msg.pose.pose.position.z = 0.0
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, odom['yaw'])
            msg.pose.pose.orientation.x = odom_quat[0]
            msg.pose.pose.orientation.y = odom_quat[1]
            msg.pose.pose.orientation.z = odom_quat[2]
            msg.pose.pose.orientation.w = odom_quat[3]
            msg.twist.twist.linear.x = odom['x_dot']
            msg.twist.twist.linear.y = odom['y_dot']
            msg.twist.twist.angular.z = odom['w']
            self._odom_pub.publish(msg)
            # Send TF
            self._tf_br.sendTransform((odom['x'],odom['y'],0),odom_quat,time_stamp,self._robot_frame,self._odom_frame)

            msg = Float64()
            msg.data = odom["v"] # Forward velocity
            self._vel_pub.publish(msg)
        except :
            rospy.logerr_throttle(1, "Error in pubOdom: check if all 4 motors are connected")
            rospy.logerr_throttle(1, "Availabled nodes = %s", self._network._network.scanner.nodes)

    def pubMotorState(self):
        # Front right motor
        msg = State()
        msg.header.stamp = rospy.Time.now()
        msg.node_id = self._wheel_ids["fr"]
        try:
            volts_dict = self._network.getVoltage(self._wheel_ids["fr"])
            volts = volts_dict['value']
            msg.voltage = volts
        except:
            pass

        try:
            curr_dict = self._network.getMotorCurrent(self._wheel_ids["fr"])
            curr = curr_dict['value']
            msg.current = curr
        except:
            pass

        try:
            err_dict = self._network.getErrorCode(self._wheel_ids["fr"])
            code = err_dict['value']
            msg.error_code = code
        except:
            pass

        self._fr_state_pub.publish(msg)

        # Front left motor
        msg = State()
        msg.header.stamp = rospy.Time.now()
        msg.node_id = self._wheel_ids["fl"]
        try:
            volts_dict = self._network.getVoltage(self._wheel_ids["fl"])
            volts = volts_dict['value']
            msg.voltage = volts
        except:
            pass

        try:
            curr_dict = self._network.getMotorCurrent(self._wheel_ids["fl"])
            curr = curr_dict['value']
            msg.current = curr
        except:
            pass

        try:
            err_dict = self._network.getErrorCode(self._wheel_ids["fl"])
            code = err_dict['value']
            msg.error_code = code
        except:
            pass

        self._fl_state_pub.publish(msg)

        # Back right motor
        msg = State()
        msg.header.stamp = rospy.Time.now()
        msg.node_id = self._wheel_ids["br"]
        try:
            volts_dict = self._network.getVoltage(self._wheel_ids["br"])
            volts = volts_dict['value']
            msg.voltage = volts
        except:
            pass

        try:
            curr_dict = self._network.getMotorCurrent(self._wheel_ids["br"])
            curr = curr_dict['value']
            msg.current = curr
        except:
            pass

        try:
            err_dict = self._network.getErrorCode(self._wheel_ids["br"])
            code = err_dict['value']
            msg.error_code = code
        except:
            pass

        self._br_state_pub.publish(msg)

        # Back left motor
        msg = State()
        msg.header.stamp = rospy.Time.now()
        msg.node_id = self._wheel_ids["bl"]
        try:
            volts_dict = self._network.getVoltage(self._wheel_ids["bl"])
            volts = volts_dict['value']
            msg.voltage = volts
        except:
            pass

        try:
            curr_dict = self._network.getMotorCurrent(self._wheel_ids["bl"])
            curr = curr_dict['value']
            msg.current = curr
        except:
            pass

        try:
            err_dict = self._network.getErrorCode(self._wheel_ids["bl"])
            code = err_dict['value']
            msg.error_code = code
        except:
            pass

        self._bl_state_pub.publish(msg)


    def mainLoop(self):
        rate = rospy.Rate(self._loop_rate)

        while not rospy.is_shutdown():
            now = time()

            dt = now - self._last_cmd_t
            if (dt > self._cmd_timeout):
                # set zero velocity
                try:
                    self._network.setVelocity(node_id=self._wheel_ids["fl"], vel=0)
                    self._network.setVelocity(node_id=self._wheel_ids["bl"], vel=0)
                    self._network.setVelocity(node_id=self._wheel_ids["br"], vel=0)
                    self._network.setVelocity(node_id=self._wheel_ids["fr"], vel=0)
                except Exception as e:
                    rospy.logerr_throttle(1, "[mainLoop] Error in setting wheel velocity: %s", e)

            # Publish wheel odom
            self.pubOdom()
            # Publish Motors state
            self.pubMotorState()
            
            rate.sleep()
  

if __name__ == "__main__":
    rospy.init_node("** Motor driver node started ** \n",anonymous=True)
    try:
        driver = Driver()

        driver.mainLoop()
    except rospy.ROSInterruptException:
        driver._network.disconnectNetwork()