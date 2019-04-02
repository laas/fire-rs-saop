#  Copyright (c) 2019, CNRS-LAAS
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice, this
#  list of conditions and the following disclaimer.
#
#  * Redistributions in binary form must reproduce the above copyright notice,
#  this list of conditions and the following disclaimer in the documentation
#  and/or other materials provided with the distribution.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
#  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
#  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
#  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""UVigo's wildfire alarm client and bridge to ROS"""

import threading
import time

import rospy

import fire_rs.libwden as libwden


class WildfireSensorNode:
    def __init__(self):
        rospy.init_node("wildfire_sensor")
        rospy.loginfo("Starting {}".format(self.__class__.__name__))

        URI = "tcp://localhost"
        SUB_PORT = 4000
        PUB_PORT = 5000

        SOURCE = "0002"
        MESSAGE_TYPE = "01"
        DESTINATION = "0004"
        NOTIFICATION = "Product_available Notification"

        MESSAGE_FILTER = "03"
        SOURCE_FILTER = "0006"

        self.client = libwden.Client()
        self.pub = self.client.add_publisher(URI, str(PUB_PORT))
        self.sub = self.client.add_subscriber(
            URI, str(SUB_PORT), libwden.Client.generate_topic_filter(MESSAGE_FILTER, SOURCE_FILTER))

    def receive(self):
        r = self.sub.receive()
        if r:
            rospy.loginfo("%s", r)

    def test_send(self):
        m = libwden.Message(libwden.MessageType.SAOP_Product_Service.value,
                            libwden.Address.LAAS.value, libwden.Address.UVigo.value,
                            "Hola caracola!")
        self.pub.send(m)


if __name__ == '__main__':
    try:
        wsensor_node = WildfireSensorNode()

        r = rospy.Rate(0.1)
        r.sleep()
        while not rospy.is_shutdown():
            wsensor_node.receive()

    except rospy.ROSInterruptException:
        pass
