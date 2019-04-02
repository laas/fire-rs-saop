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

"""Wildfire Data Exchange Network library"""

import enum
import typing as ty

import zmq


@enum.unique
class MessageType(enum.Enum):
    M2M_Service = "01"
    UAV_Product_Service = "02"
    SAOP_Product_Service = "03"


@enum.unique
class Address(enum.Enum):
    UVigo = "0002"
    UPorto = "0004"
    LAAS = "0008"


class Message:
    def __init__(self, message_type: str, source: str, destination: str, notification: str):
        self.type = message_type
        self.source = source
        self.destination = destination
        self.notification = notification

    def encoded(self) -> str:
        return str(self.type + self.destination + self.source + " " + self.notification)

    def __repr__(self):
        return "Message({}, {}, {}, {})".format(
            self.type, self.source, self.destination, self.notification)

    def __str__(self):
        return "{} from {} to {}: {}".format(MessageType(self.type).name, Address(self.source).name,
                                             Address(self.destination).name, self.notification)


class Publisher:
    def __init__(self, context: zmq.Context, address: str):
        self.address = address
        self.context = context
        self.socket = self.context.socket(zmq.PUB)
        self.socket.connect(self.address)

    def send(self, message: Message):
        self.socket.send_string(message.encoded())


class Subscriber:
    def __init__(self, context: zmq.Context, address: str, topic_filter=""):
        self.address = address
        self.context = context
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(self.address)
        self.socket.setsockopt_string(zmq.SUBSCRIBE, topic_filter)

    def receive(self, blocking=False) -> ty.Optional[str]:
        flag = 0
        if not blocking:
            flag = zmq.NOBLOCK
        message = None
        try:
            message = self.socket.recv(flags=flag).decode("utf-8")
        except zmq.ZMQError as e:
            pass
        return message


class Client:
    def __init__(self, n=1):
        self.context = zmq.Context(n)
        self.publishers = {}
        self.subscribers = {}

    def add_publisher(self, uri: str, port: str) -> Publisher:
        key = Client.address(uri, port)
        self.publishers[key] = Publisher(self.context, Client.address(uri, port))
        return self.publishers[key]

    def add_subscriber(self, uri: str, port: str, topic_filter: str = "") -> Subscriber:
        key = Client.address(uri, port)
        self.subscribers[key] = Subscriber(self.context, key, topic_filter)
        return self.subscribers[key]

    @staticmethod
    def address(uri: str, sub_port: str):
        return ":".join((uri, sub_port))

    @staticmethod
    def generate_topic_filter(message_filter, source_filter=None, destination_filter=None):
        topic_filter = ""
        if message_filter:
            topic_filter = message_filter
            if source_filter:
                topic_filter += source_filter
                if destination_filter:
                    topic_filter += destination_filter
        return topic_filter
