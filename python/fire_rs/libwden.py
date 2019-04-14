#!/usr/bin/env python
# -*- coding: utf-8 -*-

# (c) Copyright 2019 University of vigo. All rights reserved.


import zmq
import binascii


def init_wden():
    return zmq.Context(1)

def gen_publisher(context,URI,SUB_PORT):
    publisher = context.socket(zmq.PUB)
    publisher.connect(str(URI+":"+str(SUB_PORT)))
    return publisher

def gen_subscriber(context, URI, PORT):
    socket = context.socket(zmq.SUB)
    socket.connect(str(URI+":"+str(PORT)))
    socket.setsockopt_string(zmq.SUBSCRIBE,"")
    return socket

def gen_subscriber_wfilter(context, URI, PORT, topic_filter):
    socket = context.socket(zmq.SUB)
    socket.connect(str(URI+":"+str(PORT)))
    socket.setsockopt_string(zmq.SUBSCRIBE, topic_filter)
    return socket

def send_message(socket, message_type, source, destination, notification):
    socket.send(message_type+destination+source+notification)   
    return

def receive_message(socket):
    message = socket.recv()
    return message

def generate_topic_filter(message_filter, destination_filter=None, source_filter=None):
    topic_filter = None

    if message_filter:
        topic_filter = message_filter
        
        if destination_filter:
            topic_filter += destination_filter
            
            if source_filter:
                topic_filter += source_filter
    
    return topic_filter.decode('utf-8')
