#! /usr/bin/env python3
import pymorse
import base64
import numpy as np
import matplotlib.pyplot as plt
from functools import partial

im = plt.figure()
image_number = 0


def image_printer(data):
    global image_number
    # print("Incoming data! " + str(data))
    image = np.frombuffer(base64.b64decode(data["image"]), dtype=np.int8).reshape(480, 640)
    print(data)
    im.gca().imshow(image, cmap='Greys')
    im.savefig("image%d.png" % image_number)
    image_number += 1


def pose_printer(name, data):
    print(name, " Pose: ", str(data))


def done(a):
    print("done: ", a)


with pymorse.Morse("localhost", 4000) as simu:
    try:
        # Get the 'Pose' sensor datastream
        # pose = simu.drone.pose
        # cam_image = simu.drone.ircam
        # cam_pose = simu.drone.ircam_pose

        # Start the motion. It may take several seconds before finishing
        # The line below is however non-blocking
        for i in range(0, 20):
            goto_action = simu.drone.motion.translate(100, 0, 0)
            a = simu.drone.ircam.capture(1)
            b = simu.drone.ircam.get()
            image_printer(b)
        # Register a callback to know when the action is done

        # Blocks until something is available
        # print(cam_image.get())
        # print(cam_pose.get())
        #
        # # Asynchronous read: the following line do not block.
        # cam_image.subscribe(image_printer)
        # cam_pose.subscribe(partial(pose_printer, "cam"))
        # pose.subscribe(partial(pose_printer, "drone"))

        # Read for 10 sec
        simu.sleep(30)

    except Exception as e:
        print('Oups! An error occured!')
        print(e)
