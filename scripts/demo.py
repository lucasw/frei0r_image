#!/usr/bin/env python
# Copyright 2018 Lucas Walter
# BSD3 license
import math
import rospy

from imgui_ros_msgs.msg import Widget
from imgui_ros_msgs.srv import *
# from time import sleep

# TODO(lucasw) move to imgui_ros utility module
def image_sub_widget(name, tab_name, topic):
    widget = Widget()
    widget.name = name
    widget.tab_name = tab_name
    widget.topic = topic
    widget.type = Widget.SUB
    widget.sub_type = Widget.IMAGE
    return widget

def dr_widget(name, tab_name, server):
    widget = Widget()
    widget.name = name
    widget.tab_name = tab_name
    widget.topic = server
    widget.type = Widget.DYNREC
    return widget

class DemoGui:
    def __init__(self):
        pass

    def run(self, namespace=''):
        rospy.wait_for_service(namespace + '/add_window')
        self.cli = rospy.ServiceProxy(namespace + '/add_window', AddWindow)

        self.add_images()
        self.add_dr("manip", ["/mix_images"], x=200.0)
        self.add_dr("roto_zoom0", ["/roto_zoom0"], x=200.0)
        self.add_dr("frei0r0", ["/frei0r0/frei0r", "/frei0r0/selector"], x=400.0)
        self.add_dr("frei0r1", ["/frei0r1/frei0r", "/frei0r1/selector"], x=500.0)
        self.add_dr("frei0r2", ["/frei0r2/frei0r", "/frei0r2/selector"], x=600.0)

    def add_dr(self, name, servers, x=0.0, y=0.0, width=300.0, height=600.0):
        req = AddWindowRequest()
        req.name = name
        req.init = True
        req.fractional = False
        if req.fractional:
            # TODO(lucasw) fractional doesn't allow dragging of window around
            req.position.x = 0.0
            req.position.y = 0.0
            req.size.x = 0.5
            req.size.y = 0.5
        else:
            req.position.x = x
            req.position.y = y
            req.size.x = width
            req.size.y = height

        tab_name = 'dr'

        for server in servers:
            widget = dr_widget(server.replace("/"," "), tab_name, server)
            req.widgets.append(widget)

        try:
            resp = self.cli(req)
            rospy.loginfo(resp)
        except rospy.service.ServiceException as e:
            rospy.logerr(e)

    def add_images(self):
        req = AddWindowRequest()
        req.name = 'images'
        req.init = True
        req.fractional = False
        if req.fractional:
            # TODO(lucasw) fractional doesn't allow dragging of window around
            req.position.x = 0.0
            req.position.y = 0.0
            req.size.x = 0.5
            req.size.y = 0.5
        else:
            req.position.x = 100.0
            req.position.y = 0.0
            req.size.x = 300.0
            req.size.y = 400.0
        tab_name = 'images'

        if True:
            widget = Widget()
            widget.name = "image pub"
            widget.tab_name = tab_name
            widget.topic = "/image_source/image_raw"
            widget.type = Widget.PUB
            widget.sub_type = Widget.IMAGE
            req.widgets.append(widget)

        # TODO(lucasw) parse the node graph and displaying all image outputs
        # instead of hardcoding.
        for ind in range(3):
            widget = image_sub_widget("image sub {}".format(ind),
                                      tab_name,
                                      "/frei0r{}/image_out".format(ind))
            req.widgets.append(widget)

        if True:
            widget = image_sub_widget("roto", tab_name, "/rotozoom/image_out")
            req.widgets.append(widget)
            widget = image_sub_widget("mix", tab_name, "/mix/image")
            req.widgets.append(widget)

        try:
            resp = self.cli(req)
            rospy.loginfo(resp)
        except rospy.service.ServiceException as e:
            rospy.logerr(e)

def main(args=None):
    rospy.init_node("imgui_ros_demo2")

    try:
        demo = DemoGui()
        demo.run()
    finally:
        pass

if __name__ == '__main__':
    main()
