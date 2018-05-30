#!/usr/bin/env python
import rospy
import numpy as np
import cv2
import Person
import time
import argparse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String


class PersonCounter_ROS:
    def __init__(self, args):
        self.pub = rospy.Publisher(args.output, Image, queue_size=10)
        self.sub = rospy.Subscriber(args.input, Image, self.PersonCounter, queue_size=1, buff_size=2**24)

        self.cnt_up = 0
        self.cnt_down = 0

    def PersonCounter(self, data):
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(data, 'bgr8')

        h, w, channels = img.shape
        frameArea = h * w
        areaTH = frameArea / 250

        line_up = int(2 * (h / 5))
        line_down = int(3 * (h / 5))

        up_limit = int(1 * (h / 5))
        down_limit = int(4 * (h / 5))

        line_down_color = (255, 0, 0)
        line_up_color = (0, 0, 255)
        pt1 = [0, line_down]
        pt2 = [w, line_down]
        pts_L1 = np.array([pt1, pt2], np.int32)
        pts_L1 = pts_L1.reshape((-1, 1, 2))
        pt3 = [0, line_up]
        pt4 = [w, line_up]
        pts_L2 = np.array([pt3, pt4], np.int32)
        pts_L2 = pts_L2.reshape((-1, 1, 2))

        pt5 = [0, up_limit]
        pt6 = [w, up_limit]
        pts_L3 = np.array([pt5, pt6], np.int32)
        pts_L3 = pts_L3.reshape((-1, 1, 2))
        pt7 = [0, down_limit]
        pt8 = [w, down_limit]
        pts_L4 = np.array([pt7, pt8], np.int32)
        pts_L4 = pts_L4.reshape((-1, 1, 2))

        fgbg = cv2.createBackgroundSubtractorMOG2(detectShadows=True)

        kernelOp = np.ones((3, 3), np.uint8)
        kernelOp2 = np.ones((5, 5), np.uint8)
        kernelCl = np.ones((11, 11), np.uint8)

        font = cv2.FONT_HERSHEY_SIMPLEX
        persons = []
        max_p_age = 5
        pid = 1


        for i in persons:
            i.age_one()

        fgmask = fgbg.apply(img)
        fgmask2 = fgbg.apply(img)

        try:
            ret, imBin = cv2.threshold(fgmask, 200, 255, cv2.THRESH_BINARY)
            ret, imBin2 = cv2.threshold(fgmask2, 200, 255, cv2.THRESH_BINARY)

            mask = cv2.morphologyEx(imBin, cv2.MORPH_OPEN, kernelOp)
            mask2 = cv2.morphologyEx(imBin2, cv2.MORPH_OPEN, kernelOp)

            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernelCl)
            mask2 = cv2.morphologyEx(mask2, cv2.MORPH_CLOSE, kernelCl)
        except:
            print('EOF')
            print
            'UP:', self.cnt_up
            print
            'DOWN:', self.cnt_down

        _, contours0, hierarchy = cv2.findContours(mask2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours0:
            area = cv2.contourArea(cnt)
            if area > areaTH:
                M = cv2.moments(cnt)
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                x, y, w, h = cv2.boundingRect(cnt)
                new = True
                if cy in range(up_limit, down_limit):
                    for i in persons:
                        if abs(cx - i.getX()) <= w and abs(cy - i.getY()) <= h:
                            new = False
                            i.updateCoords(cx, cy)
                            if i.going_UP(line_down, line_up) == True:
                                self.cnt_up += 1;
                                print
                                "ID:", i.getId(), 'crossed going up at', time.strftime("%c")
                            elif i.going_DOWN(line_down, line_up) == True:
                                self.cnt_down += 1;
                                print
                                "ID:", i.getId(), 'crossed going down at', time.strftime("%c")
                            break
                        if i.getState() == '1':
                            if i.getDir() == 'down' and i.getY() > down_limit:
                                i.setDone()
                            elif i.getDir() == 'up' and i.getY() < up_limit:
                                i.setDone()
                        if i.timedOut():
                            index = persons.index(i)
                            persons.pop(index)
                            del i
                    if new == True:
                        p = Person.MyPerson(pid, cx, cy, max_p_age)
                        persons.append(p)
                        pid += 1
                cv2.circle(img, (cx, cy), 5, (0, 0, 255), -1)
                img_processed = cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)

        for i in persons:
            cv2.putText(img, str(i.getId()), (i.getX(), i.getY()), font, 0.3, i.getRGB(), 1, cv2.LINE_AA)

        str_up = 'UP: ' + str(self.cnt_up)
        str_down = 'DOWN: ' + str(self.cnt_down)
        frame = cv2.polylines(img, [pts_L1], False, line_down_color, thickness=2)
        frame = cv2.polylines(img, [pts_L2], False, line_up_color, thickness=2)
        frame = cv2.polylines(img, [pts_L3], False, (255, 255, 255), thickness=1)
        frame = cv2.polylines(frame, [pts_L4], False, (255, 255, 255), thickness=1)
        cv2.putText(img, str_up, (10, 40), font, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(img, str_up, (10, 40), font, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
        cv2.putText(img, str_down, (10, 90), font, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(img, str_down, (10, 90), font, 0.5, (255, 0, 0), 1, cv2.LINE_AA)


        self.pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))
        k = cv2.waitKey(30) & 0xff

def listener(args):
    rospy.init_node('person_counter', anonymous=True)
    PersonCounter_ROS(args)
    rospy.spin()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', type=str,
                        help='Specify the topic you wish to use as input.', default='/stream_1')
    parser.add_argument('--output', type=str,
                        help='Specify the topic you wish to publish the output image to.', default='/PersonCounter_1')
    args = parser.parse_args()
    listener(args)