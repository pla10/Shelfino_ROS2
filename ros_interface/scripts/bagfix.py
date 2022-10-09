#!/usr/bin/env python

import rosbag
import rospy
import sys

SCAN_TOPIC = '/scan'
ENCODERS_TOPIC = '/encoders'

def process(input, output):
  bag_in = rosbag.Bag(input)
  bag_out = rosbag.Bag(output, 'w')

  for topic, msg, t in bag_in.read_messages(topics=[SCAN_TOPIC, ENCODERS_TOPIC]):
    if topic==SCAN_TOPIC:
      msg.header.stamp -= rospy.Duration.from_sec(0.0)

      m_f = 1

      msg.angle_increment = 0.00872664625
      msg.angle_min = m_f * msg.angle_increment
      msg.angle_max = 6.27445866092 + msg.angle_min
      #msg.angle_min = 0.00872664625
      #msg.angle_max = 6.28318530718
      
    elif topic==ENCODERS_TOPIC:
      pass
    bag_out.write(topic, msg, t)
  bag_in.close()
  bag_out.close()

if __name__=="__main__":
  if len(sys.argv) < 3:
    print("usage: %s <bagfile> <outputfile>")
  else:
    process(sys.argv[1], sys.argv[2])