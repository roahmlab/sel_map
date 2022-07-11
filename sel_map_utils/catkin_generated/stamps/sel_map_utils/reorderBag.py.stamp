#from the rosbag cookbook
#http://wiki.ros.org/rosbag/Cookbook

import sys
import rosbag
import time
import subprocess
import yaml
import rospy
import os
import argparse
import math
from shutil import move

def status(length, percent):
  sys.stdout.write('\x1B[2K') # Erase entire current line
  sys.stdout.write('\x1B[0E') # Move to the beginning of the current line
  progress = "Progress: ["
  for i in range(0, length):
    if i < length * percent:
      progress += '='
    else:
      progress += ' '
  progress += "] " + str(round(percent * 100.0, 2)) + "%"
  sys.stdout.write(progress)
  sys.stdout.flush()


def main(args):
  parser = argparse.ArgumentParser(description='Reorder a bagfile based on header timestamps.')
  parser.add_argument('bagfile', nargs=1, help='input bag file')
  parser.add_argument('--max-offset', nargs=1, help='max time offset (sec) to correct.', default='60', type=float)
  args = parser.parse_args()
    
  # Get bag duration  
  
  bagfile = args.bagfile[0]
  
  info_dict = yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', bagfile], stdout=subprocess.PIPE).communicate()[0], Loader=yaml.FullLoader)
  try:
    duration = info_dict['duration']
    start_time = info_dict['start']
  except:
    print("Make sure the bag file is a valid rosbag! (If this utility crashed, check *.orig.bag)")
    return
  
  orig = os.path.splitext(bagfile)[0] + ".orig.bag"
  
  move(bagfile, orig)
  
  with rosbag.Bag(bagfile, 'w') as outbag:
      
    last_time = time.process_time()
    for topic, msg, t in rosbag.Bag(orig).read_messages():
        
      if time.process_time() - last_time > .1:
          percent = (t.to_sec() - start_time) / duration
          status(40, percent)
          last_time = time.process_time()
          
      # This also replaces tf timestamps under the assumption 
      # that all transforms in the message share the same timestamp
      if topic == "/tf" and msg.transforms:
        # Writing transforms to bag file 1 second ahead of time to ensure availability
        diff = math.fabs(msg.transforms[0].header.stamp.to_sec() - t.to_sec())
        outbag.write(topic, msg, msg.transforms[0].header.stamp - rospy.Duration(0) if diff < float(args.max_offset) else t)
      elif msg._has_header:
        diff = math.fabs(msg.header.stamp.to_sec() - t.to_sec())
        outbag.write(topic, msg, msg.header.stamp if diff < args.max_offset else t)
      else:
        outbag.write(topic, msg, t)
  status(40, 1)
  print("\ndone")

if __name__ == "__main__":
  main(sys.argv[1:])
