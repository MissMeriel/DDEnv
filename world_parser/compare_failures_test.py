#!/usr/bin/env python
import argparse
import math




def get_end_pose(failure):
   with open(failure, 'r') as f:
      lines = f.readlines()
      try:
         line = lines[-1].replace('[', '').replace(']', '')
         pose = line.split(',')
         temp = pose[5]
         pose = [float(x) for x in pose]
      except:
         try:
            line = lines[-2].replace('[', '').replace(']', '')
            pose = line.split(',')
            pose = [float(x) for x in pose]
         except:
            line = lines[-3].replace('[', '').replace(']', '')
            pose = line.split(',')
            pose = [float(x) for x in pose]
   return pose

def calc_distance3D(p1, p2):
   dist_3d = math.sqrt(math.pow(p1[0] - p2[0], 2) + math.pow(p1[1] - p2[1], 2))
   print("dist_3d : {}".format(dist_3d))
   dist_2d = math.sqrt(math.pow(p1[0] - p2[0], 2) + math.pow(p1[1] - p2[1], 2) + math.pow(p1[2] - p2[2], 2))
   print("dist_2d : {}".format(dist_2d))
   return math.sqrt(math.pow(p1[0] - p2[0], 2) + math.pow(p1[1] - p2[1], 2) + math.pow(p1[2] - p2[2], 2))

def setup_args(parser):
   #positional args
   parser.add_argument('failure1', help='filename containing info on first failure', type=str)
   parser.add_argument('failure2', help='filename containing info on second failure', type=str)
   # option args
   parser.add_argument('-e', '--epsilon', help='epsilon value to compare end poses', type=float, default=0.125)
   return parser

def main():
   global epsilon
   #python3 collect_run_info.py $world_name $goalfilename $timeout $script_dirname/temp.txt
   # setup and parse args
   parser = argparse.ArgumentParser()
   parser = setup_args(parser)
   args = parser.parse_args()
   # compare end poses
   endpose1 = get_end_pose(args.failure1)
   endpose2 = get_end_pose(args.failure2)
   print("args.failure1: {}".format(args.failure1))
   print("args.failure2: {}".format(args.failure2))
   print("endpose1 : {}".format(endpose1))
   print("endpose2 : {}".format(endpose2))
   distance = calc_distance3D(endpose1[0:3], endpose2[0:3])
   print("distance : {}".format(distance))
   print("args.epsilon : {}".format(args.epsilon))
   if distance < args.epsilon:
      #print("Similar failures found")
      exit("SAME")
   else:
      exit("DIFFERENT")
   # TODO: compare trajectory

if __name__ == "__main__":
   main()