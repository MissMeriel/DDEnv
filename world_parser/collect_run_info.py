#!/usr/bin/env python
import argparse

def setup_args(parser):
   #positional args
   parser.add_argument('world', help='world filename', type=str)
   parser.add_argument('goal', help='goal filename', type=str)
   parser.add_argument('timeout', help='timeout in seconds', type=str)
   # optional args
   parser.add_argument('-d', '--target_dir', help='directory to write new world files to', type=str)
   parser.add_argument('-s', '--spawn_pose', help='3-tuple for x,y,z spawning position', type=type([])) #, action="count")
   parser.add_argument('-t', '--trajectory', help='trajectory filename', type=str)
   parser.add_argument('-v', "--verbose", help="increase output verbosity", action="store_true")
   parser.add_argument('-c', "--combinatoric", help="use combinatoric baseline world parsing", action="store_true")
   return parser

def main():
   #python3 collect_run_info.py $world_name $goalfilename $timeout $script_dirname/temp.txt
   # setup and parse args
   parser = argparse.ArgumentParser()
   parser = setup_args(parser)
   args = parser.parse_args()


if __name__ == "__main__":
   main()