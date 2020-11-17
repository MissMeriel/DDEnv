#/usr/bin/python
import xml.etree.ElementTree as ET
import sys, os, shutil
import datetime
import itertools
import random
import math
from sklearn.cluster import KMeans
import numpy as np
import argparse

def counter(filename):
   count = 0
   new_tree = ET.parse(filename)
   new_root = new_tree.getroot()
   models = new_root[0].findall('model')
   # subtract one for ground plane
   count = len(models) - 1
   return count


def main():
   filename = sys.argv[1]
   count = counter(filename)
   #print("There are {} models in file {}".format(count, filename))
   exit(str(count))

if __name__ =='__main__':
   main()