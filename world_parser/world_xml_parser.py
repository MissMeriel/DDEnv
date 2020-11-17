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

world_params = ['light', 'model', 'physics', 'scene']
models = []

### write copy of filename to target_dir
### new world filename created using timestamp
def new_world_file(filename, target_dir, i):
   now = str(datetime.datetime.now())[:19]
   now = now.replace(":", "_")
   now = now.replace(" ", "-")
   # get name of file
   filename_min = filename.split("/")[-1]
   #dst_dir = target_dir # "/".join(filename.split("/")[0:-1])
   src_dir = filename
   dst_filename = filename_min.replace(".world", "") + str(now) + "-{}".format(i) +".world"
   if not target_dir.endswith("/"):
      target_dir = target_dir + "/"
   #print dst_dir
   #print filename
   #dst_dir = dst_dir + "/" + dst_filename
   dst_dir = target_dir +  dst_filename
   #print("src_dir: "+src_dir)
   #print("dist_dir:"+dst_dir)
   shutil.copy(src_dir, dst_dir)
   return dst_dir


def make_complement_filename(filename):
   complement_filename = filename.split('.world')
   return complement_filename[0] + "_complement.world"


def get_xy(models):
   xy_coords = []
   for model in models:
      pose = model.find('pose').text.split()
      pose = [float(x) for x in pose]
      #print(model.attrib['name'])
      #print(pose[0:2])
      xy_coords.append(pose[0:2])
   return xy_coords


def is_world_empty(tree):
   root = tree.getroot()
   models = root[0].findall('model')
   models = remove_ground_plane(models)
   if len(models) == 0:
      return True
   return False


def generate_model_combos(models):
   combos = []
   for i in range(1, len(models)):
      #combos = combos.append(itertools.combinations(root.iter('model'), i))
      combos.append(itertools.combinations(models, i))
      #myiter = iter(combos)
      #print(next(myiter))
   return combos


def remove_ground_plane(models):
   models = [x for x in models if x.attrib['name'] != 'ground_plane']
   return models


### in: two [x,y] poses
### out: Euclidean distance
def calc_distance2D(p1, p2):
   return math.sqrt(math.pow(p1[0] - p2[0], 2) + math.pow(p1[1] - p2[1], 2))


### in: two [x,y,z] poses
### out: Euclidean distance
def calc_distance3D(p1, p2):
   return math.sqrt(math.pow(p1[0] - p2[0], 2) + math.pow(p1[1] - p2[1], 2) + math.pow(p1[2] - p2[2], 2))


def get_model_pose(model):
   posetxt = model.find('pose').text
   pose = posetxt.split(' ')
   pose = [float(f) for f in pose]
   return pose


def calc_avg_trajectory_distance3D(pose, trajectory):
   avg_dist = 0.0
   for waypoint in trajectory:
      avg_dist += calc_distance3D(pose, waypoint[0:3])
   return avg_dist / len(trajectory)


def calc_min_trajectory_distance3D(pose, trajectory):
   dists = []
   for waypoint in trajectory:
      dists.append(calc_distance3D(pose, waypoint[0:3]))
   return min(dists)


def sort_by_distance(models, spawn_pose):
   distance_arr = []
   distance_dict = {}
   # make array of distances of each model from spawning pose
   for m in models:
      pose = get_model_pose(m)
      dist = calc_distance3D(pose[0:3], spawn_pose)
      distance_arr.append(dist)
   # get list of indices of distance arr when sorted smallest to largest distance
   distance_arr_sorted_indices = sorted(range(len(distance_arr)), key=lambda k: distance_arr[k])
   sorted_models = []
   for i in distance_arr_sorted_indices:
      sorted_models.append(models[i])
   return sorted_models


def cluster_by_distance(models, spawn_pose, clusters=3):
   distance_arr = []
   distance_dict = {}
   # make array of distances of each model from spawning pose
   for m in models:
      pose = get_model_pose(m)
      dist = calc_distance3D(pose[0:3], spawn_pose)
      distance_arr.append(dist)
      distance_dict[m] = dist
   # get list of indices of distance arr when sorted smallest to largest distance
   kmeans_clustered_models = KMeans(n_clusters=clusters, random_state=0).fit(np.array(list(distance_dict.values())).astype(float).reshape(-1,1))
   clustered_models = []
   for i in range(clusters):
      temp = []
      for j in range(len(models)):
         if kmeans_clustered_models.labels_[j] == i:
            temp.append(models[j])
      clustered_models.append(temp.copy())
   return clustered_models


def cluster_by_distance_from_trajectory(models, trajectory, clusters=3):
   distance_arr = []
   distance_dict = {}
   # make array of distances of each model from trajectory
   for m in models:
      pose = get_model_pose(m)
      dist = calc_avg_trajectory_distance3D(pose[0:3], trajectory)
      distance_arr.append(dist)
      distance_dict[m] = dist
   # get list of indices of distance arr when sorted smallest to largest distance
   kmeans_clustered_models = KMeans(n_clusters=clusters, random_state=0).fit(np.array(list(distance_dict.values())).astype(float).reshape(-1,1))
   clustered_models = []
   for i in range(clusters):
      temp = []
      for j in range(len(models)):
         if kmeans_clustered_models.labels_[j] == i:
            temp.append(models[j])
      clustered_models.append(temp.copy())
   return clustered_models


### input: file containing trajectory
### output: array of arrays containing goals as the 0th entry
def parse_trajectory_file_with_goals(trajectory_filename):
   goalphrase = 'GOAL '
   successphrase = 'GOAL REACHED'
   with open(trajectory_filename, 'r') as f:
      lines = f.readlines()
      trajectories = []
      arr = []
      for line in lines:
         line = line.replace('[','').replace(']','').replace(',','').replace('\n','')
         if goalphrase in line and arr == None:
            arr = []
         elif goalphrase in line:
            trajectories.append(arr.copy())
            arr = []
            
         else:
            linesplit = line.split(' ')
            linesplit = [float(i) for i in linesplit]
            arr.append(tuple(linesplit))
   print(trajectories)
   return trajectories


### input: file containing trajectory
### output: 1D array containing poses occupied by the robot
def parse_trajectory_file(trajectory_filename):
   goalphrase = 'GOAL '
   with open(trajectory_filename, 'r') as f:
      lines = f.readlines()
      trajectory = []
      for line in lines:
         line = line.replace('[','').replace(']','').replace(',','').replace('\n','')
         if goalphrase not in line:
            linesplit = line.split(' ')
            linesplit = [float(i) for i in linesplit]
            trajectory.append(tuple(linesplit))
   return trajectory


### COMBINATORIAL BASELINE
### there are N models in the file -- output N files with one model deleted
### remove models in sequential order that they appear in file
def write_new_trees_BASELINE1(filename, target_dir, n_clusters=2):
   new_tree = ET.parse(filename)
   new_root = new_tree.getroot()
   models = new_root[0].findall('model')
   models = remove_ground_plane(models)
   # remove models iteratively (one per new file)
   files = []
   print("\nWriting new world files...")
   i = 0
   for model in models:
      new_tree = ET.parse(filename)
      new_root = new_tree.getroot()
      for new_root_model in new_root[0].findall('model'):
         if model.attrib['name'] == new_root_model.attrib['name']:
            new_root[0].remove(new_root_model)
            break
      # write new tree to file
      new_world_filename = new_world_file(filename, target_dir, i)
      print("Writing new tree to "+new_world_filename)
      files.append(new_world_filename)
      new_tree.write(new_world_filename)
      i += 1
   return files


### COMBINATORIAL BASELINE WITH COMPLEMENTS
### there are N models in the file -- output N files with one model deleted
### remove models in sequential order that they appear in file
def write_new_trees_BASELINE1_with_complements(filename, target_dir, n_clusters=2):
   new_tree = ET.parse(filename)
   new_root = new_tree.getroot()
   models = new_root[0].findall('model')
   models = remove_ground_plane(models)
   # remove models iteratively (one per new file)
   files = []
   print("Writing new world files...")
   i = 0
   for model in models:
      new_tree = ET.parse(filename)
      new_root = new_tree.getroot()
      # create complement tree + models
      complement_tree = ET.parse(filename)
      complement_root = complement_tree.getroot()
      complement_models = complement_root[0].findall('model')
      complement_models = remove_ground_plane(complement_models)
      for new_root_model in new_root[0].findall('model'):
         if model.attrib['name'] == new_root_model.attrib['name']:
            new_root[0].remove(new_root_model)
            break
      # make complement
      for complement_model in complement_models:
         if complement_model.attrib['name'] != model.attrib['name']:
            complement_root[0].remove(complement_model)
            #print("model {} removed.\n".format(model.attrib['name']))
      # write new tree to file
      new_world_filename = new_world_file(filename, target_dir, i)
      print("Writing new tree to "+new_world_filename)
      
      new_tree.write(new_world_filename)
      complement_filename = make_complement_filename(new_world_filename)
      print("Writing new tree to "+complement_filename)
      complement_tree.write(complement_filename)
      #files.append(new_world_filename)
      #files.append(complement_filename)
      i += 1
   return files


### COMBINATORIAL BASELINE
### there are N models in the file -- output N files with one model deleted
### remove models in random order
def write_new_trees_BASELINE2(filename):
   new_tree = ET.parse(filename)
   new_root = new_tree.getroot()
   models = new_root[0].findall('model')
   models = remove_ground_plane(models)
   # randomly delete one model per new file until none are left
   for i in range(len(models)):
      print(i)
      randomly_chosen_model = random.choice(models)
      print(randomly_chosen_model.attrib['name'])
      # remove the corresponding link in the new tree
      for model in new_root[0].findall('model'):
         #print("model name: {}, random model name:{}".format(model.attrib['name'], randomly_chosen_model.attrib['name']))
         if model.attrib['name'] == randomly_chosen_model.attrib['name']:
            new_root[0].remove(model)
            print("model {} removed.".format(randomly_chosen_model.attrib['name']))
            break
      # write new tree to file
      new_world_filename = new_world_file(filename, i)
      new_tree.write(new_world_filename)
      # delete link from list of links
      models = [x for x in models if x is not randomly_chosen_model]
      print()
   return new_tree


def write_new_trees1(filename, target_dir):
   new_tree = ET.parse(filename)
   new_root = new_tree.getroot()
   models = new_root[0].findall('model')
   models = remove_ground_plane(models)
   # delete one model per new file until none are left
   # print("Number of models: {}".format(len(models)))
   for i in range(len(models)):
      print(i)
      chosen_model = models[i]
      print(chosen_model.attrib['name'])
      # remove the corresponding link in the new tree
      for model in new_root[0].findall('model'):
         #print("model name: {}, random model name:{}".format(model.attrib['name'], randomly_chosen_model.attrib['name']))
         if model.attrib['name'] == chosen_model.attrib['name']:
            new_root[0].remove(model)
            print("model {} removed.".format(chosen_model.attrib['name']))
            break
      # write new tree to file
      new_world_filename = new_world_file(filename, target_dir, i)
      new_tree.write(new_world_filename)
      print()
   return new_tree


### TODO: debug
### take in robot pose
### do 3D search of environment and remove according to distance from pose
def write_new_trees2(filename, target_dir, spawn_pose=[0,0,0]):
   new_tree = ET.parse(filename)
   new_root = new_tree.getroot()
   models = new_root[0].findall('model')
   models = remove_ground_plane(models)
   # sort models according to distance from spawn_pose
   models = sort_by_distance(models, spawn_pose)
   # delete models in sequence
   for m in models:
      # remove the corresponding link in the new tree
      for model in new_root[0].findall('model'):
         if model.attrib['name'] == m.attrib['name']:
            new_root[0].remove(model)
            break
      # write new tree to file
      new_world_filename = new_world_file(filename, target_dir, i)
      new_tree.write(new_world_filename)
   return new_tree


### TODO: debug
### take in robot pose
### do 3D search of environment and cluster according to distance from pose
def write_new_trees3(filename, target_dir, spawn_pose=[0,0,0], clusters=3):
   new_tree = ET.parse(filename)
   new_root = new_tree.getroot()
   models = new_root[0].findall('model')
   models = remove_ground_plane(models)
   # sort models according to distance from spawn_pose
   # cluster models according to similar distances from spawn_pose
   model_clusters = cluster_by_distance(models, spawn_pose, clusters)
   # delete model clusters in sequence
   for i in range(len(model_clusters)):
      #print("cluster {}".format(i))
      new_tree = ET.parse(filename)
      new_root = new_tree.getroot()
      models = new_root[0].findall('model')
      models = remove_ground_plane(models)
      for m in model_clusters[i]:
         # remove the corresponding link in a new tree
         for model in new_root[0].findall('model'):
            if model.attrib['name'] == m.attrib['name']:
               new_root[0].remove(model)
               #print("model {} removed.\n".format(model.attrib['name']))
               break
      # write new tree to file
      new_world_filename = new_world_file(filename, target_dir, i)
      new_tree.write(new_world_filename)
   return new_tree


### TODO: fine tune
### cluster models by weighted proximity to entire trajectory
### E.G. ConvergenceWarning: Number of distinct clusters (1) found smaller than n_clusters (3). Possibly due to duplicate points in X.
### the above happens if the robot oscillates in place as a recovery behavior or a failure to find a path
### E.G. Order clusters written to file by farthest from trajectory to closest
def write_new_trees4(filename, target_dir, trajectory_filename, clusters=3):
   new_tree = ET.parse(filename)
   new_root = new_tree.getroot()
   models = new_root[0].findall('model')
   models = remove_ground_plane(models)
   trajectory = parse_trajectory_file(trajectory_filename)
   # sort models according to average distance from trajectory
   # cluster models according to similar distances from spawn_pose
   model_clusters = cluster_by_distance_from_trajectory(models, trajectory, clusters)
   # delete model clusters in sequence
   for i in range(len(model_clusters)):
      #print("cluster {}".format(i))
      new_tree = ET.parse(filename)
      new_root = new_tree.getroot()
      models = new_root[0].findall('model')
      models = remove_ground_plane(models)
      for m in model_clusters[i]:
         # remove the corresponding link in a new tree
         for model in new_root[0].findall('model'):
            if model.attrib['name'] == m.attrib['name']:
               new_root[0].remove(model)
               #print("model {} removed.\n".format(model.attrib['name']))
               break
      # write new tree to file
      new_world_filename = new_world_file(filename, target_dir, i)
      new_tree.write(new_world_filename)
   return new_tree


### TODO: fine tune
### cluster models by weighted proximity to entire trajectory and produce complements
### E.G. ConvergenceWarning: Number of distinct clusters (1) found smaller than n_clusters (3). Possibly due to duplicate points in X.
### the above happens if the robot oscillates in place as a recovery behavior or a failure to find a path
### E.G. Order clusters written to file by farthest from trajectory to closest
def write_new_trees4_with_complements(filename, target_dir, trajectory_filename, clusters=3):
   new_tree = ET.parse(filename)
   new_root = new_tree.getroot()
   models = new_root[0].findall('model')
   models = remove_ground_plane(models)
   trajectory = parse_trajectory_file(trajectory_filename)
   # sort models according to average distance from trajectory
   # cluster models according to similar distances from spawn_pose
   model_clusters = cluster_by_distance_from_trajectory(models, trajectory, clusters)
   # delete model clusters in sequence
   for i in range(len(model_clusters)):
      #print("cluster {}".format(i))
      new_tree = ET.parse(filename)
      new_root = new_tree.getroot()
      models = new_root[0].findall('model')
      models = remove_ground_plane(models)
      #complement tree + models
      complement_tree = ET.parse(filename)
      complement_root = complement_tree.getroot()
      complement_models = complement_root[0].findall('model')
      complement_models = remove_ground_plane(complement_models)
      # for each cluster
      for m in model_clusters[i]:
         # remove the corresponding link in a new tree
         for model in new_root[0].findall('model'):
            if model.attrib['name'] == m.attrib['name']:
               new_root[0].remove(model)
               #print("model {} removed.\n".format(model.attrib['name']))
               break
         # make complement
         for complement_model in complement_models:
            if complement_model.attrib['name'] != m.attrib['name'] :
               complement_root[0].remove(complement_model)
               #print("model {} removed.\n".format(model.attrib['name']))
               break
      # write new tree to file
      new_world_filename = new_world_file(filename, target_dir, i)
      new_tree.write(new_world_filename)
      complement_filename = make_complement_filename(new_world_filename)
      complement_tree.write(complement_filename)
   return new_tree


# ### TODO: implement
# ### group environment models by collision box overlap/adjacency
# def write_new_treesX(filename, target_dir):


def setup_args(parser):
   #positional args
   parser.add_argument('world', help='world filename', type=str)
   # optional args
   parser.add_argument('-d', '--target_dir', help='directory to write new world files to', type=str)
   parser.add_argument('-s', '--spawn_pose', help='3-tuple for x,y,z spawning position', type=type([])) #, action="count")
   parser.add_argument('-t', '--trajectory', help='trajectory filename', type=str)
   parser.add_argument('-v', "--verbose", help="increase output verbosity", action="store_true")
   parser.add_argument('-c', "--combinatoric", help="use combinatoric baseline world parsing", action="store_true")
   return parser
   
   
def main():
   # setup and parse args
   parser = argparse.ArgumentParser()
   parser = setup_args(parser)
   args = parser.parse_args()
   # check if world is empty
   tree = ET.parse(args.world)
   root = tree.getroot()
   if is_world_empty(tree):
      print('World is empty; nothing to do here.')
   else:
      # begin to parse
      if args.combinatoric:
         print('\nRunning combinatoric baseline...')
         write_new_trees_BASELINE1_with_complements(args.world, args.target_dir)
      elif args.trajectory:
         print('\nRunning trajectory aware technique...')
         write_new_trees4_with_complements(args.world, args.target_dir, args.trajectory)
      else:
         print('\nRunning non-trajectory aware technique...')
         write_new_trees3(args.world, args.target_dir)
      #os.remove(new_world_filename)

if __name__ =='__main__':
   main()