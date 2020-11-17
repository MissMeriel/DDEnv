#/usr/bin/python
import xml.etree.ElementTree as ET
import sys, os, shutil
import datetime
import itertools
import random
import math
from sklearn.cluster import KMeans
import numpy as np
import argparse, copy

world_params = ['light', 'model', 'physics', 'scene']
models = []


#######################################################################################
#              BEGIN HELPER CODE FOR MIN DIST
# https://stackoverflow.com/questions/27161533/find-the-shortest-distance-between-a-point-and-line-segments-not-line
#######################################################################################

def lineseg_dists(p, a, b):
    """Cartesian distance from point to line segment

    Edited to support arguments as series, from:
    https://stackoverflow.com/a/54442561/11208892

    Args:
        - p: np.array of single point, shape (2,) or 2D array, shape (x, 2)
        - a: np.array of shape (x, 2)
        - b: np.array of shape (x, 2)
    """
    print("\nlineseg_dists({}, {}, {})".format(p, a, b))
    # print("a.shape: {}".format(a.shape))
    # print("b.shape: {}".format(b.shape))
    # print("p.shape: {}".format(p.shape))
    p = p.reshape(1,-1)
    #print("p.shape: {}".format(p.shape))
    d_ba = b - a
    print("d_ba: {}".format(d_ba))
    print("d_ba[:, 1]: {}".format(d_ba[:, 1]))
    print("np.hypot(d_ba[:, 0], d_ba[:, 1]): {}".format(np.hypot(d_ba[:, 0], d_ba[:, 1])))
    #print(": {}".format())
    d = np.divide(d_ba, (np.hypot(d_ba[:, 0], d_ba[:, 1])
                           .reshape(-1, 1)))

    # signed parallel distance components
    # rowwise dot products of 2D vectors
    s = np.multiply(a - p, d).sum(axis=1)
    t = np.multiply(p - b, d).sum(axis=1)

    # clamped parallel distance
    h = np.maximum.reduce([s, t, np.zeros(len(s))])

    # perpendicular distance component
    # rowwise cross products of 2D vectors  
    d_pa = p - a
    c = d_pa[:, 0] * d[:, 1] - d_pa[:, 1] * d[:, 0]

    #print("returning {}\n".format(np.hypot(h, c)))
    return np.hypot(h, c)



   
#######################################################################################
#              END HELPER CODE FOR MIN DIST
#######################################################################################


def change_trajectory_format(trajectory_seg):
    a = []
    b = []
    #print("len(trajectory_seg): {}".format(len(trajectory_seg)))
    #print("len(trajectory_seg[0:-1]): {}".format(len(trajectory_seg[0:-1])))
    #print(": {}".format())
    for i in range(len(trajectory_seg[0:-2])):
        #print("trajectory_seg[{}]: {}".format(i, trajectory_seg[i]))
        #print("trajectory_seg[{}+1]: {}\n".format(i, trajectory_seg[i+1]))
        a.append(trajectory_seg[i][0:2])
        b.append(trajectory_seg[i+1][0:2])
    return np.array(a), np.array(b)

### write copy of filename to target_dir
### new world filename created using timestamp
def new_world_file(filename, target_dir, i):
   now = str(datetime.datetime.now())[:19]
   now = now.replace(":", "_")
   now = now.replace(" ", "-")
   # get name of file
   filename_min = filename.split("/")[-1]
   src_dir = filename
   dst_filename = filename_min.replace(".world", "") + str(now) + "-{}".format(i) +".world"
   if not target_dir.endswith("/"):
      target_dir = target_dir + "/"
   dst_dir = target_dir +  dst_filename
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
      xy_coords.append(pose[0:2])
   return xy_coords


def is_world_empty(tree):
   root = tree.getroot()
   models = root[0].findall('model')
   models = remove_ground_plane(models)
   if len(models) == 0:
      return True
   return False


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


### find min distance from point p to line segment (v, w)
def minimum_distance(v, w, p):
  # Return minimum distance between line segment vw and point p
  v = np.array(v[0:3])
  w = np.array(w[0:3])
  p = np.array(p[0:3])
  l2 = (v-w)**2  # i.e. |w-v|^2 -  avoid a sqrt
  if (l2.all() == 0.0):
     return calc_distance3D(p, v);   # v == w case
  # Consider the line extending the segment, parameterized as v + t (w - v).
  # We find projection of point p onto the line. 
  # It falls where t = [(p-v) . (w-v)] / |w-v|^2
  # We clamp t from [0,1] to handle points outside the segment vw.
  #t = max(0, min(1, np.dot(p - v, w - v) / l2)).all();
  temp = np.dot(p - v, w - v) / l2
  temp = min(1, temp.all())
  t = max(0, temp)
  projection = v + t * (w - v);  # Projection falls on the segment
  return calc_distance3D(p, projection);


def get_model_pose(model):
   posetxt = model.find('pose').text
   pose = posetxt.split(' ')
   pose = [float(f) for f in pose]
   return pose


def get_end_pose(trajectory_arr):
    pose = None
    try:
       line = trajectory_arr[-1].replace('[', '').replace(']', '')
       pose = line.split(',')
       temp = pose[5]
    except:
       line = trajectory_arr[-2].replace('[', '').replace(']', '')
       pose = line.split(',')
    pose = [float(x) for x in pose]
    return pose


def calc_avg_trajectory_distance3D(pose, trajectory):
   avg_dist = 0.0
   for waypoint in trajectory:
      avg_dist += calc_distance3D(pose, waypoint[0:3])
   return avg_dist / len(trajectory)


def calc_min_trajectory_distance3D(pose, trajectory):
   dists = []
   for i in range(0, len(trajectory)-1):
      #print("trajectory[{}]: {}".format(i, trajectory[i]))
      d = lineseg_dists(pose, trajectory[i][0:2], trajectory[i+1][0:2])
      #print("min distance is {}".format(d))
      dists.append(d)
   return min(dists)


def calc_centroid(partition):
   new_root = partition.getroot()
   models = new_root[0].findall('model')
   models = remove_ground_plane(models)
   x = 0
   y = 0
   z = 0
   for m in models:
      pose = get_model_pose(m)[0:3]
      x += pose[0]
      y += pose[1]
      z += pose[2]
   return [float(x / len(models)), float(y / len(models)), float(z / len(models))]


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


def get_nearest_trajectory_segment(trajectory_segs, model_pose):
    distances = []
    #print("len(trajectory_segs): {}".format(len(trajectory_segs)))
    for i in range(len(trajectory_segs)):
        #print("trajectory_segs[{}]: {}".format(i, trajectory_segs[i]))
        a, b = change_trajectory_format(trajectory_segs[i])
        #print("a, b : \n\t{}, \n\t{}".format(a, b))
        if a.shape == b.shape:
            d = lineseg_dists(np.array(model_pose[0:2]), a, b)
            #print("d : {}\n".format(d))
            if d[0] is not float('nan'):
                distances.append(d[0])
            else:
                distances.append(d[0])
    #print("distances: {}".format(distances))
    min_dist_index = distances.index(min(distances))
    return min_dist_index


def get_nearest_trajectory_segment2(trajectory_segs, model_pose):
    distances = []
    #print("len(trajectory_segs): {}".format(len(trajectory_segs)))
    #min_dist_array = [[None, None] for i in range(len(trajectory_segs))]
    for i in range(len(trajectory_segs)):
        min_dist_array = []
        for j in range(len(trajectory_segs[i])-1):
            #print("trajectory_segs[{}]: {}".format(i, trajectory_segs[i]))
            #a, b = change_trajectory_format(trajectory_segs[i])
            dist = minimum_distance(trajectory_segs[i][j], trajectory_segs[i][j+1], model_pose)
            min_dist_array.append(dist)
        distances.append(min(min_dist_array))
        #print("min_dist_array: {}".format(min_dist_array))
    #print("\ndistances: {}".format(distances))
    min_dist_index = distances.index(min(distances))
    return min_dist_index


def cluster2(models, clusters=2):
   distance_arr = []
   distance_dict = {}
   # make array of distances of model poses
   for m in models:
      pose = get_model_pose(m)[0:3]
      distance_dict[m] = pose
   # get list of indices of distance arr when sorted smallest to largest distance
   #print("distances: {}".format(distance_dict.values()))
   #print("reshaped distances: {}".format(np.array(list(distance_dict.values())).astype(list).reshape(-1,1)))
   try:
      kmeans_clustered_models = KMeans(n_clusters=clusters, random_state=0).fit(np.array(list(distance_dict.values())).astype(list).reshape(-1,1))
   except:
      kmeans_clustered_models = KMeans(n_clusters=len(models), random_state=0).fit(np.array(list(distance_dict.values())).astype(list).reshape(-1,1))
   clustered_models = []
   for i in range(clusters):
      temp = []
      for j in range(len(models)):
         if kmeans_clustered_models.labels_[j] == i:
            temp.append(models[j])
      clustered_models.append(temp.copy())
   return clustered_models


def cluster(models, clusters=2):
	distance_arr = []
	distance_dict = {}
	# make array of distances of model poses
	for m in models:
		pose = get_model_pose(m)[0:3]
		distance_dict[m] = pose
		distance_arr.append([m, pose])
		# get list of indices of distance arr when sorted smallest to largest distance
		#print("distances: {}".format(distance_dict.values()))
		#print("reshaped distances: {}".format(np.array(list(distance_dict.values())).astype(list).reshape(-1,1)))
		#try:
		#   kmeans_clustered_models = KMeans(n_clusters=clusters, random_state=0).fit(np.array(list(distance_dict.values())).astype(list).reshape(-1,1))
		#except:
		#   kmeans_clustered_models = KMeans(n_clusters=len(models), random_state=0).fit(np.array(list(distance_dict.values())).astype(list).reshape(-1,1))
	#print("np.array(list(distance_dict.values())).astype(list).reshape(-1,3): \n{}".format(np.array(list(distance_dict.values())).astype(list).reshape(-1,3)))
	setup_arr = np.array(list(distance_dict.values())).astype(list).reshape(-1,3)
	try:
		kmeans =  KMeans(n_clusters=clusters, random_state=0)
		kmeans_clustered_models = kmeans.fit(setup_arr ) #, shape=(len(distance_dict), 3))
	except:
		kmeans =  KMeans(n_clusters=len(models), random_state=0)
		kmeans_clustered_models = kmeans.fit(setup_arr )
	#print("kmeans_clustered_models.labels_ : {}".format(kmeans_clustered_models.labels_))
	clustered_models = []
	for i in range(clusters):
		temp = []
		for j in range(len(models)):
			if kmeans_clustered_models.labels_[j] == i:
				temp.append(models[j])
		clustered_models.append(temp.copy())
	return clustered_models


def cluster_by_distance_from_point(models, spawn_pose, clusters=3):
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


def cluster_by_avg_distance_from_trajectory(models, trajectory, clusters=3):
   distance_arr = []
   distance_dict = {}
   # make array of distances of each model from trajectory
   for m in models:
      pose = get_model_pose(m)
      dist = calc_avg_trajectory_distance3D(pose[0:3], trajectory)
      distance_arr.append(dist)
      distance_dict[m] = dist
   # get list of indices of distance arr when sorted smallest to largest distance
   try:
      kmeans_clustered_models = KMeans(n_clusters=clusters, random_state=0).fit(np.array(list(distance_dict.values())).astype(float).reshape(-1,1))
   except ValueError:
      clusters = len(distance_dict.values())
      kmeans_clustered_models = KMeans(n_clusters=clusters, random_state=0).fit(np.array(list(distance_dict.values())).astype(float).reshape(-1,1))
   clustered_models = []
   for i in range(clusters):
      temp = []
      for j in range(len(models)):
         if kmeans_clustered_models.labels_[j] == i:
            temp.append(models[j])
      clustered_models.append(temp.copy())
   return clustered_models


def cluster_by_min_distance_from_trajectory(models, trajectory, clusters=3):
   distance_arr = []
   distance_dict = {}
   # make array of distances of each model from trajectory
   for m in models:
      pose = get_model_pose(m)
      dist = calc_min_trajectory_distance3D(pose[0:3], trajectory)
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


def cluster_by_trajectory_segments(models, trajectory_segs):
    model_dict = {}
    model_clusters = []
    for model in models:
        model_pose = get_model_pose(model)
        seg_index = get_nearest_trajectory_segment2(trajectory_segs, model_pose)
        model_dict[model] = seg_index
    #    print("model_pose: {}".format(model_pose))
    #    print("seg_index: {}".format(seg_index))
    #print("trajectory_segs: {}\n".format(trajectory_segs))
    #print(": {}".format())
    #print(": {}".format())
    for i in range(len(trajectory_segs)):
        model_cluster = [m for m in models if model_dict[m] == i]
        model_clusters.append(model_cluster)
    return model_clusters


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
   #print(trajectories)
   return trajectories


### input: file containing trajectory
### output: array of arrays containing trajectories between angular velocity changes
def parse_trajectory_file_with_goals_and_replanning(trajectory_filename):
   goalphrase = 'GOAL '
   successphrase = 'GOAL REACHED'
   replan_phrase = "REPLAN"
   with open(trajectory_filename, 'r') as f:
      lines = f.readlines()
      trajectories = []
      arr = []
      for line in lines:
         print("\nline: {}".format(line.replace('\n','')))
         
         if replan_phrase in line:
            print("replan_phrase in line")
            #print(arr)
            trajectories.append(arr.copy())
            print("len(trajectories): {}".format(len(trajectories)))
            arr = []
         elif successphrase in line:
            print("successphrase in line")
            trajectories.append(arr.copy())
            print("len(trajectories): {}".format(len(trajectories)))
            return trajectories
         elif goalphrase in line:
            print("goalphrase in line")
            #continue
         else:
            #print(line)
            print("else")
            line = line.replace('[','').replace(']','').replace(',','').replace('\n','')
            linesplit = line.split(' ')
            linesplit = [float(i) for i in linesplit]
            arr.append(linesplit)
   #print("trajectories: {}".format(trajectories))
   print("\nlen(trajectories): {}".format(len(trajectories)))
   return trajectories


def parse_trajectory_file_with_goals_and_replanning(trajectory_filename, nsegs):
   goalphrase = 'GOAL ['
   successphrase = 'GOAL REACHED'
   replan_phrase = "REPLAN"
   with open(trajectory_filename, 'r') as f:
      lines = f.readlines()
      trajectories = []
      for line in lines:
         #print("\nline: {}".format(line.replace('\n','')))
         if successphrase in line or replan_phrase in line or goalphrase in line:
            continue
         else:
            #print(line)
            #print("else")
            line = line.replace('[','').replace(']','').replace(',','').replace('\n','')
            linesplit = line.split(' ')
            linesplit = [float(i) for i in linesplit]
            trajectories.append(linesplit)
   print("trajectories: {}".format(trajectories))
   length = int(len(trajectories) / nsegs)
   print("length : {}".format(length))
   segged_trajectories = []
   print(range(nsegs))
   for i in range(nsegs):
      start = length * i
      end = length * i + length
      print("\nstart: {}".format(start))
      print("end: {}".format(end))
      print("i: {}".format(i))
      if i == nsegs - 1:
         print("trajectories[start:len(trajectories)]: {}".format(trajectories[start:len(trajectories)]))
         segged_trajectories.append(trajectories[start:len(trajectories)])
      else:
        print("trajectories[start:end]: {}".format(trajectories[start:end]))
        segged_trajectories.append(trajectories[start:end])
   print("segged_trajectories: {}".format(segged_trajectories))
   print("\nlen(segged_trajectories): {}".format(len(segged_trajectories)))
   return segged_trajectories

### input: file containing trajectory
### output: 1D array containing poses occupied by the robot
def parse_trajectory_file(trajectory_filename):
   goalphrase = 'GOAL '
   successphrase = 'GOAL REACHED'
   replan_phrase = "REPLAN"
   with open(trajectory_filename, 'r') as f:
      lines = f.readlines()
      trajectory = []
      for line in lines:
         line = line.replace('[','').replace(']','').replace(',','').replace('\n','')
         if goalphrase not in line and successphrase not in line and replan_phrase not in line:
            linesplit = line.split(' ')
            linesplit = [float(i) for i in linesplit]
            trajectory.append(tuple(linesplit))
   return trajectory


def parse_trajectory_file_2Darray(trajectory_filename):
   goalphrase = 'GOAL '
   successphrase = 'GOAL REACHED'
   with open(trajectory_filename, 'r') as f:
      lines = f.readlines()
      trajectory = []
      for line in lines:
         line = line.replace('[','').replace(']','').replace(',','').replace('\n','')
         if goalphrase not in line and successphrase not in line:
            linesplit = line.split(' ')
            linesplit = [float(i) for i in linesplit]
            trajectory.append(linesplit)
   return trajectory


def get_crash_pose(trajectory_filename):
   poses = parse_trajectory_file(trajectory_filename)
   cp = list(poses[-1])
   return cp


### COMBINATORIAL BASELINE WITH COMPLEMENTS
### there are N models in the file -- output N files with one model deleted
### remove models in sequential order that they appear in file
def partition_BASELINE_seq_with_complements(filename, target_dir, n_clusters=2):
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
def partition_BASELINE_rand_with_complements(filename):
   new_tree = ET.parse(filename)
   new_root = new_tree.getroot()
   models = new_root[0].findall('model')
   models = remove_ground_plane(models)
   # randomly delete one model per new file until none are left
   for i in range(len(models)):
      randomly_chosen_model = random.choice(models)
      # create complement tree + models
      complement_tree = ET.parse(filename)
      complement_root = complement_tree.getroot()
      complement_models = complement_root[0].findall('model')
      complement_models = remove_ground_plane(complement_models)
      # remove the corresponding link in the new tree
      for model in new_root[0].findall('model'):
         #print("model name: {}, random model name:{}".format(model.attrib['name'], randomly_chosen_model.attrib['name']))
         if model.attrib['name'] == randomly_chosen_model.attrib['name']:
            new_root[0].remove(model)
            print("model {} removed.".format(randomly_chosen_model.attrib['name']))
            break
      # make complement
      for complement_model in complement_models:
         if complement_model.attrib['name'] != model.attrib['name']:
            complement_root[0].remove(complement_model)
      # write new tree to file
      new_world_filename = new_world_file(filename, i)
      new_tree.write(new_world_filename)
      #write complement tree to file
      complement_filename = make_complement_filename(new_world_filename)
      print("Writing new tree to "+complement_filename)
      complement_tree.write(complement_filename)
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
   model_clusters = cluster(models, clusters)
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



### TODO: debug
### cluster models by proximity to each other and produce complements
def partition1_with_complements(filename, trajectory_filename, clusters=3):
   tree = ET.parse(filename)
   root = tree.getroot()
   models = root[0].findall('model')
   models = remove_ground_plane(models)
   trajectory = parse_trajectory_file(trajectory_filename)
   # cluster models
   model_clusters = cluster(models, clusters)
   new_trees = []
   complement_trees = []
   #print("model clusters: "+str(model_clusters))
   # check for empty clusters
   # TODO: better way to mitigate empty cluster issue?
   if [] in model_clusters:
      print("Empty cluster found, redistributing models...")
      model_clusters=[]
      for m in models:
         model_clusters.append([m])
      #model_clusters.remove([])
   # delete model clusters in sequence
   for i in range(len(model_clusters)):
      #print("cluster {}".format(i))
      new_tree = copy.deepcopy(tree)
      new_root = new_tree.getroot()
      new_models = new_root[0].findall('model')
      new_models = remove_ground_plane(new_models)
      #complement tree + models
      complement_tree = copy.deepcopy(tree)
      complement_root = complement_tree.getroot()
      complement_models = complement_root[0].findall('model')
      complement_models = remove_ground_plane(complement_models)
      names = [mdl.attrib['name'] for mdl in model_clusters[i]]
      for m in model_clusters[i]:
         for model in new_root[0].findall('model'):
            if model.attrib['name'] not in names and model.attrib['name'] != 'ground_plane':
               new_root[0].remove(model)
      # make complement
      for complement_model in complement_models:
         if complement_model.attrib['name'] in names:
            #print("Removing complement model "+complement_model.attrib['name'])
            complement_root[0].remove(complement_model)
      new_trees.append(copy.deepcopy(new_tree))
      complement_trees.append(copy.deepcopy(complement_tree))
   return new_trees, complement_trees


### TODO: debug
### cluster models by proximity to trajectory segments and produce complements
def partition2_with_complements(filename, trajectory_segs):
   tree = ET.parse(filename)
   root = tree.getroot()
   models = root[0].findall('model')
   models = remove_ground_plane(models)
   # cluster models according to nearest trajectory segment
   model_clusters = cluster_by_trajectory_segments(models, trajectory_segs)
   #print("len(model_clusters): {}".format(len(model_clusters)))
   #print("model_clusters: {}".format(model_clusters))
   #print(": {}".format())
   new_trees = []
   complement_trees = []
   #print("model clusters: "+str(model_clusters))
   # check for empty clusters
   # TODO: better way to mitigate empty cluster issue?
   if [] in model_clusters:
      print("Empty cluster found, redistributing models...")
      model_clusters=[]
      for m in models:
         model_clusters.append([m])
   # delete model clusters in sequence
   for i in range(len(model_clusters)):
      #print("cluster {}".format(i))
      new_tree = copy.deepcopy(tree)
      new_root = new_tree.getroot()
      new_models = new_root[0].findall('model')
      new_models = remove_ground_plane(new_models)
      #complement tree + models
      complement_tree = copy.deepcopy(tree)
      complement_root = complement_tree.getroot()
      complement_models = complement_root[0].findall('model')
      complement_models = remove_ground_plane(complement_models)
      names = [mdl.attrib['name'] for mdl in model_clusters[i]]
      for m in model_clusters[i]:
         for model in new_root[0].findall('model'):
            if model.attrib['name'] not in names and model.attrib['name'] != 'ground_plane':
               new_root[0].remove(model)
      # make complement
      for complement_model in complement_models:
         if complement_model.attrib['name'] in names:
            #print("Removing complement model "+complement_model.attrib['name'])
            complement_root[0].remove(complement_model)
      new_trees.append(copy.deepcopy(new_tree))
      complement_trees.append(copy.deepcopy(complement_tree))
   return new_trees, complement_trees


### cluster models by weighted proximity to entire trajectory and produce complements
### TODO: fine tune
### E.G. ConvergenceWarning: Number of distinct clusters (1) found smaller than n_clusters (3). Possibly due to duplicate points in X.
### the above happens if the robot oscillates in place as a recovery behavior or a failure to find a path
def partition4_with_complements(filename, trajectory_filename, clusters=3):
   tree = ET.parse(filename)
   root = tree.getroot()
   models = root[0].findall('model')
   models = remove_ground_plane(models)
   trajectory = parse_trajectory_file(trajectory_filename)
   # cluster models according to similar distances from spawn_pose
   model_clusters = cluster_by_avg_distance_from_trajectory(models, trajectory, clusters)
   new_trees = []
   complement_trees = []
   #print("model clusters: "+str(model_clusters))
   # check for empty clusters
   # TODO: better way to mitigate empty cluster issue?
   if [] in model_clusters:
      print("Empty cluster found, redistributing models...")
      model_clusters=[]
      for m in models:
         model_clusters.append([m])
   # delete model clusters in sequence
   for i in range(len(model_clusters)):
      #print("cluster {}".format(i))
      new_tree = copy.deepcopy(tree)
      new_root = new_tree.getroot()
      new_models = new_root[0].findall('model')
      new_models = remove_ground_plane(new_models)
      #complement tree + models
      complement_tree = copy.deepcopy(tree)
      complement_root = complement_tree.getroot()
      complement_models = complement_root[0].findall('model')
      complement_models = remove_ground_plane(complement_models)
      names = [mdl.attrib['name'] for mdl in model_clusters[i]]
      for m in model_clusters[i]:
         for model in new_root[0].findall('model'):
            if model.attrib['name'] not in names and model.attrib['name'] != 'ground_plane':
               new_root[0].remove(model)
      # make complement
      for complement_model in complement_models:
         if complement_model.attrib['name'] in names:
            #print("Removing complement model "+complement_model.attrib['name'])
            complement_root[0].remove(complement_model)
      new_trees.append(copy.deepcopy(new_tree))
      complement_trees.append(copy.deepcopy(complement_tree))
   return new_trees, complement_trees


### TODO: fine tune
### cluster models by proximity to crash and produce complements
def partition5_with_complements(filename, trajectory_filename, clusters=3):
   tree = ET.parse(filename)
   root = tree.getroot()
   models = root[0].findall('model')
   models = remove_ground_plane(models)
   trajectory = parse_trajectory_file(trajectory_filename)
   crash_pose = get_end_pose(trajectory)
   # cluster models according to distances from crash pose
   model_clusters = cluster_by_distance_from_point(models, crash_pose, clusters)
   new_trees = []
   complement_trees = []
   #print("model clusters: "+str(model_clusters))
   # check for empty clusters
   # TODO: better way to mitigate empty cluster issue?
   if [] in model_clusters:
      print("Empty cluster found, redistributing models...")
      model_clusters=[]
      for m in models:
         model_clusters.append([m])
   # delete model clusters in sequence
   for i in range(len(model_clusters)):
      #print("cluster {}".format(i))
      new_tree = copy.deepcopy(tree)
      new_root = new_tree.getroot()
      new_models = new_root[0].findall('model')
      new_models = remove_ground_plane(new_models)
      #complement tree + models
      complement_tree = copy.deepcopy(tree)
      complement_root = complement_tree.getroot()
      complement_models = complement_root[0].findall('model')
      complement_models = remove_ground_plane(complement_models)
      names = [mdl.attrib['name'] for mdl in model_clusters[i]]
      for m in model_clusters[i]:
         for model in new_root[0].findall('model'):
            if model.attrib['name'] not in names and model.attrib['name'] != 'ground_plane':
               new_root[0].remove(model)
      # make complement
      for complement_model in complement_models:
         if complement_model.attrib['name'] in names:
            #print("Removing complement model "+complement_model.attrib['name'])
            complement_root[0].remove(complement_model)
      new_trees.append(copy.deepcopy(new_tree))
      complement_trees.append(copy.deepcopy(complement_tree))
   return new_trees, complement_trees


### randomize partitions
### TODO: finish
def orderingA(partitions, complements, filename, target_dir):
   # write new trees to file
   for i in range(len(partitions)):
      new_tree = partitions[i]
      complement_tree = complements[i]
      new_world_filename = new_world_file(filename, target_dir, i)
      complement_filename = make_complement_filename(new_world_filename)
      complement_tree.write(complement_filename)
      new_tree.write(new_world_filename)


### distance from cluster centroid to crash
def orderingB(partitions, complements, filename, target_dir, crash_pose):
   # get distance from cluster centroid to crash
   dist_dict = {}
   for i in range(len(partitions)):
      centroid = calc_centroid(partitions[i])
      #print("centroid:"+str(centroid))
      dist = calc_distance3D(centroid, crash_pose)
      #print(dist)
      dist_dict[partitions[i]] = dist
   sorted_dict = {k: v for k, v in sorted(dist_dict.items(), key=lambda item: item[1])}
   #print("sorted_dict: "+str(sorted_dict))
   # write new trees to file
   for k in sorted_dict.keys():
      i = partitions.index(k)
      new_tree = partitions[i]
      complement_tree = complements[i]
      # write complements first so they get picked second
      new_world_filename = new_world_file(filename, target_dir, i)
      complement_filename = make_complement_filename(new_world_filename)
      complement_tree.write(complement_filename)
      new_tree.write(new_world_filename)
           

### average distance from cluster centroid to trajectory
def orderingC1(partitions, complements, world_filename, target_dir, trajectory):
   # get distance from cluster centroid to crash
   dist_dict = {}
   for i in range(len(partitions)):
      centroid = calc_centroid(partitions[i])
      #print("centroid:"+str(centroid))
      dist = calc_avg_trajectory_distance3D(centroid, trajectory)
      #print(dist)
      dist_dict[partitions[i]] = dist
   sorted_dict = {k: v for k, v in sorted(dist_dict.items(), key=lambda item: item[1])}
   #print("sorted_dict: "+str(sorted_dict))
   # write new trees to file
   for k in sorted_dict.keys():
      i = partitions.index(k)
      new_tree = partitions[i]
      complement_tree = complements[i]
      # write complements first so they get picked second
      new_world_filename = new_world_file(world_filename, target_dir, i)
      complement_filename = make_complement_filename(new_world_filename)
      complement_tree.write(complement_filename)
      new_tree.write(new_world_filename)
   
   
### minimum distance from cluster centroid to trajectory
def orderingC2(partitions, complements, world, target_dir, trajectory):
   # get distance from cluster centroid to crash
   dist_dict = {}
   for i in range(len(partitions)):
      centroid = calc_centroid(partitions[i])
      #print("centroid:"+str(centroid))
      dist = calc_min_trajectory_distance3D(centroid, trajectory)
      #print(dist)
      dist_dict[partitions[i]] = dist
   sorted_dict = {k: v for k, v in sorted(dist_dict.items(), key=lambda item: item[1])}
   #print("sorted_dict: "+str(sorted_dict))
   # write new trees to file
   for k in sorted_dict.keys():
      i = partitions.index(k)
      new_tree = partitions[i]
      complement_tree = complements[i]
      # write complements first so they get picked second
      new_world_filename = new_world_file(filename, target_dir, i)
      complement_filename = make_complement_filename(new_world_filename)
      complement_tree.write(complement_filename)
      new_tree.write(new_world_filename)


### timestep at which an element in the cluster was first sensed
### TODO: implement
def orderingD(partitions, complements, world, target_dir, trajectory):
   # get distance from cluster centroid to crash
   dist_dict = {}
   for i in range(len(partitions)):
      centroid = calc_centroid(partitions[i])
      #print("centroid:"+str(centroid))
      dist = calc_min_trajectory_distance3D(centroid, trajectory)
      #print(dist)
      dist_dict[partitions[i]] = dist
   sorted_dict = {k: v for k, v in sorted(dist_dict.items(), key=lambda item: item[1])}
   #print("sorted_dict: "+str(sorted_dict))
   # write new trees to file
   for k in sorted_dict.keys():
      i = partitions.index(k)
      new_tree = partitions[i]
      complement_tree = complements[i]
      # write complements first so they get picked second
      new_world_filename = new_world_file(filename, target_dir, i)
      complement_filename = make_complement_filename(new_world_filename)
      complement_tree.write(complement_filename)
      new_tree.write(new_world_filename)


def setup_args(parser):
   #positional args
   parser.add_argument('world', help='world filename', type=str)
   # optional args
   parser.add_argument('-pn', help='partitioning iteration number', type=int)
   parser.add_argument('-d', '--target_dir', help='directory to write new world files to', type=str)
   parser.add_argument('-s', '--spawn_pose', help='3-tuple for x,y,z spawning position', type=type([])) #, action="count")
   parser.add_argument('-v', "--verbose", help="increase output verbosity", action="store_true")
   # partitioning types
   parser.add_argument('-c', "--combinatoric", help="use combinatoric baseline world parsing", action="store_true")
   parser.add_argument('-m2m', "--model2model", help="use model2model world partitioning using N clusters", type=int)
   parser.add_argument('-t', '--trajectory', help='trajectory filename', type=str)
   parser.add_argument('-tseg', '--trajectory_segmentation', help='trajectory filename', type=str)
   parser.add_argument('-nsegs', '--num_segments', help='trajectory filename', type=str)
   # ordering types
   parser.add_argument('-r', "--random", help="use random partition ordering", action="store_true")
   parser.add_argument('-seq', "--sequential", help="use sequential partition ordering", action="store_true")
   parser.add_argument('-atd', '--avg_trajectory_distance', help='use partition ordering w.r.t. average distance from trajectory to cluster centroid', type=str)
   parser.add_argument('-cd', '--crash_distance', help='use partition ordering w.r.t. distance from crash to cluster centroid', type=str)
   parser.add_argument('-mtd', '--min_trajectory_distance', help='use partition ordering w.r.t. minimum distance from trajectory to cluster centroid', type=str)
   parser.add_argument('-ts', '--timestep', help='use partition ordering w.r.t. timestep', action="store_true")
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
      if args.combinatoric and args.sequential:
         print('\nRunning combinatoric baseline with sequential ordering...')
         partition_BASELINE_seq_with_complements(args.world, args.target_dir)
      elif args.combinatoric and args.random:
         print('\nRunning combinatoric baseline with random ordering...')
         partition_BASELINE_rand_with_complements(args.world, args.target_dir)
      #elif args.trajectory and args.avg_trajectory_distance:
      #   print('\nRunning trajectory aware partitioning...')
      #   partition4_with_complements(args.world, args.target_dir, args.trajectory)
      #   print('Running trajectory aware ordering...')
      elif args.model2model and args.crash_distance:
         #python3 partition_and_order.py ../src/husky/husky_gazebo/worlds/ditch.world -d ./test -m2m 2 -atd ../src/husky/husky_navigation/scripts/poses.log.backup
         print('\nRunning model2model cluster partitioning...')
         partitions, complements = partition1_with_complements(args.world, args.crash_distance, args.model2model)
         print('Running crash-proximal aware ordering...')
         crash_pose = get_crash_pose(args.crash_distance)
         #print("crash pose is" + str(crash_pose))
         orderingB(partitions, complements, args.world, args.target_dir, crash_pose)
      elif args.model2model and args.avg_trajectory_distance:
         #python3 partition_and_order.py ../src/husky/husky_gazebo/worlds/ditch.world -d ./test -m2m 2 -atd ../src/husky/husky_navigation/scripts/poses.log.backup
         print('\nRunning model2model cluster partitioning...')
         partitions, complements = partition1_with_complements(args.world, args.avg_trajectory_distance, args.model2model)
         print('Running average trajectory-proximal ordering...')
         trajectory = parse_trajectory_file(args.avg_trajectory_distance)
         #print("crash pose is" + str(crash_pose))
         orderingC1(partitions, complements, args.world, args.target_dir, trajectory)
      elif args.model2model and args.min_trajectory_distance:
         #python3 partition_and_order.py ../src/husky/husky_gazebo/worlds/ditch.world -d ./test -m2m 2 -atd ../src/husky/husky_navigation/scripts/poses.log.backup
         print('\nRunning model2model cluster partitioning...')
         partitions, complements = partition1_with_complements(args.world, args.min_trajectory_distance, args.model2model)
         print('Running minimum trajectory-proximal ordering...')
         trajectory = parse_trajectory_file_2Darray(args.min_trajectory_distance)
         orderingC2(partitions, complements, args.world, args.target_dir, trajectory)
      elif args.trajectory_segmentation and args.crash_distance:
         print('\nRunning trajectory segmentation partitioning...')
         #trajectory_segs = parse_trajectory_file_with_goals_and_replanning(args.trajectory_segmentation)
         trajectory_segs = parse_trajectory_file_with_goals_and_replanning(args.trajectory_segmentation, int(args.num_segments))
         print("trajectory_segs: {}".format(trajectory_segs))
         partitions, complements = partition2_with_complements(args.world, trajectory_segs)
         print('Running crash-proximal aware ordering...')
         crash_pose = get_crash_pose(args.crash_distance)
         orderingB(partitions, complements, args.world, args.target_dir, crash_pose)
      elif args.trajectory_segmentation and args.avg_trajectory_distance:
         print('\nRunning trajectory segmentation partitioning...')
         #trajectory_segs = parse_trajectory_file_with_goals_and_replanning(args.trajectory_segmentation)
         trajectory_segs = parse_trajectory_file_with_goals_and_replanning(args.trajectory_segmentation, int(args.num_segments))
         print("trajectory_segs: {}".format(trajectory_segs))
         partitions, complements = partition2_with_complements(args.world, trajectory_segs)
         print('Running average trajectory-proximal ordering...')
         trajectory = parse_trajectory_file(args.avg_trajectory_distance)
         orderingC1(partitions, complements, args.world, args.target_dir, trajectory)
      #os.remove(new_world_filename)

### TEST CALLS
# python3 partition_and_order.py ../src/husky/husky_gazebo/worlds/frictionpatch.world -d test/ -m2m 2 -atd ../src/husky/husky_navigation/scripts/poses.log.frictionpatch


if __name__ =='__main__':
   main()
