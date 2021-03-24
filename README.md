# DDEnv
Automated Environment Reduction for Debugging Robotic Systems

## Getting Started

### Installation
Clone this repo:
```bash
$ git clone https://github.com/MissMeriel/DDEnv
```

Install ros dependencies:
```bash
sudo apt install python3-pip
sudo apt install ros-kinetic-robot-localization ros-kinetic-interactive-marker-twist-server ros-kinetic-controller-manager ros-kinetic-twist-mux ros-kinetic-move-base ros-kinetic-map-server ros-kinetic-move-base-msgs ros-kinetic-amcl ros-kinetic-joint-state-controller ros-kinetic-joint-state-publisher ros-kinetic-diff-drive-controller ros-kinetic-dwa-local-planner

sudo apt install shutter
sudo apt install libnet-dbus-glib-perl
```

Install sklearn for python3:
```bash
 pip3 install --upgrade pip
 pip3 install --upgrade setuptools
 pip3 install --upgrade scikit-learn
```

### Usage

Several helper scripts are available to reduce runtime environment configuration.
These helper scripts use the scenarios defined in the paper (published in ICRA 2021, preprint available [here]()).



To recreate Table I of results in the paper, run:
```bash
$ cd world_parser/
$ ./results_runner.sh
```

This will kick off a series of subscripts to perform reduction for each environment using each schema for prioritization and partitioning. 
The output of each script will be printed to a log file in the `results` directory and track the reduction of the world at each invocation of the schema. 
The final results look like the following:

```bash
TEST METRICS:
Starting env size: 43
Minimal environment size:  2
Iterations to find minimal world:  17
Total number of worlds generated:  94
Total number of tests run:  35
Total number of reruns due to flakiness:  18
Total number of heterogeneous failures:  5
Total number of successful runs:  1

real	42m6.336s
user	2m51.120s
sys	0m52.033s
```

To recreate Figure 2 showing reduction over time of a scenario, run:
```bash
$ python gen_graph.py <logfile>
```


## Acknowledgements

This material is based in part upon work supported by the National Science Foundation under grant numbers #1924777 and #1853374.