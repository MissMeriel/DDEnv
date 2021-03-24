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

To recreate Figure showing reduction over time of a scenario, run:
```bash
$ python gen_graph.py <logfile>
```


## Acknowledgements

This material is based in part upon work supported by the National Science Foundation under grant numbers #1924777 and #1853374.