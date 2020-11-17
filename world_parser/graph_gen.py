#!/usr/bin/env python

import matplotlib.pyplot as plt



env_sizes = [43,43,24,23,23,23,18,17,16,15,14,13,12,11,10,9,8,7]
iterations = len(env_sizes)
plt.plot(range(iterations),  env_sizes)
plt.ylabel('Environment size', fontsize=16)
plt.xlabel('Iterations', fontsize=16)
plt.xticks(range(iterations))
plt.show()
