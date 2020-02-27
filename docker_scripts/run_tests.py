#!/usr/bin/env python

from __future__ import print_function
from subprocess import Popen, PIPE
import re

SAMPLE_SIZE=25
PRODUCERS=('nodelet', 'node_cpp', 'node_py')
CONSUMERS=('nodelet', 'node_cpp', 'node_py', 'node_py_largebuf')

staleness_re = re.compile(r'Average staleness:\s*([\d\.]+);\s*standard deviation:\s*([\d\.]+)', flags=re.MULTILINE)

for producer in PRODUCERS:
    for consumer in CONSUMERS:
        print("--- Producer variant: {}, consumer variant: {}".format(producer, consumer))
        # Allow roslaunch to start
        timeout = SAMPLE_SIZE + 5
        process = Popen("timeout {} stdbuf -o L roslaunch staleness_test tests.launch producer_type:={} consumer_type:={} sample_size:={}".format(
            timeout, producer, consumer, SAMPLE_SIZE
        ), stdout=PIPE, stderr=PIPE, shell=True)
        output, stderr = process.communicate()
        try:
            avg, stdev = staleness_re.findall(output)[0]
        except:
            print('Could not match re; output is {}'.format(output))
            print('stderr is {}'.format(stderr))
        print("Average: {}, stdev: {}".format(avg, stdev))
