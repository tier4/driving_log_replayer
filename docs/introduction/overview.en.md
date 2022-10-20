# Package Overview

Driving Log Replayer is the package that reads a scenario describing the conditions of evaluation, starts autoware, and outputs the evaluation results in jsonl file format.
The overview is shown in the figure below.

![overview](images/overview.drawio.svg)

## Operating Environment

The system shall be as follows.

- CPU amd64
- Ubuntu 20.04 / 22.04
- ROS galactic / humble
- Python 3.8 / 3.10
- NVIDIA GPU (required if running perception)

## Usage flow

1. acquire bags for evaluation using the actual vehicle.
2. filter the acquired bags so that only the required time and topics remain
   1. use ros2bag_extensions developed in TIER IV
   2. drop the topics output by autoware at the time of recording and leave only the sensor topics
   3. cut out time not needed for evaluation before and after driving (but leave 10 seconds before driving because the vehicle needs at least 3 seconds to be parked for initial positioning)
3. create the scenario
   1. there are some example scenarios in the sample folder.
   2. refer to the format definition for description contents.
4. If the use case is obstacle_segmentation or perception, annotate with an annotation tool that supports conversion to t4_dataset.
   1. t4_dataset conversion tool is in preparation for release
5. perform the evaluation.
