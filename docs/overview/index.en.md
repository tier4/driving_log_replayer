# Introduction

Driving Log Replayer is a package that runs Autoware in an open loop by supplying previously recorded input data using log(rosbag2) API. The package gathers information and evaluates topics output produced by Autoware.
Its use is to test the software regression and check Autoware's performance of sensing, localization, and perception components.

## Architecture

Driving Log Replayer package contains an evaluation node that extends Autoware's standard functionality.
The architecture graph is shown below.

![architecture](images/architecture.png)

## Related Documents

1. [AutowareDocumentation](https://autowarefoundation.github.io/autoware-documentation/main/)
2. [WebAutoDocumentation](https://docs.web.auto/)

## Related repositories

1. [ros2bag_extensions](https://github.com/tier4/ros2bag_extensions)
2. [perception_eval](https://github.com/tier4/autoware_perception_evaluation)
