# Release Notes

## Version 1.1.0

Major Changes

| Module                  | Feature                      | Brief summary                                            | Pull request                                                 | JIRA       | Contributor                                   |
| ----------------------- | ---------------------------- | -------------------------------------------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `obstacle_segmentation` | point cloud topic rate check | update diagnostics status name of point cloud topic rate | [#40](https://github.com/tier4/driving_log_replayer/pull/40) | T4PB-23868 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.0.12

Bug fix

| Module                  | Feature      | Brief summary                  | Pull request                                                 | JIRA       | Contributor                                   |
| ----------------------- | ------------ | ------------------------------ | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `obstacle_segmentation` | frame result | fix status and judgement logic | [#37](https://github.com/tier4/driving_log_replayer/pull/37) | T4PB-23309 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.0.11

Major Changes

| Module                  | Feature   | Brief summary                                 | Pull request                                                 | JIRA       | Contributor                                   |
| ----------------------- | --------- | --------------------------------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `obstacle_segmentation` | test mode | support 3 test mode for obstacle_segmentation | [#35](https://github.com/tier4/driving_log_replayer/pull/35) | T4PB-23309 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.0.10

Minor Tweaks

| Module | Feature       | Brief summary                 | Pull request                                                 | JIRA       | Contributor                                   |
| ------ | ------------- | ----------------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `all`  | documentation | update Japanese Documentation | [#30](https://github.com/tier4/driving_log_replayer/pull/30) | T4PB-23145 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.0.9

Bug Fix

| Module | Feature       | Brief summary         | Pull request                                                 | JIRA       | Contributor                                   |
| ------ | ------------- | --------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `all`  | documentation | fix MkDocs Dependency | [#28](https://github.com/tier4/driving_log_replayer/pull/28) | T4PB-21730 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.0.8

Minor Tweaks

| Module | Feature       | Brief summary             | Pull request                                                 | JIRA       | Contributor                                   |
| ------ | ------------- | ------------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `all`  | documentation | add English Documentation | [#20](https://github.com/tier4/driving_log_replayer/pull/20) | T4PB-21730 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.0.7

Bug Fix

| Module       | Feature                  | Brief summary | Pull request                                                 | JIRA       | Contributor                                   |
| ------------ | ------------------------ | ------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `perception` | pass fail decision logic | fix logic     | [#25](https://github.com/tier4/driving_log_replayer/pull/25) | T4PB-22918 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.0.6

Bug Fix

| Module   | Feature              | Brief summary                                                               | Pull request                                                 | JIRA       | Contributor                                   |
| -------- | -------------------- | --------------------------------------------------------------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `common` | publish initial pose | Cast the value of the initial position described in the scenario to a float | [#22](https://github.com/tier4/driving_log_replayer/pull/22) | T4PB-22700 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.0.5

Major Changes

| Module       | Feature             | Brief summary                    | Pull request                                                 | JIRA       | Contributor                                   |
| ------------ | ------------------- | -------------------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `perception` | database evaluation | output database_result.json file | [#17](https://github.com/tier4/driving_log_replayer/pull/17) | T4PB-22358 | [hayato-m126](https://github.com/hayato-m126) |
| `perception` | database evaluation | fix exit with failure 1 file     | [#18](https://github.com/tier4/driving_log_replayer/pull/18) | T4PB-22358 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.0.4

Minor Tweaks

| Module | Feature       | Brief summary                        | Pull request                                                 | JIRA       | Contributor                                   |
| ------ | ------------- | ------------------------------------ | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `all`  | documentation | Adopted MkDocs as documentation tool | [#14](https://github.com/tier4/driving_log_replayer/pull/14) | T4PB-22101 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.0.3

Major Changes

| Module       | Feature          | Brief summary                          | Pull request                                                 | JIRA       | Contributor                                   |
| ------------ | ---------------- | -------------------------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `perception` | launch parameter | delete use_pointcloud_container option | [#12](https://github.com/tier4/driving_log_replayer/pull/12) | T4PB-21729 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.0.2

Minor Tweaks

| Module | Feature       | Brief summary                     | Pull request                                               | JIRA       | Contributor                                   |
| ------ | ------------- | --------------------------------- | ---------------------------------------------------------- | ---------- | --------------------------------------------- |
| `all`  | documentation | update sample scenario and config | [#6](https://github.com/tier4/driving_log_replayer/pull/6) | T4PB-21729 | [hayato-m126](https://github.com/hayato-m126) |
| `all`  | documentation | update sample scenario            | [#7](https://github.com/tier4/driving_log_replayer/pull/7) | T4PB-21729 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.0.1

Minor Tweaks

| Module                  | Feature       | Brief summary                              | Pull request                                               | JIRA       | Contributor                                   |
| ----------------------- | ------------- | ------------------------------------------ | ---------------------------------------------------------- | ---------- | --------------------------------------------- |
| `perception`            | visualization | shorten bounding box uuid to display       | [#2](https://github.com/tier4/driving_log_replayer/pull/2) | T4PB-21729 | [hayato-m126](https://github.com/hayato-m126) |
| `obstacle_segmentation` | remap topic   | remap concatenated point cloud in bag file | [#4](https://github.com/tier4/driving_log_replayer/pull/4) | T4PB-21729 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.0.0

Major Changes

| Module | Feature | Brief summary  | Pull request                                               | JIRA | Contributor                                   |
| ------ | ------- | -------------- | ---------------------------------------------------------- | ---- | --------------------------------------------- |
| `all`  | oss     | publish as oss | [#0](https://github.com/tier4/driving_log_replayer/pull/0) | -    | [hayato-m126](https://github.com/hayato-m126) |
