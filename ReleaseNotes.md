# Release Notes

## Version 1.1.22

Minor Tweak

| Module             | Feature    | Brief summary     | Pull request                                                   | JIRA | Contributor                                   |
| ------------------ | ---------- | ----------------- | -------------------------------------------------------------- | ---- | --------------------------------------------- |
| `performance_diag` | bag record | drop camera topic | [#123](https://github.com/tier4/driving_log_replayer/pull/123) | -    | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.21

Documentation

| Module | Feature  | Brief summary               | Pull request                                                   | JIRA       | Contributor                                   |
| ------ | -------- | --------------------------- | -------------------------------------------------------------- | ---------- | --------------------------------------------- |
| `all`  | document | update input bag topic list | [#120](https://github.com/tier4/driving_log_replayer/pull/120) | T4PB-26922 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.20

Minor Tweak

| Module       | Feature | Brief summary                    | Pull request                                                   | JIRA       | Contributor                                   |
| ------------ | ------- | -------------------------------- | -------------------------------------------------------------- | ---------- | --------------------------------------------- |
| `perception` | library | update for perception_eval 1.0.4 | [#118](https://github.com/tier4/driving_log_replayer/pull/118) | T4PB-26566 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.19

Bug fix

| Module       | Feature | Brief summary            | Pull request                                                   | JIRA | Contributor                                   |
| ------------ | ------- | ------------------------ | -------------------------------------------------------------- | ---- | --------------------------------------------- |
| `perception` | metrics | fix count tp, fp, and fn | [#116](https://github.com/tier4/driving_log_replayer/pull/116) | -    | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.18

Minor Tweak, Bug fix

| Module       | Feature | Brief summary                          | Pull request                                                   | JIRA       | Contributor                                   |
| ------------ | ------- | -------------------------------------- | -------------------------------------------------------------- | ---------- | --------------------------------------------- |
| `perception` | library | update for perception_eval new feature | [#113](https://github.com/tier4/driving_log_replayer/pull/113) | T4PB-25343 | [hayato-m126](https://github.com/hayato-m126) |
| `cli`        | library | add dependent package                  | [#114](https://github.com/tier4/driving_log_replayer/pull/114) | T4PB-25810 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.17

Minor Tweak

| Module | Feature       | Brief summary        | Pull request                                                   | JIRA       | Contributor                                   |
| ------ | ------------- | -------------------- | -------------------------------------------------------------- | ---------- | --------------------------------------------- |
| `all`  | visualization | update rviz settings | [#111](https://github.com/tier4/driving_log_replayer/pull/111) | T4PB-25343 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.16

Bug fix

| Module       | Feature      | Brief summary                         | Pull request                                                   | JIRA       | Contributor                                   |
| ------------ | ------------ | ------------------------------------- | -------------------------------------------------------------- | ---------- | --------------------------------------------- |
| `perception` | json library | use simplejson to convert NaN to null | [#109](https://github.com/tier4/driving_log_replayer/pull/109) | T4PB-25810 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.15

Bug fix

| Module     | Feature   | Brief summary | Pull request                                                   | JIRA | Contributor                                     |
| ---------- | --------- | ------------- | -------------------------------------------------------------- | ---- | ----------------------------------------------- |
| `document` | html link | fix link      | [#105](https://github.com/tier4/driving_log_replayer/pull/105) | -    | [KeisukeShima](https://github.com/KeisukeShima) |
| `document` | html link | fix link      | [#106](https://github.com/tier4/driving_log_replayer/pull/106) | -    | [hayato-m126](https://github.com/hayato-m126)   |

## Version 1.1.14

Minor Tweak

| Module     | Feature        | Brief summary                                | Pull request                                                   | JIRA | Contributor                                   |
| ---------- | -------------- | -------------------------------------------- | -------------------------------------------------------------- | ---- | --------------------------------------------- |
| `document` | mkdocs setting | update setting for mkdocs material version 9 | [#102](https://github.com/tier4/driving_log_replayer/pull/102) | -    | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.13

Minor Tweak

| Module     | Feature       | Brief summary                         | Pull request                                                 | JIRA       | Contributor                                   |
| ---------- | ------------- | ------------------------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `analyzer` | visualization | update obstacle segmentation analyzer | [#94](https://github.com/tier4/driving_log_replayer/pull/94) | T4PB-24956 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.12

Minor Tweak

| Module                  | Feature       | Brief summary                 | Pull request                                                 | JIRA       | Contributor                                   |
| ----------------------- | ------------- | ----------------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `analyzer`              | refactor      | refactor analyzer             | [#89](https://github.com/tier4/driving_log_replayer/pull/89) | T4PB-24956 | [hayato-m126](https://github.com/hayato-m126) |
| `ci`                    | github action | update pre-commit-shfmt       | [#95](https://github.com/tier4/driving_log_replayer/pull/95) | -          | [hayato-m126](https://github.com/hayato-m126) |
| `perception`            | dependency    | update perception eval        | [#98](https://github.com/tier4/driving_log_replayer/pull/98) | T4PB-25347 | [hayato-m126](https://github.com/hayato-m126) |
| `obstacle_segmentation` | api           | use awapi to get stop reasons | [#99](https://github.com/tier4/driving_log_replayer/pull/99) | T4PB-25316 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.11

Bug fix

| Module | Feature             | Brief summary                                | Pull request                                                 | JIRA       | Contributor                                     |
| ------ | ------------------- | -------------------------------------------- | ------------------------------------------------------------ | ---------- | ----------------------------------------------- |
| `cli`  | kill zombie process | Fixed to not kill processes unrelated to ros | [#92](https://github.com/tier4/driving_log_replayer/pull/92) | T4PB-25129 | [M(-\_-)Sakamoto](https://github.com/Motsu-san) |

## Version 1.1.10

Bug fix

| Module                  | Feature            | Brief summary             | Pull request                                                 | JIRA       | Contributor                                   |
| ----------------------- | ------------------ | ------------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `cli`                   | sub command        | fix command name          | [#80](https://github.com/tier4/driving_log_replayer/pull/80) | -          | [hayato-m126](https://github.com/hayato-m126) |
| `all`                   | license            | use Apache-2.0            | [#82](https://github.com/tier4/driving_log_replayer/pull/82) | -          | [hayato-m126](https://github.com/hayato-m126) |
| `obstacle_segmentation` | dependency         | add exec depend           | [#83](https://github.com/tier4/driving_log_replayer/pull/83) | T4PB-23864 | [hayato-m126](https://github.com/hayato-m126) |
| `performance_diag`      | exception handling | handle TransformException | [#85](https://github.com/tier4/driving_log_replayer/pull/85) | T4PB-24969 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.9

Bug fix

| Module                  | Feature       | Brief summary    | Pull request                                                 | JIRA       | Contributor                                   |
| ----------------------- | ------------- | ---------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `obstacle_segmentation` | visualization | fix import error | [#78](https://github.com/tier4/driving_log_replayer/pull/78) | T4PB-23864 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.8

Minor Tweak, Bug fix

| Module                  | Feature       | Brief summary                                                                 | Pull request                                                 | JIRA       | Contributor                                   |
| ----------------------- | ------------- | ----------------------------------------------------------------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `obstacle_segmentation` | marker color  | The result of the topic's rate is reflected in the color of the bounding box. | [#72](https://github.com/tier4/driving_log_replayer/pull/72) | T4PB-23864 | [hayato-m126](https://github.com/hayato-m126) |
| `obstacle_segmentation` | visualization | add graph data to result.jsonl                                                | [#73](https://github.com/tier4/driving_log_replayer/pull/73) | T4PB-23864 | [hayato-m126](https://github.com/hayato-m126) |
| `performance_diag`      | localization  | fix infinite wait when localization module off                                | [#75](https://github.com/tier4/driving_log_replayer/pull/75) | T4PB-24705 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.7

Minor Tweak, Bug fix

| Module                             | Feature       | Brief summary                                        | Pull request                                                 | JIRA       | Contributor                                   |
| ---------------------------------- | ------------- | ---------------------------------------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `obstacle_segmentation`            | visualization | publish and record data for visualization            | [#67](https://github.com/tier4/driving_log_replayer/pull/67) | T4PB-23864 | [hayato-m126](https://github.com/hayato-m126) |
| `performance_diag`                 | initial pose  | delete uninitialized publisher                       | [#68](https://github.com/tier4/driving_log_replayer/pull/68) | T4PB-24491 | [hayato-m126](https://github.com/hayato-m126) |
| `localization`, `performance_diag` | initial pose  | use map fit service to correct `initialpose z value` | [#70](https://github.com/tier4/driving_log_replayer/pull/70) | T4PB-24491 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.6

Minor Tweak

| Module                             | Feature      | Brief summary               | Pull request                                                 | JIRA       | Contributor                                   |
| ---------------------------------- | ------------ | --------------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `localization`, `performance_diag` | initial pose | set initial pose via ad-api | [#65](https://github.com/tier4/driving_log_replayer/pull/65) | T4PB-24491 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.5

Bug fix

| Module       | Feature      | Brief summary                                            | Pull request                                                 | JIRA       | Contributor                                   |
| ------------ | ------------ | -------------------------------------------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `perception` | marker color | different colors are applied to tp_gt, tp_est, fp and fn | [#61](https://github.com/tier4/driving_log_replayer/pull/61) | T4PB-24425 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.4

Major Changes

| Module       | Feature        | Brief summary                                 | Pull request                                                 | JIRA       | Contributor                                   |
| ------------ | -------------- | --------------------------------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `perception` | launch process | Removed waiting for conversion of onnx files. | [#59](https://github.com/tier4/driving_log_replayer/pull/59) | T4PB-24362 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.3

Major Changes

| Module                  | Feature             | Brief summary                                    | Pull request                                                 | JIRA       | Contributor                                   |
| ----------------------- | ------------------- | ------------------------------------------------ | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `obstacle segmentation` | evaluation settings | set evaluation period for each bbox              | [#51](https://github.com/tier4/driving_log_replayer/pull/51) | T4PB-24138 | [hayato-m126](https://github.com/hayato-m126) |
| `obstacle segmentation` | result file         | output timestamp of bounding box and point cloud | [#52](https://github.com/tier4/driving_log_replayer/pull/52) | T4PB-24238 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.2

Minor Tweak

| Module | Feature    | Brief summary                                         | Pull request                                                 | JIRA       | Contributor                                   |
| ------ | ---------- | ----------------------------------------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `all`  | cli        | kill zombie process                                   | [#45](https://github.com/tier4/driving_log_replayer/pull/45) | T4PB-24124 | [hayato-m126](https://github.com/hayato-m126) |
| `all`  | cli        | fix generated command to kill zombie process          | [#46](https://github.com/tier4/driving_log_replayer/pull/46) | T4PB-24124 | [hayato-m126](https://github.com/hayato-m126) |
| `all`  | cli        | modified commands to avoid killing the running shell. | [#47](https://github.com/tier4/driving_log_replayer/pull/47) | T4PB-24124 | [hayato-m126](https://github.com/hayato-m126) |
| `all`  | dictionary | add local dictionary                                  | [#48](https://github.com/tier4/driving_log_replayer/pull/48) | T4PB-23864 | [hayato-m126](https://github.com/hayato-m126) |
| `all`  | code style | fix lit check                                         | [#49](https://github.com/tier4/driving_log_replayer/pull/49) | T4PB-23864 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.1

Major Changes

| Module     | Feature  | Brief summary                          | Pull request                                                 | JIRA       | Contributor                                     |
| ---------- | -------- | -------------------------------------- | ------------------------------------------------------------ | ---------- | ----------------------------------------------- |
| `analyzer` | analyzer | add analyzer to visualize result.jsonl | [#42](https://github.com/tier4/driving_log_replayer/pull/42) | T4PB-23864 | [KeisukeShima](https://github.com/KeisukeShima) |
| `analyzer` | analyzer | fix pre-commit check                   | [#43](https://github.com/tier4/driving_log_replayer/pull/43) | T4PB-23864 | [hayato-m126](https://github.com/hayato-m126)   |
| `analyzer` | analyzer | add cli command and ros2 launch        | [#44](https://github.com/tier4/driving_log_replayer/pull/44) | T4PB-23864 | [hayato-m126](https://github.com/hayato-m126)   |

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
