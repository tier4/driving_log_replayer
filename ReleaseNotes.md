# Release Notes

## Version 1.5.3

Minor Tweak

| Module    | Feature         | Brief summary          | Pull request                                                   | Jira | Contributor                                   |
| --------- | --------------- | ---------------------- | -------------------------------------------------------------- | ---- | --------------------------------------------- |
| `cli`     | `poetry`        | update pyproject.toml  | [#220](https://github.com/tier4/driving_log_replayer/pull/220) | -    | [hayato-m126](https://github.com/hayato-m126) |
| `eagleye` | `documentation` | add `eagleye` tutorial | [#222](https://github.com/tier4/driving_log_replayer/pull/222) | -    | [kminoda](https://github.com/kminoda)         |

## Version 1.5.2

Minor Tweak

| Module          | Feature          | Brief summary                  | Pull request                                                   | Jira     | Contributor                                   |
| --------------- | ---------------- | ------------------------------ | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `all`           | `linter`         | apply `ruff` linter EM rules   | [#211](https://github.com/tier4/driving_log_replayer/pull/211) | RT4-6363 | [hayato-m126](https://github.com/hayato-m126) |
| `all`           | `linter`         | apply `ruff` linter PIE rules  | [#212](https://github.com/tier4/driving_log_replayer/pull/212) | RT4-6363 | [hayato-m126](https://github.com/hayato-m126) |
| `all`           | `linter`         | apply `ruff` linter RET rules  | [#213](https://github.com/tier4/driving_log_replayer/pull/213) | RT4-6363 | [hayato-m126](https://github.com/hayato-m126) |
| `all`           | `linter`         | apply `ruff` linter SLF rules  | -                                                              | RT4-6363 | [hayato-m126](https://github.com/hayato-m126) |
| `all`           | `linter`         | apply `ruff` linter TCH rules  | [#214](https://github.com/tier4/driving_log_replayer/pull/214) | RT4-6363 | [hayato-m126](https://github.com/hayato-m126) |
| `all`           | `linter`         | apply `ruff` linter UP rules   | [#215](https://github.com/tier4/driving_log_replayer/pull/215) | RT4-6363 | [hayato-m126](https://github.com/hayato-m126) |
| `all`           | `linter`         | apply `ruff` linter SIM rules  | [#216](https://github.com/tier4/driving_log_replayer/pull/216) | RT4-6363 | [hayato-m126](https://github.com/hayato-m126) |
| `all`           | `linter`         | apply `ruff` linter RUF        | [#217](https://github.com/tier4/driving_log_replayer/pull/217) | RT4-6363 | [hayato-m126](https://github.com/hayato-m126) |
| `perception_2d` | `error_handling` | handle invalid evaluation task | [#218](https://github.com/tier4/driving_log_replayer/pull/218) | RT4-6363 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.5.1

Breaking change

| Module       | Feature           | Brief summary                   | Pull request                                                   | Jira     | Contributor                                   |
| ------------ | ----------------- | ------------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `perception` | fp validation log | store fp result in result.jsonl | [#206](https://github.com/tier4/driving_log_replayer/pull/206) | RT4-5559 | [hayato-m126](https://github.com/hayato-m126) |
| `all`        | `library`         | update python library version   | [#207](https://github.com/tier4/driving_log_replayer/pull/207) | -        | [hayato-m126](https://github.com/hayato-m126) |
| `all`        | `linter`          | use `ruff` linter               | [#208](https://github.com/tier4/driving_log_replayer/pull/208) | RT4-6363 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.5.0

Breaking change

| Module                                     | Feature         | Brief summary            | Pull request                                                   | Jira                                           | Contributor                                   |
| ------------------------------------------ | --------------- | ------------------------ | -------------------------------------------------------------- | ---------------------------------------------- | --------------------------------------------- |
| `perception`                               | `fp validation` | add `fp validation` mode | [#200](https://github.com/tier4/driving_log_replayer/pull/200) | RT4-5559                                       | [hayato-m126](https://github.com/hayato-m126) |
| `perception, perception_2d, traffic_light` | `object shape`  | support `unknown` label  | [#198](https://github.com/tier4/driving_log_replayer/pull/198) | RT4-5880                                       | [hayato-m126](https://github.com/hayato-m126) |
| `eagleye`                                  | `eagleye`       | add `eagleye` evaluation | [#203](https://github.com/tier4/driving_log_replayer/pull/203) | RT1-3254 [kminoda](https://github.com/kminoda) |
| `perception, perception_2d, traffic_light` | `scenario`      | update `sample scenario` | [#204](https://github.com/tier4/driving_log_replayer/pull/204) | RT4-6041                                       | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.4.1

Major change

| Module   | Feature  | Brief summary           | Pull request                                                   | Jira     | Contributor                           |
| -------- | -------- | ----------------------- | -------------------------------------------------------------- | -------- | ------------------------------------- |
| `yabloc` | `yabloc` | add `yabloc` evaluation | [#201](https://github.com/tier4/driving_log_replayer/pull/201) | RT1-3255 | [kminoda](https://github.com/kminoda) |

## Version 1.4.0

Major change
Ensure that you are using Autoware.universe that incorporates [this PR](https://github.com/autowarefoundation/autoware.universe/pull/4128).

| Module             | Feature      | Brief summary                            | Pull request                                                   | Jira     | Contributor                                   |
| ------------------ | ------------ | ---------------------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `localization`     | initial pose | use map_height_fitter to correct z value | [#176](https://github.com/tier4/driving_log_replayer/pull/176) | RT4-5532 | [hayato-m126](https://github.com/hayato-m126) |
| `performance_diag` | initial pose | use map_height_fitter to correct z value | [#179](https://github.com/tier4/driving_log_replayer/pull/179) | RT4-5532 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.3.17

Minor Tweak

| Module         | Feature              | Brief summary                                  | Pull request                                                   | Jira     | Contributor                                   |
| -------------- | -------------------- | ---------------------------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `localization` | evaluation condition | delete converged condition to start evaluation | [#193](https://github.com/tier4/driving_log_replayer/pull/193) | RT4-5630 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.3.16

Bug fix, Minor Tweak

| Module                  | Feature          | Brief summary                         | Pull request                                                   | Jira     | Contributor                                   |
| ----------------------- | ---------------- | ------------------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `obstacle_segmentation` | CMake            | fix build error due to lanelet2       | [#185](https://github.com/tier4/driving_log_replayer/pull/185) | -        | [hayato-m126](https://github.com/hayato-m126) |
| `localization`          | summary          | add average and std_dev of likelihood | [#184](https://github.com/tier4/driving_log_replayer/pull/184) | RT4-5584 | [hayato-m126](https://github.com/hayato-m126) |
| `localization`          | NDT availability | update monitoring topic               | [#187](https://github.com/tier4/driving_log_replayer/pull/187) | RT4-5570 | [kminoda](https://github.com/kminoda)         |

## Version 1.3.15

Minor Tweak

| Module | Feature | Brief summary          | Pull request                                                   | Jira | Contributor                                   |
| ------ | ------- | ---------------------- | -------------------------------------------------------------- | ---- | --------------------------------------------- |
| `all`  | ci      | add tier4 cspell-dicts | [#182](https://github.com/tier4/driving_log_replayer/pull/182) | -    | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.3.14

Minor Tweak

| Module         | Feature       | Brief summary        | Pull request                                                   | Jira     | Contributor                           |
| -------------- | ------------- | -------------------- | -------------------------------------------------------------- | -------- | ------------------------------------- |
| `localization` | state monitor | add ndt availability | [#178](https://github.com/tier4/driving_log_replayer/pull/178) | RT4-5570 | [kminoda](https://github.com/kminoda) |

## Version 1.3.13

Bug fix

| Module         | Feature  | Brief summary                | Pull request                                                   | Jira     | Contributor                                   |
| -------------- | -------- | ---------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `localization` | scenario | fill z value to initial pose | [#173](https://github.com/tier4/driving_log_replayer/pull/173) | RT4-5532 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.3.12

Documentation

| Module                              | Feature       | Brief summary                                  | Pull request                                                   | Jira     | Contributor                                   |
| ----------------------------------- | ------------- | ---------------------------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `perception, obstacle_segmentation` | documentation | add description of t4_dataset conversion tools | [#171](https://github.com/tier4/driving_log_replayer/pull/171) | RT1-2597 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.3.11

Bug fix, Minor Tweak

| Module                     | Feature        | Brief summary             | Pull request                                                   | Jira     | Contributor                                   |
| -------------------------- | -------------- | ------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `localization, perception` | error handling | catch transform exception | [#169](https://github.com/tier4/driving_log_replayer/pull/169) | RT4-5289 | [hayato-m126](https://github.com/hayato-m126) |
| `ALL`                      | cli            | support scenario.yml      | [#167](https://github.com/tier4/driving_log_replayer/pull/167) | RT4-4995 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.3.10

Minor Tweak

| Module | Feature       | Brief summary | Pull request                                                   | Jira | Contributor                               |
| ------ | ------------- | ------------- | -------------------------------------------------------------- | ---- | ----------------------------------------- |
| `ALL`  | documentation | fix link      | [#165](https://github.com/tier4/driving_log_replayer/pull/165) | -    | [vios-fish](https://github.com/vios-fish) |

## Version 1.3.9

Minor Tweak

| Module | Feature | Brief summary      | Pull request                                                   | Jira     | Contributor                                   |
| ------ | ------- | ------------------ | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `ALL`  | launch  | apply ShutdownOnce | [#163](https://github.com/tier4/driving_log_replayer/pull/163) | RT4-4690 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.3.8

Minor Tweak

| Module                     | Feature       | Brief summary             | Pull request                                                   | Jira     | Contributor                                   |
| -------------------------- | ------------- | ------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `perception, localization` | documentation | update result json format | [#161](https://github.com/tier4/driving_log_replayer/pull/161) | RT4-5057 | [hayato-m126](https://github.com/hayato-m126) |
| `perception`               | documentation | update bag topic          | [#160](https://github.com/tier4/driving_log_replayer/pull/160) | -        | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.3.7

Minor Tweak, Bug fix

| Module                                     | Feature         | Brief summary                    | Pull request                                                   | Jira     | Contributor                                   |
| ------------------------------------------ | --------------- | -------------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `perception, perception_2d`                | label attribute | support ignore attributes        | [#146](https://github.com/tier4/driving_log_replayer/pull/146) | RT4-4195 | [hayato-m126](https://github.com/hayato-m126) |
| `perception_2d`                            | target camera   | support multi camera             | [#148](https://github.com/tier4/driving_log_replayer/pull/148) | RT4-4195 | [hayato-m126](https://github.com/hayato-m126) |
| `obstacle_segmentation`                    | scenario format | check if input polygon clockwise | [#153](https://github.com/tier4/driving_log_replayer/pull/153) | RT4-4162 | [hayato-m126](https://github.com/hayato-m126) |
| `perception, perception_2d, traffic_light` | scenario format | check scenario format            | [#155](https://github.com/tier4/driving_log_replayer/pull/155) | -        | [hayato-m126](https://github.com/hayato-m126) |
| `localization, performance_diag`           | initial pose    | fix call initial pose service    | [#157](https://github.com/tier4/driving_log_replayer/pull/157) | RT4-4961 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.3.6

Minor Tweak

| Module | Feature | Brief summary                        | Pull request                                                   | Jira     | Contributor                                   |
| ------ | ------- | ------------------------------------ | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `all`  | launch  | use on_exit and delete event handler | [#151](https://github.com/tier4/driving_log_replayer/pull/151) | RT4-4162 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.3.5

Minor Tweak

| Module          | Feature    | Brief summary                 | Pull request                                                   | Jira     | Contributor                                   |
| --------------- | ---------- | ----------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `perception_2d` | tracking2d | support tracking2d evaluation | [#142](https://github.com/tier4/driving_log_replayer/pull/142) | RT4-4048 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.3.4

Minor Tweak

| Module         | Feature              | Brief summary                        | Pull request                                                   | Jira     | Contributor                                   |
| -------------- | -------------------- | ------------------------------------ | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `all`          | linter and formatter | update linter and formatter settings | [#139](https://github.com/tier4/driving_log_replayer/pull/139) | RT4-3916 | [hayato-m126](https://github.com/hayato-m126) |
| `localization` | documentation        | update scenario sample               | [#138](https://github.com/tier4/driving_log_replayer/pull/138) | -        | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.3.3

Major Changes

| Module             | Feature                  | Brief summary                      | Pull request                                                   | Jira       | Contributor                                   |
| ------------------ | ------------------------ | ---------------------------------- | -------------------------------------------------------------- | ---------- | --------------------------------------------- |
| `performance_diag` | update for ADAPI changes | delete fit_map_height service call | [#136](https://github.com/tier4/driving_log_replayer/pull/136) | T4PB-27288 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.3.2

Minor Tweak

| Module            | Feature      | Brief summary         | Pull request                                                   | Jira       | Contributor                                   |
| ----------------- | ------------ | --------------------- | -------------------------------------------------------------- | ---------- | --------------------------------------------- |
| `perception_diag` | result.jsonl | fix output msg format | [#134](https://github.com/tier4/driving_log_replayer/pull/134) | T4PB-27178 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.3.1

Minor Tweak

| Module         | Feature | Brief summary             | Pull request                                                   | Jira       | Contributor                                   |
| -------------- | ------- | ------------------------- | -------------------------------------------------------------- | ---------- | --------------------------------------------- |
| `localization` | launch  | disable perception module | [#132](https://github.com/tier4/driving_log_replayer/pull/132) | T4PB-27344 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.3.0

Major Changes

Ensure that you are using Autoware.universe that incorporates [this PR](https://github.com/autowarefoundation/autoware.universe/pull/2724).

| Module         | Feature                  | Brief summary                      | Pull request                                                   | Jira       | Contributor                                   |
| -------------- | ------------------------ | ---------------------------------- | -------------------------------------------------------------- | ---------- | --------------------------------------------- |
| `localization` | update for ADAPI changes | delete fit_map_height service call | [#129](https://github.com/tier4/driving_log_replayer/pull/129) | T4PB-27288 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.2.0

Major Changes

| Module                         | Feature       | Brief summary                                           | Pull request                                                   | Jira       | Contributor                                   |
| ------------------------------ | ------------- | ------------------------------------------------------- | -------------------------------------------------------------- | ---------- | --------------------------------------------- |
| `perception_2d, traffic_light` | new evaluator | add perception_2d_evaluator and traffic_light_evaluator | [#122](https://github.com/tier4/driving_log_replayer/pull/122) | T4PB-25343 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.22

Minor Tweak

| Module             | Feature    | Brief summary     | Pull request                                                   | Jira | Contributor                                   |
| ------------------ | ---------- | ----------------- | -------------------------------------------------------------- | ---- | --------------------------------------------- |
| `performance_diag` | bag record | drop camera topic | [#123](https://github.com/tier4/driving_log_replayer/pull/123) | -    | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.21

Documentation

| Module | Feature  | Brief summary               | Pull request                                                   | Jira       | Contributor                                   |
| ------ | -------- | --------------------------- | -------------------------------------------------------------- | ---------- | --------------------------------------------- |
| `all`  | document | update input bag topic list | [#120](https://github.com/tier4/driving_log_replayer/pull/120) | T4PB-26922 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.20

Minor Tweak

| Module       | Feature | Brief summary                    | Pull request                                                   | Jira       | Contributor                                   |
| ------------ | ------- | -------------------------------- | -------------------------------------------------------------- | ---------- | --------------------------------------------- |
| `perception` | library | update for perception_eval 1.0.4 | [#118](https://github.com/tier4/driving_log_replayer/pull/118) | T4PB-26566 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.19

Bug fix

| Module       | Feature | Brief summary            | Pull request                                                   | Jira | Contributor                                   |
| ------------ | ------- | ------------------------ | -------------------------------------------------------------- | ---- | --------------------------------------------- |
| `perception` | metrics | fix count tp, fp, and fn | [#116](https://github.com/tier4/driving_log_replayer/pull/116) | -    | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.18

Minor Tweak, Bug fix

| Module       | Feature | Brief summary                          | Pull request                                                   | Jira       | Contributor                                   |
| ------------ | ------- | -------------------------------------- | -------------------------------------------------------------- | ---------- | --------------------------------------------- |
| `perception` | library | update for perception_eval new feature | [#113](https://github.com/tier4/driving_log_replayer/pull/113) | T4PB-25343 | [hayato-m126](https://github.com/hayato-m126) |
| `cli`        | library | add dependent package                  | [#114](https://github.com/tier4/driving_log_replayer/pull/114) | T4PB-25810 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.17

Minor Tweak

| Module | Feature       | Brief summary        | Pull request                                                   | Jira       | Contributor                                   |
| ------ | ------------- | -------------------- | -------------------------------------------------------------- | ---------- | --------------------------------------------- |
| `all`  | visualization | update rviz settings | [#111](https://github.com/tier4/driving_log_replayer/pull/111) | T4PB-25343 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.16

Bug fix

| Module       | Feature      | Brief summary                         | Pull request                                                   | Jira       | Contributor                                   |
| ------------ | ------------ | ------------------------------------- | -------------------------------------------------------------- | ---------- | --------------------------------------------- |
| `perception` | json library | use simplejson to convert NaN to null | [#109](https://github.com/tier4/driving_log_replayer/pull/109) | T4PB-25810 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.15

Bug fix

| Module     | Feature   | Brief summary | Pull request                                                   | Jira | Contributor                                     |
| ---------- | --------- | ------------- | -------------------------------------------------------------- | ---- | ----------------------------------------------- |
| `document` | html link | fix link      | [#105](https://github.com/tier4/driving_log_replayer/pull/105) | -    | [KeisukeShima](https://github.com/KeisukeShima) |
| `document` | html link | fix link      | [#106](https://github.com/tier4/driving_log_replayer/pull/106) | -    | [hayato-m126](https://github.com/hayato-m126)   |

## Version 1.1.14

Minor Tweak

| Module     | Feature        | Brief summary                                | Pull request                                                   | Jira | Contributor                                   |
| ---------- | -------------- | -------------------------------------------- | -------------------------------------------------------------- | ---- | --------------------------------------------- |
| `document` | mkdocs setting | update setting for mkdocs material version 9 | [#102](https://github.com/tier4/driving_log_replayer/pull/102) | -    | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.13

Minor Tweak

| Module     | Feature       | Brief summary                         | Pull request                                                 | Jira       | Contributor                                   |
| ---------- | ------------- | ------------------------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `analyzer` | visualization | update obstacle segmentation analyzer | [#94](https://github.com/tier4/driving_log_replayer/pull/94) | T4PB-24956 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.12

Minor Tweak

| Module                  | Feature       | Brief summary                 | Pull request                                                 | Jira       | Contributor                                   |
| ----------------------- | ------------- | ----------------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `analyzer`              | refactor      | refactor analyzer             | [#89](https://github.com/tier4/driving_log_replayer/pull/89) | T4PB-24956 | [hayato-m126](https://github.com/hayato-m126) |
| `ci`                    | github action | update pre-commit-shfmt       | [#95](https://github.com/tier4/driving_log_replayer/pull/95) | -          | [hayato-m126](https://github.com/hayato-m126) |
| `perception`            | dependency    | update perception eval        | [#98](https://github.com/tier4/driving_log_replayer/pull/98) | T4PB-25347 | [hayato-m126](https://github.com/hayato-m126) |
| `obstacle_segmentation` | api           | use awapi to get stop reasons | [#99](https://github.com/tier4/driving_log_replayer/pull/99) | T4PB-25316 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.11

Bug fix

| Module | Feature             | Brief summary                                | Pull request                                                 | Jira       | Contributor                                     |
| ------ | ------------------- | -------------------------------------------- | ------------------------------------------------------------ | ---------- | ----------------------------------------------- |
| `cli`  | kill zombie process | Fixed to not kill processes unrelated to ros | [#92](https://github.com/tier4/driving_log_replayer/pull/92) | T4PB-25129 | [M(-\_-)Sakamoto](https://github.com/Motsu-san) |

## Version 1.1.10

Bug fix

| Module                  | Feature            | Brief summary             | Pull request                                                 | Jira       | Contributor                                   |
| ----------------------- | ------------------ | ------------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `cli`                   | sub command        | fix command name          | [#80](https://github.com/tier4/driving_log_replayer/pull/80) | -          | [hayato-m126](https://github.com/hayato-m126) |
| `all`                   | license            | use Apache-2.0            | [#82](https://github.com/tier4/driving_log_replayer/pull/82) | -          | [hayato-m126](https://github.com/hayato-m126) |
| `obstacle_segmentation` | dependency         | add exec depend           | [#83](https://github.com/tier4/driving_log_replayer/pull/83) | T4PB-23864 | [hayato-m126](https://github.com/hayato-m126) |
| `performance_diag`      | exception handling | handle TransformException | [#85](https://github.com/tier4/driving_log_replayer/pull/85) | T4PB-24969 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.9

Bug fix

| Module                  | Feature       | Brief summary    | Pull request                                                 | Jira       | Contributor                                   |
| ----------------------- | ------------- | ---------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `obstacle_segmentation` | visualization | fix import error | [#78](https://github.com/tier4/driving_log_replayer/pull/78) | T4PB-23864 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.8

Minor Tweak, Bug fix

| Module                  | Feature       | Brief summary                                                                 | Pull request                                                 | Jira       | Contributor                                   |
| ----------------------- | ------------- | ----------------------------------------------------------------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `obstacle_segmentation` | marker color  | The result of the topic's rate is reflected in the color of the bounding box. | [#72](https://github.com/tier4/driving_log_replayer/pull/72) | T4PB-23864 | [hayato-m126](https://github.com/hayato-m126) |
| `obstacle_segmentation` | visualization | add graph data to result.jsonl                                                | [#73](https://github.com/tier4/driving_log_replayer/pull/73) | T4PB-23864 | [hayato-m126](https://github.com/hayato-m126) |
| `performance_diag`      | localization  | fix infinite wait when localization module off                                | [#75](https://github.com/tier4/driving_log_replayer/pull/75) | T4PB-24705 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.7

Minor Tweak, Bug fix

| Module                             | Feature       | Brief summary                                        | Pull request                                                 | Jira       | Contributor                                   |
| ---------------------------------- | ------------- | ---------------------------------------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `obstacle_segmentation`            | visualization | publish and record data for visualization            | [#67](https://github.com/tier4/driving_log_replayer/pull/67) | T4PB-23864 | [hayato-m126](https://github.com/hayato-m126) |
| `performance_diag`                 | initial pose  | delete uninitialized publisher                       | [#68](https://github.com/tier4/driving_log_replayer/pull/68) | T4PB-24491 | [hayato-m126](https://github.com/hayato-m126) |
| `localization`, `performance_diag` | initial pose  | use map fit service to correct `initialpose z value` | [#70](https://github.com/tier4/driving_log_replayer/pull/70) | T4PB-24491 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.6

Minor Tweak

| Module                             | Feature      | Brief summary               | Pull request                                                 | Jira       | Contributor                                   |
| ---------------------------------- | ------------ | --------------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `localization`, `performance_diag` | initial pose | set initial pose via ad-api | [#65](https://github.com/tier4/driving_log_replayer/pull/65) | T4PB-24491 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.5

Bug fix

| Module       | Feature      | Brief summary                                            | Pull request                                                 | Jira       | Contributor                                   |
| ------------ | ------------ | -------------------------------------------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `perception` | marker color | different colors are applied to tp_gt, tp_est, fp and fn | [#61](https://github.com/tier4/driving_log_replayer/pull/61) | T4PB-24425 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.4

Major Changes

| Module       | Feature        | Brief summary                                 | Pull request                                                 | Jira       | Contributor                                   |
| ------------ | -------------- | --------------------------------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `perception` | launch process | Removed waiting for conversion of onnx files. | [#59](https://github.com/tier4/driving_log_replayer/pull/59) | T4PB-24362 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.3

Major Changes

| Module                  | Feature             | Brief summary                                    | Pull request                                                 | Jira       | Contributor                                   |
| ----------------------- | ------------------- | ------------------------------------------------ | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `obstacle segmentation` | evaluation settings | set evaluation period for each bbox              | [#51](https://github.com/tier4/driving_log_replayer/pull/51) | T4PB-24138 | [hayato-m126](https://github.com/hayato-m126) |
| `obstacle segmentation` | result file         | output timestamp of bounding box and point cloud | [#52](https://github.com/tier4/driving_log_replayer/pull/52) | T4PB-24238 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.2

Minor Tweak

| Module | Feature    | Brief summary                                         | Pull request                                                 | Jira       | Contributor                                   |
| ------ | ---------- | ----------------------------------------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `all`  | cli        | kill zombie process                                   | [#45](https://github.com/tier4/driving_log_replayer/pull/45) | T4PB-24124 | [hayato-m126](https://github.com/hayato-m126) |
| `all`  | cli        | fix generated command to kill zombie process          | [#46](https://github.com/tier4/driving_log_replayer/pull/46) | T4PB-24124 | [hayato-m126](https://github.com/hayato-m126) |
| `all`  | cli        | modified commands to avoid killing the running shell. | [#47](https://github.com/tier4/driving_log_replayer/pull/47) | T4PB-24124 | [hayato-m126](https://github.com/hayato-m126) |
| `all`  | dictionary | add local dictionary                                  | [#48](https://github.com/tier4/driving_log_replayer/pull/48) | T4PB-23864 | [hayato-m126](https://github.com/hayato-m126) |
| `all`  | code style | fix lit check                                         | [#49](https://github.com/tier4/driving_log_replayer/pull/49) | T4PB-23864 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.1

Major Changes

| Module     | Feature  | Brief summary                          | Pull request                                                 | Jira       | Contributor                                     |
| ---------- | -------- | -------------------------------------- | ------------------------------------------------------------ | ---------- | ----------------------------------------------- |
| `analyzer` | analyzer | add analyzer to visualize result.jsonl | [#42](https://github.com/tier4/driving_log_replayer/pull/42) | T4PB-23864 | [KeisukeShima](https://github.com/KeisukeShima) |
| `analyzer` | analyzer | fix pre-commit check                   | [#43](https://github.com/tier4/driving_log_replayer/pull/43) | T4PB-23864 | [hayato-m126](https://github.com/hayato-m126)   |
| `analyzer` | analyzer | add cli command and ros2 launch        | [#44](https://github.com/tier4/driving_log_replayer/pull/44) | T4PB-23864 | [hayato-m126](https://github.com/hayato-m126)   |

## Version 1.1.0

Major Changes

| Module                  | Feature                      | Brief summary                                            | Pull request                                                 | Jira       | Contributor                                   |
| ----------------------- | ---------------------------- | -------------------------------------------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `obstacle_segmentation` | point cloud topic rate check | update diagnostics status name of point cloud topic rate | [#40](https://github.com/tier4/driving_log_replayer/pull/40) | T4PB-23868 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.0.12

Bug fix

| Module                  | Feature      | Brief summary                  | Pull request                                                 | Jira       | Contributor                                   |
| ----------------------- | ------------ | ------------------------------ | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `obstacle_segmentation` | frame result | fix status and judgement logic | [#37](https://github.com/tier4/driving_log_replayer/pull/37) | T4PB-23309 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.0.11

Major Changes

| Module                  | Feature   | Brief summary                                 | Pull request                                                 | Jira       | Contributor                                   |
| ----------------------- | --------- | --------------------------------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `obstacle_segmentation` | test mode | support 3 test mode for obstacle_segmentation | [#35](https://github.com/tier4/driving_log_replayer/pull/35) | T4PB-23309 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.0.10

Minor Tweaks

| Module | Feature       | Brief summary                 | Pull request                                                 | Jira       | Contributor                                   |
| ------ | ------------- | ----------------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `all`  | documentation | update Japanese Documentation | [#30](https://github.com/tier4/driving_log_replayer/pull/30) | T4PB-23145 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.0.9

Bug Fix

| Module | Feature       | Brief summary         | Pull request                                                 | Jira       | Contributor                                   |
| ------ | ------------- | --------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `all`  | documentation | fix MkDocs Dependency | [#28](https://github.com/tier4/driving_log_replayer/pull/28) | T4PB-21730 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.0.8

Minor Tweaks

| Module | Feature       | Brief summary             | Pull request                                                 | Jira       | Contributor                                   |
| ------ | ------------- | ------------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `all`  | documentation | add English Documentation | [#20](https://github.com/tier4/driving_log_replayer/pull/20) | T4PB-21730 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.0.7

Bug Fix

| Module       | Feature                  | Brief summary | Pull request                                                 | Jira       | Contributor                                   |
| ------------ | ------------------------ | ------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `perception` | pass fail decision logic | fix logic     | [#25](https://github.com/tier4/driving_log_replayer/pull/25) | T4PB-22918 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.0.6

Bug Fix

| Module   | Feature              | Brief summary                                                               | Pull request                                                 | Jira       | Contributor                                   |
| -------- | -------------------- | --------------------------------------------------------------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `common` | publish initial pose | Cast the value of the initial position described in the scenario to a float | [#22](https://github.com/tier4/driving_log_replayer/pull/22) | T4PB-22700 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.0.5

Major Changes

| Module       | Feature             | Brief summary                    | Pull request                                                 | Jira       | Contributor                                   |
| ------------ | ------------------- | -------------------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `perception` | database evaluation | output database_result.json file | [#17](https://github.com/tier4/driving_log_replayer/pull/17) | T4PB-22358 | [hayato-m126](https://github.com/hayato-m126) |
| `perception` | database evaluation | fix exit with failure 1 file     | [#18](https://github.com/tier4/driving_log_replayer/pull/18) | T4PB-22358 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.0.4

Minor Tweaks

| Module | Feature       | Brief summary                        | Pull request                                                 | Jira       | Contributor                                   |
| ------ | ------------- | ------------------------------------ | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `all`  | documentation | Adopted MkDocs as documentation tool | [#14](https://github.com/tier4/driving_log_replayer/pull/14) | T4PB-22101 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.0.3

Major Changes

| Module       | Feature          | Brief summary                          | Pull request                                                 | Jira       | Contributor                                   |
| ------------ | ---------------- | -------------------------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `perception` | launch parameter | delete use_pointcloud_container option | [#12](https://github.com/tier4/driving_log_replayer/pull/12) | T4PB-21729 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.0.2

Minor Tweaks

| Module | Feature       | Brief summary                     | Pull request                                               | Jira       | Contributor                                   |
| ------ | ------------- | --------------------------------- | ---------------------------------------------------------- | ---------- | --------------------------------------------- |
| `all`  | documentation | update sample scenario and config | [#6](https://github.com/tier4/driving_log_replayer/pull/6) | T4PB-21729 | [hayato-m126](https://github.com/hayato-m126) |
| `all`  | documentation | update sample scenario            | [#7](https://github.com/tier4/driving_log_replayer/pull/7) | T4PB-21729 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.0.1

Minor Tweaks

| Module                  | Feature       | Brief summary                              | Pull request                                               | Jira       | Contributor                                   |
| ----------------------- | ------------- | ------------------------------------------ | ---------------------------------------------------------- | ---------- | --------------------------------------------- |
| `perception`            | visualization | shorten bounding box uuid to display       | [#2](https://github.com/tier4/driving_log_replayer/pull/2) | T4PB-21729 | [hayato-m126](https://github.com/hayato-m126) |
| `obstacle_segmentation` | remap topic   | remap concatenated point cloud in bag file | [#4](https://github.com/tier4/driving_log_replayer/pull/4) | T4PB-21729 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.0.0

Major Changes

| Module | Feature | Brief summary  | Pull request                                               | Jira | Contributor                                   |
| ------ | ------- | -------------- | ---------------------------------------------------------- | ---- | --------------------------------------------- |
| `all`  | oss     | publish as oss | [#0](https://github.com/tier4/driving_log_replayer/pull/0) | -    | [hayato-m126](https://github.com/hayato-m126) |
