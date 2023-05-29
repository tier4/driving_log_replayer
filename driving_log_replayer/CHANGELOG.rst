^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package driving_log_replayer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* docs: update document
* Contributors: Hayato Mizushima

1.3.7 (2023-05-19)
------------------
* fix: `#156 <https://github.com/tier4/driving_log_replayer/issues/156>`_ initialpose service call (`#157 <https://github.com/tier4/driving_log_replayer/issues/157>`_)
  closes: `#156 <https://github.com/tier4/driving_log_replayer/issues/156>`_
* fix: Handling incompatible scenario (`#155 <https://github.com/tier4/driving_log_replayer/issues/155>`_)
* fix: typo
* fix: Handling incompatible scenario
* feat: check if input polygon clockwise (`#153 <https://github.com/tier4/driving_log_replayer/issues/153>`_)
  closes `#143 <https://github.com/tier4/driving_log_replayer/issues/143>`_
* feat: `#147 <https://github.com/tier4/driving_log_replayer/issues/147>`_ perception 2d support multi camera (`#148 <https://github.com/tier4/driving_log_replayer/issues/148>`_)
  Co-authored-by: Kotaro Uetake <60615504+ktro2828@users.noreply.github.com>
  closes: `#147 <https://github.com/tier4/driving_log_replayer/issues/147>`_
* feat: `#145 <https://github.com/tier4/driving_log_replayer/issues/145>`_ perception support ignore attributes (`#146 <https://github.com/tier4/driving_log_replayer/issues/146>`_)
  Co-authored-by: Kotaro Uetake <60615504+ktro2828@users.noreply.github.com>
  closes: `#145 <https://github.com/tier4/driving_log_replayer/issues/145>`_
* Contributors: Hayato Mizushima

1.3.6 (2023-04-25)
------------------
* feat: use on_exit delete event handler
* Contributors: Hayato Mizushima

1.3.5 (2023-04-20)
------------------
* feat: tracking2d evaluation (`#142 <https://github.com/tier4/driving_log_replayer/issues/142>`_)
  closes: `#141 <https://github.com/tier4/driving_log_replayer/issues/141>`_
* Revert "feat: add shutdown"
  This reverts commit e2928caf26950efccace6194dc2be48823643e30.
* Revert "feat: check if input polygon is clockwise"
  This reverts commit ca994e709d811816a547ed1185b2b8806fc91611.
* feat: add shutdown
* feat: check if input polygon is clockwise
* Contributors: Hayato Mizushima

1.3.4 (2023-04-17)
------------------
* feat: update linter and formatter settings
* docs: update sample scenario
* Contributors: Hayato Mizushima

1.3.3 (2023-04-03)
------------------
* feat(diag): delete fit map height service call
* Contributors: Hayato Mizushima

1.3.2 (2023-03-30)
------------------
* chore: fix result.jsonl msg format
* Contributors: Hayato Mizushima

1.3.1 (2023-03-28)
------------------
* fix: disable perception in localization launch (`#132 <https://github.com/tier4/driving_log_replayer/issues/132>`_)
  closes: `#131 <https://github.com/tier4/driving_log_replayer/issues/131>`_
* Contributors: Hayato Mizushima

1.3.0 (2023-03-24)
------------------
* feat: delete fit_map_height service call (`#129 <https://github.com/tier4/driving_log_replayer/issues/129>`_)
  closes: `#128 <https://github.com/tier4/driving_log_replayer/issues/128>`_
* Contributors: Hayato Mizushima

1.2.0 (2023-03-23)
------------------
* feat: `#104 <https://github.com/tier4/driving_log_replayer/issues/104>`_ 2d perception (`#122 <https://github.com/tier4/driving_log_replayer/issues/122>`_)
* Contributors: Hayato Mizushima

1.1.22 (2023-03-17)
-------------------
* chore: drop camera image (`#123 <https://github.com/tier4/driving_log_replayer/issues/123>`_)
* Contributors: Hayato Mizushima

1.1.21 (2023-03-09)
-------------------
* docs: update input bag topic list
* Contributors: Hayato Mizushima

1.1.20 (2023-03-06)
-------------------
* fix: lint
* fix: lint and comment out debug code
* chore: comment out analyzer
* chore: add debug code
* chore: add debug code to count traffic singal cb
* feat: update condition
* fix: convert dict
* feat: add 2d analyzer
* feat: update 3d analyzer
* fix: rename
* feat: update traffic light node
* feat: update traffic light node
* feat: update
* feat: output metrics score
* fix: TP FP FN count
* fix: work
* fix: data access
* feat: set camera no from camera type
* feat: update 2d detection
* feat: set perception_mode
* fix: lint
* feat: add traffice light evaluator
* fix: rviz file
* feat: update node
* feat: update scenario
* fix: CMakeList
* WIP
* feat: add file
* Contributors: Hayato Mizushima

1.1.19 (2023-02-24)
-------------------
* fix: count tp fp fn (`#116 <https://github.com/tier4/driving_log_replayer/issues/116>`_)
* Contributors: Hayato Mizushima

1.1.18 (2023-02-17)
-------------------
* feat: update for perception_eval PR `#12 <https://github.com/tier4/driving_log_replayer/issues/12>`_ (`#113 <https://github.com/tier4/driving_log_replayer/issues/113>`_)
* fix: add cli dependency (`#114 <https://github.com/tier4/driving_log_replayer/issues/114>`_)
* Contributors: Hayato Mizushima

1.1.17 (2023-02-14)
-------------------
* feat: update rviz (`#111 <https://github.com/tier4/driving_log_replayer/issues/111>`_)
* Contributors: Hayato Mizushima

1.1.16 (2023-02-08)
-------------------
* fix: `#108 <https://github.com/tier4/driving_log_replayer/issues/108>`_ perception json value (`#109 <https://github.com/tier4/driving_log_replayer/issues/109>`_)
  closes: `#108 <https://github.com/tier4/driving_log_replayer/issues/108>`_
* Contributors: Hayato Mizushima

1.1.15 (2023-02-01)
-------------------
* docs: fix lint
* Contributors: Hayato Mizushima

1.1.14 (2023-01-31)
-------------------
* docs: update mkdocs setting
* Contributors: Hayato Mizushima

1.1.13 (2023-01-31)
-------------------
* feat: `#93 <https://github.com/tier4/driving_log_replayer/issues/93>`_ update obstacle segmentation analyzer (`#94 <https://github.com/tier4/driving_log_replayer/issues/94>`_)
  closes: `#93 <https://github.com/tier4/driving_log_replayer/issues/93>`_
* Contributors: Hayato Mizushima

1.1.12 (2023-01-30)
-------------------
* feat: `#96 <https://github.com/tier4/driving_log_replayer/issues/96>`_ topic stop reasons (`#99 <https://github.com/tier4/driving_log_replayer/issues/99>`_)
  closes: `#96 <https://github.com/tier4/driving_log_replayer/issues/96>`_
* feat: `#97 <https://github.com/tier4/driving_log_replayer/issues/97>`_ update perception eval (`#98 <https://github.com/tier4/driving_log_replayer/issues/98>`_)
  closes: `#97 <https://github.com/tier4/driving_log_replayer/issues/97>`_
* refactor: `#88 <https://github.com/tier4/driving_log_replayer/issues/88>`_ analyzer (`#89 <https://github.com/tier4/driving_log_replayer/issues/89>`_)
  closes: `#88 <https://github.com/tier4/driving_log_replayer/issues/88>`_
* Contributors: Hayato Mizushima

1.1.11 (2023-01-17)
-------------------
* fix: cli kill zombie process
* Contributors: Hayato Mizushima

1.1.10 (2023-01-12)
-------------------
* fix: catch TransformException (`#85 <https://github.com/tier4/driving_log_replayer/issues/85>`_)
* fix: add exec depend (`#83 <https://github.com/tier4/driving_log_replayer/issues/83>`_)
* chore: license (`#82 <https://github.com/tier4/driving_log_replayer/issues/82>`_)
* Contributors: Hayato Mizushima

1.1.9 (2022-12-25)
------------------
* fix: no module named plotly (`#78 <https://github.com/tier4/driving_log_replayer/issues/78>`_)
* Contributors: Hayato Mizushima

1.1.8 (2022-12-22)
------------------
* fix(performance_diag): infinite wait at initialization
* Feat/`#57 <https://github.com/tier4/driving_log_replayer/issues/57>`_ obstacle segmentation visualization (`#73 <https://github.com/tier4/driving_log_replayer/issues/73>`_)
* fix: change bounding box color (`#72 <https://github.com/tier4/driving_log_replayer/issues/72>`_)
* Contributors: Hayato Mizushima

1.1.7 (2022-12-20)
------------------
* feat(performance_diag): use map fit
* feat(localization): use map fit
* fix: delete uninitialized publisher (`#68 <https://github.com/tier4/driving_log_replayer/issues/68>`_)
* feat: `#57 <https://github.com/tier4/driving_log_replayer/issues/57>`_ obstacle segmentation visualization (`#67 <https://github.com/tier4/driving_log_replayer/issues/67>`_)
* Contributors: Hayato Mizushima

1.1.6 (2022-12-19)
------------------
* fix: lint
* feat: apply initial pose service for performance diag
* fix: service callback
* feat(WIP): time cb works but response is not ready
* feat(WIP): use ad-api
* Contributors: Hayato Mizushima

1.1.5 (2022-12-14)
------------------
* fix: marker color
* Contributors: Hayato Mizushima

1.1.4 (2022-12-13)
------------------
* chore: git mv
* feat: delete perception_starter
* feat: delete onnx file convert wait
* Contributors: Hayato Mizushima

1.1.3 (2022-12-13)
------------------
* feat: `#51 <https://github.com/tier4/driving_log_replayer/issues/51>`_ set evaluation period for each bbox (`#54 <https://github.com/tier4/driving_log_replayer/issues/54>`_)
  closes: `#51 <https://github.com/tier4/driving_log_replayer/issues/51>`_
* feat: `#52 <https://github.com/tier4/driving_log_replayer/issues/52>`_-output-timestamp-of-bbox-and-pcd (`#53 <https://github.com/tier4/driving_log_replayer/issues/53>`_)
  closes: `#52 <https://github.com/tier4/driving_log_replayer/issues/52>`_
* Contributors: Hayato Mizushima

1.1.2 (2022-12-07)
------------------
* feat(cli): kill zombie process
* fix: lint check (`#49 <https://github.com/tier4/driving_log_replayer/issues/49>`_)
* Contributors: Hayato Mizushima

1.1.1 (2022-12-01)
------------------
* feat: analyzer (`#44 <https://github.com/tier4/driving_log_replayer/issues/44>`_)
* fix: pre-commit-check (`#43 <https://github.com/tier4/driving_log_replayer/issues/43>`_)
* add driving_log_replayer_analyzer (`#42 <https://github.com/tier4/driving_log_replayer/issues/42>`_)
* Contributors: Hayato Mizushima, Keisuke Shima

1.1.0 (2022-11-29)
------------------
* feat(obstacle_segmentation): `#39 <https://github.com/tier4/driving_log_replayer/issues/39>`_ update diagnostic status name (`#40 <https://github.com/tier4/driving_log_replayer/issues/40>`_)
  closes: `#39 <https://github.com/tier4/driving_log_replayer/issues/39>`_
* Contributors: Hayato Mizushima

1.0.12 (2022-11-11)
-------------------
* fix: obstacle segmentation frame result (`#37 <https://github.com/tier4/driving_log_replayer/issues/37>`_)
* Contributors: Hayato Mizushima

1.0.11 (2022-11-11)
-------------------
* feat: `#33 <https://github.com/tier4/driving_log_replayer/issues/33>`_ test mode for obstacle segmentation (`#35 <https://github.com/tier4/driving_log_replayer/issues/35>`_)
  closes: `#33 <https://github.com/tier4/driving_log_replayer/issues/33>`_
* Contributors: Hayato Mizushima

1.0.10 (2022-11-07)
-------------------
* docs: update Japanese Documentation
* Contributors: Hayato Mizushima

1.0.9 (2022-11-04)
------------------
* fix: MkDocs Dependency
* Contributors: Hayato Mizushima

1.0.8 (2022-11-04)
------------------
* docs: English document
* Contributors: Hayato Mizushima

1.0.7 (2022-10-30)
------------------
* fix: frame pass fail logic (`#25 <https://github.com/tier4/driving_log_replayer/issues/25>`_)
  closes: `#24 <https://github.com/tier4/driving_log_replayer/issues/24>`_
* Contributors: Hayato Mizushima

1.0.6 (2022-10-27)
------------------
* fix: cast number from yaml file (`#22 <https://github.com/tier4/driving_log_replayer/issues/22>`_)
  closes: `#21 <https://github.com/tier4/driving_log_replayer/issues/21>`_
* Contributors: Hayato Mizushima

1.0.5 (2022-10-19)
------------------
* Fix/`#16 <https://github.com/tier4/driving_log_replayer/issues/16>`_ database result node failure 1 (`#18 <https://github.com/tier4/driving_log_replayer/issues/18>`_)
  * chore: git mv
  * fix: change database result generation command
* Feat/`#16 <https://github.com/tier4/driving_log_replayer/issues/16>`_ perception database evaluation result (`#17 <https://github.com/tier4/driving_log_replayer/issues/17>`_)
  * feat: save database result as file
  * fix: add command
  * fix: parameter
  * fix: arg name
  * fix: typo
  * fix: get pkl file and add debug program
  * fix: shutdown
  * fix: lint
  * fix: lint
* Contributors: Hayato Mizushima

1.0.4 (2022-10-14)
------------------
* docs: use MkDocs
* Contributors: Hayato Mizushima

1.0.3 (2022-10-13)
------------------
* feat: delete use pointcloud container false (`#12 <https://github.com/tier4/driving_log_replayer/issues/12>`_)
  closes: `#11 <https://github.com/tier4/driving_log_replayer/issues/11>`_
* Contributors: Hayato Mizushima

1.0.2 (2022-10-12)
------------------
* docs: update sample
* Contributors: Hayato Mizushima

1.0.1 (2022-10-11)
------------------
* Feat/obstacle segmentation remap topic in t4 dataset bag (`#4 <https://github.com/tier4/driving_log_replayer/issues/4>`_)
  * feat(obstacle_segmentation): remap concatenated_pointcloud
  * feat(obstacle_segmentation): remap tf_static
  * feat: use tf_static in bag
* Revert "chore: remap tf in bag (`#3 <https://github.com/tier4/driving_log_replayer/issues/3>`_)"
  This reverts commit e6dac86f53fa239f53df069f7da9b3bc66c31f07.
* chore: remap tf in bag (`#3 <https://github.com/tier4/driving_log_replayer/issues/3>`_)
* Chore/perception UUID (`#2 <https://github.com/tier4/driving_log_replayer/issues/2>`_)
  * feat(perception): shorten bounding box uuid
  * chore: change log
* Contributors: Hayato Mizushima

1.0.0 (2022-09-28)
------------------
* oss
* Contributors: Hayato Mizushima
