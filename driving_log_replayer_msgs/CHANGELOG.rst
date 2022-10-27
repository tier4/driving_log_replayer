^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package driving_log_replayer_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
