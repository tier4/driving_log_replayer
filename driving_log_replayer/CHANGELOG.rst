^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package driving_log_replayer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.11.1 (2023-12-20)
-------------------
* feat: add maph criteria (`#337 <https://github.com/tier4/driving_log_replayer/issues/337>`_)
  Co-authored-by: Hayato Mizushima <hayato-m126@users.noreply.github.com>
* Contributors: kminoda

1.11.0 (2023-12-19)
-------------------
* feat: cli allow arbitrary arguments (`#333 <https://github.com/tier4/driving_log_replayer/issues/333>`_)
* Contributors: Hayato Mizushima

1.10.6 (2023-12-15)
-------------------
* fix: `#331 <https://github.com/tier4/driving_log_replayer/issues/331>`_ check footprint length (`#332 <https://github.com/tier4/driving_log_replayer/issues/332>`_)
* Contributors: Hayato Mizushima

1.10.5 (2023-12-08)
-------------------
* chore: Stop PLAYER after standing for 1 second.
* refactor: cli
* Contributors: Hayato Mizushima

1.10.4 (2023-12-07)
-------------------
* fix: overwrite pose_source and twist_source (`#327 <https://github.com/tier4/driving_log_replayer/issues/327>`_)
* refactor: launch arg (`#326 <https://github.com/tier4/driving_log_replayer/issues/326>`_)
* Contributors: Hayato Mizushima

1.10.3 (2023-12-04)
-------------------
* feat: bag controller (`#319 <https://github.com/tier4/driving_log_replayer/issues/319>`_)
* feat: save the log displayed in the console as a file (`#320 <https://github.com/tier4/driving_log_replayer/issues/320>`_)
* fix: github actions deprecating command (`#321 <https://github.com/tier4/driving_log_replayer/issues/321>`_)
* Contributors: Hayato Mizushima

1.10.2 (2023-12-01)
-------------------
* fix: perception mode default (`#317 <https://github.com/tier4/driving_log_replayer/issues/317>`_)
* chore: test perception criteria custom level (`#316 <https://github.com/tier4/driving_log_replayer/issues/316>`_)
* Contributors: Hayato Mizushima

1.10.1 (2023-11-30)
-------------------
* fix: perception criteria validation bug (`#314 <https://github.com/tier4/driving_log_replayer/issues/314>`_)
* fix: fix ruff S602 rule (`#313 <https://github.com/tier4/driving_log_replayer/issues/313>`_)
* chore: type hint (`#312 <https://github.com/tier4/driving_log_replayer/issues/312>`_)
* Contributors: Hayato Mizushima

1.10.0 (2023-11-28)
-------------------
* feat: override record topics (`#301 <https://github.com/tier4/driving_log_replayer/issues/301>`_)
* feat: scenario class (`#306 <https://github.com/tier4/driving_log_replayer/issues/306>`_)
* Contributors: Hayato Mizushima

1.9.1 (2023-11-21)
------------------
* fix no data criteria (`#305 <https://github.com/tier4/driving_log_replayer/issues/305>`_)
  Co-authored-by: YoshiRi <YoshiRi@users.noreply.github.com>
* refactor: diag (`#303 <https://github.com/tier4/driving_log_replayer/issues/303>`_)
* Contributors: Hayato Mizushima, Yoshi Ri

1.9.0 (2023-11-14)
------------------
* feat: obstacle segmentation test (`#273 <https://github.com/tier4/driving_log_replayer/issues/273>`_)
* Contributors: Hayato Mizushima

1.8.4 (2023-11-08)
------------------
* feat: parameterize perception mode (`#299 <https://github.com/tier4/driving_log_replayer/issues/299>`_)
* Contributors: Hayato Mizushima

1.8.3 (2023-11-07)
------------------
* docs: update result format (`#297 <https://github.com/tier4/driving_log_replayer/issues/297>`_)
* Contributors: Hayato Mizushima

1.8.2 (2023-11-07)
------------------
* feat: perception 2d test (`#295 <https://github.com/tier4/driving_log_replayer/issues/295>`_)
* fix: cli create output directory (`#294 <https://github.com/tier4/driving_log_replayer/issues/294>`_)
* feat: perception test (`#292 <https://github.com/tier4/driving_log_replayer/issues/292>`_)
* Contributors: Hayato Mizushima

1.8.1 (2023-11-02)
------------------
* feat: traffic light test (`#255 <https://github.com/tier4/driving_log_replayer/issues/255>`_)
* refactor: common module (`#288 <https://github.com/tier4/driving_log_replayer/issues/288>`_)
* chore: delete meaningless joinpath (`#287 <https://github.com/tier4/driving_log_replayer/issues/287>`_)
* refactor: use pathlib (`#286 <https://github.com/tier4/driving_log_replayer/issues/286>`_)
* Contributors: Hayato Mizushima

1.8.0 (2023-10-19)
------------------
* perf: fixed to use `/localization/pose_estimator/initial_to_result_relative_pose` (`#282 <https://github.com/tier4/driving_log_replayer/issues/282>`_)
  Co-authored-by: Hayato Mizushima <hayato-m126@users.noreply.github.com>
* docs: update (`#283 <https://github.com/tier4/driving_log_replayer/issues/283>`_)
* Contributors: SakodaShintaro

1.7.0 (2023-10-16)
------------------
* feat(perception): allow to specify perception mode in scenario (`#279 <https://github.com/tier4/driving_log_replayer/issues/279>`_)
  Co-authored-by: Hayato Mizushima <hayato-m126@users.noreply.github.com>
* fix: TCH002 (`#278 <https://github.com/tier4/driving_log_replayer/issues/278>`_)
* Contributors: Hayato Mizushima, Kotaro Uetake

1.6.7 (2023-10-12)
------------------
* fix(perception): remove `typing_extensions` (`#277 <https://github.com/tier4/driving_log_replayer/issues/277>`_)
* Contributors: Kotaro Uetake

1.6.6 (2023-10-06)
------------------
* feat(perception): update perception criteria (`#272 <https://github.com/tier4/driving_log_replayer/issues/272>`_)
  Co-authored-by: ktro2828 <ktro2828@users.noreply.github.com>
  Co-authored-by: Hayato Mizushima <hayato-m126@users.noreply.github.com>
* fix: dependency (`#274 <https://github.com/tier4/driving_log_replayer/issues/274>`_)
* feat: eval_conversions add test (`#271 <https://github.com/tier4/driving_log_replayer/issues/271>`_)
* feat: diag test (`#269 <https://github.com/tier4/driving_log_replayer/issues/269>`_)
* Contributors: Hayato Mizushima, Kotaro Uetake

1.6.5 (2023-09-29)
------------------
* feat: poetry add group docs (`#265 <https://github.com/tier4/driving_log_replayer/issues/265>`_)
* build: add requirements and install operation to overwrite python libraries (`#266 <https://github.com/tier4/driving_log_replayer/issues/266>`_)
* Contributors: Kotaro Uetake

1.6.4 (2023-09-28)
------------------
* fix: mkdocs github actions library install
* Contributors: Hayato Mizushima

1.6.3 (2023-09-28)
------------------
* fix(yabloc): fix rosbag url (`#261 <https://github.com/tier4/driving_log_replayer/issues/261>`_)
* Contributors: kminoda

1.6.2 (2023-09-27)
------------------
* feat: add ar_tag_based_localizer evaluation (`#258 <https://github.com/tier4/driving_log_replayer/issues/258>`_)
  Co-authored-by: Hayato Mizushima <hayato-m126@users.noreply.github.com>
* feat: eagleye test (`#253 <https://github.com/tier4/driving_log_replayer/issues/253>`_)
* feat: yabloc test (`#252 <https://github.com/tier4/driving_log_replayer/issues/252>`_)
* chore: update variable name (`#257 <https://github.com/tier4/driving_log_replayer/issues/257>`_)
* feat: add frame success (`#256 <https://github.com/tier4/driving_log_replayer/issues/256>`_)
* chore: evaluation item (`#254 <https://github.com/tier4/driving_log_replayer/issues/254>`_)
* feat: update localization availability (`#251 <https://github.com/tier4/driving_log_replayer/issues/251>`_)
* chore: change topic result success initial value (`#250 <https://github.com/tier4/driving_log_replayer/issues/250>`_)
* feat: topic result (`#249 <https://github.com/tier4/driving_log_replayer/issues/249>`_)
* feat: localization test (`#248 <https://github.com/tier4/driving_log_replayer/issues/248>`_)
* feat: result test (`#246 <https://github.com/tier4/driving_log_replayer/issues/246>`_)
* Contributors: Hayato Mizushima, SakodaShintaro

1.6.1 (2023-09-12)
------------------
* fix: restore default value (`#244 <https://github.com/tier4/driving_log_replayer/issues/244>`_)
* Contributors: Hayato Mizushima

1.6.0 (2023-09-11)
------------------
* feat!: drop galactic support (`#242 <https://github.com/tier4/driving_log_replayer/issues/242>`_)
* refactor: apply ruff rules (`#241 <https://github.com/tier4/driving_log_replayer/issues/241>`_)
* refactor: apply ruff rules (`#240 <https://github.com/tier4/driving_log_replayer/issues/240>`_)
* refactor: comma (`#239 <https://github.com/tier4/driving_log_replayer/issues/239>`_)
* refactor: type hint (`#238 <https://github.com/tier4/driving_log_replayer/issues/238>`_)
* refactor: add type hint
* refactor: add type hint to main method
* fix: annotate void function
* refactor: apply ruff rules (`#237 <https://github.com/tier4/driving_log_replayer/issues/237>`_)
* fix: ERA
* fix: PLR0911
* refactor: evaluator abstract base class (`#236 <https://github.com/tier4/driving_log_replayer/issues/236>`_)
* refactor: obstacle segmentation abc (`#234 <https://github.com/tier4/driving_log_replayer/issues/234>`_)
* refactor: lookup transform (`#233 <https://github.com/tier4/driving_log_replayer/issues/233>`_)
* refactor: traffic light abc (`#232 <https://github.com/tier4/driving_log_replayer/issues/232>`_)
* refactor: 2d abc (`#231 <https://github.com/tier4/driving_log_replayer/issues/231>`_)
* refactor: perception abc (`#230 <https://github.com/tier4/driving_log_replayer/issues/230>`_)
* refactor: diag abc (`#229 <https://github.com/tier4/driving_log_replayer/issues/229>`_)
* refactor: localization abstract base class (`#228 <https://github.com/tier4/driving_log_replayer/issues/228>`_)
* Contributors: Hayato Mizushima

1.5.4 (2023-09-01)
------------------
* chore: update mkdocs i18n setting
* docs: fix eagleye downlaod link
* Contributors: Hayato Mizushima, kminoda

1.5.3 (2023-08-31)
------------------
* chore: update pyproject.toml
* docs: add eagleye tutorial
* Contributors: Hayato Mizushima, kminoda

1.5.2 (2023-08-21)
------------------
* fix: handle 2d evaluation task error (`#218 <https://github.com/tier4/driving_log_replayer/issues/218>`_)
* refactor: apply ruff specific RUF rules (`#217 <https://github.com/tier4/driving_log_replayer/issues/217>`_)
* refactor: apply simplify SIM rules (`#216 <https://github.com/tier4/driving_log_replayer/issues/216>`_)
* refactor: apply pyupgrade (`#215 <https://github.com/tier4/driving_log_replayer/issues/215>`_)
* refactor: apply type checking TCH rules (`#214 <https://github.com/tier4/driving_log_replayer/issues/214>`_)
* refactor: apply private-member-access slf rules
* refactor: apply return ret rules (`#213 <https://github.com/tier4/driving_log_replayer/issues/213>`_)
* refactor: apply errmsg em rules (`#211 <https://github.com/tier4/driving_log_replayer/issues/211>`_)
* Contributors: Hayato Mizushima

1.5.1 (2023-08-17)
------------------
* feat: use ruff linter (`#208 <https://github.com/tier4/driving_log_replayer/issues/208>`_)
* fix: store fp result in result.jsonl (`#206 <https://github.com/tier4/driving_log_replayer/issues/206>`_)
* Contributors: Hayato Mizushima

1.5.0 (2023-08-07)
------------------
* feat: update sample scenario and set None if dict key is not found (`#204 <https://github.com/tier4/driving_log_replayer/issues/204>`_)
* feat: add eagleye evaluation (`#203 <https://github.com/tier4/driving_log_replayer/issues/203>`_)
  Co-authored-by: Hayato Mizushima <hayato-m126@users.noreply.github.com>
* feat: support multiple object shapes (`#198 <https://github.com/tier4/driving_log_replayer/issues/198>`_)
  Co-authored-by: ktro2828 <kotaro.uetake@tier4.jp>
  Co-authored-by: ktro2828 <ktro2828@users.noreply.github.com>
  Co-authored-by: Kotaro Uetake <60615504+ktro2828@users.noreply.github.com>
* feat: `#199 <https://github.com/tier4/driving_log_replayer/issues/199>`_ perception fp validation (`#200 <https://github.com/tier4/driving_log_replayer/issues/200>`_)
* Contributors: Hayato Mizushima, kminoda

1.4.1 (2023-08-01)
------------------
* feat: add yabloc scenario (`#201 <https://github.com/tier4/driving_log_replayer/issues/201>`_)
  Co-authored-by: Hayato Mizushima <hayato-m126@users.noreply.github.com>
* Contributors: kminoda

1.4.0 (2023-07-06)
------------------
* feat: map height fitter for diag (`#179 <https://github.com/tier4/driving_log_replayer/issues/179>`_)
* feat: `#175 <https://github.com/tier4/driving_log_replayer/issues/175>`_ map height fitter (`#176 <https://github.com/tier4/driving_log_replayer/issues/176>`_)
  closes: `#175 <https://github.com/tier4/driving_log_replayer/issues/175>`_
* Contributors: Hayato Mizushima

1.3.17 (2023-07-06)
-------------------
* feat: `#192 <https://github.com/tier4/driving_log_replayer/issues/192>`_ delete converged condition to start evaluation (`#193 <https://github.com/tier4/driving_log_replayer/issues/193>`_)
* Contributors: Hayato Mizushima

1.3.16 (2023-07-05)
-------------------
* fix(localization): update NDT availability monitoring topic (`#187 <https://github.com/tier4/driving_log_replayer/issues/187>`_)
  Co-authored-by: kminoda <kminoda@users.noreply.github.com>
  Co-authored-by: Hayato Mizushima <hayato-m126@users.noreply.github.com>
* feat: localization likelihood average std_dev (`#184 <https://github.com/tier4/driving_log_replayer/issues/184>`_)
  Co-authored-by: kminoda <44218668+kminoda@users.noreply.github.com>
* fix: colcon build error (`#185 <https://github.com/tier4/driving_log_replayer/issues/185>`_)
* Contributors: Hayato Mizushima, kminoda

1.3.15 (2023-07-04)
-------------------
* ci: add tier4 cspell-dicts
* Contributors: Hayato Mizushima

1.3.14 (2023-07-03)
-------------------
* feat(localization): add component_state_monitor in localization scenario (`#178 <https://github.com/tier4/driving_log_replayer/issues/178>`_)
  Co-authored-by: kminoda <kminoda@users.noreply.github.com>
  Co-authored-by: Hayato Mizushima <hayato-m126@users.noreply.github.com>
* Contributors: kminoda

1.3.13 (2023-06-30)
-------------------
* fix: localization scenario
* Contributors: Hayato Mizushima

1.3.12 (2023-06-23)
-------------------
* docs: t4_dataset conversion tool
* Contributors: Hayato Mizushima

1.3.11 (2023-06-09)
-------------------
* fix: catch transform exception (`#169 <https://github.com/tier4/driving_log_replayer/issues/169>`_)
  closes: `#168 <https://github.com/tier4/driving_log_replayer/issues/168>`_
* Contributors: Hayato Mizushima

1.3.10 (2023-05-31)
-------------------
* fix: link
* Contributors: Makoto Tokunaga

1.3.9 (2023-05-29)
------------------
* feat: apply ShutdownOnce (`#163 <https://github.com/tier4/driving_log_replayer/issues/163>`_)
  closes: `#162 <https://github.com/tier4/driving_log_replayer/issues/162>`_
* Contributors: Hayato Mizushima

1.3.8 (2023-05-29)
------------------
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
