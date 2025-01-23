^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package nav2_regulated_pure_pursuit_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.18 (2025-01-23)
-------------------

1.1.17 (2024-09-20)
-------------------

1.1.16 (2024-06-04)
-------------------

1.1.15 (2024-05-01)
-------------------

1.1.14 (2024-04-24)
-------------------
* Merge pull request `#61 <https://github.com/cmrobotics/navigation2/issues/61>`_ from cmrobotics/fix-rpp-tests
  (SS-737) Fix rpp tests
* refactor: cleaning code for uncrustify
* fix: negative angle_to_path is evaluated with negative velocity
* fix: use static transform
* Contributors: Fulvio Di Luzio, fvlvivs

1.1.13 (2024-04-09)
-------------------

1.1.12 (2024-04-05)
-------------------

1.1.11 (2024-03-08)
-------------------

1.1.10 (2024-02-14)
-------------------
* Merge pull request `#56 <https://github.com/cmrobotics/navigation2/issues/56>`_ from cmrobotics/ss-697-extrapolation-into-past
  Ss 697 extrapolation into past + small fix in rviz visualization
* fix: extended collision check visualization not showing in rviz sometimes
* fix: extrapolation into past error.
  lookup transform with current time, not the timestamp that came with global plan
* Contributors: Tanmay, Tanmay Deshmukh

1.1.9 (2024-02-03)
------------------
* Merge pull request `#54 <https://github.com/cmrobotics/navigation2/issues/54>`_ from cmrobotics/proportional-rotate-to-heading
  SS-713 SS-640 Fix: overshoot of robot heading Proportional rotate to heading
* feat: default and warn about misparameterization of rotate_to_heading_proportional_gain and rotate_to_heading_angular_vel
* feat: rpp remove angaular deceleration limit since we have proportional control,
  reduce coupling with odom based curr_vel as it very noisy and simplify control law
* fix: rotate to heading not smooth because of noisy "cur_speed"
  always prefer velocities closer to zero
* fix: goal yaw tolerence check works in only one direction
* feat: rpp proportional control for rotate to heading
  smoothen rotation to heading and prevent overshoot
* Contributors: Tanmay, Tanmay Deshmukh

1.1.8 (2024-01-16)
------------------

1.1.7 (2024-01-10)
------------------
* Merge pull request `#52 <https://github.com/cmrobotics/navigation2/issues/52>`_ from cmrobotics/humble-pre-release
  Sync humble-pre-release branch
* Merge pull request `#50 <https://github.com/cmrobotics/navigation2/issues/50>`_ from cmrobotics/ppr_costmap
  Local costmap in "odom" frame instead of  "map" frame (SS-415 SS-456 BTB-181)
* Extra extendedcllisioncheck unit tests
* collision  check unit test with a fake tf.
* make local costmap glabal frame be odom instead of map
* Contributors: Arkadiusz Nowakowski, Moumouni BELEM

1.1.6 (2023-12-04)
------------------
* Initial changelog
  Co-authored-by: Aaron Chong <aaronchongth@gmail.com>
  Co-authored-by: Abdullah Enes BEDİR <46785079+enesbedir1@users.noreply.github.com>
  Co-authored-by: Adam Aposhian <adam.aposhian@fireflyautomatix.com>
  Co-authored-by: Adam Aposhian <aposhian.dev@gmail.com>
  Co-authored-by: Afif Swaidan <53655365+afifswaidan@users.noreply.github.com>
  Co-authored-by: Afif Swaidan <afif.swaidan@spexal.com>
  Co-authored-by: Alexey Merzlyakov <60094858+AlexeyMerzlyakov@users.noreply.github.com>
  Co-authored-by: Alexey Merzlyakov <alexey.merzlyakov@samsung.com>
  Co-authored-by: Arshad Mehmood <arshad.mehmood@intel.com>
  Co-authored-by: Austin Greisman <92941098+austin-InDro@users.noreply.github.com>
  Co-authored-by: Borong Yuan <yuanborong@hotmail.com>
  Co-authored-by: Daisuke Sato <43101027+daisukes@users.noreply.github.com>
  Co-authored-by: Erwin Lejeune <erwin.lejeune15@gmail.com>
  Co-authored-by: Hao-Xuan Song <44140526+Cryst4L9527@users.noreply.github.com>
  Co-authored-by: Jackson9 <k9632441@gmail.com>
  Co-authored-by: Joshua Wallace <47819219+jwallace42@users.noreply.github.com>
  Co-authored-by: Lukas Fanta <63977366+fantalukas@users.noreply.github.com>
  Co-authored-by: M. Mostafa Farzan <m2_farzan@yahoo.com>
  Co-authored-by: MartiBolet <43337758+MartiBolet@users.noreply.github.com>
  Co-authored-by: Narahari Rahul Malayanur <60045406+Naraharirahul@users.noreply.github.com>
  Co-authored-by: Nicolas Rocha Pacheco <n.nicolas98@hotmail.com>
  Co-authored-by: Nikolas Engelhard <nikolas.engelhard@gmail.com>
  Co-authored-by: Owen Hooper <17ofh@queensu.ca>
  Co-authored-by: Pedro Alejandro González <71234974+pepisg@users.noreply.github.com>
  Co-authored-by: Pradheep Krishna <padhupradheep@gmail.com>
  Co-authored-by: Ruffin <roxfoxpox@gmail.com>
  Co-authored-by: Samuel Lindgren <samuel@dynorobotics.se>
  Co-authored-by: Shrijit Singh <shrijitsingh99@gmail.com>
  Co-authored-by: Srijanee Biswas <srijanee.biswas@toyotatmh.com>
  Co-authored-by: SrijaneeBiswas <30804865+SrijaneeBiswas@users.noreply.github.com>
  Co-authored-by: Steve Macenski <stevenmacenski@gmail.com>
  Co-authored-by: Stevedan Ogochukwu Omodolor <61468301+stevedanomodolor@users.noreply.github.com>
  Co-authored-by: Steven Brills <sbrills@aethon.com>
  Co-authored-by: Sven Langner <svenlr@users.noreply.github.com>
  Co-authored-by: Tejas Kumar Shastha <tejas.kumar.shastha@ipa.fraunhofer.de>
  Co-authored-by: Tobias Fischer <info@tobiasfischer.info>
  Co-authored-by: Vinny Ruia <vinny.ruia@fireflyautomatix.com>
  Co-authored-by: Zhenpeng Ge <zhenpeng.ge@qq.com>
  Co-authored-by: autome <rohit.bhor111.rb@gmail.com>
  Co-authored-by: hodnajit <jitkahodna67@gmail.com>
  Co-authored-by: jaeminSHIN <91681721+woawo1213@users.noreply.github.com>
  Co-authored-by: kevin <kevin@floatic.io>
  Co-authored-by: nakai-omer <108797279+nakai-omer@users.noreply.github.com>
  Co-authored-by: sathak93 <sathak0730@gmail.com>
  Co-authored-by: seasony-vp <72447461+seasony-vp@users.noreply.github.com>
  Co-authored-by: shoufei <907575489@qq.com>
  Co-authored-by: stevenbrills <90438581+stevenbrills@users.noreply.github.com>
  Co-authored-by: 정찬희 <60467877+ladianchad@users.noreply.github.com>
* Contributors: Adam Aposhian, Afonso da Fonseca Braga, Alexey Merzlyakov, Chris Lalancette, Erwin Lejeune, M. Hofstätter, Nisala Kalupahana, Pradheep Krishna, Sarthak Mittal, Sathakkadhullah, Steve Macenski, Tanmay, Tanmay Deshmukh, dpm-seasony, mergify[bot], relffok, stevemacenski

0.2.0 (2019-06-28)
------------------

0.1.5 (2018-12-12)
------------------

0.1.4 (2018-12-11)
------------------

0.1.3 (2018-12-10)
------------------

0.1.2 (2018-12-06)
------------------

0.1.1 (2018-12-05)
------------------

0.1.0 (2018-10-30)
------------------
