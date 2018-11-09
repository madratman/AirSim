Welcome to AirSim
=================

AirSim is a simulator for drones, cars and more, built on `Unreal
Engine <https://www.unrealengine.com/>`_. It is open-source, cross platform and supports
hardware-in-loop with popular flight controllers such as PX4 for
physically and visually realistic simulations. It is developed as an
Unreal plugin that can simply be dropped into any Unreal environment you
want.

Our goal is to develop AirSim as a platform for AI research to
experiment with deep learning, computer vision and reinforcement
learning algorithms for autonomous vehicles. For this purpose, AirSim
also exposes APIs to retrieve data and control vehicles in a platform
independent way.

**AirSim Drone Demo Video**    

.. raw:: html

    <div style="position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden; max-width: 100%; height: auto;">
        <iframe src="//www.youtube.com/embed/-WfTr1-OBGQ" frameborder="0" allowfullscreen style="position: absolute; top: 0; left: 0; width: 100%; height: 100%;"></iframe>
    </div>

**AirSim Car Demo Video**

.. raw:: html

    <div style="position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden; max-width: 100%; height: auto;">
        <iframe src="//www.youtube.com/embed/gnz1X3UNM5Y" frameborder="0" allowfullscreen style="position: absolute; top: 0; left: 0; width: 100%; height: 100%;"></iframe>
    </div>


Installing
-------------------------------------

Windows
~~~~~~~

-  `Binaries <use_precompiled.html>`_
-  `Build from source <build_windows.html>`_

Linux
~~~~~

-  `Build from source <build_linux.html>`_


Getting started
-------------

Choosing the Mode: Car, Multirotor or ComputerVision
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

By default AirSim will prompt you to choose Car or Multirotor mode. You
can use `SimMode setting`_ to specify the default vehicle or the new
`ComputerVision mode`_.

Manual drive
~~~~~~~~~~~~

If you have remote control (RC) as shown below, you can manually control
the drone in the simulator. For cars, you can use arrow keys to drive
manually.

`More details`_

.. figure:: images/AirSimDroneManual.gif
   :alt: record screenshot

   record screenshot

.. figure:: images/AirSimCarManual.gif
   :alt: record screenshot

   record screenshot

Programmatic control
~~~~~~~~~~~~~~~~~~~~

AirSim exposes APIs so you can interact with the vehicle in the
simulation programmatically. You can use these APIs to retrieve images,
get state, control the

.. _Unreal Engine: https://www.unrealengine.com/
.. _New environments: https://github.com/Microsoft/AirSim/releases/tag/v1.2.1
.. _NoDisplay view mode: https://github.com/Microsoft/AirSim/blob/master/settings.html#viewmode
.. _Lidar Sensor: lidar.html
.. _Formula Student Technion Driverless: https://github.com/Microsoft/AirSim/wiki/technion
.. _Multi-Vehicle Capability: multi_vehicle.html
.. _ROS publisher: https://github.com/Microsoft/AirSim/pull/1135
.. _Arducopter Solo Support: https://github.com/Microsoft/AirSim/pull/1387
.. _API Upgrade: upgrade_apis.html
.. _Settings Upgrade: upgrade_settings.html
.. _upgrade instructions: unreal_upgrade.html
.. _Changelog: CHANGELOG.html
.. _Download binaries: use_precompiled
.. _Build it: build_windows.html
.. _SimMode setting: settings.html#simmode
.. _ComputerVision mode: image_apis.html#computer-vision-mode-1
.. _More details: remote_control.html

.. |AirSim Drone Demo Video| image:: images/demo_video.png
   :target: https://youtu.be/-WfTr1-OBGQ
.. |AirSim Car Demo Video| image:: images/car_demo_video.png
   :target: https://youtu.be/gnz1X3UNM5Y
- Reference 
 - API
 - Settings
- FAQ

.. toctree::
   :maxdepth: 1

   apis_cpp
   apis
   camera_views
   cmake_linux
   code_structure
   coding_guidelines
   contributing
   create_issue
   custom_drone
   design
   dev_workflow
   faq
   flight_controller
   hard_drive
   hello_drone
   image_apis
   lidar
   log_viewer
   multi_vehicle
   pfm
   playback
   px4_build
   px4_logging
   px4_setup
   px4_sitl
   python
   reinforcement_learning
   release_notes
   remote_control
   ros
   sensors
   settings
   simple_flight
   steering_wheel_installation
   unreal_blocks
   unreal_custenv
   unreal_proj
   unreal_upgrade
   upgrade_apis
   upgrade_settings
   using_car
   whats_new
   who_is_using
   working_with_plugin_contents
