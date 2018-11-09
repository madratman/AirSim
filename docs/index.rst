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


Getting started
-------------------------------------
Windows
~~~~~~~

-  `Binaries <use_precompiled.html>`_
-  `Build from source <build_windows.html>`_

Linux
~~~~~

-  `Build from source <docs/build_linux.html>`_

How to Use It
-------------
Choosing the Mode: Car, Multirotor or ComputerVision
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

By default AirSim will prompt you to choose Car or Multirotor mode. You can use `SimMode setting <docs/settings.html#simmode>`_ to specify the default vehicle or the new `ComputerVision mode <docs/image_apis.html#computer-vision-mode-1>`_.

Manual drive
^^^^^^^^^^^^

If you have remote control (RC) as shown below, you can manually control the drone in the simulator. For cars, you can use arrow keys to drive manually.

`More details <docs/remote_control.html>`_


.. image:: docs/images/AirSimDroneManual.gif
   :target: docs/images/AirSimDroneManual.gif
   :alt: record screenshot



.. image:: docs/images/AirSimCarManual.gif
   :target: docs/images/AirSimCarManual.gif
   :alt: record screenshot


Programmatic control
^^^^^^^^^^^^^^^^^^^^

AirSim exposes APIs so you can interact with the vehicle in the simulation programmatically. You can use these APIs to retrieve images, get state, control the vehicle and so on. The APIs are exposed through the RPC, and are accessible via a variety of languages, including C++, Python, C# and Java.

These APIs are also available as part of a separate, independent cross-platform library, so you can deploy them on a companion computer on your vehicle. This way you can write and test your code in the simulator, and later execute it on the real vehicles. Transfer learning and related research is one of our focus areas.

`More details <docs/apis.html>`_

Gathering training data
^^^^^^^^^^^^^^^^^^^^^^^

There are two ways you can generate training data from AirSim for deep learning. The easiest way is to simply press the record button in the lower right corner. This will start writing pose and images for each frame. The data logging code is pretty simple and you can modify it to your heart's content.


.. image:: docs/images/record_data.png
   :target: docs/images/record_data.png
   :alt: record screenshot


A better way to generate training data exactly the way you want is by accessing the APIs. This allows you to be in full control of how, what, where and when you want to log data. 

Computer Vision mode
^^^^^^^^^^^^^^^^^^^^

Yet another way to use AirSim is the so-called "Computer Vision" mode. In this mode, you don't have vehicles or physics. You can use the keyboard to move around the scene, or use APIs to position available cameras in any arbitrary pose, and collect images such as depth, disparity, surface normals or object segmentation. 

`More details <docs/image_apis.html>`_

Tutorials
---------


* `Video - Setting up AirSim with Pixhawk Tutorial <https://youtu.be/1oY8Qu5maQQ>`_ by Chris Lovett
* `Video - Using AirSim with Pixhawk Tutorial <https://youtu.be/HNWdYrtw3f0>`_ by Chris Lovett
* `Video - Using off-the-self environments with AirSim <https://www.youtube.com/watch?v=y09VbdQWvQY>`_ by Jim Piavis
* `Reinforcement Learning with AirSim <docs/reinforcement_learning.html>`_ by Ashish Kapoor
* `The Autonomous Driving Cookbook <https://aka.ms/AutonomousDrivingCookbook>`_ by Microsoft Deep Learning and Robotics Garage Chapter
* `Using TensorFlow for simple collision avoidance <https://github.com/simondlevy/AirSimTensorFlow>`_ by Simon Levy and WLU team

Participate
-----------

Paper
^^^^^

More technical details are available in `AirSim paper (FSR 2017 Conference) <https://arxiv.org/abs/1705.05065>`_. Please cite this as:

.. code-block::

   @inproceedings{airsim2017fsr,
     author = {Shital Shah and Debadeepta Dey and Chris Lovett and Ashish Kapoor},
     title = {AirSim: High-Fidelity Visual and Physical Simulation for Autonomous Vehicles},
     year = {2017},
     booktitle = {Field and Service Robotics},
     eprint = {arXiv:1705.05065},
     url = {https://arxiv.org/abs/1705.05065}
   }

Contribute
^^^^^^^^^^

Please take a look at `open issues <https://github.com/microsoft/airsim/issues>`_ if you are looking for areas to contribute to.


* `More on AirSim design <docs/design.html>`_
* `More on code structure <docs/code_structure.html>`_
* `Contribution Guidelines <docs/contributing.html>`_

Who is Using AirSim?
^^^^^^^^^^^^^^^^^^^^

We are maintaining a `list <docs/who_is_using.html>`_ of a few projects, people and groups that we are aware of. If you would like to be featured in this list please `make a request here <https://github.com/microsoft/airsim/issues>`_.

Contact
-------

Join the AirSim group on `Facebook <https://www.facebook.com/groups/1225832467530667/>`_ to stay up to date or ask any questions.

FAQ
---

If you run into problems, check the `FAQ <docs/faq.html>`_ and feel free to post issues in the  `AirSim <https://github.com/Microsoft/AirSim/issues>`_ repository.

License
-------

This project is released under the MIT License. Please review the `License file <LICENSE>`_ for more details.

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
