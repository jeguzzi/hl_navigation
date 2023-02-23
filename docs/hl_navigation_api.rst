HL Navigation API
=================

Geometry
---------

.. doxygentypedef:: hl_navigation::Radians

.. doxygentypedef:: hl_navigation::Vector2

.. doxygentypedef:: hl_navigation::Vector3

.. doxygenstruct:: hl_navigation::Pose2
   :members:

.. doxygenstruct:: hl_navigation::Twist2
   :members:

Obstacles
---------

.. doxygenstruct:: hl_navigation::Disc
   :members:

.. doxygenstruct:: hl_navigation::LineSegment
   :members:

Kinematic
---------


.. doxygentypedef:: hl_navigation::WheelSpeeds

Abstract
++++++++

.. doxygenclass:: hl_navigation::Kinematic
   :members:

.. doxygenclass:: hl_navigation::Wheeled
   :members:

Holonomic
+++++++++

.. doxygenclass:: hl_navigation::Holonomic
   :members:

Forward
+++++++

.. doxygenclass:: hl_navigation::Forward
   :members:

Two-Wheeled
+++++++++++

.. doxygenclass:: hl_navigation::TwoWheeled
   :members:

Four-Wheeled
++++++++++++

.. doxygenclass:: hl_navigation::FourWheeled
   :members:


Behaviors
---------

Generic
+++++++

.. doxygenclass:: hl_navigation::Behavior
   :members:

Dummy
+++++

A behavior that ignores collisions


.. doxygenclass:: hl_navigation::DummyBehavior
    :members:

Human-Like
++++++++++

.. doxygenclass:: hl_navigation::HLBehavior
    :members:

ORCA
++++

.. doxygenclass:: hl_navigation::ORCABehavior
    :members:

HRVO
++++
.. doxygenclass:: hl_navigation::HRVOBehavior
    :members:

Controller
----------

.. doxygenstruct:: hl_navigation::Action
    :members:

.. doxygenclass:: hl_navigation::Controller
    :members:


.. doxygenclass:: hl_navigation::Controller3
    :members:
