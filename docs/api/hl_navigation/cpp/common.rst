========
Geometry
========

Two dimensional
===============

.. code-block:: cpp
   
   #include "hl_navigation/common.h"


.. doxygentypedef:: hl_navigation::Radians

.. doxygenenum:: hl_navigation::Frame

.. doxygentypedef:: hl_navigation::Vector2

.. doxygenfunction:: hl_navigation::orientation_of

.. doxygenfunction:: hl_navigation::normalize

.. doxygenfunction:: hl_navigation::unit

.. doxygenfunction:: hl_navigation::rotate

.. doxygenfunction:: hl_navigation::clamp_norm

.. doxygenstruct:: hl_navigation::Pose2
   :members:

.. doxygenstruct:: hl_navigation::Twist2
   :members:


Three dimensional
=================

.. code-block:: cpp
   
   #include "hl_navigation/controller_3d.h"

.. doxygentypedef:: hl_navigation::Vector3

.. doxygenstruct:: hl_navigation::Pose3
   :members:

.. doxygenstruct:: hl_navigation::Twist3
   :members: