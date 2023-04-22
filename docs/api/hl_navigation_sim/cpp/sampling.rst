=========
Sampling
=========

Samplers
========

.. code-block:: cpp
   
   #include "hl_navigation_sim/sampling/sampler.h"

.. doxygenfunction:: hl_navigation_sim::set_seed


Base class
----------

.. doxygenstruct:: hl_navigation_sim::Sampler
    :members:

.. doxygenenum:: hl_navigation_sim::Wrap


Generic
-------

.. doxygenstruct:: hl_navigation_sim::ConstantSampler
    :members:

.. doxygenstruct:: hl_navigation_sim::SequenceSampler
    :members:

.. doxygenstruct:: hl_navigation_sim::ChoiceSampler
    :members:

Numbers and Vectors
-------------------

.. doxygenstruct:: hl_navigation_sim::RegularSampler
    :members:

Vectors
-------

.. doxygenstruct:: hl_navigation_sim::GridSampler
    :members:

Numbers
-------

.. doxygenstruct:: hl_navigation_sim::UniformSampler
    :members:

.. doxygenstruct:: hl_navigation_sim::NormalSampler
    :members:

Properties
----------

.. doxygenstruct:: hl_navigation_sim::PropertySampler
    :members:

Registered components
=====================

Base class
----------

.. doxygenstruct:: hl_navigation_sim::SamplerFromRegister
    :members:

Registers
---------

.. doxygenstruct:: hl_navigation_sim::BehaviorSampler
    :members:
    :undoc-members:

.. doxygenstruct:: hl_navigation_sim::KinematicsSampler
    :members:
    :undoc-members:

.. doxygentypedef:: hl_navigation_sim::TaskSampler

.. doxygentypedef:: hl_navigation_sim::StateEstimationSampler

Agents
======

.. doxygenstruct:: hl_navigation_sim::AgentSampler
    :members:
    :undoc-members:
