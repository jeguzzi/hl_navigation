=========
Sampling
=========

Samplers
========

.. code-block:: cpp
   
   #include "hl_navigation/sim/sampling/sampler.h"

.. doxygenfunction:: hl_navigation::sim::set_random_seed

.. doxygenfunction:: hl_navigation::sim::get_random_seed

.. doxygenfunction:: hl_navigation::sim::random_generator

Base class
----------

.. doxygenstruct:: hl_navigation::sim::Sampler
    :members:

.. doxygenenum:: hl_navigation::sim::Wrap


Generic
-------

.. doxygenstruct:: hl_navigation::sim::ConstantSampler
    :members:

.. doxygenstruct:: hl_navigation::sim::SequenceSampler
    :members:

.. doxygenstruct:: hl_navigation::sim::ChoiceSampler
    :members:

Numbers and Vectors
-------------------

.. doxygenstruct:: hl_navigation::sim::RegularSampler
    :members:

Vectors
-------

.. doxygenstruct:: hl_navigation::sim::GridSampler
    :members:

Numbers
-------

.. doxygenstruct:: hl_navigation::sim::UniformSampler
    :members:

.. doxygenstruct:: hl_navigation::sim::NormalSampler
    :members:

Properties
----------

.. doxygenstruct:: hl_navigation::sim::PropertySampler
    :members:

Registered components
=====================

Base class
----------

.. doxygenstruct:: hl_navigation::sim::SamplerFromRegister
    :members:

Registers
---------

.. doxygenstruct:: hl_navigation::sim::BehaviorSampler
    :members:
    :undoc-members:

.. doxygenstruct:: hl_navigation::sim::KinematicsSampler
    :members:
    :undoc-members:

.. doxygentypedef:: hl_navigation::sim::TaskSampler

.. doxygentypedef:: hl_navigation::sim::StateEstimationSampler

Agents
======

.. doxygenstruct:: hl_navigation::sim::AgentSampler
    :members:
    :undoc-members:
