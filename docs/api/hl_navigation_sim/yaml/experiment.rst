==========
Experiment
==========

Schema
^^^^^^

.. code-block:: yaml

   $schema: https://json-schema.org/draft/2020-12/schema
   $id: /schemas/experiment
   title: Experiment
   type: object
   properties:
     ...
   required: [type]

Example
^^^^^^^

.. code-block:: yaml

   modulation:
      type: linear
      upper: 1.0
   values:
      1: 0.5
      2: 0.25
   default: 0.125

