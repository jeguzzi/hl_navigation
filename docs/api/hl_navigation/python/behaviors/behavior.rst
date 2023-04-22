==========
Base class
==========

.. autoclass:: hl_navigation.Behavior.Mode
   :members: __init__, name, value
   :exclude-members: __new__
   :undoc-members:

   .. autoattribute:: move

      move towards the target position

   .. autoattribute:: follow

      follow the target velocity

   .. autoattribute:: turn

      turn-in-place towards the target orientation

   .. autoattribute:: stop

      stop
   
.. autoclass:: hl_navigation.Behavior.Heading
   :members: __init__, name, value
   :exclude-members: __new__
   :undoc-members:

   .. autoattribute:: idle

      never turn

   .. autoattribute:: target_point

      turn towards the target position

   .. autoattribute:: target_angle

      turn towards the target orientation

   .. autoattribute:: target_angular_speed

      follow a target angular speed

   .. autoattribute:: velocity

      turn towards the velocity orientation. This is the only behavior available to constrained kinematics.


.. autoclass:: hl_navigation.Behavior
   :members:
   :inherited-members:
   :exclude-members: __new__, Heading, Mode
