==========
Controller
==========

.. autoclass:: hl_navigation.Action.State
   :members: __init__, name, value
   :exclude-members: __new__
   :undoc-members:

   .. autoattribute:: idle

      the action has not started

   .. autoattribute:: running

      the action is running

   .. autoattribute:: failure

      the action failed

   .. autoattribute:: success

      the action succeeded


.. autoclass:: hl_navigation.Action
    :members:
    :exclude-members: State

.. autoclass:: hl_navigation.Controller
    :members:

