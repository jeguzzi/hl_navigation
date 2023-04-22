===========================
How to register a component
===========================

Source code
===========

C++
---

Registering a C++ sub-class ``MyClass`` in the parent class ``T``  register, requires to

#. Optionally, in case ``MyClass`` should exposed properties, register all properties.

   #. add getters and setters for all properties, e.g.:

      .. code-block:: c++

         bool get_my_property() const { return _my_field; }
         void set_my_property(bool value) { _my_field = value; }

   #. expose the properties, overriding at the same time 
      :cpp:func:`hl_navigation::HasProperties::get_properties`

      .. code-block:: c++

         const Properties &get_properties() const override { return properties; };
         inline const static std::map<std::string, Property> properties = Properties{
             {"my_property", make_property<bool, MyClass>(
                                 &MyClass::get_my_property, &MyClass::set_my_property,
                                 False, "my property description")}};


#. override :cpp:func:`hl_navigation::HasRegister::get_type`

   .. code-block:: c++
        
      std::string get_type() const override { return type; }
    

#. call :cpp:func:`hl_navigation::HasRegister::register_type` 

   .. code-block:: c++

      static inline const std::string type = register_type<MyClass>("MyClass");


Complete example
^^^^^^^^^^^^^^^^

.. code-block:: c++
    :linenos:

    class MyClass(T) {
     public:
      MyClass() : T(), _my_field(False) {}
      bool get_my_property() const { return _my_field; }
      void set_my_property(bool value) { _my_field = value; }
      const Properties &get_properties() const override { return properties; };
      inline const static std::map<std::string, Property> properties = Properties{
          {"my_property", make_property<bool, MyClass>(
                              &MyClass::get_my_property, &MyClass::set_my_property,
                              False, "my property description")}};
      std::string get_type() const override { return type; }
      static inline const std::string type = register_type<MyClass>("MyClass");
    
     private:
      bool _my_field;
    };


Python
------

Registering a Python sub-class ``MyClass`` in the parent class ``T``  register, requires to

#. Optionally, in case ``MyClass`` should exposed properties, register all properties using :py:func:`hl_navigation.registered_property`:

   .. code-block:: python
   
      @registered_property(False, "my property description")
      def my_property(self) -> bool:
          return self._my_field;
    
      @my_property.setter
      def my_property(self, value: bool) -> None:
          self._my_field = value;

#. add ``name="..."`` to the class definition

   .. code-block:: python

      class MyClass(T, name="MyClass"):
          ...

Complete example
^^^^^^^^^^^^^^^^

.. code-block:: python
    :linenos:

    from hl_navigation import registered_property
    
    
    class MyClass(T, name="MyClass"):
    
        def __init__(self):
            super().__init__()
            self._my_field = False
    
        @registered_property(False, "my property description")
        def my_property(self) -> bool:
            return self._my_field
       
        @my_property.setter
        def my_property(self, value: bool) -> None:
            self._my_field = value


Installation
============

If the registered component has to be discoverable outside of the code where it is defined, like when
you want to run an experiment using it, you have to install the component so that it can be discovered.

.. warning::

    Currently installing is only supported for Python components. 
    Support for C++ components is in progress. 

Python
------

Define an entry for each component you want to export in the ``setup.cfg`` or ``setup.py`` file of the package.

For example, to install behavior ``MyBehavior``, add 

.. code-block:: toml

   [options.entry_points]
   hl_behaviors = 
       my_behavior = <my_packages>.<my_module>:MyBehavior

to your ``setup.cfg``. The name ``my_behavior`` is currently ignored.

.. note::
    
   Following end-points are available ``hl_behaviors``, ``hl_kinematics``, ``hl_tasks``, 
   ``hl_state_estimations``, and ``hl_scenarios`` to install components of the respective type.



Then, when importing ``hl_navigation``, the behavior will be automatically discovered

.. code-block:: python

   >>> import hl_navigation 

   >>> print(hl_navigation.Behavior.types)

   [..., 'MyBehavior']

   >>> behavior = hl_navigation.Behavior.make_type("MyBehavior")
   <my_packages>.<my_module>.MyBehavior object ...>




