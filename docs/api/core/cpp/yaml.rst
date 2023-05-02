.. _core cpp yaml:

====
YAML
====

We use `yaml-cpp <https://github.com/jbeder/yaml-cpp>`_ to provide import/export 
from/to YAML. The code in ``hl_navigation/yaml`` specializes templates
of :py:class:`struct YAML::convert<T>`, which is how yaml-cpp is extended to custom classes.

You can therefore use yaml-cpp APIs, which allows to

- parse a YAML string:

  .. code-block:: c++

     #include "yaml-cpp/yaml.h"

     YAML::Node node = YAML::Load("...");

- convert a :cpp:class:`YAML::Node` to a C++ object:

  .. code-block:: c++

     T object =  node.as<T>();

- convert a C++ object to a :cpp:class:`YAML::Node`

  .. code-block:: c++

     T object;
     YAML::Node node(object);

- emit YAML from a :cpp:class:`YAML::Node`:

  .. code-block:: c++

     #include "yaml-cpp/yaml.h"

     YAML::Node node;
     YAML::Emitter out;
     out << node;
     const char * yaml = out.c_str();

In particular, we specialize the conversion from/to the following classes:

- :cpp:type:`hl_navigation::core::Vector2`
- :cpp:class:`hl_navigation::core::LineSegment`
- :cpp:class:`hl_navigation::core::Disc`
- :cpp:class:`hl_navigation::core::Neighbor`
- :cpp:enum:`hl_navigation::core::Behavior::Heading`
- :cpp:class:`hl_navigation::core::Behavior`
- :cpp:class:`hl_navigation::core::Kinematics`
- :cpp:class:`hl_navigation::core::SocialMargin`

For classes ``T`` with a register, the conversion from and to ``std::shared_ptr<T>``
write/reads the type name and all the registered properties. 
For example, when you convert from a YAML::node corresponding to

.. code-block:: YAML

   type: MySubClass
   my_property: false

to a ``T`` object, :cpp:func:`hl_navigation::core::HasRegister::make_type` is called. If successful, 
it then reads all properties from the YAML fields, calling  
:cpp:func:`hl_navigation::core::HasProperties::set` for any ``MySubClass`` 
registered property.  

.. code-block:: c++

   auto obj = node.as<std::shared_ptr<T>>();
   // will call:
   // - auto obj = make_type("MySubClass");
   // - obj.set("my_property", false);
   // in addition to setup the common fields of ``T``. 


Public API
----------

.. code-block:: cpp
   
   #include "hl_navigation/core/yaml/yaml.h"

.. cpp:namespace:: YAML

.. doxygennamespace:: YAML
   :desc-only:

.. doxygenfunction:: load_node(const Node &)

.. doxygenfunction:: load_string(const std::string &)

.. doxygenfunction:: dump(const T *object)



