# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

import subprocess
import re

project = 'hl_navigation'
copyright = '2023, Jérôme Guzzi, IDSIA'
author = 'Jérôme Guzzi, IDSIA'
release = '0.0.1'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.todo',
    'breathe',
    'sphinx_rtd_theme',
]

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

language = 'en'

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

# html_theme = 'alabaster'
html_theme = "sphinx_rtd_theme"
html_theme = 'sphinx_book_theme'
html_static_path = ['_static']

# -- Options for todo extension ----------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/extensions/todo.html#configuration

todo_include_todos = True


# subprocess.call('make clean', shell=True)
# subprocess.call('cd ../../doxygen ; doxygen', shell=True)

breathe_projects = {"hl_navigation": "_build/doxygen/xml"}
breathe_default_project = "hl_navigation"
breathe_show_include = False
breathe_show_enumvalue_initializer = True

add_module_names = False
autodoc_typehints_format = 'short'
autodoc_member_order = 'groupwise'
autodoc_class_signature = 'separated'
autodoc_inherit_docstrings = True
autoclass_content = 'class'
autodoc_docstring_signature = True
autodoc_type_aliases = {
    'PropertyField': 'PropertyField'
}

_replace = {
    "_hl_navigation.": "",
    "_hl_navigation_sim.": "",
    "numpy.ndarray[numpy.float32[2, 1]]": "Vector2",
    "Union[bool, int, float, str, Vector2, List[bool], List[int], List[float], List[str], List[Vector2]]": "PropertyField",
}

def f(app, what, name, obj, options, lines):
    if what == "property":
        if not hasattr(obj.fget, '__annotations__'):
            docs = obj.fget.__doc__
            if docs:
                docs = docs.splitlines()[0]
                rtype = docs.split(' -> ')[-1]
                lines.insert(0, '')
                lines.insert(0, f':type: {rtype}')
    for i, _ in enumerate(lines):
        if 'self' in lines[i]:
            lines[i] = re.sub(r"self: (\w+\.?)+,?\s*", "", lines[i])
        for k, v in _replace.items():
            if k in lines[i]:
                lines[i] = lines[i].replace(k, v)
        if ':py:class:`Vector2`' in lines[i]:
            lines[i] = lines[i].replace(':py:class:`Vector2`', ':py:class:`Vector2 <hl_navigation.Vector2>`')

def g(app, what, name, obj, options, signature, return_annotation):
    # print('g', what, name, signature, return_annotation)
    # return (signature, return_annotation)
    if signature:
        signature = re.sub(r"self: (\w+\.?)+,?\s*", "", signature)
        for k, v in _replace.items():
            if k in signature:
                signature = signature.replace(k, v)
        # print(signature)
    if return_annotation:
        for k, v in _replace.items():
            if k in return_annotation:
                return_annotation = return_annotation.replace(k, v)
    # if (what == 'property' and return_annotation is None):
        # print('set sig of ', name)
        # return_annotation = '-> int'
        # signature = '()'
    return (signature, return_annotation)

def h(app, obj, bound_method):
    print('h', obj)

def l(app, domain, objtype, contentnode):
    if objtype == 'property':
        print(app, domain, objtype, contentnode)

reftarget_aliases = {}
reftarget_aliases['py'] = {
    # 'hl_navigation.core._hl_navigation.Behavior': 'hl_navigation.core.Behavior',
    # 'hl_navigation.core._hl_navigation.Kinematics': 'hl_navigation.core.Kinematics',
    # 'PropertyField': 'hl_navigation.core.PropertyField',
    # 'hl_navigation.sim.NativeAgent': 'hl_navigation.sim.Agent',
    # 'Behavior': 'hl_navigation.core.Behavior',
    # 'Kinematics': 'hl_navigation.core.Kinematics',
    # 'Neighbor.id': 'hl_navigation.core.Neighbor.id',
    # 'hl_navigation.core.Property': 'hl_navigation.core._hl_navigation.Property',
    # 'Controller.go_to_position': 'hl_navigation.core.Controller.go_to_position',
    # 'Agent': 'hl_navigation::sim::Agent',
    # 'Experiment': 'hl_navigation::sim::Experiment',
    # 'World': 'hl_navigation::sim::World',
    # 'hl_navigation.registered_property': 'hl_navigation.core.registered_property',
    # 'Vector2': 'hl_navigation.Vector2',
    'hl_navigation.core._hl_navigation.Behavior': 'hl_navigation.core.Behavior',
    'hl_navigation.core._hl_navigation.Kinematics': 'hl_navigation.core.Kinematics',
    'Behavior': 'hl_navigation.core.Behavior',
    'Neighbor.id': 'hl_navigation.core.Neighbor.id',
    'hl_navigation.sim.NativeAgent': 'hl_navigation.sim.Agent',
    'hl_navigation.sim.NativeWorld': 'hl_navigation.sim.World',
    'hl_navigation.core.Property': 'hl_navigation.core._hl_navigation.Property',
    'Controller.go_to_position': 'hl_navigation.core.Controller.go_to_position',
    'hl_navigation.registered_property': 'hl_navigation.core.registered_property',
    'Vector2': 'hl_navigation.Vector2',
    'hl_navigation::sim::Scenario': 'hl_navigation.sim.Scenario',
    'hl_navigation.core.SocialMarginModulation': 'hl_navigation.core.SocialMargin.Modulation',
    'hl_navigation.Kinematics': 'hl_navigation.core.Kinematics',
    'Kinematics': 'hl_navigation.core.Kinematics',
    'hl_navigation.Behavior': 'hl_navigation.core.Behavior',
    'PropertyField': 'hl_navigation.core.PropertyField',
    'hl_navigation::core::EnvironmentState': 'hl_navigation.core.EnvironmentState',
    'Frame.absolute': 'hl_navigation.core.Frame.absolute',
    'Frame.relative': 'hl_navigation.core.Frame.relative',
    'behavior.actuated_twist': 'hl_navigation.core.Behavior.actuated_twist',
    'behavior': 'hl_navigation.core.Behavior',
    'GeometricState': 'hl_navigation.core.GeometricState',
    # 'Disc': 'hl_navigation.core.Disc',
    # 'LineSegment': 'hl_navigation.core.LineSegment',
    # 'Neighbor': 'hl_navigation.core.Neighbor',
    #
}

reftarget_aliases['cpp'] = {
    'Node': 'YAML::Node',
    'Behavior': 'hl_navigation::core::Behavior',
    'Neighbor': 'hl_navigation::core::Neighbor',
    'Kinematics': 'hl_navigation::core::Kinematics',
    'Controller': 'hl_navigation::core::Controller',
    'Pose2': 'hl_navigation::core::Pose2',
    'Twist2': 'hl_navigation::core::Twist2',
    'Vector2': 'hl_navigation::core::Vector2',
    'Disc': 'hl_navigation::core::Disc',
    'LineSegment': 'hl_navigation::core::LineSegment',
}

from docutils.nodes import Text, reference
from sphinx.ext.intersphinx import missing_reference
from sphinx.addnodes import pending_xref

def resolve_internal_aliases(app, doctree):
    pending_xrefs = doctree.traverse(condition=pending_xref)
    for node in pending_xrefs:
        alias = node.get('reftarget', None)
        d = node.get('refdomain', '')
        rs = reftarget_aliases.get(d, {})
        if alias is not None and alias in rs:
            node['reftarget'] = rs[alias]


def setup(app):
    app.connect('autodoc-process-docstring', f);
    app.connect('autodoc-process-signature', g);
    app.connect('doctree-read', resolve_internal_aliases)
    # app.connect('missing-reference', ref)
    # app.connect('object-description-transform', l)
    # app.connect('autodoc-before-process-signature', h)
    #


