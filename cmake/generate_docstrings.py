import os
import re
import subprocess
import sys


def ref(_renamed_classes, _methods):
    def f(matchobj):
        namespace, name = matchobj.groups()
        owner = ''
        if namespace:
            if namespace[0].isupper():
                namespace = namespace.split("::")[-2]
                owner = _renamed_classes.get(namespace, namespace)
            else:
                print(f"Unexpected namespace {namespace}")
        if owner:
            owner += "."

        if name[0].isupper():
            # it's a class
            name = _renamed_classes.get(name, name)
            return f":py:class:`{name}`"
        if name in _methods:
            return f":py:meth:`{owner}{name}`"
        if 'set_' in name or 'get_' in name:
            name = name[4:]
        return f":py:attr:`{owner}{name}`"

    return f


def main():

    include = sys.argv[3]
    python_src = sys.argv[1]
    library = sys.argv[2]
    deps = sys.argv[4:]

    headers = []

    for path, subdirs, files in os.walk(os.path.join(include, library)):
        for name in files:
            if name.endswith(".h"):
                headers.append(os.path.join(path, name))

    includes = sum([["-I", path] for path in [include] + deps], [])

    args = ["python3", "-m", "pybind11_mkdoc"
            ] + headers + includes + ["--std=c++17"]

    result = subprocess.run(args, capture_output=True)
    header = result.stdout.decode("utf-8")
    destination = 'docstrings.h'

    if result.returncode or len(header) == 0:
        print("Failed running pybind11_mkdoc", file=sys.stderr)
        with open(destination, 'w') as f:
            f.write("#define DOC(...) ")
        return

    header = result.stdout.decode("utf-8")

    with open(python_src, 'r') as f:
        python = f.read()

    _renamed_classes = {
        "NativeAgent": "Agent",
        "NativeWorld": "World",
        "Heading": "Behavior.Heading",
        "Field": "PropertyField"
    }

    _methods = re.findall(r".def\(\"(\w+)\",", python)
    _properties = re.findall(r".def_property\(\"(\w+)\",", python)

    header = re.sub(r"\\ref\s+(\w*::)*(\w+)", ref(_renamed_classes, _methods), header)
    extras = ''
    exposed_properties = set()

    for owner, name, doc in re.findall(r"static const char \*__doc_(\w+)_get_(\w+) =\s*R\"doc\(([\s\S\w\W.]*?)(?=:return:|\)doc)", header):
        p = f'{owner}_property_{name}'
        if p in exposed_properties:
            continue
        exposed_properties.add(p)
        extras += "\n"
        if doc.startswith("Gets "):
            doc = doc[5:]
            doc = doc.capitalize()
        doc = doc[:-1]
        extras += f'static const char *__doc_{owner}_property_{name} =\nR"doc({doc})doc";\n'

    for ns, doc in re.findall(r"static const char \*__doc_(\w+) =\s*R\"doc\(([\s\S\w\W.]*?)(?=:return:|\)doc)", header):
        ss = ns.split('_')
        if len(ss) < 5:
            continue
        if ss[3] == "get" or ss[3] == "set":
            continue
        owner = '_'.join(ss[:4])
        name = '_'.join(ss[4:])
        if name not in _properties:
            continue
        p = f'{owner}_property_{name}'
        if p in exposed_properties:
            continue
        exposed_properties.add(p)
        extras += "\n"
        if doc.startswith("Returns "):
            doc = doc[8:]
            doc = doc.capitalize()
        doc = doc[:-1]
        extras += f'static const char *__doc_{owner}_property_{name} =\nR"doc({doc})doc";\n'

    header += extras

    with open(destination, 'w') as f:
        f.write(header)


if __name__ == '__main__':
    main()
