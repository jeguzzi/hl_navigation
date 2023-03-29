# install_linux

Installing on docker

## Problems

1) I have to build geos as the installed libgeos seems to have a problem 

```
                 from /ws/src/hl_navigation/hl_navigation_examples/src/world/world.cpp:5:
/usr/include/geos/geom/Coordinate.h:164:11: fatal error: geos/geom/Coordinate.inl: No such file or directory
  164 | # include "geos/geom/Coordinate.inl"
```

But then the target is not correcly exported:

```
CMake Error in CMakeLists.txt:
  IMPORTED_LOCATION not set for imported target "GEOS::geos".
```

and hl_navigation_{examples|demos} fail to build


 - the alternative may be to build geos with colcon ...
 	- same problem ... it seems a problem of libgeos

 	```
CMake Error in CMakeLists.txt:
  IMPORTED_LOCATION not set for imported target "GEOS::geos" configuration
  "RelWithDebInfo".
```
 - it works if I first build libgeos, then I source, then I build hl_navigation_sim (and change to find_package(geos -> GEOS))


2) crash degli exec python in demos/
```
Done simulating in 33.7 ms
root@fa44dc672129:/ws# ./install/hl_navigation_demos/lib/hl_navigation_demos/native_py
Traceback (most recent call last):
  File "/ws/./install/hl_navigation_demos/lib/hl_navigation_demos/native_py", line 33, in <module>
    sys.exit(load_entry_point('hl-navigation-demos==0.0.0', 'console_scripts', 'native_py')())
  File "/ws/./install/hl_navigation_demos/lib/hl_navigation_demos/native_py", line 22, in importlib_load_entry_point
    for entry_point in distribution(dist_name).entry_points
  File "/usr/lib/python3.10/importlib/metadata/__init__.py", line 957, in distribution
    return Distribution.from_name(distribution_name)
  File "/usr/lib/python3.10/importlib/metadata/__init__.py", line 548, in from_name
    raise PackageNotFoundError(name)
importlib.metadata.PackageNotFoundError: No package metadata was found for hl-navigation-demos
```

	- si e' risolto (non avevo sourcato il repo)