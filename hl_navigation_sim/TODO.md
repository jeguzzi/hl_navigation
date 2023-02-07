- [x] project skelethon with eigen and bullet deps
- [ ] minimal simulation with no behavior but physics and [motor] update  
  - [ ] copy all relevant file from xcode

- spatial queries (for collision and sensing):
 I want an efficient way to query for overlapping bounding boxes
  - AABB (https://github.com/lohedges/aabbcc)
  - R tree libgeos
  - https://github.com/alecjacobson/computer-graphics-bounding-volume-hierarchy
  - https://libspatialindex.org/en/latest/
  - https://github.com/erincatto/box2d

 - [x] collision broadphase with libgeos
 - [ ] sensing broadphase with libgeos
