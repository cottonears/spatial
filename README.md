# Spatial
A library for spatial data structures and related functions.

## TODO:
- Add more tests + benchmark for collision detection.
- Make bounding volumes always use boxes in SquareTree.
Balls perform significantly worse because they grow overly large.
- See if there is much performance impact of using a union-type for volumes stored in leaf nodes.
- Add a dynamic tree for storing moving bodies. Can this be supported by the same underlying type? Is it useful to do so?
