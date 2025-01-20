# Spatial
A library for spatial data structures and related functions.

## Note on overlap check
The Ball2f and Box2f implementations of overlapsOther were modified to remove the isEmpty check.
This significantly improves performance,
Times (ms) shown below were recorded in overlap benchmarks for 10 trials of 20000 bodies.
|Volume	| Grid	|No empty check	| Parent empty check	| Full empty check|
| --- | --- | --- | --- | --- |
|Ball	| 4x3	| 3.3	| 3.4	| 3.6 |
|Ball | 2x6	| 5.9 | 6.1 | 6.7 |
|Box	| 4x3	| 2.0 |	2.3	| 2.5 |
|Box  | 2x6	| 3.2	| 3.5	| 4.2 |

SquareGrid uses empty bounding volumes for nodes with no data in their leaf successors.
These volumes have zero size, and are placed at (float-max, float-max).
This makes it extremely unlikely that skipping the empty check will affect results.
