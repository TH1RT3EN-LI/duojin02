# Omni lattice control set

The 2 cm / 16-heading rich control set is carried from the project's existing
duojin01 implementation, alongside its original source specification. It has
480 primitives, including lateral, diagonal, translation and rotation motions.
The resolution must match the global costmap. Do not edit the generated poses
independently of their source specification. Installed paths are resolved by launch.

Planner smoothing is disabled: the returned poses retain the lattice's SE2
footprint collision checks. The independent braking sweep validates commanded
motion against filled lethal cells and sensor observations.
