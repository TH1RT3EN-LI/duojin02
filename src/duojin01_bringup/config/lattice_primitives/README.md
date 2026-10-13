# Omni lattice control set

The source control set inherited from duojin01 uses a **2 cm discretization** and
16 headings, with 480 primitives for lateral, diagonal, translation and rotation
motions. The source discretization is not a requirement to change navigation
maps to 2 cm.

The project launch workflow generates a matching Omni control set for the actual
global costmap resolution, including the documented 1 cm and legacy 2/3 cm map
workflows. The **effective control-set resolution must match the global costmap**;
check the generated/selected set rather than using this source specification as
a runtime default. Do not edit generated poses independently of their source
specification. Installed paths are resolved by launch. Workflow history is in the
[navigation upgrade report](../../../../docs/navigation-upgrade-review.md).

Generic planner smoothing is disabled. Project-specific postprocessing has its
own filled-footprint and soft-cost checks; see the current
[planner contract](../../../../docs/navigation-repairs.md). The independent braking
sweep validates commanded motion against filled lethal cells and observations.
