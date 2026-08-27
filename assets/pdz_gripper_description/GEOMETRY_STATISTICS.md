# PDZ Gripper Slim geometry statistics

## Source and conventions

- Source: `PDZ_Gripper_Slim(1).STEP`
- Source SHA-256:
  `cecee0339da30901a8e7d2733f8d906fed6eb244912fa843f7e668c0de708c14`
- STEP solids: 35
- Pose: fully open CAD configuration
- Coordinate system: SolidWorks `Flange Mounting Point`
- Dimensions below are axis-aligned in the flange frame, not an oriented
  minimum bounding box.

## Complete assembly

The complete assembly includes the D405, robot connector and pucks, 8 mm TPU
pads, and timing belt.

| Quantity | Value |
|---|---:|
| X bounds | -84.000000 to 75.157923 mm |
| Y bounds | -117.921245 to 33.964466 mm |
| Z bounds | -5.000000 to 150.500080 mm |
| Overall X dimension | 159.157923 mm |
| Overall Y dimension | 151.885711 mm |
| Overall Z dimension | 155.500080 mm |
| Axis-aligned bounding-box volume | 3,759,030.040 mm^3 (3.759030 L) |
| Convex-hull volume | 1,620,094.415 mm^3 (1.620094 L) |
| Convex-hull surface area | 75,983.716 mm^2 |
| Sum of CAD solid volumes | 350,504.091 mm^3 (350.504 cm^3) |

Removing only the D405 does not change the outer bounds or convex hull because
the camera is contained inside the envelope established by the fingers and
robot-side connector. The summed solid volume without the D405 is
311,761.835 mm^3.

## Gripper mechanism envelope

For a mechanism-only comparison, excluding the D405 and the robot connector
and pucks but retaining the belt, motor, fingers, and tensioner:

| Quantity | Value |
|---|---:|
| Overall dimensions X x Y x Z | 159.157923 x 54.000012 x 146.950080 mm |
| Convex-hull volume | 775,602.588 mm^3 (0.775603 L) |
| Sum of CAD solid volumes | 219,021.514 mm^3 (219.022 cm^3) |

## Method and interpretation

CAD solid volumes are summed from the STEP B-rep solids. Coincident or
overlapping solids, if present, can therefore be counted more than once.

The convex hull was calculated from a tessellation of every included solid
using a 0.1 mm linear tolerance and 0.05 rad angular tolerance, followed by a
3D Qhull calculation. The complete hull used 652,353 input vertices and 3,402
hull vertices. The hull includes all empty space bridged between external
features; it is an envelope volume, not material volume and not displaced
fluid volume.
