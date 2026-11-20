# urdf-body-filter

Cut a robot's own body out of a point cloud. The URDF's collision primitives (box, cylinder, sphere) are posed at a
joint state, placed in the frame of the sensor that took the cloud, and every point inside one of them, or within
`padding_m` of it, is dropped. Same rule as a narrowphase point-vs-geometry test with a security margin (RoboPlan's
`RobotBodyFilter`), with no dependencies beyond an XML parser.

```rust
use urdf_body_filter::{Body, Options, Transform};

let body = Body::from_urdf(&std::fs::read_to_string("robot.urdf")?, &Options { padding_m: 0.01, links: None })?;
let base_from_lidar = Transform::from_xyz_rpy([0.32, 0.0, -0.013], [0.0, -1.5708, 3.1416]);
let posed = body.pose(|joint| joint_positions.get(joint).copied(), &base_from_lidar)?;
points.retain(|p| !posed.contains([p.x as f64, p.y as f64, p.z as f64]));
```

License: Apache-2.0
