//! Collision primitives and the point test: on the body when within `margin` of the solid (inside counts).

/// A URDF collision primitive in its own frame (cylinder along z, everything centred on the origin).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Shape {
    /// Half extents along x, y, z.
    Box {
        half: [f64; 3],
    },
    /// Radius and half length along z.
    Cylinder {
        radius: f64,
        half_length: f64,
    },
    Sphere {
        radius: f64,
    },
}

impl Shape {
    /// Whether `p` (shape frame) is inside the shape or within `margin` of its surface.
    #[inline]
    pub fn within(&self, p: [f64; 3], margin: f64) -> bool {
        match *self {
            Shape::Box { half } => {
                let dx = (p[0].abs() - half[0]).max(0.0);
                let dy = (p[1].abs() - half[1]).max(0.0);
                let dz = (p[2].abs() - half[2]).max(0.0);
                dx * dx + dy * dy + dz * dz <= margin * margin
            }
            Shape::Cylinder { radius, half_length } => {
                let dr = ((p[0] * p[0] + p[1] * p[1]).sqrt() - radius).max(0.0);
                let dz = (p[2].abs() - half_length).max(0.0);
                dr * dr + dz * dz <= margin * margin
            }
            Shape::Sphere { radius } => {
                let reach = radius + margin;
                p[0] * p[0] + p[1] * p[1] + p[2] * p[2] <= reach * reach
            }
        }
    }

    /// Half extents of the shape's own axis-aligned bounding box.
    pub fn half_extents(&self) -> [f64; 3] {
        match *self {
            Shape::Box { half } => half,
            Shape::Cylinder { radius, half_length } => [radius, radius, half_length],
            Shape::Sphere { radius } => [radius; 3],
        }
    }
}
