//! Rigid transforms: a rotation matrix and a translation, `parent_from_child` maps child points into parent.

/// A rigid transform; `apply` maps a point in the child frame into the parent frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Transform {
    /// Row-major rotation matrix.
    pub rotation: [[f64; 3]; 3],
    /// Translation in metres.
    pub translation: [f64; 3],
}

impl Default for Transform {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Transform {
    /// The transform that changes nothing.
    pub const IDENTITY: Transform =
        Transform { rotation: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], translation: [0.0; 3] };

    /// From a translation and a unit quaternion given as `[x, y, z, w]` (normalized here).
    pub fn from_translation_quaternion(translation: [f64; 3], xyzw: [f64; 4]) -> Transform {
        let norm = xyzw.iter().map(|v| v * v).sum::<f64>().sqrt();
        let [x, y, z, w] = xyzw.map(|v| v / norm);
        let rotation = [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ];
        Transform { rotation, translation }
    }

    /// From a translation and URDF roll, pitch, yaw (fixed axes: `Rz(yaw) * Ry(pitch) * Rx(roll)`).
    pub fn from_xyz_rpy(translation: [f64; 3], [roll, pitch, yaw]: [f64; 3]) -> Transform {
        let (sr, cr) = roll.sin_cos();
        let (sp, cp) = pitch.sin_cos();
        let (sy, cy) = yaw.sin_cos();
        let rotation = [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ];
        Transform { rotation, translation }
    }

    /// A rotation of `angle` radians about the unit `axis`.
    pub fn from_axis_angle(axis: [f64; 3], angle: f64) -> Transform {
        let [x, y, z] = axis;
        let (s, c) = angle.sin_cos();
        let t = 1.0 - c;
        let rotation = [
            [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
            [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
            [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
        ];
        Transform { rotation, translation: [0.0; 3] }
    }

    /// A pure translation.
    pub fn from_translation(translation: [f64; 3]) -> Transform {
        Transform { translation, ..Transform::IDENTITY }
    }

    /// The point `p` (child frame) in the parent frame.
    #[inline]
    pub fn apply(&self, p: [f64; 3]) -> [f64; 3] {
        let r = &self.rotation;
        let t = &self.translation;
        [
            r[0][0] * p[0] + r[0][1] * p[1] + r[0][2] * p[2] + t[0],
            r[1][0] * p[0] + r[1][1] * p[1] + r[1][2] * p[2] + t[1],
            r[2][0] * p[0] + r[2][1] * p[1] + r[2][2] * p[2] + t[2],
        ]
    }

    /// `self * other`: other's child frame into self's parent frame.
    pub fn then(&self, other: &Transform) -> Transform {
        let a = &self.rotation;
        let b = &other.rotation;
        let mut rotation = [[0.0; 3]; 3];
        for (i, row) in rotation.iter_mut().enumerate() {
            for (j, value) in row.iter_mut().enumerate() {
                *value = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
            }
        }
        Transform { rotation, translation: self.apply(other.translation) }
    }

    /// The inverse: parent points into the child frame.
    pub fn inverse(&self) -> Transform {
        let r = &self.rotation;
        let rotation = [[r[0][0], r[1][0], r[2][0]], [r[0][1], r[1][1], r[2][1]], [r[0][2], r[1][2], r[2][2]]];
        let t = self.translation;
        let back = Transform { rotation, translation: [0.0; 3] }.apply(t);
        Transform { rotation, translation: [-back[0], -back[1], -back[2]] }
    }
}
