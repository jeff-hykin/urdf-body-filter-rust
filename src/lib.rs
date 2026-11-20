//! Cut a robot's own body out of a point cloud, using the URDF's collision geometry.
//!
//! [`Body::from_urdf`] reads the collision primitives (box, cylinder, sphere) and the joint tree once.
//! [`Body::pose`] places them at a joint state, in the frame of the sensor that took the cloud, and the
//! [`PosedBody`] it returns says which points are on the body: inside a primitive or within `padding_m` of it.
//!
//! ```
//! use urdf_body_filter::{Body, Options, Transform};
//!
//! let urdf = r#"<robot name="r"><link name="base"><collision><geometry><box size="1 1 1"/></geometry></collision></link></robot>"#;
//! let body = Body::from_urdf(urdf, &Options { padding_m: 0.05, links: None })?;
//! // the sensor sits 2 m in front of the base, looking back along -x
//! let base_from_sensor = Transform::from_xyz_rpy([2.0, 0.0, 0.0], [0.0, 0.0, std::f64::consts::PI]);
//! let posed = body.pose(|_joint| None, &base_from_sensor)?;
//! assert!(posed.contains([2.0, 0.0, 0.0])); // the base's centre, 2 m ahead of the sensor
//! assert!(!posed.contains([1.0, 0.0, 0.0]));
//! let mut points = vec![[2.0_f32, 0.0, 0.0], [5.0, 0.0, 0.0]];
//! posed.retain(&mut points, |p| [p[0] as f64, p[1] as f64, p[2] as f64]);
//! assert_eq!(points, [[5.0, 0.0, 0.0]]);
//! # Ok::<(), urdf_body_filter::Error>(())
//! ```

mod shape;
mod transform;
mod urdf;

use std::collections::{HashMap, HashSet};

pub use shape::Shape;
pub use transform::Transform;
use urdf::JointKind;

/// What goes into the filter besides the URDF.
#[derive(Debug, Clone, PartialEq)]
pub struct Options {
    /// Distance (m) around the collision geometry that still counts as the body.
    pub padding_m: f64,
    /// Only these links' collision geometry; None takes every link's.
    pub links: Option<Vec<String>>,
}

/// Why a URDF could not be loaded or a joint state could not be posed.
#[derive(Debug, Clone, PartialEq)]
pub enum Error {
    /// Malformed or unreadable URDF.
    Urdf(String),
    /// Padding is negative or not finite.
    Padding(f64),
    /// A selected link the URDF does not have.
    UnknownLink(String),
    /// A selected link's collision geometry that is not a box, cylinder or sphere (link, geometry element).
    UnsupportedGeometry { link: String, geometry: String },
    /// A floating or planar joint that moves selected geometry (joint, type).
    UnsupportedJoint { joint: String, kind: String },
    /// The joint state has no (finite) position for a joint that moves selected geometry.
    MissingJoint(String),
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Error::Urdf(why) => write!(f, "bad URDF: {why}"),
            Error::Padding(padding) => write!(f, "padding must be finite and >= 0, got {padding}"),
            Error::UnknownLink(link) => write!(f, "the URDF has no link {link:?}"),
            Error::UnsupportedGeometry { link, geometry } => {
                write!(f, "link {link:?} has <{geometry}> collision geometry; only box, cylinder and sphere are supported (leave the link out of `links`)")
            }
            Error::UnsupportedJoint { joint, kind } => {
                write!(f, "joint {joint:?} is {kind}; the URDF root must be the frame the sensor is mounted in")
            }
            Error::MissingJoint(joint) => write!(f, "no position for joint {joint:?}"),
        }
    }
}

impl std::error::Error for Error {}

#[derive(Debug, Clone)]
struct Joint {
    name: String,
    kind: JointKind,
    parent: usize,
    child: usize,
    origin: Transform,
    axis: [f64; 3],
    mimic: Option<(String, f64, f64)>,
    /// whether it moves selected geometry (so its position is required)
    needed: bool,
}

#[derive(Debug, Clone)]
struct Placed {
    link: usize,
    link_from_shape: Transform,
    shape: Shape,
}

/// A URDF's collision geometry and joint tree, ready to be posed.
#[derive(Debug, Clone)]
pub struct Body {
    links: Vec<String>,
    /// parents before children
    joints: Vec<Joint>,
    shapes: Vec<Placed>,
    padding_m: f64,
}

impl Body {
    /// Load the collision geometry of `options.links` (every link when None) and the joints that move it.
    pub fn from_urdf(xml: &str, options: &Options) -> Result<Body, Error> {
        if !(options.padding_m.is_finite() && options.padding_m >= 0.0) {
            return Err(Error::Padding(options.padding_m));
        }
        let urdf = urdf::parse(xml)?;
        let index: HashMap<&str, usize> = urdf.links.iter().enumerate().map(|(i, l)| (l.as_str(), i)).collect();
        let link_index = |name: &str| index.get(name).copied().ok_or_else(|| Error::UnknownLink(name.to_string()));
        let selected: Option<HashSet<&str>> = match &options.links {
            None => None,
            Some(links) => {
                for link in links {
                    link_index(link)?;
                }
                Some(links.iter().map(String::as_str).collect())
            }
        };
        // the root is the one link no joint has as its child
        let children: HashSet<&str> = urdf.joints.iter().map(|j| j.child.as_str()).collect();
        let roots: Vec<&String> = urdf.links.iter().filter(|l| !children.contains(l.as_str())).collect();
        let [root] = roots.as_slice() else {
            return Err(Error::Urdf(format!("expected one root link, found {roots:?}")));
        };
        // breadth-first from the root, so every joint comes after the one that places its parent
        let mut ordered: Vec<Joint> = Vec::with_capacity(urdf.joints.len());
        let mut frontier = vec![root.as_str()];
        let mut seen: HashSet<&str> = HashSet::from([root.as_str()]);
        while let Some(link) = frontier.pop() {
            for joint in urdf.joints.iter().filter(|j| j.parent == link) {
                if !seen.insert(joint.child.as_str()) {
                    return Err(Error::Urdf(format!("link {:?} is reached twice", joint.child)));
                }
                frontier.push(joint.child.as_str());
                ordered.push(Joint {
                    name: joint.name.clone(),
                    kind: joint.kind,
                    parent: link_index(&joint.parent)?,
                    child: link_index(&joint.child)?,
                    origin: joint.origin,
                    axis: joint.axis,
                    mimic: joint.mimic.clone(),
                    needed: false,
                });
            }
        }
        if ordered.len() != urdf.joints.len() {
            return Err(Error::Urdf("a joint is not connected to the root link".into()));
        }
        let mut shapes = Vec::new();
        for collision in &urdf.collisions {
            if selected.as_ref().is_some_and(|s| !s.contains(collision.link.as_str())) {
                continue;
            }
            let shape = collision
                .shape
                .clone()
                .map_err(|geometry| Error::UnsupportedGeometry { link: collision.link.clone(), geometry })?;
            shapes.push(Placed { link: link_index(&collision.link)?, link_from_shape: collision.origin, shape });
        }
        // a joint is needed when selected geometry hangs below it
        let parent_joint: HashMap<usize, usize> = ordered.iter().enumerate().map(|(i, j)| (j.child, i)).collect();
        let mut needed_links: Vec<usize> = shapes.iter().map(|s| s.link).collect();
        let mut mimic_leaders: Vec<String> = Vec::new();
        while let Some(link) = needed_links.pop() {
            let Some(&j) = parent_joint.get(&link) else {
                continue;
            };
            if ordered[j].needed {
                continue;
            }
            ordered[j].needed = true;
            needed_links.push(ordered[j].parent);
            if let Some((leader, _, _)) = &ordered[j].mimic {
                mimic_leaders.push(leader.clone());
            }
        }
        for leader in mimic_leaders {
            let joint = ordered
                .iter_mut()
                .find(|j| j.name == leader)
                .ok_or_else(|| Error::Urdf(format!("mimic of unknown joint {leader:?}")))?;
            joint.needed = true;
        }
        if let Some(joint) = ordered.iter().find(|j| j.needed && j.kind == JointKind::Unsupported) {
            let kind = urdf.joints.iter().find(|u| u.name == joint.name).map_or("unsupported", |u| &u.type_name);
            return Err(Error::UnsupportedJoint { joint: joint.name.clone(), kind: kind.to_string() });
        }
        Ok(Body { links: urdf.links, joints: ordered, shapes, padding_m: options.padding_m })
    }

    /// The URDF's root link: the frame `pose` takes the sensor's mount in.
    pub fn root_link(&self) -> &str {
        let children: HashSet<usize> = self.joints.iter().map(|j| j.child).collect();
        (0..self.links.len()).find(|i| !children.contains(i)).map_or("", |i| &self.links[i])
    }

    /// The joints whose positions `pose` needs (a mimic joint can come from its leader instead).
    pub fn needed_joints(&self) -> impl Iterator<Item = &str> {
        self.joints.iter().filter(|j| j.needed && j.kind != JointKind::Fixed).map(|j| j.name.as_str())
    }

    /// How many collision primitives are filtered against.
    pub fn shape_count(&self) -> usize {
        self.shapes.len()
    }

    /// The padding (m) this body was loaded with.
    pub fn padding_m(&self) -> f64 {
        self.padding_m
    }

    fn position(&self, joint: &Joint, position_of: &dyn Fn(&str) -> Option<f64>, depth: usize) -> Option<f64> {
        if let Some(q) = position_of(&joint.name).filter(|q| q.is_finite()) {
            return Some(q);
        }
        let (leader, multiplier, offset) = joint.mimic.as_ref()?;
        let leader = self.joints.iter().find(|j| &j.name == leader)?;
        // a mimic chain longer than the joint count is a loop
        if depth > self.joints.len() {
            return None;
        }
        Some(multiplier * self.position(leader, position_of, depth + 1)? + offset)
    }

    /// The body at the joint positions `position_of` gives (by joint name), seen from a sensor whose pose in the
    /// root link is `root_from_sensor`. Joints that move no selected geometry may be missing.
    pub fn pose(
        &self,
        position_of: impl Fn(&str) -> Option<f64>,
        root_from_sensor: &Transform,
    ) -> Result<PosedBody, Error> {
        let mut root_from_link = vec![Transform::IDENTITY; self.links.len()];
        for joint in &self.joints {
            let motion = match joint.kind {
                JointKind::Fixed | JointKind::Unsupported => Transform::IDENTITY,
                kind => {
                    let q = match self.position(joint, &position_of, 0) {
                        Some(q) => q,
                        None if joint.needed => return Err(Error::MissingJoint(joint.name.clone())),
                        None => 0.0,
                    };
                    if kind == JointKind::Revolute {
                        Transform::from_axis_angle(joint.axis, q)
                    } else {
                        Transform::from_translation(joint.axis.map(|a| a * q))
                    }
                }
            };
            root_from_link[joint.child] = root_from_link[joint.parent].then(&joint.origin).then(&motion);
        }
        let sensor_from_root = root_from_sensor.inverse();
        let margin = self.padding_m;
        let mut shapes = Vec::with_capacity(self.shapes.len());
        let (mut low, mut high) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
        for placed in &self.shapes {
            let sensor_from_shape = sensor_from_root.then(&root_from_link[placed.link]).then(&placed.link_from_shape);
            // the shape's box in the sensor frame, grown by the padding
            let half = placed.shape.half_extents();
            let r = &sensor_from_shape.rotation;
            let reach: [f64; 3] = std::array::from_fn(|i| {
                r[i][0].abs() * half[0] + r[i][1].abs() * half[1] + r[i][2].abs() * half[2] + margin
            });
            let centre = sensor_from_shape.translation;
            let shape_low: [f64; 3] = std::array::from_fn(|i| centre[i] - reach[i]);
            let shape_high: [f64; 3] = std::array::from_fn(|i| centre[i] + reach[i]);
            for i in 0..3 {
                low[i] = low[i].min(shape_low[i]);
                high[i] = high[i].max(shape_high[i]);
            }
            shapes.push(PosedShape {
                low: shape_low,
                high: shape_high,
                shape_from_sensor: sensor_from_shape.inverse(),
                shape: placed.shape,
            });
        }
        Ok(PosedBody { shapes, low, high, margin })
    }
}

#[derive(Debug, Clone)]
struct PosedShape {
    low: [f64; 3],
    high: [f64; 3],
    shape_from_sensor: Transform,
    shape: Shape,
}

#[inline]
fn inside(p: &[f64; 3], low: &[f64; 3], high: &[f64; 3]) -> bool {
    p[0] >= low[0] && p[0] <= high[0] && p[1] >= low[1] && p[1] <= high[1] && p[2] >= low[2] && p[2] <= high[2]
}

/// The body at one joint state, in a sensor's frame.
#[derive(Debug, Clone)]
pub struct PosedBody {
    shapes: Vec<PosedShape>,
    low: [f64; 3],
    high: [f64; 3],
    margin: f64,
}

impl PosedBody {
    /// Whether `p` (sensor frame, metres) is on the body: inside a primitive or within the padding of one.
    #[inline]
    pub fn contains(&self, p: [f64; 3]) -> bool {
        if !inside(&p, &self.low, &self.high) {
            return false;
        }
        self.shapes
            .iter()
            .any(|s| inside(&p, &s.low, &s.high) && s.shape.within(s.shape_from_sensor.apply(p), self.margin))
    }

    /// One flag per point, true where the point is on the body.
    pub fn mask<P>(&self, points: &[P], xyz: impl Fn(&P) -> [f64; 3]) -> Vec<bool> {
        points.iter().map(|p| self.contains(xyz(p))).collect()
    }

    /// Drop the points on the body, keeping the others in order; returns how many were dropped.
    pub fn retain<P>(&self, points: &mut Vec<P>, xyz: impl Fn(&P) -> [f64; 3]) -> usize {
        let before = points.len();
        points.retain(|p| !self.contains(xyz(p)));
        before - points.len()
    }
}
