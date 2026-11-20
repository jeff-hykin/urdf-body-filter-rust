//! The parts of a URDF the filter reads: links' collision primitives and the joint tree.

use crate::shape::Shape;
use crate::transform::Transform;
use crate::Error;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum JointKind {
    Fixed,
    Revolute,
    Prismatic,
    /// floating or planar: posed from outside the URDF, so unsupported when it moves geometry
    Unsupported,
}

#[derive(Debug, Clone)]
pub(crate) struct Joint {
    pub name: String,
    pub type_name: String,
    pub kind: JointKind,
    pub parent: String,
    pub child: String,
    pub origin: Transform,
    pub axis: [f64; 3],
    /// (leader joint, multiplier, offset)
    pub mimic: Option<(String, f64, f64)>,
}

#[derive(Debug, Clone)]
pub(crate) struct Collision {
    pub link: String,
    pub origin: Transform,
    /// Err holds the geometry's element name when it is not a primitive (a mesh, ...).
    pub shape: Result<Shape, String>,
}

#[derive(Debug, Clone, Default)]
pub(crate) struct Urdf {
    pub links: Vec<String>,
    pub joints: Vec<Joint>,
    pub collisions: Vec<Collision>,
}

fn numbers<const N: usize>(text: Option<&str>, default: [f64; N], what: &str) -> Result<[f64; N], Error> {
    let Some(text) = text else {
        return Ok(default);
    };
    let values: Vec<f64> = text
        .split_whitespace()
        .map(|v| v.parse::<f64>())
        .collect::<Result<_, _>>()
        .map_err(|_| Error::Urdf(format!("{what}: not numbers: {text:?}")))?;
    values.try_into().map_err(|_| Error::Urdf(format!("{what}: expected {N} numbers, got {text:?}")))
}

fn number(node: roxmltree::Node, attribute: &str, what: &str) -> Result<f64, Error> {
    let text = node.attribute(attribute).ok_or_else(|| Error::Urdf(format!("{what}: no {attribute}")))?;
    text.trim().parse().map_err(|_| Error::Urdf(format!("{what}: {attribute} is not a number: {text:?}")))
}

fn origin(parent: roxmltree::Node, what: &str) -> Result<Transform, Error> {
    let Some(node) = parent.children().find(|n| n.has_tag_name("origin")) else {
        return Ok(Transform::IDENTITY);
    };
    let xyz = numbers(node.attribute("xyz"), [0.0; 3], what)?;
    let rpy = numbers(node.attribute("rpy"), [0.0; 3], what)?;
    Ok(Transform::from_xyz_rpy(xyz, rpy))
}

fn child<'a, 'i>(node: roxmltree::Node<'a, 'i>, tag: &str) -> Option<roxmltree::Node<'a, 'i>> {
    node.children().find(|n| n.has_tag_name(tag))
}

fn shape(geometry: roxmltree::Node, what: &str) -> Result<Result<Shape, String>, Error> {
    let Some(node) = geometry.children().find(|n| n.is_element()) else {
        return Err(Error::Urdf(format!("{what}: empty geometry")));
    };
    // a negative size would still match points (a sphere's radius + padding squared), a NaN one none: refuse both
    let size = |value: f64, name: &str| {
        if value.is_finite() && value >= 0.0 {
            Ok(value)
        } else {
            Err(Error::Urdf(format!("{what}: {name} must be finite and >= 0, got {value}")))
        }
    };
    Ok(match node.tag_name().name() {
        "box" => {
            let [x, y, z] = numbers(node.attribute("size"), [f64::NAN; 3], what)?;
            if [x, y, z].iter().any(|v| v.is_nan()) {
                return Err(Error::Urdf(format!("{what}: box without a size")));
            }
            Ok(Shape::Box { half: [size(x, "box size")? / 2.0, size(y, "box size")? / 2.0, size(z, "box size")? / 2.0] })
        }
        "cylinder" => Ok(Shape::Cylinder {
            radius: size(number(node, "radius", what)?, "radius")?,
            half_length: size(number(node, "length", what)?, "length")? / 2.0,
        }),
        "sphere" => Ok(Shape::Sphere { radius: size(number(node, "radius", what)?, "radius")? }),
        other => Err(other.to_string()),
    })
}

pub(crate) fn parse(xml: &str) -> Result<Urdf, Error> {
    let document = roxmltree::Document::parse(xml).map_err(|e| Error::Urdf(e.to_string()))?;
    let robot = document.root_element();
    if !robot.has_tag_name("robot") {
        return Err(Error::Urdf(format!("the root element is <{}>, not <robot>", robot.tag_name().name())));
    }
    let mut urdf = Urdf::default();
    for node in robot.children().filter(|n| n.has_tag_name("link")) {
        let link = node.attribute("name").ok_or_else(|| Error::Urdf("a link without a name".into()))?.to_string();
        for collision in node.children().filter(|n| n.has_tag_name("collision")) {
            let what = format!("link {link:?} collision");
            let geometry = child(collision, "geometry").ok_or_else(|| Error::Urdf(format!("{what}: no geometry")))?;
            urdf.collisions.push(Collision {
                link: link.clone(),
                origin: origin(collision, &what)?,
                shape: shape(geometry, &what)?,
            });
        }
        if urdf.links.contains(&link) {
            return Err(Error::Urdf(format!("two links named {link:?}")));
        }
        urdf.links.push(link);
    }
    for node in robot.children().filter(|n| n.has_tag_name("joint")) {
        let name = node.attribute("name").ok_or_else(|| Error::Urdf("a joint without a name".into()))?.to_string();
        let what = format!("joint {name:?}");
        if urdf.joints.iter().any(|j| j.name == name) {
            return Err(Error::Urdf(format!("two joints named {name:?}")));
        }
        let type_name = node.attribute("type").ok_or_else(|| Error::Urdf(format!("{what}: no type")))?.to_string();
        let kind = match type_name.as_str() {
            "fixed" => JointKind::Fixed,
            "revolute" | "continuous" => JointKind::Revolute,
            "prismatic" => JointKind::Prismatic,
            "floating" | "planar" => JointKind::Unsupported,
            other => return Err(Error::Urdf(format!("{what}: unknown type {other:?}"))),
        };
        let link_of = |tag: &str| -> Result<String, Error> {
            child(node, tag)
                .and_then(|n| n.attribute("link"))
                .map(str::to_string)
                .ok_or_else(|| Error::Urdf(format!("{what}: no {tag} link")))
        };
        let axis = numbers(child(node, "axis").and_then(|n| n.attribute("xyz")), [1.0, 0.0, 0.0], &what)?;
        let norm = axis.iter().map(|v| v * v).sum::<f64>().sqrt();
        if kind != JointKind::Fixed && (norm.is_nan() || norm <= 0.0) {
            return Err(Error::Urdf(format!("{what}: zero axis")));
        }
        let mimic = match child(node, "mimic") {
            None => None,
            Some(m) => Some((
                m.attribute("joint").ok_or_else(|| Error::Urdf(format!("{what}: mimic without a joint")))?.to_string(),
                m.attribute("multiplier").map_or(Ok(1.0), |_| number(m, "multiplier", &what))?,
                m.attribute("offset").map_or(Ok(0.0), |_| number(m, "offset", &what))?,
            )),
        };
        urdf.joints.push(Joint {
            parent: link_of("parent")?,
            child: link_of("child")?,
            origin: origin(node, &what)?,
            axis: if norm > 0.0 { axis.map(|v| v / norm) } else { axis },
            name,
            type_name,
            kind,
            mimic,
        });
    }
    Ok(urdf)
}
