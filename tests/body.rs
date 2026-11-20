use std::collections::HashMap;
use std::f64::consts::FRAC_PI_2;

use urdf_body_filter::{Body, Error, Options, Transform};

// A trunk box, an arm on a revolute joint (cylinder along its link's x), a prismatic slider with a sphere,
// a mimic joint, and a mesh-only link.
const URDF: &str = r#"<?xml version="1.0"?>
<robot name="test">
  <link name="base"><collision><geometry><box size="1.0 0.4 0.2"/></geometry></collision></link>
  <link name="arm"><collision><origin xyz="0.25 0 0" rpy="0 1.5707963267948966 0"/><geometry><cylinder radius="0.05" length="0.5"/></geometry></collision></link>
  <link name="slider"><collision><geometry><sphere radius="0.1"/></geometry></collision></link>
  <link name="twin"><collision><origin xyz="0.2 0 0"/><geometry><sphere radius="0.05"/></geometry></collision></link>
  <link name="camera"><collision><geometry><mesh filename="cam.stl"/></geometry></collision></link>
  <link name="plain"/>
  <joint name="shoulder" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0.5 0 0.1"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>
  <joint name="slide" type="prismatic"><parent link="base"/><child link="slider"/><origin xyz="0 0 -0.5"/><axis xyz="0 2 0"/></joint>
  <joint name="twin_joint" type="continuous"><parent link="base"/><child link="twin"/><origin xyz="-0.5 0 0.5"/><axis xyz="0 0 1"/><mimic joint="shoulder" multiplier="-1" offset="0"/></joint>
  <joint name="camera_mount" type="fixed"><parent link="base"/><child link="camera"/><origin xyz="0 0 0.3"/></joint>
  <joint name="spare" type="revolute"><parent link="base"/><child link="plain"/><axis xyz="1 0 0"/></joint>
</robot>"#;

fn body(padding_m: f64) -> Body {
    let links = ["base", "arm", "slider", "twin"].map(String::from).to_vec();
    Body::from_urdf(URDF, &Options { padding_m, links: Some(links) }).unwrap()
}

fn state(pairs: &[(&str, f64)]) -> HashMap<String, f64> {
    pairs.iter().map(|(k, v)| (k.to_string(), *v)).collect()
}

#[test]
fn a_mesh_is_refused_unless_its_link_is_left_out() {
    let all = Body::from_urdf(URDF, &Options { padding_m: 0.0, links: None });
    assert_eq!(all.unwrap_err(), Error::UnsupportedGeometry { link: "camera".into(), geometry: "mesh".into() });
    assert_eq!(body(0.0).shape_count(), 4);
    assert_eq!(body(0.0).root_link(), "base");
    let unknown = Body::from_urdf(URDF, &Options { padding_m: 0.0, links: Some(vec!["nope".into()]) });
    assert_eq!(unknown.unwrap_err(), Error::UnknownLink("nope".into()));
    assert!(matches!(Body::from_urdf(URDF, &Options { padding_m: -1.0, links: None }), Err(Error::Padding(_))));
}

#[test]
fn only_joints_that_move_geometry_are_needed() {
    let body = body(0.0);
    let mut needed: Vec<&str> = body.needed_joints().collect();
    needed.sort();
    // the mimic joint is needed but can come from its leader; "spare" moves nothing
    assert_eq!(needed, ["shoulder", "slide", "twin_joint"]);
    let missing = body.pose(|_| None, &Transform::IDENTITY).unwrap_err();
    assert!(matches!(missing, Error::MissingJoint(_)));
    let positions = state(&[("shoulder", 0.0)]);
    assert_eq!(
        body.pose(|j| positions.get(j).copied(), &Transform::IDENTITY).unwrap_err(),
        Error::MissingJoint("slide".into())
    );
    let nan = state(&[("shoulder", f64::NAN), ("slide", 0.0)]);
    assert!(body.pose(|j| nan.get(j).copied(), &Transform::IDENTITY).is_err());
}

#[test]
fn primitives_follow_their_joints() {
    let body = body(0.0);
    let positions = state(&[("shoulder", FRAC_PI_2), ("slide", 0.1)]);
    let posed = body.pose(|j| positions.get(j).copied(), &Transform::IDENTITY).unwrap();
    // trunk box: half extents 0.5, 0.2, 0.1
    assert!(posed.contains([0.0, 0.0, 0.0]));
    assert!(posed.contains([-0.49, 0.19, -0.09]));
    assert!(!posed.contains([0.0, 0.0, 0.11]));
    // the arm, turned 90 degrees, now runs along +y from (0.5, 0, 0.1)
    assert!(posed.contains([0.5, 0.45, 0.1]));
    assert!(!posed.contains([0.75, 0.0, 0.1]));
    assert!(!posed.contains([0.5, 0.45, 0.16]));
    // the slider's sphere moved 0.1 * |axis| (axis normalized) along y
    assert!(posed.contains([0.0, 0.19, -0.5]));
    assert!(!posed.contains([0.0, -0.05, -0.5]));
    // the twin follows -shoulder: its sphere at (-0.5, 0, 0.5) + Rz(-90deg) * (0.2, 0, 0)
    assert!(posed.contains([-0.5, -0.2, 0.5]));
    assert!(!posed.contains([-0.3, 0.0, 0.5]));
}

#[test]
fn padding_grows_every_primitive_by_its_distance() {
    let positions = state(&[("shoulder", 0.0), ("slide", 0.0)]);
    for (padding, expect) in [(0.0, false), (0.049, false), (0.051, true)] {
        let posed = body(padding).pose(|j| positions.get(j).copied(), &Transform::IDENTITY).unwrap();
        // 0.05 past the trunk's top face, past its corner along the diagonal, and past the sphere
        assert_eq!(posed.contains([0.0, 0.0, 0.15]), expect, "face {padding}");
        let d = 0.05 / 3f64.sqrt();
        assert_eq!(posed.contains([-0.5 - d, -0.2 - d, -0.1 - d]), expect, "corner {padding}");
        assert_eq!(posed.contains([0.0, 0.0, -0.65]), expect, "sphere {padding}");
        // the arm's cylinder: 0.05 past its rim and past its far end cap
        assert_eq!(posed.contains([0.75, 0.0, 0.2]), expect, "rim {padding}");
        assert_eq!(posed.contains([1.05, 0.0, 0.1]), expect, "cap {padding}");
    }
}

#[test]
fn points_are_taken_in_the_sensor_frame() {
    let positions = state(&[("shoulder", 0.0), ("slide", 0.0)]);
    // sensor 3 m above the base, upside down (roll 180 degrees): base points sit at z = -3 in its frame
    let root_from_sensor = Transform::from_xyz_rpy([0.0, 0.0, 3.0], [std::f64::consts::PI, 0.0, 0.0]);
    let posed = body(0.0).pose(|j| positions.get(j).copied(), &root_from_sensor).unwrap();
    assert!(posed.contains([0.0, 0.0, 3.0]));
    assert!(!posed.contains([0.0, 0.0, 0.0]));
    // the slider at y=0 in base is at y=0, z=3.5 in the flipped sensor
    assert!(posed.contains([0.0, 0.0, 3.5]));
    let mut cloud = vec![[0.0f32, 0.0, 3.0], [9.0, 9.0, 9.0], [0.3, -0.1, 3.0], [-9.0, 0.0, 0.0]];
    let mask = posed.mask(&cloud, |p| p.map(f64::from));
    assert_eq!(mask, [true, false, true, false]);
    assert_eq!(posed.retain(&mut cloud, |p| p.map(f64::from)), 2);
    assert_eq!(cloud, [[9.0, 9.0, 9.0], [-9.0, 0.0, 0.0]]);
}

#[test]
fn transforms_compose_and_invert() {
    let a = Transform::from_xyz_rpy([1.0, 2.0, 3.0], [0.3, -0.7, 2.1]);
    let q = Transform::from_translation_quaternion([0.5, 0.0, -1.0], [0.1, 0.2, 0.3, 0.9]);
    let p = [0.4, -0.2, 1.7];
    let round = a.inverse().apply(a.apply(p));
    let composed = a.then(&q).apply(p);
    let stepwise = a.apply(q.apply(p));
    for i in 0..3 {
        assert!((round[i] - p[i]).abs() < 1e-12);
        assert!((composed[i] - stepwise[i]).abs() < 1e-12);
    }
}

#[test]
fn negative_or_nan_sizes_and_repeated_names_are_refused() {
    let one = |geometry: &str| format!(r#"<robot name="r"><link name="a"><collision><geometry>{geometry}</geometry></collision></link></robot>"#);
    let load = |xml: &str| Body::from_urdf(xml, &Options { padding_m: 0.0, links: None });
    assert!(load(&one(r#"<sphere radius="0.5"/>"#)).is_ok());
    // a radius of -0.5 squared matched every point within 0.5 m, as a 0.5 m sphere would
    for bad in [r#"<sphere radius="-0.5"/>"#, r#"<cylinder radius="NaN" length="1"/>"#, r#"<cylinder radius="1" length="-1"/>"#, r#"<box size="1 -1 1"/>"#, r#"<box size="1 inf 1"/>"#] {
        assert!(matches!(load(&one(bad)), Err(Error::Urdf(_))), "{bad}");
    }
    // a second link of the same name used to replace the first in the index silently
    let twice = r#"<robot name="r"><link name="a"/><link name="a"/></robot>"#;
    assert!(matches!(load(twice), Err(Error::Urdf(why)) if why.contains("two links")));
    let joints = r#"<robot name="r"><link name="a"/><link name="b"/><link name="c"/>
        <joint name="j" type="fixed"><parent link="a"/><child link="b"/></joint>
        <joint name="j" type="fixed"><parent link="a"/><child link="c"/></joint></robot>"#;
    assert!(matches!(load(joints), Err(Error::Urdf(why)) if why.contains("two joints")));
}
