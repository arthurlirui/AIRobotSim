#!/usr/bin/env python3
"""Parse URDF files and extract key kinematic parameters for Unitree Go2 and H1 robots."""

import xml.etree.ElementTree as ET
import math
import os


def parse_floats(text, count=3):
    """Parse a space-separated string of floats."""
    if text is None:
        return None
    parts = text.strip().split()
    return [float(p) for p in parts]


def get_link_mass(link_elem):
    """Extract mass from a link element."""
    inertial = link_elem.find('inertial')
    if inertial is None:
        return None
    mass_elem = inertial.find('mass')
    if mass_elem is None:
        return None
    return float(mass_elem.get('value'))


def get_link_inertial_origin(link_elem):
    """Extract inertial origin xyz from a link element."""
    inertial = link_elem.find('inertial')
    if inertial is None:
        return None
    origin = inertial.find('origin')
    if origin is None:
        return None
    return parse_floats(origin.get('xyz'))


def get_collision_box_size(link_elem):
    """Extract collision box dimensions if present."""
    collision = link_elem.find('collision')
    if collision is None:
        return None
    geom = collision.find('geometry')
    if geom is None:
        return None
    box = geom.find('box')
    if box is not None:
        return parse_floats(box.get('size'))
    return None


def get_collision_geom_info(link_elem):
    """Extract collision geometry info (type + dimensions)."""
    collision = link_elem.find('collision')
    if collision is None:
        return None
    geom = collision.find('geometry')
    if geom is None:
        return None
    box = geom.find('box')
    if box is not None:
        return ('box', parse_floats(box.get('size')))
    cyl = geom.find('cylinder')
    if cyl is not None:
        return ('cylinder', float(cyl.get('radius')), float(cyl.get('length')))
    sphere = geom.find('sphere')
    if sphere is not None:
        return ('sphere', float(sphere.get('radius')))
    return None


def parse_urdf(filepath):
    """Parse a URDF file and return structured data."""
    tree = ET.parse(filepath)
    root = tree.getroot()

    robot_name = root.get('name')

    # Collect all links by name
    links = {}
    for link_elem in root.findall('link'):
        name = link_elem.get('name')
        links[name] = link_elem

    # Collect all joints
    joints = []
    for joint_elem in root.findall('joint'):
        j = {
            'name': joint_elem.get('name'),
            'type': joint_elem.get('type'),
        }
        origin = joint_elem.find('origin')
        if origin is not None:
            j['origin_xyz'] = parse_floats(origin.get('xyz'))
            j['origin_rpy'] = parse_floats(origin.get('rpy'))
        else:
            j['origin_xyz'] = [0, 0, 0]
            j['origin_rpy'] = [0, 0, 0]

        parent = joint_elem.find('parent')
        child = joint_elem.find('child')
        j['parent'] = parent.get('link') if parent is not None else None
        j['child'] = child.get('link') if child is not None else None

        axis = joint_elem.find('axis')
        if axis is not None:
            j['axis'] = parse_floats(axis.get('xyz'))
        else:
            j['axis'] = None

        limit = joint_elem.find('limit')
        if limit is not None:
            j['limit'] = {
                'lower': float(limit.get('lower', '0')),
                'upper': float(limit.get('upper', '0')),
                'effort': float(limit.get('effort', '0')),
                'velocity': float(limit.get('velocity', '0')),
            }
        else:
            j['limit'] = None

        joints.append(j)

    return robot_name, links, joints


def get_link_by_name(links, name):
    return links.get(name)


def print_separator(char='=', width=90):
    print(char * width)


def print_go2_summary(filepath):
    """Print kinematic summary for Go2 quadruped."""
    robot_name, links, joints = parse_urdf(filepath)
    print_separator('=')
    print(f"  ROBOT: {robot_name}  (Unitree Go2 - Quadruped)")
    print(f"  File: {filepath}")
    print_separator('=')
    print()

    # 1. Base/trunk
    base = links.get('base')
    base_mass = get_link_mass(base)
    base_box = get_collision_box_size(base)
    base_inertial_origin = get_link_inertial_origin(base)

    print("--- 1. BASE / TRUNK ---")
    print(f"  Link name:       base")
    print(f"  Mass:            {base_mass} kg")
    print(f"  Inertial origin: {base_inertial_origin} m")
    if base_box:
        print(f"  Collision box:   {base_box[0]} x {base_box[1]} x {base_box[2]} m")
        print(f"    (length x width x height)")
    print()

    # 2 & 4. Leg joints and hip offsets
    # Go2 legs: FL, FR, RL, RR each with hip_joint, thigh_joint, calf_joint
    leg_prefixes = ['FL', 'FR', 'RL', 'RR']
    leg_joint_suffixes = ['hip_joint', 'thigh_joint', 'calf_joint']

    print("--- 2. LEG JOINTS (revolute) ---")
    print()

    for prefix in leg_prefixes:
        print(f"  [{prefix} Leg]")
        for suffix in leg_joint_suffixes:
            jname = f"{prefix}_{suffix}"
            j = next((x for x in joints if x['name'] == jname), None)
            if j is None:
                print(f"    {jname}: NOT FOUND")
                continue
            print(f"    {jname}")
            print(f"      type:     {j['type']}")
            print(f"      axis:     {j['axis']}")
            print(f"      parent:   {j['parent']}")
            print(f"      child:    {j['child']}")
            print(f"      origin:   xyz={j['origin_xyz']}  rpy={j['origin_rpy']}")
            if j['limit']:
                lim = j['limit']
                print(f"      limits:   lower={lim['lower']:.4f} rad ({math.degrees(lim['lower']):.2f} deg)"
                      f"  upper={lim['upper']:.4f} rad ({math.degrees(lim['upper']):.2f} deg)")
                print(f"                effort={lim['effort']} N.m  velocity={lim['velocity']} rad/s")
            print()

    # 3. Link lengths (from joint origin offsets)
    print("--- 3. LINK LENGTHS (from joint origin offsets) ---")
    print()
    print("  Thigh length (hip_joint -> thigh_joint offset, z-component of thigh_joint origin):")

    for prefix in leg_prefixes:
        thigh_j = next((x for x in joints if x['name'] == f"{prefix}_thigh_joint"), None)
        if thigh_j:
            off = thigh_j['origin_xyz']
            print(f"    {prefix}: offset xyz = {off}  (lateral hip offset y = {off[1]:.4f} m)")

    print()
    print("  Calf length (thigh_joint -> calf_joint offset, z-component):")
    for prefix in leg_prefixes:
        calf_j = next((x for x in joints if x['name'] == f"{prefix}_calf_joint"), None)
        if calf_j:
            off = calf_j['origin_xyz']
            print(f"    {prefix}: offset xyz = {off}  (calf length z = {abs(off[2]):.4f} m)")

    print()
    print("  Foot offset (calf_joint -> foot_joint):")
    for prefix in leg_prefixes:
        foot_j = next((x for x in joints if x['name'] == f"{prefix}_foot_joint"), None)
        if foot_j:
            off = foot_j['origin_xyz']
            print(f"    {prefix}: offset xyz = {off}  (foot length z = {abs(off[2]):.4f} m)")

    print()

    # 4. Hip offsets (where legs attach to the body)
    print("--- 4. HIP OFFSETS (hip_joint origins relative to base) ---")
    print()
    for prefix in leg_prefixes:
        hip_j = next((x for x in joints if x['name'] == f"{prefix}_hip_joint"), None)
        if hip_j:
            off = hip_j['origin_xyz']
            print(f"    {prefix}_hip_joint: x={off[0]:+.4f}  y={off[1]:+.4f}  z={off[2]:+.4f} m")
    print()
    print("  Summary: all hip joints share x=+-0.1934, y=+-0.0465, z=0.0")
    print()

    # 5. Link masses
    print("--- 5. LINK MASSES ---")
    print()
    link_names_of_interest = ['base']
    for prefix in leg_prefixes:
        link_names_of_interest.extend([
            f"{prefix}_hip",
            f"{prefix}_thigh",
            f"{prefix}_calf",
            f"{prefix}_foot",
        ])
    # also head
    link_names_of_interest.extend(['Head_upper', 'Head_lower'])

    for lname in link_names_of_interest:
        lelem = links.get(lname)
        if lelem is not None:
            mass = get_link_mass(lelem)
            inert_origin = get_link_inertial_origin(lelem)
            print(f"    {lname:20s}: mass = {mass} kg  | inertial origin = {inert_origin}")
        else:
            print(f"    {lname:20s}: NOT FOUND")
    print()

    # Total mass
    total = 0.0
    for lelem in links.values():
        m = get_link_mass(lelem)
        if m is not None:
            total += m
    print(f"  Total robot mass (sum of all link masses): {total:.4f} kg")
    print()


def print_h1_summary(filepath):
    """Print kinematic summary for H1 humanoid."""
    robot_name, links, joints = parse_urdf(filepath)
    print_separator('=')
    print(f"  ROBOT: {robot_name}  (Unitree H1 - Humanoid)")
    print(f"  File: {filepath}")
    print_separator('=')
    print()

    # 1. Base/pelvis and torso
    pelvis = links.get('pelvis')
    pelvis_mass = get_link_mass(pelvis)
    pelvis_inertial_origin = get_link_inertial_origin(pelvis)
    pelvis_collision = get_collision_geom_info(pelvis)

    torso = links.get('torso_link')
    torso_mass = get_link_mass(torso)
    torso_inertial_origin = get_link_inertial_origin(torso)
    torso_collision = get_collision_geom_info(torso)

    print("--- 1. BASE / TRUNK ---")
    print(f"  Pelvis (base link):")
    print(f"    Link name:       pelvis")
    print(f"    Mass:            {pelvis_mass} kg")
    print(f"    Inertial origin: {pelvis_inertial_origin} m")
    if pelvis_collision:
        print(f"    Collision:       {pelvis_collision}")
    print()
    print(f"  Torso:")
    print(f"    Link name:       torso_link")
    print(f"    Mass:            {torso_mass} kg")
    print(f"    Inertial origin: {torso_inertial_origin} m")
    if torso_collision:
        if torso_collision[0] == 'box':
            print(f"    Collision box:   {torso_collision[1][0]} x {torso_collision[1][1]} x {torso_collision[1][2]} m")
    print()

    # 2. Leg joints (left and right, each: hip_yaw, hip_roll, hip_pitch, knee, ankle)
    leg_sides = ['left', 'right']
    leg_joint_names = [
        ('hip_yaw_joint',   'left_hip_yaw_link',  'right_hip_yaw_link'),
        ('hip_roll_joint',  'left_hip_roll_link', 'right_hip_roll_link'),
        ('hip_pitch_joint', 'left_hip_pitch_link','right_hip_pitch_link'),
        ('knee_joint',      'left_knee_link',     'right_knee_link'),
        ('ankle_joint',     'left_ankle_link',    'right_ankle_link'),
    ]

    print("--- 2. LEG JOINTS (revolute) ---")
    print()

    for side in leg_sides:
        print(f"  [{side.upper()} Leg]")
        for jtype, left_child, right_child in leg_joint_names:
            jname = f"{side}_{jtype}"
            j = next((x for x in joints if x['name'] == jname), None)
            if j is None:
                print(f"    {jname}: NOT FOUND")
                continue
            print(f"    {jname}")
            print(f"      type:     {j['type']}")
            print(f"      axis:     {j['axis']}")
            print(f"      parent:   {j['parent']}")
            print(f"      child:    {j['child']}")
            print(f"      origin:   xyz={j['origin_xyz']}  rpy={j['origin_rpy']}")
            if j['limit']:
                lim = j['limit']
                print(f"      limits:   lower={lim['lower']:.4f} rad ({math.degrees(lim['lower']):.2f} deg)"
                      f"  upper={lim['upper']:.4f} rad ({math.degrees(lim['upper']):.2f} deg)")
                print(f"                effort={lim['effort']} N.m  velocity={lim['velocity']} rad/s")
            print()
        print()

    # 3. Link lengths (from joint origin offsets)
    print("--- 3. LINK LENGTHS (from joint origin offsets) ---")
    print()

    for side in leg_sides:
        print(f"  [{side.upper()} Leg]")

        # hip_yaw -> hip_roll offset
        roll_j = next((x for x in joints if x['name'] == f"{side}_hip_roll_joint"), None)
        if roll_j:
            print(f"    hip_yaw -> hip_roll:  xyz={roll_j['origin_xyz']}  (x offset = {roll_j['origin_xyz'][0]:.4f} m)")

        # hip_roll -> hip_pitch offset
        pitch_j = next((x for x in joints if x['name'] == f"{side}_hip_pitch_joint"), None)
        if pitch_j:
            print(f"    hip_roll -> hip_pitch: xyz={pitch_j['origin_xyz']}  (lateral y = {pitch_j['origin_xyz'][1]:.4f} m)")

        # hip_pitch -> knee offset (thigh length)
        knee_j = next((x for x in joints if x['name'] == f"{side}_knee_joint"), None)
        if knee_j:
            print(f"    hip_pitch -> knee:     xyz={knee_j['origin_xyz']}  (thigh length z = {abs(knee_j['origin_xyz'][2]):.4f} m)")

        # knee -> ankle offset (shank length)
        ankle_j = next((x for x in joints if x['name'] == f"{side}_ankle_joint"), None)
        if ankle_j:
            print(f"    knee -> ankle:         xyz={ankle_j['origin_xyz']}  (shank length z = {abs(ankle_j['origin_xyz'][2]):.4f} m)")
        print()

    # 4. Hip offsets (where legs attach to pelvis)
    print("--- 4. HIP OFFSETS (hip_yaw_joint origins relative to pelvis) ---")
    print()
    for side in leg_sides:
        yaw_j = next((x for x in joints if x['name'] == f"{side}_hip_yaw_joint"), None)
        if yaw_j:
            off = yaw_j['origin_xyz']
            print(f"    {side}_hip_yaw_joint: x={off[0]:+.4f}  y={off[1]:+.4f}  z={off[2]:+.4f} m")
    print()
    print("  Summary: hip yaw joints at x=0, y=+-0.0875, z=-0.1742 relative to pelvis")
    print()

    # Also torso joint offset
    torso_j = next((x for x in joints if x['name'] == 'torso_joint'), None)
    if torso_j:
        print(f"  Torso joint offset (pelvis -> torso): xyz={torso_j['origin_xyz']}  rpy={torso_j['origin_rpy']}")
        print()

    # 5. Link masses
    print("--- 5. LINK MASSES ---")
    print()

    leg_link_names = []
    for side in leg_sides:
        leg_link_names.extend([
            f"{side}_hip_yaw_link",
            f"{side}_hip_roll_link",
            f"{side}_hip_pitch_link",
            f"{side}_knee_link",
            f"{side}_ankle_link",
        ])

    arm_link_names = []
    for side in leg_sides:
        arm_link_names.extend([
            f"{side}_shoulder_pitch_link",
            f"{side}_shoulder_roll_link",
            f"{side}_shoulder_yaw_link",
            f"{side}_elbow_link",
        ])

    all_interest = ['pelvis', 'torso_link'] + leg_link_names + arm_link_names

    for lname in all_interest:
        lelem = links.get(lname)
        if lelem is not None:
            mass = get_link_mass(lelem)
            inert_origin = get_link_inertial_origin(lelem)
            print(f"    {lname:30s}: mass = {mass} kg  | inertial origin = {inert_origin}")
        else:
            print(f"    {lname:30s}: NOT FOUND")
    print()

    # Arm joint summary (bonus)
    print("--- BONUS: ARM JOINTS (for completeness) ---")
    print()
    arm_joint_names = [
        'left_shoulder_pitch_joint', 'left_shoulder_roll_joint',
        'left_shoulder_yaw_joint', 'left_elbow_joint',
        'right_shoulder_pitch_joint', 'right_shoulder_roll_joint',
        'right_shoulder_yaw_joint', 'right_elbow_joint',
    ]
    for jname in arm_joint_names:
        j = next((x for x in joints if x['name'] == jname), None)
        if j is None:
            continue
        lim_str = ""
        if j['limit']:
            lim = j['limit']
            lim_str = (f"lower={lim['lower']:.4f} ({math.degrees(lim['lower']):.1f}deg) "
                       f"upper={lim['upper']:.4f} ({math.degrees(lim['upper']):.1f}deg) "
                       f"effort={lim['effort']} vel={lim['velocity']}")
        print(f"    {jname:35s} axis={j['axis']}  origin={j['origin_xyz']}  {lim_str}")
    print()

    # Torso joint
    if torso_j:
        print(f"  torso_joint: axis={torso_j['axis']}  origin={torso_j['origin_xyz']}")
        if torso_j['limit']:
            lim = torso_j['limit']
            print(f"    limits: lower={lim['lower']:.4f} ({math.degrees(lim['lower']):.1f}deg) "
                  f"upper={lim['upper']:.4f} ({math.degrees(lim['upper']):.1f}deg) "
                  f"effort={lim['effort']} vel={lim['velocity']}")
    print()

    # Total mass
    total = 0.0
    for lelem in links.values():
        m = get_link_mass(lelem)
        if m is not None:
            total += m
    print(f"  Total robot mass (sum of all link masses): {total:.4f} kg")
    print()


def main():
    go2_path = r"E:/Code/AIRobotSim/assets/external/unitree_ros/robots/go2_description/urdf/go2_description.urdf"
    h1_path = r"E:/Code/AIRobotSim/assets/external/unitree_ros/robots/h1_description/urdf/h1.urdf"

    print_go2_summary(go2_path)
    print()
    print()
    print_h1_summary(h1_path)


if __name__ == '__main__':
    main()
