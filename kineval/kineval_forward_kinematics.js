
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | forward kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded Reasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotForwardKinematics = function robotForwardKinematics () {
    if (typeof kineval.buildFKTransforms === 'undefined') {
        textbar.innerHTML = "forward kinematics not implemented";
        return;
    }
    // STENCIL: implement 
    kineval.buildFKTransforms();

}

kineval.buildFKTransforms = function buildFKTransforms () {
    traverseFKBase();
    for (let i = 0; i < robot.links[robot.base].children.length; i++)
        traverseFKJoint(robot.links[robot.base].children[i]);
}
    // STENCIL: reference code alternates recursive traversal over 
    //   links and joints starting from base, using following functions: 
    //     traverseFKBase
    //     traverseFKLink
    //     traverseFKJoint
    //
    // user interface needs the heading (z-axis) and lateral (x-axis) directions
    //   of robot base in world coordinates stored as 4x1 matrices in
    //   global variables "robot_heading" and "robot_lateral"
    //
    // if geometries are imported and using ROS coordinates (e.g., fetch),
    //   coordinate conversion is needed for kineval/threejs coordinates:
    //

function traverseFKBase() {
    let rot_x = generate_rotation_matrix_X(robot.origin.rpy[0]);
    let rot_y = generate_rotation_matrix_Y(robot.origin.rpy[1]);
    let rot_z = generate_rotation_matrix_Z(robot.origin.rpy[2]);
    let rotate = matrix_multiply(matrix_multiply(rot_x, rot_y), rot_z);
    let trans = generate_translation_matrix(robot.origin.xyz[0], 
                                            robot.origin.xyz[1], 
                                            robot.origin.xyz[2]);
    robot.links[robot.base].xform = matrix_multiply(trans, rotate);

    let local_heading = [0, 0, 1, 1];
    let local_lateral = [1, 0, 0, 1];
    robot_heading = matrix_multiply(robot.links[robot.base].xform,local_heading);
    robot_lateral = matrix_multiply(robot.links[robot.base].xform,local_lateral);
    
    if (robot.links_geom_imported) {
      let rotx_fixed = generate_rotation_matrix_X(-Math.PI/2);
      let rotz_fixed = generate_rotation_matrix_Z(-Math.PI/2);
      let ros_fixed = matrix_multiply(rotx_fixed, rotz_fixed);
      robot.links[robot.base].xform = matrix_multiply(ros_fixed, robot.links[robot.base].xform);
    }
}

function traverseFKLink(link) {
    robot.links[link].xform = robot.joints[robot.links[link].parent].xform;
    if (typeof robot.links[link].children === 'undefined'){
        return;
    }
    for (let i = 0; i < robot.links[link].children.length; i++){
        traverseFKJoint(robot.links[link].children[i]);
    }
}

function traverseFKJoint(joint) {
    let rot_x = generate_rotation_matrix_X(robot.joints[joint].origin.rpy[0]);
    let rot_y = generate_rotation_matrix_Y(robot.joints[joint].origin.rpy[1]);
    let rot_z = generate_rotation_matrix_Z(robot.joints[joint].origin.rpy[2]);
    let rotate = matrix_multiply(matrix_multiply(rot_x, rot_y), rot_z);
    let trans = generate_translation_matrix(robot.joints[joint].origin.xyz[0], 
                                            robot.joints[joint].origin.xyz[1], 
                                            robot.joints[joint].origin.xyz[2]);
    let R_qn;

    let angle = robot.joints[joint].angle;
    let axis = robot.joints[joint].axis;
    R_qn = quaternion_to_rotation_matrix(quaternion_normalize(quaternion_from_axisangle(angle, axis))); 
    
    let parent_xform = robot.links[robot.joints[joint].parent].xform;
    let transform = matrix_multiply(trans, rotate);
    robot.joints[joint].xform = matrix_multiply(R_qn, matrix_multiply(parent_xform, transform));
    traverseFKLink(robot.joints[joint].child);
}


