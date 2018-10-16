
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
    var i;
    for (i = 0; i < robot.links[robot.base].children.length; i++){
        traverseFKJoint(robot.links[robot.base].children[i]);
    }
    
}

function traverseFKBase() {
    var R = matrix_multiply(matrix_multiply(generate_rotation_matrix_Z(robot.origin.rpy[2]),generate_rotation_matrix_Y(robot.origin.rpy[1])),generate_rotation_matrix_X(robot.origin.rpy[0]));
    var D = generate_translation_matrix(robot.origin.xyz[0], robot.origin.xyz[1], robot.origin.xyz[2]);
    robot.links[robot.base].xform = matrix_multiply(D,R);

    var local_headingZ = [[0],[0],[1],[1]];
    var local_lateralX = [[1],[0],[0],[1]];
    robot_heading = matrix_multiply(robot.links[robot.base].xform,local_headingZ);
    robot_lateral = matrix_multiply(robot.links[robot.base].xform,local_lateralX);
    
    if (robot.links_geom_imported) {
        var offset_xform = matrix_multiply(generate_rotation_matrix_Y(-Math.PI/2),generate_rotation_matrix_X(-Math.PI/2));
        robot.links[robot.base].xform = matrix_multiply(robot.links[robot.base].xform, offset_xform);
    }
    
    
}

function traverseFKLink(link) {
    robot.links[link].xform = robot.joints[robot.links[link].parent].xform;
    if (typeof robot.links[link].children === 'undefined'){
        return;
    }
    var i;
    for (i = 0; i < robot.links[link].children.length; i++){
        traverseFKJoint(robot.links[link].children[i]);
    }
}

function traverseFKJoint(joint) {
    var R = matrix_multiply(matrix_multiply(generate_rotation_matrix_Z(robot.joints[joint].origin.rpy[2]),generate_rotation_matrix_Y(robot.joints[joint].origin.rpy[1])),generate_rotation_matrix_X(robot.joints[joint].origin.rpy[0]));
    var D = generate_translation_matrix(robot.joints[joint].origin.xyz[0], robot.joints[joint].origin.xyz[1], robot.joints[joint].origin.xyz[2]);
    var R_qn;
    if (robot.links_geom_imported){
        if (robot.joints[joint].type === "revolute" ||  robot.joints[joint].type === "continuous"){
            R_qn = quaternion_to_rotation_matrix(quaternion_normalize(quaternion_from_axisangle(robot.joints[joint].angle,robot.joints[joint].axis)));
        }
        else if (robot.joints[joint].type === "prismatic"){
            R_qn = generate_translation_matrix(robot.joints[joint].angle*robot.joints[joint].axis[0],robot.joints[joint].angle*robot.joints[joint].axis[1],robot.joints[joint].angle*robot.joints[joint].axis[2]);
        }
        else{
            R_qn = generate_identity(4);
        }
    }
    else{
        R_qn = quaternion_to_rotation_matrix(quaternion_normalize(quaternion_from_axisangle(robot.joints[joint].angle,robot.joints[joint].axis))); 
    }
    
    
    robot.joints[joint].xform = matrix_multiply(matrix_multiply(robot.links[robot.joints[joint].parent].xform,matrix_multiply(D,R)), R_qn);
    traverseFKLink(robot.joints[joint].child);
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
    //   if (robot.links_geom_imported) {
    //       var offset_xform = matrix_multiply(generate_rotation_matrix_Y(-Math.PI/2),generate_rotation_matrix_X(-Math.PI/2));

