
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | inverse kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotInverseKinematics = function robot_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // compute joint angle controls to move location on specified link to Cartesian location
    if ((kineval.params.update_ik)||(kineval.params.persist_ik)) { 
        // if update requested, call ik iterator and show endeffector and target
        kineval.iterateIK(endeffector_target_world, endeffector_joint, endeffector_position_local);
        if (kineval.params.trial_ik_random.execute)
            kineval.randomizeIKtrial();
        else // KE: this use of start time assumes IK is invoked before trial
            kineval.params.trial_ik_random.start = new Date();
    }

    kineval.params.update_ik = false; // clear IK request for next iteration
}

kineval.randomizeIKtrial = function randomIKtrial () {

    // update time from start of trial
    cur_time = new Date();
    kineval.params.trial_ik_random.time = cur_time.getTime()-kineval.params.trial_ik_random.start.getTime();

    // STENCIL: see instructor for random time trial code
    endeffector_world = matrix_multiply(robot.joints[robot.endeffector.frame].xform,
                                        robot.endeffector.position);

    // compute distance of endeffector to target
    kineval.params.trial_ik_random.distance_current = Math.sqrt(
           (kineval.params.ik_target.position[0][0]-endeffector_world[0][0])**2
           +(kineval.params.ik_target.position[1][0]-endeffector_world[1][0])**2
           +(kineval.params.ik_target.position[2][0]-endeffector_world[2][0])**2);

    if (kineval.params.trial_ik_random.distance_current < 0.01) {
       kineval.params.ik_target.position[0][0] = 1.2 * (Math.random() - 0.5);
       kineval.params.ik_target.position[1][0] = 1.2 * (Math.random() - 0.5) + 1.5;
       kineval.params.ik_target.position[2][0] = 0.7 * (Math.random() - 0.5) + 0.5;
       kineval.params.trial_ik_random.targets += 1;
      textbar.innerHTML = "Target reached: " + 
        kineval.params.trial_ik_random.targets + " reached at time " + 
        kineval.params.trial_ik_random.time / 1000.0 + "sec";
    }
}

kineval.iterateIK = function iterate_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

  // STENCIL: implement inverse kinematics iteration
  //Step1: Generate delta x
  //console.log("IK start");
  let trans_end2world = robot.joints[endeffector_joint].xform;
  let world_pos = matrix_multiply(trans_end2world, endeffector_position_local);
  let world_r = Math.atan2(trans_end2world[2][1], trans_end2world[2][2]);
  let world_p = Math.atan2(-trans_end2world[2][0],
                           Math.sqrt(trans_end2world[2][1]**2 + trans_end2world[2][2]**2));
  let world_y = Math.atan2(trans_end2world[1][0], trans_end2world[0][0]);
  let world_angle = [world_r, world_p, world_y];

  //console.log("cal delta_x");
  let delta_x = [];
  for(let i = 0; i < 6; ++i){
    delta_x[i] = [];
    if(i < 3) delta_x[i][0] = endeffector_target_world.position[i] - world_pos[i][0];
    else{
      if(kineval.params.ik_orientation_included)
        delta_x[i][0] = endeffector_target_world.orientation[i-3] - world_angle[i];
      else delta_x[i][0] = 0;
    }
  }
  //console.log(delta_x);
  //Step2: Generate Jacobian
  //prismatic: [Z(i-1), 0] / rotational: [Z(i-1)x(o(n)-o(i-1)); z(i-1)]
  //o(n) for endeffector and o(i-1) for joint origin
  //console.log("cal jacob");
  let jacob = [];
  let world_joint_axis = []; //Z
  let curr_joint = endeffector_joint;
  let count = 0;
  while(1){
    //console.log("curr_joint",curr_joint, robot.joints[curr_joint]);
    let axis_trans = [];
    for(let i = 0; i < 3; ++i){
      axis_trans[i] = [];
      for(let j = 0; j < 3; ++j){
        axis_trans[i][j] = robot.joints[curr_joint].xform[i][j];
      }
    }
    let axis = [[robot.joints[curr_joint].axis[0]],
                [robot.joints[curr_joint].axis[1]],
                [robot.joints[curr_joint].axis[2]]];
    world_joint_axis = matrix_multiply(axis_trans, axis);
    world_joint_axis = [world_joint_axis[0][0], world_joint_axis[1][0], world_joint_axis[2][0]];
    world_joint_axis = vector_normalize(world_joint_axis);
    if(robot.joints[curr_joint].type == "prismatic")
      jacob[count] = [world_joint_axis[0], world_joint_axis[1], world_joint_axis[2], 0, 0, 0];
    else{
      let world_joint_origin = matrix_multiply(robot.joints[curr_joint].xform, [[0], [0], [0], [1]]); 
      world_joint_origin = [world_joint_origin[0][0], world_joint_origin[1][0], world_joint_origin[2][0]];
      let origin_diff = [world_pos[0][0] - world_joint_origin[0],
                         world_pos[1][0] - world_joint_origin[1],
                         world_pos[2][0] - world_joint_origin[2]];
      let origin_cross = vector_cross(world_joint_axis, origin_diff);
      jacob[count] = [origin_cross[0], origin_cross[1], origin_cross[2],
                      world_joint_axis[0], world_joint_axis[1], world_joint_axis[2]];
    }

    if(robot.joints[curr_joint].parent == robot.base){
      //console.log("break");
      break;
    }
    count += 1;
    curr_joint = robot.links[robot.joints[curr_joint].parent].parent;
  }
  //console.log(jacob);


  //generate state control
  //console.log("state control");
  state_control = [];
  if(kineval.params.ik_pseudoinverse){
    pseudo_inv = matrix_pseudoinverse(matrix_transpose(jacob));
    state_control = matrix_multiply(pseudo_inv, delta_x)
  }
  else{
    state_control = matrix_multiply(jacob, delta_x);
  }
  
  curr_joint = endeffector_joint;
  count = 0;
  while(1){
    robot.joints[curr_joint].control += kineval.params.ik_steplength * state_control[count][0];
    if(robot.joints[curr_joint].parent == robot.base) break;
    count += 1;
    curr_joint = robot.links[robot.joints[curr_joint].parent].parent;
  }
  //console.log(state_control);

}



