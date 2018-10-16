//////////////////////////////////////////////////
/////     QUATERNION TRANSFORM ROUTINES 
//////////////////////////////////////////////////

    // STENCIL: reference quaternion code has the following functions:
    //   quaternion_from_axisangle
    //   quaternion_normalize
    //   quaternion_to_rotation_matrix
    //   quaternion_multiply

// returns a vector that is the quaternion from axisangle
function quaternion_from_axisangle(theta, axis) {
  let qx = axis[0] * Math.sin(theta / 2);
  let qy = axis[1] * Math.sin(theta / 2);
  let qz = axis[2] * Math.sin(theta / 2);
  let qw = Math.cos(theta / 2);
  return [qw, qx, qy, qz]; 
}

// returns a vector that is the normalization of quaternion
function quaternion_normalize(q) {
  let norm = 0;
  for(let i = 0; i < 4; ++i) norm += q[i] ** 2;
  for(let i = 0; i < 4; ++i) q[i] /= norm;
	return q;

}

// returns a rotation matrix transferred from quaternion
function quaternion_to_rotation_matrix(q) {
	return [[1-2*(q[2]**2 + q[3]**2), 2*(q[1]*q[2]-q[0]*q[3]), 2*(q[0]*q[2]+q[1]*q[3]),0],
          [2*(q[1]*q[2]+q[0]*q[3]), 1-2*(q[1]**2 + q[3]**2), 2*(q[2]*q[3]-q[0]*q[1]),0],
          [2*(q[1]*q[3]-q[0]*q[2]), 2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1]**2 + q[2]**2),0],
          [                      0,                       0,                       0,1]];
}

// returns a vector that is the multiplication of two quaternions
function quaternion_multiply(q1,q2) {
  let new_qi = q1[0]*q2[0]-q1[1]*q2[1]-q1[2]*q2[2]-q1[3]*q2[3];
  let new_qj = q1[0]*q2[1]+q1[1]*q2[0]+q1[2]*q2[3]-q1[3]*q2[2];
  let new_qk = q1[0]*q2[2]-q1[1]*q2[3]+q1[2]*q2[0]+q1[3]*q2[1];
  let new_qw = q1[0]*q2[3]+q1[1]*q2[2]-q1[2]*q2[1]+q1[3]*q2[0];

  return [new_qi, new_qj, new_qk, new_qw];
}


