//////////////////////////////////////////////////
/////     QUATERNION TRANSFORM ROUTINES 
//////////////////////////////////////////////////

    // STENCIL: reference quaternion code has the following functions:
    //   quaternion_from_axisangle
    //   quaternion_normalize
    //   quaternion_to_rotation_matrix
    //   quaternion_multiply

// returns a vector that is the quaternion from axisangle
function quaternion_from_axisangle(theta, axis_normlized) {
	return [Math.cos(theta/2), Math.sin(theta/2)*axis_normlized[0], Math.sin(theta/2)*axis_normlized[1], Math.sin(theta/2)*axis_normlized[2]];
}

// returns a vector that is the normalization of quaternion
function quaternion_normalize(q) {
	var norm = Math.sqrt(Math.pow(q[0],2)+Math.pow(q[1],2)+Math.pow(q[2],2)+Math.pow(q[3],2));
	return [q[0]/norm, q[1]/norm, q[2]/norm, q[3]/norm];

}

// returns a rotation matrix transferred from quaternion
function quaternion_to_rotation_matrix(q) {
	return [
                [1-2*(Math.pow(q[2],2)+Math.pow(q[3],2)), 2*(q[1]*q[2]-q[0]*q[3]), 2*(q[0]*q[2]+q[1]*q[3]),0],
                [2*(q[1]*q[2]+q[0]*q[3]), 1-2*(Math.pow(q[1],2)+Math.pow(q[3],2)), 2*(q[2]*q[3]-q[0]*q[1]),0],
                [2*(q[1]*q[3]-q[0]*q[2]), 2*(q[0]*q[1]+q[2]*q[3]), 1-2*(Math.pow(q[1],2)+Math.pow(q[2],2)),0],
                [0,0,0,1]
    ];
}

// returns a vector that is the multiplication of two quaternions
function quaternion_multiply(q1,q2) {
	return [q1[0]*q2[0]-q1[1]*q2[1]-q1[2]*q2[2]-q1[3]*q2[3], q1[0]*q2[1]+q1[1]*q2[0]+q1[2]*q2[3]-q1[3]*q2[2], q1[0]*q2[2]-q1[1]*q2[3]+q1[2]*q2[0]+q1[3]*q2[1], q1[0]*q2[3]+q1[1]*q2[2]-q1[2]*q2[1]+q1[3]*q2[0]];
}


