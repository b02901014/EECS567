//////////////////////////////////////////////////
/////     MATRIX ALGEBRA AND GEOMETRIC TRANSFORMS 
//////////////////////////////////////////////////

function matrix_copy(m1) {
    // returns 2D array that is a copy of m1

    var mat = [];
    var i,j;

    for (i=0;i<m1.length;i++) { // for each row of m1
        mat[i] = [];
        for (j=0;j<m1[0].length;j++) { // for each column of m1
            mat[i][j] = m1[i][j];
        }
    }
    return mat;
}

    // STENCIL: reference matrix code has the following functions:
    //   matrix_multiply
    //   matrix_transpose
    //   matrix_pseudoinverse
    //   matrix_invert_affine
    //   vector_normalize
    //   vector_cross
    //   generate_identity
    //   generate_translation_matrix
    //   generate_rotation_matrix_X
    //   generate_rotation_matrix_Y
    //   generate_rotation_matrix_Z

// returns 2D array that is a multiplication of m1 and m2
function matrix_multiply(m1, m2) {
    
    let mat = [];
    for (let i = 0; i < m1.length; i++){
        mat[i] = [];
        for (let j = 0; j < m2[0].length; j++){
            mat[i][j] = 0;
            for (let k = 0; k < m2.length; k++)
               mat[i][j] += m1[i][k]*m2[k][j];
        }
    }

    return mat;
}

// returns 2D array that is the transpose of m1
function matrix_transpose(m1) {
    
    let mat = [];
    for (let i = 0; i < m1[0].length; i++){
        mat[i] = [];
        for (let j = 0; j < m1.length; j++)
           mat[i][j] = m1[j][i];
    }
    return mat;
}

// returns 2D array that is the pseudoinverse of m1
function matrix_pseudoinverse(m1) {
    let m1_t = matrix_transpose(m1);
    return matrix_multiply(numeric.inv(matrix_multiply(m1_t,m1)), m1_t); 
}

// returns 2D array that is the matrix_invert_affine of m1
function matrix_invert_affine(m1) {

}

// returns a vector that normalizes the input vector of v1
function vector_normalize(v1) {
    let sum = 0;
    let v1_norm = [];
    for (let i = 0; i < v1.length; i++)
        sum += v1[i] ** 2;
    sum = Math.sqrt(sum);
    for (let i = 0; i < v1.length; i++)
        v1_norm[i] = v1[i]/sum;
    return v1_norm;
}

// returns the cross product of v1 and v2
function vector_cross(v1, v2) {
    let v_cross = [];
    v_cross[0] = v1[1]*v2[2] - v1[2]*v2[1];
    v_cross[1] = v1[2]*v2[0] - v1[0]*v2[2];
    v_cross[2] = v1[0]*v2[1] - v1[1]*v2[0];
    return v_cross;
}

// returns a identity matrix with input size of n 
function generate_identity(n) {
    let mat = [];
    for (let i = 0; i < n; i++){
        mat[i] = [];
        for (let j = 0; j < n; j++){
            if (i == j) mat[i][j] = 1;
            else mat[i][j] = 0; 
        }

    }
    return mat;
}

// returns the translation matrix with input of tx, ty and tz
function generate_translation_matrix(tx, ty, tz) {
    let mat = generate_identity(4);
    mat[0][3] = tx;
    mat[1][3] = ty;
    mat[2][3] = tz;
    return mat;
}

// returns the rotation matrix about x-axis with theta
function generate_rotation_matrix_X(theta) {
    let mat = generate_identity(4);
    mat[1][1] = Math.cos(theta);
    mat[1][2] = -Math.sin(theta);
    mat[2][1] = Math.sin(theta);
    mat[2][2] = Math.cos(theta);
    return mat;
}

// returns the rotation matrix about y-axis with theta
function generate_rotation_matrix_Y(theta) {
    let mat = generate_identity(4);
    mat[0][0] = Math.cos(theta);
    mat[0][2] = Math.sin(theta);
    mat[2][0] = -Math.sin(theta);
    mat[2][2] = Math.cos(theta);
    return mat;
}

// returns the rotation matrix about z-axis with theta
function generate_rotation_matrix_Z(theta) {
    let mat = generate_identity(4);
    mat[0][0] = Math.cos(theta);
    mat[0][1] = -Math.sin(theta);
    mat[1][0] = Math.sin(theta);
    mat[1][1] = Math.cos(theta);
    return mat;
}




