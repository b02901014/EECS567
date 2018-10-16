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
    
    var mat = [];
    var i,j,k;
    for (i = 0; i < m1.length; i++){
        mat[i] = [];
        for (j = 0; j < m2[0].length; j++){
            mat[i][j] = 0;
            for (k = 0; k < m2.length; k++){
               mat[i][j] += m1[i][k]*m2[k][j];
            }
        }
    }

    return mat;
}

// returns 2D array that is the transpose of m1
function matrix_transpose(m1) {
    
    var mat = [];
    var i,j;

    for (i = 0; i < m1[0].length; i++){
        mat[i] = [];
        for (j = 0; j < m1.length; j++){
           mat[i][j] = m1[j][i];
        }
    }
    return mat;
}

// returns 2D array that is the pseudoinverse of m1
function matrix_pseudoinverse(m1) {
    
    var mat = [];
    var i,j;
    if (m1.length == m1[0].length){
        return numeric.inv(m1);
    }
    else{
        var m1_T = matrix_transpose(m1);
        if (m1.length > m1[0].length){
            return matrix_multiply(numeric.inv(matrix_multiply(m1_T,m1)), m1_T);
        }
        else{
            return matrix_multiply(m1_T, numeric.inv(matrix_multiply(m1,m1_T)));
        }
        
    }
}

// returns a struct that is the LU decomposition matrix of m1
function LU_decomposition(m1) {
    var n = m1.length;
    var i,j,k;
    var l_matrix = [];
    var u_matrix = [];
    for (i = 0; i < n; i++){
        l_matrix[i] = [];
        u_matrix[i] = [];
    }
    for (i = 0; i < n; i++){
        for (j = 0; j < n; j++){
            if (j < i){
                l_matrix[j][i] = 0;
            }
            else{
                l_matrix[j][i] = m1[j][i];
                for (k = 0; k < i; k++){
                    l_matrix[j][i] = l_matrix[j][i] - l_matrix[j][k] * u_matrix[k][i];
                }
            }
        }
        for (j = 0; j < n; j++){
            if (j == i){
                u_matrix[i][j] = 1;
            }
            else if (j < i){
                u_matrix[i][j] = 0;
            }
            else{
                u_matrix[i][j] = m1[i][j] / l_matrix[i][i];
                for (k = 0; k < i; k++){
                    u_matrix[i][j] = u_matrix[i][j] - l_matrix[i][k] * u_matrix[k][j] / l_matrix[i][i];
                }
            }
        }
    }
    return {
        l_matrix: l_matrix, 
        u_matrix: u_matrix
    }; 
}

// returns a vector that is linear solution of m1 with b1
function linear_solved(m1, b1) {
    var x_solution = [];
    var y_partial = [];
    var i, j;
    var n = m1.length;
    var LU_matrix = LU_decomposition(m1);
    var l_matrix = LU_matrix.l_matrix;
    var u_matrix = LU_matrix.u_matrix;
    for (i = 0; i < n; i++){
        y_partial[i] = b1[i]/l_matrix[i][i];
        if (i != 0){
            for (j = 0; j < i; j++){
                y_partial[i] -= l_matrix[i][j]*y_partial[j]/l_matrix[i][i];
            }   
        }
    }
    for (i = n-1; i >= 0; i--){
        x_solution[i] = y_partial[i]/u_matrix[i][i];
        if (i != n-1){
            for (j = i+1; j < n; j++){
                x_solution[i] -= u_matrix[i][j]*x_solution[j]/u_matrix[i][i];
            }   
        }
    }
    return x_solution;
}

// returns a 2D array that is the inverse of m1 
function matrix_inverse(m1) {
    var iden_matrix = generate_identity(m1.length);
    var m1_inv = [[],[]];
    var i;
    for (i = 0; i < m1.length; i++){
        m1_inv[i] = linear_solved(m1, iden_matrix[i]);
    }
    return matrix_transpose(m1_inv);
}

// returns 2D array that is the matrix_invert_affine of m1
function matrix_invert_affine(m1) {

}

// returns a vector that normalizes the input vector of v1
function vector_normalize(v1) {
    var sum = 0;
    var v1_norm = [];
    var i;
    for (i = 0; i < v1.length; i++){
        sum += Math.pow(v1[i], 2);
    }
    sum = Math.sqrt(sum);
    for (i = 0; i < v1.length; i++){
        v1_norm[i] = v1[i]/sum;
    }
    return v1_norm;
}

// returns the cross product of v1 and v2
function vector_cross(v1, v2) {
    var v_cross = [];
    v_cross[0] = v1[1]*v2[2] - v1[2]*v2[1];
    v_cross[1] = v1[2]*v2[0] - v1[0]*v2[2];
    v_cross[2] = v1[0]*v2[1] - v1[1]*v2[0];
    return v_cross;
}

// returns a identity matrix with input size of n 
function generate_identity(n) {
    var mat = [];
    var i,j;
    for (i = 0; i < n; i++){
        mat[i] = [];
        for (j = 0; j < n; j++){
            if (i == j){
                mat[i][j] = 1;
            }
            else{
                mat[i][j] = 0; 
            }
        }

    }
    return mat;
}

// returns the translation matrix with input of tx, ty and tz
function generate_translation_matrix(tx, ty, tz) {
    var mat = generate_identity(4);
    mat[0][3] = tx;
    mat[1][3] = ty;
    mat[2][3] = tz;
    return mat;
}

// returns the rotation matrix about x-axis with theta
function generate_rotation_matrix_X(theta) {
    var mat = generate_identity(4);
    mat[1][1] = Math.cos(theta);
    mat[1][2] = -Math.sin(theta);
    mat[2][1] = Math.sin(theta);
    mat[2][2] = Math.cos(theta);
    return mat;
}

// returns the rotation matrix about y-axis with theta
function generate_rotation_matrix_Y(theta) {
    var mat = generate_identity(4);
    mat[0][0] = Math.cos(theta);
    mat[0][2] = Math.sin(theta);
    mat[2][0] = -Math.sin(theta);
    mat[2][2] = Math.cos(theta);
    return mat;
}

// returns the rotation matrix about z-axis with theta
function generate_rotation_matrix_Z(theta) {
    var mat = generate_identity(4);
    mat[0][0] = Math.cos(theta);
    mat[0][1] = -Math.sin(theta);
    mat[1][0] = Math.sin(theta);
    mat[1][1] = Math.cos(theta);
    return mat;
}




