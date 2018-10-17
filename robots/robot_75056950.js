//   CREATE ROBOT STRUCTURE

// KE 

links_geom_imported = false;

//////////////////////////////////////////////////
/////     DEFINE ROBOT AND LINKS
//////////////////////////////////////////////////

// create robot data object
robot = new Object(); // or just {} will create new object

// give the robot a name
robot.name = "frankwang";

// initialize start pose of robot in the world
robot.origin = {xyz: [0,0,0], rpy:[0,0,0]};  

// specify base link of the robot; robot.origin is transform of world to the robot base
robot.base = "base";  

        
// specify and create data objects for the links of the robot
robot.links = {
    "base": {},  
    "clavicle_right": {}, 
    "clavicle_left": {} , 
    "shoulder_right": {}, 
    "upperarm_right": {}, 
    "forearm_right": {},
    "head":{},

    "leg1_upper": {}, 
    "leg1_middle": {}, 
    "leg1_lower": {}, 
  
    "leg2_upper": {}, 
    "leg2_middle": {}, 
    "leg2_lower": {},
  
    "leg3_upper": {}, 
    "leg3_middle": {}, 
    "leg3_lower": {}, 
  
    "leg4_upper": {}, 
    "leg4_middle": {}, 
    "leg4_lower": {}, 
};
/* for you to do
, "shoulder_left": {}  , "upperarm_left": {} , "forearm_left": {} };
*/

//////////////////////////////////////////////////
/////     DEFINE JOINTS AND KINEMATIC HIERARCHY
//////////////////////////////////////////////////

/*      joint definition template
        // specify parent/inboard link and child/outboard link
        robot.joints.joint1 = {parent:"link1", child:"link2"};
        // joint origin's offset transform from parent link origin
        robot.joints.joint1.origin = {xyz: [5,3,0], rpy:[0,0,0]}; 
        // joint rotation axis
        robot.joints.joint1.axis = [0.0,0.0,1.0]; 
*/


// roll-pitch-yaw defined by ROS as corresponding to x-y-z 
//http://wiki.ros.org/urdf/Tutorials/Create%20your%20own%20urdf%20file

// specify and create data objects for the joints of the robot
robot.joints = {};

robot.joints.clavicle_right_yaw = {parent:"base", child:"clavicle_right"};
robot.joints.clavicle_right_yaw.origin = {xyz: [0.3,0.4,0.0], rpy:[-Math.PI/2,0,0]};
robot.joints.clavicle_right_yaw.axis = [0.0,0.0,-1.0]; 

robot.joints.shoulder_right_yaw = {parent:"clavicle_right", child:"shoulder_right"};
robot.joints.shoulder_right_yaw.origin = {xyz: [0.0,-0.15,0.85], rpy:[Math.PI/2,0,0]};
robot.joints.shoulder_right_yaw.axis = [0.0,0.707,0.707]; 

robot.joints.upperarm_right_pitch = {parent:"shoulder_right", child:"upperarm_right"};
robot.joints.upperarm_right_pitch.origin = {xyz: [0.0,0.0,0.7], rpy:[0,0,0]};
robot.joints.upperarm_right_pitch.axis = [0.0,1.0,0.0]; 

robot.joints.forearm_right_yaw = {parent:"upperarm_right", child:"forearm_right"};
robot.joints.forearm_right_yaw.origin = {xyz: [0.0,0.0,0.7], rpy:[0,0,0]};
robot.joints.forearm_right_yaw.axis = [1.0,0.0,0.0]; 

robot.joints.clavicle_left_roll = {parent:"base", child:"clavicle_left"};
robot.joints.clavicle_left_roll.origin = {xyz: [-0.3,0.4,0.0], rpy:[-Math.PI/2,0,0]};
robot.joints.clavicle_left_roll.axis = [0.0,0.0,1.0]; 

robot.joints.head_roll = {parent:"clavicle_left", child:"head"};
robot.joints.head_roll.origin = {xyz: [0.7,0.4,0.0], rpy:[0,0,0]};
robot.joints.head_roll.axis = [0.0,0.0,1.0]; 

//leg1
robot.joints.leg1_hip = {parent:"base", child:"leg1_upper"};
robot.joints.leg1_hip.origin = {xyz: [0.3,0.0,0.9], rpy:[0,Math.PI/4,0]};
robot.joints.leg1_hip.axis = [0.0,1.0,0.0]; 

robot.joints.leg1_knee = {parent:"leg1_upper", child:"leg1_middle"};
robot.joints.leg1_knee.origin = {xyz: [0.0,0.0,0.4], rpy:[-Math.PI/4,0,0]};
robot.joints.leg1_knee.axis = [1.0,0.0,0.0]; 

robot.joints.leg1_ankle = {parent:"leg1_middle", child:"leg1_lower"};
robot.joints.leg1_ankle.origin = {xyz: [0.0,0.0,0.6], rpy:[-0.25 + Math.PI/2,0,0]};
robot.joints.leg1_ankle.axis = [1.0,0.0,0.0]; 

//leg2
robot.joints.leg2_hip = {parent:"base", child:"leg2_upper"};
robot.joints.leg2_hip.origin = {xyz: [-0.4,0.0,0.8], rpy:[0,-Math.PI/4,0]};
robot.joints.leg2_hip.axis = [0.0,1.0,0.0]; 

robot.joints.leg2_knee = {parent:"leg2_upper", child:"leg2_middle"};
robot.joints.leg2_knee.origin = {xyz: [0.0,0.0,0.4], rpy:[-Math.PI/4,0,0]};
robot.joints.leg2_knee.axis = [1.0,0.0,0.0]; 

robot.joints.leg2_ankle = {parent:"leg2_middle", child:"leg2_lower"};
robot.joints.leg2_ankle.origin = {xyz: [0.0,0.0,0.6], rpy:[-0.25 + Math.PI/2,0,0]};
robot.joints.leg2_ankle.axis = [1.0,0.0,0.0]; 

//leg3
robot.joints.leg3_hip = {parent:"base", child:"leg3_upper"};
robot.joints.leg3_hip.origin = {xyz: [0.45,0.0,-0.7], rpy:[0,Math.PI*3/4,0]};
robot.joints.leg3_hip.axis = [0.0,1.0,0.0]; 

robot.joints.leg3_knee = {parent:"leg3_upper", child:"leg3_middle"};
robot.joints.leg3_knee.origin = {xyz: [0.0,0.0,0.4], rpy:[-Math.PI/4,0,0]};
robot.joints.leg3_knee.axis = [1.0,0.0,0.0]; 

robot.joints.leg3_ankle = {parent:"leg3_middle", child:"leg3_lower"};
robot.joints.leg3_ankle.origin = {xyz: [0.0,0.0,0.6], rpy:[-0.25 + Math.PI/2,0,0]};
robot.joints.leg3_ankle.axis = [1.0,0.0,0.0]; 

//leg4
robot.joints.leg4_hip = {parent:"base", child:"leg4_upper"};
robot.joints.leg4_hip.origin = {xyz: [-0.4,0.0,-0.9], rpy:[0,-Math.PI*3/4,0]};
robot.joints.leg4_hip.axis = [0.0,1.0,0.0]; 

robot.joints.leg4_knee = {parent:"leg4_upper", child:"leg4_middle"};
robot.joints.leg4_knee.origin = {xyz: [0.0,0.0,0.4], rpy:[-Math.PI/4,0,0]};
robot.joints.leg4_knee.axis = [1.0,0.0,0.0]; 

robot.joints.leg4_ankle = {parent:"leg4_middle", child:"leg4_lower"};
robot.joints.leg4_ankle.origin = {xyz: [0.0,0.0,0.6], rpy:[-0.25 + Math.PI/2,0,0]};
robot.joints.leg4_ankle.axis = [1.0,0.0,0.0]; 
// specify name of endeffector frame
robot.endeffector = {};
robot.endeffector.frame = "forearm_right_yaw";
robot.endeffector.position = [[0],[0],[0.5],[1]]

//////////////////////////////////////////////////
/////     DEFINE LINK threejs GEOMETRIES
//////////////////////////////////////////////////

/*  threejs geometry definition template, will be used by THREE.Mesh() to create threejs object
    // create threejs geometry and insert into links_geom data object
    links_geom["link1"] = new THREE.CubeGeometry( 5+2, 2, 2 );

    // example of translating geometry (in object space)
    links_geom["link1"].applyMatrix( new THREE.Matrix4().makeTranslation(5/2, 0, 0) );

    // example of rotating geometry 45 degrees about y-axis (in object space)
    var temp3axis = new THREE.Vector3(0,1,0);
    links_geom["link1"].rotateOnAxis(temp3axis,Math.PI/4);
*/

// define threejs geometries and associate with robot links 
links_geom = {};

links_geom["base"] = new THREE.CubeGeometry( 1, 0.4, 2);
links_geom["base"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0.2, 0) );

links_geom["clavicle_right"] = new THREE.CubeGeometry( 0.3, 0.3, 1 );
links_geom["clavicle_right"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.5) );

links_geom["clavicle_left"] = new THREE.CubeGeometry( 0.3, 0.3, 1 );
links_geom["clavicle_left"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.5) );

links_geom["shoulder_right"] = new THREE.CubeGeometry( 0.3, 0.3, 0.7 );
links_geom["shoulder_right"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.35) );

links_geom["upperarm_right"] = new THREE.CubeGeometry( 0.3, 0.3, 0.7 );
links_geom["upperarm_right"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.35) );

links_geom["forearm_right"] = new THREE.CubeGeometry( 0.3, 0.3, 0.5 );
links_geom["forearm_right"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.25) );

links_geom["head"] = new THREE.CubeGeometry( 0.7, 0.7, 0.7 );
links_geom["head"].applyMatrix( new THREE.Matrix4().makeTranslation(-0.7, -0.35, 1.35) );

links_geom["leg1_upper"] = new THREE.CubeGeometry( 0.3, 0.3, 0.3 );
links_geom["leg1_upper"].applyMatrix( new THREE.Matrix4().makeTranslation(0.1, 0.2, 0.15) );

links_geom["leg1_middle"] = new THREE.CubeGeometry( 0.3, 0.3, 0.6 );
links_geom["leg1_middle"].applyMatrix( new THREE.Matrix4().makeTranslation(0.1, 0.2, 0.3) );

links_geom["leg1_lower"] = new THREE.CubeGeometry( 0.3, 0.3, 1 );
links_geom["leg1_lower"].applyMatrix( new THREE.Matrix4().makeTranslation(0.1, 0.15, 0.4) );

links_geom["leg2_upper"] = new THREE.CubeGeometry( 0.3, 0.3, 0.3 );
links_geom["leg2_upper"].applyMatrix( new THREE.Matrix4().makeTranslation(0.1, 0.2, 0.15) );

links_geom["leg2_middle"] = new THREE.CubeGeometry( 0.3, 0.3, 0.6 );
links_geom["leg2_middle"].applyMatrix( new THREE.Matrix4().makeTranslation(0.1, 0.2, 0.3) );

links_geom["leg2_lower"] = new THREE.CubeGeometry( 0.3, 0.3, 1 );
links_geom["leg2_lower"].applyMatrix( new THREE.Matrix4().makeTranslation(0.1, 0.15, 0.4) );

links_geom["leg3_upper"] = new THREE.CubeGeometry( 0.3, 0.3, 0.3 );
links_geom["leg3_upper"].applyMatrix( new THREE.Matrix4().makeTranslation(0.1, 0.2, 0.15) );

links_geom["leg3_middle"] = new THREE.CubeGeometry( 0.3, 0.3, 0.6 );
links_geom["leg3_middle"].applyMatrix( new THREE.Matrix4().makeTranslation(0.1, 0.2, 0.3) );

links_geom["leg3_lower"] = new THREE.CubeGeometry( 0.3, 0.3, 1 );
links_geom["leg3_lower"].applyMatrix( new THREE.Matrix4().makeTranslation(0.1, 0.15, 0.4) );

links_geom["leg4_upper"] = new THREE.CubeGeometry( 0.3, 0.3, 0.3 );
links_geom["leg4_upper"].applyMatrix( new THREE.Matrix4().makeTranslation(0.1, 0.2, 0.15) );

links_geom["leg4_middle"] = new THREE.CubeGeometry( 0.3, 0.3, 0.6 );
links_geom["leg4_middle"].applyMatrix( new THREE.Matrix4().makeTranslation(0.1, 0.2, 0.3) );

links_geom["leg4_lower"] = new THREE.CubeGeometry( 0.3, 0.3, 1 );
links_geom["leg4_lower"].applyMatrix( new THREE.Matrix4().makeTranslation(0.1, 0.15, 0.4) );
