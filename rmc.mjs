// rmc - resolved momentum control

// ECMAScript module

//npm install https://github.com/PeterTadich/camera-perspective https://github.com/PeterTadich/javascript-data-structures https://github.com/PeterTadich/lu-decomposition https://github.com/PeterTadich/singular-value-decomposition https://github.com/PeterTadich/matlab-javascript https://github.com/PeterTadich/elementary-rotations https://github.com/PeterTadich/homogeneous-transformations https://github.com/PeterTadich/matrix-computations

// Resolved Momentum Control.
// ref: Resolved Momentum Control: Humanoid Motion Planning based on the Linear and Angular Momentum (KAJITA).

// Robot Model:
//   - Hovis lite
//   - build using function buildRobotTu('left'), see generate_graph() in file transformGraph.js and deleteME13walk3.js
//   - add on frame for end effector

// Required input:
//   - Pref_x,y,z eq. (27,28)
//      - c_bar_ref the target position of total CoM
//      - c_dot_bar_ref the target speed of the total CoM
//      - zB the target height of the pelvis
//   - Lref eq. (29)
//      - 0(3 x 1)

// To do:
//   - add 'pb' (T0baseLink) to the robot object (needed in 'theAmatrix')

import * as hlao from 'matrix-computations';
import * as mcht from 'homogeneous-transformations';
import * as mtojs from 'matlab-javascript';
import * as mcer from 'elementary-rotations';
import * as ludcmp from 'lu-decomposition';
import * as svdcmp from 'singular-value-decomposition';
import * as jds from 'javascript-data-structures';
import * as cvp from 'camera-perspective';
/*
import * as hlao from '../matrix-computations/hlao.mjs';
import * as mcht from '../homogeneous-transformations/mcht.mjs';
import * as mtojs from '../matlab-javascript/mtojs.mjs';
import * as mcer from '../elementary-rotations/mcer.mjs';
import * as ludcmp from '../lu-decomposition/ludcmp.mjs';
import * as svdcmp from '../singular-value-decomposition/svdcmp.mjs';
import * as jds from '../javascript-data-structures/jds.mjs';
import * as cvp from '../camera-perspective/cvp.mjs';
*/

var T_BaseFrame = [
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0]
];

//skew_symmetric_matrix
function S(X){
    var S = [
        [         0.0, -1.0*X[2][0],      X[1][0]],
        [     X[2][0],          0.0, -1.0*X[0][0]],
        [-1.0*X[1][0],      X[0][0],          0.0]
    ];
    
    return S;
}

var quad = {};

// RMC
//   - see Resolved Momentum Control: Humanoid Motion Planning based on the Linear and Angular Momentum
function RMC(){
    if(typeof quad.timer == 'undefined'){
        //   - initialize timer, servo ID, robot object
        quad.timer = setInterval(RMC, 100);
        quad.index = 0;
    
        //Load the 'Tu' robot into a graph data structure (see file deleteME13walk3.js).
        jds.generate_graph(); //(transformGraph.js) buildRobotTu('left') - "LinksRightToLeft" (see function in Tu_DHparameters.js)
        jds.init_queue();
        jds.initialize_search();
        
        //build parents array 'parents[]' which is declared in transformGraph.js
        jds.bfs(1);
        
        //body reference frame
        //   - homogeneous transformation matrix of the 'base link'
        var nodePath = [50, 1]; //N50 to N1 (inertial frame to body frame (base link))
        var tmData = jds.TM(nodePath,T_BaseFrame); //returns [htma, T0, T0n]
        quad.T0baseLink = tmData[2];
        console.log("'base link': " + quad.T0baseLink);
        //   - 'base link' position vector
        var pb = [
            [quad.T0baseLink[0][3]], //x
            [quad.T0baseLink[1][3]], //y
            [quad.T0baseLink[2][3]]  //z
        ];
        console.log("'base link' position vector: " + quad.T0baseLink);
        //plot base link frame
        cvp.setColour('cyan');
        cvp.plotFrame(cvp.camera(quad.T0baseLink));
        cvp.setColour('black');

        //define the four open links
        //   - define the body side transform (to inertial frame) for each open link
        //      - define T02 (z2 defined in inertial frame)
        nodePath = [1, 2]; //z1 to z2
        var tmData = jds.TM(nodePath,T_BaseFrame); //returns [htma, T0, T0n]
        var T02 = tmData[2];
        console.log(T02);
        
        //      - define T06 (z6 defined in inertial frame)
        nodePath = [1, 6]; //z1 to z6
        var tmData = jds.TM(nodePath,T_BaseFrame); //returns [htma, T0, T0n]
        var T06 = tmData[2];
        console.log(T06);
        
        //      - define T010 (z10 defined in inertial frame)
        nodePath = [1, 10]; //z1 to z10
        var tmData = jds.TM(nodePath,T_BaseFrame); //returns [htma, T0, T0n]
        var T010 = tmData[2];
        console.log(T010);
        
        //      - define T014 (z14 defined in inertial frame)
        nodePath = [1, 14]; //z1 to z14
        var tmData = jds.TM(nodePath,T_BaseFrame); //returns [htma, T0, T0n]
        var T014 = tmData[2];
        console.log(T014);
        
        //   - make a copy of the T_BaseFrame
        quad.T_BaseFrame_COPY = mcht.DHDeepCopy(T_BaseFrame);
        
        //   - leg 1 end-effector
        //      - node path: node 2 (z2) to node 5 (z5)
        //jds.path = [];
        jds.resetPath();
        jds.find_path(2,5); //Array [ 2, 3, 4, 5 ]
        console.log(jds.path);
        T_BaseFrame = mcht.DHDeepCopy(T02); //setup jds.TM() function as it uses 'T_BaseFrame'
        var tmData = jds.TM(jds.path,T_BaseFrame); //returns [htma, T0, T0n]
        var T0_leg1 = [];
        for(var i=0;i<tmData[1].length;i=i+1){
            T0_leg1.push(mcht.DHDeepCopy(tmData[1][i])); //get T0
        }
        T_BaseFrame = mcht.DHDeepCopy(quad.T_BaseFrame_COPY); //'T_BaseFrame' back to original setting
        //      - add T02 to the start of the 'T0' to draw link 1.
        T0_leg1.unshift(T02);
        //      - plot robot
        cvp.drawRobotUsingT0(T0_leg1,'pink'); //see drawRobotSkeleton.js
        console.log(T0_leg1);
        
        //   - leg 2 end-effector
        //      - node path: node 6 (z6) to node 9 (z9)
        //jds.path = [];
        jds.resetPath();
        jds.find_path(6,9); //Array [ 6, 7, 8, 9 ]
        console.log(jds.path);
        T_BaseFrame = mcht.DHDeepCopy(T06); //setup jds.TM() function as it uses 'T_BaseFrame'
        var tmData = jds.TM(jds.path,T_BaseFrame); //returns [htma, T0, T0n]
        var T0_leg2 = [];
        for(var i=0;i<tmData[1].length;i=i+1){
            T0_leg2.push(mcht.DHDeepCopy(tmData[1][i])); //get T0
        }
        T_BaseFrame = mcht.DHDeepCopy(quad.T_BaseFrame_COPY); //'T_BaseFrame' back to original setting
        //      - add T06 to the start of the 'T0' to draw link 1.
        T0_leg2.unshift(T06);
        //      - plot robot
        cvp.drawRobotUsingT0(T0_leg2,'pink'); //see drawRobotSkeleton.js
        console.log(T0_leg2);
        
        //   - leg 3 end-effector
        //      - node path: node 10 (z10) to node 13 (z13)
        //jds.path = [];
        jds.resetPath();
        jds.find_path(10,13); //Array [ 10, 11, 12, 13 ]
        console.log(jds.path);
        T_BaseFrame = mcht.DHDeepCopy(T010); //setup jds.TM() function as it uses 'T_BaseFrame'
        var tmData = jds.TM(jds.path,T_BaseFrame); //returns [htma, T0, T0n]
        var T0_leg3 = [];
        for(var i=0;i<tmData[1].length;i=i+1){
            T0_leg3.push(mcht.DHDeepCopy(tmData[1][i])); //get T0
        }
        T_BaseFrame = mcht.DHDeepCopy(quad.T_BaseFrame_COPY); //'T_BaseFrame' back to original setting
        //      - add T010 to the start of the 'T0' to draw link 1.
        T0_leg3.unshift(T010);
        //      - plot robot
        cvp.drawRobotUsingT0(T0_leg3,'pink'); //see drawRobotSkeleton.js
        console.log(T0_leg3);
        
        //   - leg 4 end-effector
        //      - node path: node 14 (z14) to node 17 (z17)
        //jds.path = [];
        jds.resetPath();
        jds.find_path(14,17); //Array [ 14, 15, 16, 17 ]
        console.log(jds.path);
        T_BaseFrame = mcht.DHDeepCopy(T014); //setup jds.TM() function as it uses 'T_BaseFrame'
        var tmData = jds.TM(jds.path,T_BaseFrame); //returns [htma, T0, T0n]
        var T0_leg4 = [];
        for(var i=0;i<tmData[1].length;i=i+1){
            T0_leg4.push(mcht.DHDeepCopy(tmData[1][i])); //get T0
        }
        T_BaseFrame = mcht.DHDeepCopy(quad.T_BaseFrame_COPY); //'T_BaseFrame' back to original setting
        //      - add T014 to the start of the 'T0' to draw link 1.
        T0_leg4.unshift(T014);
        //      - plot robot
        cvp.drawRobotUsingT0(T0_leg4,'pink'); //see drawRobotSkeleton.js
        console.log(T0_leg4);
        
        //build the mass column vector and inertia matrix for the limbs, include the end-effector
        //   - leg 1
        //      - leg 1 end-effector (transform from frame {z5} (node 5) to the inertial frame, end-effector defined in the z5 frame)
        //                                                                             [       ai, alpha_i,  di,  vi]
        var EE_offset = 0.08;
        var T0_EE_leg1 = hlao.matrix_multiplication(T0_leg1[T0_leg1.length-1],mcht.Aij([EE_offset,     0.0, 0.0, 0.0])); //get the extremity joint
        //      - build the limb object
        var limb_leg1 = {
            limb: {
                T0: T0_leg1,
                dynamicParameters: dynamicParameters(T0_leg1) //returns [m, I_CoM]
            },
            endEffector: {
                T0: T0_EE_leg1,
                dynamicParameters: endEffector() //returns [m, I_CoM]
            }
        };
        console.log('Leg 1:');
        console.log(limb_leg1);
        
        //   - leg 2
        //      - leg 2 end-effector (transform from frame {z9} (node 9) to the inertial frame, end-effector defined in the z9 frame)
        //                                                                             [       ai, alpha_i,  di,  vi]
        var T0_EE_leg2 = hlao.matrix_multiplication(T0_leg2[T0_leg2.length-1],mcht.Aij([EE_offset,     0.0, 0.0, 0.0])); //get the extremity joint
        //      - build the limb object
        var limb_leg2 = {
            limb: {
                T0: T0_leg2,
                dynamicParameters: dynamicParameters(T0_leg2) //returns [m, I_CoM]
            },
            endEffector: {
                T0: T0_EE_leg2,
                dynamicParameters: endEffector() //returns [m, I_CoM]
            }
        };
        console.log('Leg 2:');
        console.log(limb_leg2);
        
        //   - leg 3
        //      - leg 3 end-effector (transform from frame {z13} (node 13) to the inertial frame, end-effector defined in the z13 frame)
        //                                                                             [       ai, alpha_i,  di,  vi]
        var T0_EE_leg3 = hlao.matrix_multiplication(T0_leg3[T0_leg3.length-1],mcht.Aij([EE_offset,     0.0, 0.0, 0.0])); //get the extremity joint
        //      - build the limb object
        var limb_leg3 = {
            limb: {
                T0: T0_leg3,
                dynamicParameters: dynamicParameters(T0_leg3) //returns [m, I_CoM]
            },
            endEffector: {
                T0: T0_EE_leg3,
                dynamicParameters: endEffector() //returns [m, I_CoM]
            }
        };
        console.log('Leg 3:');
        console.log(limb_leg3);
        
        //   - leg 4
        //      - leg 4 end-effector (transform from frame {z17} (node 17) to the inertial frame, end-effector defined in the z17 frame)
        //                                                                             [       ai, alpha_i,  di,  vi]
        var T0_EE_leg4 = hlao.matrix_multiplication(T0_leg4[T0_leg4.length-1],mcht.Aij([EE_offset,     0.0, 0.0, 0.0])); //get the extremity joint
        //      - build the limb object
        var limb_leg4 = {
            limb: {
                T0: T0_leg4,
                dynamicParameters: dynamicParameters(T0_leg4) //returns [m, I_CoM]
            },
            endEffector: {
                T0: T0_EE_leg4,
                dynamicParameters: endEffector() //returns [m, I_CoM]
            }
        };
        console.log('Leg 4:');
        console.log(limb_leg4);
        
        //build the robot
        quad.robot = {
            leg1: limb_leg1,
            leg2: limb_leg2,
            leg3: limb_leg3,
            leg4: limb_leg4
        };
        
        //solve equation (12) from 'Resolved Momentum Control'
        var vx = 0.015; var vy = 0.0; var vz = 0.0; //works in vx, vz
        var wx = 0.0; var wy = 0.0; var wz = 0.0;
        var vB_ref = [[vx],[vy],[vz]]; //translational column vector
        var wB_ref = [[wx],[wy],[wz]]; //rotational column vector
        //var thetaDotFree_ref = [[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]]; //IMPORTANT: right arm then left arm (theta free angular velocity (20 dof robot - 12 leg joint - 1 head - 1 torso = 6 free))
        var thetaDotFree_ref = [[]];
        var goalVelocity = { //reference foot (end-effector) velocities (IMPORTANT: these need to be updated)
            leg1: [[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]], //[vx, vy, vx, wx, wy, wz]T
            leg2: [[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]], //[vx, vy, vx, wx, wy, wz]T
            leg3: [[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]], //[vx, vy, vx, wx, wy, wz]T
            leg4: [[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]]  //[vx, vy, vx, wx, wy, wz]T
        };
        quad.refVelocity = {
            vBRef: vB_ref,
            wBRef: wB_ref,
            thetaDotFreeRef: thetaDotFree_ref,
            endEffectorGoalVelocity: goalVelocity
        };
        var mKp = totalMass(quad.robot)*0.25; //IMPORTANT: fix this
        quad.refMomentum = { //IMPORTANT: Linear and angular momentum of the whole mechanism (ref: Resolved Momentum Control: Humanoid Motion Planning based on the Linear and Angular Momentum (KAJITA). (page 1645))
            Pref: hlao.vector_multiplication_scalar([[vx],[vy],[vz]],mKp),
            Lref: hlao.vector_multiplication_scalar([[wx],[wy],[wz]],mKp)
        };
        
        //copy of the original homogeneous transformation matrix
        //   - leg 1
        //      - node path: node 2 (z2) to node 5 (z5)
        //jds.path = [];
        jds.resetPath();
        jds.find_path(2,5); //Array [ 2, 3, 4, 5 ]
        console.log(jds.path);
        //      - build the homogenous transformation matrix
        var tmData = jds.TM(jds.path,T_BaseFrame); //returns [htma, T0, T0n]
        var leg1_htm = [];
        for(var i=0;i<tmData[0].length;i=i+1){
            leg1_htm.push(mcht.DHDeepCopy(tmData[0][i]));
        }
        
        //   - leg 2
        //      - node path: node 6 (z6) to node 9 (z9)
        //jds.path = [];
        jds.resetPath();
        jds.find_path(6,9); //Array [ 6, 7, 8, 9 ]
        console.log(jds.path);
        //      - build the homogenous transformation matrix
        var tmData = jds.TM(jds.path,T_BaseFrame); //returns [htma, T0, T0n]
        var leg2_htm = [];
        for(var i=0;i<tmData[0].length;i=i+1){
            leg2_htm.push(mcht.DHDeepCopy(tmData[0][i]));
        }
        
        //   - leg 3
        //      - node path: node 10 (z10) to node 13 (z13)
        //jds.path = [];
        jds.resetPath();
        jds.find_path(10,13); //Array [ 10, 11, 12, 13 ]
        console.log(jds.path);
        //      - build the homogenous transformation matrix
        var tmData = jds.TM(jds.path,T_BaseFrame); //returns [htma, T0, T0n]
        var leg3_htm = [];
        for(var i=0;i<tmData[0].length;i=i+1){
            leg3_htm.push(mcht.DHDeepCopy(tmData[0][i]));
        }
        
        //   - leg 4
        //      - node path: node 14 (z14) to node 17 (z17)
        //jds.path = [];
        jds.resetPath();
        jds.find_path(14,17); //Array [ 14, 15, 16, 17 ]
        console.log(jds.path);
        //      - build the homogenous transformation matrix
        var tmData = jds.TM(jds.path,T_BaseFrame); //returns [htma, T0, T0n]
        var leg4_htm = [];
        for(var i=0;i<tmData[0].length;i=i+1){
            leg4_htm.push(mcht.DHDeepCopy(tmData[0][i]));
        }
        
        //store the original transforms
        quad.originalTransforms = {
            leg1: {
                htm: leg1_htm,
                T0n: T02
            },
            leg2: {
                htm: leg2_htm,
                T0n: T06
            },
            leg3: {
                htm: leg3_htm,
                T0n: T010
            },
            leg4: {
                htm: leg4_htm,
                T0n: T014
            }
        }
        
        //q's (joint angles) total rotation
        //qi = qi + delta q
        //[ 2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 17] //servo ID's
        //[ 2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 17] //nodes
        //[z2, z3, z4, z5, z6, z7, z8, z9,z10,z11,z12,z13,z14,z15,z16,z17]
        quad.qtr = hlao.zeros_vector(16,'row'); //row vector with 16 elements initialized as zero
        
        //selection matrix 'S'
        quad.selectionMatrix = [
            [1.0,0.0,0.0,0.0,0.0,0.0], //e1
            [0.0,1.0,0.0,0.0,0.0,0.0], //e2
            //[0.0,0.0,1.0,0.0,0.0,0.0], //e3
            //[0.0,0.0,0.0,1.0,0.0,0.0], //e4
            //[0.0,0.0,0.0,0.0,1.0,0.0], //e5
            [0.0,0.0,0.0,0.0,0.0,1.0]  //e6  
        ];
        
        quad.maxHip = -1000.0;
        quad.minHip = +1000.0;
        
        quad.samples = 100;
        //quad.samples = feetRight.length;
        //quad.samples = feetLeft.length;
        //quad.samples = quad.LSP.REET.vx.length;
        quad.dt = 2.0*Math.PI/quad.samples;
        //quad.dt = Tsup/101;
        console.log('number of samples (time step):' + quad.samples + '(' + quad.dt + ')');
        console.log('total mass of the robot: ' + totalMass(quad.robot));
        console.log('CoM of the robot: ' + totalCoM(quad.robot));
        quad.tcr = ""; //used to print total change in 'q' (total change in rotation)
    }
    
    console.log('quad.index: ' + quad.index);
    
    //get the target speed which realizes the reference momentum and speed
    var target = targetSpeed(quad.robot, quad.refVelocity, quad.refMomentum, quad.selectionMatrix); //returns [vB, wB, thetaFree]
    //console.log('Target:');
    //console.log(target);
    
    //update 'base link' (IMPORTANT: infinitesimally angular changes)
    //   - incremental 'displacement' of the 'origin' of the coordinate frame (page 51, ref Robotics, Vision and Control)
    //   - incremental 'rotation' about the 'origin' of the coordinate frame
    //      - the direction of the vector [[wx], [wy], [wz]] defines the instantaneous axis of rotation (at particular instant of time)
    //      - magnitude of [[wx], [wy], [wz]] is the rate of rotation about the axis
    //   - delta pose (change in position and orientation by integrating the linear and angular velocity)
    //   - [[vx],[vy],[vz],[wx],[wy],[wz]]
    //     [[ b],[ b],[ b],[ b],[ b],[ b]] where b = body
    //     [[ 0],[ 1],[ 2],[ 3],[ 4],[ 5]] index
    var d = hlao.vector_multiplication_scalar([[target[0][0]],[target[1][0]],[target[2][0]],[target[3][0]],[target[4][0]],[target[5][0]]],quad.dt);
    //   - IMPORTANT: delta theta now defined in world frame (page 52, ref Robotics, Vision and Control)
    quad.T0baseLink = 
        hlao.matrix_multiplication(
            mcht.delta2tr(d),
            quad.T0baseLink //IMPORTANT: is it pre- or post- multiplication (is it 3x3 or 4x4 identity)
        );
    console.log("T0 of the 'base link':" + quad.T0baseLink);
    
    //update T02
    quad.originalTransforms.leg1.T0n = 
        hlao.matrix_multiplication(
            mcht.delta2tr(d),
            quad.originalTransforms.leg1.T0n //IMPORTANT: is it pre- or post- multiplication (is it 3x3 or 4x4 identity)
        );
    //console.log("T05 update:" + quad.originalTransforms.leg1.T0n);
    //update T06
    quad.originalTransforms.leg2.T0n = 
        hlao.matrix_multiplication(
            mcht.delta2tr(d),
            quad.originalTransforms.leg2.T0n //IMPORTANT: is it pre- or post- multiplication (is it 3x3 or 4x4 identity)
        );
    //console.log("T06 update:" + quad.originalTransforms.leg2.T0n);
    //update T010
    quad.originalTransforms.leg3.T0n = 
        hlao.matrix_multiplication(
            mcht.delta2tr(d),
            quad.originalTransforms.leg3.T0n //IMPORTANT: is it pre- or post- multiplication (is it 3x3 or 4x4 identity)
        );
    //console.log("T010 update:" + quad.originalTransforms.leg3.T0n);
    //update T014
    quad.originalTransforms.leg4.T0n = 
        hlao.matrix_multiplication(
            mcht.delta2tr(d),
            quad.originalTransforms.leg4.T0n //IMPORTANT: is it pre- or post- multiplication (is it 3x3 or 4x4 identity)
        );
    //console.log("T014 update:" + quad.originalTransforms.leg4.T0n);
    
    //update the homogeneous transformation matrix (legs)
    //   - calc. 'base link' position vector IMPORTANT: not the current position.
    var pb = [
        [quad.T0baseLink[0][3]], //x
        [quad.T0baseLink[1][3]], //y
        [quad.T0baseLink[2][3]]  //z
    ];
    //   - leg 1
    //      - calc. leg angular velocities
    var legOneVelocities = legJointSpeeds(pb,quad.robot.leg1,quad.refVelocity.endEffectorGoalVelocity.leg1,[[target[0][0]],[target[1][0]],[target[2][0]],[target[3][0]],[target[4][0]],[target[5][0]]]); //leg 1 (target is of the body)
    //      - delta pose (change in position and orientation by integrating the linear and angular velocity)
    //      - [[vx],[vy],[vz],[wx],[wy],[wz]] for foot leg 1
    var deltaPoseLeg1 = hlao.vector_multiplication_scalar(legOneVelocities,quad.dt); //delta in position, orientation
    //console.log('leg 1 delta theta: ' + deltaPoseLeg1);
    var holder = updateTransforms(quad.T_BaseFrame_COPY,quad.originalTransforms.leg1,deltaPoseLeg1);
    //console.log('delta leg 1: ' + deltaPoseLeg1);
    quad.robot.leg1.limb.T0 = holder[0];
    quad.originalTransforms.leg1.htm = holder[1];
    //      - update the total rotation (leg 1)
    quad.qtr[0] = quad.qtr[0] + deltaPoseLeg1[0][0]; //node 2, z2
    quad.qtr[1] = quad.qtr[1] + deltaPoseLeg1[1][0]; //node 3, z3
    quad.qtr[2] = quad.qtr[2] + deltaPoseLeg1[2][0]; //node 4, z4
    quad.qtr[3] = quad.qtr[3] + deltaPoseLeg1[3][0]; //node 5, z5
    //   - leg 2
    //      - calc. leg angular velocities
    var legTwoVelocities = legJointSpeeds(pb,quad.robot.leg2,quad.refVelocity.endEffectorGoalVelocity.leg2,[[target[0][0]],[target[1][0]],[target[2][0]],[target[3][0]],[target[4][0]],[target[5][0]]]); //leg 2 (target is of the body)
    //      - delta pose (change in position and orientation by integrating the linear and angular velocity)
    //      - [[vx],[vy],[vz],[wx],[wy],[wz]] for foot leg 2
    var deltaPoseLeg2 = hlao.vector_multiplication_scalar(legTwoVelocities,quad.dt); //delta in position, orientation
    console.log('leg 2 delta theta: ' + deltaPoseLeg2);
    var holder = updateTransforms(quad.T_BaseFrame_COPY,quad.originalTransforms.leg2,deltaPoseLeg2);
    //console.log('delta leg 2: ' + deltaPoseLeg2);
    quad.robot.leg2.limb.T0 = holder[0];
    quad.originalTransforms.leg2.htm = holder[1];
    //      - update the total rotation (leg 2)
    quad.qtr[4] = quad.qtr[4] + deltaPoseLeg2[0][0]; //node 6, z6
    quad.qtr[5] = quad.qtr[5] + deltaPoseLeg2[1][0]; //node 7, z7
    quad.qtr[6] = quad.qtr[6] + deltaPoseLeg2[2][0]; //node 8, z8
    quad.qtr[7] = quad.qtr[7] + deltaPoseLeg2[3][0]; //node 9, z9
    //   - leg 3
    //      - calc. leg angular velocities
    var legThreeVelocities = legJointSpeeds(pb,quad.robot.leg3,quad.refVelocity.endEffectorGoalVelocity.leg3,[[target[0][0]],[target[1][0]],[target[2][0]],[target[3][0]],[target[4][0]],[target[5][0]]]); //leg 3 (target is of the body)
    //      - delta pose (change in position and orientation by integrating the linear and angular velocity)
    //      - [[vx],[vy],[vz],[wx],[wy],[wz]] for foot leg 3
    var deltaPoseLeg3 = hlao.vector_multiplication_scalar(legThreeVelocities,quad.dt); //delta in position, orientation
    console.log('leg 3 delta theta: ' + deltaPoseLeg3);
    var holder = updateTransforms(quad.T_BaseFrame_COPY,quad.originalTransforms.leg3,deltaPoseLeg3);
    //console.log('delta leg 3: ' + deltaPoseLeg3);
    quad.robot.leg3.limb.T0 = holder[0];
    quad.originalTransforms.leg3.htm = holder[1];
    //      - update the total rotation (leg 3)
    quad.qtr[8]  = quad.qtr[8]  + deltaPoseLeg3[0][0]; //node 10, z10
    quad.qtr[9]  = quad.qtr[9]  + deltaPoseLeg3[1][0]; //node 11, z11
    quad.qtr[10] = quad.qtr[10] + deltaPoseLeg3[2][0]; //node 12, z12
    quad.qtr[11] = quad.qtr[11] + deltaPoseLeg3[3][0]; //node 13, z13
    //   - leg 4
    //      - calc. leg angular velocities
    var legFourVelocities = legJointSpeeds(pb,quad.robot.leg4,quad.refVelocity.endEffectorGoalVelocity.leg4,[[target[0][0]],[target[1][0]],[target[2][0]],[target[3][0]],[target[4][0]],[target[5][0]]]); //leg 4 (target is of the body)
    //      - delta pose (change in position and orientation by integrating the linear and angular velocity)
    //      - [[vx],[vy],[vz],[wx],[wy],[wz]] for foot leg 4
    var deltaPoseLeg4 = hlao.vector_multiplication_scalar(legFourVelocities,quad.dt); //delta in position, orientation
    console.log('leg 4 delta theta: ' + deltaPoseLeg4);
    var holder = updateTransforms(quad.T_BaseFrame_COPY,quad.originalTransforms.leg4,deltaPoseLeg4);
    //console.log('delta leg 4: ' + deltaPoseLeg4);
    quad.robot.leg4.limb.T0 = holder[0];
    quad.originalTransforms.leg4.htm = holder[1];
    //      - update the total rotation (leg 4)
    quad.qtr[12] = quad.qtr[12] + deltaPoseLeg4[0][0]; //node 14, z14
    quad.qtr[13] = quad.qtr[13] + deltaPoseLeg4[1][0]; //node 15, z15
    quad.qtr[14] = quad.qtr[14] + deltaPoseLeg4[2][0]; //node 16, z16
    quad.qtr[15] = quad.qtr[15] + deltaPoseLeg4[3][0]; //node 17, z17
    
    /*
    if(quad.qtr[6] > quad.maxHip){
        quad.maxHip = quad.qtr[6];
        quad.maxIndex = quad.index;
    }
    if(quad.qtr[6] < quad.minHip){
        quad.minHip = quad.qtr[6];
        quad.minIndex = quad.index;
    }
    //console.log(quad.qtr[6].toFixed(4) + ', ' + quad.qtr[11].toFixed(4));
    */
    
    //store total rotations (total change in 'q') in a string
    quad.tcr = quad.tcr + '[';
    for(var j=0;j<quad.qtr.length;j=j+1){
        //   - convert radians to degrees
        var degrees_delta = 180.0*quad.qtr[j]/Math.PI;
        //   - convert to raw data
        var raw_delta = ~~(degrees_delta/0.325); //float to integer
        quad.tcr = quad.tcr + raw_delta.toString();
        if(j<(quad.qtr.length - 1)) quad.tcr = quad.tcr + ',';
    }
    quad.tcr = quad.tcr + '],\n';
    
    //plot
    //   - clear the screen
    cvp.clearCanvas();
    //   - now draw
    cvp.drawRobotUsingT0(quad.robot.leg1.limb.T0,'blue');
    cvp.drawRobotUsingT0(quad.robot.leg2.limb.T0,'green');
    cvp.drawRobotUsingT0(quad.robot.leg3.limb.T0,'yellow');
    cvp.drawRobotUsingT0(quad.robot.leg4.limb.T0,'orange');
    //   - plot base link frame
    cvp.setColour('cyan');
    cvp.plotFrame(cvp.camera(quad.T0baseLink));
    cvp.setColour('black');
    
    //increment index
    quad.index = quad.index + 1;
    
    //   - clear timer
    if(quad.index >= (quad.samples - 1)){
    //if(quad.index >= (1)){
        clearInterval(quad.timer);
        delete(quad.timer);
        //alert('Job Complete.');
        
        //print min., max. ID 6 joint angle
        //min.: -0.09348993522666355 [556], max.: 0.08103242078603275 [49]
        //console.log('min.: ' + quad.minHip + ' [' + quad.minIndex + ']' + ', max.: ' + quad.maxHip + ' [' + quad.maxIndex + ']');
        
        //console.log(quad.tcr); //print the total change in rotations ('q')
        
        //return {vx: quad.CoM.vx, vy: quad.CoM.vy};
        return 1;
    }
}

function legJointSpeeds(pb,openLink,footVelocities,bodyVelocities){
    //get the foot velocities
    //get the body velocities
    //console.log('foot velocities:');
    //console.log(footVelocities);
    //console.log('body velocities:');
    //console.log(bodyVelocities);
    
    //build the identity matrix
    var E = hlao.identity_matrix(3);
    
    //calc. Jacobian inverse
    //   - Jacobian matrix (6 x 6) calculated from the leg configuration.
    var Jleg = gJ(openLink); //see also DUMMYgeometricJacobian() from file transformGraph.js
    //console.log('Jacobian matrix:');
    //console.log(Jleg);
    //   - JacobianInverse_svdcmp(Jleg); //run diagnostics
    //JacobianInverse_svdcmp(Jleg);
    
    var JlegInverse = leftPseudoInverse(Jleg); //pseudo-inverse
    //var JlegInverse = JacobianInverse(Jleg); //square matrix
    //console.log('Jacobian inverse:');
    //console.log(JlegInverse);
    
    //skew symmetric matrix
    var rB_F = S([
        [openLink.endEffector.T0[0][3] - pb[0][0]], //x
        [openLink.endEffector.T0[1][3] - pb[1][0]], //y
        [openLink.endEffector.T0[2][3] - pb[2][0]]  //z
    ]);
    //console.log('Position vector B->Fi:');
    //console.log(rB_F);
    
    //equ.(3) J^-1([[vfi],[wfi]] - [[E -mrB->Fi], [0 E]][[vB],[wB]])
    var legVelocities = 
        hlao.matrix_multiplication(
            JlegInverse,
            hlao.matrix_arithmetic(
                footVelocities,
                hlao.matrix_multiplication(
                    [
                        [E[0][0],E[0][1],E[0][2],rB_F[0][0],rB_F[0][1],rB_F[0][2]],
                        [E[1][0],E[1][1],E[1][2],rB_F[1][0],rB_F[1][1],rB_F[1][2]],
                        [E[2][0],E[2][1],E[2][2],rB_F[2][0],rB_F[2][1],rB_F[2][2]],
                        [      0,      0,      0,   E[0][0],   E[0][1],   E[0][2]],
                        [      0,      0,      0,   E[1][0],   E[1][1],   E[1][2]],
                        [      0,      0,      0,   E[2][0],   E[2][1],   E[2][2]],
                    ],
                    bodyVelocities
                ),
                '-'
            )
        );
    
    return legVelocities;
}

//update the homogenous transformation matrix due to change in pose
function updateTransforms(T_BaseFrame_COPY,originalTransforms,deltaPose){
    //   - get the number of transforms to update
    var n = originalTransforms.htm.length; //get the number of rows
    
    /*
    for(var i=0;i<n;i=i+1){
        //   - update transformation matrix
        //      - get the old transform (before the update)
        var Tb = originalTransforms.htm[i];
        var Tz = mcer.trotz(deltaPose[i][0]);
        //      - convert to homogenous transform
        var T = hlao.matrix_multiplication(Tz,Tb);
        //var T = hlao.matrix_multiplication(Tb,Tz);
        //      - update the old transformation matrix based on the delta theta
        originalTransforms.htm[i] = mcht.DHDeepCopy(T);
    }
    */
    
    for(var i=0;i<n;i=i+1){
        var d = [[0.0],[0.0],[0.0],[0.0],[0.0],[deltaPose[i]]];
        originalTransforms.htm[i] = mcht.DHDeepCopy(
            hlao.matrix_multiplication(
                mcht.delta2tr(d),
                originalTransforms.htm[i]
            )
        );
    }
    
    //   - convert transforms to the inertia frame
    T_BaseFrame = mcht.DHDeepCopy(originalTransforms.T0n); //setup jds.TM() function as it uses 'T_BaseFrame'
    var tmData = jds.buildTransformsBaseFrame(originalTransforms.htm,T_BaseFrame); //returns [T0, T0n]
    var T0update = [];
    for(var j=0;j<tmData[0].length;j=j+1){
        T0update.push(mcht.DHDeepCopy(tmData[0][j])); //get T0
    }
    T_BaseFrame = mcht.DHDeepCopy(T_BaseFrame_COPY); //'T_BaseFrame' back to original setting
    //      - add T0n to the start of the 'T0' to draw link 1.
    T0update.unshift(originalTransforms.T0n);
    
    return [T0update,originalTransforms.htm];
}

//see rightFoot.html
function feetVelocities_right(){
    var v = []; //[[...], [[vx],[vy],[vz],[wx],[wy],[wz], ...]
    
    //ref: https://en.wikipedia.org/wiki/Sine_wave
    //y(t) = A*Math.sin(2*Math.PI*f*t+phase);
    //where:
    //'A' amplitude
    //'f' number of cycles each time period
    //'phase' where in its cycle
    
    var Ax = 0.0;
    var Az = 0.01;
    var fX = 0.5;
    var fZ = 0.5;
    var phaseX = 0.0;
    var phaseZ = 0.0;
    var samples = 100;
    for(var t=0;t<=samples;t=t+1){
        v.push([
            [Az*Math.sin((2.0*Math.PI*fZ*t)/samples+phaseZ)], //vz (ref: fig. 5 (Resolved Momentum Control))
            [0.0], //vy
            [Ax*Math.sin((2.0*Math.PI*fX*t)/samples+phaseX)], //vx (ref: fig. 5 (Resolved Momentum Control))
            [0.0], //wx
            [0.0], //wy
            [0.0]  //wz
        ]);
    }
    
    return v;
}

//see leftFoot.html
function feetVelocities_left(){
    var v = []; //[[...], [[vx],[vy],[vz],[wx],[wy],[wz], ...]
    
    //ref: https://en.wikipedia.org/wiki/Sine_wave
    //y(t) = A*Math.sin(2*Math.PI*f*t+phase);
    //where:
    //'A' amplitude
    //'f' number of cycles each time period
    //'phase' where in its cycle
    
    var Ax = 0.02;
    var Az = 0.0;
    var fX = 0.5;
    var fZ = 0.5;
    var phaseX = 0.0;
    var phaseZ = 0.0;
    var samples = 100;
    var timeStep = 2.0*Math.PI*fX/samples;
    console.log('left foot trajectories time step: ' + timeStep);
    var totalXdisplacement = 0.0;
    var totalZdisplacement = 0.0;
    for(var t=0;t<=samples;t=t+1){
        v.push([
            [Az*Math.sin(((2.0*Math.PI*fZ*t)/samples)+phaseZ)], //vz
            [0.0], //vy
            [Ax*Math.sin(((2.0*Math.PI*fX*t)/samples)+phaseX)], //vx
            [0.0], //wx
            [0.0], //wy
            [0.0]  //wz
        ]);
        
        if(t < samples){
            totalXdisplacement = totalXdisplacement + timeStep * v[t][2]; //all points but the last point. IMPORTANT: check this numerical integration
            totalZdisplacement = totalZdisplacement + timeStep * v[t][0];
        }
    }
    console.log('total x displacement: ' + totalXdisplacement);
    console.log('total z displacement: ' + totalZdisplacement);
    
    return v;
}

function dynamicParameters(T0){
    var density = 8050.0; // kg/m3 (Steel/Density)
    var radius = 0.030/2.0; //meters
    var V = (4.0/3.0)*Math.PI*Math.pow(radius,3);
    var mass = density*V; //kg
    var Ixx = (2.0/5.0) * mass * Math.pow(radius,2); // Slender Rod moment of inertia. ref: Hibbeler dynamics back of book.
    var Iyy = Ixx;
    var Izz = Ixx;
    
    //console.log(mass);
    //console.log(Ixx);
    
    var m = []; //column vector
    var I_CoM = [];
    
    var n = T0.length - 1; //one less the number of transforms
    if(n == 0){
        //console.log('Generating dynamic parameters for the end-effector.');
        n = 1; //end-effector
    }
    
    for(var i=0;i<n;i=i+1){
        m.push(mass);
        I_CoM.push([
            [Ixx, 0.0, 0.0],
            [0.0, Iyy, 0.0],
            [0.0, 0.0, Izz]
        ]);
    }
    
    return [m, I_CoM];
}

function endEffector(){
    var density = 8050.0; // kg/m3 (Steel/Density)
    var radius = 0.030/2.0; //meters
    var V = (4.0/3.0)*Math.PI*Math.pow(radius,3);
    var mass = density*V; //kg
    var Ixx = (2.0/5.0) * mass * Math.pow(radius,2); // Slender Rod moment of inertia. ref: Hibbeler dynamics back of book.
    var Iyy = Ixx;
    var Izz = Ixx;
    
    var m = mass;
    
    var I_CoM = [
        [Ixx, 0.0, 0.0],
        [0.0, Iyy, 0.0],
        [0.0, 0.0, Izz]
    ];
    
    return [m, I_CoM];
}

// 3 x 3 inertia matrix with respect to the CoM
function I_CoM(){
    // Ii = Ri x Iili x RiT + mi x ST(rci) x S(rci)
    //   - where rci is the position vector from baseframe to CoM is link i
    //   - refer to equation (7.12) and (7.71) for example
}


// Total Mass.
function totalMass(robot){
    //console.log(robot);
    var openLinks = Object.keys(robot);
    //console.log(openLinks); //[ "leg1", "leg2", "leg3", "leg4" ]
    var massTotal = 0.0;
    var linkCount = 0;
    for(var i=0;i<openLinks.length;i=i+1){
        //console.log(openLinks[i]);
        //console.log(Object.keys(robot[openLinks[i]])); //[ "limb", "endEffector" ]
        //limb
        for(var j=0;j<robot[openLinks[i]].limb.dynamicParameters[0].length;j=j+1){
            //console.log(robot[openLinks[i]].limb.dynamicParameters[0][j]);
            massTotal = massTotal + robot[openLinks[i]].limb.dynamicParameters[0][j];
            linkCount = linkCount + 1;
        }
        //end-effector
        massTotal = massTotal + robot[openLinks[i]].endEffector.dynamicParameters[0];
        linkCount = linkCount + 1;
    }
    //console.log('Total mass [count]: ' + massTotal + '[' + linkCount + ']');
    
    return massTotal;
}

// Total CoM Position.
//   - ref: Hibbeler (Statics) page 246.
//   - Defined in frame inertial reference frame (Cartesian frame fixed to ground).
function totalCoM(robot){
    var x_bar = 0;
    var y_bar = 0;
    var z_bar = 0;
    
    //total mass of the robot
    var massTotal = totalMass(robot);
    
    //console.log(robot);
    var openLinks = Object.keys(robot);
    //console.log(openLinks); //[ "leg1", "leg2", "leg3", "leg4" ]
    
    var linkCount = 0;
    for(var i=0;i<openLinks.length;i=i+1){ //for each limb
        //console.log(openLinks[i]);
        //console.log(Object.keys(robot[openLinks[i]])); //[ "limb", "endEffector" ]
        
        //limb
        for(var j=0;j<(robot[openLinks[i]].limb.T0.length - 1);j=j+1){
            //   - position vector of CoM for each link
            var c = [
                [robot[openLinks[i]].limb.T0[j][0][3] + (robot[openLinks[i]].limb.T0[j+1][0][3] - robot[openLinks[i]].limb.T0[j][0][3])/2.0],
                [robot[openLinks[i]].limb.T0[j][1][3] + (robot[openLinks[i]].limb.T0[j+1][1][3] - robot[openLinks[i]].limb.T0[j][1][3])/2.0],
                [robot[openLinks[i]].limb.T0[j][2][3] + (robot[openLinks[i]].limb.T0[j+1][2][3] - robot[openLinks[i]].limb.T0[j][2][3])/2.0]
            ];
            //console.log('Link CoM:');
            //console.log(c);
            
            //   - plot CoM
            //plotPoint(robot[openLinks[i]].limb.T0[j],robot[openLinks[i]].limb.T0[j+1]);
            
            //   - mass of each link (IMPORTANT: too many masses against position of CoM for link (should only be one mass per CoM of link))
            var mass = robot[openLinks[i]].limb.dynamicParameters[0][j];
            
            //   - update CoM
            x_bar = x_bar + c[0][0] * mass;
            y_bar = y_bar + c[1][0] * mass;
            z_bar = z_bar + c[2][0] * mass;
            
            //   - increment link counter
            linkCount = linkCount + 1;
        }
        
        //end-effector
        //   - position vector of CoM for the end-effector
        var index_T0n = robot[openLinks[i]].limb.T0.length - 1;
        var c = [
            [robot[openLinks[i]].limb.T0[index_T0n][0][3] + (robot[openLinks[i]].endEffector.T0[0][3] - robot[openLinks[i]].limb.T0[index_T0n][0][3])/2.0],
            [robot[openLinks[i]].limb.T0[index_T0n][1][3] + (robot[openLinks[i]].endEffector.T0[1][3] - robot[openLinks[i]].limb.T0[index_T0n][1][3])/2.0],
            [robot[openLinks[i]].limb.T0[index_T0n][2][3] + (robot[openLinks[i]].endEffector.T0[2][3] - robot[openLinks[i]].limb.T0[index_T0n][2][3])/2.0]
        ];
        //console.log('End-effector CoM:');
        //console.log(c);
        
        //   - mass of the end-effector
        var mass = robot[openLinks[i]].endEffector.dynamicParameters[0];
        
        //   - update CoM
        x_bar = x_bar + c[0][0] * mass;
        y_bar = y_bar + c[1][0] * mass;
        z_bar = z_bar + c[2][0] * mass;
        
        //   - increment link counter
        linkCount = linkCount + 1;
    }
    //console.log('Link count: ' + linkCount);
    
    //Total CoM
    x_bar = x_bar/massTotal;
    y_bar = y_bar/massTotal;
    z_bar = z_bar/massTotal;
    //console.log('Robot CoM: (' + x_bar + ', ' + y_bar + ', ' + z_bar + ')');
    
    return [[x_bar],[y_bar],[z_bar]];
}

function plotPoint(Ts,Te,colour){
    var Ts = cvp.camera(Ts);
    var Te = cvp.camera(Te);
    
    var c = [
        [Ts[0][3] + (Te[0][3] - Ts[0][3])/2.0],
        [Ts[1][3] + (Te[1][3] - Ts[1][3])/2.0],
        [Ts[2][3] + (Te[2][3] - Ts[2][3])/2.0]
    ];
    //console.log('Plotting point: ' + c);
    
    var scale = cvp.scaleSkeleton*20;
    var xy = cvp.getCanvasWH();
    var cx = xy[0]/2.0;
    var cy = xy[1]/2.0;
    
    tools.strokeStyle = colour;
    tools.beginPath();
    tools.moveTo(c[0][0]*scale+cx,-1.0*c[1][0]*scale+cy); //scale and flip on 'y'
    tools.arc(c[0][0]*scale+cx,-1.0*c[1][0]*scale+cy,5,0,2.0*Math.PI); //scale and flip on 'y'
    tools.closePath();
    tools.fill();
    tools.strokeStyle = 'black';
}

// Target Speed - equation (12)
//   - input vB_ref, wB_ref, thetaFree_ref
//   - output vB, wB, thetaFree
function targetSpeed(robot, refVelocity, refMomentum, sm){
    var debug = 0;
    
    // step 0. preliminaries
    //   - load reference velocities
    var vB_ref = refVelocity.vBRef;
    var wB_ref = refVelocity.wBRef;
    var thetaFree_ref = refVelocity.thetaDotFreeRef;
    
    //   - get the number of free joints 'n'
    var dim = [];
    [dim[0],dim[1]] = [refVelocity.thetaDotFreeRef.length,refVelocity.thetaDotFreeRef[0].length];
    //console.log("'thetaDotFreeRef' vector dimension: " + dim[0] + ', ' + dim[1]);
    if(typeof(dim[1]) === 'undefined'){
        console.log('Warning: the theta free reference (joint angular velocities) is a row vector, require a column vector.');
    } else {
        if(dim[1] > 1){
            console.log('Warning: the theta free reference (joint angular velocities) is a matrix, require a column vector.');
        } else {
            var n = dim[0];
        }
    }
    var n = 0;
    //console.log('The number of free joints: ' + n);
    
    //   - create the 'n x n' identiy matrix
    var E = hlao.identity_matrix((n + mtojs.size(vB_ref,1) + mtojs.size(wB_ref,1))); //include the body reference velocities
    if(debug){
        console.log("'E' (identity matrix) matrix dimension: " + mtojs.size(E,1) + ', ' + mtojs.size(E,2));
        console.log(E);
    }
    
    // step 1. calc. the 'A' matrix
    var A = hlao.matrix_multiplication(sm,theAmatrix(robot));
    if(debug){
        console.log("'A' matrix dimension: " + mtojs.size(A,1) + ', ' + mtojs.size(A,2));
        console.log(A);
    }
    
    // step 2. calc. the 'Adagger' matrix (pseudo inverse of the 'A' matrix)
    var Adagger = AdaggerMatrix(A); // pseudo inverse (U+2020)
    if(debug){
        console.log("'Adagger' matrix dimension: " + mtojs.size(Adagger,1) + ', ' + mtojs.size(Adagger,2));
        console.log(Adagger);
    }
    
    // step 3. calc. the 'y' column vector
    var y =  hlao.matrix_multiplication(sm,y_vector(robot, refVelocity, refMomentum));
    if(debug){
        console.log("'y' matrix dimension: " + mtojs.size(y,1) + ', ' + mtojs.size(y,2));
        console.log(y);
    }
    
    // step 4. calc. vB, wB, thetaFree
    //   - build the n x 1 speed column vector
    var refSpeed = [[vB_ref[0][0]], [vB_ref[1][0]], [vB_ref[2][0]], [wB_ref[0][0]], [wB_ref[1][0]], [wB_ref[2][0]]];
    for(var i=0; i<n; i=i+1){
        refSpeed.push([thetaFree_ref[i][0]]);
    }
    if(debug){
        console.log("'refSpeed' matrix dimension: " + mtojs.size(refSpeed,1) + ', ' + mtojs.size(refSpeed,2));
        console.log(refSpeed);
    }
    
    //   - calc. target (vB, wB)
    var partA = hlao.matrix_multiplication(Adagger,y);
    if(debug){
        console.log('partA calc.:');
        console.log("'partA' matrix dimension: " + mtojs.size(partA,1) + ', ' + mtojs.size(partA,2));
        console.log(partA);
    }
    
    if(debug){
        console.log("Adagger x A");
        console.log(hlao.matrix_multiplication(Adagger,A));
    }
    
    var partB = hlao.matrix_arithmetic(E,hlao.matrix_multiplication(Adagger,A),'-');
    if(debug){
        console.log('partB calc.:');
        console.log("'partB' matrix dimension: " + mtojs.size(partB,1) + ', ' + mtojs.size(partB,2));
        console.log(partB);
    }
    var partC = hlao.matrix_multiplication(partB,refSpeed);
    if(debug){
        console.log('partC calc.:');
        console.log("'partC' matrix dimension: " + mtojs.size(partC,1) + ', ' + mtojs.size(partC,2));
        console.log(partC);
    }
    var target = hlao.matrix_arithmetic(partA,partC,'+');
    
    // step 5. return [vB, wB, thetaFree]T;
    if(debug){
        console.log('Target:');
        console.log("'target' matrix dimension: " + mtojs.size(target,1) + ', ' + mtojs.size(target,2));
        console.log(target);
    }
    return target;
}

function y_vector(robot, refVelocity, refMomentum){
    var l1 = refVelocity.endEffectorGoalVelocity.leg1;
    var l2 = refVelocity.endEffectorGoalVelocity.leg2;
    var l3 = refVelocity.endEffectorGoalVelocity.leg3;
    var l4 = refVelocity.endEffectorGoalVelocity.leg4;
    
    //console.log('Leg 1 end-effector velocities:');
    //console.log(l1);
    //console.log('Leg 2 end-effector velocities:');
    //console.log(l2);
    //console.log('Leg 3 end-effector velocities:');
    //console.log(l3);
    //console.log('Leg 4 end-effector velocities:');
    //console.log(l4);
    
    var Pref = refMomentum.Pref;
    var Lref = refMomentum.Lref;
    
    //build [Pref, Lref]T
    var partA = [
        [Pref[0][0]],
        [Pref[1][0]],
        [Pref[2][0]],
        [Lref[0][0]],
        [Lref[1][0]],
        [Lref[2][0]]
    ];
    //console.log('Linear and angular reference momentum:');
    //console.log(partA);
    
    //Inertia matrices under constraint.
    //console.log('Inertia matrices under constraint.');
    //   - leg 1
    //console.log('   - Leg 1:');
    var imsFleg1 = inertiaMatricesUnderConstraints(robot.leg1);
    //console.log('imsFleg1:');
    //console.log(imsFleg1);
    
    //   - leg 2
    //console.log('   - Leg 2:');
    var imsFleg2 = inertiaMatricesUnderConstraints(robot.leg2);
    //console.log('imsFleg2:');
    //console.log(imsFleg2);
    
    //   - leg 3
    //console.log('   - Leg 3:');
    var imsFleg3 = inertiaMatricesUnderConstraints(robot.leg3);
    //console.log('imsFleg3:');
    //console.log(imsFleg3);
    
    //   - leg 4
    //console.log('   - Leg 4:');
    var imsFleg4 = inertiaMatricesUnderConstraints(robot.leg4);
    //console.log('imsFleg4:');
    //console.log(imsFleg4);
    
    //summation of leg 1 and leg 2 of [[Mleg],[Hleg]]J-1
    var sum1_2 = 
        hlao.matrix_arithmetic(
            hlao.matrix_multiplication(imsFleg1,
            [
                [l1[0][0]],
                [l1[1][0]],
                [l1[2][0]],
                [l1[3][0]],
                [l1[4][0]],
                [l1[5][0]]
            ]),
            hlao.matrix_multiplication(imsFleg2,
            [
                [l2[0][0]],
                [l2[1][0]],
                [l2[2][0]],
                [l2[3][0]],
                [l2[4][0]],
                [l2[5][0]]
            ]),
            '+'
        );
    //summation of leg 3 and leg 4 of [[Mleg],[Hleg]]J-1
    var sum3_4 = 
        hlao.matrix_arithmetic(
            hlao.matrix_multiplication(imsFleg3,
            [
                [l3[0][0]],
                [l3[1][0]],
                [l3[2][0]],
                [l3[3][0]],
                [l3[4][0]],
                [l3[5][0]]
            ]),
            hlao.matrix_multiplication(imsFleg4,
            [
                [l4[0][0]],
                [l4[1][0]],
                [l4[2][0]],
                [l4[3][0]],
                [l4[4][0]],
                [l4[5][0]]
            ]),
            '+'
        );
    var partB = hlao.matrix_arithmetic(sum1_2,sum3_4,'+');
    
    //element wise subtraction of [[mE -mrB->c]T [0 I]T]
    var y = hlao.matrix_arithmetic(partA,partB,'-');
    //console.log('y:');
    //console.log(y);
    
    return y;
}

//Pseudo Inverse of the 'A' matrix
//ref: page 17, I:\robot\7_Mathematics\books\biped\wholeBody\A Momentum-based Bipedal Balance Controller.ppt
//J+ = (J^T.J)^-1 = J^T(J.J^T)-1
function AdaggerMatrix(A){
    var debug = 0;
    
    var dim = mtojs.size(A);
    var m = dim[0]; //'m' rows
    var n = dim[1]; //'n' columns
    if(debug) console.log("'A' size: " + mtojs.size(A));
    
    var AT = hlao.matrix_transpose(A); //transpose the 'A' matrix
    var AAT = hlao.matrix_multiplication(A,AT);
    var dim_sq = mtojs.size(AAT);
    var m_sq = dim_sq[0]; //'m' rows
    var n_sq = dim_sq[1]; //'n' columns
    if(m_sq !== n_sq) console.log('WARNING AdaggerMatrix not square.');
    if(m_sq !== 6) console.log('WARNING AdaggerMatrix does not have dim[0] of 6.');
    
    //insert dummy value into start of each row vector
    for(var i=0;i<m_sq;i=i+1){
        AAT[i].unshift(0.0);
    }
    //insert a dummy row
    var offsetRow = [];
    for(var i=0;i<(n_sq+1);i=i+1){ //one extract column added of dummy values
        offsetRow.push(0.0);
    }
    AAT.unshift(offsetRow);
    if(debug){
        console.log("Size of 'A.AT': " + mtojs.size(AAT));
        console.log('Padded A.A^T');
        console.log(AAT);
    }
    
    //calc. the inverse
    var AATinverse = ludcmp.matrixInverseLU(AAT,m_sq);
    if(debug){
        console.log('Inverse of A.A^T');
        console.log(AATinverse);
    }
    
    //remove dummy row
    AATinverse.shift();
    //remove the dummy column
    for(var i=0;i<m_sq;i=i+1){ //'m' dropping element per row.
        AATinverse[i].shift(0.0);
    }
    if(debug){
        console.log('Cleaned inverse of A.A^T');
        console.log(AATinverse);
    }
    
    //do final calc.
    var Adagger = hlao.matrix_multiplication(AT,AATinverse);
    
    return Adagger;
}

function theAmatrix(robot){
    //identity matrix (3 x 3)
    var E = hlao.identity_matrix(3);

    //total mass of the robot
    var m_tilde = totalMass(robot);
    //console.log('Total mass of the robot: ' + m_tilde);
    
    //CoM of the robot (position vector defined in inertial frame with reference to the interial frame)
    var c_tilde = totalCoM(robot); //total CoM Position
    //console.log('Robot CoM: (' + c_tilde[0][0] + ', ' + c_tilde[1][0] + ', ' + c_tilde[2][0] + ')');
    
    //base link position vector
    /*
    var pb = [
        [robot.rightLeg.limb.T0[0][0][3] + (robot.leftLeg.limb.T0[0][0][3] - robot.rightLeg.limb.T0[0][0][3])/2], //x
        [robot.rightLeg.limb.T0[0][1][3] + (robot.leftLeg.limb.T0[0][1][3] - robot.rightLeg.limb.T0[0][1][3])/2], //y
        [robot.rightLeg.limb.T0[0][2][3] + (robot.leftLeg.limb.T0[0][2][3] - robot.rightLeg.limb.T0[0][2][3])/2]  //z
    ];
    */
    var pb = [
        [quad.T0baseLink[0][3]], //x
        [quad.T0baseLink[1][3]], //y
        [quad.T0baseLink[2][3]]  //z
    ];
    //console.log('Base link position vector: (' + pb[0][0] + ',' + pb[1][0] + ',' + pb[2][0] + ')');
    
    //vector from base link to total CoM
    //   rB->c = pCoM - pb
    var rB_C = [
        [c_tilde[0][0] - pb[0][0]],
        [c_tilde[1][0] - pb[1][0]],
        [c_tilde[2][0] - pb[2][0]]
    ];
    //console.log('rB->c: (' + rB_C[0][0] + ', ' + rB_C[1][0] + ', ' + rB_C[2][0] + ')');

    //inertia matrix 3x3 with respect to the CoM defined in the inertia frame
    var I_tilde = interiaMatrix_CoM(robot, c_tilde);
    //console.log('Inertia matrix of the robot: ' + I_tilde);

    //position vector (3x1) from the base frame to the foot frame (end-effector)
    //   rB->F = pfoot - pb
    var rB_F = {
        leg1: [
            [robot.leg1.endEffector.T0[0][3] - pb[0][0]], //x
            [robot.leg1.endEffector.T0[1][3] - pb[1][0]], //y
            [robot.leg1.endEffector.T0[2][3] - pb[2][0]]  //z
        ],
        leg2:  [
            [robot.leg2.endEffector.T0[0][3] - pb[0][0]], //x
            [robot.leg2.endEffector.T0[1][3] - pb[1][0]], //y
            [robot.leg2.endEffector.T0[2][3] - pb[2][0]]  //z
        ],
        leg3: [
            [robot.leg3.endEffector.T0[0][3] - pb[0][0]], //x
            [robot.leg3.endEffector.T0[1][3] - pb[1][0]], //y
            [robot.leg3.endEffector.T0[2][3] - pb[2][0]]  //z
        ],
        leg4:  [
            [robot.leg4.endEffector.T0[0][3] - pb[0][0]], //x
            [robot.leg4.endEffector.T0[1][3] - pb[1][0]], //y
            [robot.leg4.endEffector.T0[2][3] - pb[2][0]]  //z
        ]
    };
    //console.log('rB->F (leg1): (' + rB_F.leg1[0][0] + ',' + rB_F.leg1[1][0] + ',' + rB_F.leg1[2][0] + ')');
    //console.log('rB->F (leg2): (' + rB_F.leg2[0][0] + ',' + rB_F.leg2[1][0] + ',' + rB_F.leg2[2][0] + ')');
    //console.log('rB->F (leg3): (' + rB_F.leg3[0][0] + ',' + rB_F.leg3[1][0] + ',' + rB_F.leg3[2][0] + ')');
    //console.log('rB->F (leg4): (' + rB_F.leg4[0][0] + ',' + rB_F.leg4[1][0] + ',' + rB_F.leg4[2][0] + ')');

    // step 1 and 2. calc. leg inertia matrices under constraint
    //   - leg 1
    //console.log('Leg 1 inertia matrix construction.');
    var imsFleg1 = inertiaMatricesUnderConstraints(robot.leg1);
    //console.log('imsFleg1:');
    //console.log(imsFleg1);
    //   - leg 2
    //console.log('Leg 2 inertia matrix construction.');
    var imsFleg2 = inertiaMatricesUnderConstraints(robot.leg2);
    //console.log('imsFleg2:');
    //console.log(imsFleg2);
    //   - leg 3
    //console.log('Leg 3 inertia matrix construction.');
    var imsFleg3 = inertiaMatricesUnderConstraints(robot.leg3);
    //console.log('imsFleg3:');
    //console.log(imsFleg3);
    //   - leg 4
    //console.log('Leg 4 inertia matrix construction.');
    var imsFleg4 = inertiaMatricesUnderConstraints(robot.leg4);
    //console.log('imsFleg4:');
    //console.log(imsFleg4);
    
    // step 3. calc. of: 
    //   [[m_tilde*E, -1*m_tilde*rB_C],[0, I_tilde]] - ((imsFleg2 * [[E,rB_F],[0,E]]) + (imsFleg1 * [[E,rB_F],[0,E]]) + (imsFleg3 * [[E,rB_F],[0,E]]) + (imsFleg4 * [[E,rB_F],[0,E]]));
    
    //   - skew_symmetric_matrix (see file I:\code\spatial_v2\js\inertia\inertia.js)
    //   IMPORTANT: S() different to that defined in I:\robot\7_Mathematics\books\biped\wholeBody\A Momentum-based Bipedal Balance Controller.ppt
    var rbc = S(rB_C); //skew symmetric matrix for vector from base frame to total CoM of the robot
    
    var rbf = { //skew symmetric matrix for vector from base frame to the leg end-effector
        a: S(rB_F.leg1),
        b: S(rB_F.leg2),
        c: S(rB_F.leg3),
        d: S(rB_F.leg4)
    };
    
    //build [[mE -mrB->c], [0 I]] (excludes 'm' total mass of the robot)
    var partA = [
        [E[0][0],E[0][1],E[0][2],-1.0*rbc[0][0],-1.0*rbc[0][1],-1.0*rbc[0][2]],
        [E[1][0],E[1][1],E[1][2],-1.0*rbc[1][0],-1.0*rbc[1][1],-1.0*rbc[1][2]],
        [E[2][0],E[2][1],E[2][2],-1.0*rbc[2][0],-1.0*rbc[2][1],-1.0*rbc[2][2]],
        [      0,      0,      0, I_tilde[0][0], I_tilde[0][1], I_tilde[0][2]],
        [      0,      0,      0, I_tilde[1][0], I_tilde[1][1], I_tilde[1][2]],
        [      0,      0,      0, I_tilde[2][0], I_tilde[2][1], I_tilde[2][2]],
    ];
    
    //include 'm' the total mass of the robot
    for(var i=0;i<3;i=i+1){ //for the first 3 rows
        for(var j=0;j<6;j=j+1){ //and the 6 columns
            partA[i][j] = m_tilde*partA[i][j]; //multiply by the total mass of the robot
        }
    }
    
    //summation of leg 1, 2, 3, 4 of [[Mleg],[Hleg]]J-1[[E -mrB->Fi], [0 E]]
    var sum1_2 = 
        hlao.matrix_arithmetic(
            hlao.matrix_multiplication(imsFleg1,
            [
                [E[0][0],E[0][1],E[0][2],rbf.a[0][0],rbf.a[0][1],rbf.a[0][2]],
                [E[1][0],E[1][1],E[1][2],rbf.a[1][0],rbf.a[1][1],rbf.a[1][2]],
                [E[2][0],E[2][1],E[2][2],rbf.a[2][0],rbf.a[2][1],rbf.a[2][2]],
                [      0,      0,      0,    E[0][0],    E[0][1],    E[0][2]],
                [      0,      0,      0,    E[1][0],    E[1][1],    E[1][2]],
                [      0,      0,      0,    E[2][0],    E[2][1],    E[2][2]],
            ]),
            hlao.matrix_multiplication(imsFleg2,
            [
                [E[0][0],E[0][1],E[0][2],rbf.b[0][0],rbf.b[0][1],rbf.b[0][2]],
                [E[1][0],E[1][1],E[1][2],rbf.b[1][0],rbf.b[1][1],rbf.b[1][2]],
                [E[2][0],E[2][1],E[2][2],rbf.b[2][0],rbf.b[2][1],rbf.b[2][2]],
                [      0,      0,      0,    E[0][0],    E[0][1],    E[0][2]],
                [      0,      0,      0,    E[1][0],    E[1][1],    E[1][2]],
                [      0,      0,      0,    E[2][0],    E[2][1],    E[2][2]],
            ]),
            '+'
        );
        
    var sum3_4 = 
        hlao.matrix_arithmetic(
            hlao.matrix_multiplication(imsFleg3,
            [
                [E[0][0],E[0][1],E[0][2],rbf.c[0][0],rbf.c[0][1],rbf.c[0][2]],
                [E[1][0],E[1][1],E[1][2],rbf.c[1][0],rbf.c[1][1],rbf.c[1][2]],
                [E[2][0],E[2][1],E[2][2],rbf.c[2][0],rbf.c[2][1],rbf.c[2][2]],
                [      0,      0,      0,    E[0][0],    E[0][1],    E[0][2]],
                [      0,      0,      0,    E[1][0],    E[1][1],    E[1][2]],
                [      0,      0,      0,    E[2][0],    E[2][1],    E[2][2]],
            ]),
            hlao.matrix_multiplication(imsFleg4,
            [
                [E[0][0],E[0][1],E[0][2],rbf.d[0][0],rbf.d[0][1],rbf.d[0][2]],
                [E[1][0],E[1][1],E[1][2],rbf.d[1][0],rbf.d[1][1],rbf.d[1][2]],
                [E[2][0],E[2][1],E[2][2],rbf.d[2][0],rbf.d[2][1],rbf.d[2][2]],
                [      0,      0,      0,    E[0][0],    E[0][1],    E[0][2]],
                [      0,      0,      0,    E[1][0],    E[1][1],    E[1][2]],
                [      0,      0,      0,    E[2][0],    E[2][1],    E[2][2]],
            ]),
            '+'
        );
    
    var partB = hlao.matrix_arithmetic(sum1_2,sum3_4,'+');
    
    //element wise subtraction of [[mE -mrB->c]T [0 I]T]
    var imsB = hlao.matrix_arithmetic(partA,partB,'-');
    //console.log('imsB:');
    //console.log(imsB);
    
    /*
    // step 4. calc. Mfree and Hfree (inertia matrices)
    //   arms (Mfree, Hfree):
    //      - right arm 'ra'
    var imsRightArm = inertiaMatrices(robot.rightArm); // 'ims' Inertia MatriceS.
    //console.log(imsRightArm);
    //inertia matrix (linear momentum) 3xn
    var Mra = imsRightArm.M; //right arm
    //console.log("right arm linear momentum (the 'M' matrix):");
    //console.log('   - length: ' + mtojs.size(Mra,2)); //number of columns
    //console.log(Mra);
    //inertia matrix (angular momentum) 3xn
    var Hra = imsRightArm.H; //right arm
    //console.log("right arm angular momentum (the 'H' matrix):");
    //console.log(Hra);
    
    //      - left arm 'la' and
    var imsLeftArm = inertiaMatrices(robot.leftArm); // 'ims' Inertia MatriceS.
    //console.log(imsLeftArm);
    //inertia matrix (linear momentum) 3xn
    var Mla = imsLeftArm.M; //left arm
    //console.log("left arm linear momentum (the 'M' matrix):");
    //console.log('   - length: ' + mtojs.size(Mla,2)); //number of columns
    //console.log(Mla);
    //inertia matrix (angular momentum) 3xn
    var Hla = imsLeftArm.H; //left arm
    //console.log("left arm angular momentum (the 'H' matrix):");
    //console.log(Hla);
    
    //   head (Mfree, Hfree):
    //      - MfreeH, HfreeH - head 'H'
    */
    
    // step 7. build the 'A' matrix
    var A = [];
    for(var i=0;i<6;i=i+1){ //6 rows to the 'A' matrix
        //reset the 'row' vector
        var row = [];
        
        //push the 'i'th row of the 'imsB' matrix
        for(var j=0;j<6;j=j+1){
            row.push(imsB[i][j]);
        }
        
        /*
        if(i<3){ //push linear momentum matrix
            //push right arm by column
            for(var j=0;j<3;j=j+1){
                row.push(Mra[i][j][0]);
            }
            //push left arm by column
            for(var j=0;j<3;j=j+1){
                row.push(Mla[i][j][0]);
            }
        } else { //push the angular momentum matrix
            //push right arm by column
            for(var j=0;j<3;j=j+1){
                row.push(Hra[i-3][j][0]);
            }
            //push left arm by column
            for(var j=0;j<3;j=j+1){
                row.push(Hla[i-3][j][0]);
            }
        }
        */
        
        //push the 'i'th row
        A.push(row);
    }
    
    //console.log("The 'A' matrix:");
    //console.log(A);
    return A;
}

//inertial matrix as observed from the CoM (defined in the inertial frame)
//IMPORTANT: included end-effector to the global inertia matrix
function interiaMatrix_CoM(robot, c_tilde){
    var IT = hlao.zeros_matrix(3,3); // Total inertia at global CoM defined in the inertial frame.
    
    var openLinks = Object.keys(robot);

    for(var j=0;j<openLinks.length;j=j+1){
        //console.log('Limb: ' + openLinks[j]);
        
        //**links (end-effector below):
        // Get the:
        //   - homogenous transformation matrix in the inertial frame
        //   - mass of each link
        //   - inertia matrix defined in the link frame
        var T0 = robot[openLinks[j]].limb.T0;
        var m = robot[openLinks[j]].limb.dynamicParameters[0]; //mass
        var I_CoM = robot[openLinks[j]].limb.dynamicParameters[1];
        
        // Get the coordinates of the CoM of each link defined in inertial frame.
        var rcom = [];
        for(var i=0;i<(T0.length - 1);i=i+1){
            var x = T0[i][0][3] + (T0[i+1][0][3] - T0[i][0][3])/2;
            var y = T0[i][1][3] + (T0[i+1][1][3] - T0[i][1][3])/2;
            var z = T0[i][2][3] + (T0[i+1][2][3] - T0[i][2][3])/2;
            //console.log('i = ' + i + ': (' + x + ',' + y + ',' + z + ')');
            
            x = c_tilde[0][0] - x; //pCoM - pi, where pCoM is the global robot CoM and pi is the link CoM
            y = c_tilde[1][0] - y;
            z = c_tilde[2][0] - z;
            //console.log('Vector from link CoM to global CoM: [' + x + ', ' + y + ', ' + z + ']');
            rcom.push([[x],[y],[z]]); //position vector from link CoM to the global CoM
        }
        
        // Get the 'R' matrix to transform link 'i' coordinate frame to the inertial frame.    
        var R = [];
        for(var i=0;i<T0.length;i=i+1){
            R.push([
                [T0[i][0][0],T0[i][0][1],T0[i][0][2]],
                [T0[i][1][0],T0[i][1][1],T0[i][1][2]],
                [T0[i][2][0],T0[i][2][1],T0[i][2][2]]
            ]);
        }
        //console.log(R);
        
        //transformation of the link inertia matrix defined in link coordinates to that defined in the inertial frame
        //[I'] = [T][I][T]T
        
        //inertia tensor relative to the overall centre of mass
        //S^T(ci-->ctilde)S(ci-->ctilde)
        
        // I[i] = R[i] x I_CoM[i] x R[i]T + mass x ST(rcom) x S(rcom)
        //   - where rcom is the position vector from link CoM to the global CoM (defined in the inertial frame)
        //   - for examples refer to equation (7.12) and (7.71)
        var I = [];
        for(var i=0;i<(R.length - 1);i=i+1){ //IMPORTANT: the last rotation matrix is dropped
            // Transform link 'i' inertia (frame {i}) to inertial frame {0} (transform to ground-fixed frame (eq. 16.30, REF: A, page 374; eq. 7.12, REF: B, page 251))
            I.push(hlao.matrix_multiplication(R[i],hlao.matrix_multiplication(I_CoM[i],hlao.matrix_transpose(R[i])))); //Transform from frame {i} to frame {0}. eq. 7.12, REF: B, page 251
            I[i] = hlao.matrix_arithmetic(I[i],hlao.matrix_multiplication(hlao.matrix_multiplication_scalar(hlao.matrix_transpose(S(rcom[i])),m[i]),S(rcom[i])),'+'); //Steiner theorem eq. 7.60, REF: B, page 251
            IT = hlao.matrix_arithmetic(IT,I[i],'+');
        }
        
        //**end-effector
        // Get the:
        //   - homogenous transformation matrix in the inertial frame
        //   - mass of each link
        //   - inertia matrix defined in the link frame
        var T0_EE = robot[openLinks[j]].endEffector.T0;
        var m = robot[openLinks[j]].endEffector.dynamicParameters[0]; //mass
        var I_CoM = robot[openLinks[j]].endEffector.dynamicParameters[1];
        
        // Get the coordinates of the CoM of each link defined in inertial frame.
        var n = T0.length;
        var rcom = [ //(column vector)
            [T0[n-1][0][3] + (T0_EE[0][3] - T0[n-1][0][3])/2.0], //x
            [T0[n-1][1][3] + (T0_EE[1][3] - T0[n-1][1][3])/2.0], //y
            [T0[n-1][2][3] + (T0_EE[2][3] - T0[n-1][2][3])/2.0]  //z
        ];
        
        // Get the 'R' matrix to transform link 'i' coordinate frame to the inertial frame.
        var R = [
            [T0_EE[0][0],T0_EE[0][1],T0_EE[0][2]],
            [T0_EE[1][0],T0_EE[1][1],T0_EE[1][2]],
            [T0_EE[2][0],T0_EE[2][1],T0_EE[2][2]]
        ];
        //console.log(R);
        
        // Transform link 'i' inertia (frame {i}) to inertial frame {0} (transform to ground-fixed frame (eq. 16.30, REF: A, page 374; eq. 7.12, REF: B, page 251))
        var I = hlao.matrix_multiplication(R,hlao.matrix_multiplication(I_CoM,hlao.matrix_transpose(R))); //Transform from frame {i} to frame {0}.
        I = hlao.matrix_arithmetic(I,hlao.matrix_multiplication(hlao.matrix_multiplication_scalar(hlao.matrix_transpose(S(rcom)),m),S(rcom)),'+');
        IT = hlao.matrix_arithmetic(IT,I,'+');
    }
    
    //console.log(IT);
    return IT;
}

//inertia matrices under constraints (foot)
//   - pass in per leg
function inertiaMatricesUnderConstraints(openLink){
    var debug = 0;
    
    //inertia matrix linear and angular
    //legs (Mleg, Hleg):
    //   - MlegLL, HlegLL - left leg 'LL' and
    //   - MlegRL, HlegRL - right leg 'RL'
    var ims = inertiaMatrices(openLink); // 'ims' Inertia MatriceS.
    if(debug) console.log(ims);
    
    //inertia matrix (linear momentum) 3xn
    var Mleg = ims.M;
    if(debug){
        console.log("linear momentum (the 'M' matrix):");
        console.log(Mleg);
    }

    //inertia matrix (angular momentum) 3xn
    var Hleg = ims.H;
    if(debug){
        console.log("angular momentum (the 'H' matrix):");
        console.log(Hleg);
    }
    
    //Jacobian matrix (6 x 6) calculated from the leg configuration.
    var Jleg = gJ(openLink); //see also DUMMYgeometricJacobian() from file transformGraph.js
    if(debug){
        console.log('Jacobian matrix:');
        console.log('Jacobian matrix size: ' + mtojs.size(Jleg))
        console.log(Jleg);
    }
    
    JacobianInverse_svdcmp(Jleg); //run diagnostics
    
    var JlegInverse = leftPseudoInverse(Jleg); //pseudo-inverse
    //var JlegInverse = JacobianInverse(Jleg); //square matrix
    if(debug){
        console.log('Jacobian inverse:');
        console.log(JlegInverse);
    }

    //imsF = [[M*Fi],[H*Fi]]*J^-1
    /*
    //6 links
    var MH = [ //IMPORTANT: this is done badly, need a better way to get 'm' and 'h'
        [Mleg[0][0][0],Mleg[1][0][0],Mleg[2][0][0],Mleg[3][0][0],Mleg[4][0][0],Mleg[5][0][0]],
        [Mleg[0][1][0],Mleg[1][1][0],Mleg[2][1][0],Mleg[3][1][0],Mleg[4][1][0],Mleg[5][1][0]],
        [Mleg[0][2][0],Mleg[1][2][0],Mleg[2][2][0],Mleg[3][2][0],Mleg[4][2][0],Mleg[5][2][0]],
        [Hleg[0][0][0],Hleg[1][0][0],Hleg[2][0][0],Hleg[3][0][0],Hleg[4][0][0],Hleg[5][0][0]],
        [Hleg[0][1][0],Hleg[1][1][0],Hleg[2][1][0],Hleg[3][1][0],Hleg[4][1][0],Hleg[5][1][0]],
        [Hleg[0][2][0],Hleg[1][2][0],Hleg[2][2][0],Hleg[3][2][0],Hleg[4][2][0],Hleg[5][2][0]]
    ];
    */
    //4 links
    var MH = [ //IMPORTANT: this is done badly, need a better way to get 'm' and 'h'
        [Mleg[0][0][0],Mleg[1][0][0],Mleg[2][0][0],Mleg[3][0][0]],
        [Mleg[0][1][0],Mleg[1][1][0],Mleg[2][1][0],Mleg[3][1][0]],
        [Mleg[0][2][0],Mleg[1][2][0],Mleg[2][2][0],Mleg[3][2][0]],
        [Hleg[0][0][0],Hleg[1][0][0],Hleg[2][0][0],Hleg[3][0][0]],
        [Hleg[0][1][0],Hleg[1][1][0],Hleg[2][1][0],Hleg[3][1][0]],
        [Hleg[0][2][0],Hleg[1][2][0],Hleg[2][2][0],Hleg[3][2][0]]
    ];
    if(debug){
        console.log('MH matrix:');
        console.log(MH);
    }
    
    var imsF = hlao.matrix_multiplication(MH,JlegInverse);
    
    return imsF;
}

//Jacobian inversion (returns J^-1)
//IMPORTANT: better to use svdcmp() too inspect singular values
function JacobianInverse(J){
    var dim = mtojs.size(J); //get the dimension of the Jacobian matrix
    var m = dim[0]; //number of rows
    
    //   - adjust matrix for inversion (add dummy zeroes to the start of each row)
    for(var j=0;j<m;j=j+1){ //For each row of J[] add an extra element '0.0' to the beginning of the array (beginning of the row).
        J[j].unshift(0.0);
    }
    //   - create an array of zeroes (row vector) to the J[]
    var offsetRow = [];
    for(var j=0;j<(m+1);j=j+1){ //only 6 + 1 elements as J[] is 6 x 6 matrix or 6 x 7 matrix with padded zeroes
        offsetRow.push(0.0);
    }
    J.unshift(offsetRow); //add the row vector of zeroes to the beginning of J[] array - now a 7 x 7 matrix
    //console.log('Adjusted Jacobian (padded with zeroes):');
    //printJacobian(J);

    //   - calc. J^-1
    var Jinverse = ludcmp.matrixInverseLU(J,m);
    
    //   - re-adjust the matrix
    Jinverse.shift();
    
    //   - quick print
    //for(var j=0;j<m;j=j+1){
    //    console.log(Jinverse[j]);
    //}
    
    for(var j=0;j<m;j=j+1){
        Jinverse[j].shift(); //For each row of Jinverse[] remove the first element.
        //console.log(Jinverse[j]);
    }
    //console.log(Jinverse);
    
    return Jinverse;
}

function JacobianInverse_svdcmp(J){
    var dim = mtojs.size(J);
    var m = dim[0]; //matrix rank - number of independent rows (number of rows)
    var n = dim[1]; //number of columns
    //console.log('m: ' + m + ', n: ' + n);

    var Jnull = [];
    for(var j=0;j<m;j=j+1){
        var rowData = [];
            for(var k=0;k<n;k=k+1){
                rowData.push(J[j][k]);
            }
        Jnull.push(rowData);
    }
    //console.log('Jnull:');
    //console.log(Jnull);
    
    //   - adjust matrix for inversion
    for(var j=0;j<m;j=j+1){ //For each row of Jnull[] add an extra element '0.0' to the beginning of the array (beginning of the row).
        Jnull[j].unshift(0.0);
    }
    //   - create an array of zeroes (row vector)
    var offsetRow = [];
    for(var j=0;j<(n+1);j=j+1){
        offsetRow.push(0.0);
    }
    Jnull.unshift(offsetRow); //add the row vector of zeroes to the beginning of Jnull[] array - now a 7 x 7 matrix

    //console.log(mtojs.size(Jnull));
    //console.log(Jnull);

    var w = hlao.zeros_vector((n+1),'row'); //row vector where index = 0 is undefined
    var v = hlao.zeros_matrix((n+1),(n+1)); //matrix
    var uwv = svdcmp.svdcmp(Jnull, m, n, w, v);
    //console.log(uwv);

    // Get the singular values (positive or zero).
    var u = uwv[0];
    //console.log("Size of 'u' (not-cleaned):" + mtojs.size(u));
    //remove dummy row
    u.shift();
    //remove the dummy column
    for(var i=0;i<m;i=i+1){ //'m' because we are going by row.
        u[i].shift(0.0);
    }
    //console.log("Size of 'u' (cleaned): " + mtojs.size(u));
    //console.log('Cleaned inverse:');
    //console.log(u);
    
    var w = uwv[1];
    w.shift(); // Drop the first element in the array as it is zero.
    //console.log('W = ' + w);

    // Get the rank.
    var rank = hlao.matrix_rank(w);
    //console.log('Matrix rank = ' + rank);

    // Get the condition number of the matrix.
    var w_min = +1e6;
    var w_max = -1e6;
    for(var j=0;j<w.length;j=j+1){
        if(w_max < w[j]) w_max = w[j];
        if((w_min > w[j]) && (w[j] > 1e-6)) w_min = w[j]; // w_min must be non zero.
    }
    var cond_numb = w_max/w_min;
    //console.log('sigma 1 = ' + w_max);
    //console.log('sigma r = ' + w_min);
    //console.log('Matrix condition number = ' + cond_numb);
    if(cond_numb > 900.0) console.log('WARNING: Condition number: ' + cond_numb);
}


//geometric Jacobian
function gJ(openLink){
    //get the transforms defined in the inertial frame
    var T0 = openLink.limb.T0;
    var n = T0.length;
    
    //revolute equ. (3.30) (REF: Robotics Modelling, Planning and Control, Page 112 (section 3.1.3))
    var JP = []; //contribution to the linear velocity
    var JO = []; //contribution to the angular velocity
    var J = []; //Jacobian
    var z = []; //unit vector of the joint axis
    var p = []; //position vector of the joint
    
    var pe = [[openLink.endEffector.T0[0][3]], [openLink.endEffector.T0[1][3]], [openLink.endEffector.T0[2][3]]];
    
    for(var i=0;i<n;i=i+1){
        p.push([[T0[i][0][3]], [T0[i][1][3]], [T0[i][2][3]]]); //hlao.matrix_multiplication(T0[i],p0) where p0 = [[0.0],[0.0],[0.0],[1.0]], see (Equ. 3.33)
        z.push([[T0[i][0][2]], [T0[i][1][2]], [T0[i][2][2]]]); //hlao.matrix_multiplication(T0[i],z0) where z0 = [[0.0],[0.0],[1.0]], see (equ. 3.31)
    }
    
    for(var i=1;i<=n;i=i+1){
        //   - linear velocity
        JP[i] = hlao.vector_cross(z[i-1],hlao.matrix_arithmetic(pe,p[i-1],'-'));
        //   - angular velocity
        JO[i] = z[i-1];
        
        J.push([JP[i][0][0],JP[i][1][0],JP[i][2][0],JO[i][0][0],JO[i][1][0],JO[i][2][0]]); //row vector
    }
    
    //console.log('J before transpose:');
    //printJacobian(J);
    J = hlao.matrix_transpose(J); //convert each row vector into a column vector
    //console.log(J);
    
    return J;
}

//print the Jacobian
function printJacobian(J){
    var dim = mtojs.size(J); //get the dimension of the Jacobian matrix
    var m = dim[0]; //number of rows
    var n = dim[1]; //number of columns
    
    var Jstr = "";
    for(var j=0;j<m;j=j+1){ //row
        for(var k=0;k<n;k=k+1){ //col
            Jstr = Jstr + J[j][k] + ' ';
        }
        Jstr = Jstr + '\n';
    }
    console.log(Jstr);
}

//Left Pseudo Inverse of the 'J' matrix (ref: RoboticsVisionAndControl.pdf, page 183)
/*
Javascript:
var J = [
    [  0.0, -0.086,     0, 0, 0, 1], 
    [-0.16,      0, 0.057, 0, 1, 0], 
    [-0.16,      0,     0, 0, 1, 0], 
    [-0.08,      0,     0, 0, 1, 0]
];
var JT = hlao.matrix_transpose(J); 
var lpi = leftPseudoInverse(JT);
    0 -0.0854      0 0    0 0.993 
    0       0  17.54 0    0 0 
-12.5       0 -17.54 0 -1.0 0 
 12.5       0      0 0  2.0 0 
 
MATLAB:
J = [
    [  0.0, -0.086,     0, 0, 0, 1], 
    [-0.16,      0, 0.057, 0, 1, 0], 
    [-0.16,      0,     0, 0, 1, 0], 
    [-0.08,      0,     0, 0, 1, 0]
];
JT = J';
lpi = pinv(JT);
         0   -0.0854         0         0         0    0.9927
         0    0.0000   17.5439         0    0.0000   -0.0000
  -12.5000   -0.0000  -17.5439         0   -1.0000   -0.0000
   12.5000    0.0000    0.0000         0    2.0000    0.0000
*/
function leftPseudoInverse(J){
    var debug = 0;
    
    var dim = mtojs.size(J);
    var m = dim[0]; //'m' rows
    var n = dim[1]; //'n' columns
    if(debug) console.log("'J' size: " + mtojs.size(J));
    
    var JT = hlao.matrix_transpose(J); //transpose the 'J' matrix
    var JTJ = hlao.matrix_multiplication(JT,J);
    var dim_sq = mtojs.size(JTJ);
    var m_sq = dim_sq[0]; //'m' rows
    var n_sq = dim_sq[1]; //'n' columns
    if(m_sq !== n_sq) console.log('WARNING: J^T.J matrix is not square.');
    
    //insert dummy value into start of each row vector
    for(var i=0;i<m_sq;i=i+1){
        JTJ[i].unshift(0.0);
    }
    //insert a dummy row
    var offsetRow = [];
    for(var i=0;i<(n_sq+1);i=i+1){ //one extract column added of dummy values
        offsetRow.push(0.0);
    }
    JTJ.unshift(offsetRow);
    if(debug){
        console.log("Size of 'J^T.J': " + mtojs.size(JTJ));
        console.log('Padded J^T.J');
        console.log(JTJ);
    }
    
    //calc. the inverse
    var JTJinverse = ludcmp.matrixInverseLU(JTJ,m_sq);
    if(debug){
        console.log('Inverse of J^T.J');
        console.log(JTJinverse);
    }
    
    //remove dummy row
    JTJinverse.shift();
    //remove the dummy column
    for(var i=0;i<m_sq;i=i+1){ //'m' dropping element per row.
        JTJinverse[i].shift(0.0);
    }
    if(debug){
        console.log('Cleaned inverse of J^T.J');
        console.log(JTJinverse);
    }
    
    //do final calc.
    var Jdagger = hlao.matrix_multiplication(JTJinverse,JT);
    
    return Jdagger;
}

//D(r), equation (26)
//   - input, two column vectors.
function D(a,b){
    //console.log('D (equ.(26)):');
    //console.log(a);
    //console.log(b);
    var r0 = a[0][0] - b[0][0];
    var r1 = a[1][0] - b[1][0];
    var r2 = a[2][0] - b[2][0];
    //IMPORTANT: ref: A Momentum-based Bipedal Balance Controller.ppt defines the skew symmetric matrix differently to function S().
    var D = hlao.matrix_multiplication(hlao.matrix_transpose([[ 0.0, r2, -1.0*r1],[-1.0*r2, 0.0, r0],[r1, -1.0*r0, 0.0]]),[[ 0.0, r2, -1.0*r1],[-1.0*r2, 0.0, r0],[r1, -1.0*r0, 0.0]]);
    return D;
}

//calculation of the inertia matrices (of all link structure driven by joint 'j')
//   - ref: figure 2 link configuration (Resolved Momentum Control: Humanoid Motion Planning
//          based on the Linear and Angular Momentum).
//   - input:
//      - m, mass of each link
//      - I, inertia matrix of each link defined at the CoM of the link with respect to the link reference frame
function inertiaMatrices(openLink){
    //T0, mass, I_CoM, EE_m, EE_c, EE_I_CoM, EE_R
    var T0 = openLink.limb.T0;
    var n = T0.length;
    //console.log(n);
    
    var mass = openLink.limb.dynamicParameters[0];
    var I_CoM = openLink.limb.dynamicParameters[1];
    var T0_EE = openLink.endEffector.T0;
    var EE_m = openLink.endEffector.dynamicParameters[0];
    var EE_I_CoM = openLink.endEffector.dynamicParameters[1];
    var EE_R = [
        [T0_EE[0][0],T0_EE[0][1],T0_EE[0][2]],
        [T0_EE[1][0],T0_EE[1][1],T0_EE[1][2]],
        [T0_EE[2][0],T0_EE[2][1],T0_EE[2][2]]
    ];
    //CoM of the end-effector
    var EE_c = [ //(column vector)
        [T0[n-1][0][3] + (T0_EE[0][3] - T0[n-1][0][3])/2.0],
        [T0[n-1][1][3] + (T0_EE[1][3] - T0[n-1][1][3])/2.0],
        [T0[n-1][2][3] + (T0_EE[2][3] - T0[n-1][2][3])/2.0]
    ];

    //inertia matrix (linear momentum) 3xn
    var M = [];

    //inertia matrix (angular momentum) 3xn
    var H = [];

    //position vector of the CoM of each link defined in inertial frame with reference to the interial frame
    var c = [];
    
    //rotation axis vector of joint 'j'
    var a = [];
    
    //position vector of joint 'j' with respect to the inertial reference frame defined in the inertial reference frame
    var r = [];
    
    // Get the 'R' matrix to transform link 'i' coordinate frame to the inertial reference frame.    
    var R = [];
    
    for(var i=0; i<n; i=i+1){
        if(i < (n-1)){
            c.push([ //(column vector)
                [T0[i][0][3] + (T0[i+1][0][3] - T0[i][0][3])/2.0],
                [T0[i][1][3] + (T0[i+1][1][3] - T0[i][1][3])/2.0],
                [T0[i][2][3] + (T0[i+1][2][3] - T0[i][2][3])/2.0]
            ]);
        }
        
        a.push([[T0[i][0][2]], [T0[i][1][2]], [T0[i][2][2]]]); //(column vector)
        
        r.push([[T0[i][0][3]], [T0[i][1][3]], [T0[i][2][3]]]); //(column vector)
        
        R.push([
            [T0[i][0][0],T0[i][0][1],T0[i][0][2]],
            [T0[i][1][0],T0[i][1][1],T0[i][1][2]],
            [T0[i][2][0],T0[i][2][1],T0[i][2][2]]
        ]);
    }
    
    //console.log(c);
    //console.log(a);
    //console.log(r);
    //console.log(R);
    
    //mass (all link structure driven by joint 'j', see equation (23) - extremity to body side)
    var m_tilde = [];
    
    //centre of mass (CoM) (all link structure driven by joint 'j', see equation (24) - extremity to body side)
    var c_tilde = [];
    
    //inertia tensor (all link structure driven by joint 'j', see equation (25) - extremity to body side)
    var I_tilde = [];
    
    //the first outer link
    c.push();
    var j = (n-1);
    m_tilde[j] = EE_m; //mass of end-effector
    c_tilde[j] = EE_c; //position vector (column vector) of the CoM , , 
    I_tilde[j] = hlao.matrix_multiplication(EE_R,hlao.matrix_multiplication(EE_I_CoM,hlao.matrix_transpose(EE_R))); //transform I_CoM from link 'i' frame to the inertial reference frame
    //console.log('End-effector properties:');
    //console.log(m_tilde[j]);
    //console.log(c_tilde[j]);
    //console.log(I_tilde[j]);
    
    for(var j=(n-1);j>=1;j=j-1){
        //console.log(j);
        m_tilde[j-1] = m_tilde[j] + mass[j-1]; //equ. (23)
        c_tilde[j-1] = 
            hlao.vector_multiplication_scalar(
                hlao.matrix_arithmetic(hlao.vector_multiplication_scalar(c_tilde[j],m_tilde[j]),hlao.vector_multiplication_scalar(c[j-1],mass[j-1]),'+'),
                (1.0/(m_tilde[j] + mass[j-1]))
            ); //equ. (24)
        I_tilde[j-1] = 
            hlao.matrix_arithmetic(
                I_tilde[j], 
                hlao.matrix_arithmetic(
                    hlao.matrix_multiplication_scalar(D(c_tilde[j],c_tilde[j-1]),m_tilde[j]), 
                    hlao.matrix_arithmetic(
                        hlao.matrix_multiplication(R[j-1],hlao.matrix_multiplication(I_CoM[j-1],hlao.matrix_transpose(R[j-1]))), 
                        hlao.matrix_multiplication_scalar(D(c[j-1],c_tilde[j-1]),mass[j-1]),
                    '+'),
                '+'),
            '+'); //equ. (25)
    }
    
    //console.log('m, c, I tilde');
    //console.log(m_tilde);
    //console.log(c_tilde);
    //console.log(I_tilde);
    
    //build 'm' (linear) and 'h' (angular) both 3x1
    for(var j=0;j<n;j=j+1){
        var m = hlao.matrix_multiplication_scalar(hlao.vector_cross(a[j],hlao.matrix_arithmetic(c_tilde[j],r[j],'-')),m_tilde[j]); //equ.(18)
        var h = hlao.matrix_arithmetic(hlao.vector_cross(c_tilde[j],m),hlao.matrix_multiplication(I_tilde[j],a[j]),'+'); //equ.(19)
        M.push(m);
        //H.push(h);
        //console.log("'m' (linear): [[" + m[0][0] + '][' + m[1][0] + '][' + m[2][0] + ']');
        //console.log("'h' (angular): [[" + h[0][0] + '][' + h[1][0] + '][' + h[2][0] + ']');
        
        H.push(hlao.matrix_arithmetic(h,hlao.matrix_multiplication(S(c_tilde),m),'-')); //equ.(21)
    }
    
    return {M:M,H:H};
}

export {
    T_BaseFrame,
    S,
    quad,
    RMC,
    legJointSpeeds,
    updateTransforms,
    feetVelocities_right,
    feetVelocities_left,
    dynamicParameters,
    endEffector,
    I_CoM,
    totalMass,
    totalCoM,
    plotPoint,
    targetSpeed,
    y_vector,
    AdaggerMatrix,
    theAmatrix,
    interiaMatrix_CoM,
    inertiaMatricesUnderConstraints,
    JacobianInverse,
    JacobianInverse_svdcmp,
    gJ,
    printJacobian,
    leftPseudoInverse,
    D,
    inertiaMatrices
};
