% 0 initialization
toPause = true;
getJointAvailable = false;
toPublish = false;
useGripperController = false;
T = 10;
offset = 4;
off = [-0.1,0,0.15]';
pregrasp_pos = [0.75,-0.5,0.7]'+off; % pregrasp pose (x,y,z)
pregrasp_orient =  angle2quat(0,0,-0.1)';  

grasp_pos = [0.8,-0.5,0.7]'+off; % grasp pose (x,y,z)
grasp_orient =  angle2quat(0,0,-0.1)';  

hold1_pos = [0.8,-0.5,0.75]'+off; % grasp pose (x,y,z)
hold1_orient =  angle2quat(0,0,-0.1)';  

hold2_pos = [0.8,-0.5,0.75]'+off; % grasp pose (x,y,z)
hold2_orient =  angle2quat(0,0,1.57)'; 

hold3_pos = [0.8,-0.7,0.75]'+off; % grasp pose (x,y,z)
hold3_orient =  angle2quat(0,0,1.57)'; 

put_pos = [0.8,-0.7,0.7]'+off; % grasp pose (x,y,z)
put_orient =  angle2quat(0,0,1.57)'; 

postput_pos = [0.75,-0.5,0.7]'+off; % grasp pose (x,y,z)
postput_orient =  angle2quat(0,0,1.57)'; 

basefixed = true;
torsofixed = true;

warning('off','all');
DRAKE_PATH = '/home/drc/drc/software/drake';
if(~exist('r','var'))
  fprintf('Loading Robot Model...');
  r = RigidBodyManipulator(strcat(DRAKE_PATH,'/examples/PR2/pr2.urdf'),struct('floating',true));
  fprintf('Done\n');
end
planner = pr2Planner(r);
dof = r.getNumDOF;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 1. Prepare
disp('1. Prepare');
%  -1 Get Current joints
if getJointAvailable  
  q0 = getCurrentQfromLCM();
else
  q0 = planner.getPrepareQ();
end
%  -2 Set destination to prepare
qdest = planner.getPrepareQ();

%  -3 Create joint plan
[xtraj,snopt_info,infeasible_constraint,q_end] = ...
    planner.createJointPlan(q0,qdest,T,basefixed,torsofixed);
%  -4 Play it in viewer
planner.v.playback(xtraj,'slider');
mypause()
%  -5 Publish
if toPublish
  planner.publishTraj(xtraj,snopt_info);
  mypause()
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2. Pregrasp
display('2. Pregrasp');
%  -1 Get Current joints
if getJointAvailable  
  q0 = getCurrentQfromLCM();
else
  q0 = q_end(1:dof,1); %% todo
end
%  -2 Set destination to reach
pos_final_xyz = pregrasp_pos;
pos_final_orient = pregrasp_orient;  % (yaw, pitch, roll)

%  -3 Create joint plan
keepSameOrient = true;
[xtraj,snopt_info,infeasible_constraint,q_end] = ...
            planner.createPointPlanWOrient(q0, pos_final_xyz, pos_final_orient, T, ...
            basefixed, torsofixed,keepSameOrient);
        
%  -4 Play it in viewer
planner.v.playback(xtraj,'slider');
mypause()
%  -5 Publish
if toPublish
  planner.publishTraj(xtraj,snopt_info);
  mypause()
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% open gripper and wait
display('open gripper and wait');
q0 = q_end(1:dof,1);
% Set destination to open gripper
qdest = q0;
qdest(r.findJointInd('r_gripper_l_finger_joint')+offset) = 0.2;  %r_gripper_l_finger_joint
qdest(r.findJointInd('r_gripper_r_finger_joint')+offset) = 0.2;  %r_gripper_l_finger_joint

% Create joint plan
[xtraj,snopt_info,infeasible_constraint,q_end] = ...
  planner.createJointPlan(q0,qdest,T,basefixed,torsofixed);
% Play it in viewer
planner.v.playback(xtraj);
mypause()
% Publish
if useGripperController
  system('rosrun simple_gripper simple_gripper open');
  mypause()
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 4. Go grasp
display('4. Go grasp');
%  -1 Get Current joints
if getJointAvailable  
  q0 = getCurrentQfromLCM();
else
  q0 = q_end(1:dof,1); 
end
%  -2 Set destination to drawer open pos
pos_final_xyz = grasp_pos;
pos_final_orient = grasp_orient;  % (yaw, pitch, roll)
keepSameOrient = true;
%  -3 Create line plan
[xtraj,snopt_info,infeasible_constraint,q_end] = ...
            planner.createLinePlanWOrient(q0, pos_final_xyz, pos_final_orient, T, ...
            basefixed, torsofixed, keepSameOrient);
%  -4 Play it in viewer
planner.v.playback(xtraj);
mypause()
%  -5 Publish
if toPublish
  planner.publishTraj(xtraj,snopt_info);
  mypause()
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% close gripper and wait
display('close gripper and wait');
q0 = q_end(1:dof,1);

% 1.2 Set destination to prepare
qdest = q0;
qdest(r.findJointInd('r_gripper_l_finger_joint')+offset) = 0.1;  %r_gripper_l_finger_joint
qdest(r.findJointInd('r_gripper_r_finger_joint')+offset) = 0.1;  %r_gripper_l_finger_joint

% 1.3 Create joint plan
[xtraj,snopt_info,infeasible_constraint,q_end] = ...
    planner.createJointPlan(q0,qdest,T,basefixed,torsofixed);
% 1.4 Play it in viewer
planner.v.playback(xtraj);
mypause()
% 1.5 Publish
if useGripperController
  system('rosrun simple_gripper simple_gripper close');
  mypause()
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 5. Pick up
display('5. Pick up');
%  -1 Get Current joints
if getJointAvailable  
  q0 = getCurrentQfromLCM();
else
  q0 = q_end(1:dof,1); 
end
%  -2 Set destination to drawer open pos
pos_final_xyz = hold1_pos;
pos_final_orient = hold1_orient;  % (yaw, pitch, roll)
keepSameOrient = false;
%  -3 Create line plan
[xtraj,snopt_info,infeasible_constraint,q_end] = ...
            planner.createLinePlanWOrient(q0, pos_final_xyz, pos_final_orient, T, ...
            basefixed, torsofixed, keepSameOrient);
%  -4 Play it in viewer
planner.v.playback(xtraj);
mypause()
%  -5 Publish
if toPublish
  planner.publishTraj(xtraj,snopt_info);
  mypause()
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 6. Rotate
display('6. Rotate');
%  -1 Get Current joints
if getJointAvailable  
  q0 = getCurrentQfromLCM();
else
  q0 = q_end(1:dof,1); 
end
%  -2 Set destination to drawer open pos
pos_final_xyz = hold2_pos;
pos_final_orient = hold2_orient;  % (yaw, pitch, roll)
keepSameOrient = true;
%  -3 Create line plan
[xtraj,snopt_info,infeasible_constraint,q_end] = ...
            planner.createLinePlanWOrient(q0, pos_final_xyz, pos_final_orient, T, ...
            basefixed, torsofixed, keepSameOrient);
%  -4 Play it in viewer
planner.v.playback(xtraj);
mypause()
%  -5 Publish
if toPublish
  planner.publishTraj(xtraj,snopt_info);
  mypause()
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 7. Move to preput (stay with same orientation)
display('7. Move to preput');
%  -1 Get Current joints
if getJointAvailable  
  q0 = getCurrentQfromLCM();
else
  q0 = q_end(1:dof,1); 
end
%  -2 Set destination to drawer open pos
pos_final_xyz = hold3_pos;
pos_final_orient = hold3_orient;  % (yaw, pitch, roll)
keepSameOrient = true;
%  -3 Create line plan
[xtraj,snopt_info,infeasible_constraint,q_end] = ...
            planner.createPointPlanWOrient(q0, pos_final_xyz, pos_final_orient, T, ...
            basefixed, torsofixed, keepSameOrient);
%  -4 Play it in viewer
planner.v.playback(xtraj);
mypause()
%  -5 Publish
if toPublish
  planner.publishTraj(xtraj,snopt_info);
  mypause()
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 8. Put
display('8. Put');
%  -1 Get Current joints
if getJointAvailable  
  q0 = getCurrentQfromLCM();
else
  q0 = q_end(1:dof,1); 
end
%  -2 Set destination to drawer open pos
pos_final_xyz = put_pos;
pos_final_orient = put_orient;  % (yaw, pitch, roll)
keepSameOrient = true;
%  -3 Create line plan
[xtraj,snopt_info,infeasible_constraint,q_end] = ...
            planner.createLinePlanWOrient(q0, pos_final_xyz, pos_final_orient, T, ...
            basefixed, torsofixed, keepSameOrient);
%  -4 Play it in viewer
planner.v.playback(xtraj);
%  -5 Publish
planner.publishTraj(xtraj,snopt_info);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% open gripper and wait
display('open gripper and wait');
q0 = q_end(1:dof,1);
% Set destination to open gripper
qdest = q0;
qdest(r.findJointInd('r_gripper_l_finger_joint')+offset) = 0.2;  %r_gripper_l_finger_joint
qdest(r.findJointInd('r_gripper_r_finger_joint')+offset) = 0.2;  %r_gripper_l_finger_joint

% Create joint plan
[xtraj,snopt_info,infeasible_constraint,q_end] = ...
  planner.createJointPlan(q0,qdest,T,basefixed,torsofixed);
% Play it in viewer
planner.v.playback(xtraj);
mypause()
% Publish
if useGripperController
  system('rosrun simple_gripper simple_gripper open');
  mypause()
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 9. Postput
display('9. Postput');
%  -1 Get Current joints
if getJointAvailable  
  q0 = getCurrentQfromLCM();
else
  q0 = q_end(1:dof,1); 
end
%  -2 Set destination to drawer open pos
pos_final_xyz = postput_pos;
pos_final_orient = postput_orient;  % (yaw, pitch, roll)
keepSameOrient = true;
%  -3 Create line plan
[xtraj,snopt_info,infeasible_constraint,q_end] = ...
            planner.createLinePlanWOrient(q0, pos_final_xyz, pos_final_orient, T, ...
            basefixed, torsofixed, keepSameOrient);
%  -4 Play it in viewer
planner.v.playback(xtraj);
mypause()
%  -5 Publish
if toPublish
  planner.publishTraj(xtraj,snopt_info);
  mypause()
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 10. Back to Prepare
display('10. Back to Prepare');
%  -1 Get Current joints
if getJointAvailable  
  q0 = getCurrentQfromLCM();
else
  q0 = q_end(1:dof,1); 
end
%  -2 Set destination to prepare
qdest = planner.getPrepareQ();

%  -3 Create joint plan
[xtraj,snopt_info,infeasible_constraint,q_end] = ...
    planner.createJointPlan(q0,qdest,T,basefixed,torsofixed);
%  -4 Play it in viewer
planner.v.playback(xtraj);
mypause()
%  -5 Publish
if toPublish
  planner.publishTraj(xtraj,snopt_info);
  mypause()
end