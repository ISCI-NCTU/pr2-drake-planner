
% 0 initialization
toPause = true;
getJointAvailable = false;
toPublish = true;
useGripperController = true;
T = 10;
offset = 4;
%drawer_close_pos = [0.62,-0.5,0.375]'; % specify drawer pose (x,y,z)
%drawer_open_pos =  [0.4,-0.5,0.375]'; 

%drawer_close_pos = [0.62,-0.52,0.45]'; % specify drawer pose (x,y,z)
if getJointAvailable  
    q0 = getCurrentQfromLCM();
end

% real pr2 blue position
drawer_open_pos =  [0.4,-0.52,0.45]'; 

grasp_pos = drawer_close_pos; % pregrasp pose (x,y,z)
grasp_orient =  angle2quat(0,0,-0.1)'; 

pregrasp_pos = drawer_close_pos - [0.05,0,0]'; % pregrasp pose (x,y,z)
pregrasp_orient = grasp_orient; 

release_pos = drawer_open_pos; 
release_orient =  angle2quat(0,0,-0.1)'; 

postrelease_pos = drawer_open_pos - [0.05,0,0]'; % pregrasp pose (x,y,z)
postrelease_orient = grasp_orient; 

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
% 1. Set to prepare pose
disp('1. Prepare');
% 1.1 Get Current joints
if getJointAvailable  
  q0 = getCurrentQfromLCM();
else
  q0 = planner.getPrepareQ();
end
% 1.2 Set destination to prepare
qdest = planner.getPrepareQ();

% 1.3 Create joint plan
[xtraj,snopt_info,infeasible_constraint,q_end] = ...
    planner.createJointPlan(q0,qdest,T,basefixed,torsofixed);
% 1.4 Play it in viewer
planner.v.playback(xtraj);
mypause()
% 1.5 Publish
if toPublish
  planner.publishTraj(xtraj,snopt_info);
  mypause()
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% open gripper and wait
disp('Open gripper');
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
  planner.publishTraj(xtraj,snopt_info);
  system('rosrun simple_gripper simple_gripper open');
  mypause()
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2 Pregrasp
disp('Go to Pregrasp pose');
% 2.1 Get Current joints
if getJointAvailable  
  q0 = getCurrentQfromLCM();
else
  q0 = q_end(1:dof,1); %% todo
end
% 2.2 Set destination to reach
pos_final_xyz = pregrasp_pos;
pos_final_orient = pregrasp_orient;  % (yaw, pitch, roll)

% 2.3 Create joint plan
keepSameOrient = false;
addCollision = true;
[xtraj,snopt_info,infeasible_constraint,q_end] = ...
            planner.createPointPlanWOrient(q0, pos_final_xyz, pos_final_orient, T, ...
            basefixed, torsofixed, keepSameOrient, addCollision);

% debug
ts = xtraj.pp.breaks;
q = xtraj.eval(ts);
dof = r.getNumDOF;
rpos = [];
for i=1:length(ts)
  kinsol = r.doKinematics(q(1:dof,i));
  r_gripper_pt = [0.18,0,0]';
  reachpos = r.forwardKin(kinsol,findLinkInd(r,'r_gripper_palm_link'),r_gripper_pt); 
  rpos = [rpos reachpos ];
end
rpos
         
% 2.4 Play it in viewer
planner.v.playback(xtraj);
mypause()
% 2.5 Publish
if toPublish
  planner.publishTraj(xtraj,snopt_info);
  mypause()
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 3 Go grasp
disp('3. Go to grasp pose');
% 3.1 Get Current joints
if getJointAvailable  
  q0 = getCurrentQfromLCM();
else
  q0 = q_end(1:dof,1); 
end
% 3.2 Set destination to drawer open pos
pos_final_xyz = grasp_pos;
pos_final_orient = grasp_orient;  % (yaw, pitch, roll)
keepSameOrient = true;
% 3.3 Create line plan
[xtraj,snopt_info,infeasible_constraint,q_end] = ...
            planner.createLinePlanWOrient(q0, pos_final_xyz, pos_final_orient, T, ...
            basefixed, torsofixed, keepSameOrient);
% 3.4 Play it in viewer
planner.v.playback(xtraj);
mypause()
% 3.5 Publish
if toPublish
  planner.publishTraj(xtraj,snopt_info);
  mypause()
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% close gripper and wait
disp('Close gripper');
q0 = q_end(1:dof,1);

%  -2 Set destination to prepare
qdest = q0;
qdest(r.findJointInd('r_gripper_l_finger_joint')+offset) = 0.1;  %r_gripper_l_finger_joint
qdest(r.findJointInd('r_gripper_r_finger_joint')+offset) = 0.1;  %r_gripper_l_finger_joint

%  -3 Create joint plan
[xtraj,snopt_info,infeasible_constraint,q_end] = ...
    planner.createJointPlan(q0,qdest,T,basefixed,torsofixed);
%  -4 Play it in viewer
planner.v.playback(xtraj);
mypause()
%  -5 Publish
if useGripperController
  planner.publishTraj(xtraj,snopt_info);
  system('rosrun simple_gripper simple_gripper close');
  mypause()
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 4. Open the drawer
disp('4. Open the drawer');
%  -1 Get Current joints
if getJointAvailable  
  q0 = getCurrentQfromLCM();
else
  q0 = q_end(1:dof,1); 
end
%  -2 Set destination to drawer open pos
pos_final_xyz = release_pos;
pos_final_orient = release_orient;  % (yaw, pitch, roll)
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Open gripper and wait
% Open Gripper
disp('Open Gripper');
if getJointAvailable  
  q0 = getCurrentQfromLCM();
else
  q0 = q_end(1:dof,1);
end
% 1.2 Set destination to prepare
qdest = q0;
qdest(r.findJointInd('r_gripper_l_finger_joint')+offset) = 0.2;  %r_gripper_l_finger_joint
qdest(r.findJointInd('r_gripper_r_finger_joint')+offset) = 0.2;  %r_gripper_l_finger_joint

% 1.3 Create joint plan
[xtraj,snopt_info,infeasible_constraint,q_end] = ...
    planner.createJointPlan(q0,qdest,T,basefixed,torsofixed);
% 1.4 Play it in viewer
planner.v.playback(xtraj);
mypause()
% 1.5 Publish
if useGripperController
  planner.publishTraj(xtraj,snopt_info);
  system('rosrun simple_gripper simple_gripper open');
  mypause()
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 5. Move to postrelease
disp('5. Move to postrelease');
%  -1 Get Current joints
if getJointAvailable  
  q0 = getCurrentQfromLCM();
else
  q0 = q_end(1:dof,1); 
end
%  -2 Set destination to drawer open pos
pos_final_xyz = postrelease_pos;
pos_final_orient = postrelease_orient;  % (yaw, pitch, roll)
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 6. Back to prepare
disp('6. Back to prepare');
%  -1 Get Current joints
if getJointAvailable  
  q0 = getCurrentQfromLCM()
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

