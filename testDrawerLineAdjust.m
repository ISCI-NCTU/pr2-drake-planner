
% 0 initialization
toPause = true;
getJointAvailable = false;
toPublish = false;
useGripperController = false;
T = 10;
offset = 4;
leftOrRight = 'r';  % left hand or right hand

% real drawer knob position
drawer_close_pos = [0.82,-0.52,1.19]'; % specify drawer pose (x,y,z)
drawer_open_pos =  [0.4,-0.52,1.19]'; 
drawer_close_pos = [0.82782; -0.18899; 0.79926+0.02]; % specify drawer pose (x,y,z)
drawer_open_pos =  [0.82782-0.2; -0.18899; 0.79926+0.02];

grasp_pos = drawer_close_pos; % pregrasp pose (x,y,z)
grasp_orient =  angle2quat(0,0,1.57-0.18)';   % -0.18 to make it horizontal

pregrasp_pos = drawer_close_pos - [0.05,0,0]'; % pregrasp pose (x,y,z)
pregrasp_orient = grasp_orient; 

release_pos = drawer_open_pos; 
release_orient =  angle2quat(0,0,1.57-0.18)'; 

postrelease_pos = drawer_open_pos - [0.05,0,0]'; % pregrasp pose (x,y,z)
postrelease_orient = grasp_orient; 

basefixed = true;
torsofixed = true;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Initialize the planner with robot model
warning('off','all');
if(~exist('r','var'))
  fprintf('Loading Robot Model...');
  r = RigidBodyManipulator(strcat(getDrakePath(),'/examples/PR2/pr2.urdf'), ...
      struct('floating',true));
  %r = RigidBodyManipulator('/home/drc/pr2data/pr2mm.urdf',struct('floating',true));
  fprintf('Done\n');
end
planner = pr2Planner(r, leftOrRight);
dof = r.getNumDOF;
planner.leftOrRight = leftOrRight;  

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%1. Set to prepare pose
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
  controlGripper('open', leftOrRight);
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
  %reachpos = r.forwardKin(kinsol,findLinkInd(r,'r_gripper_palm_link'),r_gripper_pt); 
  reachpos = r.forwardKin(kinsol,findLinkInd(r,sprintf('%s_gripper_tool_frame', leftOrRight)),[0,0,0]'); 
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
            planner.createLinePlanWOrient(q0, pos_final_xyz, pos_final_orient, T*0.2, ...
            basefixed, torsofixed, keepSameOrient, 5);
%  -4 Play it in viewer
planner.v.playback(xtraj);
mypause()
%  -5 Publish
if toPublish
  planner.publishTraj(xtraj,snopt_info);
  mypause()
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% close gripper and wait
disp('Close gripper');
q0 = q_end(1:dof,1);

%  -2 Set destination to prepare for visualization
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
  %planner.publishTraj(xtraj,snopt_info);
  controlGripper('close', leftOrRight);
  mypause()
end





%testPID

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 4. Open the drawer with PID

disp('4. Open the drawer');
%  -1 Get Current joints
if getJointAvailable  
  q0 = getCurrentQfromLCM();
else
  q0 = q_end(1:dof,1); 
end
%  -2 Set destination to drawer open pos

kinsol = r.doKinematics(q0(1:dof,1));
gripper_pt = [0.18,0,0]';
currpos = r.forwardKin(kinsol,findLinkInd(r,sprintf('%s_gripper_palm_link', leftOrRight)), gripper_pt); 
currpos(1) = currpos(1) - 0.1;


pos_final_xyz = release_pos;  % todo
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
  controlGripper('open', leftOrRight);
  mypause()
end