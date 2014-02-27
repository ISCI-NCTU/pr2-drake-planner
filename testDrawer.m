
% 0 initialization
inSim = false;
T = 3;
offset = 4;
drawer_close_pos = [0.6,-0.5,0.375]'; % specify drawer pose (x,y,z)
drawer_open_pos =  [0.4,-0.5,0.375]';  
warning('off','all');
DRAKE_PATH = '/home/drc/drc/software/drake';
if(~exist('r','var'))
  fprintf('Loading Robot Model...');
  r = RigidBodyManipulator(strcat(DRAKE_PATH,'/examples/PR2/pr2.urdf'),struct('floating',true));
  fprintf('Done\n');
end
planner = pr2Planner(r);
dof = r.getNumDOF;
basefixed = true;
torsofixed = true;


% 1.1 Get Current joints
if inSim  
  q0 = getCurrentQfromLCM();
else
  q0 = zeros(dof, 1);
end
% 1.2 Set destination to prepare
qdest = zeros(dof, 1);
%qdest(r.findJointInd('r_shoulder_lift_joint')+offset) = 0.8;  %r_shoulder_lift_joint
%qdest(r.findJointInd('r_elbow_flex_joint')+offset) = -2;   %r_elbow_flex_joint
qdest(24) = 0.8;  %r_shoulder_lift_joint
qdest(26) = -2;   %r_elbow_flex_joint

% 1.3 Create joint plan
[xtraj,snopt_info,infeasible_constraint,q_end] = ...
    planner.createJointPlan(q0,qdest,T,basefixed,torsofixed)
% 1.4 Play it in viewer
planner.v.playback(xtraj);
pause()
% 1.5 Publish
planner.publishTraj(xtraj,snopt_info);
pause()
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Todo open gripper and wait
if inSim  
  q0 = getCurrentQfromLCM();
else
  q0 = q_end(1:dof,1);
  % 1.2 Set destination to prepare
  qdest = q0;
  qdest(r.findJointInd('r_gripper_l_finger_joint')+offset) = 0.2;  %r_gripper_l_finger_joint
  qdest(r.findJointInd('r_gripper_r_finger_joint')+offset) = 0.2;  %r_gripper_l_finger_joint

  % 1.3 Create joint plan
  [xtraj,snopt_info,infeasible_constraint,q_end] = ...
      planner.createJointPlan(q0,qdest,T,basefixed,torsofixed)
  % 1.4 Play it in viewer
  planner.v.playback(xtraj);
  pause()
  % 1.5 Publish
  planner.publishTraj(xtraj,snopt_info);
  system('rosrun simple_gripper simple_gripper open');
  pause()
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2.1 Get Current joints
if inSim  
  q0 = getCurrentQfromLCM();
else
  q0 = q_end(1:dof,1); %% todo
end
% 2.2 Set destination to reach
pos_final_xyz = drawer_close_pos;
pos_final_orient = angle2quat(0,0,1.57)';  % (yaw, pitch, roll)

% 2.3 Create joint plan
[xtraj,snopt_info,infeasible_constraint,q_end] = ...
            planner.createPointPlanWOrient(q0, pos_final_xyz, pos_final_orient, T, ...
            basefixed, torsofixed);
        
% 2.4 Play it in viewer
planner.v.playback(xtraj);
pause()
% 2.5 Publish
planner.publishTraj(xtraj,snopt_info);
pause()

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Todo close gripper and wait
if inSim  
  % close command
else
  q0 = q_end(1:dof,1);
  
  % 1.2 Set destination to prepare
  qdest = q0;
  qdest(r.findJointInd('r_gripper_l_finger_joint')+offset) = 0.1;  %r_gripper_l_finger_joint
  qdest(r.findJointInd('r_gripper_r_finger_joint')+offset) = 0.1;  %r_gripper_l_finger_joint

  % 1.3 Create joint plan
  [xtraj,snopt_info,infeasible_constraint,q_end] = ...
      planner.createJointPlan(q0,qdest,T,basefixed,torsofixed)
  % 1.4 Play it in viewer
  planner.v.playback(xtraj);
  pause()
  % 1.5 Publish
  planner.publishTraj(xtraj,snopt_info);
  pause()
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 3.1 Get Current joints
if inSim  
  q0 = getCurrentQfromLCM();
else
  q0 = q_end(1:dof,1); 
end
% 3.2 Set destination to drawer open pos
pos_final_xyz = drawer_open_pos;
pos_final_orient = angle2quat(0,0,1.57)';  % (yaw, pitch, roll)
keepSameOrient = true;
% 3.3 Create line plan
[xtraj,snopt_info,infeasible_constraint,q_end] = ...
            planner.createLinePlanWOrient(q0, pos_final_xyz, pos_final_orient, T, ...
            basefixed, torsofixed, keepSameOrient);
% 3.4 Play it in viewer
planner.v.playback(xtraj);
% 3.5 Publish
planner.publishTraj(xtraj,snopt_info);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Todo open gripper and wait
if inSim  
  q0 = getCurrentQfromLCM();
else
  q0 = q_end(1:dof,1);
  % 1.2 Set destination to prepare
  qdest = q0;
  qdest(r.findJointInd('r_gripper_l_finger_joint')+offset) = 0.2;  %r_gripper_l_finger_joint
  qdest(r.findJointInd('r_gripper_r_finger_joint')+offset) = 0.2;  %r_gripper_l_finger_joint

  % 1.3 Create joint plan
  [xtraj,snopt_info,infeasible_constraint,q_end] = ...
      planner.createJointPlan(q0,qdest,T,basefixed,torsofixed)
  % 1.4 Play it in viewer
  planner.v.playback(xtraj);
  pause()
  % 1.5 Publish
  planner.publishTraj(xtraj,snopt_info);
  system('rosrun simple_gripper simple_gripper open');
  pause()
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 4.1 Get Current joints
if inSim  
  q0 = getCurrentQfromLCM()
else
  q0 = q_end(1:dof,1);
end
% 4.2 Set destination to prepare
qdest = zeros(dof, 1);
qdest(r.findJointInd('r_shoulder_lift_joint')+offset) = 0.8;  %r_shoulder_lift_joint
qdest(r.findJointInd('r_elbow_flex_joint')+offset) = -2;   %r_elbow_flex_joint

% 4.3 Create joint plan
[xtraj,snopt_info,infeasible_constraint,q_end] = ...
    planner.createJointPlan(q0,qdest,T,basefixed,torsofixed)
% 4.4 Play it in viewer
planner.v.playback(xtraj);
pause()
% 4.5 Publish
planner.publishTraj(xtraj,snopt_info);
pause()

