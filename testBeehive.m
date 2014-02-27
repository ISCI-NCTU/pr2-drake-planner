% 0 initialization
getJointAvailable = false;
T = 3;
offset = 4;
pregrasp_pos = [0.6,-0.5,0.3]'; % pregrasp pose (x,y,z)
pregrasp_orient =  angle2quat(0,0,0)';  

grasp_pos = [0.6,-0.5,0.3]'; % grasp pose (x,y,z)
grasp_orient =  angle2quat(0,0,0)';  

hold1_pos = [0.6,-0.5,0.5]'; % grasp pose (x,y,z)
hold1_orient =  angle2quat(0,0,0)';  

hold2_pos = [0.6,-0.5,0.5]'; % grasp pose (x,y,z)
hold2_orient =  angle2quat(0,0,1.57)'; 

hold3_pos = [0.6,-0.3,0.5]'; % grasp pose (x,y,z)
hold3_orient =  angle2quat(0,0,1.57)'; 

put_pos = [0.6,-0.3,0.3]'; % grasp pose (x,y,z)
put_orient =  angle2quat(0,0,1.57)'; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 1.1 Get Current joints
if getJointAvailable  
  q0 = getCurrentQfromLCM();
else
  q0 = zeros(dof, 1);
end
% 1.2 Set destination to prepare
qdest = zeros(dof, 1);
qdest(r.findJointInd('r_shoulder_lift_joint')+offset) = 0.8;  %r_shoulder_lift_joint
qdest(r.findJointInd('r_elbow_flex_joint')+offset) = -2;   %r_elbow_flex_joint

% 1.3 Create joint plan
[xtraj,snopt_info,infeasible_constraint,q_end] = ...
    planner.createJointPlan(q0,qdest,T,basefixed,torsofixed)
% 1.4 Play it in viewer
planner.v.playback(xtraj);
pause()
% 1.5 Publish
planner.publishTraj(xtraj,snopt_info);
pause()

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2.1 Open gripper

q0 = q_end(1:dof,1);
% 2.2 Set destination to prepare
qdest = q0;
qdest(r.findJointInd('r_gripper_l_finger_joint')+offset) = 0.2;  %r_gripper_l_finger_joint
qdest(r.findJointInd('r_gripper_r_finger_joint')+offset) = 0.2;  %r_gripper_l_finger_joint

% 2.3 Create joint plan
[xtraj,snopt_info,infeasible_constraint,q_end] = ...
  planner.createJointPlan(q0,qdest,T,basefixed,torsofixed)
% 2.4 Play it in viewer
planner.v.playback(xtraj);
pause()
% 2.5 Publish
planner.publishTraj(xtraj,snopt_info);
system('rosrun simple_gripper simple_gripper open');
pause()

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 3.1 Get Current joints
if inSim  
  q0 = getCurrentQfromLCM();
else
  q0 = q_end(1:dof,1); %% todo
end
% 3.2 Set destination to reach
pos_final_xyz = pregrasp_pos;
pos_final_orient = pregrasp_orient;  % (yaw, pitch, roll)

% 3.3 Create joint plan
[xtraj,snopt_info,infeasible_constraint,q_end] = ...
            planner.createPointPlanWOrient(q0, pos_final_xyz, pos_final_orient, T, ...
            basefixed, torsofixed);
        
% 3.4 Play it in viewer
planner.v.playback(xtraj);
pause()
% 3.5 Publish
planner.publishTraj(xtraj,snopt_info);
pause()

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 3.1 Get Current joints
if getJointAvailable  
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

