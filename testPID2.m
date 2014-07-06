%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PID control
ftoffset = 13.4;
controlGripper('close', leftOrRight);
ft = getFTfromLCM();   %transformForceToGlobal

force_x = -(ft(1) - ftoffset);
torque_x = ft(4);
force_threshold = 0.5;
fprintf('force_x = %f, torque_x = %f\n', force_x, torque_x);
while abs(force_x) > force_threshold
  %grasp_pos(3) = 0.002 * force_x + grasp_pos(3);  %% or set directly to current pos
  %controlGripper('open', leftOrRight);
  
  
  %% show current tip position
  if getJointAvailable  
    q = getCurrentQfromLCM();
  else
    q = planner.getPrepareQ();
  end
  
  kinsol = r.doKinematics(q(1:dof,1));
  gripper_pt = [0.18,0,0]';
  currpos = r.forwardKin(kinsol,findLinkInd(r,sprintf('%s_gripper_palm_link', leftOrRight)), gripper_pt); 
  grasp_pos
  currpos
  grasp_pos = currpos;
  grasp_pos(3) = grasp_pos(3) + 0.001 * force_x;
  
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
              planner.createLinePlanWOrient(q0, pos_final_xyz, pos_final_orient, 1, ...
              basefixed, torsofixed, keepSameOrient, 5);
  %  -4 Play it in viewer
  planner.v.playback(xtraj);
  mypause()
  %  -5 Publish
  if toPublish
    planner.publishTraj(xtraj,snopt_info);
    mypause()
  end

  
  
  %controlGripper('close', leftOrRight);
  ft = getFTfromLCM();   %transformForceToGlobal
  force_x = -(ft(1) - ftoffset);
  fprintf('force_x = %f, torque_x = %f\n', force_x, torque_x);
end
