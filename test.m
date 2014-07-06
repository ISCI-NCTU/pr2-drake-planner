% 3 Go grasp
disp('3. Go to grasp pose');
%  -1 Get Current joints
if getJointAvailable  
  q0 = getCurrentQfromLCM();
else
  q0 = q_end(1:dof,1); 
end
%  -2 Set destination to drawer open pos
pos_final_xyz = [0.82782; -0.18899; 0.79926];
pos_final_orient = grasp_orient;  % (yaw, pitch, roll)
keepSameOrient = true;
%  -3 Create line plan
[xtraj,snopt_info,infeasible_constraint,q_end] = ...
            planner.createPointPlanWOrient(q0, pos_final_xyz, pos_final_orient, T*0.2, ...
            basefixed, torsofixed, keepSameOrient, 5);
%  -4 Play it in viewer
planner.v.playback(xtraj);
mypause()
%  -5 Publish
if toPublish
  planner.publishTraj(xtraj,snopt_info);
  mypause()
end