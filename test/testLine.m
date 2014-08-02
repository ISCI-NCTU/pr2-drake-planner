
warning('off','all');

DRAKE_PATH = '/home/drc/drc/software/drake';

if(~exist('r','var'))
  fprintf('Loading Robot Model...');
  r = RigidBodyManipulator(strcat(DRAKE_PATH,'/examples/PR2/pr2.urdf'),struct('floating',true));
  fprintf('Done\n');
end

dof = r.getNumDOF;
q0 = zeros(dof, 1);
q0(24) = 0.8;  %r_shoulder_lift_joint
q0(26) = -2;   %r_elbow_flex_joint

planner = pr2Planner(r);

drawer_close_pos = [0.8,0,0.375]'; % specify drawer pose (x,y,z)
drawer_open_pos =  [0.7,0,0.375]';  

T = 10; % seconds
basefixedOnReach = true;
basefixedOnLine = true;
torsofixedOnReach = true;
torsofixedOnLine = true;
[xtraj,snopt_info,infeasible_constraint,q_end] = ...
    planner.createReachAndLinePlan(q0, drawer_close_pos, drawer_open_pos, T, basefixedOnReach, basefixedOnLine, torsofixedOnReach, torsofixedOnLine);
snopt_info;

kinsol = r.doKinematics(q_end(1:dof));
r_gripper_pt = [0.18,0,0]';
reality_reach2 = r.forwardKin(kinsol,findLinkInd(r,'r_gripper_palm_link'),r_gripper_pt);
reality_reach2



[ts,q,snopt_info_vector] = planner.getTraj(xtraj,snopt_info);
planner.publishTraj(xtraj,snopt_info);
planner.v.playback(xtraj);






% [xtraj_reach,snopt_info_reach,infeasible_constraint_reach, qnow1] = ...
%     planner.createPointPlan(q0, drawer_close_pos, 10, false);
% 
% kinsol = r.doKinematics(qnow1);
% reality_reach1 = r.forwardKin(kinsol,findLinkInd(r,'r_gripper_palm_link'),[0,0,0]');
% reality_reach1
% snopt_info_reach
% 
% [xtraj_reach,snopt_info_reach,infeasible_constraint_reach, qnow2] = ...
%     planner.createLinePlan(qnow1, reality_reach1, drawer_open_pos, 30, true);
% 
% snopt_info_reach
% infeasible_constraint_reach
% kinsol = r.doKinematics(qnow2);
% reality_reach2 = r.forwardKin(kinsol,findLinkInd(r,'r_gripper_palm_link'),[0,0,0]');
% reality_reach2

