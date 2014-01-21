%clear all
%close all
warning('off','all');

DRAKE_PATH = '/home/drc/drc/software/drake';

if(~exist('r','var'))
  fprintf('Loading Robot Model...');
  r = RigidBodyManipulator(strcat(DRAKE_PATH,'/examples/PR2/pr2.urdf'),struct('floating',true));
  fprintf('Done\n');
end

dof = r.getNumDOF;
q0 = zeros(dof, 1);

planner = pr2Planner(r);

drawer_close_pos = [1,1,1]'; % specify drawer pose (x,y,z)
drawer_open_pos =  [0.5,1,1]';  


T = 50; % seconds
basefixedOnReach = false;
basefixedOnLine = true;
[xtraj,snopt_info,infeasible_constraint,q_end] = ...
    planner.createReachAndLinePlan(q0, drawer_close_pos, drawer_open_pos, T, basefixedOnReach, basefixedOnLine);
snopt_info;

kinsol = r.doKinematics(q_end(1:dof));
reality_reach2 = r.forwardKin(kinsol,findLinkInd(r,'r_gripper_palm_link'),[0,0,0]');
reality_reach2


% lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'pr2');
% 
% lcmgl.glColor3f(1,0,0);
% lcmgl.sphere(drawer_close_pos,0.1,5,5);
% lcmgl.glColor3f(0,1,0);
% lcmgl.sphere(drawer_open_pos,0.1,5,5);
% lcmgl.switchBuffers;







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

