r = RigidBodyManipulator(strcat(getDrakePath(),'/examples/PR2/pr2.urdf'), ...
      struct('floating',true));
%r = RigidBodyManipulator('/home/drc/pr2/pr2.urdf', ...
%
  
  
planner = pr2Planner(r, 'r');
  

%1. Set to prepare pose
disp('1. Prepare');
% 1.2 Set destination to prepare
qdest = planner.getPrepareQ();

% 1.3 Create joint plan
[xtraj,snopt_info,infeasible_constraint,q_end] = ...
    planner.createJointPlan(q0,qdest,T,basefixed,torsofixed);
% 1.4 Play it in viewer
planner.v.playback(xtraj);



lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'pr2');


drawAxis(r, lcmgl, 'r_gripper_tool_frame', planner.getPrepareQ());

drawAxis(r, lcmgl, 'r_gripper_palm_link', planner.getPrepareQ());

drawAxis(r, lcmgl, 'r_forearm_link', planner.getPrepareQ());



lcmgl.switchBuffers;