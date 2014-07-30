classdef pr2Planner
  %NOTEST
  % A testing class for generating and publishing (by LCM) plans
  % General sequence:
  %   -Costruct a pr2Planner
  %   -createInitialReachPlan (reach to pre-grasp pose)
  %   - a) createLinePlan (move in a line)
  %   - b) createCirclePlan (move in a circle)
  properties
    r
    doVisualization
    v
    plan_pub
    leftOrRight
    gripper_pt
    gripper_frame
    gripper_idx
    gripper_l_finger_joint_idx
    gripper_r_finger_joint_idx
  end
  
  methods 
    function obj = pr2Planner(r, leftOrRight)
      obj.r = r;
      obj.doVisualization = false;
      
      options=struct();
      options.viewer = 'BotVisualizer';
      obj.v = obj.r.constructVisualizer(options);
      obj.v.playback_speed = 5;
      
      joint_names = r.getStateFrame.coordinates(1:r.getNumDOF);
      % todo add: walk plan publisher
      obj.plan_pub = RobotPlanPublisherWKeyFrames_PR2('CANDIDATE_MANIP_PLAN',true,joint_names);
      obj.leftOrRight = leftOrRight;
      
      obj.gripper_pt = [0.18,0,0]';
      if leftOrRight == 'r'
        obj.gripper_pt = obj.gripper_pt + [0.0356,0,0]';
      end
      obj.gripper_frame = sprintf('%s_gripper_palm_link', obj.leftOrRight);
      obj.gripper_idx = findLinkInd(obj.r,sprintf('%s_gripper_palm_link', obj.leftOrRight));
      obj.gripper_l_finger_joint_idx = findJointInd(obj.r,sprintf('%s_gripper_l_finger_joint', obj.leftOrRight));
      obj.gripper_r_finger_joint_idx = findJointInd(obj.r,sprintf('%s_gripper_r_finger_joint', obj.leftOrRight));
    end
    
    function q = getPrepareQ(obj)
      offset = 4;
      q = zeros(obj.r.getNumDOF, 1);
      q(obj.r.findJointInd('r_shoulder_lift_joint')+offset) = 0;  %r_shoulder_lift_joint
      q(obj.r.findJointInd('r_elbow_flex_joint')+offset) = -2;   %r_elbow_flex_joint
      q(obj.r.findJointInd('r_shoulder_pan_joint')+offset) = -1.50;   %r_elbow_flex_joint
      q(obj.r.findJointInd('l_shoulder_lift_joint')+offset) = 0;  %r_shoulder_lift_joint
      q(obj.r.findJointInd('l_elbow_flex_joint')+offset) = -2;   %r_elbow_flex_joint
      q(obj.r.findJointInd('l_shoulder_pan_joint')+offset) = 1.50;   %r_elbow_flex_joint
    end
    
    function [xtraj,snopt_info,infeasible_constraint,q_end] = createPointPlan(obj, q0, pos_final, T, basefixed)
      N = 10;
      t_vec = linspace(0,T,N);
      
      lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'pr2');
      lcmgl.switchBuffers;
      
      % create posture constraint
      posture_constraint = PostureConstraint(obj.r);
      base_idx = findLinkInd(obj.r,'base_link');
      % stick on the ground
      ground_cons = cell(1,4);
      ground_cons{1} = WorldPositionConstraint(obj.r,base_idx,[0,0,0]',[  -inf -inf 0 ]',[ inf inf 0 ]');
      ground_cons{2} = WorldPositionConstraint(obj.r,base_idx,[0,1,0]',[  -inf -inf 0 ]',[ inf inf 0 ]');
      ground_cons{3} = WorldPositionConstraint(obj.r,base_idx,[1,0,0]',[  -inf -inf 0 ]',[ inf inf 0 ]');
      ground_cons{4} = WorldPositionConstraint(obj.r,base_idx,[1,1,0]',[  -inf -inf 0 ]',[ inf inf 0 ]');
      
      if(basefixed)
        posture_constraint.setJointLimits(1,q0(1),q0(1));
        posture_constraint.setJointLimits(2,q0(2),q0(2));
        posture_constraint.setJointLimits(3,q0(3),q0(3));
      end
      
      ikoptions = IKoptions(obj.r);
      ikoptions = ikoptions.setIterationsLimit(1000000);
      
      % create hand position constraints
      %n_pts = 4;
      r_gripper_cons = WorldPositionConstraint(obj.r,obj.gripper_idx,obj.gripper_pt,pos_final,pos_final,[1,1]);
  
      
      % compute seeds
      q_start_nom = q0;
      [q_end_nom,snopt_info_ik,infeasible_constraint_ik] = inverseKin(obj.r,q_start_nom,q_start_nom,...
        posture_constraint,r_gripper_cons,ground_cons{:},ikoptions);
      qtraj_guess = PPTrajectory(foh([0 T],[q_start_nom, q_end_nom]));
      
      % do IK
      % function [xtraj,info,infeasible_constraint]= inverseKinTraj(obj,t,q_seed_traj,q_nom_traj,varargin)
      [xtraj,snopt_info,infeasible_constraint] = inverseKinTraj(obj.r,...
        t_vec,qtraj_guess,qtraj_guess,...
        posture_constraint,r_gripper_cons,ground_cons{:},ikoptions);
    
      q_end = xtraj.eval(xtraj.tspan(end));

      % do visualize
      if obj.doVisualization && snopt_info <= 50
        obj.v.playback(xtraj);
      end
      
      % publish plan
      obj.publishTraj(xtraj,snopt_info);
    end

    function [xtraj,snopt_info,infeasible_constraint,q_end] = ...
            createJointPlan(obj, q0, qdest, T, ...
            basefixed, torsofixed, N_)
      
      N = 20;
      if exist('N_', 'var')
        N = N_;
      end
      t_vec = linspace(0,T,N);
      Allcons = cell(0,1);
      
      
      % 1.2 create posture constraint, stick on the ground
      posture_constraint = PostureConstraint(obj.r);
      posture_constraint.setJointLimits(3,0,0); % base_z
      posture_constraint.setJointLimits(4,0,0); % base_roll
      posture_constraint.setJointLimits(5,0,0); % base_pitch
      Allcons{end+1} = posture_constraint;
      
      % 1.3 set iteration limit
      ikoptions = IKoptions(obj.r);
      ikoptions = ikoptions.setIterationsLimit(1000000);
      ikoptions = ikoptions.setDebug(true);
         
      
      % 1.4 fix base while reaching
      if(basefixed)        
        Allcons{end+1} = PostureChangeConstraint(obj.r,1,0,0); % base_x
        Allcons{end+1} = PostureChangeConstraint(obj.r,2,0,0); % base_y
        Allcons{end+1} = PostureChangeConstraint(obj.r,3,0,0); % base_z
        Allcons{end+1} = PostureChangeConstraint(obj.r,4,0,0); % base_roll
        Allcons{end+1} = PostureChangeConstraint(obj.r,5,0,0); % base_pitch
        Allcons{end+1} = PostureChangeConstraint(obj.r,6,0,0); % base_yaw
      end
      
      % 1.5 fix torso while reaching
      if(torsofixed)
        torso_id = 19;    %'torso_lift_joint'
        Allcons{end+1} = PostureChangeConstraint(obj.r,torso_id,0,0); 
      end
      
      
      % 1.6 create hand joint constraint for reaching
      dest_posture_constraint = PostureConstraint(obj.r, [T T]);
      for i=23:29
        dest_posture_constraint.setJointLimits(i,qdest(i),qdest(i));  % set constraint on arm joints
      end
      %for i=36:42
      %  dest_posture_constraint.setJointLimits(i,qdest(i),qdest(i));  % set constraint on arm joints
      %end
      Allcons{end+1} = dest_posture_constraint;
      
      % 1.7 
      qtraj_guess = PPTrajectory(foh([0 T],[q0, qdest]));
      
      
      %% 4. do IK Traj
      [xtraj,snopt_info,infeasible_constraint] = ...
        inverseKinTraj(obj.r,...
        t_vec,qtraj_guess,qtraj_guess,...
        Allcons{:},ikoptions);
      fprintf('snopt_info=%d\n', snopt_info);
      if (~isempty(infeasible_constraint)); displayInfeasible(infeasible_constraint); end
      q_end = xtraj.eval(xtraj.tspan(end));
      
      if obj.doVisualization && snopt_info <= 10
        obj.v.playback(xtraj);
      end
      
    end
    
    function [xtraj,snopt_info,infeasible_constraint,q_end] = ...
            createPointPlanWOrient(obj, q0, pos_final_xyz, pos_final_orient, T, ...
            basefixed, torsofixed, keepSameOrient, addCollision)
      if nargin < 9
        addCollision = false;
      end
        
      N = 20;
      t_vec = linspace(0,T,N);
      Allcons = cell(0,1);
      
      % draw start and end pose
      kinsol = obj.r.doKinematics(q0(1:obj.r.getNumDOF));
      pos0 = obj.r.forwardKin(kinsol,obj.gripper_idx,obj.gripper_pt);
      
      lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'pr2');
  
      lcmgl.glColor3f(1,0,1);
      lcmgl.sphere(pos0,0.01,100,100);
      lcmgl.glColor3f(0,0,1);
      lcmgl.sphere(pos_final_xyz,0.01,100,100);
      lcmgl.switchBuffers;
      
      
      % 1.1 create posture constraint, stick on the ground
      posture_constraint = PostureConstraint(obj.r);
      posture_constraint.setJointLimits(3,0,0); % base_z
      posture_constraint.setJointLimits(4,0,0); % base_roll
      posture_constraint.setJointLimits(5,0,0); % base_pitch
      Allcons{end+1} = posture_constraint;
      
      % 1.2 set iteration limit
      ikoptions = IKoptions(obj.r);
      ikoptions = ikoptions.setIterationsLimit(1000000);
      ikoptions = ikoptions.setDebug(true);
         
      
      % 1.3 fix base while reaching
      if(basefixed)        
        Allcons{end+1} = PostureChangeConstraint(obj.r,1,0,0); % base_x
        Allcons{end+1} = PostureChangeConstraint(obj.r,2,0,0); % base_y
        Allcons{end+1} = PostureChangeConstraint(obj.r,3,0,0); % base_z
        Allcons{end+1} = PostureChangeConstraint(obj.r,4,0,0); % base_roll
        Allcons{end+1} = PostureChangeConstraint(obj.r,5,0,0); % base_pitch
        Allcons{end+1} = PostureChangeConstraint(obj.r,6,0,0); % base_yaw
      end
      
      % 1.4 fix torso while reaching
      if(torsofixed)
        torso_id = 19;    %'torso_lift_joint'
        Allcons{end+1} = PostureChangeConstraint(obj.r,torso_id,0,0); 
      end
      
      % 1.5 Main constraint for destination pos_final_xyz
      r_gripper_cons_xyz = WorldPositionConstraint(obj.r,obj.gripper_idx,obj.gripper_pt,pos_final_xyz,pos_final_xyz,[T,T]);
      Allcons{end+1} = r_gripper_cons_xyz;
      
      % 1.6 Main constraint for destination pos_final_orient
      quat_des = pos_final_orient;
      if keepSameOrient
        tspan = [-inf inf];
      else 
        tspan = [T,T];
      end 
      tol = 0;
      r_gripper_cons_orient = WorldQuatConstraint(obj.r,obj.gripper_idx,quat_des,tol,tspan);
      
      Allcons{end+1} = r_gripper_cons_orient;
      
      % 1.7 collision constraint
      if (addCollision)
        Allcons{end+1} = WorldPositionConstraint(obj.r,obj.gripper_idx,obj.gripper_pt,-[inf inf inf]',[pos_final_xyz(1) inf inf]');
      end
      % 1.8 compute seeds
      q_start_nom = q0;
      ReachCons = cell(0,0);
      for i=1:length(Allcons)
          if(~isa(Allcons{i},'PostureChangeConstraint'))
              if(Allcons{i}.isTimeValid(T))
                 ReachCons{end+1} = Allcons{i};
              end
          end
      end
      [q_reach_nom,snopt_info_ik,infeasible_constraint_ik1] = ...
          inverseKin(obj.r,q_start_nom,q_start_nom,...
          ReachCons{:}, ikoptions);
      fprintf('snopt_info_ik=%d\n', snopt_info_ik);
      if (~isempty(infeasible_constraint_ik1)); displayInfeasible(infeasible_constraint_ik1); end
      qtraj_guess = PPTrajectory(foh([0 T],[q0, q_reach_nom]));
      
      
      %% 4. do IK Traj
      [xtraj,snopt_info,infeasible_constraint] = ...
        inverseKinTraj(obj.r,...
        t_vec,qtraj_guess,qtraj_guess,...
        Allcons{:},ikoptions);
      fprintf('snopt_info=%d\n', snopt_info);
      if (~isempty(infeasible_constraint)); displayInfeasible(infeasible_constraint); end
      q_end = xtraj.eval(xtraj.tspan(end));
      
      if obj.doVisualization && snopt_info <= 10
        obj.v.playback(xtraj);
      end
      
    end
    
    
    function [xtraj,snopt_info,infeasible_constraint,q_end] = ...
            createLinePlanWOrient(obj, q0, pos_final_xyz, pos_final_orient, T, ...
            basefixed, torsofixed, keepSameOrient, N_)
      N = 20;
      if exist('N_', 'var')
        N = N_;
      end
      t_vec = linspace(0,T,N);
      Allcons = cell(0,1);
      
      kinsol = obj.r.doKinematics(q0(1:obj.r.getNumDOF));
      pos0 = obj.r.forwardKin(kinsol,obj.gripper_idx,obj.gripper_pt);
      
      gripper_pos = repmat(pos0,1,N) + (pos_final_xyz - pos0)*linspace(0,1,N);
      
      %% 0. draw targets
      lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'pr2');
      lcmgl.glColor3f(1,0,1);
      lcmgl.sphere(pos0,0.01,100,100);
      lcmgl.glColor3f(0,0,1);
      lcmgl.sphere(pos_final_xyz,0.01,100,100);
      lcmgl.glColor3f(0,1,1);
      for i =1:N
          lcmgl.sphere(gripper_pos(:,i),0.01,100,100);
      end
      lcmgl.switchBuffers;
      
      %% 1. Reach
      
      % 1.1 create posture constraint
      posture_constraint = PostureConstraint(obj.r);
      posture_constraint.setJointLimits(3,0,0); % base_z
      posture_constraint.setJointLimits(4,0,0); % base_roll
      posture_constraint.setJointLimits(5,0,0); % base_pitch
      Allcons{end+1} = posture_constraint;
      
      % 1.2 set iteration limit
      ikoptions = IKoptions(obj.r);
      ikoptions = ikoptions.setIterationsLimit(1000000);
      ikoptions = ikoptions.setDebug(true);
      
      % 1.3 fix base while reaching
      if(basefixed)        
        Allcons{end+1} = PostureChangeConstraint(obj.r,1,0,0); % base_x
        Allcons{end+1} = PostureChangeConstraint(obj.r,2,0,0); % base_y
        Allcons{end+1} = PostureChangeConstraint(obj.r,3,0,0); % base_z
        Allcons{end+1} = PostureChangeConstraint(obj.r,4,0,0); % base_roll
        Allcons{end+1} = PostureChangeConstraint(obj.r,5,0,0); % base_pitch
        Allcons{end+1} = PostureChangeConstraint(obj.r,6,0,0); % base_yaw
      end
      
      % 1.4 fix torso while reaching
      if(torsofixed)
        torso_id = 19;    %'torso_lift_joint'
        Allcons{end+1} = PostureChangeConstraint(obj.r,torso_id,0,0); 
      end
      
      
      % 1.5 Main constraint for trajectory with destination pos_final_xyz
      for i=1:N,
        Allcons{end+1} = WorldPositionConstraint(obj.r,obj.gripper_idx,obj.gripper_pt,...
            gripper_pos(:,i),gripper_pos(:,i),[t_vec(i) t_vec(i)]);
      end
      
      % 1.6 Main constraint for destination pos_final_orient
      quat_des = pos_final_orient;
      if keepSameOrient
        tspan = [0.1*T inf];  % give it a little tolerance at the beginning
      else 
        tspan = [T,T];
      end 
      tol = 0;
      r_gripper_cons_orient = WorldQuatConstraint(obj.r,obj.gripper_idx,quat_des,tol,tspan);
      Allcons{end+1} = r_gripper_cons_orient;
      
      %% 3. compute seeds
      FinalCons = cell(0,0);
      for i=1:length(Allcons)
          if(~isa(Allcons{i},'PostureChangeConstraint'))
              if(Allcons{i}.isTimeValid(T))
                 FinalCons{end+1} = Allcons{i};
              end
          end
      end
    
      [q_final_nom,snopt_info_ik,infeasible_constraint_ik2] = ...
          inverseKin(obj.r,q0,q0,...
          FinalCons{:}, ikoptions);
      
      qtraj_guess = PPTrajectory(foh([0 T],[q0, q_final_nom]));
      
      
      %% 4. do IK Traj
      [xtraj,snopt_info,infeasible_constraint] = ...
        inverseKinTraj(obj.r,...
        t_vec,qtraj_guess,qtraj_guess,...
        Allcons{:},ikoptions);
      fprintf('snopt_info=%d\n', snopt_info);
      if (~isempty(infeasible_constraint)); display(infeasible_constraint); end
      q_end = xtraj.eval(xtraj.tspan(end));

      % do visualize
      if obj.doVisualization && snopt_info <= 10
        obj.v.playback(xtraj);
      end
    end
    
    
    function [xtraj,snopt_info,infeasible_constraint,q_end] = ...
            createReachAndLinePlan(obj, q0, pos_reach, pos_final, T, ...
            basefixedOnReach,basefixedOnLine,torsofixedOnReach,torsofixedOnLine)
      Nr = 10;
      Nl = 10;
      Tr = T/2;
      Tl = T;
      t_vec_r = linspace(0,Tr,Nr);
      t_vec_l = linspace(Tr,Tl,Nl);
      t_vec = linspace(0,T,Nr+Nl-1);
      Allcons = cell(0,1);
      gripper_pos = repmat(pos_reach,1,Nl) + (pos_final - pos_reach)*linspace(0,1,Nl);
      
      %% 0. draw targets
      lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'pr2');

      lcmgl.glColor3f(1,0,1);
      lcmgl.sphere(pos_reach,0.01,100,100);
      lcmgl.glColor3f(0,0,1);
      lcmgl.sphere(pos_final,0.01,100,100);
      lcmgl.glColor3f(0,1,1);
      for i =1:Nl
          lcmgl.sphere(gripper_pos(:,i),0.01,100,100);
      end
      lcmgl.switchBuffers;
      
      %% 1. Reach
      
      % 1.1 create posture constraint
      posture_constraint = PostureConstraint(obj.r);
      
      % 1.2 stick on the ground
      posture_constraint.setJointLimits(3,0,0); % base_z
      posture_constraint.setJointLimits(4,0,0); % base_roll
      posture_constraint.setJointLimits(5,0,0); % base_pitch
      Allcons{end+1} = posture_constraint;
      
      % 1.3 set iteration limit
      ikoptions = IKoptions(obj.r);
      ikoptions = ikoptions.setIterationsLimit(1000000);
      ikoptions = ikoptions.setDebug(true);
      
      
      % 1.5 create hand position constraint for reaching
      r_gripper_cons = WorldPositionConstraint(obj.r,obj.gripper_idx,obj.gripper_pt,pos_reach,pos_reach,[Tr,Tr]);
      Allcons{end+1} = r_gripper_cons;
      
      if(basefixedOnReach)        
        Allcons{end+1} = PostureChangeConstraint(obj.r,1,0,0,[0,Tr]); % base_x
        Allcons{end+1} = PostureChangeConstraint(obj.r,2,0,0,[0,Tr]); % base_y
        Allcons{end+1} = PostureChangeConstraint(obj.r,3,0,0,[0,Tr]); % base_z
        Allcons{end+1} = PostureChangeConstraint(obj.r,4,0,0,[0,Tr]); % base_roll
        Allcons{end+1} = PostureChangeConstraint(obj.r,5,0,0,[0,Tr]); % base_pitch
        Allcons{end+1} = PostureChangeConstraint(obj.r,6,0,0,[0,Tr]); % base_yaw
      end
      
      % 1.6 fix torso while doing line
      if(torsofixedOnReach)
        torso_id = 19;    %'torso_lift_joint'
        Allcons{end+1} = PostureChangeConstraint(obj.r,torso_id,0,0,[0,Tr]); 
      end
      
      
      %% 2. Line trajectory
      % 2.1 create hand position constraint for final pose
      
      %r_gripper_cons = WorldPositionConstraint(obj.r,r_gripper_idx,r_gripper_pt,pos_reach,pos_reach,[Tr,Tr]);
     
      
      for i=1:Nl,
        Allcons{end+1} = WorldPositionConstraint(obj.r,obj.gripper_idx,obj.gripper_pt,...
            gripper_pos(:,i),gripper_pos(:,i),[t_vec_l(i) t_vec_l(i)]);
      end
      
      if(basefixedOnLine)        
        Allcons{end+1} = PostureChangeConstraint(obj.r,1,0,0,[Tr,Tl]); % base_x
        Allcons{end+1} = PostureChangeConstraint(obj.r,2,0,0,[Tr,Tl]); % base_y
        Allcons{end+1} = PostureChangeConstraint(obj.r,3,0,0,[Tr,Tl]); % base_z
        Allcons{end+1} = PostureChangeConstraint(obj.r,4,0,0,[Tr,Tl]); % base_roll
        Allcons{end+1} = PostureChangeConstraint(obj.r,5,0,0,[Tr,Tl]); % base_pitch
        Allcons{end+1} = PostureChangeConstraint(obj.r,6,0,0,[Tr,Tl]); % base_yaw
      end
      
      % 2.2 fix torso while doing line
      if(torsofixedOnLine)
        torso_id = 19;    %'torso_lift_joint'
        Allcons{end+1} = PostureChangeConstraint(obj.r,torso_id,0,0,[Tr,Tl]); 
      end
      
      %% 3 compute seeds
      q_start_nom = q0;
      ReachCons = cell(0,0);
      for i=1:length(Allcons)
          if(~isa(Allcons{i},'PostureChangeConstraint'))
              if(Allcons{i}.isTimeValid(Tr))
                 ReachCons{end+1} = Allcons{i};
              end
          end
      end
      [q_reach_nom,snopt_info_ik,infeasible_constraint_ik1] = ...
          inverseKin(obj.r,q_start_nom,q_start_nom,...
          ReachCons{:}, ikoptions);
    
      fprintf('snopt_info_ik=%d\n', snopt_info_ik);
      if (~isempty(infeasible_constraint_ik1)); display(infeasible_constraint_ik1); end
    
      FinalCons = cell(0,0);
      for i=1:length(Allcons)
          if(~isa(Allcons{i},'PostureChangeConstraint'))
              if(Allcons{i}.isTimeValid(Tl))
                 FinalCons{end+1} = Allcons{i};
              end
          end
      end
    
      [q_final_nom,snopt_info_ik,infeasible_constraint_ik2] = ...
          inverseKin(obj.r,q_reach_nom,q_reach_nom,...
          FinalCons{:}, ikoptions);
      fprintf('snopt_info_ik=%d\n', snopt_info_ik);
      if (~isempty(infeasible_constraint_ik2)); display(infeasible_constraint_ik2); end
    
    
      
      %% 4. do IK Traj
      qtraj_guess = PPTrajectory(foh([0 Tr Tl],[q_start_nom, q_reach_nom, q_final_nom]));
      [xtraj,snopt_info,infeasible_constraint] = ...
        inverseKinTraj(obj.r,...
        t_vec,qtraj_guess,qtraj_guess,...
        Allcons{:},ikoptions);
      fprintf('snopt_info=%d\n', snopt_info);
      if (~isempty(infeasible_constraint)); display(infeasible_constraint); end
      q_end = xtraj.eval(xtraj.tspan(end));

      % do visualize
      if obj.doVisualization && snopt_info <= 10
        obj.v.playback(xtraj);
      end
    end

    function [xtraj,snopt_info,infeasible_constraint,q_end] = ...
            createReachAndCirclePlan(obj, q0, pos_center, radius, deg1, deg2, T, basefixedOnReach,basefixedOnCircle,torsofixedOnReach,torsofixedOnCircle)
      Nr = 30;
      Nl = 30;
      Tr = T/2;
      Tl = T;
      t_vec_r = linspace(0,Tr,Nr);
      t_vec_l = linspace(Tr,Tl,Nl);
      t_vec = linspace(0,T,Nr+Nl-1);
      Allcons = cell(0,1);
      xh = [1,0,0]';
      yh = [0,1,0]';
      pos_reach = pos_center + radius*(xh*cos(deg1) + yh*sin(deg1));
      pos_final = pos_center + radius*(xh*cos(deg2) + yh*sin(deg2));
      gripper_pos = repmat(pos_center,1,Nl) + radius*(xh*cos(linspace(deg1,deg2,Nl)) + yh*sin(linspace(deg1,deg2,Nl)));
      
      %% 0. draw targets
      lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'pr2');

      lcmgl.glColor3f(1,0,1);
      lcmgl.sphere(pos_reach,0.05,100,100);
      lcmgl.glColor3f(0,0,1);
      lcmgl.sphere(pos_final,0.05,100,100);
      lcmgl.glColor3f(0,1,1);
      for i =1:Nl
          lcmgl.sphere(gripper_pos(:,i),0.05,100,100);
      end
      lcmgl.switchBuffers;

      
      %% 1. Reach
      
      % 1.1 create posture constraint
      posture_constraint = PostureConstraint(obj.r);
      
      % 1.2 stick on the ground
      posture_constraint.setJointLimits(3,0,0); % base_z
      posture_constraint.setJointLimits(4,0,0); % base_roll
      posture_constraint.setJointLimits(5,0,0); % base_pitch
      Allcons{end+1} = posture_constraint;
      
      % 1.3 set iteration limit
      ikoptions = IKoptions(obj.r);
      ikoptions = ikoptions.setIterationsLimit(1000000);
      ikoptions = ikoptions.setDebug(true);
      
      % 1.4 find gripper and base indices
      base_idx = findLinkInd(obj.r,'base_link');
      base_pt = [0,0,0]';
      
      % 1.5 create hand position constraint for reaching
      r_gripper_cons = WorldPositionConstraint(obj.r,obj.gripper_idx,obj.gripper_pt,pos_reach,pos_reach,[Tr,Tr]);
      Allcons{end+1} = r_gripper_cons;
      
      if(basefixedOnReach)        
        Allcons{end+1} = PostureChangeConstraint(obj.r,1,0,0,[0,Tr]); % base_x
        Allcons{end+1} = PostureChangeConstraint(obj.r,2,0,0,[0,Tr]); % base_y
        Allcons{end+1} = PostureChangeConstraint(obj.r,3,0,0,[0,Tr]); % base_z
        Allcons{end+1} = PostureChangeConstraint(obj.r,4,0,0,[0,Tr]); % base_roll
        Allcons{end+1} = PostureChangeConstraint(obj.r,5,0,0,[0,Tr]); % base_pitch
        Allcons{end+1} = PostureChangeConstraint(obj.r,6,0,0,[0,Tr]); % base_yaw
      end
      
      % 1.6 fix torso while doing circle
      if(torsofixedOnReach)
        %torso_id = obj.r.findJointInd('torso_lift_joint');  bad index
        torso_id = 19;
        Allcons{end+1} = PostureChangeConstraint(obj.r,torso_id,0,0,[0,Tr]); 
      end
      
      
      %% 2. Circle trajectory
      % 2.1 create hand position constraint for final pose
      
      %r_gripper_cons = WorldPositionConstraint(obj.r,r_gripper_idx,r_gripper_pt,pos_reach,pos_reach,[Tr,Tr]);
     
      %gripper_pos = repmat(pos_reach,1,Nl) + (pos_final - pos_reach)*linspace(0,1,Nl);
      
      for i=1:Nl,
        Allcons{end+1} = WorldPositionConstraint(obj.r,obj.gripper_idx,obj.gripper_pt,...
            gripper_pos(:,i),gripper_pos(:,i),[t_vec_l(i) t_vec_l(i)]);
      end
      
      if(basefixedOnCircle)        
        Allcons{end+1} = PostureChangeConstraint(obj.r,1,0,0,[Tr,Tl]); % base_x
        Allcons{end+1} = PostureChangeConstraint(obj.r,2,0,0,[Tr,Tl]); % base_y
        Allcons{end+1} = PostureChangeConstraint(obj.r,3,0,0,[Tr,Tl]); % base_z
        Allcons{end+1} = PostureChangeConstraint(obj.r,4,0,0,[Tr,Tl]); % base_roll
        Allcons{end+1} = PostureChangeConstraint(obj.r,5,0,0,[Tr,Tl]); % base_pitch
        Allcons{end+1} = PostureChangeConstraint(obj.r,6,0,0,[Tr,Tl]); % base_yaw
      end
      
      % 2.2 fix torso while doing circle
      if(torsofixedOnCircle)
        %torso_id = obj.r.findJointInd('torso_lift_joint');  bad index
        torso_id = 19;
        Allcons{end+1} = PostureChangeConstraint(obj.r,torso_id,0,0,[Tr,Tl]); 
      end
      
      
      %% 3 compute seeds
      q_start_nom = q0;
      ReachCons = cell(0,0);
      for i=1:length(Allcons)
          if(~isa(Allcons{i},'PostureChangeConstraint'))
              if(Allcons{i}.isTimeValid(Tr))
                 ReachCons{end+1} = Allcons{i};
              end
          end
      end
      [q_reach_nom,snopt_info_ik,infeasible_constraint_ik1] = ...
          inverseKin(obj.r,q_start_nom,q_start_nom,...
          ReachCons{:}, ikoptions);
    
      fprintf('snopt_info_ik=%d\n', snopt_info_ik);
      if (~isempty(infeasible_constraint_ik1)); display(infeasible_constraint_ik1); end
    
      FinalCons = cell(0,0);
      FinalCons{end+1} = PostureChangeConstraint(obj.r,torso_id,0,0,[-inf,inf]); 
      for i=1:length(Allcons)
          if(~isa(Allcons{i},'PostureChangeConstraint'))
              if(Allcons{i}.isTimeValid(Tl))
                 FinalCons{end+1} = Allcons{i};
              end
          end
      end
    
      [q_final_nom,snopt_info_ik,infeasible_constraint_ik2] = ...
          inverseKin(obj.r,q_reach_nom,q_reach_nom,...
          FinalCons{:}, ikoptions);
      fprintf('snopt_info_ik=%d\n', snopt_info_ik);
      if (~isempty(infeasible_constraint_ik2)); display(infeasible_constraint_ik2); end
      qtraj_guess = PPTrajectory(foh([0 Tr Tl],[q_start_nom, q_reach_nom, q_final_nom]));
      
      %% 4. do IK Traj
      [xtraj,snopt_info,infeasible_constraint] = ...
        inverseKinTraj(obj.r,...
        t_vec,qtraj_guess,qtraj_guess,...
        Allcons{:},ikoptions);
      
      fprintf('snopt_info=%d\n', snopt_info);
      if (~isempty(infeasible_constraint)); display(infeasible_constraint); end
      
      q_end = xtraj.eval(xtraj.tspan(end));

      % do visualize
      if obj.doVisualization && snopt_info <= 10
        obj.v.playback(xtraj);
      end
    end
    
    function publishTraj(obj,xtraj,snopt_info)
      if snopt_info > 10
        fprintf('IK fail snopt_info: %d\n', snopt_info);
        exit()
      end
        
      utime = etime(clock,[1970 1 1 0 0 0])*1e6;
      ts = xtraj.pp.breaks;
      q = xtraj.eval(ts);
      xtraj_pr2(:,:) = q;
      snopt_info_vector = snopt_info*ones(1, size(xtraj_pr2,2));
      
      obj.plan_pub.publish(xtraj_pr2,ts,utime,snopt_info_vector);
      
    end
    
    function publishEvent(obj, str)
        msg = planner.string_t();

        msg.utime = get_timestamp_now();
        msg.data = str;

        obj.plan_pub.lc.publish('MANIP_EVENT', msg);
    end
    
    function [ts,q,snopt_info_vector] = getTraj(obj,xtraj,snopt_info)
      utime = etime(clock,[1970 1 1 0 0 0])*1e6;
      nq_pr2 = obj.r.getNumDOF;
      ts = xtraj.pp.breaks;
      q = xtraj.eval(ts);
      xtraj_pr2(:,:) = q;
      snopt_info_vector = snopt_info*ones(1, size(xtraj_pr2,2));
      
    end
  end
end