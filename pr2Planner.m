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
  end
  
  methods 
    function obj = pr2Planner(r)
      obj.r = r;
      obj.doVisualization = false;
      
      obj.v = obj.r.constructVisualizer;
      obj.v.playback_speed = 5;
      
      joint_names = r.getStateFrame.coordinates(1:r.getNumDOF);
      % todo add: walk plan publisher
      obj.plan_pub = RobotPlanPublisherWKeyFrames_PR2('CANDIDATE_MANIP_PLAN',true,joint_names);
      
    end
    
    function [xtraj,snopt_info,infeasible_constraint,q_end] = createPointPlan(obj, q0, pos_final, T, basefixed)
      N = 10;
      t_vec = linspace(0,T,N);
      
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
      bodyA_idx = findLinkInd(obj.r,'r_gripper_palm_link');
      r_gripper_pt = [0,0,0]';
      r_gripper_cons = WorldPositionConstraint(obj.r,bodyA_idx,r_gripper_pt,pos_final,pos_final,[1,1]);
  
      
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
            createJointPlan(obj, q0, qend, T, ...
            basefixed, torsofixed)
      %todo: coding here
    end
    
    function [xtraj,snopt_info,infeasible_constraint,q_end] = ...
            createPointPlanWOrient(obj, q0, pos_final_xyz, pos_final_orient, T, ...
            basefixed, torsofixed)
      %todo: coding here
    end
    
    
    function [xtraj,snopt_info,infeasible_constraint,q_end] = ...
            createLinePlanWOrient(obj, q0, pos_final_xyz, pos_final_orient, T, ...
            basefixed, torsofixed)
      %todo: some coding here
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
      
      % 1.4 find gripper and base indices
      r_gripper_idx = findLinkInd(obj.r,'r_gripper_palm_link');
      r_gripper_pt = [0.18,0,0]';
      base_idx = findLinkInd(obj.r,'base_link');
      base_pt = [0,0,0]';
      
      % 1.5 create hand position constraint for reaching
      r_gripper_cons = WorldPositionConstraint(obj.r,r_gripper_idx,r_gripper_pt,pos_reach,pos_reach,[Tr,Tr]);
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
        Allcons{end+1} = WorldPositionConstraint(obj.r,r_gripper_idx,r_gripper_pt,...
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
    snopt_info_ik
    infeasible_constraint_ik1
    
    
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
    snopt_info_ik
    infeasible_constraint_ik2
      qtraj_guess = PPTrajectory(foh([0 Tr Tl],[q_start_nom, q_reach_nom, q_final_nom]));
      
      %% 4. do IK Traj
      [xtraj,snopt_info,infeasible_constraint] = ...
        inverseKinTraj(obj.r,...
        t_vec,qtraj_guess,qtraj_guess,...
        Allcons{:},ikoptions);
    snopt_info
    infeasible_constraint
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
      r_gripper_idx = findLinkInd(obj.r,'r_gripper_palm_link');
      r_gripper_pt = [0.13,0,0]';
      base_idx = findLinkInd(obj.r,'base_link');
      base_pt = [0,0,0]';
      
      % 1.5 create hand position constraint for reaching
      r_gripper_cons = WorldPositionConstraint(obj.r,r_gripper_idx,r_gripper_pt,pos_reach,pos_reach,[Tr,Tr]);
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
        Allcons{end+1} = WorldPositionConstraint(obj.r,r_gripper_idx,r_gripper_pt,...
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
    snopt_info_ik
    infeasible_constraint_ik1
    
    
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
    snopt_info_ik
    infeasible_constraint_ik2
      qtraj_guess = PPTrajectory(foh([0 Tr Tl],[q_start_nom, q_reach_nom, q_final_nom]));
      
      %% 4. do IK Traj
      [xtraj,snopt_info,infeasible_constraint] = ...
        inverseKinTraj(obj.r,...
        t_vec,qtraj_guess,qtraj_guess,...
        Allcons{:},ikoptions);
    
    snopt_info
    infeasible_constraint
      q_end = xtraj.eval(xtraj.tspan(end));

      % do visualize
      if obj.doVisualization && snopt_info <= 10
        obj.v.playback(xtraj);
      end
    end
    
    function publishTraj(obj,xtraj,snopt_info)
      utime = etime(clock,[1970 1 1 0 0 0])*1e6;
      nq_pr2 = obj.r.getNumDOF;
      ts = xtraj.pp.breaks;
      q = xtraj.eval(ts);
      xtraj_pr2(:,:) = q;
      snopt_info_vector = snopt_info*ones(1, size(xtraj_pr2,2));
      
      obj.plan_pub.publish(xtraj_pr2,ts,utime,snopt_info_vector);
      
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