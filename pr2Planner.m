classdef pr2Planner
  %NOTEST
  % A testing class for generating and publishing (by LCM) plans
  % General sequence:
  %   -Costruct a pr2Planner
  %   -createInitialReachPlan (reach to pre-grasp pose)
  %   - a) createLinePlan (move in a line)
  %   - b) createCirclePlan (move in a circle)
  % todo: zero velocity constraints?
  % todo: look into the fixed initial state option
  properties
    r
    doVisualization
    v
  end
  
  methods 
    function obj = pr2Planner(r)
      obj.r = r;
      obj.doVisualization = true;
      
      if obj.doVisualization
        obj.v = obj.r.constructVisualizer;
        obj.v.playback_speed = 5;
      end
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
    end
    
    
    function [xtraj,snopt_info,infeasible_constraint,q_end] = ...
            createReachAndLinePlan(obj, q0, pos_reach, pos_final, T, basefixedOnReach,basefixedOnLine)
      Nr = 10;
      Nl = 10;
      Tr = T/2;
      Tl = T;
      t_vec_r = linspace(0,Tr,Nr);
      t_vec_l = linspace(Tr,Tl,Nl);
      t_vec = linspace(0,T,Nr+Nl);
      Allcons = cell(0,1);
      
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
      
      % 1.4 find gripper and base indices
      r_gripper_idx = findLinkInd(obj.r,'r_gripper_palm_link');
      r_gripper_pt = [0,0,0]';
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
      
      %% 2. Line trajectory
      % 2.1 create hand position constraint for final pose
      
      %r_gripper_cons = WorldPositionConstraint(obj.r,r_gripper_idx,r_gripper_pt,pos_reach,pos_reach,[Tr,Tr]);
     
      gripper_pos = repmat(pos_reach,1,Nl) + (pos_final - pos_reach)*linspace(0,1,Nl);
      for i=1:Nl,
        Allcons{end+1} = WorldPositionConstraint(obj.r,r_gripper_idx,r_gripper_pt,gripper_pos(:,i),gripper_pos(:,i),[t_vec_l(i) t_vec_l(i)]);
      end
      
      if(basefixedOnLine)        
        Allcons{end+1} = PostureChangeConstraint(obj.r,1,0,0,[Tr,Tl]); % base_x
        Allcons{end+1} = PostureChangeConstraint(obj.r,2,0,0,[Tr,Tl]); % base_y
        Allcons{end+1} = PostureChangeConstraint(obj.r,3,0,0,[Tr,Tl]); % base_z
        Allcons{end+1} = PostureChangeConstraint(obj.r,4,0,0,[Tr,Tl]); % base_roll
        Allcons{end+1} = PostureChangeConstraint(obj.r,5,0,0,[Tr,Tl]); % base_pitch
        Allcons{end+1} = PostureChangeConstraint(obj.r,6,0,0,[Tr,Tl]); % base_yaw
      end
      
      %% 3 compute seeds
      q_start_nom = q0;
      [q_reach_nom,snopt_info_ik,infeasible_constraint_ik] = ...
          inverseKin(obj.r,q_start_nom,q_start_nom,...
          Allcons{:}, ikoptions);
    
      [q_final_nom,snopt_info_ik,infeasible_constraint_ik] = ...
          inverseKin(obj.r,q_reach_nom,q_reach_nom,...
          Allcons{:}, ikoptions);
    
      qtraj_guess = PPTrajectory(foh([0 Tr Tl],[q_start_nom, q_reach_nom, q_final_nom]));
      
      %% 4. do IK Traj
      [xtraj,snopt_info,infeasible_constraint] = ...
        inverseKinTraj(obj.r,...
        t_vec,qtraj_guess,qtraj_guess,...
        Allcons{:},ikoptions);
    
      q_end = xtraj.eval(xtraj.tspan(end));

      % do visualize
      if obj.doVisualization && snopt_info <= 10
        obj.v.playback(xtraj);
      end
    end
    
    
    function [xtraj,snopt_info,infeasible_constraint,q_end_nom] = createLinePlan(obj, q0, pos_init, pos_final, T, basefixed)
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
        posture_constraint.setJointLimits(1,q0(1),q0(1)); % base_x
        posture_constraint.setJointLimits(2,q0(2),q0(2)); % base_y
        posture_constraint.setJointLimits(3,q0(3),q0(3)); % base_z
        posture_constraint.setJointLimits(4,q0(4),q0(4)); % base_roll
        posture_constraint.setJointLimits(5,q0(5),q0(5)); % base_pitch
        posture_constraint.setJointLimits(6,q0(6),q0(6)); % base_yaw
      end
      
      
      
      ikoptions = IKoptions(obj.r);
      ikoptions = ikoptions.setIterationsLimit(1000000);
      
      % create hand position constraints
      %n_pts = 4;
      bodyA_idx = findLinkInd(obj.r,'r_gripper_palm_link');
      r_gripper_pt = [0,0,0]';
      
      % create drill position constraints
      
      pos_t = repmat(pos_init,1,N) + (pos_final - pos_init)*linspace(0,1,N);
      pos_constraint = cell(1,N);
      for i=1:N,
        pos_constraint{i} = WorldPositionConstraint(obj.r,bodyA_idx,r_gripper_pt,pos_t(:,i),pos_t(:,i),[t_vec(i) t_vec(i)]);
      end
      r_gripper_cons = WorldPositionConstraint(obj.r,bodyA_idx,r_gripper_pt,pos_final,pos_final,[1,1]);
  
      
      % compute seeds
      q_start_nom = q0;
      [q_end_nom,snopt_info_ik,infeasible_constraint_ik] = inverseKin(obj.r,q_start_nom,q_start_nom,...
        posture_constraint,r_gripper_cons,ground_cons{:},pos_constraint{:},ikoptions);
      qtraj_guess = PPTrajectory(foh([0 T],[q_start_nom, q_end_nom]));
      
      % do IK
      % function [xtraj,info,infeasible_constraint]= inverseKinTraj(obj,t,q_seed_traj,q_nom_traj,varargin)
      [xtraj,snopt_info,infeasible_constraint] = inverseKinTraj(obj.r,...
        t_vec,qtraj_guess,qtraj_guess,...
        posture_constraint,r_gripper_cons,ground_cons{:},pos_constraint{:},ikoptions);
    
      q_end = xtraj.eval(xtraj.tspan(end));
      %q_end

      % do visualize
      if obj.doVisualization && snopt_info <= 50
        obj.v.playback(xtraj);
      end
    end
%     function [xtraj,snopt_info,infeasible_constraint] = createLinePlanx(obj, q0, x_drill_init, x_drill_final, T)
%       N = 10;
%       t_vec = linspace(0,T,N);
%       
%       % create posture constraint
%       posture_index = setdiff((1:obj.r.num_q)',obj.joint_indices');
%       posture_constraint = PostureConstraint(obj.r);
%       posture_constraint = posture_constraint.setJointLimits(posture_index,q0(posture_index),q0(posture_index));
%       
%       % create drill direction constraint
%       drill_dir_constraint = WorldGazeDirConstraint(obj.r,obj.hand_body,obj.drill_axis_on_hand,...
%         obj.drilling_world_axis,obj.default_axis_threshold);
%       
%       % create drill position constraints
%       x_drill = repmat(x_drill_init,1,N) + (x_drill_final - x_drill_init)*linspace(0,1,N);
%       drill_pos_constraint = cell(1,N);
%       for i=1:N,
%         drill_pos_constraint{i} = WorldPositionConstraint(obj.r,obj.hand_body,obj.drill_pt_on_hand,x_drill(:,i),x_drill(:,i),[t_vec(i) t_vec(i)]);
%       end
%       
%       % Find nominal poses
%       [q_start_nom,snopt_info_ik,infeasible_constraint_ik] = inverseKin(obj.r,q0,q0,...
%         drill_pos_constraint{1},drill_dir_constraint,posture_constraint,obj.ik_options);
%       
%       if(snopt_info_ik > 10)
%         send_msg = sprintf('snopt_info = %d. The IK (init) fails.',snopt_info_ik);
%         send_status(4,0,0,send_msg);
%         display(infeasibleConstraintMsg(infeasible_constraint_ik));
%         warning(send_msg);
%       end
%       
%       [q_end_nom,snopt_info_ik,infeasible_constraint_ik] = inverseKin(obj.r,q_start_nom,q_start_nom,...
%         drill_pos_constraint{end},drill_dir_constraint,posture_constraint,obj.ik_options);
%       
%       if(snopt_info_ik > 10)
%         send_msg = sprintf('snopt_info = %d. The IK (end) fails.',snopt_info_ik);
%         send_status(4,0,0,send_msg);
%         display(infeasibleConstraintMsg(infeasible_constraint_ik));
%         warning(send_msg);
%       end
%       
%       
%       qtraj_guess = PPTrajectory(foh([0 T],[q_start_nom, q_end_nom]));
%       
%       [xtraj,snopt_info,infeasible_constraint] = inverseKinTraj(obj.r,...
%         t_vec,qtraj_guess,qtraj_guess,...
%         drill_pos_constraint{:},drill_dir_constraint,posture_constraint,obj.free_ik_options);
%       
%       if(snopt_info > 10)
%         send_msg = sprintf('snopt_info = %d. The IK traj fails.',snopt_info);
%         send_status(4,0,0,send_msg);
%         display(infeasibleConstraintMsg(infeasible_constraint));
%         warning(send_msg);
%       end
%       
%       if obj.doVisualization && snopt_info <= 10
%         obj.v.playback(xtraj);
%       end
%       
%       if obj.doPublish && snopt_info <= 10
%         obj.publishTraj(xtraj,snopt_info);
%       end
%     end
  end
end