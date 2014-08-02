
  for i=1:0.5:T
    q = getCurrentQfromLCM();
    [~,xtraj,~,~,~] = evalc('planner.createJointPlan(q,q,0.001,true,true, 2);');
    planner.v.playback(xtraj);
    pause(0.5);
  end
