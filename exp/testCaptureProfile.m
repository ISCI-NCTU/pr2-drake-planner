% Capture profile

% 0 initialization
toPause = true;
getJointAvailable = true;
toPublish = true;
useGripperController = true;
doPrepare = true;
T = 10;
offset = 4;
leftOrRight = 'r';  % left hand or right hand

drawer_close_pos = [0.82,-0.52,1.19-0.2]'; % specify drawer pose (x,y,z)
drawer_open_pos =  [0.4+0.1,-0.52,1.19-0.2-0.02+0.01]'; 

%~/pr2/ros_ws/simple_head$ bin/point_head 0.5 -0.52 0.99
%system('rosbag', 'camera', '/ft_compensated')
for i = 1:50
  fprintf('%dth iteration\n', i);
  testDrawerOpenCloseProfile();
  doPrepare = false;
end

