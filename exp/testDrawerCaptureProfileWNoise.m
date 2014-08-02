% Capture profile

testAngle = false;
testPos = true;

testAngle_e = false;
testPos_e = false;

% 0 initialization
toPause = true;
getJointAvailable = true;
toPublish = true;
useGripperController = true;
doPrepare = true;
doPlaybackPause = false;
T = 10;
offset = 4;
leftOrRight = 'r';  % left hand or right hand

drawer_close_pos_o = [0.82,-0.52,1.19-0.2-0.01]'; % specify drawer pose (x,y,z)
drawer_open_pos_o =  [0.4+0.1,-0.52,1.19-0.2-0.01]'; 
drawer_close_angle_o = [0,0,1.57-0.18];
drawer_open_angle_o = [0,0,1.57-0.18];

step = 0.005;  % 5mm
steprpy = 5/180*pi;  % 5deg

if testPos
  deltas = [-2 -1 0 1 2] * step;
else
  deltas = 0;
end

if testAngle
  deltarpy = [-2 -1 0 1 2] * steprpy;
else
  deltarpy = 0;
end

if testPos_e
  deltas_e = [-2 -1 0 1 2] * step;
else
  deltas_e = 0;
end

if testAngle_e
  deltarpy_e = [-2 -1 0 1 2] * steprpy;
else
  deltarpy_e = 0;
end



allResult = zeros(0,6);
%~/pr2/ros_ws/simple_head$ bin/point_head 0.5 -0.52 0.99
%system('rosbag', 'camera', '/ft_compensated')
i = 0;
for dx = 0
for dy = deltas
for dz = deltas
for dr = deltarpy
for dp = deltarpy
for dya = deltarpy
for dx_e = deltas_e
for dy_e = deltas_e
for dz_e = deltas_e
for dr_e = deltarpy_e
for dp_e = deltarpy_e
for dya_e = deltarpy_e
  i = i + 1;
  drawer_close_pos = drawer_close_pos_o + [dx dy dz]';
  drawer_close_angle = myangle2quat(drawer_close_angle_o+[dr dp dya])';
  drawer_open_pos = drawer_open_pos_o + [dx_e dy_e dz_e]';
  drawer_open_angle = myangle2quat(drawer_open_angle_o+[dr_e dp_e dya_e])';
  
  experiment_info = sprintf('%dth iteration dxyz_start(%g %g %g) drpy_start(%g %g %g) dxyz_end(%g %g %g) drpy_end(%g %g %g)\n', ...
             i, dx,dy,dz, dr,dp,dya, dx_e, dy_e, dz_e, dr_e, dp_e, dya_e);
  fprintf(experiment_info);
  testDrawerOpenCloseProfile();
  doPrepare = false;
  fprintf('result: %g %g %g %g %g %g\n', ft(1),ft(2),ft(3),ft(4),ft(5),ft(6));
  
  allResult=[allResult; reshape(ft, 1, 6)];
end
end
end
end
end
end
end
end
end
end
end
end
%end


