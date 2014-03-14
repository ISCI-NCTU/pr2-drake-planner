    
lc = lcm.lcm.LCM.getSingleton();
aggregator = lcm.lcm.MessageAggregator();

lc.subscribe('KINECT_TO_WORLD', aggregator);

while true
    disp waiting
    millis_to_wait = 1000;
    msg = aggregator.getNextMessage(millis_to_wait);
    if length(msg) > 0
        break
    end
end

disp(sprintf('channel of received message: %s', char(msg.channel)))
%disp(sprintf('raw bytes of received message:'))
%disp(sprintf('%d ', msg.data'))

m = planner.tf_t(msg.data);

%disp(sprintf('decoded message:\n'))
disp([ 'timestamp:   ' sprintf('%d ', m.utime) ])
disp([ 'trans:    ' sprintf('%f ', m.trans) ])
disp([ 'rot: ' sprintf('%f ', m.rot) ])


kinect2world_pos = reshape(m.trans, 3,1); % pregrasp pose (x,y,z)
kinect2world_orient = reshape(m.rot, 4, 1);  

%grasp_pos = [0 0 1]';
%grasp_orient = [0 0 0 1]';

kinect2world_orient = quat_xyzw2wxyz(kinect2world_orient);

grasp_orient = quat_xyzw2wxyz(grasp_orient);
moveto_orient = quat_xyzw2wxyz(moveto_orient);
postgrasp_orient = quat_xyzw2wxyz(postgrasp_orient);

grasp_pos = quatrotate(kinect2world_orient', grasp_pos')'+ kinect2world_pos;
grasp_orient = quatmultiply(grasp_orient', kinect2world_orient')';

moveto_pos = quatrotate(kinect2world_orient', moveto_pos')'+ kinect2world_pos;
moveto_orient = quatmultiply(moveto_orient', kinect2world_orient')';

postgrasp_pos = quatrotate(kinect2world_orient', postgrasp_pos')' + kinect2world_pos;
postgrasp_orient = quatmultiply(postgrasp_orient', kinect2world_orient')';


grasp_pos
grasp_orient

moveto_orient
moveto_orient
