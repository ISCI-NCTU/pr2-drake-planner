    
lc = lcm.lcm.LCM.getSingleton();
aggregator = lcm.lcm.MessageAggregator();
aggregator.setMaxMessages(3);
aggregator.setMaxBufferSize(2^20);  % MB

lc.subscribe('KINECT_TO_WORLD', aggregator);

while true
    disp waiting
    millis_to_wait = 1000;
    msg = aggregator.getNextMessage(millis_to_wait);
    if length(msg) > 0
        break
    end
end


lc.unsubscribe('KINECT_TO_WORLD',aggregator);
%lc.close()

clear aggregator
clear lc
    

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

grasp_pos = quatrotate(kinect2world_orient', (grasp_pos- kinect2world_pos)')'; %;
%grasp_orient = quatmultiply(grasp_orient', kinect2world_orient')';
grasp_orient = quatdivide(grasp_orient', kinect2world_orient')';

grasp_pos
grasp_orient


release_orient = quat_xyzw2wxyz(release_orient);
release_pos = quatrotate(kinect2world_orient', (release_pos- kinect2world_pos)')';
release_orient = quatdivide(release_orient', kinect2world_orient')';

postrelease_orient = quat_xyzw2wxyz(postrelease_orient);
postrelease_pos = quatrotate(kinect2world_orient', (postrelease_pos- kinect2world_pos)')';
%postrelease_pos = quatrotate(kinect2world_orient', (postrelease_pos- kinect2world_pos)')' - [0.05,0,0]';
postrelease_orient = quatdivide(postrelease_orient', kinect2world_orient')';


pregrasp_pos = release_pos;
pregrasp_orient = release_orient;

release_pos
postrelease_pos



