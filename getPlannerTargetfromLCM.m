    
lc = lcm.lcm.LCM.getSingleton();
aggregator = lcm.lcm.MessageAggregator();

lc.subscribe('PLANNER_TARGET', aggregator);

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

m = planner.keypose_t(msg.data);

%disp(sprintf('decoded message:\n'))
disp([ 'timestamp:   ' sprintf('%d ', m.utime) ])
disp([ 'grasp_position:    ' sprintf('%f ', m.grasp_pos) ])
disp([ 'grasp_orientation: ' sprintf('%f ', m.grasp_orientation) ])
disp([ 'moveto_position:    ' sprintf('%f ', m.moveto_pos) ])
disp([ 'moveto_orientation: ' sprintf('%f ', m.moveto_orientation) ])
disp([ 'postgrasp_position:    ' sprintf('%f ', m.postgrasp_pos) ])
disp([ 'postgrasp_orientation: ' sprintf('%f ', m.postgrasp_orientation) ])


grasp_pos = reshape(m.grasp_pos, 3,1); % pregrasp pose (x,y,z)
grasp_orient = reshape(m.grasp_orientation, 4, 1);  

release_pos = reshape(m.moveto_pos, 3, 1); 
release_orient =  reshape(m.moveto_orientation, 4, 1); 

postrelease_pos = reshape(m.postgrasp_pos, 3, 1); 
postrelease_orient = reshape(m.postgrasp_orientation, 4, 1); 
    
