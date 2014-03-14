function joint_pos = getCurrentQfromLCM()
    
    lc = lcm.lcm.LCM.getSingleton();
    aggregator = lcm.lcm.MessageAggregator();
    
    lc.subscribe('PR2_STATE', aggregator);
    
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
    
    m = planner.pr2_state_t(msg.data);
    
    %disp(sprintf('decoded message:\n'))
    disp([ 'timestamp:   ' sprintf('%d ', m.utime) ])
    disp([ 'position:    ' sprintf('%f ', m.joint_position) ])
    %size(m.joint_position)
    
    joint_pos = zeros(48,1);
    offset = 6;  % base xyzrpy
  
    %joint_pos((offset+1):(offset+42)) = wrapToPi(reshape(m.joint_position(1:42), 42,1)); 
    joint_pos((offset+1):(offset+42)) = reshape(m.joint_position(1:42), 42,1);    
    

end