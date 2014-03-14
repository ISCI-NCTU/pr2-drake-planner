while true
    
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
    disp(sprintf('raw bytes of received message:'))
    disp(sprintf('%d ', msg.data'))
    
    m = planner.pose_t(msg.data);
    
    disp(sprintf('decoded message:\n'))
    disp([ 'timestamp:   ' sprintf('%d ', m.utime) ])
    disp([ 'position:    ' sprintf('%f ', m.pos) ])
    disp([ 'orientation: ' sprintf('%f ', m.orientation) ])
    

end
