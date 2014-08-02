function ft = getFTfromLCM()

ChannelName = 'PR2_COMPENSATED_FT';
lc = lcm.lcm.LCM.getSingleton();
aggregator = lcm.lcm.MessageAggregator();
aggregator.setMaxMessages(3);
aggregator.setMaxBufferSize(2^20);  % MB

lc.subscribe(ChannelName, aggregator);

nsample = 100;
cnt = nsample;
ft = zeros(6,1);
while cnt > 0
    %disp waiting
    millis_to_wait = 1000;
    msg = aggregator.getNextMessage(millis_to_wait);
    if length(msg) > 0
        cnt = cnt - 1;
    end
    
    m = planner.force_torque_t(msg.data);
    ft = ft+ [m.r_hand_force(:); m.r_hand_torque(:)];
end

ft = ft / nsample;

lc.unsubscribe(ChannelName, aggregator);
%lc.close()

clear aggregator
clear lc
    
%disp(sprintf('channel of received message: %s', char(msg.channel)))
%disp(sprintf('raw bytes of received message:'))
%disp(sprintf('%d ', msg.data'))

%m = planner.force_torque_t(msg.data);

%disp([ 'timestamp:   ' sprintf('%d ', m.utime) ])
%disp([ 'r_hand_force:    ' sprintf('%f ', m.r_hand_force) ])
%disp([ 'r_hand_torque:    ' sprintf('%f ', m.r_hand_torque) ])
%disp([ 'l_hand_force:    ' sprintf('%f ', m.l_hand_force) ])
%disp([ 'l_hand_torque:    ' sprintf('%f ', m.l_hand_torque) ])


%ft = [m.r_hand_force(:); m.r_hand_torque(:)];


