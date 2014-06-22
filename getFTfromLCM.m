function ft = getFTfromLCM()

lc = lcm.lcm.LCM.getSingleton();
aggregator = lcm.lcm.MessageAggregator();

lc.subscribe('PR2_FT', aggregator);

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

m = planner.force_torque_t(msg.data);

disp([ 'timestamp:   ' sprintf('%d ', m.utime) ])
disp([ 'r_hand_force:    ' sprintf('%f ', m.r_hand_force) ])
disp([ 'r_hand_torque:    ' sprintf('%f ', m.r_hand_torque) ])
disp([ 'l_hand_force:    ' sprintf('%f ', m.l_hand_force) ])
disp([ 'l_hand_torque:    ' sprintf('%f ', m.l_hand_torque) ])

ft = [m.r_hand_force(:); m.r_hand_torque(:)];


