function getActionReplyFromLCM()
    ChannelName = 'ACTION_REPLY';
    lc = lcm.lcm.LCM.getSingleton();
    aggregator = lcm.lcm.MessageAggregator();
    aggregator.setMaxMessages(3);
    aggregator.setMaxBufferSize(2^20);  % MB
    
    lc.subscribe(ChannelName, aggregator);
    
    
    disp waiting
    cnt = 0;
    while true
        cnt = cnt + 1;
        if mod(cnt,100) == 0
          fprintf('.');
        end
        millis_to_wait = 10;
        msg = aggregator.getNextMessage(millis_to_wait);
        if length(msg) > 0
            break
        end
    end
    
    fprintf('LCM Received: %s : %s\n', ChannelName, char(msg.data))
    
    lc.unsubscribe(ChannelName, aggregator);
    %lc.close()
    
    clear aggregator
    clear lc
end