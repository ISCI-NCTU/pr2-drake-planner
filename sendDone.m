function sendDone()
    
lc = lcm.lcm.LCM.getSingleton();
     
    msg = planner.reply_t();

    msg.timestamp = 0;
    msg.name = 'OK';
    
display('sending Drake DONE')
lc.publish('DRAKE', msg);

end