function sendDone()

lc = lcm.lcm.LCM.getSingleton();

msg = planner.reply_t();

msg.err = 'OK';

display('sending Drake DONE')
lc.publish('DRAKE', msg);

end