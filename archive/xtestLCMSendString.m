lc = lcm.lcm.LCM.getSingleton();

msg = planner.string_t();

msg.utime = 0;
msg.data = 'example string';

lc.publish('MANIP_EVENT', msg);
