function controlTorso(value) % value 0.01 down ~0.33 up
    system(sprintf('rosrun roslcm_bridge lcmros_torso pos %f', value));