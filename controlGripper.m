function controlGripper(openOrClose, leftOrRight)
  system(sprintf('rosrun simple_gripper simple_gripper %s %s', openOrClose, leftOrRight));