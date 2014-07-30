function controlHead(direction)
  system(sprintf('rosrun simple_head point_head %f %f %f', direction(1), direction(2), direction(3)));