
mysetenv;

DRAKE_PATH = '/home/drc/drc/software/drake';
r = RigidBodyManipulator(strcat(DRAKE_PATH,'/examples/PR2/pr2.urdf'),struct('floating',true));