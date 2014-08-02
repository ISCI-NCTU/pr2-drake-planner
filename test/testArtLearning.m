while true
    warning('off','all');
    DRAKE_PATH = '/home/drc/drc/software/drake';
    if(~exist('r','var'))
      fprintf('Loading Robot Model...');
      r = RigidBodyManipulator(strcat(DRAKE_PATH,'/examples/PR2/pr2.urdf'),struct('floating',true));
      fprintf('Done\n');
    end
    
    
    getPlannerTargetfromLCM;
    getKinect2WorldTFfromLCM;
    
    
    testDrawer_withArt;
end