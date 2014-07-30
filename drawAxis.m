function drawAxis(r,lcmgl,axisName, q0)


scale = 0.5;

r_gripper_idx = findLinkInd(r,axisName);
o_pt = [0,0,0]';
x_pt = [1,0,0]' * scale;
y_pt = [0,1,0]' * scale;
z_pt = [0,0,1]' * scale;

kinsol = r.doKinematics(q0(1:r.getNumDOF));
poso = r.forwardKin(kinsol,r_gripper_idx,o_pt);
posx = r.forwardKin(kinsol,r_gripper_idx,x_pt);
posy = r.forwardKin(kinsol,r_gripper_idx,y_pt);
posz = r.forwardKin(kinsol,r_gripper_idx,z_pt);

lcmgl.glColor3f(1,0,0);
lcmgl.line3(poso(1),poso(2),poso(3),posx(1),posx(2),posx(3));
lcmgl.glColor3f(0,1,0);
lcmgl.line3(poso(1),poso(2),poso(3),posy(1),posy(2),posy(3));
lcmgl.glColor3f(0,0,1);
lcmgl.line3(poso(1),poso(2),poso(3),posz(1),posz(2),posz(3));
