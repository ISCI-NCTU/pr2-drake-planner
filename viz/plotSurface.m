
% x = [-1.33176 0.377255 -2.32682 0.00899993 -0.00242568 0.0464276;
% 3.28247 2.04405 -4.51126 0.233672 -0.860645 -0.000594114;
% 7.28813 3.45998 -7.50478 0.375245 -1.44991 -0.0461277;
% 9.13594 4.09016 -8.68164 0.402953 -1.48247 -0.0679276;
% 9.54509 4.38052 -9.94586 0.369687 -1.33424 -0.115868;
% -0.727677 0.657395 -2.77354 0.0912028 -0.248047 0.121889;
% 3.98436 2.11028 -5.68688 0.287786 -1.15493 0.105162;
% 7.49306 3.18658 -8.45414 0.427045 -1.72266 0.0772135;
% 7.82968 3.55345 -9.61919 0.405269 -1.49249 -0.0165568;
% 9.80886 4.4535 -11.0468 0.426207 -1.56579 -0.111034;
% -1.11278 0.618784 -2.66523 0.0905386 -0.171743 0.139085;
% 3.83708 2.01375 -5.6756 0.27438 -1.11249 0.123159;
% 7.64492 3.23998 -8.67659 0.431486 -1.72915 0.108014;
% 8.3832 3.65651 -9.95242 0.426718 -1.57236 0.0283987;
% 9.85299 4.35347 -11.0386 0.404538 -1.50783 -0.0700926;
% -1.01443 0.553063 -2.766 0.0799879 -0.205311 0.147274;
% 3.80121 2.0321 -5.49366 0.278146 -1.09674 0.13678;
% 7.67858 3.33989 -8.53551 0.443155 -1.70868 0.125189;
% 8.45195 3.78992 -9.84426 0.442735 -1.54235 0.0575701;
% 9.90344 4.37604 -11.1117 0.407704 -1.49036 -0.0471371;
% -1.23467 1.10198 -2.97157 0.0947826 -0.210462 0.104573;
% 3.43948 2.49068 -5.67513 0.31712 -1.06237 0.135181;
% 7.57262 3.46353 -8.84483 0.449958 -1.69052 0.139924;
% 8.40755 3.77723 -9.86024 0.440483 -1.52212 0.0811846;
% 9.66003 4.49144 -11.0786 0.429338 -1.45975 -0.016163];



figure
zz = double(allResult(:,1));
z = reshape(zz,5,5)';
surf(deltas,deltas,z);
xlabel('dy');
ylabel('dz');