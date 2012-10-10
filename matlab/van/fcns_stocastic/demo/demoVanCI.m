clear all;

% random global poses
Si = diag([0.1;0.1;0.1;1*DTOR;1*DTOR;1*DTOR]);
Sj = diag([.05;0.05;0.05;1*DTOR;1*DTOR;1*DTOR]);
x_wi = Si*randn(6,1)+ diag([1,1,0,1,1,360])*rand(6,1);
x_wj = Sj*randn(6,1);

% SLAM filter covariances with x_wi being more uncertain that x_wj
S = blkdiag(Si,Sj);
Pt_joint = S*randcov(12)*S';   % true-joint covariance
Pt_x_wi = Pt_joint(1:6,1:6);
Pt_x_wj = Pt_joint(7:12,7:12);

% conservative covariance estimate for x_wi
Pc_x_wi = Pt_x_wi + Si*randcov(6)*Si';

% by replacing Pt_x_wi with Pc_x_wi_UB when can construct a
% convervative joint-covariance estimate Pc_joint
Pc_joint = Pt_joint;
Pc_joint(1:6,1:6) = Pc_x_wi;

% this test just illustrates that as long as Pc_x_wi - Pt_x_wi > 0
% is true, then Pc_joint will also be positive definite
[R,notPosDef] = chol(Pc_joint);
if notPosDef; 
  error('Pc_joint not Pos Def!\n'); 
end;

CIfusionDemo(x_wi,x_wj,Pt_joint,Pc_x_wi);
