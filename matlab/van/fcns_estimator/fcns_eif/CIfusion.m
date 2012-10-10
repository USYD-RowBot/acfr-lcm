function [P_x_lvi_CI,omega,dt] = CIfusion(TheJournal,meas_t,TheConfig);
%function [P_x_lvi_CI,omega,dt] = CIfusion(TheJournal,meas_t,TheConfig);  
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    11-27-2004      rme         Created and written.
%    11-28-2004      rme         Fixed error in five2six.
%    11-28-2004      rme         Don't update if fni==1, because Pc_xlv1 is nearly singular.
%    12-06-2004      rme         Solve for "truth" in case Pc_joint is not SPD
%    12-08-2004      rme         Renamed internal function to pseudoPoseMeas()
%    12-21-2004      rme         Added output dt which doesn't include plot time.

aclock('tic');

% dissect meas_t
fni  = meas_t.fni;
fnj  = meas_t.fnj;
z_cjci = meas_t.z;
R_z_cjci = meas_t.R;

% shorthand index
Xp_i  = TheJournal.Index.Xp_i;
Xfi_i = TheJournal.Index.Xf_ii{fni}(Xp_i);
Xfj_i = TheJournal.Index.Xf_ii{fnj}(Xp_i);

% local-level vehicle poses
x_lvi = TheJournal.Eif.mu(Xfi_i);
x_lvj = TheJournal.Eif.mu(Xfj_i);

% conservative SLAM filter covariance
Pt_x_lvj = TheJournal.Eif.SigmaCol(Xfj_i,:);  % "true" covariance of x_lvj
Pt_vivj  = TheJournal.Eif.SigmaCol(Xfi_i,:);  % "true" cross-covariance
Pc_x_lvi = TheJournal.Eif.SigmaCI{fni};       % "conservative" covariance of x_lvi
Pc_joint = [Pc_x_lvi, Pt_vivj; ...            % Pc_joint is a "conservative" SPD covariance matrix
	    Pt_vivj', Pt_x_lvj];

% static camera to vehicle xform
x_vc = TheConfig.SensorXform.PXF.x_vs;

% this test just illustrates that as long as Pc_x_wi - Pt_x_wi > 0
% is true, then Pc_joint will also be positive definite
[R,notPosDef] = chol(Pc_joint);
if notPosDef; 
  setTerminal('error');
  fprintf('***%s: Pc_joint not Pos Def!\n',mfilename);
  setTerminal('restore');
  Pt_x_lvi = computeTruth(fni);
  vpause(30,'Pc\_joint not Pos Def!',1);
  % reset to true
  P_x_lvi_CI = Pt_x_lvi;
  omega = -1; % set warning flag
  dt = aclock('toc');
  return;
end;

% PSEUDO GLOBAL POSE MEASUREMENT
%====================================================
% pseudo measurement of x_lvi and its covariance
[x_lvi_meas,Rc_x_lvi_meas] = pseudoPoseMeas(x_lvi,x_lvj,z_cjci,x_vc,Pc_joint,R_z_cjci);
% heuristically bump up covariance a bit
%S = diag([1.75,1.75,1.75,1.25,1.25,1.25]);
S = diag([1.05,1.05,1,1,1,1]);
%S = diag([1,1,1,1,1,1]);
Rc_x_lvi_meas = S*Rc_x_lvi_meas*S';


% use covariance intersection to get a better estimate of global pose x_lvi
%-----------------------------------------------------------
if fni == 1;
  % do not update covariance bound for 1st image which is the "anchor" image
  P_x_lvi_CI = Pc_x_lvi;
  omega = 1.0;
else;
  try;
    [P_x_lvi_CI,omega] = covintersect(Pc_x_lvi,Rc_x_lvi_meas,'trace');
  catch;
    disp('Cierror');
    keyboard;
    P_x_lvi_CI = zeros(6);
    omega = 1;
  end;
end

% this test just illustrates that as long as Pc_x_wi - Pt_x_wi > 0
% is true, then Pc_joint will also be positive definite
Pc_joint_update = Pc_joint;
Pc_joint_update(1:6,1:6) = P_x_lvi_CI;
[R,notPosDef] = chol(Pc_joint_update);
if notPosDef; 
  setTerminal('error');
  fprintf('***%s: Pc_joint_update not Pos Def!\n',mfilename);
  setTerminal('restore');
  Pt_x_lvi = computeTruth(fni);
  vpause(30,'Pc\_joint\_update not Pos Def!',1);
  % reset to true
  P_x_lvi_CI = Pt_x_lvi;
  omega = -1; % set warning flag
  dt = aclock('toc');
  return;
end;

dt = aclock('toc');

if TheConfig.Plot.CIfusion;
  private_plotCIfusion(fni,fnj,omega,Pc_x_lvi,Rc_x_lvi_meas,P_x_lvi_CI);
end;

%=====================================================================
function [x_lvi_meas0,Rc_x_lvi_meas0] = pseudoPoseMeas(x_lvi,x_lvj,z_cjci,x_vc,Pc_joint,R_z_cjci);
% vehicle to camera xform
x_cv = inverse(x_vc);
% parameter vector
pvec = [x_lvi; x_lvj; z_cjci];

% pseudo global vehicle pose measurement
x_lvi_meas0 = five2six(pvec,x_vc,x_cv);
% pseudo measurement Jacobian
J_x_lvi_meas0 = numerical_jacobian(@five2six,pvec,x_lvi_meas0,[],x_vc,x_cv);
% sudo measurement covariance
Rc_x_lvi_meas0 = J_x_lvi_meas0*blkdiag(Pc_joint,R_z_cjci)*J_x_lvi_meas0';

%[x_lvi_meas1,Rc_x_lvi_meas1] = sigmaTransfer(@five2six,pvec,blkdiag(Pc_joint,R_z_cjci),[],x_vc,x_cv);

%=====================================================================
function x_lvi_meas = five2six(pvec,x_vc,x_cv);

% decompose parameter vector
x_lvi  = pvec(1:6);
x_lvj  = pvec(7:12);
z_cjci = pvec(13:17);

% map vehicle poses to camera poses
x_lci = head2tail(x_lvi,x_vc);
x_lcj = head2tail(x_lvj,x_vc);

% baseline magnitude according to nav
x_cjci = tail2tail(x_lcj,x_lci);
scale = norm(x_cjci(1:3));

% z_cjci = [az,el,r,p,h]
ae  = z_cjci(1:2);
rph = z_cjci(3:5);

% construct a direction-magnitude vector t_dm = [az,el,mag]
t_dm = [ae;scale];

% map the direction-magnitude vector to a translation vector t = [tx,ty,tz]
t = dm2trans(t_dm);

% relative pose vector
x_cjci_meas = [t;rph];

% map relative camera pose meas to a relative vehicle pose meas
x_cjvi_meas = head2tail(x_cjci_meas,x_cv);
x_vjvi_meas = head2tail(x_vc,x_cjvi_meas);

% sudo global vehicle pose measurement
x_lvi_meas = head2tail(x_lvj,x_vjvi_meas);

%=======================================================================================
function Pt_x_lvi = computeTruth(fni);
global TheJournal;

% pointers into TheJournal
Np     = TheJournal.Index.Np;
Naug   = TheJournal.Index.Naug;
Xp_i   = TheJournal.Index.Xp_i;
Xfi_i  = TheJournal.Index.Xf_ii{fni}(Xp_i);
Xa_i   = TheJournal.Index.Xa_i;
Lambda = TheJournal.Eif.Lambda(Xa_i,Xa_i);

% basis vectors of most recent delayed state pose elements
E = spalloc(Naug,Np,Np);
E(Xfi_i,:) = speye(Np);
  
% true covariance
SigmaCol = Lambda \ E;
Pt_x_lvi = symmetrize(full(SigmaCol(Xfi_i,:)));

%=======================================================================================
function private_plotCIfusion(fni,fnj,omega,Pc_x_lvi,Rc_x_lvi_meas,P_x_lvi_CI);
global TheJournal;

Pt_x_lvi = computeTruth(fni);

% plot comparison
mu = [0;0];
alpha = 1-2*normcdf(-3);
k2 = chi2inv(alpha,2);
h(1) = draw_ellipse(mu,Pt_x_lvi(1:2,1:2),k2,'k');
hold on;
h(2) = draw_ellipse(mu,Pc_x_lvi(1:2,1:2),k2,'g-.');
draw_ellipse(mu,P_x_lvi_CI(1:2,1:2),k2,'c');
axis equal;
axe = axis;
h(3) = draw_ellipse(mu,Rc_x_lvi_meas(1:2,1:2),k2,'m-.');
h(4) = draw_ellipse(mu,P_x_lvi_CI(1:2,1:2),k2,'c');
legend(h,'Pt x_{lvi}','Pc x_{lvi}','Rc x_{lvi}', 'CI');
title(sprintf('CI for (fni,fnj) = (%d,%d)\n\\omega=%.2f',fni,fnj,omega));
hold off;
axis(axe);

% sanity check
detPt = det(Pt_x_lvi)^(1/length(Pt_x_lvi));
detPc = det(Pc_x_lvi)^(1/length(Pc_x_lvi));
if detPc < detPt;
  setTerminal('warning');
  fprintf('***%s: detPc=%f < detPt=%f\n',mfilename,detPc,detPt);
  setTerminal('restore');
  evalin('caller','keyboard');
end;    
