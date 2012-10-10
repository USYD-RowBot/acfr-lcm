% to test horn's relative orientation with matlab
%
% usage:
%  $ cd "to van direcotry"
%  $ addpath(genpath(pwd));
%  $ cd "to perls/opt/examples/libvision/"
%  $ test_relorient

function test_relorient

dir = '~/perls/opt/examples/libvision/_test_corr_files/';
k = load([dir,'k.txt']);
x21 = load([dir,'x21.txt']);
x12 = load([dir,'x12.txt']);
p21 = load([dir,'p21.txt']);
p12 = load([dir,'p12.txt']);
uv1 = load([dir,'uv1.txt']);
uv2 = load([dir,'uv2.txt']);

n =1000;
uv1 = reshape(uv1,n,2)';
uv2 = reshape(uv2,n,2)';
k = reshape(k,3,3)';
p21 = reshape(p21,6,6)';
p12 = reshape(p12,6,6)';

u1 = uv1(1,:)'; v1 = uv1(2,:)';
u2 = uv2(1,:)'; v2 = uv2(2,:)';

% normalize coordinate
R=rotxyz(x21(4:6));
xy1_h=inv(k)*[u1'; v1'; ones(1,n)];
[x1, y1]=dehomogenize(xy1_h);
xy2_h=inv(k)*[u2'; v2'; ones(1,n)];
[x2, y2]=dehomogenize(xy2_h);

% test relorient_horn
tic;
[R,t,E] = relorient_horn(x1,y1,x2,y2,R);
toc;

save _test_corr_files/ans_R_horn.txt R -ASCII -DOUBLE
save _test_corr_files/ans_t_horn.txt t -ASCII -DOUBLE
save _test_corr_files/ans_E_horn.txt E -ASCII -DOUBLE

%% test relorient sample
% normalize pose prior 5 DOF camera measurement
[t_dm,J] = trans2dm(x21(1:3));
npp_21(1:2,1) = t_dm(1:2);
npp_21(3:5,1) = x21(4:6);
J = [J(1:2,:),  zeros(2,3); ...
     zeros(3),  eye(3)];
Cov_npp21 = J*p21*J';

[R,t,cost] = relorient_sample(x1,y1,x2,y2, ...
     	                      x21(4:6),p21(4:6,4:6),-1);

% for each R,t solution compute its Mahalanobis distance w.r.t. our prior
number_of_solns = size(t,2);
min_mdist = inf;
for ii=1:number_of_solns;
  R_ambig = squeeze(R(:,:,ii));  rph_ambig = rot2rph(R_ambig); t_ambig = t(:,ii);
  % compute normalized 5 DOF measurement unwrapping the solution so that
  % it is consistent with pose prior
  t_dm = trans2dm(t_ambig);
  npp_ambig = [t_dm(1:2); rph_ambig];
  npp_ambig = funwrap([npp_21,npp_ambig],2);
  npp_ambig = npp_ambig(:,2);
  % compute the mahalanobis distance
  mdist(ii) = mahalanobis_dist(npp_ambig,npp_21,Cov_npp21);
  % keep the solution with the minimum Mahalanobis distance
  if mdist(ii) < min_mdist;
    min_mdist = mdist(ii);
    msel = ii;
    rph_o = rph_ambig;
    R_o = rotxyz(rph_o);
    t_o = t_ambig;
  end;
end;

p_o = [t_o; rot2rph(R_o);];
save _test_corr_files/ans_p21_horn.txt p_o -ASCII -DOUBLE
              
%% test mahalanobis distance              
[t_dm,J] = trans2dm(x12(1:3));
x12_5dof = [t_dm(1:2); x12(4:6)];
J = [J(1:2,:),  zeros(2,3); zeros(3),  eye(3)];
p12_5dof = J*p12*J';

[t_dm,J] = trans2dm(x21(1:3));
x21_5dof = [t_dm(1:2); x21(4:6)];
J = [J(1:2,:),  zeros(2,3); zeros(3),  eye(3)];
p21_5dof = J*p21*J';

mdist1 = mahalanobis_dist (x12_5dof, x21_5dof, p12_5dof);
mdist2 = mahalanobis_dist (x12_5dof, x21_5dof, p21_5dof);

mdist = min(mdist1, mdist2);
save _test_corr_files/ans_mdist.txt mdist -ASCII -DOUBLE
