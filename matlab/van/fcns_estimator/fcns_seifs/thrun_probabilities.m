clc;

echo on;
if ~exist('loaddat','var') || loaddat ~= 1
  loaddat = 1;
  load /files1/processed/van/output/jhu04-6_gridsurvey3/CamResults/2000-3000wc.20040421.mat
  load /files1/processed/van/output/jhu04-6_gridsurvey3/nav_t;
end
echo off;
  
% covariance matrix
Paug = ssa_t.TheJournal{end}.Paug;
Paug = Paug(13:end,13:end);

% information matrix
Haug = inv(Paug);

% normalized information matrix
thresh = 1e-10;
nHaug = rhomatrix(Haug);
ii = find(abs(nHaug) < thresh);

% modified information matrix
% set small normalized elements to zero
Haug_mod = Haug;
Haug_mod(ii) = 0;

% modified covariance matrix
Paug_mod = inv(Haug_mod);

% states xi & xj
n = 3;
xi = [1:n]' + 6*20;
%xj = [1:n]' + 6*30;
xj =  [];
xij = [xi; xj];

% exact covariance matrix of joint distribution
Pij = Paug_mod(xij,xij);

% markov blankets associated with each
xi_plus = markov_blanket(Haug_mod,xi)';
xj_plus = markov_blanket(Haug_mod,xj)';

mb = intersect(xi_plus,xj_plus);
if isempty(mb)
  fprintf('Null Markov blanket intersection\n');
  xij_plus = [xi_plus; xj_plus];
else
  fprintf('Full blanket\n');
  xij_plus = markov_blanket(Haug_mod,xij)';
end

% approx covariance matrix of joint distribution
kk = [xij; xij_plus];
Hij_blanket = Haug_mod(kk,kk);
Pij_blanket = Hij_blanket^-1;
mm = 1:length(xij);
Pij_tilde = Pij_blanket(mm,mm);

disp('Pij'); disp(full(Pij));
disp('Pij_tilde'); disp(full(Pij_tilde));
disp('Rij'); disp(full(rhomatrix(Pij)));
disp('Rij_tilde'); disp(full(rhomatrix(Pij_tilde)));
