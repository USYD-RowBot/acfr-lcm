function relaxMultiGrid(fh,Ncycle,Niter);


% do k v-cycle's
for k=1:Ncycle;
  TheJournal.Eif.mu(Xa_i) = vcycle(level,Nv,Xp_i,Niter);
end

%===============================================================
function eh_out = vcycle(level,fh,Nv,Xp_i,Niter);

global MG;

COARSEST_LEVEL = length(MG.Nh{level+1}) < MG.Nmin;


persistent eh;
if level == 0;
  eh{level+1} = xh;
end
if length(eh) < level+1
  if level > 0;
    eh{level+1} = zeros(size(xh));
  end
end;

if COARSEST_LEVEL;
  % solve directly
  Nh = length(bh)/Nv
  eh{level+1} = Ah\bh;
else;
  %figure(level+1); spy(Ah); title(sprintf('A^%d',level));
  % relax the system
  eh{level+1} = smooth(Ah,bh,eh{level+1},Nv,S);
  % compute our interpolator
  [IhH,xH] = interpolator(xh,Nv,Xp_i);
  % restrict our residual and linear system
  bH = IhH'*(bh - Ah*eh{level+1});
  AH = IhH'*Ah*IhH;
  % solve residual equation at next level
  eH = vcycle(level+1,AH,bH,xH,Nv,Xp_i,S);
  % correct solution
  eh{level+1} = eh{level+1} + IhH*eH;
  % relax the system
  eh{level+1} = smooth(Ah,bh,eh{level+1},Nv,S);
end;

eh_out = eh{level+1};

%================================================================
function eh = smooth(Ah,bh,eh,Nv,Ncycles);
Nf = length(eh)/Nv;
for k=1:Ncycles;
  % 1 iteration of Gauss-Siedel
  ih = 1:Nv;
  for fh=Nf:-1:1;
    eh(ih) = eh(ih) + Ah(ih,ih)\(bh(ih) - Ah(ih,:)*eh);
    ih = ih + Nv;
  end;
end;

%================================================================
function [IhH,xH] = interpolator(xh,Nv,Xp_i);
% number of fine states
Nh  = length(xh)/Nv;
% number of coarse states
if mod(Nh,2) == 0; % Nh even
  NH = Nh/2+1;
else;              % Nh odd
  NH = (Nh+1)/2;
end;

[xi,yi,zi] = deal(Xp_i(1),Xp_i(2),Xp_i(3));

ih = 1:Nv;
iH = 1:Nv;
xH  = zeros(Nv*NH,1);
IhH = spalloc(Nv*Nh,Nv*NH,1.5*Nv*Nh);
for fh=0:Nh-1;
  if mod(fh,2) == 0; %even
    xH(iH) = xh(ih);
    IhH(ih,iH) = speye(Nv);
    ih = ih+Nv;
    iH = iH+Nv;
  elseif fh==Nh-1;   % odd and "last"
    xH(iH) = xh(ih);
    IhH(ih,iH) = speye(Nv);
    ih = ih+Nv;
  else;              % otherwise
    a = xh(ih-Nv); b = xh(ih); c = xh(ih+Nv);

    % solve for weight gamma
    gamma = (b(zi)-a(zi))/(c(zi)-a(zi));
    gamma = max(1,min(-1,gamma)); % clip to [-1,1]
    % solve for weights alpha,beta
    M = [(c(xi)-a(xi)), -(c(yi)-a(yi)); (c(yi)-a(yi)), (c(xi)-a(xi))];
    z = [(b(xi)-a(xi));(b(yi)-a(yi))];
    y = M \ z;
    alpha = y(1); beta = y(2);
    alpha = max(1,min(0,y(1)));   % clip to [0,1]
    beta  = max(1,min(-1,y(2)));  % clip to [-1,1]
    
    % compute interpolation matrix E1
    E1 = 0.5*speye(Nv);
    E1(1,1) = (1-alpha); E1(1,2) = beta;
    E1(2,1) = -beta;     E1(2,2) = (1-alpha); 
    E1(3,3) = (1-gamma);
    % compute interpolation matrix E2
    E2 = 0.5*speye(Nv);
    E2(1,1) = alpha; E2(1,2) = -beta;
    E2(2,1) = beta;  E2(2,2) = alpha; 
    E2(3,3) = gamma;

    IhH(ih,[iH-Nv,iH]) = [E1,E2];
    ih = ih+Nv;
  end
end;
