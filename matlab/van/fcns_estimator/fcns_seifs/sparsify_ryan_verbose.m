function [eta_tilde,Lambda_tilde,Lambda_U,Lambda_V,Lambda_D] = sparsify_ryan_verbose(eta,Lambda,xti,Yoi,Ypi,Ymi)

N = length(xti)+length(Yoi)+length(Ypi)+length(Ymi);
I = speye(N);
Sx      = I(:,xti);            % S_xt
SYo     = I(:,Yoi);            % S_Yo
SYp     = I(:,Ypi);            % S_Y+
SYm     = I(:,Ymi);            % S_Y-
SxYo    = I(:,[xti,Yoi]);      % S_x_Yo
SxYp    = I(:,[xti,Ypi]);      % S_x_Y+
SxYm    = I(:,[xti,Ymi]);      % S_x_Y-
SYpYm   = I(:,[Ypi,Ymi]);      % S_Y+_Y-
SYoYp   = I(:,[Yoi,Ypi]);      % S_Yo_Y+
SYoYm   = I(:,[Yoi,Ymi]);      % S_Yo_Y-
SxYoYp  = I(:,[xti,Yoi,Ypi]);  % S_x_Yo_Y+
SxYoYm  = I(:,[xti,Yoi,Ymi]);  % S_x_Yo_Y-
SxYpYm  = I(:,[xti,Ypi,Ymi]);  % S_x_Yo_Y+
SYoYpYm = I(:,[Yoi,Ypi,Ymi]);  % S_Yo_Y+_Y-



Lambda_U1 = SxYp'*Lambda*SxYp
Lambda_U2 = SxYp'*Lambda*SYoYm
Lambda_U3inv = SYoYm'*Lambda*SYoYm
Lambda_U = SxYp'*Lambda*SxYp - SxYp'*Lambda*SYoYm*full(SYoYm'*Lambda*SYoYm)^-1*SYoYm'*Lambda*SxYp;
eta_U    = SxYp'*eta         - SxYp'*Lambda*SYoYm*full(SYoYm'*Lambda*SYoYm)^-1*SYoYm'*eta;

Lambda_V1 = SYp'*Lambda*SYp
Lambda_V2 = SYp'*Lambda*SxYoYm
Lambda_V3inv = SxYoYm'*Lambda*SxYoYm
Lambda_V = SYp'*Lambda*SYp - SYp'*Lambda*SxYoYm*full(SxYoYm'*Lambda*SxYoYm)^-1*SxYoYm'*Lambda*SYp;
eta_V    = SYp'*eta        - SYp'*Lambda*SxYoYm*full(SxYoYm'*Lambda*SxYoYm)^-1*SxYoYm'*eta;

Lambda_D1 = SYoYpYm'*Lambda*SYoYpYm
Lambda_D2 = SYoYpYm'*Lambda*Sx
Lambda_D3 = Sx'*Lambda*Sx
Lambda_D = SYoYpYm'*Lambda*SYoYpYm - SYoYpYm'*Lambda*Sx*full(Sx'*Lambda*Sx)^-1*Sx'*Lambda*SYoYpYm;
eta_D    = SYoYpYm'*eta            - SYoYpYm'*Lambda*Sx*full(Sx'*Lambda*Sx)^-1*Sx'*eta;


% calculate the sparsification rule according to Ryan
Lambda_tilde = SxYp*Lambda_U*SxYp' - SYp*Lambda_V*SYp' + SYoYpYm*Lambda_D*SYoYpYm';
eta_tilde    = SxYp*eta_U          - SYp*eta_V         + SYoYpYm*eta_D;
