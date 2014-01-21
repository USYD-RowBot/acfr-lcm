function zpredict = om_rdi_displacement(Xaug,X2i,X1i)
%INPUTS:
%  Xaug is the augmented state vector
%  X1i index of delayed state pose at time t1
%  X2i index of delayed state pose at time t2
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    08-18-2003      rme         Created and written.
%    09-28-2003      rme         Removed X_dtype code baggage.


% predicted displacement measurement 
zpredict = Xaug(X2i(1:3)) - Xaug(X1i(1:3));
