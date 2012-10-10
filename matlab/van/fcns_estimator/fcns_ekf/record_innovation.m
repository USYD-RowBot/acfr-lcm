function nu_t = record_innovation(nu_t,fhandle,t,nu,S,fni,fnj)
%function nu_t = record_innovation(nu_t,fhandle,t,nu,S,fni,fnj)
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    05-10-2004      rme         Created and written.

% create field
field = func2str(fhandle);
if ~isfield(nu_t,field)
  nu_t.(field).t   = t;
  nu_t.(field).nu  = nu;
  nu_t.(field).S   = S;
  nu_t.(field).fni = [];
  nu_t.(field).fnj = [];
else
  % record innovation data
  nu_t.(field).t(1,end+1)   = t;
  nu_t.(field).nu(:,end+1)  = nu;
  nu_t.(field).S(:,:,end+1) = S;
end

if exist('fni','var')
  nu_t.(field).fni(1,end+1) = fni;
  nu_t.(field).fnj(1,end+1) = fnj;
end
