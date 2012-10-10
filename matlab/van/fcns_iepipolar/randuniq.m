function rsel = randuniq(setsize,sampsize);
%RANDUNIQ  Randomly picks unique indexs into sample vector.
%  RSEL = RANDUNIQ(SETSIZE,SAMPSIZE) returns RSEL which is a
%  randomly chosen index vector of length SAMPSIZE into the sample vector
%  of length SETSIZE.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    12-2002         op          Created and written.
%    12/20/2002      rme         Added header.
%    09-27-2003      rme         Renamed to randuniq.m

if (setsize >= sampsize)
  rsel = unique(ceil(setsize*rand(sampsize,1)));
  while length(rsel) < sampsize
    rsel = unique([rsel; ceil(setsize*rand(sampsize,1))]);
  end
  rsel = rsel(1:sampsize);
else
  error('sample size greater than set size');
end
 
