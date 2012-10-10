function ii = clippoints(Pts,dataset);
%function ii = clippoints(Pts,dataset);
%
%  Run `type clippoints` to view configured datasets in the switch statement.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    05-09-2005      rme         Created and written.
%    01-13-2006      rme         Added dataset flag and help file.

switch dataset;
case 'titanic04';
 tolN = 0.05;    % unit baseline triangulation error
 tolS = 0.075;   % nav scaled triangulation error
 minS =  6.5;    % min scene depth from camera, scale set by nav
 maxS = 11;      % max scene depth from camera, scale set by nav
 minN = 2;       % min scene depth from camera, unit baseline
 maxN = 12;      % max scene depth from camera, unit baseline
case 'chios05-03';
 tolN = 0.05;
 tolS = 0.075;
 minS = 2.5;
 maxS = 3.0;
 minN = 2;
 maxN = 12; 
end;    fprintf('\r');
ii = (-tolN < Pts.gammaN & Pts.gammaN < tolN ) & ...
     (-tolS < Pts.gammaS & Pts.gammaS < tolS ) & ...
     ( minS < Pts.alphaS & Pts.alphaS < maxS ) & ...
     ( minN < Pts.alphaN & Pts.alphaN < maxN ) & ...	   
     ( minS < Pts.betaS  & Pts.betaS  < maxS ) & ...
     ( minN < Pts.betaN  & Pts.betaN  < maxN );
ii = ii'; % column-vector
if sum(ii) == 0;
  error('No structure meets the specified criteria!');
  return;
else;
  nraw = length(ii);
  ngood = sum(ii>0);
  nbad = sum(ii==0);
  fprintf('Raw: %d  Good/Bad:  %d/%d   %.2f%%/%.2f%%\n', ...
	  nraw,ngood,nbad,ngood/nraw*100,nbad/nraw*100);
end;

clf;
fscatter3(Pts.Xw(1,ii),Pts.Xw(2,ii),Pts.Xw(3,ii),Pts.Xw(3,ii),jet);
axis equal;
grid on
xlabel('East [m]');
ylabel('North [m]');
