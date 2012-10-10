function plot_featcov(fignum,ssa_t,link_t,ind,meas,vstime,inc)
%PLOT_FEATCOV plots feature covariance.
%   PLOT_FEATCOV(FIGNUM,SSA_T) plots the trace of each delayed state's
%   covariance as a function of the number of delayed states added to
%   the augmented state vector.  Each feature is shown with a
%   different color so you can track it across the plot.  Note that
%   transformation from local-level to world is taken care of
%   internally so that Y is North and X is East like nav data.
%
%   PLOT_FEATCOV(FIGNUM,SSA_T,LINK_T) additionally plots the number
%   of camera links for that pose.  Green links are temporal and
%   red links are spatial.  Use an empty matrix for LINK_T if you
%   do not want this additional plot when passing optional arguments.
%
%   PLOT_FEATCOV(FIGNUM,SSA_T,LINK_T,IND) calculates the trace for the
%   pose elements specified by IND.  For each feature, pose
%   elements are arranged [x,y,z,r,p,h].
%
%   PLOT_FEATCOV(FIGNUM,SSA_T,LINK_T,IND,MEAS) uses different measures
%   instead of the trace.
%   MEAS = 1 trace
%   MEAS = 2 determinant
%   MEAS = 3 (determinant)^1/dim
%
%   PLOT_FEATCOV(FIGNUM,SSA_T,LINK_T,IND,MEAS,VSTIME) if VSTIME is 1
%   then the plot is versus mission time and not image number.
%
%   PLOT_FEATCOV(FIGNUM,SSA_T,LINK_T,IND,MEAS,VSTIME,INC) if INC=2
%   then every other feature is plotted.  INC can be any positive
%   integer.
%
%   Ex:
%   %only shows the trace of the covariance for X, Y.
%   plot_featcov(1,ssa_t,[],[1,2]) 
%   %same as above but also shows camera links plot
%   plot_featcov(1,ssa_t,link_t,[1,2]) 
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    10-30-2003      rme         Created and written.
%    11-04-2003      rme         Added determinant measure and time
%                                axis option.
%    11-04-2003      rme         Added option of link subplot

if ~exist('ind','var') || isempty(ind)
  ind = 1:6;
end
if ~exist('meas','var')
  meas = 1;
end
if ~exist('vstime','var')
  vstime = 0;
end
if ~exist('link_t','var')
  link_t = [];
end
if ~exist('inc','var')
  inc = 1;
end

% local-level to world rotation matrix
Rwl = [0  1  0;
       1  0  0;
       0  0 -1];
T = [Rwl, zeros(3);
     zeros(3), eye(3)];

figure(fignum); clf;
Ns = length(ssa_t.TheJournal);
tr = cell(ssa_t.TheJournal{end}.index.Nf,1);
t = zeros(ssa_t.TheJournal{end}.index.Nf,1);
for ii=1:Ns % for each snapshot
  TheJournal = ssa_t.TheJournal{ii};  
  Nf = TheJournal.Index.Nf;
  t(ii) = TheJournal.t;
  JJ = 2:inc:Nf;
  if JJ(end)~=Nf; JJ(end+1)=Nf; end
  for jj=JJ % for each feature
    if TheJournal.Index.featureLUT(jj) < 0
      continue; % skip dvl delayed state
    end

    Xfi = TheJournal.Index.Xf{jj};  % feature index
    P = full(TheJournal.Ekf.Sigma(Xfi,Xfi)); % feature covariance
    % put in world frame
    P = T*P*T';
    % use user specified measure
    if meas == 1
      tr{jj} = [tr{jj};trace(P(ind,ind))];
    elseif meas == 2
      tr{jj} = [tr{jj};det(P(ind,ind))];
    elseif meas == 3
      tr{jj} = [tr{jj};det(P(ind,ind))^(1/length(ind))];
    end
  end
end

if ~isempty(link_t)
  subplot('position',[0.1 0.25 0.8 0.65]);
end
featureLUT = ssa_t.TheJournal{end}.index.featureLUT;
NextPlot = get(gca,'NextPlot');
set(gca,'NextPlot','add');
color = brightColorOrder;
color(6,:) = []; % remove yellow
cc = 1;
for ii=1:length(tr)
  if isempty(tr{ii}); continue; end
  if vstime
    plot(t(ii-1),tr{ii}(1),'.','Color',color(cc,:));
    plot(t([ii:end]-1),tr{ii},'-','Color',color(cc,:));
  else
    plot(featureLUT(ii),tr{ii}(1),'.','Color',color(cc,:));
    plot(featureLUT([0:length(tr{ii})-1]+ii),tr{ii},'-','Color',color(cc,:));
  end
    cc = cc + 1;
  if cc > size(color,1); cc = 1; end;
end
set(gca,'NextPlot',NextPlot);
set(gca,'xgrid','on');

xyzrph = 'xyzrph';
switch meas
 case 1; ylabel(sprintf(' trace(P) of sub-block %s',xyzrph(ind)));
 case 2; ylabel(sprintf(' det(P) of sub-block %s',xyzrph(ind)));
 case 3; ylabel(sprintf(' det(P)^{1/%d} of sub-block %s',length(ind),xyzrph(ind)));
 otherwise % do nothing
end % switch


if ~isempty(link_t)
  set(gca,'XTickLabel',[]);
  xlim = get(gca,'xlim');
  subplot('position',[0.1 0.1 0.8 0.1]);
  NextPlot = get(gca,'NextPlot');
  set(gca,'NextPlot','add');  
  [ii,jj] = find(link_t.vlinks==1);
  tlink = zeros(Nf,1);
  slink = zeros(Nf,1);
  for kk=1:length(jj)
    Fnum = jj(kk);
    Inum = featureLUT(Fnum);
    if Inum< 0; continue; end % skip DVL delayed state
    if jj(kk)-ii(kk)==1 % temporal
      tlink(Fnum) = 1;
    else % spatial
      slink(Fnum) = slink(Fnum)+1;
    end
  end

  if vstime
    plot(t,tlink(2:end),'g-');
    plot(t,slink(2:end),'r-');
  else
    %plot(featureLUT(2:end),tlink(2:end),'g-');
    %plot(featureLUT(2:end),slink(2:end),'r-');
    hdl = bar(featureLUT(2:end),[tlink(2:end),slink(2:end)],'stacked');
    set(hdl(1),'FaceColor','g');
    set(hdl(2),'FaceColor','r');
  end
  set(gca,'NextPlot',NextPlot);
  set(gca,'xgrid','on');
  set(gca,'Xlim',xlim);
end % if ~isempty(link_t)


ylabel(sprintf('Number\n of Links'));
if vstime
  xlabel('Mission Time [s]');
else
  xlabel('Image Number');
end
