function imacurrent_step()


% uvclog
try
    iver_t = evalin('base','iver_t');
    IVER=1;
catch
    IVER=0;
    errordlg('This plot requires UVC log data','Error');
    return;
end

% timebase hack 
iver_t.elaptime(1) = 0;
iver_t.elaptime(end) = iver_t.ENDTIME-iver_t.STARTTIME;

% get plot axes
xextents = get(gca,'XLim');
yextents = get(gca,'YLim');
xrange = xextents(2) - xextents(1);
yrange = yextents(2) - yextents(1);

% build image
step_max = max(iver_t.ref.Current_Step);
imagemax = 64;
colorstep = imagemax*1.0/step_max;

Cx = iver_t.ref.Current_Step*colorstep;
for i = 1:length(yextents)
    C(i,:) = Cx';
end

% build transparancy map
alphadata = ones(size(C,1),size(C,2))*40;

% plot
hold on
    image(iver_t.elaptime,yextents,C,'AlphaData',alphadata,'AlphaDataMapping','direct');
hold off

