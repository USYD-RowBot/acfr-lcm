function imaleak_status()


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

% get length of iver_t.ErrorState
lengthOfError = length(iver_t.ErrorState);

% build image
Color=zeros(lengthOfError);
for i = 1:lengthOfError
    if (strcmp(iver_t.ErrorState(i),'N'))
        Color(i) = 1;% Cyan for No errors
    elseif (strcmp(iver_t.ErrorState(i),'E'))
        Color(i) = 2;% Grey for Battery below 10%
    elseif (strcmp(iver_t.ErrorState(i),'SOP'))
        Color(i)= 3;% Blue for Over pitch
    elseif (strcmp(iver_t.ErrorState(i),'STL'))
        Color(i) = 4;% Green for Exceed time limit
    elseif (strcmp(iver_t.ErrorState(i),'SLE'))
        Color(i) = 5;% Red for Leak
    elseif (strcmp(iver_t.ErrorState(i),'SFP'))
        Color(i) = 6;% Yellow for No forward progress
    elseif (strcmp(iver_t.ErrorState(i),'SED'))
        Color(i) = 7;% Purple for Exceed max depth
    elseif (strcmp(iver_t.ErrorState(i),'SUP'))
        Color(i) = 8;% Burgundy for No upward progress
    elseif (strcmp(iver_t.ErrorState(i),'STF'))
        Color(i) = 9;% Aqua for Safety tow float engaged
    elseif (strcmp(iver_t.ErrorState(i),'SRP'))
        Color(i) = 10;% Pink for Safety return path engaged
    elseif (strcmp(iver_t.ErrorState(i),'SND'))
        Color(i) = 11;% Orange for DFS has not changed
    elseif (strcmp(iver_t.ErrorState(i),'SNC'))
        Color(i) = 12;% Magenta for Compass has stopped working
    end
end

C = zeros(length(yextents),lengthOfError);
for j = 1:length(yextents)
    C(j,:) = Color(1,:);
end

% build transparancy map
alphadata = ones(size(C,1),size(C,2))*40;

% build colormap
map=[   .5 1 .85; .5 .5 .5; 0 0 1; 0 1 0;...
        1 0 0; 1 1  0; .5 .2 .8; .6 .1 .1;...
        .25 .8 .5; 1 .7 .75; 1 .6 0; 1 0 1 ];
colormap(map);

% plot
hold on
    
    image(iver_t.elaptime,yextents,C,'AlphaData',alphadata,'AlphaDataMapping','direct');
    text(xextents(1)+0.05*xrange, yextents(1)+0.3*yrange, ...
        sprintf(['Cyan: No errors\nGrey: Low battery\nBlue:Over pitch\n'...
        'Green: Exceed time limit\nRed:  Leak\nYellow: No fwd progress\n'...
        'Purple: Exceed max depth\nBrown: No upward progress\nAqua: Safety tow float engaged\n'...
        'Pink: Safety return path engaged\nOrange: DFS has not changed\n'...
        'Magenta: Compass has stopped working end']));
hold off


