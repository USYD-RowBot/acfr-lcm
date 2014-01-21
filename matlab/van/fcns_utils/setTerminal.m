function setTerminal(mode);
%function setTerminal(mode);
%
%   mode can be: normal, dim, emphasis, warning, error, highlight, restore
%
% History
% DATE          WHO              WHAT
%----------    --------------    ------------------------------
% 2004-11-05   Ryan Eustice      Created & written.
% 2004-12-11   rme               Added 'restore' capability.
% 2004-13-11   rme               Fixed bug in 'restore' option.

if ~exist('mode','var');
  mode = 'restore';
end;

persistent termopts restoreopts;
switch lower(mode);
case 'emphasis';
 restoreopts = termopts;
 termopts = {'bright','white','black'};
case 'dim';
 restoreopts = termopts;
 termopts = {'dim','white','black'};
case 'normal';
 restoreopts = termopts;
 termopts = {'reset','white','black'};
case 'warning';
 restoreopts = termopts;
 termopts = {'reset','yellow','black'};
case 'error';
 restoreopts = termopts;
 termopts = {'bright','red','black'};
case 'highlight';
 restoreopts = termopts;
 termopts = {'reset','cyan','black'};
otherwise; % restore
 termopts = restoreopts;
 textcolor(restoreopts{:});
 return;
end;
textcolor(termopts{:});
