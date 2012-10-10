function [] = record_stats(field,dt,imageEvent,TheConfig);
%function [] = record_stats(field,dt,imageEvent,TheConfig);  
%
% History
% Date          Who        Comment
% ----------    ---        -----------------------------------
% 2004-11-08    rme        Created & written.

if ~TheConfig.Estimator.record_stats; return; end;

global stats_t TheJournal;

% store data
if ~isfield(stats_t,field);
  stats_t.(field) = initialize;
end;
cc = stats_t.(field).cc;
stats_t.(field).t(cc)          = TheJournal.t;
stats_t.(field).dt(cc)         = dt;
stats_t.(field).imageEvent(cc) = imageEvent;
stats_t.(field).Naug(cc)       = TheJournal.Index.Naug;
stats_t.(field).cc             = cc+1;

%************************************************************************
function X = initialize
X.t          = zeros(1,2.5e5);
X.dt         = X.t;
X.imageEvent = X.t;
X.Naug       = X.t;
X.cc         = 1;
