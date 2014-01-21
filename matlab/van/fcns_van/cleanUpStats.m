function cleanUpStats;
%function cleanUpStats;
%
% History
% Date          Who        Comment
% ----------    ---        -----------------------------------
% 2004-11-10    rme        Created & written.

global stats_t;

fnames = fieldnames(stats_t);
for ii=1:length(fnames);
  field = fnames{ii};
  cc = stats_t.(field).cc;
  % remove unused elements
  stats_t.(field).t(cc:end)          = [];
  stats_t.(field).dt(cc:end)         = [];
  stats_t.(field).imageEvent(cc:end) = [];
  stats_t.(field).Naug(cc:end)       = [];
end;
