function plot_stats(stats_t);
%function plot_stats(stats_t);  
%
% HISTORY      WHO     WHAT
%----------    ----    -------------------------------------
% 11-08-2004   rme     Created and written.
% 11-27-2004   rme     Added passing of plot handles to legend()

persistent cc firsttime;

fignum = gcf;

if isempty(firsttime);
  firsttime = false;
  figure(fignum); clf;
  grid on;
  cc = ones(10,1);
end;

color = ['b';'g';'r';'c';'m';'k'];
fields = fieldnames(stats_t);
n = length(fields);
figure(fignum);
hold on;
for ii=1:n;
  X = stats_t.(fields{ii});
  h(ii) = plot(X.t(cc(ii):X.cc-1), X.dt(cc(ii):X.cc-1),color(ii));
  legendString{ii} = fields{ii};
end;
hold off;
legend(h,legendString{:});
xlabel('Mission Time [s]');
ylabel('Process Time [s]');
