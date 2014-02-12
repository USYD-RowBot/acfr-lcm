% UNIX_AVI_EXAMPLE

% this script illustrates how to make a compressed AVI file under Linux.
% Note that mencoder is required - it is an optional component of mplayer

% rme 2003/10/23

%=====================================
% CREATE UNCOMPRESSED AVI USING MATLAB
%=====================================
fig=figure;
set(fig,'DoubleBuffer','on');
set(gca,'xlim',[-80 80],'ylim',[-80 80],...
	'NextPlot','replace','Visible','off')
mov = avifile(fullfile('/tmp','uncompressed.avi'));
x = -pi:.1:pi;
radius = [0:length(x)];
for i=1:length(x)
  h = patch(sin(x)*radius(i),cos(x)*radius(i),[abs(cos(x(i))) 0 0]);
  set(h,'EraseMode','xor');
  F = getframe(gca);
  mov = addframe(mov,F);
end
mov = close(mov);

%======================================
% USE MENCODER TO COMPRESS THE AVI
%======================================
% note that mencoder bombs when trying to read Matlab AVI's directly,
% that why the we cat it first.
%cmd = ['cat /tmp/uncompressed.avi | mencoder -ovc lavc -lavcopts ', ...
%	       'vcodec=msmpeg4v2 -o /tmp/compressed.avi -'];
%unix(cmd);
compressavi(mov,'/tmp/compressed.avi');

cmd = 'mplayer /tmp/compressed.avi';
unix(cmd);
cmd = 'ls -lh /tmp/*compressed.avi';
unix(cmd);
