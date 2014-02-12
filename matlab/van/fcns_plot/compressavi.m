function compressavi(mov,ofile,varargin)
%COMPRESSAVI  compresses a Matlab generated AVI using mencoder under Linux.
%   COMPRESSAVI(MOV) MOV can be either a Matlab AVI object as returned by
%   AVIFILE or the filename of the matlab AVI to compress.  By default, the
%   compressed AVI will overwrite the uncompressed AVI.
%
%   COMPRESSAVI(MOV,OFILE) writes the compressed AVI to filename OFILE
%   instead of overwriting the uncompressed AVI given by MOV.
%
%   COMPRESSAVI(MOV,OFILE,OPT1,OPT2) allows the user to directly pass
%   options to mencoder rather than use the default settings.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    10-24-2003      rme         Created and written.

%======================================
% USE MENCODER TO COMPRESS THE AVI
%======================================
% note that mencoder bombs when trying to read Matlab AVI's directly,
% that's why the we cat it first.
if isobject(mov)
  % matlab avi object
  ifile = mov.Filename;
else
  % user supplied avi filename
  ifile = mov;
end
DELETE_SOURCE = 0;
if ~exist('ofile','var') || strcmp(ifile,ofile)
  % user did not specify an ouput file name, overwrite uncompressed avi
  % with compressed avi
  ofile = ifile;
  [pathstr,name,ext,versn] = fileparts(ifile);
  ifile = fullfile(pathstr,'tmp123456.avi');
  cmd = sprintf('/bin/mv %s %s',ofile,ifile);
  unix(cmd);
  DELETE_SOURCE = 1;
end
if nargin <= 2
  % use default settings
  cmd = sprintf(['!/bin/cat %s | mencoder -ovc lavc -lavcopts ', ...
		 'vcodec=msmpeg4v2 -o %s -'],ifile,ofile);
else
  % use user supplied mencoder options
  opts = sprintf('%s ',varagin{:});
  cmd = sprintf('!/bin/cat %s | mencoder %s -o %s',ifile,opts,ofile);
end
eval(cmd);

if DELETE_SOURCE
  cmd = sprintf('/bin/rm -f %s',ifile);
  unix(cmd);
end
