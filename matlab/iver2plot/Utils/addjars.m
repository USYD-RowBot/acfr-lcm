% adding jar files 
% (1) to use lcmlog: lcm.jar
% (2) to find definition for lcmtypes: perlcm.jar
rootdir = which('iver2plot.m');
filenamesize = size('iver2plot.m',2);
rootdir = rootdir(1:end-filenamesize);
eval(['javaaddpath ',rootdir,'../../third-party/lcm/lcm-java/lcm.jar']);
%javaaddpath ../../lib/perlcm.jar
eval(['javaaddpath ',rootdir,'../../lib/senlcm.jar']);
clear rootdir;
clear filenamesize;