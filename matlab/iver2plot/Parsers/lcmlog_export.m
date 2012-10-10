function nav_t=lcmlog_export(filename)

%get lcmlog file path
ctlhdl = get(2,'UserData');
directory = get(ctlhdl.dir_edittext,'UserData');
filepath = sprintf('%s/%s',directory,filename);

%get root directory
rootdir = which('iver2plot.m');
filenamesize = size('iver2plot.m',2);
rootdir = rootdir(1:end-filenamesize);

%generate m-file for loading data
strip = '--strip "IVER28_.|IVER28_|IVER31_.|IVER31_|TOPSIDE_" ';
cmd = [rootdir,'../../build/bin/perls-core-lcmlog-export -m ',strip, ' --delim_replace "." ' , filepath,' -d ',tempdir,'lcmlog_export'];

fprintf ('sending command: %s\n', cmd);
[status, result] = system (['unset LD_LIBRARY_PATH; ', cmd])
for ii=1:length (filename)
    if (~isstrprop (filename(ii),'alphanum'))
                filename(ii) = '_';
    end
end
addpath(genpath([tempdir,'lcmlog_export']));
nav_t=eval(filename);

try
    nav_t.PROSILICA_M_ATTR = nav_t.PROSILICA_M_ATTRIBUTES;
catch
    fprintf ('failed M_ATTR\n');
end
try
    nav_t.PROSILICA_C_ATTR = nav_t.PROSILICA_C_ATTRIBUTES;
catch
    fprintf ('failed C_ATTR\n');
end

try
    nav_t.PROSILICA_M = nav_t.PROSILICA_M_SYNC;
catch
    fprintf ('failed M_SYNC\n');
end
try
    nav_t.PROSILICA_C = nav_t.PROSILICA_C_SYNC;
catch
    fprintf ('failed C_SYNC\n');
end

if isfield(nav_t, 'PROSILICA_M')
   nav_t.PROSILICA_M.frame = [0:length(nav_t.PROSILICA_M.utime)-1]';
end
if isfield(nav_t, 'PROSILICA_C')
   nav_t.PROSILICA_C.frame = [0:length(nav_t.PROSILICA_C.utime)-1]';
end
