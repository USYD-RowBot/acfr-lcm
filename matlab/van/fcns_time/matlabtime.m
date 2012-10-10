function matlab_time = matlabtime(utime)
%MATLABTIME Converts unix time to matlab DATENUM format.

matlab_time = utime/(24*3600) + 719529; 