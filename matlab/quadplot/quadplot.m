function quadplot(dataTypes, data)
% USAGE:
%--------------------------------------------------%
% dataTypes : list of ints.  Should be taken from the .mat
%             file from quad_log_to_mat
% data      : struct of data to which the .mat file was imported
%
% EXAMPLE:
%--------------------------------------------------%
% (from bash):
% $ quad_log_to_mat.py input.csv output.mat
%
% <from matlab>
% >>> data = load('output.mat');
% >>> quadplot([data.AnglePitch data.AngleRoll data.AngleYaw], data);

SAMPLE_RATE = 0.2;

getVarName=@(x) inputname(1);

time = (0:size(data.data,1) - 1)*SAMPLE_RATE / 60;
  
figure;plot(time, data.data(:,dataTypes));
xlabel('Minutes');
legend(data.colheaders(dataTypes))
