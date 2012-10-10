function [dt,sensors] = checkForAsyncSensorMeas(t,nav_t,mindex_t,TheConfig)
%function [dt,sensors] = checkForAsyncSensorMeas(t,nav_t,mindex_t,TheConfig)  
%
% the purpose of this function is to try and calculate the dt to the next
% asynchronous sensor measurement.  the dt is used during the EKF prediction
% step to propogate our state vector forward.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    04-09-2004      rme         Created and written.
%    10-28-2004      rme         Renamed from calcdt.m to more descriptive name.
%    12-06-2004      rme         Cleaned up sensor_dt a bit.

TOL    = 0.005;
MAX_HZ = 10;
MAX_DT = 1/MAX_HZ;

% the variable 'sensors' is a cell array of
% fieldnames setup in the TheConfig.ObservationModel structure
sensors = fieldnames(mindex_t);
num_sensors = length(sensors);
sensor_dt = zeros(num_sensors,1);

% setup a "for loop" to find which sensor measurement
% occurs next
for k=1:num_sensors;
  sensor = sensors{k};
  mindex = mindex_t.(sensor);                 % measurement index into sensor structure
  max_index = length(nav_t.(sensor).rovtime); % maximum sensor index

  if (mindex <= max_index) || strcmp(sensor,'PXF');
    sensor_rovtime = nav_t.(sensor).rovtime(mindex); 
    sensor_dt(k) = sensor_rovtime - t;
  else;
    % this sensor is either turned off in the config file
    % or its measurement index is greater than the maximum
    % data index.  in either case skip it ...
    sensor_dt(k) = inf;
    continue;
  end;
end;

% find the minimum dt
dt = min(sensor_dt);
if dt > MAX_DT;
  dt = MAX_DT;
elseif dt < 0;
  fprintf('%s: Time warp!\n',mfilename);
  fprintf('sensor      dt\n');
  fprintf('----------  -----------\n');
  for k=1:length(sensor_dt)
    fprintf('%-10s  %+10.3f\n',sensors{k},sensor_dt(k))
  end
  vpause(30,'Time Warp!',1);
end;

% find all sensors which fall within TOL of dt
ii = find(abs(sensor_dt - dt) < TOL);
sensors = {sensors{ii}};
