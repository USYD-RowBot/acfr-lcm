function [mindex_t,t0] = initializeMeasurementIndex(nav_t,TheConfig,tau)
%function [mindex_t,t0] = initializeMeasurementIndex(nav_t,TheConfig,tau)  
%
%  Create a measurement index structure assocated with image number imgnum.
%  For tau < 0, initialize the nav sensors mindex to a time tau seconds prior
%  to the time associated with imgnum.  For tau > 0, initialize to a time
%  tau seconds after the time associated with imgnum.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    04-12-2004      rme         Created & written.
%    09-01-2004      rme         Modified to use TheConfig.ObservationModel field  
%    10-28-2004      rme         Major code reorganization.
%    05-01-2006      rme         Added temporary workaround for a empty imageSequence

if ~isempty(TheConfig.Data.imageSequence);
  % get the image number of the 1st image in the process sequence
  imgnum = TheConfig.Data.imageSequence(1);

  % setup measurement index structure associated with the 1st image in the sequence
  mindex_t.PXF = find(nav_t.PXF.imgnum == imgnum);
  to = nav_t.PXF.rovtime(mindex_t.PXF);
else;
  to = nav_t.RDI.rovtime(1);
end;

TOL= 5; % seconds
t0 = inf;
sensors = fieldnames(TheConfig.ObservationModel);
for k=1:length(sensors)
  sensor = sensors{k};
  if ~isempty(TheConfig.ObservationModel.(sensor)) && ~strcmp(sensor,'PXF')
    % sensor rovtime field
    sensor_rovtime = nav_t.(sensor).rovtime;
    % find index of sensor measurement which occurs tau seconds
    % prior to the first camera measurement
    if tau <= 0
      ii = find((sensor_rovtime <= to) & (sensor_rovtime > to + tau));
      ii = ii(end:-1:1);
    else
      ii = find((sensor_rovtime >= to) & (sensor_rovtime < to + tau));      
    end
    
    % set this sensor's measurement index to this point in time
    if ~isempty(ii);
      ii = ii(end);
      mindex_t.(sensor) = ii;
      % keep track of earliest sensor time
      if sensor_rovtime(ii) < t0
	t0 = sensor_rovtime(ii);
      end
    end
  end
end

