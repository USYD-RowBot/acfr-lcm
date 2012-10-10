function  [meas_t,mindex_t] = poll_nav_sensors(sensorEvents,nav_t,mindex_t,TheConfig)
%function  [meas_t,mindex_t] = poll_nav_sensors(sensorEvents,nav_t,mindex_t,TheConfig)  
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    04-09-2004      rme         Created and written.
%    09-09-2004      rme         Updated ii indexing.
%    09-09-2004      rme         Renamed poll_sensors.m to poll_navsensors.m
%    10-18-2004      rme         Renamed to poll_nav_sensors.m
%    10-28-2004      rme         Major code reorganization.
%    01-02-2004      rme         Added RDI zero velocity check.
%    01-07-2005      rme         Moved RDI zero velocity check to cook_nav.m

ii = 1;  
found = true;
for k=1:length(sensorEvents)
  sensor = sensorEvents{k};
  switch upper(sensor);
  case 'PARO';
   meas_t(ii) = add_paro_sensor_meas(nav_t,mindex_t,TheConfig);
   mindex_t.PARO = mindex_t.PARO + 1;
  case 'PHINS';
   meas_t(ii) = add_phins_sensor_meas(nav_t,mindex_t,TheConfig);
   mindex_t.PHINS = mindex_t.PHINS + 1;
  case 'XBOW';
   meas_t(ii) = add_xbow_sensor_meas(nav_t,mindex_t,TheConfig);
   mindex_t.XBOW = mindex_t.XBOW + 1;
  case 'RDI';
   meas_t(ii) = add_rdi_sensor_meas(nav_t,mindex_t,TheConfig);
   mindex_t.RDI = mindex_t.RDI + 1;
  otherwise; % didn't match any of the known nav sensor types
   found = false;
  end; % END: switch sensor

  % if found, increment array index, otherwise reset flag
  if found; ii = ii+1; else; found = true; end;
  
end; % END: for k=1:length(sensorEvents)

% if no measurements occurred, return an empty structure
if ~exist('meas_t','var');
  meas_t = [];
end;
