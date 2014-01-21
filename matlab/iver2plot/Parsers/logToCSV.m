function logToCSV(infilename)
%LOGTOCSV - convert iver2 logfile into a comma-separated values file (.csv)
%  LOGTOCSV is used to make CSV files for use with sites like gpsvisualizer
%  and everytrail (both .com)
%
%  History:
%  2008/07/01  logToCSV created (Andrew Richardson)
%
    auv = iver2ParseToStruct(infilename);

    indices = strfind(infilename,'.');
    outfilename = [infilename(1:indices(end)) 'csv']
    out = fopen(outfilename,'w');
    if(out == -1)
        disp(['err opening ' outfilename])
        return
    end
    
    datasize = size(auv.elapsed,2);
    lat = zeros(datasize,1);
    lon = zeros(datasize,1);
    speed = zeros(datasize,1);
    
    fprintf(out,'name, latitude, longitude, speed\n');
    for k = 1:datasize
        fprintf(out,'%f, %f, %f, %f\n',auv.elapsed(k),auv.gps_Latitude(k),auv.gps_Longitude(k),auv.alt_DR_Speed(k));
    end
    fprintf('\n');
    
    if(fclose(out))
        disp([outfilename ' close failed']);
    end
    disp('done');
end

