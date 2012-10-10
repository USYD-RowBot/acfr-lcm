function nav_t = lcmlog_parser2(filename)
%function output = lcmlog_parser2(filename, channels, msgtypes)
% modified read_from_log.m
% out is 'nav_t.iverlcm' structure
% lcm uses microsec in unixtime (us from 1970 jan 1)

fprintf('%s: %s\n', mfilename, filename);

try 
    log = lcm.logging.Log(filename,'r');
catch
    addjars;
    log = lcm.logging.Log(filename,'r');
end

% event: see /lcm/lcm-java/lcm/logging/Log.java
% public long   utime;
% public long   eventNumber
% public byte   data[];
% public String channel;

tic;
while (true)
    try
        event = log.readNext();
    catch
        break;
    end
    
    channel = char(event.channel);
    bytestream = java.io.ByteArrayInputStream(event.data, 0, numel(event.data));
    datastream = java.io.DataInputStream(bytestream);
    
    data = [];
    switch (channel)
        case 'OS_COMPASS'
            data = struct(senlcm.os_compass_t(datastream));
            type = 'os_compass_t';
        case 'KVH'
            data = struct(senlcm.kvh_dsp3000_t(datastream));
            data = rmfield(data,{'RATE_MODE','DELTA_MODE','ANGLE_MODE'});
            type = 'kvh_dsp3000_t';
        case 'DESERT_STAR'
            data = struct(senlcm.dstar_ssp1_t(datastream));
            type = 'dstar_ssp1_t';
        otherwise
            % nothing
    end
    
    if ~isempty(data)
        data = rmfield(data,{'LCM_FINGERPRINT','LCM_FINGERPRINT_BASE'});
        try
            index.(type) = index.(type) + 1;
            nav_t.(type)(index.(type)) = data;
        catch
            index.(type) = 1;
            nav_t.(type)(10e6,1) = data;
            nav_t.(type)(1) = data;
        end
    end
    
end
toc
log.close();
disp ('END');

fields = fieldnames(nav_t);
for i=1:length(fields)
   field = fields{i};
   nav_t.(field) = nav_t.(field)(1:index.(field));
end

