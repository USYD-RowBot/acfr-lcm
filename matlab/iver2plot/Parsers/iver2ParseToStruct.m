function restrans = iver2ParseToStruct(filename, uvc_ver)
%IVER2PARSETOSTRUCT read
% get enums from class
disp(sprintf('Using UVC version %g',uvc_ver))
if(uvc_ver == 2.7)
    enums = auv_enums();
elseif(uvc_ver == 3.0 || uvc_ver == 3.1)
    enums = auv_enums3();
elseif(uvc_ver == 3.31)
    enums = auv_enums331();
elseif(uvc_ver == 4.2)
    enums = auv_enums42();
elseif(uvc_ver == 4.5)
    enums = auv_enums45();
elseif(uvc_ver == 4.7)
    enums = auv_enums47();
end

% open file
FILE = fopen(filename,'r');
if(FILE==0)
    errordlg('***iver2ParseToStruct:: error opening FILE', 'Error');
else
    % skip the first line
    line = fgetl(FILE);
    line = fgetl(FILE);
    i = 1;
    while(1)
        % get the semicolon indices
        semis = strfind(line,';');
        last = 0;
        j = 1;
        % for each index, cast the text to a cell array
        for semi = semis
            cellline{j} = line(last+1:semi-1);
            last = semi;
            j = j + 1;
        end

        % pick each field from the cell array based on the enums struct
        Time{i} = cellline{enums.Time};
        temp = sscanf(Time{i},'%d:%d:%d %s');
        time.hour(i) = temp(1);
        time.minute(i) = temp(2);
        time.second(i) = temp(3);
        if(temp(4) == 80) % PM: convert to 12-23
            if(time.hour(i) ~= 12) % do not add to 12PM
                time.hour(i) = time.hour(i) + 12;
            end
        else % AM
            if(time.hour(i) == 12) % convert 12am to 0
                time.hour(i) = 0;
            end
        end

        Date{i} = cellline{enums.Date};
        temp = sscanf(Date{i},'%d/%d/%d');
        date.month(i) = temp(1);
        date.day(i) = temp(2);
        date.year(i) = temp(3);

        res.iver2.unixtime(i) = unixtime( date.year(i), date.month(i),...
            date.day(i), time.hour(i),...
            time.minute(i), time.second(i));

        %             res.iver2.Leak(i) = sscanf(cellline{enums.Leak},'%g'); 2.7~3.1
        %             res.iver2.State{i} = cellline{enums.State};
        if(uvc_ver==3.31 || uvc_ver==4.2 || uvc_ver==4.5 || uvc_ver==4.7)
            if (findstr(cellline{enums.ErrorState},'N'))
                res.iver2.ErrorState(i,:) = 'NON';% No errors
            elseif (findstr(cellline{enums.ErrorState},'E'))
                res.iver2.ErrorState(i,:) = 'BTT';% Battery below 10%
            elseif (findstr(cellline{enums.ErrorState},'Sop'))
                res.iver2.ErrorState(i,:) = 'SOP';% Over pitch
            elseif (findstr(cellline{enums.ErrorState},'Stl'))
                res.iver2.ErrorState(i,:) = 'STL';% Exceed time limit
            elseif (findstr(cellline{enums.ErrorState},'Sle'))
                res.iver2.ErrorState(i,:) = 'SLE';% Leak
            elseif (findstr(cellline{enums.ErrorState},'Sfp'))
                res.iver2.ErrorState(i,:) = 'SFP';% No forward progress
            elseif (findstr(cellline{enums.ErrorState},'Sed'))
                res.iver2.ErrorState(i,:) = 'SED';% Exceed max depth
            elseif (findstr(cellline{enums.ErrorState},'Sup'))
                res.iver2.ErrorState(i,:) = 'SUP';% No upward progress
            elseif (findstr(cellline{enums.ErrorState},'Stf'))
                res.iver2.ErrorState(i,:) = 'STF';% Safety tow float engaged
            elseif (findstr(cellline{enums.ErrorState},'Srp'))
                res.iver2.ErrorState(i,:) = 'SRP';% Safety return path engaged
            elseif (findstr(cellline{enums.ErrorState},'Snd'))
                res.iver2.ErrorState(i,:) = 'SND';% DFS has not changed
            elseif (findstr(cellline{enums.ErrorState},'Snc'))
                res.iver2.ErrorState(i,:) = 'SNC';% Compass has stopped working
            end
        end

        res.iver2.gps.Latitude(i) = sscanf(cellline{enums.gps_Latitude},'%g');
        res.iver2.gps.Longitude(i) = sscanf(cellline{enums.gps_Longitude},'%g');
        res.iver2.gps.Num_Sats(i) = sscanf(cellline{enums.gps_Num_Sats},'%g');
        res.iver2.gps.Speed_Knots(i) = sscanf(cellline{enums.gps_Speed_Knots},'%g');
        res.iver2.gps.True_Heading(i) = sscanf(cellline{enums.gps_True_Heading},'%g');
        res.iver2.gps.Magnetic_Variation(i) = sscanf(cellline{enums.gps_Magnetic_Variation},'%g');

        res.iver2.comp.Magnetic_Heading(i) = sscanf(cellline{enums.comp_Magnetic_Heading},'%g');
        res.iver2.comp.Pitch_Angle(i) = sscanf(cellline{enums.comp_Pitch_Angle},'%g');
        res.iver2.comp.Roll_Angle(i) = sscanf(cellline{enums.comp_Roll_Angle},'%g');
        res.iver2.comp.True_Heading(i) = sscanf(cellline{enums.comp_True_Heading},'%g');
        res.iver2.comp.Inside_Temp(i) = sscanf(cellline{enums.comp_Inside_Temp},'%g');
        res.iver2.comp.DFS_Depth(i) = sscanf(cellline{enums.comp_DFS_Depth},'%g');
        if(uvc_ver == 2.7); res.iver2.comp.DFS2_Depth(i) = sscanf(cellline{enums.comp_DFS2_Depth},'%g'); end

        if(uvc_ver == 3.0 || uvc_ver == 3.1) res.iver2.alt.Sounder_Speed(i) = sscanf(cellline{enums.alt_Sounder_Speed},'%g'); end
        if(uvc_ver == 3.0 || uvc_ver == 3.1) res.iver2.alt.DR_Speed(i) = sscanf(cellline{enums.alt_DR_Speed},'%g'); end
        res.iver2.alt.DTB_Height(i) = sscanf(cellline{enums.alt_DTB_Height},'%g');

        res.iver2.pow.Batt_Percent(i) = sscanf(cellline{enums.pow_Batt_Percent},'%g');
        res.iver2.pow.Power_Watts(i) = sscanf(cellline{enums.pow_Power_Watts},'%g');
        res.iver2.pow.Watt_Hours(i) = sscanf(cellline{enums.pow_Watt_Hours},'%g');
        res.iver2.pow.Batt_Volts(i) = sscanf(cellline{enums.pow_Batt_Volts},'%g');
        res.iver2.pow.Batt_Ampers(i) = sscanf(cellline{enums.pow_Batt_Ampers},'%g');
        res.iver2.pow.Time_to_Empty(i) = sscanf(cellline{enums.pow_Time_to_Empty},'%g');

        if(uvc_ver == 3.0 || uvc_ver == 3.1) res.iver2.ctrl.Servo_State(i) = sscanf(cellline{enums.ctrl_Servo_State},'%g'); end
        %           res.iver2.ctrl.Cur_True_Heading(i) = sscanf(cellline{enums.ctrl_Cur_True_Heading},'%g');
        %           res.iver2.ctrl.Roll_Angle(i) = sscanf(cellline{enums.ctrl_Roll_Angle},'%g');
        %           res.iver2.ctrl.Pitch_Angle(i) = sscanf(cellline{enums.ctrl_Pitch_Angle},'%g');
        res.iver2.ctrl.Pitch_R(i) = sscanf(cellline{enums.ctrl_Pitch_R},'%g');
        res.iver2.ctrl.Yaw_T(i) = sscanf(cellline{enums.ctrl_Yaw_T},'%g');
        res.iver2.ctrl.Yaw_B(i) = sscanf(cellline{enums.ctrl_Yaw_B},'%g');
        if(uvc_ver == 3.0 || uvc_ver == 3.1) res.iver2.ctrl.Yaw_K_Error(i) = sscanf(cellline{enums.ctrl_Yaw_K_Error},'%g'); end
        res.iver2.ctrl.Pitch_L(i) = sscanf(cellline{enums.ctrl_Pitch_L},'%g');
        if(uvc_ver == 2.7); res.iver2.ctrl.Depth_iTerm(i) = sscanf(cellline{enums.ctrl_Depth_iTerm},'%g'); end
        if(uvc_ver == 2.7); res.iver2.ctrl.Depth_pTerm(i) = sscanf(cellline{enums.ctrl_Depth_pTerm},'%g'); end
        if(uvc_ver == 2.7); res.iver2.ctrl.Depth_iSTATE(i) = sscanf(cellline{enums.ctrl_Depth_iSTATE},'%g'); end
        if(uvc_ver == 2.7); res.iver2.ctrl.Pitch_dTerm(i) = sscanf(cellline{enums.ctrl_Pitch_dTerm},'%g'); end
        if(uvc_ver == 2.7); res.iver2.ctrl.Pitch_err(i) = sscanf(cellline{enums.ctrl_Pitch_err},'%g'); end
        if(uvc_ver == 2.7); res.iver2.ctrl.Pitch_Calc(i) = sscanf(cellline{enums.ctrl_Pitch_Calc},'%g'); end

        res.iver2.ref.Dist_To_Next(i) = sscanf(cellline{enums.ref_Dist_To_Next},'%g');
        res.iver2.ref.Next_Speed(i) = sscanf(cellline{enums.ref_Next_Speed},'%g');
        res.iver2.ref.Next_Heading(i) = sscanf(cellline{enums.ref_Next_Heading},'%g');
        res.iver2.ref.Next_Long(i) = sscanf(cellline{enums.ref_Next_Long},'%g');
        res.iver2.ref.Next_Lat(i) = sscanf(cellline{enums.ref_Next_Lat},'%g');
        res.iver2.ref.Next_Depth(i) = sscanf(cellline{enums.ref_Next_Depth},'%g');
        res.iver2.ref.Current_Step(i) = sscanf(cellline{enums.ref_Current_Step},'%g');
        res.iver2.ref.Depth_K_Goal(i) = sscanf(cellline{enums.ref_Depth_K_Goal},'%g');
        res.iver2.ref.Pitch_K_Goal(i) = sscanf(cellline{enums.ref_Pitch_K_Goal},'%g');
        res.iver2.ref.Speed_CMD(i) = sscanf(cellline{enums.ref_Speed_CMD},'%g');
        if(uvc_ver == 2.7); res.iver2.ref.Dgoal(i) = sscanf(cellline{enums.ref_Dgoal},'%g')*FTOM; end
        if(uvc_ver == 2.7); res.iver2.ref.Pitch_goal(i) = sscanf(cellline{enums.ref_Pitch_goal},'%g'); end


        % not used with our hardware
        %         res.iver2.unkn.Temp_C(i) = sscanf(cellline{enums.unkn_Temp_C},'%g');
        %         res.iver2.unkn.SpCond_uSpcm(i) = sscanf(cellline{enums.unkn_SpCond_uSpcm},'%g');
        %         res.iver2.unkn.Sal_ppt(i) = sscanf(cellline{enums.unkn_Sal_ppt},'%g');
        %         res.iver2.unkn.Depth_meters(i) = sscanf(cellline{enums.unkn_Depth_meters},'%g');
        %         res.iver2.unkn.ODOsat_percent(i) = sscanf(cellline{enums.unkn_ODOsat_percent},'%g');
        %         res.iver2.unkn.ODO_mgpL(i) = sscanf(cellline{enums.unkn_ODO_mgpL},'%g');
        %         res.iver2.unkn.Chl_ugpL(i) = sscanf(cellline{enums.unkn_Chl_ugpL},'%g');
        %         res.iver2.unkn.Chl_RFU(i) = sscanf(cellline{enums.unkn_Chl_RFU},'%g');
        %         res.iver2.unkn.Battery_volts(i) = sscanf(cellline{enums.unkn_Battery_volts},'%g');

        % rdi speed
        res.iver2.rdi_Cur_Speed(i) = sscanf(cellline{enums.rdi_Cur_Speed},'%g');

        % get the next line, break if necessary
        line = fgetl(FILE);
        if ~ischar(line), break; end
        i = i + 1;
    end

    clear Time Date time date temp;

    % get time terms
    res.iver2.elaptime = res.iver2.unixtime - res.iver2.unixtime(1);
    res.iver2.STARTTIME = res.iver2.unixtime(1);
    res.iver2.ENDTIME = res.iver2.unixtime(end);


    % intuitive depth, negative z points down
    if(uvc_ver == 2.7); res.iver2.ref.Dgoal = -res.iver2.ref.Dgoal; end

    % get x,y,z
    orglat = res.iver2.ref.Next_Lat(1);
    orglon = res.iver2.ref.Next_Long(1);
    [res.iver2.x, res.iver2.y] = ll2xy(res.iver2.gps.Latitude,res.iver2.gps.Longitude,orglat, orglon);
    if (uvc_ver == 4.7) 
        res.iver2.z = -res.iver2.comp.DFS_Depth;
    else
        res.iver2.z = -res.iver2.comp.DFS_Depth*FTOM;
    end

    % depth-to-bottom height in meters
    if (uvc_ver == 4.7)
        res.iver2.alt.DTB_Height = res.iver2.alt.DTB_Height;
    else
        res.iver2.alt.DTB_Height = res.iver2.alt.DTB_Height*FTOM;
    end

    % goal depth in meters
    if (uvc_ver == 4.7)
        res.iver2.ref.Next_Depth = -res.iver2.ref.Next_Depth;
        res.iver2.ref.Depth_K_Goal = -res.iver2.ref.Depth_K_Goal;  
    else
        res.iver2.ref.Next_Depth = -res.iver2.ref.Next_Depth*FTOM;
        res.iver2.ref.Depth_K_Goal = -res.iver2.ref.Depth_K_Goal*FTOM;
    end

    % set next_lats and next_longs which are equal to zero equal to NaN, instead (stops plotting of them)
    res.iver2.ref.Next_Lat( find(res.iver2.ref.Next_Lat  == 0)) = NaN;
    res.iver2.ref.Next_Long(find(res.iver2.ref.Next_Long == 0)) = NaN;

    % seafloor
    res.iver2.seafloor = res.iver2.z - res.iver2.alt.DTB_Height;
    %	res.iver2.seafloor = -res.iver2.alt.DTB_Height;

    % get x,y goals
    [res.iver2.xref, res.iver2.yref] = ll2xy(res.iver2.ref.Next_Lat,res.iver2.ref.Next_Long,orglat,orglon);

    % get valid gps region (bool)
    threshold = 4; % min satellites for gps fix
    res.iver2.validgps = (res.iver2.gps.Num_Sats < threshold);


    clear threshold

    % close the file and return
    if(fclose(FILE))
        errordlg('***iver2ParseToStruct:: error closing FILE','Error');
    end
    restrans.iver2.unixtime=res.iver2.unixtime';
    restrans.iver2.elaptime=res.iver2.elaptime';
    restrans.iver2.ErrorState=res.iver2.ErrorState';
    restrans.iver2.gps.Latitude=res.iver2.gps.Latitude';
    restrans.iver2.gps.Longitude=res.iver2.gps.Longitude';
    restrans.iver2.gps.Num_Sats=res.iver2.gps.Num_Sats';
    restrans.iver2.gps.Speed_Knots=res.iver2.gps.Speed_Knots';
    restrans.iver2.gps.True_Heading=res.iver2.gps.True_Heading';
    restrans.iver2.gps.Magnetic_Variation=res.iver2.gps.Magnetic_Variation';
    restrans.iver2.comp.Magnetic_Heading=res.iver2.comp.Magnetic_Heading';
    restrans.iver2.comp.Pitch_Angle=res.iver2.comp.Pitch_Angle';
    restrans.iver2.comp.Roll_Angle=res.iver2.comp.Roll_Angle';
    restrans.iver2.comp.True_Heading=res.iver2.comp.True_Heading';
    restrans.iver2.comp.Inside_Temp=res.iver2.comp.Inside_Temp';
    restrans.iver2.alt.DTB_Height=res.iver2.alt.DTB_Height';
    restrans.iver2.pow.Batt_Percent=res.iver2.pow.Batt_Percent';
    restrans.iver2.pow.Power_Watts=res.iver2.pow.Power_Watts';
    restrans.iver2.pow.Watt_Hours=res.iver2.pow.Watt_Hours';
    restrans.iver2.pow.Batt_Volts=res.iver2.pow.Batt_Volts';
    restrans.iver2.pow.Batt_Ampers=res.iver2.pow.Batt_Ampers';
    restrans.iver2.pow.Time_to_Empty=res.iver2.pow.Time_to_Empty';
    restrans.iver2.ctrl.Pitch_R=res.iver2.ctrl.Pitch_R';
    restrans.iver2.ctrl.Yaw_T=res.iver2.ctrl.Yaw_T';
    restrans.iver2.ctrl.Yaw_B=res.iver2.ctrl.Yaw_B';
    restrans.iver2.ctrl.Pitch_L=res.iver2.ctrl.Pitch_L';
    restrans.iver2.ref.Dist_To_Next=res.iver2.ref.Dist_To_Next';
    restrans.iver2.ref.Next_Speed=res.iver2.ref.Next_Speed';
    restrans.iver2.ref.Next_Heading=res.iver2.ref.Next_Heading';
    restrans.iver2.ref.Next_Long=res.iver2.ref.Next_Long';
    restrans.iver2.ref.Next_Lat=res.iver2.ref.Next_Lat';
    restrans.iver2.ref.Next_Depth=res.iver2.ref.Next_Depth';
    restrans.iver2.ref.Current_Step=res.iver2.ref.Current_Step';
    restrans.iver2.ref.Depth_K_Goal=res.iver2.ref.Depth_K_Goal';
    restrans.iver2.ref.Pitch_K_Goal=res.iver2.ref.Pitch_K_Goal';
    restrans.iver2.ref.Speed_CMD=res.iver2.ref.Speed_CMD';
    restrans.iver2.rdi_Cur_Speed=res.iver2.rdi_Cur_Speed';
    restrans.iver2.x=res.iver2.x';
    restrans.iver2.y=res.iver2.y';
    restrans.iver2.z=res.iver2.z';
    restrans.iver2.seafloor=res.iver2.seafloor';
    restrans.iver2.xref=res.iver2.xref';
    restrans.iver2.yref=res.iver2.yref';
    restrans.iver2.validgps=res.iver2.validgps';
    restrans.iver2.STARTTIME=res.iver2.STARTTIME';
    restrans.iver2.ENDTIME=res.iver2.ENDTIME';
    restrans.iver2.ORGLAT = orglat;
    restrans.iver2.ORGLON = orglon;
end
