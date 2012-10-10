function vmtranslate(ifile,ofile,wpRef,xy,ll)
%VMTRANSLATE Translates an Ocean-Server VectorMap mission file.
%
%  VMTRANSLATE(IFILE,OFILE,WPREF,XY,LL)
%    IFILE: input file name
%    OFILE: output file name
%    WPREF: reference waypoint number to recenter
%    XY: [deltaX, deltaY] or []  %meters
%    LL: [Lat, Lon] or []        %decimal minutes
%
%-----------------------------------------------------------------
%    History:
%    Date              Who        What
%    -----------       -------    --------------------------------
%    2008-08-18        RME        Created and written.

ifid = fopen(ifile,'r');
ofid = fopen(ofile,'w');
 
wpedit = false;
secondPass = false;

for p=1:2
    while 1
        tline = fgetl(ifid);
        if ~ischar(tline)
            break;
        end

        if strcmp(tline,'START')
            if secondPass
                fprintf(ofid,'START\n');
            end
            wpedit = true;
            continue;
        elseif strcmp(tline,'END')
            if secondPass
                fprintf(ofid,'END\n');
            end
            wpedit = false;
            continue;
        end

        if wpedit
            [wpNum,rem] = strtok(tline,';');
            [oldLat,rem] = strtok(rem,';');
            [oldLon,rem] = strtok(rem,';');

            wpNum = str2num(wpNum);
            oldLat = str2num(oldLat);
            oldLon = str2num(oldLon);

            if wpNum == wpRef
                orgLat = oldLat;
                orgLon = oldLon;
                if ~isempty(xy)
                    deltaX = xy(1);
                    deltaY = xy(2);
                elseif ~isempty(ll)
                    [deltaX,deltaY] = ll2xy(ll(1),ll(2),orgLat,orgLon);
                else
                    error('Hey dufus, specify a lat/lon or deltaXY!');
                end
            end

            if secondPass
                [oldX,oldY] = ll2xy(oldLat,oldLon,orgLat,orgLon);
                newX = oldX + deltaX;
                newY = oldY + deltaY;
                [newLat,newLon] = xy2ll(newX,newY,orgLat,orgLon);
                fprintf(ofid,'%d; %16.13f; %16.13f%s\n',wpNum,newLat,newLon,rem);
            end
        else
            if secondPass
                fprintf(ofid,'%s\n',tline);
            end
        end
    end
    secondPass = true;
    frewind(ifid);
end

fclose(ifid);
fclose(ofid);