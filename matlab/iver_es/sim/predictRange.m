function predictRange(counter, rise, run)
global iver state
pitch = state.pitch(counter-1);
alt = state.alt(counter);
slopeDvl = state.slopeDvl(counter);
beamAngle = pitch - 3;
bottomAngle = mean(atan2(rise, -run))*180/pi;
if bottomAngle < beamAngle
    state.predRange(counter) = 0;
else
    phi = -(beamAngle + bottomAngle);
    if phi < 0;
        state.predRange(counter) = 0;
        return
    end
    esX = iver.dvl2es*cosd(pitch) - iver.radius*sind(pitch);
    esY = iver.dvl2es*sind(pitch) + iver.radius*cosd(pitch);
    altES = alt + esY + slopeDvl*esX;
    state.predRange(counter) = (altES*sind(90-bottomAngle))/sind(phi);
    if state.predRange(counter) < 0.30
        state.predRange(counter) = 0.30;
    elseif state.predRange(counter) > 50
        state.predRange(counter) = 0;
    end
end
end