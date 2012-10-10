function altModulation(counter)
global iver state horizon control
control.altModFloat = 0;
if horizon.flagSlope > iver.maxSlope && horizon.index < 8;
    control.abort = 1;
    disp('ABORT!')
elseif horizon.flagSlope > iver.turnSlope && horizon.index < 15;
    control.loopAround = 1;
%     control.altModConst = control.altModConst + 2;
    disp('Turn Around');
elseif horizon.index == 0
    return
elseif horizon.good(horizon.index) == 1 &&...
        horizon.slope > iver.greenSlope
    dist = horizon.index + horizon.offset;
    pseudoBottomDepth = horizon.depth(horizon.index) - ...
        iver.greenSlope*dist;
    pseudoAlt = state.depth(counter) - pseudoBottomDepth;
    control.altModFloat = state.alt(counter) - pseudoAlt;
elseif horizon.good(horizon.index) < 1 &&...
        horizon.slope > iver.yellowSlope
    dist = horizon.index + horizon.offset;
    pseudoBottomDepth = horizon.depth(horizon.index) - ...
        iver.yellowSlope*dist;
    pseudoAlt = state.depth(counter) - pseudoBottomDepth;
    control.altModFloat = state.alt(counter) - pseudoAlt;
end
end