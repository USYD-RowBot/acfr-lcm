function [deg,min] = degdec2degmin(degdec);

deg = fix(degdec);
min = abs((degdec-deg)*60);
