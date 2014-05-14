function [x, y] = collapse(xin, yin)
%COLLAPSE - collapse repeated value vector into a value of unique values

x = [];
y = [];
tmpx = [];
tmpy = [];
for i = 1:length(xin)
	if i == 1
		tmpx = xin(i);
		tmpy = yin(i);
		x = cat(2,x,[xin(i)]);
		y = cat(2,y,[yin(i)]);
	elseif (xin(i) ~= tmpx) || (yin(i) ~= tmpy)
		tmpx = xin(i);
		tmpy = yin(i);
		x = cat(2,x,[xin(i)]);
		y = cat(2,y,[yin(i)]);
    end
end
