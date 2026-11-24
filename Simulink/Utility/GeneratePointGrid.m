% config

xlim = [-1 1]; %m
ylim = [-1 1]; %m

xres = 0.3048; %m
yres = 0.3048; %m

% generate
xscope = xlim(2)-xlim(1);
yscope = ylim(2)-ylim(1);
xnum = floor(xscope/xres);
ynum = floor(yscope/yres);

floorGrid = [];

for i = 0:xnum
    for j = 0:ynum
        floorGrid(end+1, :) = [(xlim(1) + i*xres) (ylim(1) + j*yres) -0.007];
    end
end
