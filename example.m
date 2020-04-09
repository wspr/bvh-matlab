%% Reading a BVH file and animating it over time
%
% BVH is a text file which contains skeletal data, but its contents needs
% additional processing to draw the wireframe and create the animation.

clear all
close all

name = 'louise';
[skeleton,time] = loadbvh(name);
Njoints = numel(skeleton);

frame_first = 1;
frame_last  = 1500; % =inf to play until the end
frame_step  = 20;
write_video = true;

%% Initialise figure

hf = figure(1); clf; hold on
hf.Color = 'white';
ha = gca;

% General view
title(sprintf('%1.2f seconds (frame %i)',0,1))
view(30,-30)
axis equal
axis off

% Set axes to show all points across all time
xmax = 0; xmin = 0;
ymax = 0; ymin = 0;
zmax = 0; zmin = 0;
for nn = 1:Njoints
  xmax = max(xmax,max(skeleton(nn).Dxyz(1,:)));
  ymax = max(ymax,max(skeleton(nn).Dxyz(2,:)));
  zmax = max(zmax,max(skeleton(nn).Dxyz(3,:)));
  xmin = min(xmin,min(skeleton(nn).Dxyz(1,:)));
  ymin = min(ymin,min(skeleton(nn).Dxyz(2,:)));
  zmin = min(zmin,min(skeleton(nn).Dxyz(3,:)));
end
scalefactor = 1.2;
xmax = (xmax+xmin)/2 + scalefactor*(xmax-xmin)/2;
ymax = (ymax+ymin)/2 + scalefactor*(ymax-ymin)/2;
zmax = (zmax+zmin)/2 + scalefactor*(zmax-zmin)/2;
xmin = (xmax+xmin)/2 - scalefactor*(xmax-xmin)/2;
ymin = (ymax+ymin)/2 - scalefactor*(ymax-ymin)/2;
zmin = (zmax+zmin)/2 - scalefactor*(zmax-zmin)/2;
axis([xmin xmax ymin ymax zmin zmax])

ha.XLimMode = 'manual';
ha.YLimMode = 'manual';
ha.ZLimMode = 'manual';

% plot the first time point
for ff = frame_first
  for nn = Njoints:-1:1
    hp(nn) = plot3(skeleton(nn).Dxyz(1,ff),skeleton(nn).Dxyz(2,ff),skeleton(nn).Dxyz(3,ff),'.','markersize',20);
    parent = skeleton(nn).parent;
    if parent > 0
      hl(nn) = plot3([skeleton(parent).Dxyz(1,ff) skeleton(nn).Dxyz(1,ff)],...
                     [skeleton(parent).Dxyz(2,ff) skeleton(nn).Dxyz(2,ff)],...
                     [skeleton(parent).Dxyz(3,ff) skeleton(nn).Dxyz(3,ff)]);
    end
  end
end

%% Run animation

if write_video, vidObj = VideoWriter(name); open(vidObj); end 

for ff = max(frame_first,1):frame_step:min(frame_last,skeleton(1).Nframes)

  title(sprintf('%1.2f seconds (frame %i)',time(ff),ff))

  % update plot to subsequent time point values
  for nn = 1:Njoints

    hp(nn).XData = skeleton(nn).Dxyz(1,ff);
    hp(nn).YData = skeleton(nn).Dxyz(2,ff);
    hp(nn).ZData = skeleton(nn).Dxyz(3,ff);

    parent = skeleton(nn).parent;
    if parent > 0
      hl(nn).XData = [skeleton(parent).Dxyz(1,ff) skeleton(nn).Dxyz(1,ff)];
      hl(nn).YData = [skeleton(parent).Dxyz(2,ff) skeleton(nn).Dxyz(2,ff)];
      hl(nn).ZData = [skeleton(parent).Dxyz(3,ff) skeleton(nn).Dxyz(3,ff)];
    end

  end

  drawnow
  if write_video, writeVideo(vidObj,getframe(gca)); end

end

if write_video, close(vidObj); end 
