%% Taking data from Arena
%
% BVH is a text file which contains skeletal data, but its contents needs
% additional processing to draw the wireframe and create the animation.

clear all
close all

name = 'louise';
[skeleton,time] = loadbvh(name);

frame_step = 10;
write_video = false;

%% Initialise figure

h = figure(1); clf; hold on
set(h,'color','white')
view(-30,30)
axis equal off
Njoints = numel(skeleton);

for ff = 1
  for nn = 1:Njoints

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

for ff = 1:frame_step:skeleton(1).Nframes

  title(sprintf('%1.2f seconds',time(ff)))

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

  view(-30,30)
  axis equal off
  drawnow
  
  if write_video, writeVideo(vidObj,getframe); end

end

if write_video, close(vidObj); end 
