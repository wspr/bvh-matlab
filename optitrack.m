%% Taking data from Arena
%
% BVH is a text file which contains skeletal data, but its contents needs
% additional processing to draw the wireframe and create the animation.

name = 'louise';
[skeleton,time] = loadbvh(name);

%%

write_video = false;

% Prepare the new video file.
if write_video, vidObj = VideoWriter(name); open(vidObj); end

fincr = 5;
for ff = 1:fincr:length(time) %#ok<FORPF>

  h = figure(1); clf; hold on
  title(sprintf('%1.2f seconds',time(ff)))
  set(h,'color','white')

% From the BVH model exported by arena, it's clear that "y" is vertical
% and "z" is medial-lateral. (From the "offsets" between the joints.)
% Therefore, flip Y and Z when plotting to have Matlab's "vertical" z-axis
% match up.

  for nn = 1:length(skeleton)
    
    plot3(skeleton(nn).Dxyz(1,ff),skeleton(nn).Dxyz(3,ff),skeleton(nn).Dxyz(2,ff),'.','markersize',20)

    parent = skeleton(nn).parent;
    if parent > 0
      plot3([skeleton(parent).Dxyz(1,ff) skeleton(nn).Dxyz(1,ff)],...
            [skeleton(parent).Dxyz(3,ff) skeleton(nn).Dxyz(3,ff)],...
            [skeleton(parent).Dxyz(2,ff) skeleton(nn).Dxyz(2,ff)])
    end

  end
  
  view(-30,30)
  axis equal off
  drawnow
  
  if write_video, writeVideo(vidObj,getframe); end

end

if write_video, close(vidObj); end