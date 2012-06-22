%% Taking data from Arena
%
% BVH is a text file which contains skeletal data calculated by Arena, but
% its contents needs additional processing to draw the wireframe and create
% the animation.

skeleton = loadbvh('louise');

video = false;

% Prepare the new video file.
if video
  vidObj = VideoWriter('louise');
  open(vidObj);
end

fincr = 5;

ccaxis = [0 0 0 0 0 0];
for ff = 500:fincr:1400
  h = figure(1); clf; hold on
  title(num2str(ff))
  set(h,'color','white')
  
% From the BVH model exported by arena, it's clear that "y" is vertical
% and "z" is medial-lateral. (From the "offsets" between the joints.)
% Therefore, flip Y and Z when plotting to have Matlab's "vertical" z-axis
% match up.

  for nn = 1:Nnodes
    
    if false
      if ~isempty(skeleton(nn).name)
        text(skeleton(nn).Dxyz(1,ff),skeleton(nn).Dxyz(3,ff),skeleton(nn).Dxyz(2,ff),skeleton(nn).name)
      end
    end
    plot3(skeleton(nn).Dxyz(1,ff),skeleton(nn).Dxyz(3,ff),skeleton(nn).Dxyz(2,ff),'.','markersize',20)

    parent = skeleton(nn).parent;
    if parent > 0
      plot3([skeleton(parent).Dxyz(1,ff) skeleton(nn).Dxyz(1,ff)],...
            [skeleton(parent).Dxyz(3,ff) skeleton(nn).Dxyz(3,ff)],...
            [skeleton(parent).Dxyz(2,ff) skeleton(nn).Dxyz(2,ff)])
    end
  end
  
  view(-30,30)
  axis([-100 200 0 200 -50 200])
  
  drawnow
  
  if video, writeVideo(vidObj,getframe); end

end

if video, close(vidObj); end