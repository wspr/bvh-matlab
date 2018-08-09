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

flip_yz = @(x) [x(1); x(3); x(2)];

for ff = 50

  h = figure(1); clf; hold on
  title(sprintf('%1.2f seconds',time(ff)))
  set(h,'color','white')

  for nn = 1:length(skeleton)

%    if skeleton(nn).Nchannels ~= 0
%      plot_coord( skeleton(nn).Dxyz(:,ff) , 'length',8,'headlength',3,'index',num2str(nn),'rotate',skeleton(nn).trans(1:3,1:3,ff))
%    end

    plot3(skeleton(nn).Dxyz(1,ff),skeleton(nn).Dxyz(2,ff),skeleton(nn).Dxyz(3,ff),'.','markersize',20)

    parent = skeleton(nn).parent;
    if parent > 0
      plot3([skeleton(parent).Dxyz(1,ff) skeleton(nn).Dxyz(1,ff)],...
            [skeleton(parent).Dxyz(2,ff) skeleton(nn).Dxyz(2,ff)],...
            [skeleton(parent).Dxyz(3,ff) skeleton(nn).Dxyz(3,ff)])
    end

  end

  view(-30,30)
  axis equal off
  drawnow

  if write_video, writeVideo(vidObj,getframe); end

end

if write_video, close(vidObj); end
