%% Taking data from Arena
%
% Exporting in two formats: C3D and BVH.
%
% C3D is a binary format and we use the external function "loadc3d" (open
% source) to extract the information. This gives us marker points only.
%
% BVH is a text file which contains skeletal data calculated by Arena, but
% its contents needs additional processing to draw the wireframe and create
% the animation.

%% C3D example
%
% The wireframe is approximate and may potentially change with different takes.

fname = 'louise';

c3ddata = loadc3d([fname,'.c3d']);
Nmarkers = size(c3ddata,2);

connect_points = { ...
  [14 15 16 14] , ... left hand
  [20 21 22 20] , ... right hand
  [8 9 10 8] , [5 9 10 5] , ... "head"
  [1 2 3 1] , ... waist
  [1 6 7 1] , [5 6 7 5] , ...   torso
  [3 7] , [2 6] , [13 19 2 3 13] , ... more torso
  [22 17 18 19 5 13 12 11 15] , ... join arms
  [29 30 32 29] , ... right leg
  [23 24 26 23] , ...   left leg
  [30 2 29 30] , [3 23 24 3] , ... more legs
  [31 33 34 31] , ... right foot
  [27 28 25 27] ,  ... left foot
  [31 32] , [25 26] ... join feet
  };
Nconnect = length(connect_points);

Nframes = size(c3ddata,1);

% every tenth point only for the animation:
for tt = 500:10:1200
  
  figure(1); clf; hold on
  view(-23,18)
  axis([-1000 1000 -500 1000 0 1500])
  
  % plot each individual marker with a label
  for mm = 1:Nmarkers
    plot3(c3ddata(tt,mm,1),c3ddata(tt,mm,2),c3ddata(tt,mm,3),'o')
    text(c3ddata(tt,mm,1),c3ddata(tt,mm,2),c3ddata(tt,mm,3),['   ',num2str(mm)])
  end
  
  % plot the "body segments", but really need a skeletal model
  for ff = 1:Nconnect
    plot3(c3ddata(tt,connect_points{ff},1),c3ddata(tt,connect_points{ff},2),c3ddata(tt,connect_points{ff},3))
  end
  
  
end

%% BVH example


%% Plot

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
  
  % plot c3D markers as well:
  mrange = 1:Nmarkers;
  mrange([4 35 36]) = [];
  
  for mm = mrange
    plot3(-c3ddata(ff,mm,1)/10,c3ddata(ff,mm,2)/10,c3ddata(ff,mm,3)/10,'o','color','red')
    % text(-c3ddata(ff,mm,1)/10,c3ddata(ff,mm,2)/10,c3ddata(ff,mm,3)/10,num2str(mm))
  end
  
  view(-30,30)
  axis([-100 200 0 200 -50 200])
  
  drawnow
  
  if video, writeVideo(vidObj,getframe); end

end

if video, close(vidObj); end