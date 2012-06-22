function [skeleton,time] = loadbvh(fname)
%% LOADBVH  Load a .bvh file.

fname = [name,'.bvh'];

%% Load and parse data

% File access:
fid = fopen(fname);
C = textscan(fid,'%s');
fclose(fid);
C = C{1};

% Initialise:
skeleton = [];
ii = 1;
nn = 0;
brace_count = 1;

% Parse:
while ~strcmp( C(ii) , 'MOTION' )
  
  ii = ii+1;
  
  if strcmp( C(ii) , '{' )
    brace_count = brace_count + 1;
  elseif strcmp( C(ii) , '}' )
    brace_count = brace_count - 1;
  elseif strcmp( C(ii) , 'JOINT' ) || strcmp( C(ii) , 'ROOT' )
    % Regular joint
    
    nn = nn+1;
    
    skeleton(nn).name = C(ii+1);
    skeleton(nn).offset = [str2double(C(ii+4)) ; str2double(C(ii+5)) ; str2double(C(ii+6))];
    skeleton(nn).Nchannels = str2double(C(ii+8));
    skeleton(nn).nestdepth = brace_count;

    if brace_count == 1
      % root node
      skeleton(nn).parent = 0;
    elseif skeleton(nn-1).nestdepth + 1 == brace_count;
      % if I am a child, the previous node is my parent:
      skeleton(nn).parent = nn-1;
    else
      % if not, what is the node corresponding to this brace count?
      skeleton(nn).parent = skeleton(brace_count).parent;
    end
    
    % What is the order of the rotations?
    if skeleton(nn).Nchannels == 3
      skeleton(nn).order = [C{ii+9}(1),C{ii+10}(1),C{ii+11}(1)];
    elseif skeleton(nn).Nchannels == 6
      skeleton(nn).order = [C{ii+12}(1),C{ii+13}(1),C{ii+14}(1)];
    end
        
  elseif strcmp( [C{ii},' ',C{ii+1}] , 'End Site' )
    % End effector; unnamed terminating joint
    
    nn = nn+1;
    
    skeleton(nn).name = ' ';
    skeleton(nn).offset = [str2double(C(ii+4)) ; str2double(C(ii+5)) ; str2double(C(ii+6))];
    skeleton(nn).parent = nn-1; % always the direct child
    skeleton(nn).nestdepth = brace_count;
    skeleton(nn).Nchannels = 0;
        
  end
  
end

%% Initial processing and error checking

Nnodes = numel(skeleton);
Nchannels = sum([skeleton.Nchannels]);
Nchainends = sum([skeleton.Nchannels]==0);

% Calculate number of header lines:
%  - 5 lines per joint
%  - 4 lines per chain end
%  - 4 additional lines (first one and last three)
Nheaderlines = (Nnodes-Nchainends)*5 + Nchainends*4 + 4;

rawdata = importdata(fname,' ',Nheaderlines);

index = strncmp(rawdata.textdata,'Frames:',7);
Nframes = sscanf(rawdata.textdata{index},'Frames: %f');

index = strncmp(rawdata.textdata,'Frame Time:',10);
frame_time = sscanf(rawdata.textdata{index},'Frame Time: %f');

time = frame_time*(0:Nframes-1);

if size(rawdata.data,2) ~= Nchannels
  error('Error reading BVH file: channels count does not match.')
end

if size(rawdata.data,1) ~= Nframes
  warning('LOADBVH:frames_wrong','Error reading BVH file: frames count does not match; continuing anyway.')
end

%% Load data into skeleton structure
%
% Assume for now that all rotations are in the order ZXY

channel_count = 0;

for nn = 1:Nnodes
    
  if skeleton(nn).Nchannels == 6
    
    % assume translational data is always ordered XYZ
    skeleton(nn).Dxyz = repmat(skeleton(nn).offset,[1 Nframes]) + rawdata.data(:,channel_count+[1 2 3])';
    
    skeleton(nn).rot = nan(3,Nframes);
    for ii = 1:length(skeleton(nn).order)
      switch skeleton(nn).order(ii)
        case 'X'
          skeleton(nn).rot(1,:) = rawdata.data(:,channel_count+3+ii)';
        case 'Y'
          skeleton(nn).rot(2,:) = rawdata.data(:,channel_count+3+ii)';
        case 'Z'
          skeleton(nn).rot(3,:) = rawdata.data(:,channel_count+3+ii)';
      end
    end
    
    % Kinematics of the root element:
    skeleton(nn).trans = nan(4,4,Nframes);
    for ff = 1:Nframes
      skeleton(nn).trans(:,:,ff) = transformation_matrix(skeleton(nn).Dxyz(:,ff) , skeleton(nn).rot(:,ff) , skeleton(nn).order);
    end
    
  elseif skeleton(nn).Nchannels == 3
        
    skeleton(nn).rot = nan(3,Nframes);
    for ii = 1:length(skeleton(nn).order)
      switch skeleton(nn).order(ii)
        case 'X'
          skeleton(nn).rot(1,:) = rawdata.data(:,channel_count+ii)';
        case 'Y'
          skeleton(nn).rot(2,:) = rawdata.data(:,channel_count+ii)';
        case 'Z'
          skeleton(nn).rot(3,:) = rawdata.data(:,channel_count+ii)';
      end
    end
    
    skeleton(nn).Dxyz  = nan(3,Nframes);
    skeleton(nn).trans = nan(4,4,Nframes);
    
  elseif skeleton(nn).Nchannels == 0
    
    skeleton(nn).Dxyz  = nan(3,Nframes);
    
  else
    error('Not sure how to handle not (0 or 3 or 6) number of channels.')
  end
  
  channel_count = channel_count + skeleton(nn).Nchannels;
  
end

%% Calculate kinematics

for nn = find([skeleton.parent] ~= 0 & [skeleton.Nchannels] ~= 0)
  
  parent = skeleton(nn).parent;
  
  for ff = 1:Nframes
    
    transM = transformation_matrix( skeleton(nn).offset , skeleton(nn).rot(:,ff) , skeleton(nn).order );
    skeleton(nn).trans(:,:,ff) = skeleton(parent).trans(:,:,ff) * transM;
    skeleton(nn).Dxyz(:,ff) = skeleton(nn).trans([1 2 3],4,ff);
    
  end

end

% For an end effector we don't have rotation data;
% just need to calculate the final position.
for nn = find([skeleton.Nchannels] == 0)
  
  parent = skeleton(nn).parent;
  
  for ff = 1:Nframes
    transM = skeleton(parent).trans(:,:,ff) * [eye(3), skeleton(nn).offset; 0 0 0 1];
    skeleton(nn).Dxyz(:,ff) = transM([1 2 3],4);
  end

end

end



function transM = transformation_matrix(displ,rxyz,order)

cx = cosd(rxyz(1));
cy = cosd(rxyz(2));
cz = cosd(rxyz(3));
sx = sind(rxyz(1));
sy = sind(rxyz(2));
sz = sind(rxyz(3));

rotM = eye(3);
for ii = 1:length(order)
  switch order(ii)
    case 'X'
      rotM = rotM*[1 0 0; 0 cx -sx; 0 sx cx];
    case 'Y'
      rotM = rotM*[cy 0 sy; 0 1 0; -sy 0 cy];
    case 'Z'
      rotM = rotM*[cz -sz 0; sz cz 0; 0 0 1];
  end
end

transM = [rotM, displ; 0 0 0 1];

end

