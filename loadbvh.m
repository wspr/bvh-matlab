function skeleton = loadbvh(name)
%% LOADBVH  Load a .bvh file.

name = 'louise';
fname = [name,'.bvh'];

%% Load and parse data

% Markers:
c3ddata = loadc3d([name,'.c3d']);
Nmarkers = size(c3ddata,2);

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
    
    nn = nn+1;
    
    skeleton(nn).name = C(ii+1);
    skeleton(nn).offset = [str2double(C(ii+4)) ; str2double(C(ii+5)) ; str2double(C(ii+6))];
    skeleton(nn).channels = str2double(C(ii+8));
    skeleton(nn).nestdepth = brace_count;
    skeleton(nn).is_end_effector = false;

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
    
    if skeleton(nn).channels == 3
      skeleton(nn).order = [C{ii+9}(1),C{ii+10}(1),C{ii+11}(1)];
    elseif skeleton(nn).channels == 6
      skeleton(nn).order = [C{ii+12}(1),C{ii+13}(1),C{ii+14}(1)];
    end
    
  elseif strcmp( C(ii) , 'End' )
    if ~strcmp( C(ii+1) , 'Site' ), error('"End" keyword must be followed by "Site".'); end
    
    nn = nn+1;
    
    skeleton(nn).offset = [str2double(C(ii+4)) ; str2double(C(ii+5)) ; str2double(C(ii+6))];
    skeleton(nn).parent = nn-1; % always the direct child
    skeleton(nn).nestdepth = brace_count;
    skeleton(nn).is_end_effector = true;
    skeleton(nn).channels = 0;
        
  end
  
end

%% Initial processing and error checking

Nnodes = length(skeleton);
Nchannels = sum([skeleton.channels]);
Nchainends = sum([skeleton.is_end_effector]);

% Calculate number of header lines:
%  - 5 lines per joint
%  - 4 lines per chain end
%  - 4 additional lines (first one and last three)
Nheaderlines = (Nnodes-Nchainends)*5 + Nchainends*4 + 4;

rawdata = importdata(fname,' ',Nheaderlines);

index = strncmp(rawdata.textdata,'Frames:',7);
Nframes = sscanf(rawdata.textdata{index},'Frames: %f');

index = strncmp(rawdata.textdata,'Frame Time:',10);
frame_time = sscanf(rawdata.textdata{index},'Frame time: %f');

if size(rawdata.data,2) ~= Nchannels
  error('Error reading BVH file: channels count does not match.')
end

if size(rawdata.data,1) ~= Nframes
  warning('LOADBVH:frames_wrong','Error reading BVH file: frames count does not match; continuing anyway.')
end

%% Define transformations

Rx = @(t) [1 0 0; 0 cosd(t) -sind(t); 0 sind(t) cosd(t)];
Ry = @(t) [cosd(t) 0 sind(t); 0 1 0; -sind(t) 0 cosd(t)];
Rz = @(t) [cosd(t) -sind(t) 0; sind(t) cosd(t) 0; 0 0 1];
TransM = @(R,d) [R, d; 0 0 0 1];

%% Load data into skeleton structure
%
% Assume for now that all rotations are in the order ZXY

channel_count = 0;

for nn = 1:Nnodes
    
  if skeleton(nn).channels == 6
    
    skeleton(nn).Dxyz = repmat(skeleton(1).offset,[1 Nframes]) + rawdata.data(:,channel_count+[1 2 3])';
    rot_data = rawdata.data(:,channel_count+[4 5 6])';
    
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
      
      rotM = eye(3);
      for ii = 1:length(skeleton(nn).order)
        switch skeleton(nn).order(ii)
          case 'X'
            rotM = rotM*Rx(skeleton(nn).rot(1,ff));
          case 'Y'
            rotM = rotM*Ry(skeleton(nn).rot(2,ff));
          case 'Z'
            rotM = rotM*Rz(skeleton(nn).rot(3,ff));
        end
      end
    
      skeleton(nn).trans(:,:,ff) = TransM(rotM,skeleton(nn).Dxyz(:,ff));
      
    end
    
  elseif skeleton(nn).channels == 3
    
    rot_data = rawdata.data(:,channel_count+[1 2 3])';
    
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
    
  elseif skeleton(nn).channels == 0
    
    skeleton(nn).Dxyz  = nan(3,Nframes);
    
  else
    error('Not sure how to handle not (0 or 3 or 6) number of channels.')
  end
  
  channel_count = channel_count + skeleton(nn).channels;
  
end

%% Calculate kinematics

for nn = find([skeleton.parent] ~= 0)
  
  parent = skeleton(nn).parent;
  
  for ff = 1:Nframes
    
    rotM = eye(3);
    for ii = 1:length(skeleton(nn).order)
      switch skeleton(nn).order(ii)
        case 'X'
          rotM = rotM*Rx(skeleton(nn).rot(1,ff));
        case 'Y'
          rotM = rotM*Ry(skeleton(nn).rot(2,ff));
        case 'Z'
          rotM = rotM*Rz(skeleton(nn).rot(3,ff));
      end
    end
      
    skeleton(nn).trans(:,:,ff) = skeleton(parent).trans(:,:,ff) * TransM(rotM,skeleton(nn).offset);
    skeleton(nn).Dxyz(:,ff) = skeleton(nn).trans([1 2 3],4,ff);
    
  end

end






  end

end

