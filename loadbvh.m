function [skeleton,time] = loadbvh(fname)
%% LOADBVH  Load a .bvh (Biovision) file.
%
% Loads BVH file specified by FNAME (with or without .bvh extension)
% and parses the file, calculating joint kinematics and storing the
% output in SKELETON.

%% Load and parse header data
%
% The file is opened for reading, primarily to extract the header data (see
% next section). However, I don't know how to ask Matlab to read only up
% until the line "MOTION", so we're being a bit inefficient here and
% loading the entire file into memory. Oh well.

if ~strncmpi(fliplr(fname),'hvb.',4)
  fname = [fname,'.bvh'];
end

fid = fopen(fname);
C = textscan(fid,'%s');
fclose(fid);
C = C{1};


%% Parse data
%
% This is a cheap tokeniser, not particularly clever.
% Iterate word-by-word, counting braces and extracting data.

% Initialise:
skeleton = [];
ii = 1;
nn = 0;
brace_count = 1;

while ~strcmp( C(ii) , 'MOTION' )
  
  ii = ii+1;
  token = C{ii};
  
  if strcmp( token , '{' )
    
    brace_count = brace_count + 1;
    
  elseif strcmp( token , '}' )
    
    brace_count = brace_count - 1;
    
  elseif strcmp( token , 'OFFSET' )
    
    skeleton(nn).offset = [str2double(C(ii+1)) ; str2double(C(ii+2)) ; str2double(C(ii+3))];
    ii = ii+3;
    
  elseif strcmp( token , 'CHANNELS' )
    
    skeleton(nn).Nchannels = str2double(C(ii+1));
    
    % What is the order of the rotations?
    if skeleton(nn).Nchannels == 3
      skeleton(nn).order = [C{ii+2}(1),C{ii+3}(1),C{ii+4}(1)];
    elseif skeleton(nn).Nchannels == 6
      skeleton(nn).order = [C{ii+5}(1),C{ii+6}(1),C{ii+7}(1)];
    else
      error('Not sure how to handle not (3 or 6) number of channels.')
    end
    
    % The 'norder' field is an index corresponding to the order of 'X' 'Y' 'Z'.
    for rr = 1:3
      % "X" == 88, "Y" == 89, "Z" == 90 
      skeleton(nn).norder(rr) = skeleton(nn).order(rr)-87;
    end
    
    ii = ii + skeleton(nn).Nchannels + 1;

  elseif strcmp( token , 'JOINT' ) || strcmp( token , 'ROOT' )
    % Regular joint
    
    nn = nn+1;
    
    skeleton(nn).name = C(ii+1);
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
    
    ii = ii+1;
            
  elseif strcmp( [C{ii},' ',C{ii+1}] , 'End Site' )
    % End effector; unnamed terminating joint
    %
    % N.B. The "two word" token here is why we don't use a switch statement
    % for this code.
    
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

%% Load motion data into skeleton structure
%
% We have three possibilities for each node we come across:
% (a) a root node that has displacements already defined,
%     for which the transformation matrix can be directly calculated;
% (b) a joint node, for which the transformation matrix must be calculated
%     from the previous points in the chain; and
% (c) an end effector, which only has displacement to calculate from the
%     previous node's transformation matrix and the offset of the end
%     joint.
%
% These are indicated in the skeleton structure, respectively, by having
% six, three, and zero "channels" of data.
% In this section of the code, the channels are read in where appropriate
% and the relevant arrays are pre-initialised for the subsequent calcs.

channel_count = 0;

for nn = 1:Nnodes
    
  if skeleton(nn).Nchannels == 6 % root node
    
    % assume translational data is always ordered XYZ
    skeleton(nn).Dxyz = repmat(skeleton(nn).offset,[1 Nframes]) + rawdata.data(:,channel_count+[1 2 3])';
    
    skeleton(nn).rot = nan(3,Nframes);
    for ii = 1:3
      rr = skeleton(nn).norder(ii);
      skeleton(nn).rot(rr,:) = rawdata.data(:,channel_count+3+ii)';
    end
        
    % Kinematics of the root element:
    skeleton(nn).trans = nan(4,4,Nframes);
    for ff = 1:Nframes
      skeleton(nn).trans(:,:,ff) = transformation_matrix(skeleton(nn).Dxyz(:,ff) , skeleton(nn).rot(:,ff) , skeleton(nn).order);
    end
    
  elseif skeleton(nn).Nchannels == 3 % joint node
        
    skeleton(nn).rot = nan(3,Nframes);
    for ii = 1:3
      rr = skeleton(nn).norder(ii);
      skeleton(nn).rot(rr,:) = rawdata.data(:,channel_count+ii)';
    end
    
    skeleton(nn).Dxyz  = nan(3,Nframes);
    skeleton(nn).trans = nan(4,4,Nframes);
    
  elseif skeleton(nn).Nchannels == 0 % end node
    
    skeleton(nn).Dxyz  = nan(3,Nframes);
    
  end
  
  channel_count = channel_count + skeleton(nn).Nchannels;
  
end


%% Calculate kinematics
%
% No calculations are required for the root nodes.

% For each joint, calculate the transformation matrix and for convenience
% extract each position in a separate vector.
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
% Constructs the transformation for given displacement, DISPL, and
% rotations RXYZ. The vector RYXZ is of length three corresponding to
% rotations around the X, Y, Z axes.
%
% The third input, ORDER, is a string indicating which planar rotations
% to apply and in which order. E.g.,
%
%  - 'ZXY' refers applying rotations RXYZ around Z first, then X, then Y.
%  - 'XY' would apply only rotations around X then Y.
%  - 'XX' would apply the rotation around the x-axis twice,
%    but I don't know why you'd want to do that.
%
% Years ago we benchmarked that multiplying the separate rotation matrices
% was more efficient than pre-calculating the final rotation matrix
% symbolically, so we don't "optimise" by having a hard-coded rotation
% matrix for, say, 'ZXY' which seems more common in BVH files.
% Should revisit this assumption one day.

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
