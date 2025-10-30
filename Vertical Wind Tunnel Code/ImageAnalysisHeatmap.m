%% Image Analysis and Heatmap Generator
% Images with more saturation at areas of greater drop spacial
% residence in time are refered to as "heatmaps".
%
% This code is designed to run an Mp4 file for the drop and a Chix file for
% drop data. If not using a chix file, then code must be changed to
% manually enter in frame rate. See "readCIHX" in the internal helper function
% "getfile" (at the bottom of this script).
%
% Once ran, the code will prompt for the oppening of an Mp4 file containing
% the drop. NOTE: the chix file MUST have the same name as the Mp4 file
% (only the file extention is different).
%
% Drops will be binarized, analyzed, converted to a heatmap image, and
% stored. Figures will appear of the aggragate and characteristic shapes. A
% new folder will be created at the location of the Mp4 file titled
% "images" and will contain both figures as tiff files, a binarized video,
% and a heatmap video.
%
% The characteristic diameter will be outputted into the terminal and saved
% along with other drop data in the Cache.
% NOTE: this script is designed to be immediately ran before
% "frequencySpectra.m" and so should be followed with it. 
% NOTE: this script does not perminately store the cache, it is designed to
% be overriden.
%
% Editable lines: The PixelFilter and MinBlobArea both filter out "blobs" of
% less than their respective sizes in pixels. The MaxBlobArea will filter
% out "blobs" greater than its size in pixels, it is designed for large
% obstructions that do not directly interfere with the drop. All opperate
% AFTER binarization occurs according to the adjustable sensitivity. 
% The sensitivity is the MOST important value to adjust if having issues 
% with drop binarization. 
% The Resolution is the [pxls/mm] measurement of the viewing plane. This value
% must be known or at least approximated in advacnce. Lastly, the
% heatMapDimention is the height and with in pixels of the heatmap. This
% value must be greater than the with of the drop AT ALL FRAMES. I
% recommend over approximating instead of underapproximating this value.
%
% Issues: Most processing issues will have to do with drop definition. The
% internal helper function (at the bottom of this script)
% "process(processInfo,currFrame,pxlxy,dim,exl)" is where image processing
% takes place and should be where major adjustments should happen if
% adjusting the PixelFilter, Min/MaxBlobArea, and sensitivity do not work.
%
%
% Syed Jaffar Raza and Johsua Watkins 06/27/2025


clear; close all; clc
format long
addpath("Functions")

%% Editable lines

% Input values
processInfo1.PixelFilter = 80;
processInfo1.MinBlobArea = 500;
processInfo1.MaxBlobArea = 10000;
processInfo1.sensitivity = 0.48;  % Recommend sensitivity ~0.2-0.3 or 0.53 with pixel filter on

% set resolution
resolution=10; %pixels/mm

% Set with and height of heatmap
heatMapDimention = 300;


%%

% Ask the user to specify the .mp4 file to be analyzed
[cam1Video]= getfile();
cam1 = cam1Video.VideoReader;

% Setup Fps
secondsPerFrame_cam1 = cam1.FrameRate^-1;

% Create OutputVideos
mkdir(cam1Video.pathname, 'images')
outputVideo1=makeVid(cam1Video,'Binarized');
vp1=vision.VideoPlayer('Position',[100,400,800,400]);
outputVideo3=makeVid(cam1Video,'Heatmap_');
vp3=vision.VideoPlayer('Position',[100,400,800,400]);


% Set drop initial location and time
preveousxy1 = [0,0];
cam1.CurrentTime = 0;

% Set up heatmap canvas
canvas1 = zeros([heatMapDimention,heatMapDimention]);
TempCan1 = canvas1;

% Find center of video
heatMapCentroid = [heatMapDimention/2,heatMapDimention/2];


statsList_1 = [];
while hasFrame(cam1)
    % Read Frames
    currFrame_1 = rgb2gray(readFrame(cam1));
    
    % Current Frame number
    currFrameNumber = round(cam1.CurrentTime*cam1.FrameRate);
    

    if cam1.CurrentTime <= secondsPerFrame_cam1

        % Binarize Frames
        [filteredImage_1,BW_1]=process(processInfo1,currFrame_1,preveousxy1,heatMapDimention,true);

        % Select First image and crop
        firstImg_1 = filteredImage_1;
        [cropX_1,cropY_1]=cropBounds(firstImg_1);
        
        % Select the first image to filter the background and detect the
        % position of the drop
        firstImg_1 = firstImg_1(cropY_1(1):cropY_1(2), cropX_1(1):cropX_1(2));  % Apply cropping bounds
        makeImage(currFrameNumber,cam1Video.pathname,cam1Video.fileName, firstImg_1,2)

        firstImg_1 = filteredImage_1;
        [preveousxy1(1),preveousxy1(2)]=selectCentroid(firstImg_1,cropX_1,cropY_1);
    else

        % Binarize Frames
        [filteredImage_1,BW_1]=process(processInfo1,currFrame_1,preveousxy1,heatMapDimention,false);

    end

    % Filter frames
    filteredImage_1=filteredImage_1(cropY_1(1):cropY_1(2), cropX_1(1):cropX_1(2));

    % Read the processed image and Store Stats
    [stats,filt_1,preveousxy1]=IMGanalyze(filteredImage_1,preveousxy1);

    
    canCropX = round([heatMapCentroid(1)-(stats.BoundingBox(3)./2) , heatMapCentroid(1)+(stats.BoundingBox(3)./2)]);
    canCropY = round([heatMapCentroid(2)-(stats.BoundingBox(4)./2) , heatMapCentroid(2)+(stats.BoundingBox(4)./2)]);
    
    g1c = filteredImage_1(round(stats.BoundingBox(2)):round((stats.BoundingBox(2)+stats.BoundingBox(4))), round(stats.BoundingBox(1)):round((stats.BoundingBox(1)+stats.BoundingBox(3))));
    canvas1(canCropY(1):canCropY(2), canCropX(1):canCropX(2)) = canvas1(canCropY(1):canCropY(2), canCropX(1):canCropX(2))+g1c;
    TempCan1(canCropY(1):canCropY(2), canCropX(1):canCropX(2)) = TempCan1(canCropY(1):canCropY(2), canCropX(1):canCropX(2))+g1c;

    Vert1 = max(sum(g1c,1));
    Hori1 = max(sum(g1c,2));
    stats.Hori = Hori1;
    stats.Vert = Vert1;
    stats.DiskVol = sum(((sum(g1c,2)/2).^2)*pi);

    if stats.Nullity == true
        statslist_1(currFrameNumber) = statslist_1(currFrameNumber-1);
        fprintf("\n REPEAT USED \n")
    else
        statslist_1(currFrameNumber) = stats;
    end
    
    % Update video player
    vp1.step(filt_1)
    gray1 = mat2gray(canvas1,[0 max(max(canvas1))]);
    vp3.step(gray1)

    % Write the image to the video
    writeVideo(outputVideo1, filt_1);
    writeVideo(outputVideo3,gray1);

end


% Close the VideoWriter
close(outputVideo1);
close(outputVideo3);

% Display a message when the video is successfully created
disp(['Video saved as ' outputVideo1.Path]);


% Save Relevent Data
[camera1Stats,camera1StatsMM]=organData(statslist_1,resolution,cam1Video.frameRate);





%%

% Time-averaged area
mean1 = mean(camera1Stats.area(:,2));

% Generate Characteristic Shape
canvas1High = canvas1;
lvl = levelmaxn(canvas1High,mean1);
canvas1High(canvas1High<lvl) = 0;

% Show shape
figure;
hold on
imshow(canvas1High)
title("Characteristic Shape")
hold off

% Characteristic Diameter
CharVol = (sum(((sum((canvas1High>0),2)./2).^2)*pi));
CharDiam = 2*((CharVol*(3/4)/pi)^(1/3));
camera1Stats.eqCharDiam = CharDiam;

% Print Diameter
fprintf("Characteristic Diameter: %4.2f mm \n",camera1Stats.eqCharDiam./resolution);

% Characteristic shape properties
charShape = regionprops((canvas1High>0),"Area","BoundingBox","Centroid","Eccentricity","Circularity","EquivDiameter","MajorAxisLength","MinorAxisLength","Orientation");
camera1Stats.charShape = charShape;
camera1Stats.charShape.Hori = max(sum(canvas1High,2));
camera1Stats.charShape.Vert = max(sum(canvas1High,1));


%%
% Remove videoReader and save .mat in a folder
processInfo.processInfo1 = processInfo1;
cam1Video = rmfield(cam1Video,"VideoReader");
save(strcat('Cache/rawData.mat'))


imwrite(canvas1High, strcat(cam1Video.pathname,'images/CharShapeC001.tiff'), 'tiff','Compression','none');
imwrite(mat2gray(canvas1,[0 max(max(canvas1))]), strcat(cam1Video.pathname,'images/AggShapeC001.tiff'), 'tiff','Compression','none');

% Make sound when done
beep


%% Helper Functions

function [camerastats,camerastatsMM]=organData(stats,resolution,frameRate)
   
    for i=1:length(stats)
        camerastats.centroid(i,:) =[i stats(i).Centroid];%pixels
        camerastats.bbox(i,:)     =[i stats(i).BoundingBox];
        camerastats.ma(i,:)       =[i stats(i).MajorAxisLength];
        camerastats.mi(i,:)       =[i stats(i).MinorAxisLength];
        camerastats.angle(i,:)    =[i stats(i).Orientation];
       camerastats.area(i,:)    =[i stats(i).Area];
      camerastats.diam(i,:)    =[i stats(i).EquivDiameter];
       camerastats.perimeter(i,:)    =[i stats(i).Perimeter];
       camerastats.Hori(i,:)    =[i stats(i).Hori];
       camerastats.Vert(i,:)    =[i stats(i).Vert];
       camerastats.DiskVol(i,:)    =[i stats(i).DiskVol];
%        camerastats.RelativeX(i,:)    =[i stats(i).RelativeX];
%        camerastats.RelativeY(i,:)    =[i stats(i).RelativeY];
       camerastats.frameRate = frameRate;

    end
    for i=1:length(stats)% units of mm
       camerastatsMM.centroid(i,:)   =[i stats(i).Centroid.*resolution];%
        camerastatsMM.bbox(i,:)      =[i stats(i).BoundingBox.*resolution];%mm(all)
        camerastatsMM.ma(i,:)        =[i stats(i).MajorAxisLength*resolution];
        camerastatsMM.mi(i,:)        =[i stats(i).MinorAxisLength*resolution];
        camerastatsMM.angle(i,:)     =[i stats(i).Orientation];%degrees
       camerastatsMM.area(i,:)       =[i stats(i).Area*resolution];
       camerastatsMM.diam(i,:)       =[i stats(i).EquivDiameter*resolution];
       camerastatsMM.perimeter(i,:)  =[i stats(i).Perimeter*resolution];
       camerastatsMM.frameRate = frameRate;

    end


end

function  [stats_fin,filt,preveousxy]=IMGanalyze(filteredImage,preveousxy) 
    filt = double(filteredImage);
    stats= regionprops(filteredImage, 'Area', 'Centroid', 'MajorAxisLength', 'MinorAxisLength','BoundingBox','Orientation','Perimeter','EquivDiameter','PixelList');
    
    if isempty(stats)
        stats_fin.Nullity = true;
    else
        stats_fin = stats(1);
        % only read largest Area object to minimize noise
        for i = 1:length(stats)
            if stats(i).Area > stats_fin.Area
                stats_fin = stats(i);
            end
        end
    
        % only read object closest to drop in previous frame
        passed = false;
        if preveousxy(1) ~= 0
            
            for i = 1:length(stats)
                pxldist = sqrt((preveousxy(1)-stats(i).Centroid(1))^2 + (preveousxy(2)-stats(i).Centroid(2))^2);
                
                if pxldist < 50
                    stats_fin = stats(i);
                    passed = true;
                end
            end
        end
    
        if ~passed
            preveousxy = preveousxy;
            fprintf("\n Drop Location Lost \n")
        else
            preveousxy = stats_fin.Centroid;
        end
        stats_fin.Nullity = false;
    end


end

function [cam1Video,frameRate] = getfile

    % First CAM 
    [filenameA, pathnameA] = uigetfile('*.mp4');
    fileA = fullfile(pathnameA, filenameA);
    infoA = VideoReader(fileA);  % Get video info
    numImagesA = infoA.NumFrames;  % Number of frames in the mp4

    % CIHX File
    splitfilename = strsplit(filenameA,".");
    ChixFile = readCIHX(strcat(pathnameA,splitfilename{1},'.cihx'));
    frameRateA = ChixFile.FrameRate;

    
    parts = strsplit(filenameA, '_');
    lastpart=char(parts(end));

    % Assign variables
    cam1Video.numImages = numImagesA;
    cam1Video.pathname = pathnameA;
    cam1Video.VideoReader = infoA;
    cam1Video.fileName = filenameA;
    cam1Video.frameRate = frameRateA;


end

function [cropX,cropY]=cropBounds(first_image)
    % Ask the user to select cropping bounds for the rest of the analysis
    imshow2(first_image);
    title('Click upper left then bottom right of area of interest')
    [cropX, cropY] = ginput(2);  % User specifies the top/left and bottom-right of the tunnel
    close all
    cropX = round(cropX);
    cropY = round(cropY);

end

function [cropX,cropY]=selectCentroid(first_image, cropBX,cropBY)
    % Ask the user to select cropping bounds for the rest of the analysis
    imshow2(first_image);
    title('Select 1st Centriod of drop')
    [cropX, cropY] = ginput(1);  % User specifies the top/left and bottom-right of the tunnel
    close all
    cropX = round(cropX);
    cropY = round(cropY);
    cropX = cropX(1)-cropBX(1);
    cropY = cropY(1)-cropBY(1);

end


function [filteredImage,BW]=process(processInfo,currFrame,pxlxy,dim,exl)
    
    % Step 1 % Cleam image
    currFrame = imadjust(currFrame);
    currFrame = localcontrast(currFrame);
    currFrame = imflatfield(currFrame,50);

    BW = imbinarize(currFrame, 'adaptive', 'ForegroundPolarity', 'dark', 'Sensitivity', processInfo.sensitivity);
    blackandwhite = BW;
    blackandwhite = ~blackandwhite;  % Invert BW image

    % Step 2 % Turn pixels beyond Heatmap Dimention zero
    if ~exl
        crop2 = [pxlxy(1)-dim pxlxy(2)-dim];
        crop2(crop2<0) = 1;
        crop2U = [pxlxy(1)+dim pxlxy(2)+dim];
        if crop2U(1) > width(blackandwhite)
            crop2U(1) = width(blackandwhite);
        end
        if crop2U(2) > height(blackandwhite)
            crop2U(2) = height(blackandwhite);
        end
        crop2 = [crop2 crop2U];
        blackandwhite(1:crop2(2),:) = 0;
        blackandwhite(crop2(4):height(blackandwhite),:) = 0;
        blackandwhite(:,1:crop2(1)) = 0;
        blackandwhite(:,crop2(3):width(blackandwhite)) = 0;
    end
    
    % Step 3 % Remove noise and fill in drop
    filteredImage = bwareafilt(blackandwhite,[processInfo.MinBlobArea processInfo.MaxBlobArea]);
    filteredImage = bwareaopen(filteredImage, processInfo.PixelFilter);
    filteredImage = imfill(filteredImage,4,"holes");
    
    % Step 4 % if poor contrast. Uncomment. Bridges gaps in drop outline.
    %filteredImage = bwmorph(filteredImage,'bridge',2);
    %filteredImage = imfill(filteredImage,4,"holes");
    %filteredImage = bwmorph(filteredImage,'spur');

end

function  makeImage(t,pathname, fileName , currFrame,natural)
    % Save the original frame as .png files in the 'images' directory
    if natural==1
        opBaseFileName = strcat('images/natural/',fileName(1:end-4),sprintf('Frame_%3.3d.png', t));
    elseif natural==2
        opBaseFileName = strcat('images/',fileName(1:end-4),'-centerimg.png');
    else
        opBaseFileName = strcat('images/filter/',fileName(1:end-4),sprintf('Frame_%3.3d.png', t));
    end
    opFullFileName = fullfile(pathname, opBaseFileName);
    imwrite(currFrame, opFullFileName, 'png');

end

function outputVideo=makeVid(video,identificationString)
    % Define the file name and format of the output video
    outputVideoFile = strcat('images/',identificationString,video.fileName);
    opvidname = fullfile(video.pathname, outputVideoFile);

    % Create a VideoWriter object to write the video
    outputVideo = VideoWriter(opvidname, 'MPEG-4');%
    outputVideo.Quality = 100;  % Set the frame rate of the output video (adjust as needed)

    outputVideo.FrameRate = video.VideoReader.FrameRate;  % Set the frame rate of the output video (adjust as needed)

    % Open the VideoWriter for writing
    open(outputVideo);

end


%% Check Functions

function is_Match(value1,value2)
    if value1 ~= value2
        error("ERROR; Values Do not Match")
    end
end

function lvl = levelmaxn(mat,n)
    
    B = reshape(mat,[],1);
    maxB = max(B);
    for i = 1:length(B)
        if sum(B>maxB-i) >= n
            lvl = maxB-i;
            return
        end
    end

end
