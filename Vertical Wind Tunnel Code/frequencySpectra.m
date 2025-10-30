%% Perimeter and Frequency Spectra
% This code relies on the Cache from ImageAnalysisHeatmap.m. Run that file
% before running this one.
% This code produces a time-radii plot and a saves a perimeter video of
% drop motion centered around the centroid.
% most errors occur due to improper drop binariazation (adjust in
% ImageAnalysisHeatmap.m).
%
% Editable lines: 
% line 24 (number of degrees in each bin of the FFT | Default: 5 degrees)
% line 27 (Sampling rate of the image sequence | Default: 2000 Hz)
%
% Syed Jaffar Raza and Johsua Watkins 06/27/2025


clear; close all; clc
format long
addpath("Functions")
load("Cache\rawData.mat")
close all;
%% Editable lines

% Number of Degrees in each bin
numDegperBin = 5; % Degrees

% Frames per second / Sampling frequency (for FFTs)
fftfps = 2000; %fps


%%


% Retrieve binarized video
cam1 = VideoReader(strcat(outputVideo1.Path,"/",outputVideo1.Filename));

% Set up perimeter video
outputVideoNew=makeVid(cam1Video,'Perimeter_');
vp1=vision.VideoPlayer('Position',[100,400,800,400]);

% Set up perimeter canvas
TempCan1 = zeros([heatMapDimention,heatMapDimention]);

% Find center of video
heatMapCentroid = [heatMapDimention/2,heatMapDimention/2];

% Set up array for radii measurements
AllocationFactor = 1.75; % Allocate more spots for potential data
relativeX1 =  zeros([length(camera1Stats.centroid) round(AllocationFactor*max(camera1Stats.perimeter(:,2)))]);
relativeY1 =  zeros([length(camera1Stats.centroid) round(AllocationFactor*max(camera1Stats.perimeter(:,2)))]);

% Set frame index
framIdx = 1;
while hasFrame(cam1)

    % Read Frame
    g1 = rgb2gray(readFrame(cam1));

    % Manually binarize
    g1(g1 < 200) = 0; %binarization theshold set
    g1 = logical(g1);

    % Region Prop analysis
    [stats_fin1,~] = IMGanalyze(g1);
    canCropX = round([heatMapCentroid(1)-(stats_fin1.BoundingBox(3)./2) , heatMapCentroid(1)+(stats_fin1.BoundingBox(3)./2)]);
    canCropY = round([heatMapCentroid(2)-(stats_fin1.BoundingBox(4)./2) , heatMapCentroid(2)+(stats_fin1.BoundingBox(4)./2)]);
    g1c = g1(round(stats_fin1.BoundingBox(2)):round((stats_fin1.BoundingBox(2)+stats_fin1.BoundingBox(4))), round(stats_fin1.BoundingBox(1)):round((stats_fin1.BoundingBox(1)+stats_fin1.BoundingBox(3))));
    TempCan1(canCropY(1):canCropY(2), canCropX(1):canCropX(2)) = TempCan1(canCropY(1):canCropY(2), canCropX(1):canCropX(2))+g1c;


    % Leave only drop perimeter
    TempCan1 = bwmorph(TempCan1,"remove");

    % Analyze drop perimeter
    [Useless,~] = IMGanalyze(TempCan1);
    periPxlList1 = Useless.PixelList;

    % Play and write video
    vp1.step(TempCan1)
    writeVideo(outputVideoNew, mat2gray(TempCan1,[0,1]));

    % Make list of pixels at perimeter
    zeroEl = zeros([1 length(relativeX1(framIdx,:))-length(periPxlList1)]);
    relativeX1(framIdx,:) = [(periPxlList1(:,1) - heatMapCentroid(1))' zeroEl];
    relativeY1(framIdx,:) = [(periPxlList1(:,2) - heatMapCentroid(1))' zeroEl];


    % Update for next itteration
    framIdx = framIdx+1;
    TempCan1(:,:) = 0;

end

% Close the VideoWriter
close(outputVideoNew);
close all

%% Edge Radii Analysis
edgeRadii1 = (sqrt(relativeX1.^2 + relativeY1.^2))-(mean(camera1Stats.diam(:,2))/2);
theta1 = atan2(relativeY1,relativeX1);


% Big Time Mat
timeMat1 = (zeros([height(edgeRadii1) width(edgeRadii1)]) + (1:height(edgeRadii1))')/camera1Stats.frameRate;

%
nullValue1 = -(mean(camera1Stats.diam(:,2))/2);
viewIdx1 = edgeRadii1~=-(mean(camera1Stats.diam(:,2))/2);
CS1 = figure;
scatter(theta1(viewIdx1),timeMat1(viewIdx1),10,edgeRadii1(viewIdx1),"filled");
xlim([-pi,pi])
ylim([0,1])
shading interp
xlabel("Theta [rad]")
ylabel("Time [ms]")
zlabel("Radius Differnce [mm]")
colormap(jet(4096));
colorbar;

% Number of bins
binWidth = numDegperBin; % Degrees
bins = 360/binWidth;
binWidth = 2*pi/bins;

RadiiThroughTime = zeros([length(edgeRadii1) bins])-50;% Row: Theta Col: Time

NumActualBins = zeros([length(edgeRadii1) 1]);
for i=1:length(edgeRadii1)
    % Current Time
    currFrame = edgeRadii1(i,:);
    currFrame = currFrame(currFrame~=nullValue1);
    currThetas = theta1(i,:);
    currThetas = currThetas(currFrame~=nullValue1);
    binIds = discretize(currThetas,(-pi):(2*pi/bins):(pi));
    NumActualBins(i,1) = length(unique(discretize(currThetas,(-pi):(2*pi/bins):(pi))));
    clear j
    for j=1:NumActualBins(i,1)
        currBinId =  binIds==j;
        RadiiThroughTime(i,j) = mean(currFrame(currBinId));
    end

end
% FFT
Fs = fftfps;% Sampling frequency
T = 1/Fs;% Sampling period
L = length(edgeRadii1);% Length of signal

Y = fft(RadiiThroughTime);
P2 = abs(Y/L);
P1 = P2(1:L/2+1,:); % single side, include Niq
P1(2:end-1,:) = 2*P1(2:end-1,:);
f = Fs/L*(0:(L/2));

edges = (-pi):(2*pi/bins):(pi);

meanTheta = repmat(edges(1:end-1)+((2*pi/bins)/2),length(f),1);
frep = repmat(f,bins,1)';
%%
CP2Z = figure;
pcolor(meanTheta,frep,P1)
shading interp
colormap(jet(4096))
colorbar;
%title("Camera 1")
ylim([1,100])
xlabel("$\theta \ \mathrm{[rad]}$","Interpreter","latex")
ylabel("$F \ \mathrm{[Hz]}$","Interpreter","latex")
zlabel("$Single Sided Amplitude (mm)$","Interpreter","latex")
set(gcf,'color','w');
set(gca, 'FontName', 'Times New Roman'); set(gca, 'FontSize', 20);







%% Functions

function outputVideo=makeVid(video,identificationString)
% Define the file name and format of the output video
outputVideoFile = strcat('images/',identificationString,video.fileName);
opvidname = fullfile(video.pathname, outputVideoFile);

% Create a VideoWriter object to write the video
outputVideo = VideoWriter(opvidname, 'MPEG-4');%
outputVideo.Quality = 100;  % Set the frame rate of the output video (adjust as needed)

outputVideo.FrameRate = 30;  % Set the frame rate of the output video (adjust as needed)

% Open the VideoWriter for writing
open(outputVideo);

end

function  [stats_fin,filt]=IMGanalyze(filteredImage)
filt = double(filteredImage);
stats= regionprops(filteredImage, 'Area', 'Centroid', 'MajorAxisLength', 'MinorAxisLength','BoundingBox','Orientation','Perimeter','EquivDiameter','PixelList');

if isempty(stats)
    stats_fin.Nullity = true;
else
    stats_fin = stats(1);

    % Area Filter
    for i = 1:length(stats)
        if stats(i).Area > stats_fin.Area
            stats_fin = stats(i);
        end
    end
    stats_fin.Nullity = false;
end

end

function  [stats_fin,filt]=IMGanalyze2(filteredImage)
filt = double(filteredImage);
stats= regionprops(filteredImage, 'Area', 'Centroid', 'MajorAxisLength', 'MinorAxisLength','BoundingBox','Orientation','Perimeter','EquivDiameter','PixelList');

if isempty(stats)
    stats_fin.Nullity = true;
else
    stats_fin = stats(1);
    % Area Filter
    for i = 1:length(stats)
        if stats(i).Area > stats_fin.Area
            stats_fin = stats(i);
        end
    end

    stats_fin.Nullity = false;
end

end
