clear;
close all;
clc;
vid = videoinput('pointgrey', 1, 'F7_Mono8_808x608_Mode0');
src = getselectedsource(vid);
vid.FramesPerTrigger = 1;
figure('Toolbar','none',...
       'Menubar', 'none',...
       'NumberTitle','Off',...
       'Name','My Preview Window','Position',[1921,1,1920,1080]);
vidRes = vid.VideoResolution;
nBands = vid.NumberOfBands;
hImage = image( zeros(vidRes(2), vidRes(1), nBands) );

preview(vid,hImage);
%I =  getsnapshot(vid);%rgb2gray(imread('Capture1.jpg'));