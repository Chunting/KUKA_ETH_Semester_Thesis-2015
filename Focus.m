function [ FM ] = Focus(i,vid)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
%vid = videoinput('pointgrey', 1, 'F7_Mono8_808x608_Mode0');
%src = getselectedsource(vid);
%vid.FramesPerTrigger = 1;
%preview(vid);
I =  getsnapshot(vid);%rgb2gray(imread('Capture1.jpg'));
%imshow(I);
FM = fmeasure(I,'GDER',[]);
if(i==1)
imwrite(I,'Focus.png');
%figure(1)
%imshow(I);
end
if(i==2)
imwrite(I,'Focus1.png');
%figure(2)
%imshow(I);
end
if(i==3)
imwrite(I,'Focus2.png');
%figure(3)
%imshow(I);
end
disp(FM);
end

