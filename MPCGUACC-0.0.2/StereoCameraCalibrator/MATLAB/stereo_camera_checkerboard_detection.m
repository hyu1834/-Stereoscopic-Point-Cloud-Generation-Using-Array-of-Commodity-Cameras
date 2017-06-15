clear all;

%checkboard square size in MM
squareSize = 32.74;

%camera baseline in MM
baseline = 140;

leftCamNum = 3;
rightCamNum = 1;

calibrationPhotoBaseDir = sprintf('D:\\Graduated_Research\\Projects\\StereoscopicPi\\Data\\5MP\\v5-1080\\%d-%d\\', leftCamNum, rightCamNum); 
calibrationParamDir = 'D:\Graduated_Research\Projects\StereoscopicPi\stereoscopicPi-0.0.1\calibration_params\';

%set input imagefile path
leftCameraImageDict = strcat(calibrationPhotoBaseDir, sprintf('%d_calibration', leftCamNum));
rightCameraImageDict = strcat(calibrationPhotoBaseDir, sprintf('%d_calibration', rightCamNum));
% generate list of image path
imagePaths1 = imageSet(leftCameraImageDict).ImageLocation;
imagePaths2 = imageSet(rightCameraImageDict).ImageLocation;

%=========================================================================%
%   Here we consider this as DEBUGGING USE ONLY
%	Try to locate all the pair that were able to locate checkerboard
%	individually
%   Delete those that are not able to detect, so that it will not affect
%   the result of the real calibration
validCheckerboardPic = false(1, length(imagePaths1));
for i = 1 : length(imagePaths1)
    [imagePoints, boardSize, pairsUsed] = ...
                detectCheckerboardPoints(imagePaths1{i}, imagePaths2{i});
    if(pairsUsed)
        validCheckerboardPic(i) = 1;
    else
        validCheckerboardPic(i) = 0;
    end
end

imagePaths1 = imagePaths1(validCheckerboardPic);
imagePaths2 = imagePaths2(validCheckerboardPic);
for i = 1 : length(imagePaths1)
   fprintf('%s\n', imagePaths1{i}(end-8: end)); 
end
%=========================================================================%