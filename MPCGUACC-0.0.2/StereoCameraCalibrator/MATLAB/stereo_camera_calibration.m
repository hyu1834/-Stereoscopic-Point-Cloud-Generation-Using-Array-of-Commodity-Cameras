clear all;

%checkboard square size in MM
squareSize = 32.74;

%camera baseline in MM
baseline = 140;
% baseline = 280;

%Image Resolution
resolution = '1080';

leftCamNum = 4;
rightCamNum = 3;

%base directory
calibrationPhotoPath = ['D:\Seafile\Graduate Research\Projects\Point Cloud Generation '...
                        'Using Arrays of Commodity Cameras\Data\Calibration\', sprintf('%d-%d\\', leftCamNum, rightCamNum)];
calibrationParamDict = 'D:\Seafile\Graduate Research\Projects\Point Cloud Generation Using Arrays of Commodity Cameras\PCGUACC-0.0.1\StereoCameraTools\CalibrationParams2\';
calibrationParamName = ['stereo_params_', resolution, sprintf('_%d-%d_opencv.xml', leftCamNum, rightCamNum)];
calibrationParamPath = [calibrationParamDict, calibrationParamName];

%set input imagefile path
leftCameraImageDict = [calibrationPhotoPath, sprintf('%d\\', leftCamNum)];
rightCameraImageDict = [calibrationPhotoPath, sprintf('%d\\', rightCamNum)];
% generate list of image path
imagePaths1 = imageSet(leftCameraImageDict).ImageLocation;
imagePaths2 = imageSet(rightCameraImageDict).ImageLocation;

%=========================================================================%
%                       Stereo Camera Calibration
%   Now perform stereo camera calibration by the given set of image from
%   left and right camera. 
%   List of image paths are stored in imagePaths1 (left) and imagePaths2
%   (right)
%   First detect checkerboard on image
[imagePoints, boardSize, cbFound] = detectCheckerboardPoints(imagePaths1, imagePaths2);
imagePaths1 = imagePaths1(cbFound);
imagePaths2 = imagePaths2(cbFound);

%   Then get the coordinate of the detected square
worldPoints = generateCheckerboardPoints(boardSize, squareSize);
% 
%   Then calibrate the stereo camera by the world points
[stereoParams, pairsUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints,...
                                            'NumRadialDistortionCoefficients', 3, 'WorldUnits', 'mm');
% 
% %Show the distribution
showReprojectionErrors(stereoParams);
% 
% % Finally output the calibrated params to the xml that fit opencv in XML format
stereo_camera_calibration_parameters_opencv(calibrationParamPath, stereoParams, baseline);
% 
%=========================================================================%