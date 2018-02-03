%恢复校准以后扭曲的图像
load('./camera_cal/cameraParams');

AFig = imread('calibration2.jpg');
figure();
imshow(AFig);
[FixedFig, ~] = undistortImage(AFig, cameraParams, 'OutputView', 'same');
figure();
imshow(FixedFig);