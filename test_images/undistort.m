%�ָ�У׼�Ժ�Ť����ͼ��
load('../camera_cal/cameraParams');

AFig = imread('straight_lines1.jpg');
figure();
imshow(AFig);
[FixedFig, ~] = undistortImage(AFig, cameraParams, 'OutputView', 'same');
figure();
imshow(FixedFig);