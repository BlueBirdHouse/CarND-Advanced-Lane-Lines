%% 视频文件读取和播放
v = VideoReader('project_video.mp4');

OutVideo = VideoWriter('OutPut.mp4','MPEG-4');
OutVideo.open();
%设置视频播放的起始时间
%v.CurrentTime = 39;
v.CurrentTime = 0;

%% 读入相机校准参数
load('./camera_cal/cameraParams');
%相机内部参数结构体
camIntrinsics = cameraIntrinsics(cameraParams.FocalLength, cameraParams.PrincipalPoint, cameraParams.ImageSize);
%相机距离地面高度（估计）。因为一类车的车头高度1.3，加上摄像头加上架子基本上就是这个数值。
height = 1.4;
sensor = monoCamera(camIntrinsics, height);

%% 生成播放窗口
hf = figure;
hf.NextPlot = 'replaceChildren';

%% 初始化找到的交通标志线
last_Boundaries = [];

%% 处理逻辑
while hasFrame(v)
    AFrame = readFrame(v);
    %% 执行相机校准功能
    [AFrame, ~] = undistortImage(AFrame, cameraParams, 'OutputView', 'same');
    AFrame_Vehicle = AFrame;
    
    %% 执行透视图变换
    distAheadOfSensor = 25; % in meters, as previously specified in monoCamera height input
    spaceToOneSide    = 2.5;  % all other distance quantities are also in meters
    bottomOffset      = 5.5;
    outView   = [bottomOffset, distAheadOfSensor, -spaceToOneSide, spaceToOneSide]; % [xmin, xmax, ymin, ymax]
    imageSize = [NaN, 865]; % output image width in pixels; height is chosen automatically to preserve units per pixel ratio
    birdsEyeConfig = birdsEyeView(sensor, outView, imageSize);
    
    AFrame = transformImage(birdsEyeConfig, AFrame);
    %备份俯视图原数据
    birdsEye_RGB = AFrame;
    
    %% 执行图像亮度调整和对比度增强
    %illuminan = illumpca(AFrame, 0.1);
    %AFrame = chromadapt(AFrame, illuminan, 'ColorSpace', 'linear-rgb');
    %AFrame = localcontrast(AFrame);
     
    %% 标记属于交通标志线的像素
    %使用均值和方差阈值+EM算法的一种方法来提取交通标志线
     AFrame = rgb2gray(AFrame);
     %这里可以对交通标志线的识别方位可以做重新微调
     vehicleROI = outView;
     laneSensitivity = 0.25;
     approxLaneMarkerWidthVehicle = 0.25; % 25 centimeters
     AFrame = segmentLaneMarkerRidge(AFrame, birdsEyeConfig, approxLaneMarkerWidthVehicle,...
     'ROI', vehicleROI, 'Sensitivity', laneSensitivity);
     
     %% 初始化交通线建模过程
     %转换到车体坐标系下，并假设路面是平整的。
     [imageX, imageY] = find(AFrame);
     %注意，这里X、Y坐标一定是反着的。因为图像坐标和矩阵坐标之间的关系。
     xyBoundaryPoints = birdsEyeConfig.imageToVehicle([imageY, imageX]);

     %交通标志线是 25 centimeters宽
     approxLaneMarkerWidth = 0.25;
     [boundaries] = find_Lines(xyBoundaryPoints,approxLaneMarkerWidth,birdsEyeConfig);
     
     %% 删选找到的边界线
     xVehiclePoints = bottomOffset:distAheadOfSensor;
     %初始化上一次交通标志线
     if((isempty(last_Boundaries) == true)&&(size(boundaries,2) == 2)&&(boundarSimilarity(boundaries(1),boundaries(2),xVehiclePoints)))
         last_Boundaries = boundaries;
     end
     
     boundaries = laneBoundaryFilter2(boundaries,xVehiclePoints);
     
     if(isempty(last_Boundaries) == false)
        [boundaries] = laneBoundaryFilter1(last_Boundaries,boundaries,xVehiclePoints);
        last_Boundaries = boundaries;
     end    
     
    %% 在图上绘制找到的标志线
    AFrame_Vehicle = insertLaneBoundary(AFrame_Vehicle,boundaries,sensor,xVehiclePoints,'Color','Red');
    
    %% 输出交通标志线的位置
    boundariesTests = cell(2,1);
    boundariesTests{1} = num2str(boundaries(1).computeBoundaryModel(0));
    if size(boundaries,2) > 1
        boundariesTests{2} = num2str(boundaries(2).computeBoundaryModel(0));
    else
        boundariesTests{2} = '  ';
    end
    AFrame_Vehicle = insertText(AFrame_Vehicle,[5 5],'边线距离摄像头正下方：','Font','方正兰亭超细黑简体','TextColor','black','BoxColor','white');
    AFrame_Vehicle = insertText(AFrame_Vehicle,[150 5 ; 150 30],boundariesTests,'Font','方正兰亭超细黑简体','TextColor','black','BoxColor','white');

    %% 执行视频显示屏录象
    imshow(AFrame_Vehicle,'InitialMagnification','fit');
    hf.Name = num2str(v.CurrentTime);
    OutVideo.writeVideo(AFrame_Vehicle);
    drawnow;
     if(v.CurrentTime > inf)
         OutVideo.close();
         break;
     end
end
OutVideo.close();

%% 函数定义区
function [Similarity] = boundarSimilarity(BoundaryA,BoundaryB,xVehiclePoints)
    %计算A、B两个分界线之间的相似度。
    yVehiclePointsA = BoundaryA.computeBoundaryModel(xVehiclePoints);
    yVehiclePointsB = BoundaryB.computeBoundaryModel(xVehiclePoints);
    Similarity = norm(yVehiclePointsA - yVehiclePointsB,2);
end

function [boundaries] = laneBoundaryFilter1(last_Boundaries,this_boundaries,xVehiclePoints)
%% 通过在当前找到的边界中寻找与上一次找到边界最相似的边界作为新的边界。
% 如果当前只找到一个边界，则沿用上一次找到的边界。

    boundaries = last_Boundaries;
    
    Counter = size(this_boundaries,2);
    
    if(Counter == 0)
        return;
    end
    
    if(Counter == 1)
        similarityMatrix = zeros(2,1);
        similarityMatrix(1,1) = boundarSimilarity(last_Boundaries(1),this_boundaries(1),xVehiclePoints);
        similarityMatrix(2,1) = boundarSimilarity(last_Boundaries(2),this_boundaries(1),xVehiclePoints);
        [~,I] = sort(similarityMatrix(:));
        [I_row, ~] = ind2sub(size(similarityMatrix),I(1));
        boundaries(I_row) = this_boundaries(1);
        return;
    end
    
    similarityMatrix = zeros(2,Counter);
    %填充相似矩阵
    for i = 1:Counter
        similarityMatrix(1,i) = boundarSimilarity(last_Boundaries(1),this_boundaries(i),xVehiclePoints);
        similarityMatrix(2,i) = boundarSimilarity(last_Boundaries(2),this_boundaries(i),xVehiclePoints);
    end
    [~,I] = sort(similarityMatrix(1,:));
    boundaries(1) = this_boundaries(I(1));
    
    [~,I] = sort(similarityMatrix(2,:));
    boundaries(2) = this_boundaries(I(1));
end

function [curvature] = laneBoundaryCurvature(Aboundary,xVehiclePoints)
    A = Aboundary.Parameters(1);
    B = Aboundary.Parameters(2);
    curvature = ((2*A*xVehiclePoints + B).^2+1).^(3/2)/abs(2*A);
end

function [boundaries] = laneBoundaryFilter2(this_boundaries,xVehiclePoints)
    %% 利用曲率半径屏蔽掉不合理的边界线
    boundaries = [];
    Counter = size(this_boundaries,2);
    for i = 1:Counter
        minCurvature = min(laneBoundaryCurvature(this_boundaries(i),xVehiclePoints));
        if(minCurvature > 100)
            boundaries = [boundaries this_boundaries(i)];
        end
    end
end
