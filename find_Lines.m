function [boundaries] = find_Lines(BoundaryPoints,approxLaneMarkerWidth,birdsEyeConfig)
    %% 这是一个改造过的RANSAC交通标志线寻找函数，在循环寻找一段时间后，会自动停下来
    % 这个函数工作在公制坐标系下，而不是像素坐标系下。
    
    % BoundaryPoints是机车坐标系下标志线对应的点
    % approxLaneMarkerWidthPixel 是标志线对应的宽度

    %birdsEyeConfig测试用代码
    
    %关掉警告信息
    wstate      = warning('off','vision:ransac:maxTrialsReached');
    resetWState = onCleanup(@()warning(wstate));  
    
    %最大需要寻找的边线个数
    maxNumBoundaries = 20;
    %初始化结果数据集
    boundaries = repmat(SnakeWindow.aBoundary.empty(),[1,maxNumBoundaries]);
    %使用二次函数拟合
    Model_N = 2;
    %退出条件为一定数量的点在寻找过程中被移除
    
    StopNum = size(BoundaryPoints,1)*(1/4);
    numLanes = 1;
    
    %% 开始寻找
    for candidateIdx=1:maxNumBoundaries
        %循环寻找所有的边线
        [~, inlierIdx] = fitPolynomialRANSAC(BoundaryPoints, Model_N, approxLaneMarkerWidth,'Confidence', 99, 'MaxNumTrials', 500);
        
        if any(inlierIdx)
            inlierXY = BoundaryPoints(inlierIdx,:);
            params = polyfit(inlierXY(:,1),inlierXY(:,2),Model_N);
            boundaries(numLanes) = SnakeWindow.aBoundary(params);
            boundaries(numLanes).boundaryPoints = inlierXY;
            numLanes = numLanes + 1;
        end
        
        %清除已经找到的边界
        BoundaryPoints = BoundaryPoints(~inlierIdx, :);
        %showPoints(BoundaryPoints,birdsEyeConfig);
        
        %% 退出条件
        if(size(BoundaryPoints,1) < StopNum)
            break;
        end
    end
    
    
end

%% 调试用的函数
function showPoints(BoundaryPoints,birdsEyeConfig)
    %% 用于显示BoundaryPoints.
    %BoundaryPoints是车体坐标系上的点，本来不能用图像显示。这里将其还原为图像。
    imagePoints = round(birdsEyeConfig.vehicleToImage(BoundaryPoints));
    AFig = false(birdsEyeConfig.ImageSize);
    AFig(sub2ind(size(AFig), imagePoints(:,2), imagePoints(:,1))) = true;
    imshow(AFig,'InitialMagnification','fit');
end