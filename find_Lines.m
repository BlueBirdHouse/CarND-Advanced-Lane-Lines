function [boundaries] = find_Lines(BoundaryPoints,approxLaneMarkerWidth,birdsEyeConfig)
    %% ����һ���������RANSAC��ͨ��־��Ѱ�Һ�������ѭ��Ѱ��һ��ʱ��󣬻��Զ�ͣ����
    % ������������ڹ�������ϵ�£���������������ϵ�¡�
    
    % BoundaryPoints�ǻ�������ϵ�±�־�߶�Ӧ�ĵ�
    % approxLaneMarkerWidthPixel �Ǳ�־�߶�Ӧ�Ŀ��

    %birdsEyeConfig�����ô���
    
    %�ص�������Ϣ
    wstate      = warning('off','vision:ransac:maxTrialsReached');
    resetWState = onCleanup(@()warning(wstate));  
    
    %�����ҪѰ�ҵı��߸���
    maxNumBoundaries = 20;
    %��ʼ��������ݼ�
    boundaries = repmat(SnakeWindow.aBoundary.empty(),[1,maxNumBoundaries]);
    %ʹ�ö��κ������
    Model_N = 2;
    %�˳�����Ϊһ�������ĵ���Ѱ�ҹ����б��Ƴ�
    
    StopNum = size(BoundaryPoints,1)*(1/4);
    numLanes = 1;
    
    %% ��ʼѰ��
    for candidateIdx=1:maxNumBoundaries
        %ѭ��Ѱ�����еı���
        [~, inlierIdx] = fitPolynomialRANSAC(BoundaryPoints, Model_N, approxLaneMarkerWidth,'Confidence', 99, 'MaxNumTrials', 500);
        
        if any(inlierIdx)
            inlierXY = BoundaryPoints(inlierIdx,:);
            params = polyfit(inlierXY(:,1),inlierXY(:,2),Model_N);
            boundaries(numLanes) = SnakeWindow.aBoundary(params);
            boundaries(numLanes).boundaryPoints = inlierXY;
            numLanes = numLanes + 1;
        end
        
        %����Ѿ��ҵ��ı߽�
        BoundaryPoints = BoundaryPoints(~inlierIdx, :);
        %showPoints(BoundaryPoints,birdsEyeConfig);
        
        %% �˳�����
        if(size(BoundaryPoints,1) < StopNum)
            break;
        end
    end
    
    
end

%% �����õĺ���
function showPoints(BoundaryPoints,birdsEyeConfig)
    %% ������ʾBoundaryPoints.
    %BoundaryPoints�ǳ�������ϵ�ϵĵ㣬����������ͼ����ʾ�����ｫ�仹ԭΪͼ��
    imagePoints = round(birdsEyeConfig.vehicleToImage(BoundaryPoints));
    AFig = false(birdsEyeConfig.ImageSize);
    AFig(sub2ind(size(AFig), imagePoints(:,2), imagePoints(:,1))) = true;
    imshow(AFig,'InitialMagnification','fit');
end