%% ��Ƶ�ļ���ȡ�Ͳ���
v = VideoReader('project_video.mp4');

OutVideo = VideoWriter('OutPut.mp4','MPEG-4');
OutVideo.open();
%������Ƶ���ŵ���ʼʱ��
%v.CurrentTime = 39;
v.CurrentTime = 0;

%% �������У׼����
load('./camera_cal/cameraParams');
%����ڲ������ṹ��
camIntrinsics = cameraIntrinsics(cameraParams.FocalLength, cameraParams.PrincipalPoint, cameraParams.ImageSize);
%����������߶ȣ����ƣ�����Ϊһ�೵�ĳ�ͷ�߶�1.3����������ͷ���ϼ��ӻ����Ͼ��������ֵ��
height = 1.4;
sensor = monoCamera(camIntrinsics, height);

%% ���ɲ��Ŵ���
hf = figure;
hf.NextPlot = 'replaceChildren';

%% ��ʼ���ҵ��Ľ�ͨ��־��
last_Boundaries = [];

%% �����߼�
while hasFrame(v)
    AFrame = readFrame(v);
    %% ִ�����У׼����
    [AFrame, ~] = undistortImage(AFrame, cameraParams, 'OutputView', 'same');
    AFrame_Vehicle = AFrame;
    
    %% ִ��͸��ͼ�任
    distAheadOfSensor = 25; % in meters, as previously specified in monoCamera height input
    spaceToOneSide    = 2.5;  % all other distance quantities are also in meters
    bottomOffset      = 5.5;
    outView   = [bottomOffset, distAheadOfSensor, -spaceToOneSide, spaceToOneSide]; % [xmin, xmax, ymin, ymax]
    imageSize = [NaN, 865]; % output image width in pixels; height is chosen automatically to preserve units per pixel ratio
    birdsEyeConfig = birdsEyeView(sensor, outView, imageSize);
    
    AFrame = transformImage(birdsEyeConfig, AFrame);
    %���ݸ���ͼԭ����
    birdsEye_RGB = AFrame;
    
    %% ִ��ͼ�����ȵ����ͶԱȶ���ǿ
    %illuminan = illumpca(AFrame, 0.1);
    %AFrame = chromadapt(AFrame, illuminan, 'ColorSpace', 'linear-rgb');
    %AFrame = localcontrast(AFrame);
     
    %% ������ڽ�ͨ��־�ߵ�����
    %ʹ�þ�ֵ�ͷ�����ֵ+EM�㷨��һ�ַ�������ȡ��ͨ��־��
     AFrame = rgb2gray(AFrame);
     %������ԶԽ�ͨ��־�ߵ�ʶ��λ����������΢��
     vehicleROI = outView;
     laneSensitivity = 0.25;
     approxLaneMarkerWidthVehicle = 0.25; % 25 centimeters
     AFrame = segmentLaneMarkerRidge(AFrame, birdsEyeConfig, approxLaneMarkerWidthVehicle,...
     'ROI', vehicleROI, 'Sensitivity', laneSensitivity);
     
     %% ��ʼ����ͨ�߽�ģ����
     %ת������������ϵ�£�������·����ƽ���ġ�
     [imageX, imageY] = find(AFrame);
     %ע�⣬����X��Y����һ���Ƿ��ŵġ���Ϊͼ������;�������֮��Ĺ�ϵ��
     xyBoundaryPoints = birdsEyeConfig.imageToVehicle([imageY, imageX]);

     %��ͨ��־���� 25 centimeters��
     approxLaneMarkerWidth = 0.25;
     [boundaries] = find_Lines(xyBoundaryPoints,approxLaneMarkerWidth,birdsEyeConfig);
     
     %% ɾѡ�ҵ��ı߽���
     xVehiclePoints = bottomOffset:distAheadOfSensor;
     %��ʼ����һ�ν�ͨ��־��
     if((isempty(last_Boundaries) == true)&&(size(boundaries,2) == 2)&&(boundarSimilarity(boundaries(1),boundaries(2),xVehiclePoints)))
         last_Boundaries = boundaries;
     end
     
     boundaries = laneBoundaryFilter2(boundaries,xVehiclePoints);
     
     if(isempty(last_Boundaries) == false)
        [boundaries] = laneBoundaryFilter1(last_Boundaries,boundaries,xVehiclePoints);
        last_Boundaries = boundaries;
     end    
     
    %% ��ͼ�ϻ����ҵ��ı�־��
    AFrame_Vehicle = insertLaneBoundary(AFrame_Vehicle,boundaries,sensor,xVehiclePoints,'Color','Red');
    
    %% �����ͨ��־�ߵ�λ��
    boundariesTests = cell(2,1);
    boundariesTests{1} = num2str(boundaries(1).computeBoundaryModel(0));
    if size(boundaries,2) > 1
        boundariesTests{2} = num2str(boundaries(2).computeBoundaryModel(0));
    else
        boundariesTests{2} = '  ';
    end
    AFrame_Vehicle = insertText(AFrame_Vehicle,[5 5],'���߾�������ͷ���·���','Font','������ͤ��ϸ�ڼ���','TextColor','black','BoxColor','white');
    AFrame_Vehicle = insertText(AFrame_Vehicle,[150 5 ; 150 30],boundariesTests,'Font','������ͤ��ϸ�ڼ���','TextColor','black','BoxColor','white');

    %% ִ����Ƶ��ʾ��¼��
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

%% ����������
function [Similarity] = boundarSimilarity(BoundaryA,BoundaryB,xVehiclePoints)
    %����A��B�����ֽ���֮������ƶȡ�
    yVehiclePointsA = BoundaryA.computeBoundaryModel(xVehiclePoints);
    yVehiclePointsB = BoundaryB.computeBoundaryModel(xVehiclePoints);
    Similarity = norm(yVehiclePointsA - yVehiclePointsB,2);
end

function [boundaries] = laneBoundaryFilter1(last_Boundaries,this_boundaries,xVehiclePoints)
%% ͨ���ڵ�ǰ�ҵ��ı߽���Ѱ������һ���ҵ��߽������Ƶı߽���Ϊ�µı߽硣
% �����ǰֻ�ҵ�һ���߽磬��������һ���ҵ��ı߽硣

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
    %������ƾ���
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
    %% �������ʰ뾶���ε�������ı߽���
    boundaries = [];
    Counter = size(this_boundaries,2);
    for i = 1:Counter
        minCurvature = min(laneBoundaryCurvature(this_boundaries(i),xVehiclePoints));
        if(minCurvature > 100)
            boundaries = [boundaries this_boundaries(i)];
        end
    end
end
