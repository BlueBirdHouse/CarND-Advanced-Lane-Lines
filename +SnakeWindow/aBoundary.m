classdef aBoundary < parabolicLaneBoundary
    %aBoundary 存储检查得到的边界信息
    %   
    
    properties
        boundaryPoints;
        %在做RANSAC过程中找到的内点。
        
        %isLeft = true;
        %标记交通标志位于建模抛物线的左侧还是右侧
        
    end
    
    methods
        function obj = aBoundary(parabolaParameters)
            %aBoundary 
            %   通过为内置的parabolicLaneBoundary增加功能来生成新类。
            obj = obj@parabolicLaneBoundary();
            obj.Parameters = parabolaParameters;
            %obj.isLeft = true;
        end

    end
end

