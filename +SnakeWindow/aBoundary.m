classdef aBoundary < parabolicLaneBoundary
    %aBoundary �洢���õ��ı߽���Ϣ
    %   
    
    properties
        boundaryPoints;
        %����RANSAC�������ҵ����ڵ㡣
        
        %isLeft = true;
        %��ǽ�ͨ��־λ�ڽ�ģ�����ߵ���໹���Ҳ�
        
    end
    
    methods
        function obj = aBoundary(parabolaParameters)
            %aBoundary 
            %   ͨ��Ϊ���õ�parabolicLaneBoundary���ӹ������������ࡣ
            obj = obj@parabolicLaneBoundary();
            obj.Parameters = parabolaParameters;
            %obj.isLeft = true;
        end

    end
end

