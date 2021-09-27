function res = calPtDisToPolygon(pt, polygonVertices)
    % colsed polygon add the first vertex to end
    polygonVertices = [polygonVertices polygonVertices(:, 1)]; 
    verticesNum = size(polygonVertices, 2);
    res = realmax;
    sampleNum = 50;
    for i = 1 : verticesNum - 1
        startVertex = polygonVertices(:, i);
        endVertex = polygonVertices(:, i + 1);
        sideVec = endVertex - startVertex;
        sideMinDis = realmax;
        for j = 0 : sampleNum
            sample = startVertex + j / sampleNum * sideVec;
            disVec = pt - sample;
            curDisSqr = disVec' * disVec;
            if curDisSqr < sideMinDis
                sideMinDis = curDisSqr;
            end
        end
        
        if sideMinDis < res
            res = sideMinDis;
        end
    end
    res  = sqrt(res);
end