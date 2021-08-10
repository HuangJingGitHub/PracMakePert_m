function res = closestDistoPathFunc(feedbackPt, path)
    minDisSqr = 1e6;
    for i = 1 : size(path, 2)
        curDisVec = feedbackPt - path(:, i);
        curDisSqr = curDisVec' * curDisVec;
        if curDisSqr < minDisSqr
            minDisSqr = curDisSqr;
        end
    end
    res = sqrt(minDisSqr);
end