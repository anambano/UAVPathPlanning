function test = testMap(test,obsMat)
% Reset function for multi agent area coverage example

    isvalid = false;
    while ~isvalid
        rows = [2 11 3];
        cols = [2 4 10];
        s0 = [rows' cols'];
        if all(~all(s0(1,:) == obsMat,2)) && all(~all(s0(2,:) == obsMat,2)) && all(~all(s0(3,:) == obsMat,2))
            isvalid = true;
        end
    end
    test = setVariable(test, 's0', s0);
end