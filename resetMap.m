function in = resetMap(in,obsMat)
% Reset function for multi agent area coverage example

% Copyright 2020 The MathWorks, Inc.

    gridSize = [12 12];
    isvalid = false;
    while ~isvalid
        rows = randperm(gridSize(1),3);
        cols = randperm(gridSize(2),3);
        s0 = [rows' cols'];
        if all(~all(s0(1,:) == obsMat,2)) && all(~all(s0(2,:) == obsMat,2)) && all(~all(s0(3,:) == obsMat,2))
            isvalid = true;
        end
    end
    in = setVariable(in, 's0', s0);
end