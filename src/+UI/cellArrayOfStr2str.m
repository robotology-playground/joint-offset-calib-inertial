function str = cellArrayOfStr2str( separator,aCellArrayOfStr )
%cellArrayOfStr2str Convert a cell array of vector chars into a single
%vector char

concatenatedStr = cellfun(...
    @(str) [str separator],...
    aCellArrayOfStr,...
    'UniformOutput',false);
str = strtrim(cell2mat(concatenatedStr));

end
