function string = toLatexInterpreterCompliant(string)
%toLatexInterpreterCompliant Allows to be displayed through a Latex interpreter as is.
%   This function allows to escape special characters like '_' that could be interpreted
%   by a Latex interpreter as operators (e.g. '_' defines the following character as a
%   subscript).

string = strrep(string,'_','\_');

end

