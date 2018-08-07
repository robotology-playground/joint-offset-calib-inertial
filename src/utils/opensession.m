function opensession(sessionFileName)
%This redefines the Open file handler for .session file types
%   (You can create your own OPENXXX functions to set up handlers 
%    for new file types.  open will call whatever OPENXXX function 
%    it finds on the path.)

System.JdesktopHandler.loadSession(sessionFileName);

end
