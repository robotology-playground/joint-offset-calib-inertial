classdef JdesktopHandler
    %Wraps all java methods providing the Editor services
    %   Like getting the list of open files, saving/loading a session...
    
    properties
    end
    
    methods(Static)
        function [jEditor,jDesktop] = getEditor()
            try
                % Get the Editor handle (Matlab 7 or +)
                jDesktop = com.mathworks.mde.desk.MLDesktop.getInstance;
                jEditor = jDesktop.getGroupContainer('Editor').getTopLevelAncestor;
            catch
            end
        end
        
        function fileName = getCurrentActiveFile()
            try
                % Get the Editor
                [jEditor,~] = System.JdesktopHandler.getEditor();
                
                % Get the current active file
                title = jEditor.getTitle;
                fileName = char(title.replaceFirst('Editor - ',''));
            catch
            end
        end
        
        function [allEditorFilenames,openFiles,jEditor] = getAllEditorFilenames(varargin)
            
            try
                % Get the jDesktop
                if nargin>0 && ~isempty(varargin{1})
                    jDesktop = varargin{1};
                else
                    [jEditor,jDesktop] = System.JdesktopHandler.getEditor();
                end
                
                % Get the entire list of open file names
                openFiles = jDesktop.getWindowRegistry.getClosers.toArray.cell;
                allEditorFilenames = cellfun(@(c)c.getTitle.char,openFiles,'un',0);
            catch
            end
        end
        
        function saveSession(sessionName)
            [allEditorFilenames,~] = System.JdesktopHandler.getAllEditorFilenames();
            save([sessionName '.session'],'allEditorFilenames','-mat');
        end
        
        function saved_jDesktop = loadSession(sessionFileName)
            % Load saved list of open files
            load(sessionFileName,'-mat','allEditorFilenames');
            % Close all the files in the current editor window
            edhandle = com.mathworks.mlservices.MLEditorServices;
            editorApp=edhandle.getEditorApplication;
            editorApp.close;
            % Open restored list of files
            for file = allEditorFilenames(:)'
                open (cell2mat(file));
            end
        end
    end
    
end
