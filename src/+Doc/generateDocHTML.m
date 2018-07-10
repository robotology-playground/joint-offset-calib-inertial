% Generate the documentation using M2HTML.
% 
% M2HTML documentation system overview and tutorial:
% https://www.artefact.tk/software/matlab/m2html/
% 

m2html('mfiles','src', 'htmldir','doc', 'recursive','on', 'global','on','template','frame', 'index','menu','graph','on');
