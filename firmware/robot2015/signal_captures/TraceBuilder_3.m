% Close any previous windows
close all

% The path where files are located
% dir_path = [getenv('HOMEDRIVE'), getenv('HOMEPATH'), '\Documents\GitHub\robocup-software\firmware\robot\cpu\data'];

% dir_path = [getenv('HOME'), filesep,'Documents',filesep,'GitHub',filesep,'robocup-software',filesep,'firmware',filesep,'robot',filesep,'cpu',filesep,'data'];
% addpath(dir_path);
dir_path = 'kicker';

% Sort the file names
files = dir(dir_path);
isWanted = ~strncmp('.', {files.name}, 1) & ~[files.isdir];

% Cell array index that has the names
data = {files(isWanted').name};

% Read in the data
for i=1:size(data,2)
%     filename = [dir_path, '\', data{1,i}];
    filename = [dir_path, filesep, data{1,i}];
    [val1, val2] = textread(filename, '%f %u');
    data{2,i} = val1;
    data{3,i} = val2;
end

% no need keeping this stuff
clearvars('files', 'filename', 'i', 'isWanted', 'val1', 'val2');

% Now generate the figures - return array that contains handles for all
% figure windows
figs = RJ_plot_data(data, 'Robot Data');

% Move to the directory where the data is
cd(dir_path);

% Create a directory to store the output
dir_name = 'MATLAB Exports';
if exist(dir_name) == 7
    rmdir(dir_name, 's');
end
mkdir(dir_name);
% old_path = cd([dir_path, filesep, dir_name]);
cd(dir_name);

% Save the figures
for j=1:size(figs,1)
    nbr = sprintf(' %02u', j);
    saveas(figs(j), ['Data Results', nbr], 'pdf');
end

% Cleanup and go back up one directory
% cd(old_path);
cd ../..;

% Open the first exported file to indicate everything is finished
open([dir_path,filesep,dir_name,filesep,'Data Results 01.pdf']);

clearvars('i' ,'j', 'figs', 'nbr', 'old_path', 'dir_name','data','dir_path');