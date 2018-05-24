%move files from one directory to another directory

dataRoot = '/home/fujenchu/projects/robotArm/catkin_ws/src/simData/warehouse/models';
dataDir= dir(dataRoot);

for idx_folder = 3:length(dataDir)
    FolderName = dataDir(idx_folder).name;
    fileRoot = [dataRoot '/' FolderName '/' FolderName '_origin.dae'];
    [pathstr,filename] = fileparts(fileRoot);
    mkdir([dataRoot '/' FolderName '/mesh']);
    copyfile([dataRoot '/' FolderName '/*.dae'], [dataRoot '/' FolderName '/mesh']); 
    mkdir([dataRoot '/' FolderName '/sdf']);
    fsdf = fopen([dataRoot '/' FolderName '/sdf/description.sdf'],'wt');
    fprintf(fsdf, '<?xml version=''1.0''?>\n'); %write ' to a file needs '' in the string
    fprintf(fsdf, '<sdf version="1.4">\n');
    fprintf(fsdf, ['<model name="' filename '">\n']);
    fprintf(fsdf, '  <static>true</static>\n');
    fprintf(fsdf, '    <link name="link">\n');
    fprintf(fsdf, '      <collision name="collision">\n');
    fprintf(fsdf, '        <surface>\n');
    fprintf(fsdf, '          <contact>\n');
    fprintf(fsdf, '            <collide_without_contact>true</collide_without_contact>\n');
    fprintf(fsdf, '          </contact>\n');
    fprintf(fsdf, '        </surface>\n');
    fprintf(fsdf, '        <geometry>\n');
    fprintf(fsdf, '          <box>\n');
    fprintf(fsdf, '            <size>0.1 0.1 0.1</size>\n');
    fprintf(fsdf, '          </box>\n');
    fprintf(fsdf, '        </geometry>\n');
    fprintf(fsdf, '        </collision>\n');
    fprintf(fsdf, '      <visual name="visual">\n');
    fprintf(fsdf, '        <geometry>\n');
    fprintf(fsdf, '          <mesh>\n');
    fprintf(fsdf, ['            <uri>model://' FolderName '/mesh/' filename '.dae</uri>\n']);
    fprintf(fsdf, '            <scale>1 1 1</scale>\n');
    fprintf(fsdf, '          </mesh>\n');
    fprintf(fsdf, '        </geometry>\n');
    fprintf(fsdf, '      </visual>\n');
    fprintf(fsdf, '    </link>\n');
    fprintf(fsdf, '  </model>\n');
    fprintf(fsdf, '</sdf>');
    fclose(fsdf);
    allFiles = dir(pathstr);
    dirFlags = [allFiles.isdir];
    subFolders = allFiles(dirFlags);
    %% move color folders into /mesh foler
    idx = 1;
    while idx <= length(subFolders)
        if strcmp(subFolders(idx).name,'mesh') ||  strcmp(subFolders(idx).name,'sdf')
            subFolders(idx)=[]; 
        else
            idx = idx +1;
        end
    end
    for idx = 3:length(subFolders)
        movefile(fullfile(dataRoot, FolderName, subFolders(idx).name), fullfile(dataRoot, FolderName, 'mesh'));
    end
    
end