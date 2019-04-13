featurecodepath =  fullfile(pwd);%set to the repository root dir
assert(exist(fullfile(featurecodepath,'nnf_sparse_code','build','Release','3DDIS_OMP.exe'), 'file') == 2,'not in code root folder')
addpath(genpath(featurecodepath))
set_path = 'D:\OneDrive\TOPKIDS'; %path to SHREC topology dataset
cd(set_path)
for j = 16:25
        tic;
        cut_name = ['kid' num2str(j)]
        mkdir(cut_name);
        Dfile = fullfile(cut_name,'D');
        cut_file = [cut_name '.off'];
        shape = off_to_shape(cut_file);
        tic;D = fastmarchdistancematrixpar(shape);toc;
        save(Dfile,'D');
end
cd(featurecodepath)
        
        