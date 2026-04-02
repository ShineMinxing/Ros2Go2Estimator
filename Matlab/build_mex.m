mexName = 'fusion_estimator_mex';

mexFile = [mexName, '.mexw64']; 
if exist(mexFile, 'file') == 3
    fprintf('[INFO] MEX file exists, deleting...\n');
    delete(mexFile);
end

fprintf('[INFO] Compiling...\n');

root = pwd;
parent = fileparts(root);
fusion  = fullfile(parent, 'FusionEstimator');
assert(exist(fusion,'dir')==7, 'Missing folder: %s', fusion);

src = [ ...
    dir(fullfile(fusion,'**','*.cpp')); dir(fullfile(fusion,'**','*.c')); ...
    dir(fullfile(fusion,'**','*.cc'));  dir(fullfile(fusion,'**','*.cxx')); ...
];

keep = true(numel(src),1);
for i=1:numel(src)
    p = lower(fullfile(src(i).folder, src(i).name));
    n = lower(src(i).name);

    if ~isempty(strfind(p, [filesep 'demo' filesep])) || ~isempty(strfind(p, [filesep 'examples' filesep]))
        keep(i)=false; continue;
    end
    if strcmp(n,'main.cpp') || strcmp(n,'main.c')
        keep(i)=false; continue;
    end
    if ~isempty(strfind(n,'mex'))
        keep(i)=false; continue;
    end
end
src = src(keep);

srcfiles = cell(numel(src),1);
for i=1:numel(src)
    srcfiles{i} = fullfile(src(i).folder, src(i).name);
end

EIGEN_INC = '';

inc = {['-I' root], ['-I' fusion]};
if ~isempty(EIGEN_INC)
    inc{end+1} = ['-I' EIGEN_INC];
end

defs = {'-DNOMINMAX','-D_USE_MATH_DEFINES','-D_CRT_SECURE_NO_WARNINGS'};

inc      = inc(~cellfun(@isempty, inc));
defs     = defs(~cellfun(@isempty, defs));
srcfiles = srcfiles(~cellfun(@isempty, srcfiles));

mex('-v','-largeArrayDims', defs{:}, ...
    'COMPFLAGS=$COMPFLAGS /std:c++17 /EHsc', ...
    inc{:}, 'fusion_estimator_mex.cpp', srcfiles{:});

fprintf('[OK] MEX compiled.\n');