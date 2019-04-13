function D = fastmarchdistancematrixpar(shape,idx,dist_th,symmetric,workers)
n = size(shape.X,1);
if nargin < 2
    idx = 1:n;
end
if nargin < 3
    dist_th = Inf;
end
if nargin < 5
    workers = 6;
end
second_dim=numel(idx);
second_idx = idx;
if nargin < 4 
    if numel(idx)~=n
        symmetric = false;
    else
        symmetric=true;
    end
end

if ~symmetric
   second_dim = n;
   second_idx =1:n;
end

batches=cell(1,workers);
batch=floor(numel(idx)/workers);
for i=1:workers-1
    batches{i}=idx(1+(i-1)*batch:i*batch);
end
D=zeros(numel(idx),second_dim);
batches{workers}=idx(1+i*batch:end);
D_batch=cell(1,workers);
parfor i=1:workers
D_batch{i}=fastmarchdistancematrix(shape,batches{i},dist_th,false);
end
for i=1:workers-1
    D(1+(i-1)*batch:i*batch,:)=D_batch{i};
end
D(1+i*batch:end,:)=D_batch{workers};
if symmetric
    [x,y]=find(D>1e10); D(x,y)=D(y,x);
    [x,y]=find(D==0); D(x,y)=D(y,x);
    D=min(D,D');
end
end

