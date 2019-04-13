function D = fastmarchdistancematrix(shape,idx,dist_th,symmetric)
n = size(shape.X,1);
if nargin < 2
    idx = 1:n;
end
if nargin < 3
    dist_th = 1e7;
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

D = zeros(length(idx),second_dim);
f = fastmarchmex('init', int32(shape.TRIV-1), double(shape.X(:)), double(shape.Y(:)), double(shape.Z(:)));
for i = 1 : length(idx)
    src = Inf(n,1);
    src(idx(i)) = 0;
    d = fastmarchmex('march', f, double(src), double(dist_th));
    D(i,second_idx) = d(second_idx)';
end

fastmarchmex('deinit', f);

if symmetric
    [x,y]=find(D>1e10); D(x,y)=D(y,x);
    [x,y]=find(D==0); D(x,y)=D(y,x);
    D=min(D,D');
end

end

