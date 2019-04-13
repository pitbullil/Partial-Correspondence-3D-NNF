function shape_to_ply(Path,shape)
if ~isfield(shape,'VERT')
    shape.VERT=zeros(3,length(shape.X));
    shape.VERT(1,:) = shape.X';
    shape.VERT(2,:) = shape.Y';
    shape.VERT(3,:) = shape.Z';
end
    if ~isfield(shape,'color')
        C = [];
    else
        C = shape.color;
    end
    TRI =shape.TRIV';
    write_ply(shape.VERT,TRI,Path,C);

