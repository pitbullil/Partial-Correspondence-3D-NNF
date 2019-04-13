function [shape] = ply_to_shape(Path)
    [VERT,TRI] = read_ply(Path);
    shape.VERT = VERT;
    shape.X = VERT(:,1);
    shape.Y = VERT(:,2);
    shape.Z = VERT(:,3);
    shape.TRIV = TRI;
    shape.n = numel(shape.Z);
end

