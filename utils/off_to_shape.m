function [shape] = off_to_shape(Path)
    [V,TRI] = read_off(Path);
    V=V';
    shape.VERT = V;
    shape.X = V(:,1);
    shape.Y = V(:,2);
    shape.Z = V(:,3);
    shape.TRIV = TRI';
    shape.n = size(shape.X,1);
    shape.m = size(shape.TRIV,1);

end
