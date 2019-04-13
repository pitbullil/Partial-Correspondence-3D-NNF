function [h] = vis_partial_dense_matches( part, model, yin,outfile)
%VIS_PARTIAL_MATCHES Summary of this function goes here
%   Detailed explanation goes here

%        boundary = extract_boundary( part);
        colors = create_colormap(model,model);
        h=figure('units','normalized','outerposition',[0 0 1 1]);
        ax1=subplot(1,2,1);colormap(colors);
        plot_scalar_map(model,[1: size(model.VERT,1)]');freeze_colors;title('Model');
                   hold on,
        lim = axis;
         axis equal;
    shading interp
            lighting gouraud

    camlight
    set(gca, 'xgrid', 'off')
    set(gca, 'ygrid', 'off')
    set(gca, 'zgrid', 'off')
set(gca,'Visible','off')

        ax2=subplot(1,2,2);colormap(colors(yin,:));
        plot_scalar_map(part,[1: size(part.VERT,1)]');freeze_colors;title('After slantization');
        hold on;
        %for i=1:size(boundary,1)
        %    p1 = part.VERT(boundary(i,1),:);p2 = part.VERT(boundary(i,2),:);plot3([p1(1) p2(1)],[p1(2) p2(2)],[p1(3) p2(3)],'-r','LineWidth',1);hold on;
        %end
        %lim2 = axis;
         
        %axis([ax1 ax2],2*[lim])
            %axis equal;
        shading interp
            lighting gouraud

        camlight
        light('Position',[-1 0 0],'Style','local')
        lighting gouraud
        white = 200 * [1 1 1];

        colormap white,

        set(gca, 'xgrid', 'off')
        set(gca, 'ygrid', 'off')
        set(gca, 'zgrid', 'off')
        set(gca,'Visible','off')

        Cmesh = colorize_mesh(part,colors(yin,:));
%hlink = linkprop([ax1,ax2],{'CameraPosition','CameraUpVector'}); 
rotate3d on
pause
if nargin >3
    saveas(gcf,[outfile '.png']);
    shape_to_ply([outfile '.ply'],Cmesh)
    Cmesh = colorize_mesh(model,colors);
    shape_to_ply([outfile '_base.ply'],Cmesh)

end
close;
end

