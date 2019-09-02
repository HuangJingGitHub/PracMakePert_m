function  fig_update(h, T, origin_pos, rotation, cau_shape)
    ratio = 0.15;
    R = T(1:3, 1:3);
    X = T(1:3, 4);
    Ori = ratio * R;
    p1 = origin_pos;
    
    figure(h);
    cla
    
%     plane = rotation * [-0.1 0.1 0.1 -0.1; -0.1 -0.1 0.1 0.1; 0 0 0 0] * 0.8;
%     %p = fill3([-0.1 0.1 0.1 -0.1],[-0.1 -0.1 0.1 0.1],[0 0 0 0],[0.7 0.7 0.7 0.7],'edgecolor','none');
%     p = fill3(plane(1,:), plane(2,:), plane(3,:), [0 0 0], 'edgecolor','none');
%     alpha(p, 0.5);
    bottom  = rotation * [0.05, 0.05, -0.05, -0.05; 0, -0.05, -0.05, 0; 0.05, 0.05, 0.05, 0.05] * 0.5;
    top     = rotation * [0.05, 0.05, -0.05, -0.05; 0, -0.05, -0.05, 0; 0.6, 0.6, 0.6, 0.6] * 0.5;
    sidep_1 = rotation * [0.05, 0.05, 0.05, 0.05; 0, -0.05, -0.05, 0; 0.05, 0.05, 0.6, 0.6] *0.5;
    sidep_2 = rotation * [-0.05, -0.05, -0.05, -0.05; 0, -0.05, -0.05, 0; 0.05, 0.05, 0.6, 0.6] *0.5;    
    sidep_3 = rotation * [0.05, -0.05, -0.05, 0.05; 0, 0, 0, 0; 0.05 0.05 0.6 0.6] *0.5;
    sidep_4 = rotation * [0.05, -0.05, -0.05, 0.05; 0, 0, 0, 0; 0.05 0.05 0.6 0.6] *0.5;
    plane_1 = fill3(bottom(1,:), bottom(2,:), bottom(3,:), [0.6 0.6 0.6], 'edgecolor','k');
    plane_2 = fill3(top(1,:), top(2,:), top(3,:), [0.6 0.6 0.6], 'edgecolor','k');
    plane_3 = fill3(sidep_1(1,:), sidep_1(2,:), sidep_1(3,:), [0.6 0.6 0.6], 'edgecolor','k');    
    plane_4 = fill3(sidep_2(1,:), sidep_2(2,:), sidep_2(3,:), [0.6 0.6 0.6], 'edgecolor','k'); 
    plane_5 = fill3(sidep_3(1,:), sidep_3(2,:), sidep_3(3,:), [0.6 0.6 0.6], 'edgecolor','k'); 
    ref_p = fill3([-0.1 0.1 0.1 -0.1],[-0.1 -0.1 0.1 0.1],[0 0 0 0],[0.7 0.7 0.7 0.7],'edgecolor','none');
    alpha(ref_p, 0.3);
    
    plot3(cau_shape(1,:), cau_shape(2,:), cau_shape(3,:), '.k', 'MarkerSize',10);
    hold on
    
    text(-0.01, 0, 0, 'RCM', 'Fontsize',10);
    text(ratio, 0, 0, 'x_0');
    text(0, ratio, 0, 'y_0');
    text(0, 0, ratio, 'z_0');
    base_x = quiver3(0, 0, 0, ratio, 0, 0, 'LineWidth', 1.5, 'Color', 'b');
    hold on
    base_y = quiver3(0, 0, 0, 0, ratio, 0, 'LineWidth', 1.5, 'Color', 'g');
    hold on
    base_z = quiver3(0, 0, 0, 0, 0, ratio, 'LineWidth', 1.5, 'Color', 'r');
    hold on
    text(X(1)+Ori(1,1), X(2)+Ori(2,1), X(3)+Ori(3,1), 'x_{tip}');
    text(X(1)+Ori(1,2), X(2)+Ori(2,2), X(3)+Ori(3,2), 'y_{tip}');
    text(X(1)+Ori(1,3), X(2)+Ori(2,3), X(3)+Ori(3,3), 'z_{tip}');
    tip_x = quiver3(X(1), X(2), X(3), Ori(1,1), Ori(2,1), Ori(3,1), 'LineWidth', 1.5, 'Color', 'b');
    hold on
    tip_x = quiver3(X(1), X(2), X(3), Ori(1,2), Ori(2,2), Ori(3,2), 'LineWidth', 1.5, 'Color', 'g');
    hold on
    tip_x = quiver3(X(1), X(2), X(3), Ori(1,3), Ori(2,3), Ori(3,3), 'LineWidth', 1.5, 'Color', 'r');
    hold on
    link_34 = line([p1(1,3), p1(1,4)], [p1(2,3), p1(2,4)], [p1(3,3), p1(3,4)],...
                   'Color', 'r', 'Marker', '.', 'MarkerSize', 10, 'LineWidth', 3);
%     hold on
%     link_cannula = line([0, p2(1)], [0, p2(2)], [0, p2(3)],...
%                    'Color', 'k', 'Marker', '.', 'MarkerSize', 10, 'LineWidth', 3);               
    %%% Test on frame {4}
%     hold on
%     R = T4(1:3, 1:3);
%     X = T4(1:3, 4);
%     Ori = ratio * R;
%     text(X(1)+Ori(1,1), X(2)+Ori(2,1), X(3)+Ori(3,1), 'x_4');
%     text(X(1)+Ori(1,2), X(2)+Ori(2,2), X(3)+Ori(3,2), 'y_4');
%     text(X(1)+Ori(1,3), X(2)+Ori(2,3), X(3)+Ori(3,3), 'z_4');
%     tip_x = quiver3(X(1), X(2), X(3), Ori(1,1), Ori(2,1), Ori(3,1), 'LineWidth', 1.5, 'Color', 'b');
%     hold on
%     tip_x = quiver3(X(1), X(2), X(3), Ori(1,2), Ori(2,2), Ori(3,2), 'LineWidth', 1.5, 'Color', 'g');
%     hold on
%     tip_x = quiver3(X(1), X(2), X(3), Ori(1,3), Ori(2,3), Ori(3,3), 'LineWidth', 1.5, 'Color', 'r');
    

%     p = patch('Faces',[1,2,3,4],'Vertices',[-0.1 -0.1 0; 0.1 -0.1 0; 0.1 0.1 0;-0.1 0.1 0],'Facecolor','b')
%     alpha(p,0.3)
    axis equal
    axis([-0.4 0.35 -0.3 0.4 -0.3 0.35]);
    grid on
    view(171.5, 10.5);
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title('PSM Tip Trajectory with Cannula');

end