function fkVis(steps,dt,Ori,X,X_1,X_3,X_4,X_5)
%%% Forward kinematics visualization / animation.

ratio = 0.6;              % ratio is set for the length adjustment of arrow for better visual effect.
Ori = ratio * Ori;
Ori_x = [X(1,1)+Ori(1,1), X(2,1)+Ori(2,1),X(3,1)+Ori(3,1)];    % The arrow end position.
Ori_y = [X(1,1)+Ori(1,2), X(2,1)+Ori(2,2),X(3,1)+Ori(3,2)];
Ori_z = [X(1,1)+Ori(1,3), X(2,1)+Ori(2,3),X(3,1)+Ori(3,3)];
% Ori_X = [X, X, X] + Ori;

%%%******Plot the tip trajectory******%%%
figure('Position',[0 -200 1050 800]);
plot3(X(1,:),X(2,:),X(3,:));
axis([0,4 0,4, -2,2]);
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Tip Trajectory')
% view(70,35);
hlink1 = line([0, X_1(1,1)],...
              [0, X_1(2,1)],...
              [0, X_1(3,1)],...
              'Color',[0.67 0 1],'Marker','.','MarkerSize',20, 'LineWidth',2);
hlink2 = line([X_1(1,1), X_3(1,1)],...
              [X_1(2,1), X_3(2,1)],...
              [X_1(3,1), X_3(3,1)],...
              'Color',[0.67 0 1],'Marker','.','MarkerSize',20, 'LineWidth',2);
hlink3 = line([X_3(1,1), X_4(1,1)],...
              [X_3(2,1), X_4(2,1)],...
              [X_3(3,1), X_4(3,1)],...
              'Color',[0.67 0 1],'Marker','.','MarkerSize',20, 'LineWidth',2);
hlink4 = line([X_4(1,1), X_5(1,1)],...
              [X_4(2,1), X_5(2,1)],...
              [X_4(3,1), X_5(3,1)],...
              'Color',[0.67 0 1],'Marker','.','MarkerSize',20, 'LineWidth',2);
hlink = line([X_5(1,1), X(1,1)],...
             [X_5(2,1), X(2,1)],...
             [X_5(3,1), X(3,1)],...
             'Color',[0.67 0 1],'Marker','.','MarkerSize',20, 'LineWidth',2);
hold on
arrow_x = arrow3(X(:,1)', Ori_x, 'k');      % Plot the tip frame coordiate using function arrow.  
arrow_y = arrow3(X(:,1)', Ori_y, 'r');
arrow_z = arrow3(X(:,1)', Ori_z, 'g');
Name_x = text(Ori_x(1), Ori_x(2), Ori_x(3), 'X', 'fontsize', 10);
Name_y = text(Ori_y(1), Ori_y(2), Ori_y(3), 'Y', 'color', 'r', 'fontsize', 10);
Name_z = text(Ori_z(1), Ori_z(2), Ori_z(3), 'Z', 'color', 'g', 'fontsize', 10);

 for i=2:steps
      id_x = 3*i - 2;       % Index number for Ori matrix in each loop. 
      id_y = id_x + 1;
      id_z = id_x + 2;
      Ori_x = [X(1,i)+Ori(1,id_x), X(2,i)+Ori(2,id_x), X(3,i)+Ori(3,id_x)];  % Update Ori_x, Ori_y, Ori_z.
      Ori_y = [X(1,i)+Ori(1,id_y), X(2,i)+Ori(2,id_y), X(3,i)+Ori(3,id_y)];
      Ori_z = [X(1,i)+Ori(1,id_z), X(2,i)+Ori(2,id_z), X(3,i)+Ori(3,id_z)];
      
      delete(arrow_x);
      delete(arrow_y);
      delete(arrow_z);
      
      % Update the linkages and tip frame.
      set(hlink1,'xdata',[0, X_1(1,i)],...        
                 'ydata',[0, X_1(2,i)],...
                 'zdata',[0, X_1(3,i)]);
      set(hlink2,'xdata',[X_1(1,i), X_3(1,i)],...
                 'ydata',[X_1(2,i), X_3(2,i)],...
                 'zdata',[X_1(3,i), X_3(3,i)]);
      set(hlink3,'xdata',[X_3(1,i), X_4(1,i)],...
                 'ydata',[X_3(2,i), X_4(2,i)],...
                 'zdata',[X_3(3,i), X_4(3,i)]);
      set(hlink4,'xdata',[X_4(1,i), X_5(1,i)],...
                 'ydata',[X_4(2,i), X_5(2,i)],...
                 'zdata',[X_4(3,i), X_5(3,i)]);   
      set(hlink,'xdata',[X_5(1,i), X(1,i)],...
                'ydata',[X_5(2,i), X(2,i)],...
                'zdata',[X_5(3,i), X(3,i)]);
  
      arrow_x = arrow3(X(:,i)' ,Ori_x, 'k');
      arrow_y = arrow3(X(:,i)' ,Ori_y, 'r');
      arrow_z = arrow3(X(:,i)' ,Ori_z, 'g');
      set(Name_x, 'position', Ori_x);
      set(Name_y, 'position', Ori_y);
      set(Name_z, 'position', Ori_z);
      
      % drawnow;
      pause(dt*5);       % Pause in every step.
 end
 
end
 
