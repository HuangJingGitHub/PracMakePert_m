% run_test_p_poly_dist
% script to run the unit test for p_poly_dist function

xp3 = [-1 0 2];
yp3 = [-3 -2 1];

xv7 = [-3 -1 0 1 2 2 0];
yv7 = [-1 0 1 1 -1 -2 -3];

xp7 = [-2 -1 0 1 2 1 0];
yp7 = [-1 0 1 1 -1 -2 -3];

xv3 = [-2 -1 0];
yv3 = [-1 0 1];

xp1 = [-1];
yp1 = [-3];

xv2 = [-2 -1];
yv2 = [-1 0];

find_in_out = false;
[hf, hp, d_min, x_d_min, y_d_min, is_vertex, idx_c, xc, yc, is_in_seg, Cer, Ppr] = test_p_poly_dist(xp3(:), yp3(:), xv7(:), yv7(:), find_in_out);
%[hf, hp, d_min, x_d_min, y_d_min, is_vertex, idx_c, xc, yc, Cer, Ppr] = test_p_poly_dist(xp7, yp7, xv3, yv3);
%[hf, hp, d_min, x_d_min, y_d_min, is_vertex, idx_c, xc, yc, Cer, Ppr] = test_p_poly_dist(xp1, yp1, xv2, yv2);
%[hf, hp, d_min, x_d_min, y_d_min, is_vertex, idx_c, xc, yc, Cer, Ppr] = test_p_poly_dist(xp1, yp1, xv3, yv3, true);


nruns = 10;
max_np = 10;
max_nv = 10;

xrange = [-10 10];
yrange = [-10 10];

vnp = randi([1 max_np], nruns, 1);
vnv = randi([3 max_nv], nruns, 1);

for j=1:nruns,
   xv = xrange(1) + (xrange(2)-xrange(1)).*rand(vnv(j),1);   
   yv = yrange(1) + (yrange(2)-yrange(1)).*rand(vnv(j),1);
   for k = 1:nruns,
      xp = xrange(1) + (xrange(2)-xrange(1)).*rand(vnp(k),1);
      yp = yrange(1) + (yrange(2)-yrange(1)).*rand(vnp(k),1);
      
      d_min = p_poly_dist(xp, yp, xv, yv);
      d = zeros(1, vnp(k));
      
      for m=1:vnp(k)
         d(m) = p_poly_dist1(xp(m), yp(m), xv, yv);
      end
      
      disp(max(abs((d_min(:) - d(:))')));
   end
end
