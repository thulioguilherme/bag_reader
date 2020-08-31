function getMetrics(filename)
  data = dlmread(filename, ',', 0, 0);
  b = [];
  x = [];
  
  num_uavs = columns(data);
  for i = 1:rows(data)
    b_r = 0.0;
    b_i = 0.0;
    for j = 1:columns(data)
      b_r = b_r + cos(data(i, j));
      b_i = b_i + sin(data(i, j)); 
    endfor
    b = [b; (1/columns(data)) * sqrt(power(b_r,2) + power(b_i,2)) ];
    x = [x; i - 1];
  endfor
  
  % Steady-state value
  u = sum(b(end-100:end)) / 100;
  printf("Steady-state value: %f\n", u);
  
  steady_str = cstrcat("Steady-state value: ", num2str(u, "%f"));
  
  % Settling time 
  t = rows(b);
  while (b(t) >= 0.95)
    t = t - 1;
  endwhile
  printf("Settling time: %d\n", t+1);
  title_str = cstrcat(num2str(num_uavs, "%d"), " UAVs - Only P");
  settling_str = cstrcat("Settling time: ", num2str(t, "%d"));
  
  plot(x, b);
  xlabel("Time (s)");
  ylabel("Order - \\psi");
  title({title_str, steady_str, settling_str});
  saveas(1, "order.png");
  
endfunction