pkg load instrument-control

s1 = serial("/dev/ttyACM0", 115200, 1) 

pid_data = cell(10,1)
pid_plot = 0
pid_sp = 5500.0

while true    
  for i = 1:10
    data_serial = str2double(char(srl_read(s1,50)))
    pid_data{i,1} = data_serial
  endfor
  pid_plot = cat(1, pid_plot, pid_data{1:10})
  plot(pid_plot)
  hold on
  hline(pid_sp)
  pause(1)
endwhile
srl_close(s1)