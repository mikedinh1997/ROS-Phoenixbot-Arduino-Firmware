pkg load instrument-control

clear all

s1 = serial("/dev/ttyUSB0", 115200) 
set(s1, 'timeout', -1)

pid_data = cell(10,1)
pid_plot = 0
pid_sp = 5500.0

while true    
  for i = 1:10
    data_serial = srl_read(s1,4)
    data_float = typecast(uint8(data_serial), 'single')
    pid_data{i,1} = data_float
  endfor
  pid_plot = cat(1, pid_plot, pid_data{1:10})
  plot(pid_plot)
  %hold on
  %hline(pid_sp)
  pause(1)
endwhile
srl_close(s1)