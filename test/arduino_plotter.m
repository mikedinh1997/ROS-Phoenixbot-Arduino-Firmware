pkg load instrument-control

clear all

s1 = serial("COM4", 115200);
set(s1, 'timeout', -1);
srl_flush(s1);

pid_plot = 0;
pid_sp = 0;

while true    
  data_serial = srl_read(s1,4)
  data_float = typecast(uint8(data_serial), 'single')
  pid_plot = cat(1, pid_plot, data_float)
  plot(pid_plot, "color", "b")
  hold on
  plot([pid_sp,pid_sp], "color", "r")
  drawnow()
  pause(.0001)
endwhile
srl_close(s1)