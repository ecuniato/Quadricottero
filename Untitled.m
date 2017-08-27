conn = tcpip('localhost', 27015, 'NetworkRole', 'client');
conn.ByteOrder = 'littleEndian';
fopen(conn);
fwrite(conn,'b');
t = 0 ;
x = 0 ;
y = 0;
time = 0;
startSpot = 0;
interv = 1000000 ; % considering 1000 samples
step = 1 ; % lowering step has a number of cycles and then acquire more data
while ( t <interv )
    rate = fread(conn,1,'float');
    gyro = fread(conn,1,'double');
    time = [time, t];
    x = [ x, rate];
    y = [ y, gyro];
    plot(time,x, 'red', time,y, 'blue');
      if ((t/step)-500 < 0)
          startSpot = 0;
      else
          startSpot = (t/(step+1))-500;
      end
      axis([ startSpot, (t/step+50), -20 , 20 ]);
      grid
      t = t + step;
      drawnow;
      pause(0.2)
end
 close(conn);