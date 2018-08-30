
function server2()
t = tcpip('0.0.0.0', 30001, 'NetworkRole', 'server');
%Open a connection. This will not return until a connection is received.
%set(t,'BytesAvailable',100)
fopen(t);
%Read the waveform and confirm it visually by plotting it.

%for i = 1:5
rec = fread(t,8);
rec
%end%plot(data);
