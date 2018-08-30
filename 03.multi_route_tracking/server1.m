
function [other_state, other_u0] = server1()
t = tcpip('0.0.0.0', 30000, 'NetworkRole', 'server');
%Open a connection. This will not return until a connection is received.
%set(t,'BytesAvailable',100)
fopen(t);
%Read the waveform and confirm it visually by plotting it.

%for i = 1:5
rec = fread(t, 8);
rec
other_state = rec(1:3);
other_u0 = rec(4:8);
%end%plot(data);
