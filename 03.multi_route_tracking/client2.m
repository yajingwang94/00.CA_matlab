function client2()

%plot(data);
%Create a client interface and open it.

t = tcpip('localhost', 30000, 'NetworkRole', 'client');
fopen(t)
%Write the waveform to the server session.

i = ones(1,8);
%for i = 1:5
fwrite(t, i)
fclose(t)

%end



