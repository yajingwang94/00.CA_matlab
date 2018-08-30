function client1(send_data)

%plot(data);
%Create a client interface and open it.

t = tcpip('localhost', 30001, 'NetworkRole', 'client');
fopen(t)
%Write the waveform to the server session.


%for i = 1:5
fwrite(t, sen_data)

%end



