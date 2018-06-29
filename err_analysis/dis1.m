function dis=dis1(ref_Lon,ref_Lat,Lon,Lat)
A2R=pi/180;
R2A=180/pi;
R= 6371004;
Ca=cos((Lon-ref_Lon)*A2R);
Cclat=cos(ref_Lat*A2R);
Sclat=sin(ref_Lat*A2R);
Calat=cos(Lat*A2R);
Salat=sin(Lat*A2R);

len=length(Calat);
for i=1:len
temp2(i)=Calat(i)*Cclat(i)*Ca(i)+Salat(i)*Sclat(i);
if(temp2(i)<-1)
    temp2(i)=-1;
end
if(temp2(i)>1)
    temp2(i)=1;
end
temp(i)=R*acos(temp2(i));
end
dis=temp;
end