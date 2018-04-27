function ang=angle1(P1Lon,P1Lat,P2Lon,P2Lat)
A2R=pi/180;
R2A=180/pi;
R= 6371004;
temp_n=P2Lat-P1Lat;
temp_e=(P2Lon-P1Lon)*cos(P1Lat*A2R);
len=length(temp_n);
if len>1
for i=1:len
 Len(i)=sqrt(temp_n(i)^2+temp_e(i)^2);
    angg(i)=acos(temp_n(i)/Len(i));
    if(temp_e(i)>=0)
        angg(i)=angg(i)*R2A;
    else
        angg(i)=360-angg(i)*R2A;
    end
if (angg(i)>=360)
    angg(i)=angg(i)-360;
else if(angg(i)<=0)
        angg(i)=angg(i)+360;
    end
end

if angg(i)<0
    angg(i)=0;
end
if angg(i)>360
    angg(i)=360;
end
end

else
  Len=sqrt(temp_n^2+temp_e^2);

    angg=acos(temp_n/Len);
    if(temp_e>=0)
        angg=angg*R2A;
    else
        angg=360-angg*R2A;
    end

if (angg>=360)
    angg=angg-360;
else if(angg<0)
        angg=angg+360;
    end
end

if angg<0
    angg=0;
end
if angg>360
    angg=360;
end  
    
end
ang=angg;
    
end