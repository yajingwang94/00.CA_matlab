function [x,y]=Convert(ref_Lon,ref_Lat,Lon,Lat)

angle2=angle1(ref_Lon,ref_Lat,Lon,Lat);
dis=dis1(ref_Lon,ref_Lat,Lon,Lat);
x=dis*cos(angle2*pi/180);
y=dis*sin(angle2*pi/180);
end