function [x,y]=ConvertLL2XY(refLon, refLat, cLon, cLat)
     ANG2RAD=0.0174532925;
    angle=TwoPointsCourseAngle(refLon,refLat,cLon,cLat);
	dis=TwoPointsHorDistance(refLon,refLat,cLon,cLat);
	x=dis*cos(angle*ANG2RAD);
  	y=dis*sin(angle*ANG2RAD);
end

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
 function dis=TwoPointsHorDistance(cLon, cLat, aLon, aLat)
    R=6371004.00;
    ANG2RAD=0.0174532925;
	Ca_c = cos((aLon - cLon)*ANG2RAD);
	Cclat = cos(cLat*ANG2RAD);
	Sclat = sin(cLat*ANG2RAD);
	Calat = cos(aLat*ANG2RAD);
	Salat = sin(aLat*ANG2RAD);
	temp2=Calat*Cclat*Ca_c + Salat*Sclat;
	if temp2<-1
		temp2=-1;
    end
	if(temp2>1)
		temp2=1;
    end
	dis = R * acos(temp2);
end

function angle=TwoPointsCourseAngle(P1Lo,P1La,P2Lo,P2La)
    
    ANG2RAD=0.0174532925;
    RAD2ANG=57.295779469;
	angle=0;

	temp_n= P2La - P1La;	  
	temp_e=(P2Lo - P1Lo)*cos(P1La*ANG2RAD);
	Len = sqrt(temp_n*temp_n+temp_e*temp_e);  
	  
	if (Len> 0)
	  angle = acos(temp_n/Len);
	  if(temp_e>=0)
	    angle =angle *RAD2ANG; 
      else
	   angle = 360.0-angle * RAD2ANG;  
      end
    end
    
	if (angle >= 360.0)
	   angle=angle - 360.0;
    elseif (angle < 0)
	    angle=angle + 360.0;
    end
	  
   if(angle < 0)
		angle = 0;
   elseif(angle > 360.0)
		angle = 360.0; 
   end;	 
end