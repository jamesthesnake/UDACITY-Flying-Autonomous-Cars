1.Generate Motor commands
 lines 72-82, get the four Taus of reactive moment  by the fou thrusts  by the order of the drag ratio, created vars and a thrust to track this
  float r = momentCmd.z / (kappa * 4.f);
  float var_root_two = L / sqrtf(2.f);
  float q = momentCmd.x / (var_root_two * 4.f);
  float c = collThrustCmd / 4.f;
  float b = momentCmd.y / (var_root_two * 4.f);

  cmd.desiredThrustsN[2] = c + b - q - r; 
  cmd.desiredThrustsN[0] = c + b + q + r; 
  cmd.desiredThrustsN[1] = c - b + q - r; 

  cmd.desiredThrustsN[3] = c - b - q + r;
2. Body rate Control
Lines 124 to 132  its the angular accleration around the x,yz, axis times the moment of interation, to get the tau of the resulting moment

  V3F I_moment;
  I_moment.x = Ixx;
  I_moment.y = Iyy;
  I_moment.z = Izz;

![alt text](images/image_1.png)

3. PITCH ROLL 
lines 162-188  , get the body rate around the x and y axis as scalrs and feed that to the body rate control. Based around the R rotation matrix of the altiude of the drone.
    float xa_mat = R(0,2);
    float ya_mat = R(1,2);
    float Rot_33_mat = R(2,2);
    float Rot_21_mat = R(1,0);
    float Rot_22_mat = R(1,1);
    float Rot_12_mat = R(0,1);
    float Rot_11_mat = R(0,0);

4. Altitude control
Lines 217-241, get the  Force of the thurt basd on the zaxis of the interial frame , and control gain parameters

5.Lateral postion 
Lines 264-278 , like Altidue control but for along the x and y  axis frame.
   V3F error_in_pos = posCmd - pos ;

   if (velCmd.mag() > maxSpeedXY) {
      velCmd = velCmd.norm() * maxSpeedXY;
    }
   
  V3F err_vel = velCmd - vel ;


6. Yaw
Lines 313-321, get the angular velocity around the z axis , not impacted by roll or pitch so applied last and analyzled seperatley .

    float yerror = yawRateCmd - yaw;
    yerror = fmodf(yerror, F_PI*2.f);

