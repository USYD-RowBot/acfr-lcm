classdef auv_enums3
    properties
        % basic
        Time = 3
        Date = 4
        Leak = 17
        State = 21

        % gps
        gps_Latitude = 1
        gps_Longitude = 2
        gps_Num_Sats = 5
        gps_Speed_Knots = 6
        gps_True_Heading = 7
        gps_Magnetic_Variation = 8

        % compass
        comp_Magnetic_Heading = 9
        comp_Pitch_Angle = 10
        comp_Roll_Angle = 11
        comp_True_Heading = 12
        comp_Inside_Temp = 13
        comp_DFS_Depth = 14
        
        % altimeter
        alt_DTB_Height = 23
        alt_DR_Speed = 26
        alt_Sounder_Speed = 24
        
        % power
        pow_Batt_Percent = 15
        pow_Power_Watts = 16
        pow_Watt_Hours = 18
        pow_Batt_Volts = 19
        pow_Batt_Ampers = 20
        pow_Time_to_Empty = 22

        % control
        ctrl_Cur_True_Heading = 33 %duplicate of compass measurement
        ctrl_Roll_Angle = 34       %duplicate of compass measurement
        ctrl_Pitch_Angle = 35      %duplicate of compass measurement
        ctrl_Servo_State = 36
        ctrl_Pitch_L = 43
        ctrl_Pitch_R = 37
        ctrl_Yaw_T = 38
        ctrl_Yaw_K_Error = 40
        
        % command references
        ref_Dist_To_Next = 27
        ref_Next_Speed = 28
        ref_Next_Heading = 29
        ref_Next_Long = 30
        ref_Next_Lat = 31
        ref_Next_Depth = 32
        ref_Current_Step = 25 
        ref_Depth_K_Goal = 41
        ref_Pitch_K_Goal = 42
        ref_Speed_CMD = 39
	end
end
