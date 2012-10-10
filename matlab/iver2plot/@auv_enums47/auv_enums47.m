classdef auv_enums47
    properties
        % basic
        Time = 3
        Date = 4
        ErrorState = 36

        % gps
        gps_Latitude = 1
        gps_Longitude = 2
        gps_Num_Sats = 5
        gps_Speed_Knots = 6
        gps_True_Heading = 7
        gps_Magnetic_Variation = 8
        
        % rdi current speed
        rdi_Cur_Speed =  28

        % compass
        comp_Magnetic_Heading = 10
        comp_Pitch_Angle = 12
        comp_Roll_Angle = 13
        comp_True_Heading = 11
        comp_Inside_Temp = 14
        comp_DFS_Depth = 15
        
        % altimeter
        alt_DTB_Height = 16
        
        % power
        pow_Batt_Percent = 18
        pow_Power_Watts = 19
        pow_Watt_Hours = 20
        pow_Batt_Volts = 21
        pow_Batt_Ampers = 22
        pow_Time_to_Empty = 24

        % command references
        ref_Dist_To_Next = 26
        ref_Next_Speed = 27
        ref_Next_Heading = 30
        ref_Next_Long = 31
        ref_Next_Lat = 32
        ref_Next_Depth = 33
        ref_Current_Step = 25 
        ref_Depth_K_Goal = 34
        ref_Pitch_K_Goal = 39
        ref_Speed_CMD = 29        
        
        % control
        ctrl_Pitch_L = 38
        ctrl_Pitch_R = 37
        ctrl_Yaw_T = 40
        ctrl_Yaw_B = 41
        ctrl_Yaw_goal = 42
        ctrl_Roll = 43
	end
end
