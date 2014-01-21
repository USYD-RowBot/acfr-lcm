classdef auv_enums42
    properties
        % basic
        Time = 3
        Date = 4
        ErrorState = 34

        % gps
        gps_Latitude = 1
        gps_Longitude = 2
        gps_Num_Sats = 5
        gps_Speed_Knots = 6
        gps_True_Heading = 7
        gps_Magnetic_Variation = 8
        
        % rdi current speed
        rdi_Cur_Speed =  26

        % compass
        comp_Magnetic_Heading = 9
        comp_Pitch_Angle = 10
        comp_Roll_Angle = 11
        comp_True_Heading = 12
        comp_Inside_Temp = 13
        comp_DFS_Depth = 14
        
        % altimeter
        alt_DTB_Height = 15
        
        % power
        pow_Batt_Percent = 16
        pow_Power_Watts = 17
        pow_Watt_Hours = 18
        pow_Batt_Volts = 19
        pow_Batt_Ampers = 20
        pow_Time_to_Empty = 22

        % command references
        ref_Dist_To_Next = 24
        ref_Next_Speed = 25
        ref_Next_Heading = 28
        ref_Next_Long = 29
        ref_Next_Lat = 30
        ref_Next_Depth = 31
        ref_Current_Step = 23 
        ref_Depth_K_Goal = 32
        ref_Pitch_K_Goal = 37
        ref_Speed_CMD = 27        
        
        % control
        ctrl_Pitch_L = 36
        ctrl_Pitch_R = 35
        ctrl_Yaw_T = 38
        ctrl_Yaw_B = 39
        ctrl_Yaw_goal = 40
        ctrl_Roll = 41
	end
end
