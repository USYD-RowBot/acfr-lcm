classdef auv_enums
    properties
        % basic
        Time = 3
        Date = 4
        Leak = 17
        State = 22

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
        comp_DFS2_Depth = 18
        
        % altimeter
        alt_DTB_Height = 24
        alt_DR_Speed = 27
        alt_Sounder_Speed = 25
        
        % power
        pow_Batt_Percent = 15
        pow_Power_Watts = 16
        pow_Watt_Hours = 19
        pow_Batt_Volts = 20
        pow_Batt_Ampers = 21
        pow_Time_to_Empty = 23

        % control
        ctrl_Cur_True_Heading = 34 %duplicate of compass measurement
        ctrl_Roll_Angle = 35       %duplicate of compass measurement
        ctrl_Pitch_Angle = 36      %duplicate of compass measurement
        ctrl_Servo_State = 37
        ctrl_Pitch_R = 38
        ctrl_Yaw_T = 39
        ctrl_Yaw_K_Error = 41
        ctrl_Pitch_L = 44
        ctrl_Depth_iTerm = 54
        ctrl_Depth_pTerm = 55
        ctrl_Depth_iSTATE = 57
        ctrl_Pitch_dTerm = 58
        ctrl_Pitch_err = 59
        ctrl_Pitch_Calc = 61
        
        % command references
        ref_Dist_To_Next = 28
        ref_Next_Speed = 29
        ref_Next_Heading = 30
        ref_Next_Long = 31
        ref_Next_Lat = 32
        ref_Next_Depth = 33
        ref_Current_Step = 26        
        ref_Depth_K_Goal = 42     %duplicate of Dgoal
        ref_Pitch_K_Goal = 43     %duplicate of Pitch_goal
        ref_Speed_CMD = 40
        ref_Dgoal = 56
        ref_Pitch_goal = 60
        
        % unused
        unkn_Temp_C = 45
        unkn_SpCond_uSpcm = 46
        unkn_Sal_ppt = 47
        unkn_Depth_meters = 48
        unkn_ODOsat_percent = 49
        unkn_ODO_mgpL = 50
        unkn_Chl_ugpL = 51
        unkn_Chl_RFU = 52
        unkn_Battery_volts = 53
	end
end
