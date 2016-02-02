/*
This module reads the inputs from the hall effect sensors on bus 'h' and
outputs buses 'u' for which phase to drive high and 'z' for which phase will be 
high impedance (disconected from Vdd and gnd), the third non-specified phase 
where, h[n] and z[n] are both zero, is driven low. h[2] is hall 1, h[1] is 
hall 2, etc. u[2] and z[2] correspond to motor phase 1 or A, u[1] and z[1] 
correspond to motor phase 2 or B, etc. 

With only three bits in input bus h there are only eight posible states for the 
hall sensors, two of which indcate an error: '000' and '111'. If these states 
are ever encountere, all three motor phasese signaled to go into the high 
impedance (disconnected) state. Otherwise, the high phase, low phase, and high 
impedance phase for each hall state is specified in the document linked below.

//http://www.maxonmotor.com/medias/sys_master/root/8815461662750/EC-Technology-short-and-to-the-point-14-EN-32-35.pdf?attachment<=true
--Doho*/

`ifndef _HALL_EFFECT_SENSOR_
`define _HALL_EFFECT_SENSOR_

module Hall_Effect_Sensor ( hall, u, z );

input    [2:0]  hall;   // Hall Effect sensor input
output   [2:0]   u;     // High phase output
output   [2:0]   z;     // High Impedance output

localparam  A       =       3'b100;
localparam  B       =       3'b010;
localparam  C       =       3'b001; 
localparam  ALL_ON  =       3'b111;
localparam  ALL_OFF =       3'b000;

// going from 1 -> 6 is CCW
localparam STATE1 =         3'b101;
localparam STATE2 =         3'b100;
localparam STATE3 =         3'b110;
localparam STATE4 =         3'b010;
localparam STATE5 =         3'b011;
localparam STATE6 =         3'b001;
// error or no connection states
localparam STATE_FAULT   =  3'b000;
localparam STATE_NO_CONN =  3'b111;


wire [2:0] u =
    ( hall == STATE1        ) ? A:
    ( hall == STATE2        ) ? A:
    ( hall == STATE3        ) ? B:
    ( hall == STATE4        ) ? B:
    ( hall == STATE5        ) ? C:
    ( hall == STATE6        ) ? C:
    ( hall == STATE_NO_CONN )   ? ALL_OFF:
    ( hall == STATE_FAULT   )   ? ALL_OFF:
                                  ALL_OFF;
                       
wire [2:0] z =
    ( hall == STATE1        )   ? C:
    ( hall == STATE2        )   ? B:
    ( hall == STATE3        )   ? A:
    ( hall == STATE4        )   ? C:
    ( hall == STATE5        )   ? B:
    ( hall == STATE6        )   ? A:
    ( hall == STATE_NO_CONN )   ? ALL_OFF:
    ( hall == STATE_FAULT   )   ? ALL_OFF:
                                  ALL_ON;

endmodule

`endif
