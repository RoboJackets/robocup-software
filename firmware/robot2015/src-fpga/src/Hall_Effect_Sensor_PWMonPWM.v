/*
This module reads the inputs from the hall effect sensors on bus 'h' and
outputs buses 'u' for which phase to drive high and 'z' for which phase will be 
high impedance (disconected from Vdd and gnd).
*/

`ifndef _HALL_EFFECT_SENSOR_PWM_ON_PWM_
`define _HALL_EFFECT_SENSOR_PWM_ON_PWM_

module Hall_Effect_Sensor ( hall, direction, u, z );

input    [2:0]  hall;       // Hall Effect sensor input
input           direction;  // Drive direction for commutation states
output   [2:0]  u;          // High phase output
output   [2:0]  z;          // High Impedance output

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

wire [2:0] u = direction ? (
                                ( hall == STATE1        )   ? A: // -> B
                                ( hall == STATE2        )   ? A: // -> C
                                ( hall == STATE3        )   ? B: // -> C
                                ( hall == STATE4        )   ? B: // -> A
                                ( hall == STATE5        )   ? C: // -> A
                                ( hall == STATE6        )   ? C: // -> B
                                ( hall == STATE_NO_CONN )   ? ALL_OFF:
                                ( hall == STATE_FAULT   )   ? ALL_OFF:ALL_OFF
                             ) :
                             (
                                ( hall == STATE1        )   ? B: // -> A
                                ( hall == STATE2        )   ? C: // -> A
                                ( hall == STATE3        )   ? C: // -> B
                                ( hall == STATE4        )   ? A: // -> B
                                ( hall == STATE5        )   ? A: // -> C
                                ( hall == STATE6        )   ? B: // -> C
                                ( hall == STATE_NO_CONN )   ? ALL_OFF:
                                ( hall == STATE_FAULT   )   ? ALL_OFF:ALL_OFF
                             );
                 
wire [2:0] z = direction ? (
                                ( hall == STATE1        )   ? B: // -> A
                                ( hall == STATE2        )   ? C: // -> A
                                ( hall == STATE3        )   ? C: // -> B
                                ( hall == STATE4        )   ? A: // -> B
                                ( hall == STATE5        )   ? A: // -> C
                                ( hall == STATE6        )   ? B: // -> C
                                ( hall == STATE_NO_CONN )   ? ALL_OFF:
                                ( hall == STATE_FAULT   )   ? ALL_ON:ALL_ON
                             ) :
                             (
                                ( hall == STATE1        )   ? A: // -> B
                                ( hall == STATE2        )   ? A: // -> C
                                ( hall == STATE3        )   ? B: // -> C
                                ( hall == STATE4        )   ? B: // -> A
                                ( hall == STATE5        )   ? C: // -> A
                                ( hall == STATE6        )   ? C: // -> B
                                ( hall == STATE_NO_CONN )   ? ALL_OFF:
                                ( hall == STATE_FAULT   )   ? ALL_ON:ALL_ON
                             );

endmodule

`endif
