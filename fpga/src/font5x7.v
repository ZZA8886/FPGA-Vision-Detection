`timescale 1ns/1ps
module font5x7(input wire [7:0] character,input wire [2:0] column,output wire [7:0] pixels);
    reg [39:0] glyph;
    always @* begin
        case(character)
            8'd35:glyph=40'h147f147f14;
            8'd48:glyph=40'h3e4549513e;
            8'd49:glyph=40'h00407f4200;
            8'd50:glyph=40'h4649516142;
            8'd51:glyph=40'h314b454121;
            8'd52:glyph=40'h107f121418;
            8'd53:glyph=40'h3945454527;
            8'd54:glyph=40'h3049494a3c;
            8'd55:glyph=40'h0305097101;
            8'd56:glyph=40'h3649494936;
            8'd57:glyph=40'h1e29494906;
            8'd65:glyph=40'h7e1111117e;
            8'd66:glyph=40'h364949497f;
            8'd67:glyph=40'h224141413e;
            8'd68:glyph=40'h1c2241417f;
            8'd69:glyph=40'h414949497f;
            8'd70:glyph=40'h010909097f;
            8'd71:glyph=40'h7a4949413e;
            8'd72:glyph=40'h7f0808087f;
            8'd73:glyph=40'h00417f4100;
            8'd74:glyph=40'h013f414020;
            8'd75:glyph=40'h412214087f;
            8'd76:glyph=40'h404040407f;
            8'd77:glyph=40'h7f020c027f;
            8'd78:glyph=40'h7f1008047f;
            8'd79:glyph=40'h3e4141413e;
            8'd80:glyph=40'h060909097f;
            8'd81:glyph=40'h5e2151413e;
            8'd82:glyph=40'h462919097f;
            8'd83:glyph=40'h3149494946;
            8'd84:glyph=40'h01017f0101;
            8'd85:glyph=40'h3f4040403f;
            8'd86:glyph=40'h1f2040201f;
            8'd87:glyph=40'h3f4038403f;
            8'd88:glyph=40'h6314081463;
            8'd89:glyph=40'h0708700807;
            8'd90:glyph=40'h4345495161;
            8'd43:glyph=40'h08083e0808;
            8'd45:glyph=40'h0808080808;
            8'd46:glyph=40'h0000606000;
            8'd58:glyph=40'h0000363600;
            8'd47:glyph=40'h0204081020;
            8'd61:glyph=40'h1414141414;
            8'd62:glyph=40'h0814224100;
            8'd60:glyph=40'h0041221408;
            8'd42:glyph=40'h14083e0814;
            8'd40:glyph=40'h0041221c00;
            8'd41:glyph=40'h001c224100;
            default:glyph=0;
        endcase
    end
    wire [39:0] shifted=glyph >> (column*8);
    assign pixels=column<5 ? shifted[7:0] : 8'b0;
endmodule
