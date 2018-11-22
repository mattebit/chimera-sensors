function [value] = comp2_16bit_unsigned(msb,lsb)
%Two's complement
%   16bit
tmp = zeros(1)
for i=1:length(msb)
    tmp(i,1) = msb(i)*256 + lsb(i)*256^0;
end
value = tmp;
end


