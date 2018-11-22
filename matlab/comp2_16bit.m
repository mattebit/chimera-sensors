function [value] = comp2_16bit(msb,lsb)
%Two's complement
%   16bit
tmp = zeros(1)
for i=1:length(msb)
    if msb(i)>127
        tmp(i,1) = -(bitcmp(msb(i)*256^1 + lsb(i)*256^0,'uint16')+1);
    else
        tmp(i,1) = msb(i)*256 + lsb(i)*256^0;
    end
end
value = tmp;
end



