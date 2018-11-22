function [value] = comp2_24bit(msb,byte2,lsb)
%Two's complement
%   24bit
for i=1:length(msb)
    if msb(i)>127
        tmp(i,1) = -(bitcmp(msb(i)*256^2 + byte2(i)*256^1 + lsb(i)*256^0,'uint24')+1);
    else
        tmp(i,1) = msb(i)*256^2 + byte2(i)*256^1 + lsb(i)*256^0;
    end
end
value = tmp;
end

