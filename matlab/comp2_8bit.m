function [value] = comp2_8bit(msb)
%Two's complement
%   8bit
tmp = zeros(1);
for i=1:length(msb)
    if msb(i)>127
        tmp(i,1) = -(bitcmp(msb(i)*256^0,'uint8')+1);
    else
        tmp(i,1) = msb(i)*256^0;
    end
end
value = tmp;
end

