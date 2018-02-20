function [CSpace] = create_CSpace(Map,radius)

SE = strel('disk', radius, 0);
CSpace = imdilate(Map,SE);

%radius is passed in as pixel length
pad = 2;
%go through the space once, padding the edges of the map
for i = 1:(size(CSpace,1))
    for j = 1:(size(CSpace,2))
        if (i<radius)|| (i>(size(CSpace,1) - radius)) || (j<radius) || (j>(size(CSpace,2) - radius))
            CSpace(i,j) = pad;
        end
    end
end
% [xgrid, ygrid] = meshgrid(1:size(CSpace,2), 1:size(CSpace,1));
% %go through it once more and pad the obstacles in the map
% for i = (1+radius):(size(CSpace,1) - radius)
%     for j = (1+radius):(size(CSpace,2) - radius)
%         if (CSpace(i,j) == 1)
%             %window = CSpace(i-radius:i+radius,j-radius:j+radius);
%             mask = ((xgrid-j).^2 + (ygrid-i).^2) <= radius.^2;
%             window = CSpace(mask);
%             indices = window == 0;
%             window(indices) = pad;
%             CSpace(mask) = window;
%         end
%     end
% end
end