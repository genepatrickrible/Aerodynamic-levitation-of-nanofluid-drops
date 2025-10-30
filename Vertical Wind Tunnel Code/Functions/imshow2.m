function [img] = imshow2(img,getCB)
%Never liked imshow itself, due to it being normalized to 255, and not auto
%contrasted. Now it's normalized versus the minimum and maximum.
if ~exist('getCB','var')
    getCB = 0;
end
if ndims(img) == 3
    img = double(img)/double(max(img,[],'all'));
    imshow(img)
elseif length(unique(img)) == 2 %If already binarized, do nothing...
    imshow(img)
else %Otherwise, normalize it to its minimum and maximum.
    imshow(img, [min(img,[],'all'),max(img,[],'all')])
    if getCB == 1
        colorbar
    end
end
end