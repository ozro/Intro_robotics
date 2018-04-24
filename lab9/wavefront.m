function wavemap = wavefront(wavemap)
    wavemap = uint32(wavemap);
    label = 1;
    working = true;
    while working
        working = false;
        
        for row = 1:size(wavemap, 1)
            for col = 1:size(wavemap, 2)
                if wavemap(row, col) == label
                    for r = -1:1
                        for c = -1:1
                            if r == 0 && c == 0
                                continue;
                            end
                            
                            if row + r < 1 || row + r > size(wavemap,1) ...
                                    || col + c < 1 || col + c > size(wavemap,2)
                                continue;
                            end
                            
                            if wavemap(row+r, col+c) == 0
                                wavemap(row+r, col+c) = label + 1;
                                working = true;
                            end
                        end
                    end
                end
            end
        end
        
        label = label + 1;
    end
end