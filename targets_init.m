function target_points = targets_init(sizeOfFiled,resolution)

xs = linspace(-sizeOfFiled(1)/2,sizeOfFiled(1)/2,resolution);
ys = linspace(-sizeOfFiled(2)/2,sizeOfFiled(2)/2,resolution);
target_points = [xs',ys'];

end

