function flag = triangle_intersection(P1, P2)
% triangle_test : returns true if the triangles overlap and false otherwise

%%% All of your code should be between the two lines of stars.
% *******************************************************************
flag = false;

%判断两三角形是否重合
num_com = 0;%两个三角形共点的个数
for i = 1:3
    for j = 1:3
        if P1(i,:) == P2(j,:)
            num_com = num_com + 1;
        end
    end
end
if num_com == 3%三个点相同，两个三角形重合
    flag = true;
    return;
end

%判断边是否有相交
%思路：先求两直线的交点，再判断交点是否在线段上，注意如果交点正好位于线段的边缘，
%或者线段是水平或垂直的，则简单比较可能会出现问题
num = 1;
for i = 1:2
    for j = i+1:3
        A1(num) =  P1(i,2) - P1(j,2);
        B1(num) = -P1(i,1) + P1(j,1);
        C1(num) =  P1(j,1) * P1(i,2) - P1(i,1) * P1(j,2);
        
        A2(num) =  P2(i,2) - P2(j,2);
        B2(num) = -P2(i,1) + P2(j,1);
        C2(num) =  P2(j,1) * P2(i,2) - P2(i,1) * P2(j,2);
        
        num = num + 1;
    end
end   %转化为Ax+By=C的形式

for i = 1:3
    for j = 1:3
        A = [A1(i), B1(i); A2(j), B2(j)];
        B = [C1(i); C2(j)];
        X = pinv(A) * B;
%         X = A\B;
        x = X(1);
        y = X(2);           %计算交点
        
        switch i
            case 1
                x11 = P1(1,1); x12 = P1(2,1); y11 = P1(1,2); y12 = P1(2,2);
            case 2
                x11 = P1(1,1); x12 = P1(3,1); y11 = P1(1,2); y12 = P1(3,2);
            case 3
                x11 = P1(2,1); x12 = P1(3,1); y11 = P1(2,2); y12 = P1(3,2);
        end
        switch j
            case 1
                x21 = P2(1,1); x22 = P2(2,1); y21 = P2(1,2); y22 = P2(2,2);
            case 2
                x21 = P2(1,1); x22 = P2(3,1); y21 = P2(1,2); y22 = P2(3,2);
            case 3
                x21 = P2(2,1); x22 = P2(3,1); y21 = P2(2,2); y22 = P2(3,2);
        end
        if  ((min(x11, x12) <= x && x <= max(x11, x12)) && (min(y11, y12) <= y && y <= max(y11, y12)) ...
          && (min(x21, x22) <= x && x <= max(x21, x22)) && (min(y21, y22) <= y && y <= max(y21, y22)))
            flag = true;
            return;
        end                 %判断是否有交点
    end
end
    
%判断是否是包含关系
for i = 1:3
    b1 = sign(P2(i, :), P1(1, :), P1(2, :)) <= 0;
    b2 = sign(P2(i, :), P1(2, :), P1(3, :)) <= 0;
    b3 = sign(P2(i, :), P1(3, :), P1(1, :)) <= 0;
    if b1 == b2 && b2 == b3
        flag = true;
        return;
    end
end

for i = 1:3
    b1 = sign(P1(i, :), P2(1, :), P2(2, :)) < 0;
    b2 = sign(P1(i, :), P2(2, :), P2(3, :)) < 0;
    b3 = sign(P1(i, :), P2(3, :), P2(1, :)) < 0;
    if b1 == b2 && b2 == b3
        flag = true;
        return;
    end
end


function result = sign(p1, p2, p3)
    %判断p1点在p2和p3点构成的线段的一侧. 通过sign函数的正负可以区分在两侧.
    result =  (p1(1) - p3(1)) * (p2(2) - p3(2)) - (p2(1) - p3(1)) * (p1(2) - p3(2));
end
    % *******************************************************************
end

