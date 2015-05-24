function x = norm_dim(X,dim)

[m,n] = size(X);

if dim == 1
    x = zeros(m,1);
    for i = 1:m
        x(i) = norm(X(i,:));
    end
elseif dim == 2
    x = zeros(1,n);
    for i = 1:n
        x(i) = norm(X(:,i));
    end
end