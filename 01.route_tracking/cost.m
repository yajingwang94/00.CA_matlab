function cost = fun(x0, xd)
	cost = norm(x - xd, 2)^2;