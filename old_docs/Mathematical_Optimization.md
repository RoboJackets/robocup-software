
# Mathematical Optimization

Mathematical optimization involves the maximization or minimization of a function through clever choices of the input variables and the analysis of the the output. They are mostly iterative as the underlying function is not easily analysed.

Below is the comparison of the optimization algorithms that we use and their general use cases.

## Gradient Ascent

Gradient Ascent involves using the first order derivative to choose the next point. It will then move in the direction of the fastest increasing or decreasing gradient. As it passes the maximum, the step size decreases. Once the gradient is close enough to zero or the step size is small enough, the optmization algorithm exits.

Pros:
 * Very good at finding the the optimum point of a smooth unimodal function.
 * Parallelizable
 * Various configurable attributes to speed up convergence

Cons:
 * The gradient must be known at all points.
 * Has trouble with noisy functions
 * Can become stuck in local optimums.
 * Somewhat slow in convergence

## Parallel Gradient Ascent

Parallel Gradient Ascent combines multiple independent Gradient Ascents together and operates on them as a set. If two inputs are near to each other, the two Gradient Ascents are combined into one.

Pros:
 * Starting points can be pre-calculated to increase convergence speed
 * Can be multithreaded

Cons:
 * The general location of the critical point must be known to find the global optimum
 * Initialization is cumbersome

## Nelder-Mead

Nelder-Mead involves using a simplex to "flip" its way up a hill, shrinking and expanding when necessary. The simplex has N+1 vertices in N dimensions. For example, in the two dimensional case, the simplex is a triangle.

Pros:
 * Minimizes the number of function calls
 * Somewhat fast in convergence

Cons:
 * Finding the global or local optimum is determined by the starting step size
 * May converge to a non-stationary point
 * Has trouble with noisy functions