import numpy as np
import factorgraph as fg

# Make an empty graph
g = fg.Graph()

# Add some discrete random variables (RVs)
def to_str(i, j):
    return str(i) + "_" + str(j)

width = 20
height = 20
for i in range(height):
    for j in range(width):
        g.rv(to_str(i, j), 2)
        print(to_str(i, j))
        if i == height/2 and j == width / 2:
            g.factor([to_str(i, j)], potential=np.array([0.1, 1.0]))
        else:
            g.factor([to_str(i, j)], potential=np.array([1.0, 0.1]))

edge_potential = np.array([
    [1.0, 0.5],
    [0.5, 1.0],
])

for i in range(height):
    for j in range(width):
        if i > 0:
            g.factor([to_str(i, j), to_str(i - 1, j)], potential=edge_potential)

        if j > 0:
            g.factor([to_str(i, j), to_str(i, j - 1)], potential=edge_potential)
        
        if i < height - 1:
            g.factor([to_str(i, j), to_str(i + 1, j)], potential=edge_potential)

        if j < width - 1:
            g.factor([to_str(i, j), to_str(i, j + 1)], potential=edge_potential)

# Run (loopy) belief propagation (LBP)
iters, converged = g.lbp(normalize=True)
# print('LBP ran for %d iterations. Converged = %r' % (iters, converged))
# print()

# Print out the final messages from LBP
# g.print_messages()
# print()

# Print out the final marginals
g.print_rv_marginals()

# import numpy as np
# import factorgraph as fg

# Make an empty graph
# g = fg.Graph()

# Add some discrete random variables (RVs)
# g.rv('a', 2)
# g.rv('b', 3)

# # Add some factors, unary and binary
# g.factor(['a'], potential=np.array([0.3, 0.7]))
# g.factor(['b', 'a'], potential=np.array([
#         [0.2, 0.8],
#         [0.4, 0.6],
#         [0.1, 0.9],
# ]))

# # Run (loopy) belief propagation (LBP)
# iters, converged = g.lbp(normalize=True)
# print 'LBP ran for %d iterations. Converged = %r' % (iters, converged)
# print

# # Print out the final messages from LBP
# g.print_messages()
# print

# # Print out the final marginals
# g.print_rv_marginals()