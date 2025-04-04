# Import any libraries required

# No imports



# The main path planning function. Additional functions, classes, 
# variables, libraries, etc. can be added to the file, but this
# function must always be defined with these arguments and must 
# return an array ('list') of coordinates (col,row).
#DO NOT EDIT THIS FUNCTION DECLARATION
def do_a_star(grid, start, end, display_message):
    #EDIT ANYTHING BELOW HERE


    # Get the size of the grid
    COL = len(grid)
    ROW = len(grid[0])

    # 'path' is initialised as an empty list
    path = []

    # 'open_path' is initialised as an empty list rather than a set as 
    # it requires sorting and therefore needs to be ordered to use .sort()
    open_path = []

    # 'closed_path' is initialised as a set as it does not need to be ordered and
    # by storing closed_path as a set, it is checked faster than a list construct
    # which improves the algorithm's speed
    closed_path = set()

    # 'end_node' is initialised with no parent node and a position equal
    # to the given 'end' position
    end_node = Node(None, end)

    # 'start_node' is initialised with no parent node and a position equal
    # to the given 'start' position
    start_node = Node(None, start)

    # The heuristic cost is calculated using the start node's position
    # and the end node's position, using the 'calc_h' function
    start_node.h_cost = calc_h(start_node.node_position, end_node.node_position)
    start_node.function_cost = start_node.g_cost + start_node.h_cost
    open_path.append(start_node) # Fully initialised start node is added to the open_path list

    """
    While the open_path list is not empty (open_path = 1), the open path is sorted with a custom key function so that
    the node with the lowest function cost is at the start of the list. The 'current_node' is
    then set as the lowest function cost node and that node is removed from the open_path list as it has been
    checked. The position of the current node is added to the closed_path set.
    """

    while open_path:
        open_path.sort(key=retrieve_f_cost) # Sorted open_path using retrieve_f_cost() key
        current_node = open_path[0] # Current node selected as node with lowest function cost
        del open_path[0] # Visited open_path node is deleted from the open_path list
        closed_path.add(current_node.node_position) # Current node is added to the closed_path set

        """
        The current node is checked against the end node position - if the end is the same as the current node
        then the path is retrieved using get_path() with the current node and path as parameters. This path is
        then displayed on the GUI to allow the user to see the full path. The function then ends using 'return path'
        which passes the complete path to the GUI for path creation.
        """

        if current_node.node_position == end:
            # Stores path from start to end using get_path()
            path = get_path(current_node, path)
            # Prints the entire path
            display_message("Full path: " + str(path))
            # Ends the function and provides the complete path to the GUI
            return path

        # All valid neighbour node positions are retrieved using next_position_options(),
        # reducing the looping requirement
        neighbours = next_position_options(current_node.node_position, grid)

        """
        For each provided neighbour position a node is created and has the neighbour position assigned
        to it. The position of this node is compared with the node positions in the closed list to 
        check if it already exists. If it exists, the neighbour node is skipped, otherwise the g_cost,
        h_cost and f_cost are calculated for the node using calc_g(), calc_h() and combining the g and h costs.
        """

        for neighbour_pos in neighbours:

            new_node = Node(current_node, neighbour_pos) # New node created with neighbour node position and current_node parent

            if new_node.node_position in closed_path:
                continue
            
            # Calculation of g, h and function costs for the new node
            # using associated functions and addition
            new_node.g_cost = calc_g(current_node)
            new_node.h_cost = calc_h(neighbour_pos, end)
            new_node.function_cost = new_node.g_cost + new_node.h_cost

            """
            Each neighbour node is then checked for in the open_path list using a comparison between its position
            and the positions in the list as each node is assigned a single position. If the neighbour node is already
            in the open_path list and its function cost is greater than or equal to the compared node, it is skipped.
            If the node does not appear in the closed_path set, or open_path list with an equal/larger function cost,
            it is appended to the open_path list.
            """

            # Comparison of new node with nodes in open_path as described above
            for node in open_path:
                if new_node.node_position == node.node_position and new_node.g_cost >= node.g_cost:
                    break
            else:
                open_path.append(new_node)

    # Send the path points back to the gui to be displayed
    #FUNCTION MUST ALWAYS RETURN A LIST OF (col,row) COORDINATES
    return path


# A class of node was created in order to store the position, g, h and function costs
# in a single object type. The parent node position is also stored in the node class.
class Node:
    def __init__(self, parent_node=None, node_position=None):
        # Node position initialised as None by default but can be passed an argument
        self.node_position = node_position
        # Costs intialised as zero
        self.function_cost = 0
        self.g_cost = 0
        self.h_cost = 0
        # Parent node initialised as None by default but can be passed an argument
        self.parent_node = parent_node

# A custom key subfunction which returns the function cost of a node and
# is used to sort nodes in the open_list by ascending function cost
def retrieve_f_cost(node):
    return node.function_cost

# Subfunction used to calculate new g_cost value which is the previous value +
# the distance between the previous node and passed node (1)
def calc_g(node):
    return node.g_cost + 1

# Subfunction used to calculate the heuristic function which is the Euclidean distance between
# the passed node's position and the end position
def calc_h(node_pos, end_position):
    return (((node_pos[0] - end_position[0])**2) + ((node_pos[1] - end_position[1])**2))**0.5

"""
Subfunction used to return the completed path from the start node to the end node.
The path is cleared to ensure there are no unwanted nodes and then until the parent node
value is equal to 'None' (indicating the start position has been reached), the current
node position is appended to the path and the current node becomes its parent node. The
path is then reversed to ensure it is from start to end and returned.
"""
def get_path(current_node, path):
    path.clear() # Ensuring no unwanted items are in the path list
    while current_node is not None:
        # Current node is appended to the path list and then turned into its parent node
        path.append((current_node.node_position[0], current_node.node_position[1]))
        current_node = current_node.parent_node
    path.reverse() # Ensures path list is from start to end
    return path

# Subfunction that checks the passed node is both within the
# boundaries of the grid and is not a blocked grid space
def valid_node(node, cols, rows, grid):
    return (node[0] >= 0 and node[0] < cols) and (node[1] >= 0 and node[1] < rows) and (grid[node[0]][node[1]] == 1)

# Subfunction that produces horizontal and vertical adjacent neighbour grid spaces,
# only keeping ones that are valid from valid_node()
def next_position_options(path_point, grid):
    # List of potential adjacent position using the passed node's
    # position and a modifier for each direction
    option_list = [(path_point[0] + 1, path_point[1]),
                   (path_point[0] - 1, path_point[1]),
                   (path_point[0], path_point[1] + 1),
                   (path_point[0], path_point[1] - 1)
                   ]
    
    # Neighbour node position list initialised as empty and valid node positions are appended to it
    neighbours = []
    for neighbour_option in option_list:
        if valid_node(neighbour_option, len(grid), len(grid[0]), grid):
            neighbours.append(neighbour_option)
    
    return neighbours


#end of file