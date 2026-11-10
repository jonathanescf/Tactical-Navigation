import numpy as np
import math
import heapq
import matplotlib.pyplot as plt




def A_star(cost_map:np.ndarray, start_ind:tuple, goal_ind:tuple, blocked_threshold:int = 255,  w = 10):

    # g_score: calculated path cost from start to current
    # h_score: estimated cost from current to end. euclidian distance
    # f_score: g_score + h_score

    if cost_map[start_ind] >= blocked_threshold: 
        print("start node is an obstacle")
        return None
    if cost_map[goal_ind] >= blocked_threshold: 
        print("end node is an obstacle")
        return None

    g_score_dict : dict[tuple, float] = {start_ind:0.0}
    h_start_score = math.hypot(goal_ind[0]-start_ind[0], goal_ind[1]-start_ind[1])
    f_start_score = g_score_dict[start_ind] + h_start_score
    
    open_heap = [(f_start_score, start_ind)]
    closed_set = set()
    parent_dict : dict[tuple, tuple]= {start_ind:None}

    def reconstructed_path(current_ind):
        path = []
        while current_ind is not None:
            path.append(current_ind)
            current_ind = parent_dict[current_ind]
        return path[::-1]

    while open_heap:
        f_current_score, current_ind = heapq.heappop(open_heap) # selects the lement with the lowest f_score

        #checks if wa already treated. (one node can appera multiple times in the heap with old outdated f_scores)
        if current_ind in closed_set:
            continue

        # Check if goal_node is reached
        if current_ind == goal_ind:
            return reconstructed_path(current_ind)

        closed_set.add(current_ind)

        # Check all neighboring nodes
        potential_nb_list = [((current_ind[0] + di, current_ind[1] + dj), math.hypot(di, dj))
                          for di in (-1, 0, 1) for dj in (-1, 0, 1)
                          if (di, dj) != (0,0)]
        
        for potential_nb, nb_dist in potential_nb_list:

            if 0 <= potential_nb[0] < cost_map.shape[0] and  0 <= potential_nb[1] < cost_map.shape[1]:
                nb_ind = (potential_nb[0], potential_nb[1])
            else:
                continue

            # Was nb already set as current? is it blocked?
            if nb_ind in closed_set or cost_map[nb_ind] >= blocked_threshold:
                continue

            tentative_g = g_score_dict[current_ind] + nb_dist * (1 + w *cost_map[nb_ind])

            # Was nb already seen ? (seen => inside open_heap  => inside g_score_dict)
            # if so, is the new g_score better then the existing one?
            if nb_ind not in g_score_dict or tentative_g < g_score_dict[nb_ind]: # if first true, second not executed so no error.
                parent_dict[nb_ind] = current_ind
                g_score_dict[nb_ind] = tentative_g
                h_score_nb_ind = math.hypot(goal_ind[0] - nb_ind[0], goal_ind[1] - nb_ind[1])
                f_score_nb_ind = tentative_g + h_score_nb_ind
                heapq.heappush(open_heap, (f_score_nb_ind, nb_ind))

    #open_list is empty and no node equals goal_node: No path available.
    return None



if __name__ == "__main__":

    X = 255  # obstacle
    map_test = np.array([
    [0, X, 0, 0, 0, X, 0, 0, 0, X, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, X, 0, X, 0, 0, 0, 0, 0, X, 0, 0, 0, 0, 0, 0, 0, 0, 0, X, 0, 0, 0, 0, 0, X, 0, 0, 0, 0],
    [0, X, 0, X, 0, X, 0, X, 0, X, 0, X, X, X, X, X, X, X, X, X, 0, X, 0, X, 0, X, X, X, 0, X, 0, X, X, X, X, X, 0, X, 0, X, 0, X, X, X, 0, X, 0, X, 0, 0],
    [0, X, 0, X, 0, X, 0, X, 0, X, 0, 0, 0, 0, 0, 0, 0, 0, 0, X, 0, X, 0, X, 0, X, 0, 0, 0, X, 0, 0, 0, X, 0, 0, 0, X, 0, X, 0, 0, 0, 0, 0, X, 0, X, 0, 0],
    [0, X, 0, X, 0, X, 0, X, 0, X, X, X, 0, X, 0, X, X, X, 0, X, 0, X, 0, X, 0, X, 0, X, X, X, X, X, 0, X, 0, X, X, X, 0, X, 0, X, X, X, 0, X, X, X, 0, 0],
    [0, 0, 0, X, 0, 0, 0, X, 0, X, 0, 0, 0, X, 0, 0, 0, X, 0, 0, 0, 0, 0, 0, 0, X, 0, 0, 0, 0, 0, 0, 0, X, 0, X, 0, X, 0, X, 0, 0, 0, X, 0, X, 0, 0, 0, 0],
    [X, X, X, X, X, X, X, X, 0, X, 0, X, X, X, X, X, 0, X, X, X, X, X, 0, X, X, X, X, X, X, X, 0, X, X, X, 0, X, 0, X, 0, X, 0, X, 0, X, 0, X, 0, X, X, 0],
    [0, 0, 0, 0, 0, 0, 0, X, 0, 0, 0, X, 0, 0, 0, X, 0, 0, 0, 0, 0, 0, 0, 0, 0, X, 0, 0, 0, X, 0, X, 0, 0, 0, X, 0, 0, 0, X, 0, X, 0, X, 0, X, 0, 0, 0, 0],
    [0, X, 0, X, X, X, 0, X, X, X, X, X, 0, X, 0, X, X, X, X, X, 0, X, X, X, X, X, 0, X, 0, X, X, 0, 0, X, X, X, 0, X, X, X, 0, X, 0, X, 0, X, X, X, 0, 0],
    [0, 0, 0, X, 0, X, 0, X, 0, 0, 0, 0, 0, X, 0, 0, 0, 0, 0, X, 0, 0, 0, X, 0, 0, 0, X, 0, 0, 0, 0, 0, X, 0, 0, 0, X, 0, X, 0, X, 0, X, 0, 0, 0, 0, 0, 0],
    [X, X, 0, X, 0, X, 0, X, 0, X, X, X, X, X, X, X, 0, X, 0, X, X, X, 0, X, 0, X, X, X, X, X, X, X, X, X, 0, X, X, X, 0, X, 0, X, 0, X, X, X, X, X, X, 0],
    [0, 0, 0, X, 0, 0, 0, X, 0, X, 0, X, 0, 0, 0, 0, 0, X, 0, X, 0, X, 0, 0, 0, X, 0, 0, 0, 0, 0, 0, 0, X, 0, X, 0, 0, 0, X, 0, X, 0, 0, 0, X, 0, 0, 0, 0],
    [0, X, X, X, X, X, X, X, 0, X, 0, X, 0, X, X, X, X, X, 0, X, 0, X, X, X, X, X, 0, X, X, X, 0, X, X, X, 0, X, 0, 0, 0, X, 0, X, X, X, 0, X, 0, X, 0, 0],
    [0, 0, 0, X, 0, 0, 0, X, 0, 0, 0, 0, 0, 0, 0, X, 0, X, 0, X, 0, X, 0, 0, 0, 0, 0, X, 0, 0, 0, X, 0, 0, 0, X, 0, X, 0, X, 0, 0, 0, X, 0, 0, 0, X, 0, 0],
    [0, X, 0, X, 0, X, 0, X, 0, X, X, X, X, X, 0, X, 0, X, 0, X, 0, X, 0, 0, 0, X, X, X, 0, X, X, X, 0, X, X, X, 0, X, X, X, 0, X, 0, X, X, X, X, X, 0, 0],
    [0, X, 0, X, 0, X, 0, X, 0, X, 0, 0, 0, X, 0, X, 0, X, 0, X, 0, X, 0, X, 0, 0, 0, 0, 0, 0, 0, X, 0, X, 0, 0, 0, X, 0, 0, 0, 0, 0, X, 0, 0, 0, 0, 0, 0],
    [0, X, 0, X, 0, X, 0, X, 0, X, 0, X, 0, X, 0, X, 0, X, 0, X, 0, X, 0, X, 0, X, X, X, X, X, X, X, 0, X, X, 0, 0, X, 0, X, X, X, X, X, 0, X, X, X, 0, 0],
    [0, X, 0, 0, 0, X, 0, 0, 0, X, 0, X, 0, 0, 0, X, 0, X, 0, 0, 0, X, 0, X, 0, 0, 0, X, 0, 0, 0, X, 0, 0, 0, X, 0, 0, 0, X, 0, 0, 0, 0, 0, X, 0, 0, 0, 0],
    [0, X, X, X, X, X, X, X, X, X, 0, X, X, X, X, X, 0, X, X, X, 0, X, 0, X, X, X, 0, X, 0, X, 0, X, X, 0, 0, X, X, X, X, X, 0, X, X, X, X, 0, 0, X, X, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, X, 0, 0, 0, X, 0, 0, 0, 0, 0, 0, 0, X, 0, 0, 0, X, 0, 0, 0, X, 0, 0, 0, X, 0, 0, 0, 0, 0, 0, 0, X, 0, 0, 0, X, 0, X, 0, 0],
    [X, X, X, X, X, X, X, X, 0, X, X, X, 0, X, X, X, X, X, X, X, 0, X, X, X, 0, X, X, X, X, X, X, X, 0, X, X, X, X, X, X, X, 0, X, 0, X, 0, X, 0, X, 0, 0],
    [0, 0, 0, X, 0, 0, 0, X, 0, X, 0, 0, 0, X, 0, 0, 0, 0, 0, 0, 0, 0, 0, X, 0, X, 0, 0, 0, 0, 0, X, 0, 0, 0, 0, 0, X, 0, 0, 0, 0, 0, X, 0, 0, 0, X, 0, 0],
    [0, X, 0, X, 0, X, 0, X, 0, 0, 0, 0, 0, X, 0, X, X, X, X, X, 0, X, X, X, 0, X, 0, X, X, X, X, 0, 0, X, X, X, 0, X, X, X, 0, X, 0, X, X, X, X, X, 0, 0],
    [0, X, 0, X, 0, X, 0, 0, 0, X, 0, X, 0, X, 0, 0, 0, X, 0, X, 0, 0, 0, X, 0, X, 0, X, 0, 0, 0, 0, 0, X, 0, X, 0, 0, 0, 0, 0, X, 0, 0, 0, 0, 0, X, 0, 0],
    [X, X, 0, X, 0, X, X, X, X, X, 0, X, X, X, X, X, 0, X, 0, X, X, X, 0, X, 0, X, 0, X, 0, X, X, X, X, X, 0, X, X, X, 0, X, X, X, X, X, X, X, 0, X, 0, 0],
    [0, 0, 0, X, 0, 0, 0, X, 0, 0, 0, X, 0, 0, 0, X, 0, 0, 0, X, 0, 0, 0, X, 0, 0, 0, X, 0, X, 0, 0, 0, 0, 0, 0, 0, X, 0, X, 0, 0, 0, 0, 0, X, 0, X, 0, 0],
    [0, X, 0, X, X, X, 0, X, 0, X, X, X, 0, 0, 0, X, 0, X, X, X, 0, X, 0, 0, X, X, 0, X, 0, X, X, X, X, X, 0, X, 0, X, 0, X, 0, X, X, X, 0, X, 0, X, 0, 0],
    [0, X, 0, 0, 0, 0, 0, X, 0, 0, 0, 0, 0, X, 0, X, 0, X, 0, 0, 0, X, 0, X, 0, X, 0, X, 0, 0, 0, 0, 0, 0, 0, X, 0, 0, 0, X, 0, 0, 0, X, 0, 0, 0, X, 0, 0],
    [0, X, 0, X, X, 0, X, X, X, X, X, X, X, X, 0, X, 0, 0, 0, 0, X, X, 0, X, 0, X, 0, X, X, X, 0, X, 0, X, 0, X, X, X, X, X, 0, X, 0, X, X, X, X, X, 0, 0],
    [0, X, 0, 0, 0, X, 0, 0, 0, 0, 0, 0, 0, X, 0, 0, 0, X, 0, X, 0, 0, 0, 0, 0, X, 0, X, 0, 0, 0, X, 0, X, 0, 0, 0, 0, 0, X, 0, X, 0, 0, 0, 0, 0, X, 0, 0],
    [0, X, X, X, 0, X, 0, X, 0, X, X, X, X, X, X, X, X, X, 0, X, X, X, X, X, X, X, 0, X, 0, X, 0, X, 0, X, X, X, X, X, X, X, 0, X, 0, X, 0, X, 0, X, 0, 0],
    [0, X, 0, 0, 0, X, 0, X, 0, X, 0, 0, 0, 0, 0, 0, 0, X, 0, 0, 0, 0, 0, 0, 0, X, 0, 0, 0, X, 0, X, 0, 0, 0, 0, 0, X, 0, 0, 0, X, 0, X, 0, X, 0, 0, 0, 0],
    [0, X, 0, X, X, X, 0, X, X, X, 0, X, 0, X, X, X, X, X, X, X, X, X, X, X, 0, X, X, X, X, X, 0, X, X, X, X, X, 0, X, 0, X, X, X, X, X, 0, X, X, X, 0, 0],
    [0, 0, 0, X, 0, 0, 0, 0, 0, X, 0, X, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, X, 0, 0, 0, 0, 0, X, 0, 0, 0, X, 0, 0, 0, X, 0, X, 0, 0, 0, 0, 0, X, 0, X, 0, 0],
    [0, X, X, X, X, X, X, X, 0, X, 0, 0, X, X, X, X, X, X, X, X, X, X, 0, X, X, X, X, X, 0, X, X, X, 0, X, 0, X, 0, X, 0, X, X, X, 0, X, X, X, 0, X, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, X, 0, X, 0, 0, 0, X, 0, 0, 0, 0, 0, X, 0, 0, 0, X, 0, 0, 0, 0, 0, X, 0, 0, 0, X, 0, X, 0, X, 0, X, 0, 0, 0, X, 0, 0, 0, X, 0, 0],
    [X, X, X, X, X, X, 0, X, 0, X, X, X, 0, X, 0, 0, X, X, 0, X, 0, X, 0, X, 0, 0, X, X, X, X, 0, X, X, X, 0, X, 0, X, 0, X, 0, X, 0, X, 0, X, X, X, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, X, 0, X, 0, 0, 0, X, 0, X, 0, 0, 0, 0, 0, X, 0, 0, 0, 0, 0, 0, 0, X, 0, 0, 0, X, 0, X, 0, X, 0, X, 0, X, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, X, X, X, X, X, X, X, 0, X, 0, X, X, X, 0, X, X, X, X, X, X, X, 0, X, X, 0, X, X, 0, X, X, X, 0, X, 0, X, 0, 0, 0, X, 0, X, 0, X, 0, X, 0, X, X, 0],
    [0, X, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, X, 0, X, 0, 0, 0, X, 0, 0, 0, 0, 0, X, 0, 0, 0, 0, 0, X, 0, 0, 0, X, 0, 0, 0, 0, 0, X, 0, X, 0, X, 0, 0, 0, 0],
    [0, X, X, X, X, X, 0, X, X, X, X, X, X, X, 0, X, 0, X, 0, X, X, X, X, X, 0, X, 0, X, X, X, X, X, 0, X, 0, X, X, X, X, X, X, X, 0, 0, 0, X, X, X, 0, 0],
    [0, 0, 0, 0, 0, X, 0, X, 0, 0, 0, 0, 0, X, 0, X, 0, X, 0, 0, 0, 0, 0, X, 0, X, 0, 0, 0, 0, 0, 0, 0, X, 0, 0, 0, 0, 0, 0, 0, X, 0, X, 0, 0, 0, 0, 0, 0],
    [X, X, X, 0, 0, X, X, X, 0, X, X, X, 0, X, 0, X, 0, X, X, X, X, X, 0, X, 0, X, X, X, X, X, X, X, 0, X, X, X, X, X, X, X, 0, X, 0, X, X, X, X, X, X, 0],
    [0, 0, 0, X, 0, 0, 0, 0, 0, X, 0, 0, 0, X, 0, X, 0, 0, 0, 0, 0, X, 0, X, 0, X, 0, X, 0, 0, 0, X, 0, X, 0, 0, 0, 0, 0, 0, 0, X, 0, X, 0, 0, 0, 0, 0, 0],
    [X, X, 0, X, X, 0, X, X, X, X, 0, X, X, X, 0, X, X, X, X, X, 0, X, 0, X, 0, X, 0, X, 0, X, 0, X, X, X, 0, X, X, X, X, X, 0, X, 0, X, X, X, 0, X, 0, 0],
    [0, 0, 0, 0, 0, X, 0, 0, 0, 0, 0, X, 0, 0, 0, 0, 0, 0, 0, 0, 0, X, 0, X, 0, 0, 0, X, 0, X, 0, 0, 0, 0, 0, X, 0, 0, 0, 0, 0, X, 0, 0, 0, 0, 0, X, 0, 0],
    [0, X, X, X, X, X, 0, X, X, X, X, X, X, X, 0, 0, X, X, X, X, X, X, 0, X, X, X, 0, X, 0, X, X, X, 0, X, X, X, X, X, X, X, X, X, X, X, X, X, X, X, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, X, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, X, 0, X, 0, X, 0, X, 0, X, 0, 0, 0, X, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, X, 0, 0],
    [0, X, X, X, X, X, X, X, X, X, 0, X, X, 0, X, X, X, X, X, X, 0, X, 0, X, 0, X, 0, X, 0, X, X, X, X, X, 0, X, X, X, X, X, X, X, 0, X, X, X, X, X, 10, 10],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, X, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, X, 0, 0, 0, X, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, X, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    ])


    n = map_test.shape[0]
    start, goal = (0, 0), (n - 1, n - 1)

    result = A_star(map_test, start, goal, blocked_threshold=255)

    plt.figure(figsize=(6, 6))
    plt.imshow(np.sqrt(map_test), cmap="gray_r")
    plt.xticks(range(n)); plt.yticks(range(n))
    plt.gca().set_xticks(np.arange(-.5, n, 1), minor=True)
    plt.gca().set_yticks(np.arange(-.5, n, 1), minor=True)
    plt.grid(which="minor", color="gray", linewidth=0.5)
    plt.tick_params(which="minor", length=0)

    if result is not None:
        path = result
        plt.plot([p[1] for p in path], [p[0] for p in path], "r-o", linewidth=2, markersize=4)
    else:
        plt.title("Aucun chemin trouvé")
        print("Aucun chemin trouvé")

    plt.show()
