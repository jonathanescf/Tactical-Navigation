import numpy as np
import math
import matplotlib.pyplot as plt




def A_star(cost_map:np.ndarray, start_ind:tuple, goal_ind:tuple, blocked_threshold:int = 255,  w = 10):

    # g_score: calculated path cost from start to current
    # h_score: estimated cost from current to end. euclidian distance
    # f_score: g_score + h_score

    if cost_map[start_ind] >= blocked_threshold: print("start node is an obstacle")
    if cost_map[goal_ind] >= blocked_threshold: print("end node is an obstacle")


    open_list = [start_ind]
    closed_set = set()
    g_score : dict[tuple, float] = {start_ind:0.0}
    f_score : dict[tuple, float] = {start_ind:math.hypot(goal_ind[0]-start_ind[0],
                                                         goal_ind[1]-start_ind[1])}
    parent : dict[tuple, tuple]= {start_ind:None}

    def reconstructed_path(current_ind):
        path = []
        while current_ind is not None:
            path.append(current_ind)
            current_ind = parent[current_ind]
        return path[::-1]

    while open_list:
        current_ind = open_list.pop() 

        # Check if goal_node is reached
        if current_ind == goal_ind:
            return reconstructed_path(current_ind)

        closed_set.add(current_ind)

        # Check all neighboring nodes
        potential_nb_list = [((current_ind[0] + di, current_ind[1] + dj), math.hypot(di, dj))
                          for di in (-1, 0, 1) for dj in (-1, 0, 1)
                          if (di, dj) != (0,0)]
        
        for potential_nb, nb_dist in potential_nb_list:
            
            if potential_nb[0] < 0 or potential_nb[1] < 0 or potential_nb[0] >= cost_map.shape[0] or potential_nb[1] >= cost_map.shape[1]:
                continue
            else:
                nb_ind = (potential_nb[0], potential_nb[1])

            if nb_ind in closed_set or cost_map[nb_ind] >= blocked_threshold:
                continue

            tentative_g = g_score[current_ind] + nb_dist * (1 + 10 *cost_map[nb_ind])

            if nb_ind not in open_list:
                f_score[nb_ind] = tentative_g + math.hypot(goal_ind[0] - nb_ind[0],
                                                           goal_ind[1] - nb_ind[1])
                # ADD TO OPEN LIST WITH RESPECT TO F SCORE VALUE
                open_list.append(nb_ind)
                pass
            
            elif tentative_g >= g_score[nb_ind]:
                continue

            # New path is he best so far so we update the neighbor properties
            parent[nb_ind] = current_ind
            g_score[nb_ind] = tentative_g

    #open_list is empty and no node equals goal_node: No path available.
    return None



if __name__ == "__main__":

    map_test = np.array([
        [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
        [1, 1, 0, 1, 0, 1, 1, 1, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
        [0, 1, 1, 1, 1, 1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 1, 0, 1, 0],
        [0, 1, 1, 1, 1, 0, 1, 0, 0, 0],
        [0, 1, 0, 0, 1, 0, 1, 1, 1, 1],
        [0, 1, 0, 0, 1, 0, 0, 0, 0, 0],
        [1, 1, 1, 0, 1, 1, 1, 1, 1, 0],
        [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
    ])

    start, goal = (0, 0), (9, 0)

    path = A_star(map_test, start, goal, 1)

    plt.figure(figsize=(5, 5))
    plt.imshow(map_test, cmap="gray_r")
    plt.xticks(range(10)); plt.yticks(range(10))
    plt.gca().set_xticks(np.arange(-.5, 10, 1), minor=True)
    plt.gca().set_yticks(np.arange(-.5, 10, 1), minor=True)
    plt.grid(which="minor", color="gray", linewidth=0.5)
    plt.tick_params(which="minor", length=0)

    if path:
        plt.plot([p[1] for p in path], [p[0] for p in path], "r-o", linewidth=2, markersize=4)
    else:
        print("Aucun chemin trouvé")

    plt.show()
