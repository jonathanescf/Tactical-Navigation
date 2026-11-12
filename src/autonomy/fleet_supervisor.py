from src.autonomy.guidance.global_planner import *



class supervisor:
    def __init__(self, world_grid, brains, mission_info):
        self.world_grid = world_grid
        self.mission_info = mission_info
        self.brains = brains
        self.asset_init = False


    def generate_global_path(self, asset_id, wp_goal):
        x_asset, y_asset = self.brains[asset_id].current_x_y_position()
        start_ind = self.world_grid.get_ind_from_coord(x_asset, y_asset)
        goal_ind =  self.world_grid.get_ind_from_coord(*wp_goal)
        path_ind = A_star(self.world_grid.costmap, start_ind, goal_ind)

        path = [tuple(map(float, self.world_grid.gridmap[i, j])) for (i, j) in path_ind]

        return path + [wp_goal]

    def fulfill_mission(self):

        if self.mission_info["type"] == "go to waypoints":

            if self.asset_init == False:
                asset = list(self.brains.values())[0]
                asset.waypoints = self.mission_info["waypoints"]
                asset.arrival_radius = self.mission_info["arrival_radius"]
                asset.wp_index = 0
                self.asset_init = True

            if self.asset_init == True:
                asset = list(self.brains.values())[0]
                asset.brain_step()

        if self.mission_info["type"] == "a_star":

            asset_id = "usv1"
            
            if self.asset_init == False:

                wp_goal = self.mission_info["waypoints"][-1]
                path = self.generate_global_path(asset_id,wp_goal)

                asset = self.brains[asset_id]
                asset.waypoints = path
                asset.arrival_radius = self.mission_info["arrival_radius"]
                asset.wp_index = 0
                self.asset_init = True

            if self.asset_init == True:
                asset = self.brains[asset_id]
                asset.brain_step()
        return

                    

            
            


    