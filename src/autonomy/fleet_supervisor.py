from src.autonomy.guidance.global_planner import *



class supervisor:
    def __init__(self, world_model, mission_info, asset_dict):
        self.world_model = world_model
        self.mission_info = mission_info
        self.asset_dict = asset_dict
        self.asset_init = False


    def generate_global_path(self, asset_id, wp_goal):
        x_asset, y_asset = self.asset_dict[asset_id].do.current_x_y_position()
        start_ind = self.world_model.get_ind_from_coord(x_asset, y_asset)
        goal_ind =  self.world_model.get_ind_from_coord(*wp_goal)
        path_ind = A_star(self.world_model.costmap, start_ind, goal_ind)

        path = [tuple(map(float, self.world_model.gridmap[i, j])) for (i, j) in path_ind]

        return path + [wp_goal]

    def fulfill_mission(self):

        if self.mission_info["type"] == "go to waypoints":

            if self.asset_init == False:
                asset = list(self.asset_dict.values())[0]
                asset.do.waypoints = self.mission_info["waypoints"]
                asset.do.arrival_radius = self.mission_info["arrival_radius"]
                asset.do.wp_index = 0
                self.asset_init = True

            if self.asset_init == True:
                asset = list(self.asset_dict.values())[0]
                asset.do.brain_step()

        if self.mission_info["type"] == "a_star":

            asset_id = "usv1"
            
            if self.asset_init == False:

                wp_goal = self.mission_info["waypoints"][-1]
                path = self.generate_global_path(asset_id,wp_goal)

                asset = self.asset_dict[asset_id]
                asset.do.waypoints = path
                asset.do.arrival_radius = self.mission_info["arrival_radius"]
                asset.do.wp_index = 0
                self.asset_init = True

            if self.asset_init == True:
                asset = self.asset_dict[asset_id]
                asset.do.brain_step()
    pass

                    

            
            


    