


class USV_wrapper:
    def wrapper(self):
        self.running = True
        mission = input(
            f"""
            Choose the USV mission (wpts or test):
            """)
        while self.running ==True:
            if mission == "test":
                self.test()
            elif mission == "wpts":
                self.go_to_waypoints()
