def run(self):
        """ Main loop of the planner. The planner is in charge of computing the next goal based on the result of the road sign. The planner is in different modes: discovery mode, navigation mode, and recovery mode.
        The planner is in discovery mode when the robot is searching for the road sign. The planner is in navigation mode when the robot is navigating to the next goal. The planner is in recovery mode when the robot has been kidnapped and it needs to be relocated to the ideal relocation point."""
        if self.initial_pose_flag == False or self.is_kidnapped:
            return        

        if self.nav_thread is not None and not self.nav_thread.is_alive():
            # if the robot has reached the goal, then we need to join the navigation thread and set the flag to True, so that the robot can compute the next goal
            self.nav_thread.join()
            self.nav_thread = None
            self.discovery_flag = True
                
        
        if self.flag and (self.nav_thread is None or not self.nav_thread.is_alive()): # If the robot has reached the goal, or it is the first time the robot is deployed or the robot is not running any navigation task, look for the next sign road, compute the next goal and start the navigation towards the next goal
            self.flag = False #TODO change the name of the flag, it is not clear!!! 
                              # The flag is used to avoid entering in the if statement multiple times.
            # get the current pose of the robot
            # Transition to discovery mode
            
            if self.first_discovery: # TODO: Is it necessary to have the first_discovery flag?
                x = self.amcl_pose.pose.pose.position.x
                y = self.amcl_pose.pose.pose.position.y
                theta = self.get_angle(self.amcl_pose.pose.pose.orientation)
                pose = (x, y, theta)
                distance_to_goal = (self.next_goal[0] - x) ** 2 + (self.next_goal[1] - y) ** 2
                if distance_to_goal > self.rho:
                    # if the robot is too far from the goal, then we need to approach the goal
                    x, y ,angle= map(float, self.nav_goal)
                    self.navigator.startToPose(self.navigator.getPoseStamped((x, y), angle))        
                else:
                    self.nav_goal = pose # If the robot is close to the goal then the navigation goal is the current pose of the robot, so there will be no problem for the discovery mode
                self.first_discovery = False
                self.discovery_flag = True
            else:    
                # compute the navigation goal: which is a particular point on the way to the next goal where the robot has to search for the road sign
                self.get_logger().info("THe nav goal is: " + str(self.nav_goal))
                self.get_logger().info("THe action payload is: " + str(self.action_payload))
                self.get_logger().info(f"\n***********\nNEXT GOAL: {self.next_goal}\n***********")
            
                # Start the navigation in a separate thread
                x, y ,angle= map(float, self.nav_goal)
                self.nav_thread = threading.Thread(target=self.start_navigation, args=(x, y, angle))
                self.nav_thread.start()
            
        if self.discovery_flag == True and not self.is_kidnapped:
            self.get_logger().info("self.discovery_flag == True")
            x = self.amcl_pose.pose.pose.position.x
            y = self.amcl_pose.pose.pose.position.y
            theta = self.get_angle(self.amcl_pose.pose.pose.orientation)
            pose = (x, y, theta)

            self.last_goal = self.next_goal # save the last goal
            self.last_nav_goal = self.nav_goal # save the last navigation goal

            self.result = self.discovery_mode(pose)
            if self.result == "Canceled": # If the discovery mode has been canceled, then we need to abort the current iteration
                self.flag = True
                self.discovery_flag = False
                return

            self.get_logger().info("RESULT: " + str(self.result))
            
            # if the result is stop, then it means that the robot has reached the final goal
            if self.result == "stop":
                self.get_logger().info("The robot has stopped")
                self.abort()
                #TODO: VALUTARE SE DEVO FARE QUALCOSA DOPO L'ABORT
                raise ExitException("The robot has reached the final goal")

                        
            if self.result == "random":
                # take all the possible neighbors 
                self.get_logger().info("The robot has to choose a random goal")
                neighbors = self.map[self.next_goal]
                self.next_goal = random.choice(neighbors)[0]
                self.get_logger().info("The next (random) goal is: " + str(self.next_goal))
            else:
                # compute the next goal based on the result
                self.next_goal = self.action_result2goal(self.result, (x, y, theta), self.next_goal)

            # computing the next navigation goal
            x = self.amcl_pose.pose.pose.position.x
            y = self.amcl_pose.pose.pose.position.y
            
            # compute the angolar coefficient of the line that connects the next goal to the last goal. This will be the approach angle to the next goal
            approach_angle = self.get_approach_angle(self.next_goal, self.last_goal, self.amcl_pose.pose.pose.orientation)
            #pose = (x, y, approach_angle)
            
            
            intersection_points, angle = self.get_intersection_points(self.next_goal, approach_angle)
            points = self.order_by_distance(intersection_points, x, y)
            self.nav_goal = (points[0][0], points[0][1], angle)
            self.action_payload = (points[1][0],points[1][1],angle)

            self.flag = True
            self.discovery_flag = False