from neo4j import GraphDatabase
from time import time
import json
import math 

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def norm(self):
        return math.sqrt(self.x ** 2 + self.y ** 2)
    
    def normalize(self):
        return self / self.norm()
    
    def __add__(self, p):
        return Point(self.x + p.x, self.y + p.y)

    def __sub__(self, p):
        return Point(self.x - p.x, self.y - p.y)
    
    def __mul__(self, k):
        return Point(k * self.x, k * self.y)
    
    def __truediv__(self, k):
        return Point(self.x / k, self.y / k)


def dist(A: Point, B: Point):
    return (B - A).norm()


class Path:
    def __init__(self, p):
        self.p = p

    def length(self):
        sum = 0

        for i in range(1, len(self.p)):
            sum += dist(self.p[i-1], self.p[i])

        return sum
    
    def __getitem__(self, i):
        return self.p[i]
    
    def __iter__(self):
        return iter(self.p)
    
    def __len__(self):
        return len(self.p)
    

class Params:
    def __init__(self, max_samples_per_waypoint, lookahead, sampling_period, cutoff_distance, goal_cutoff_distance, mass, max_velocity, goal_attraction, valley_attraction, A, B):
        self.max_samples_per_waypoint = max_samples_per_waypoint
        self.lookahead = lookahead
        self.sampling_period = sampling_period
        self.cutoff_distance = cutoff_distance
        self.goal_cutoff_distance = goal_cutoff_distance
        self.mass = mass
        self.max_velocity = max_velocity
        self.goal_attraction = goal_attraction
        self.valley_attraction = valley_attraction
        self.A = A
        self.B = B

class CTRParams:
    def __init__(self, alpha, robot_radius, turning_radius, k_d, k_o, k_lambda, theta_start, theta_goal, force_theta_goal, interpolation_window_ratio):
        self.alpha = alpha
        self.robot_radius = robot_radius
        self.turning_radius = turning_radius
        self.k_d = k_d
        self.k_o = k_o
        self.k_lambda = k_lambda
        self.theta_start = theta_start
        self.theta_goal = theta_goal
        self.force_theta_goal = force_theta_goal
        self.interpolation_window_ratio = interpolation_window_ratio

class Neo4jConnection:
    def __init__(self, uri, user, password):
        self._driver = GraphDatabase.driver(uri, auth=(user, password))

    def close(self):
        self._driver.close()

    def run_query(self, query, parameters=None):
        with self._driver.session() as session:
            result = session.run(query, parameters)
            return [record for record in result]  # Result is consumed
        
def nearest_neighbor_neo4j_query(p, var_name):
    return f"""CALL {{WITH point({{x: {p.x}, y: {p.y}}}) AS robotLocation
    MATCH (n: Node)
    WHERE n.free = true
    WITH n, robotLocation, point({{x: n.x, y: n.y}}) AS nodeLocation
    WITH n, robotLocation, point.distance(robotLocation, nodeLocation) AS distance
    RETURN n.rawIdx AS {var_name}
    ORDER BY distance
    LIMIT 1}}
    """

class GoalPotential:
    def __init__(self, path, waypoint_attraction, window_size, lookahead):
        self.path = path
        self.waypoint_attraction = waypoint_attraction
        self.window_size = window_size
        self.lookahead = lookahead

    def extract_window(self, p):
        # Find nearest waypoint and its index
        p_nearest, index = self.path[0], 0

        for i in range(len(self.path)):
            if dist(p, self.path[i]) < dist(p, p_nearest):
                p_nearest, index = self.path[i], i

        index = min([index + self.lookahead, len(self.path) - 1])
        
        # Extract window
        adjusted_window_size = min([self.window_size, len(self.path) - index])
        window = [Point(0, 0) for _ in range(adjusted_window_size)]

        for i in range(adjusted_window_size):
            if i + index < len(self.path):
                window[i] = self.path[index + i]

        return window


    def get_potential(self, p):
        force, potential = Point(0, 0), 0
        window = self.extract_window(p)

        for attractor in window:
            distance = dist(p, attractor)
            potential += self.waypoint_attraction * distance
            force = force + (attractor - p) * self.waypoint_attraction / max([distance, 1e-6])

        return potential / len(window), force / len(window)
    
class MapPotential:
    def __init__(self, conn, scale, alpha, dist_obs_max):
        self.conn = conn
        self.scale = scale
        self.alpha = alpha
        self.dist_obs_max = dist_obs_max

    def nearest_obstacle_point(self, p):
        query = f"""WITH point({{x: {p.x}, y: {p.y}}}) AS robotLocation
        MATCH (n: Node)
        WHERE n.free = false
        WITH n, robotLocation, point({{x: n.x, y: n.y}}) AS nodeLocation
        WITH n, robotLocation, point.distance(robotLocation, nodeLocation) AS distance
        RETURN n.x as x, n.y as y
        ORDER BY distance
        LIMIT 1
        """
        result = self.conn.run_query(query)
        return Point(result[0]['x'], result[0]['y'])
    
    def nearest_voronoi_skeleton_point(self, p):
        query = f"""WITH point({{x: {p.x}, y: {p.y}}}) AS robotLocation
        MATCH (u: Node)
        WHERE u.free = true
        MATCH (v: Node)-[:NEIGHBORING]->(u), (v)-[:MEMBER_OF]->(vv), (u)-[:MEMBER_OF]->(vu)
        WHERE v.free = true and vv <> vu
        
        WITH u, robotLocation, point({{x: u.x, y: u.y}}) AS nodeLocation
        WITH u, robotLocation, point.distance(robotLocation, nodeLocation) AS distance
        RETURN u.x as x, u.y as y
        ORDER BY distance
        LIMIT 1
        """
        result = self.conn.run_query(query)
        return Point(result[0]['x'], result[0]['y'])

    def get_potential(self, p):
        p_obs = self.nearest_obstacle_point(p)
        obs_dist = dist(p, p_obs)

        p_vor = self.nearest_voronoi_skeleton_point(p)
        vor_dist = dist(p, p_vor)

        if obs_dist > self.dist_obs_max:
            potential = 0
        else:
            potential = self.scale * (self.alpha / (self.alpha + obs_dist)) * (vor_dist / (obs_dist + vor_dist)) *\
                    ((obs_dist - self.dist_obs_max) ** 2 / (self.dist_obs_max ** 2))

        nabla_obs = \
            (self.alpha / (self.alpha + obs_dist)) * (vor_dist / (obs_dist + vor_dist)) *\
            ((obs_dist - self.dist_obs_max) / (self.dist_obs_max ** 2)) *\
            (-(obs_dist - self.dist_obs_max) / (self.alpha + obs_dist) - (obs_dist - self.dist_obs_max) / (obs_dist + vor_dist) + 2)
        
        nabla_vor = (self.alpha / (self.alpha + obs_dist)) *\
                       ((obs_dist - self.dist_obs_max) ** 2 / (self.dist_obs_max ** 2)) * \
                       (obs_dist / ((obs_dist + vor_dist) ** 2));

        force = ((p - p_obs).normalize() * nabla_obs + (p - p_vor).normalize() * nabla_vor) * (-self.scale)

        return potential, force



def main():
    uri = 'neo4j://localhost:7687'
    user = 'neo4j'
    password = 'flame-chemist-vienna-nova-student-5397'
    conn = Neo4jConnection(uri, user, password)

    # Load json
    json_file = open('json/config.json')
    data = json.load(json_file)

    # Define start and goal nodes
    start = Point(data['pathPlanning']['start']['x'], data['pathPlanning']['start']['y'])
    goal = Point(data['pathPlanning']['goal']['x'], data['pathPlanning']['goal']['y'])

    params = Params(
         data['socialForce']['linearParams']['maxSamplesPerWaypoint'],
         data['socialForce']['linearParams']['lookahead'],
         data['socialForce']['linearParams']['samplingPeriod'],
         data['socialForce']['linearParams']['cutoffDistance'],
         data['socialForce']['linearParams']['goalCutoffDistance'],
         data['socialForce']['linearParams']['mass'],
         data['socialForce']['linearParams']['maxVelocity'],
         data['socialForce']['linearParams']['goalAttraction'],
         data['socialForce']['linearParams']['valleyAttraction'],
         data['socialForce']['linearParams']['A'],
         data['socialForce']['linearParams']['B']
    )

    ctr_params = CTRParams(
            data['socialForce']['rotationParams']['alpha'],
            data['socialForce']['rotationParams']['robotRadius'],
            data['socialForce']['rotationParams']['turningRadius'],
            data['socialForce']['rotationParams']['k_d'],
            data['socialForce']['rotationParams']['k_o'],
            data['socialForce']['rotationParams']['k_lambda'],
            data['socialForce']['rotationParams']['thetaStart'],
            data['socialForce']['rotationParams']['thetaGoal'],
            data['socialForce']['rotationParams']['forceThetaGoal'],
            data['socialForce']['rotationParams']['interpolationWindowRatio']
    )

    # Get basic plan via database queries (Dijkstra)
    print('Planning skeleton path...')
    skeleton_path_start = time()
    skeleton_path_query = nearest_neighbor_neo4j_query(start, 'startNodeIdx') + '\n' + nearest_neighbor_neo4j_query(goal, 'goalNodeIdx') + '\n'
    skeleton_path_query += """MATCH (u:Node)
    WHERE u.rawIdx = startNodeIdx
    MATCH (v:Node)
    WHERE v.rawIdx = goalNodeIdx
    MATCH p = shortestPath((u)-[:NEIGHBORING*]->(v))
    RETURN nodes(p) as pathNodes"""
    print(skeleton_path_query)

    result = conn.run_query(skeleton_path_query)
    skeleton_path = Path([Point(node['x'], node['y']) for node in result[0]['pathNodes']])
    skeleton_path_end = time()
    print(f'Elapsed time: {skeleton_path_end - skeleton_path_start} [s]')

    # Path smoothing
    path_length = skeleton_path.length()
    velocity = [None in range(1)]
    position = [None in range(1)]
    theta = [None in range(1)]
    omega = [None in range(1)]
    
    goal_potential_function = GoalPotential(
        skeleton_path, 
        data['potentialField']['drivingPotential']['waypointAttraction'],
        data['potentialField']['drivingPotential']['lookaheadPotentialParams']['windowSize'],
        data['potentialField']['drivingPotential']['lookaheadPotentialParams']['lookahead']
    )

    map_potential_function = MapPotential(
        conn,
        data['potentialField']['mapPotential']['voronoiFieldPotentialParams']['scale'],
        data['potentialField']['mapPotential']['voronoiFieldPotentialParams']['alpha'],
        data['potentialField']['mapPotential']['voronoiFieldPotentialParams']['distObsMax']
    )

    position[0] = skeleton_path[0]
    velocity[0] = Point(0, 0)
    theta[0] = ctr_params.theta_start
    omega[0] = 0

    # Leapfrog integration    
    for i in range(100000):
        print(f'Iteration {i}...\n')
        iter_start = time()

        velocity.append(Point(0, 0))
        position.append(Point(0, 0))
        theta.append(0)
        omega.append(0)

        # Get potential values via database queries
        goal_potential, force_goal = goal_potential_function.get_potential(position[i - 1])
        map_potential, force_map = map_potential_function.get_potential(position[i - 1])
        potential = goal_potential + map_potential
        force_global = force_goal + force_map

        # Decompose force along direction of motion
        r_forward = Point(math.cos(theta[i - 1]), math.sin(theta[i - 1]))
        r_orthogonal = Point(-r_forward.y, r_forward.x)

        force = Point(0, 0)
        force.x = r_forward.x * force_global.x + r_forward.y * force_global.y;
        ctr_params.k_o * (r_orthogonal.x * force_map.x + r_orthogonal.y * force_map.y) - ctr_params.k_d * velocity[i - 1].y

        # Compute k_theta and k_omega
        moment_of_inertia = 0.5 * params.mass * ctr_params.robot_radius ** 2
        k_theta = moment_of_inertia * ctr_params.k_lambda * force.norm()
        k_omega = moment_of_inertia * (1 + ctr_params.alpha) * math.sqrt(ctr_params.k_lambda * force.norm() / ctr_params.alpha)

        # Compute torque
        theta_ref = math.atan2(force_goal.y, force_goal.x)
        diff_theta = theta[i - 1] - theta_ref

        if abs(diff_theta) > math.pi:
            correction_sign = -1 if diff_theta > math.pi else 1
            diff_theta = correction_sign * (2 * math.pi - math.abs(diff_theta))

        torque = -k_theta * diff_theta - k_omega * omega[i - 1]
        alpha = torque / moment_of_inertia
        omega[i] = omega[i - 1] + alpha * params.sampling_period
        theta[i] = theta[i - 1] + omega[i] * params.sampling_period

        # Adjust for atan2 range
        if theta[i] > math.pi:
            theta[i] -= 2 * math.pi
        
        if theta[i] < -math.pi:
            theta[i] += 2 * math.pi

        # Compute acceleration, taking into account flow resistance
        flow_resistance = potential / (params.max_velocity ** 2);
        acceleration = Point(0, 0)
        v_sign = 1 if velocity[i - 1].x > 0 else -1

        flow_resistance_factor = v_sign * flow_resistance * velocity[i - 1].norm() ** 2
        acceleration.x = (force.x - flow_resistance_factor) / params.mass
        acceleration.y = (force.y) / params.mass

        # Update velocity
        velocity[i] = velocity[i - 1] + acceleration * params.sampling_period

        # Rotate velocity
        v_global = Point(0, 0)
        v_global.x = velocity[i].x * math.cos(theta[i - 1]) - velocity[i].y * math.sin(theta[i - 1])
        v_global.y = velocity[i].x * math.sin(theta[i - 1]) + velocity[i].y * math.cos(theta[i - 1])

        # Update position
        position[i] = position[i - 1] + v_global * params.sampling_period;

        # Termination condition
        if dist(goal, position[i]) < params.goal_cutoff_distance:
            break

        iter_end = time()
        print(f'Elapsed time: {iter_end - iter_start} [s]')

    conn.close()
    output = (position, theta, [v.norm() for v in velocity], omega)
    return output


if __name__ == '__main__':
    main()
