from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

# Step 1: Create the data model
def create_data_model():
    data = {}
    # Example distance matrix (representing travel time or distance between locations)
    data['distance_matrix'] = [
        [0, 9, 8, 7],  # Depot (location 0)
        [9, 0, 4, 6],  # Location 1
        [8, 4, 0, 3],  # Location 2
        [7, 6, 3, 0]   # Location 3
    ]
    data['num_vehicles'] = 2  # Number of vehicles (drivers)
    data['depot'] = 0         # Starting point (Depot)
    return data

# Step 2: Create the routing model
def main():
    data = create_data_model()
    
    # Create the manager for routing nodes (locations) and vehicles
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']), data['num_vehicles'], data['depot'])
    
    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)
    
    # Create a distance callback function that returns the distance between locations
    def distance_callback(from_index, to_index):
        # Convert routing indices to distance matrix indices
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    # Register the distance callback with the routing model
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    
    # Set the cost of travel for all vehicles
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    
    # Define search parameters for the solver
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    # Solve the problem
    solution = routing.SolveWithParameters(search_parameters)

    # Step 3: Print the solution (routes for all vehicles)
    if solution:
        print_solution(manager, routing, solution, data['num_vehicles'])

# Step 4: Print the solution in a readable format
def print_solution(manager, routing, solution, num_vehicles):
    print(f'Objective: {solution.ObjectiveValue()} (total distance)')
    for vehicle_id in range(num_vehicles):
        index = routing.Start(vehicle_id)
        route_distance = 0
        route = f'Route for vehicle {vehicle_id}:\n'
        while not routing.IsEnd(index):
            route += f'{manager.IndexToNode(index)} -> '
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
        route += f'{manager.IndexToNode(index)}\n'
        print(route)
        print(f'Distance of the route: {route_distance}\n')

if __name__ == '__main__':
    main()
