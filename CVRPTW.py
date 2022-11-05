from functools import partial
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import sys, os
import numpy as np
from own_tools import *

def create_data_model(parameters: list):
    """Stores the data for the problem."""
    instances = os.listdir(parameters[1])
    data = read_vrplib(parameters[1] + '/' + instances[int(parameters[2])])
    return data


# Problem Constraints #

def create_distance_evaluator(data):
    """Creates callback to return distance between points."""
    _distances = data['duration_matrix']

    def distance_evaluator(manager, from_node, to_node):
        """Returns the manhattan distance between the two nodes"""
        return _distances[manager.IndexToNode(from_node)][manager.IndexToNode(
            to_node)]

    return distance_evaluator


def create_demand_evaluator(data):
    """Creates callback to get demands at each location."""
    _demands = data['demands']

    def demand_evaluator(manager, node):
        """Returns the demand of the current node"""
        return _demands[manager.IndexToNode(node)]

    return demand_evaluator


def add_capacity_constraints(routing, data, demand_evaluator_index):
    """Adds capacity constraint"""
    capacity = 'Capacity'
    routing.AddDimension(
        demand_evaluator_index,
        0,  # null capacity slack
        data['capacity'],
        True,  # start cumul to zero
        capacity)


def create_time_evaluator(data):
    """Creates callback to get total times between locations."""

    def service_time(data, node):
        """Gets the service time for the specified location."""
        return data['demands'][node]

    def travel_time(data, from_node, to_node):
        """Gets the travel times between two locations."""
        if from_node == to_node:
            travel_time = 0
        else:
            travel_time = data['duration_matrix'][from_node][to_node]
        return travel_time

    _total_time = {}
    # precompute total time to have time callback in O(1)
    for from_node in range(len(data['duration_matrix'])):
        _total_time[from_node] = {}
        for to_node in range(len(data['duration_matrix'][0])):
            if from_node == to_node:
                _total_time[from_node][to_node] = 0
            else:
                _total_time[from_node][to_node] = travel_time(data, from_node, to_node) + service_time(data, from_node)

    def time_evaluator(manager, from_node, to_node):
        """Returns the total time between the two nodes"""
        return _total_time[manager.IndexToNode(from_node)][manager.IndexToNode(to_node)]

    return time_evaluator


def add_time_window_constraints(routing, manager, data, time_evaluator_index):
    """Add Global Span constraint"""
    time = 'Time'
    horizon = 40000
    routing.AddDimension(
        time_evaluator_index,
        horizon,  # allow waiting time
        horizon,  # maximum time per vehicle
        False,  # don't force start cumul to zero since we are giving TW to start nodes
        time)

    time_dimension = routing.GetDimensionOrDie(time)
    # Add time window constraints for each location except depot
    # and 'copy' the slack var in the solution object (aka Assignment) to print it
    for location_idx, time_window in enumerate(data['time_windows']):
        if location_idx == 0:
            continue
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])
        routing.AddToAssignment(time_dimension.SlackVar(index))

    # Add time window constraints for each vehicle start node
    # and 'copy' the slack var in the solution object (aka Assignment) to print it
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        time_dimension.CumulVar(index).SetRange(data['time_windows'][0][0],
                                                data['time_windows'][0][1])
        routing.AddToAssignment(time_dimension.SlackVar(index))
        # Warning: Slack var is not defined for vehicle's end node
        #routing.AddToAssignment(time_dimension.SlackVar(self.routing.End(vehicle_id)))


# print solution
def print_solution(manager, routing, assignment):
    """Prints assignment on console"""
    print(f'Objective: {assignment.ObjectiveValue()}')
    time_dimension = routing.GetDimensionOrDie('Time')
    capacity_dimension = routing.GetDimensionOrDie('Capacity')
    total_distance = 0
    total_load = 0
    total_time = 0
    for vehicle_id in range(manager.GetNumberOfVehicles()):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        distance = 0
        while not routing.IsEnd(index):
            load_var = capacity_dimension.CumulVar(index)
            time_var = time_dimension.CumulVar(index)
            slack_var = time_dimension.SlackVar(index)
            plan_output += ' {0} Load({1}) Time({2},{3}) Slack({4},{5}) ->'.format(
                manager.IndexToNode(index),
                assignment.Value(load_var),
                assignment.Min(time_var),
                assignment.Max(time_var),
                assignment.Min(slack_var), assignment.Max(slack_var))
            previous_index = index
            index = assignment.Value(routing.NextVar(index))
            distance += routing.GetArcCostForVehicle(previous_index, index,
                                                     vehicle_id)
        load_var = capacity_dimension.CumulVar(index)
        time_var = time_dimension.CumulVar(index)
        slack_var = time_dimension.SlackVar(index)
        plan_output += ' {0} Load({1}) Time({2},{3})\n'.format(
            manager.IndexToNode(index),
            assignment.Value(load_var),
            assignment.Min(time_var), assignment.Max(time_var))
        plan_output += 'Distance of the route: {0}m\n'.format(distance)
        plan_output += 'Load of the route: {}\n'.format(
            assignment.Value(load_var))
        plan_output += 'Time of the route: {}\n'.format(
            assignment.Value(time_var))
        print(plan_output)
        total_distance += distance
        total_load += assignment.Value(load_var)
        total_time += assignment.Value(time_var)
    print('Total Distance of all routes: {0}m'.format(total_distance))
    print('Total Load of all routes: {}'.format(total_load))
    print('Total Time of all routes: {0}min'.format(total_time))


def main():
    """Solve the Capacitated VRP with time windows."""
    
    # getting data
    parameters = sys.argv
    data = create_data_model(parameters)

    data = retype_data(data)
    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(data['num_locations'], data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Define weight of each edge.
    distance_evaluator_index = routing.RegisterTransitCallback(
        partial(create_distance_evaluator(data), manager))

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(distance_evaluator_index)

    # Add Capacity constraint.
    demand_evaluator_index = routing.RegisterUnaryTransitCallback(
        partial(create_demand_evaluator(data), manager))
    add_capacity_constraints(routing, data, demand_evaluator_index)

    # Add Time Window constraint.
    time_evaluator_index = routing.RegisterTransitCallback(
        partial(create_time_evaluator(data), manager))

    add_time_window_constraints(routing, manager, data, time_evaluator_index)

    # Setting first solution heuristic (cheapest addition).
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.local_search_metaheuristic = (routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.FromSeconds(2)
    search_parameters.log_search = True

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        print_solution(manager, routing, solution)
    else:
        print('No solution found!')


if __name__ == '__main__':
    main()