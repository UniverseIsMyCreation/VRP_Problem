import numpy as np

def read_vrplib(filename, rounded=True):
    loc = []
    demand = []
    mode = ''
    capacity = None
    edge_weight_type = None
    edge_weight_format = None
    duration_matrix = []
    service_t = []
    timewi = []
    with open(filename, 'r') as f:
        
        for line in f:
            line = line.strip(' \t\n')
            if line == "":
                continue
            elif line.startswith('CAPACITY'):
                capacity = int(line.split(" : ")[1])
            elif line.startswith('EDGE_WEIGHT_TYPE'):
                edge_weight_type = line.split(" : ")[1]
            elif line.startswith('VEHICLES'):
                num_vehicles = int(line.split(" : ")[1])
            elif line.startswith('EDGE_WEIGHT_FORMAT'):
                edge_weight_format = line.split(" : ")[1]
            elif line == 'NODE_COORD_SECTION':
                mode = 'coord'
            elif line == 'DEMAND_SECTION':
                mode = 'demand'
            elif line == 'DEPOT_SECTION':
                mode = 'depot'
            elif line == "EDGE_WEIGHT_SECTION":
                mode = 'edge_weights'
                assert edge_weight_type == "EXPLICIT"
                assert edge_weight_format == "FULL_MATRIX"
            elif line == "TIME_WINDOW_SECTION":
                mode = "time_windows"
            elif line == "SERVICE_TIME_SECTION":
                mode = "service_t"
            elif line == "EOF":
                break
            elif mode == 'coord':
                node, x, y = line.split()  # Split by whitespace or \t, skip duplicate whitespace
                node = int(node)
                x, y = (int(x), int(y)) if rounded else (float(x), float(y))
                
                if node == 1:
                    depot = (x, y)
                else:
                    assert node == len(loc) + 2 # 1 is depot, 2 is 0th location
                    loc.append((x, y))
            elif mode == 'demand':
                node, d = [int(v) for v in line.split()]
                if node == 1:
                    assert d == 0
                demand.append(d)
            elif mode == 'edge_weights':
                duration_matrix.append(list(map(int if rounded else float, line.split())))
            elif mode == 'service_t':
                node, t = line.split()
                node = int(node)
                t = int(t) if rounded else float(t)
                if node == 1:
                    assert t == 0
                assert node == len(service_t) + 1
                service_t.append(t)
            elif mode == 'time_windows':
                node, l, u = line.split()
                node = int(node)
                l, u = (int(l), int(u)) if rounded else (float(l), float(u))
                assert node == len(timewi) + 1
                timewi.append([l, u])
    
    return {
        'is_depot': np.array([1] + [0] * len(loc), dtype=bool),
        'demands': np.array(demand),
        'capacity': capacity,
        'time_windows': np.array(timewi),
        'service_times': np.array(service_t),
        'num_vehicles': num_vehicles,
        'duration_matrix': np.array(duration_matrix) if len(duration_matrix) > 0 else None
    }

def retype_data(data):
    data['num_locations'] = len(data['duration_matrix'])

    demand_lst = list()
    for demand in data['demands']:
        demand_lst.append(int(demand))
    data['demands'] = demand_lst

    data['depot'] = int(np.amax(data['is_depot']*np.arange(data['is_depot'].shape[0])))
    data['num_locations'] = len(data['duration_matrix'])
    del data['is_depot']

    distances = {}
    for from_node in range(data['num_locations']):
        distances[from_node] = {}
        for to_node in range(data['num_locations']):
            distances[from_node][to_node] = int(data['duration_matrix'][from_node][to_node])
    data['duration_matrix'] = distances

    data['capacity'] = int(data['capacity'])

    time_windows = list()
    for time_window in data['time_windows']:
        open = int(time_window[0])
        close = int(time_window[1])
        time_windows.append((open, close))
    data['time_windows'] = time_windows

    service_time_lst = list()
    for service_time in data['service_times']:
        service_time_lst.append(int(service_time))
    data['service_time'] = service_time_lst

    data['num_vehicles'] = int(data['num_vehicles'])

    return data