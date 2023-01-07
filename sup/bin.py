
# test case for sanity
# beacons = [
#     Beacon(x0=[1, 1, 1]),
#     Beacon(x0=[-1, 1, 1]),
#     Beacon(x0=[-1, -1, -1]),
#     Beacon(x0=[1, -1, 1])
# ]

# x = agent.get_state()

# print(np.linalg.norm(Z - agent.get_state()[:3]))
# print(Z, agent.get_state()[:3])

# if np.linalg.norm(f.x[:3] - agent.get_state()[:3]) > 1:
#     print("Large diff!!!")
#     exit()
# x, P = predict(x, P, F, u)
# (Z, success) = agent.triangulate_pos(beacons)
# if success:
#     x, P = update(x, P, Z, H, R)
#     # agent.update(x)
# path.append(x[:2])
# print(x, np.max(P), Z.T, success)
# agent.update(x)

# x0 is very important for convergence !!!
# def triangulate_pos(self, beacons: list, debug: bool = False, x0=None):
#     # def error(x, c, r):
#     #     return sum([(np.linalg.norm(x - c[i]) - r[i]) ** 2 for i in range(len(c))])
#     def error(x, c, r):
#         return np.array([np.linalg.norm(x - c[i]) - r[i] for i in range(len(c))])

#     def dist(a, b):
#         return np.linalg.norm(a - b, 2)

#     distances_to_station = np.array(  # increasing noise to +-0.5m brakes filter
#         [dist(self.__x[:3], b.get_pos() + (np.random.random()-.5)*.1) for b in beacons])
#     stations_coordinates = [b.get_pos().T for b in beacons]
#     l = len(stations_coordinates)  # number of stations
#     S = np.sum(distances_to_station)
#     # compute weight vector for initial guess
#     W = [((l - 1) * S) / (S - w) for w in distances_to_station]
#     # get initial guess of point location
#     if x0 is None:
#         x0 = sum([W[i] * stations_coordinates[i] for i in range(l)])
#     # optimize distance from signal origin to border of spheres

#     sol = least_squares(error, x0.reshape(3,), args=(stations_coordinates,
#                                          distances_to_station), method='lm')

#     # sol = minimize(error, x0, args=(stations_coordinates,
#     #                distances_to_station), method='BFGS')
#     # if not sol.success:
#     #     raise Exception("Optimization failed!")
#     if debug:
#         print(W)
#         print(x0)
#         print(distances_to_station)
#         print(stations_coordinates)
#         print(sol)
#     # if np.linalg.norm(sol.x - np.ones((3,))) > 1e-3:
#     #     print(np.linalg.norm(sol.x - np.ones((3,))))
#     # exit()
#     return sol.x.reshape(3, 1), sol.success


# initial ranging, find enough beacons to start localization
success = False
while not success:
    print("Looking for enough active beacons...")
    (success, z) = get_measurements(1)
    print(f"Found {z.shape[0]} active beacons...")
print("Starting system...")


# RUN echo keyboard-configuration keyboard-configuration/layout select 'English (US)' | sudo debconf-set-selections
# RUN echo keyboard-configuration keyboard-configuration/layoutcode select 'us' | sudo debconf-set-selections
# source ~/asta_ws/vrpn_ws/devel/setup.bash add this to /ros_entrypoint.sh : )
# and maybe add ros_entrypoint.sh to ~/.bashrc : )
# ENTRYPOINT ["/bin/bash", "source /ros_entrypoint.sh"]

# def get_measurements(timeout=.1):
#     start = time.time()
#     found_enough_active_beacons = False
#     while not found_enough_active_beacons and (time.time() - start) < timeout:
#         # update from measurements
#         devices = read_measurements(timeout/2)
#         for id, meas in devices.items():
#             BEACON_MAP[id].update_range(meas["Range"])
#         # count active ones
#         active_count = sum([beac.is_active()
#                            for _, beac in BEACON_MAP.items()])
#         if active_count >= 4:  # 4 is enough
#             found_enough_active_beacons = True
#     return found_enough_active_beacons, np.array([v.get_range() for k, v in BEACON_MAP.items()]).reshape((active_count, 1))
