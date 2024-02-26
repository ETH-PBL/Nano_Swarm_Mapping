#
# This code is adapted from the following work:
# T. Polonelli, C. Feldmann, V. Niculescu, H. Müller, M. Magno and L. Benini,
# "Towards Robust and Efficient On-board Mapping for Autonomous Miniaturized UAVs,"
# 2023 9th International Workshop on Advances in Sensors and Interfaces (IWASI),
# Monopoli (Bari), Italy, 2023, pp. 9-14, doi: 10.1109/IWASI58316.2023.10164476.
# https://ieeexplore.ieee.org/abstract/document/10164476
#


from ctypes import *
import numpy as np

get_map = cdll.LoadLibrary('libmapping.so').get_occ_map
get_map2 = cdll.LoadLibrary('libmapping.so').get_occ_map2

# This function works with the following data:
# pos: numpy array of shape (n, 3), where the columns are drone_pos_x [m], drone_pos_y [m], drone_yaw [rad]
# tof_mm: numpy array of shape (n, 4, 8), distances are in mm, and invalid pixels are -1
# max_range_mm what maximum range to consider among the tof measurements. Higher values will be discarded
# min_x, min_y: coordinates in m of the left bottom point where the map starts
# l_x, l_y length considered in m starting from (min_x, min_y)
# res: resolution in m
def pcloud_to_map(pos, tof_mm, max_range_mm, min_x, min_y, l_x, l_y, res):
	n = int(pos.shape[0])

	pos_list = pos.reshape(-1).tolist()
	tof_list = tof_mm.reshape(-1).tolist()

	pos_size = n * 3
	tof_size = n * 32

	pos_in = (c_float * pos_size)(*pos_list)
	tof_in = (c_short * tof_size)(*tof_list)
	out_dim = (c_short * 2)()
	out_map = (c_byte * 1000000)()

	get_map(pos_in, tof_in, c_short(n), c_short(max_range_mm), c_float(min_x), c_float(min_y), c_float(l_x), c_float(l_y), c_float(res), out_dim, out_map)
	[n, m] = out_dim[:]
	map = np.zeros((n, m))
	map_array = out_map[:]
	for i in range(n):
		for j in range(m):
			map[i, j] = map_array[i*m + j]
	return map

# This function works similar to the one above, but considers already the projected tof measurements
# pos: numpy array of shape (n, 3), where the columns are drone_pos_x [m], drone_pos_y [m], drone_yaw [rad]
# scan_xy: projected tof measurements in the global frame. Expressed in m
# max_range_mm what maximum range to consider among the tof measurements. Higher values will be discarded
# min_x, min_y: coordinates in m of the left bottom point where the map starts
# l_x, l_y length considered in m starting from (min_x, min_y)
# res: resolution in m
def pcloud_to_map2(pos, scan_xy, max_range_mm, min_x, min_y, l_x, l_y, res):
	n = int(pos.shape[0])

	pos_list = pos.reshape(-1).tolist()
	scan_list = scan_xy.reshape(-1).tolist()

	pos_size = n * 3
	scan_size = n * 2

	pos_in = (c_float * pos_size)(*pos_list)
	scan_in = (c_float * scan_size)(*scan_list)
	out_dim = (c_short * 2)()
	out_map = (c_byte * 1000000)()

	get_map2(pos_in, scan_in, c_short(n), c_short(max_range_mm), c_float(min_x), c_float(min_y), c_float(l_x), c_float(l_y), c_float(res), out_dim, out_map)
	[n, m] = out_dim[:]
	map = np.zeros((n, m))
	map_array = out_map[:]
	for i in range(n):
		for j in range(m):
			map[i, j] = map_array[i*m + j]
	return map

