##Description: This message defines for a specific range of distances [minDistanceMeters, maxDistanceMeters) (between humans and robots) how much time was spent there.


#minimum value of range (lowerBoundInclusive)
uint64 min_distance_meters

#maximum value of range (higherBoundExclusive)
uint64 max_distance_meters

#time that distance between human and robot was inside the range [minDistanceMeters, maxDistanceMeters)
uint64 time_spent